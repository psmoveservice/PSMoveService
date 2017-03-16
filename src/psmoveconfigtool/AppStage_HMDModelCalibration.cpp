//-- inludes -----
#include "AppStage_HMDModelCalibration.h"
#include "AppStage_HMDSettings.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "AssetManager.h"
#include "Camera.h"
#include "GeometryUtility.h"
#include "Logger.h"
#include "MathEigen.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include "SharedTrackerState.h"
#include "MathGLM.h"
#include "MathEigen.h"

#include "SDL_keycode.h"
#include "SDL_opengl.h"

#include "ICP.h"

#include "PSMoveClient_CAPI.h"

#include <imgui.h>
#include <sstream>
#include <vector>
#include <set>

//-- typedefs ----
namespace SICP
{
	typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Vertices;
};

//-- statics ----
const char *AppStage_HMDModelCalibration::APP_STAGE_NAME = "HMDModelCalibration";

//-- constants -----
static const int k_max_projection_points = 16;
static const int k_morpheus_led_count = 9;
static const int k_led_position_sample_count = 100;

static float k_cosine_aligned_camera_angle = cosf(60.f *k_degrees_to_radians);

static const float k_default_correspondance_tolerance = 0.2f;

static const float k_icp_point_snap_distance = 3.0; // cm

static const glm::vec3 k_psmove_frustum_color = glm::vec3(0.1f, 0.7f, 0.3f);
static const glm::vec3 k_psmove_frustum_color_no_track = glm::vec3(1.0f, 0.f, 0.f);

//-- private methods -----
static glm::mat4 computeGLMCameraTransformMatrix(const PSMTracker *tracker_view);
static cv::Matx34f computeOpenCVCameraExtrinsicMatrix(const PSMTracker *tracker_view);
static cv::Matx33f computeOpenCVCameraIntrinsicMatrix(const PSMTracker *tracker_view);
static cv::Matx34f computeOpenCVCameraPinholeMatrix(const PSMTracker *tracker_view);
static bool triangulateHMDProjections(PSMHeadMountedDisplay *hmd_view, TrackerPairState *tracker_pair_state, std::vector<Eigen::Vector3f> &out_triangulated_points);
static PSMVector2f projectWorldPositionOnTracker(const PSMVector3f &worldSpacePosition, const PSMTracker *trackerView);
static void drawHMD(PSMHeadMountedDisplay *hmdView, const glm::mat4 &transform);

//-- private structures -----
struct TrackerState
{
	PSMTracker *trackerView;
	class TextureAsset *textureAsset;
};

struct TrackerPairState
{
	union
	{
		TrackerState list[2];
		struct
		{
			TrackerState a;
			TrackerState b;
		} pair;
	} trackers;

	int pendingTrackerStartCount;
	int renderTrackerIndex;

	Eigen::Matrix3f F_ab; // Fundamental matrix from tracker A to tracker B
	float tolerance;

	void init()
	{
		memset(this, 0, sizeof(TrackerPairState));
		tolerance = k_default_correspondance_tolerance;
	}

	bool do_points_correspond(
		const cv::Mat &pointA,
		const cv::Mat &pointB,
		float tolerance)
	{
		//See if image point A * Fundamental Matrix * image point B <= tolerance
		const Eigen::Vector3f a(pointA.at<float>(0,0), pointA.at<float>(1,0), 1.f);
		const Eigen::Vector3f b(pointB.at<float>(0,0), pointB.at<float>(1,0), 1.f);
		const float epipolar_distance = fabsf(a.transpose() * F_ab * b);

		return epipolar_distance <= tolerance;
	}
};

struct LEDModelSamples
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Eigen::Vector3f position_samples[k_led_position_sample_count];
	Eigen::Vector3f average_position;
	int position_sample_count;

	void init()
	{
		average_position = Eigen::Vector3f::Zero();
		position_sample_count = 0;
	}

	bool add_position(const Eigen::Vector3f &point)
	{
		bool bAdded = false;

		if (position_sample_count < k_led_position_sample_count)
		{
			position_samples[position_sample_count] = point;
			++position_sample_count;

			// Recompute the average position
			if (position_sample_count > 1)
			{
				const float N = static_cast<float>(position_sample_count);

				average_position = Eigen::Vector3f::Zero();
				for (int position_index = 0; position_index < position_sample_count; ++position_index)
				{
					average_position += position_samples[position_index];
				}
				average_position /= N;
			}
			else
			{
				average_position = position_samples[0];
			}

			bAdded = true;
		}

		return bAdded;
	}
};

class HMDModelState
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	HMDModelState(int trackerLEDCount)
		: m_expectedLEDCount(trackerLEDCount)
		, m_ledSampleSet(new LEDModelSamples[trackerLEDCount])
		, m_seenLEDCount(0)
		, m_totalLEDSampleCount(0)
	{
		m_icpTransform = Eigen::Affine3d::Identity();

		for (int led_index = 0; led_index < trackerLEDCount; ++led_index)
		{
			m_ledSampleSet[led_index].init();
		}
	}

	~HMDModelState()
	{
		delete[] m_ledSampleSet;
	}

	bool getIsComplete() const
	{
		const int expectedSampleCount = m_expectedLEDCount * k_led_position_sample_count;

		return m_totalLEDSampleCount >= expectedSampleCount;
	}

	float getProgressFraction() const 
	{
		const float expectedSampleCount = static_cast<float>(m_seenLEDCount * k_led_position_sample_count);
		const float fraction = static_cast<float>(m_totalLEDSampleCount) / expectedSampleCount;

		return fraction;
	}

	void recordSamples(PSMHeadMountedDisplay *hmd_view, TrackerPairState *tracker_pair_state)
	{
		if (triangulateHMDProjections(hmd_view, tracker_pair_state, m_lastTriangulatedPoints))
		{
			const int source_point_count = static_cast<int>(m_lastTriangulatedPoints.size());

			if (source_point_count >= 3)
			{
				if (m_seenLEDCount > 0)
				{
					// Copy the triangulated vertices into a 3xN matric the SICP algorithms can use
					SICP::Vertices icpSourceVertices;
					icpSourceVertices.resize(Eigen::NoChange, source_point_count);
					for (int source_index = 0; source_index < source_point_count; ++source_index)
					{
						const Eigen::Vector3f &point = m_lastTriangulatedPoints[source_index];

						icpSourceVertices(0, source_index) = point.x();
						icpSourceVertices(1, source_index) = point.y();
						icpSourceVertices(2, source_index) = point.z();
					}

					// Build kd-tree of the current set of target vertices
					nanoflann::KDTreeAdaptor<SICP::Vertices, 3, nanoflann::metric_L2_Simple> kdtree(m_icpTargetVertices);

					// Attempt to align the new triangulated points with the previously found LED locations
					// using the ICP algorithm
					SICP::Parameters params;
					params.p = .5;
					params.max_icp = 15;
					params.print_icpn = true;
					SICP::point_to_point(icpSourceVertices, m_icpTargetVertices, params);

					// Update the LED models based on the alignment
					bool bUpdateTargetVertices = false;
					for (int source_index = 0; source_index < icpSourceVertices.cols(); ++source_index)
					{
						const Eigen::Vector3d source_vertex = icpSourceVertices.col(source_index).cast<double>();
						const int closest_led_index = kdtree.closest(source_vertex.data());
						const Eigen::Vector3d closest_led_position = m_ledSampleSet[closest_led_index].average_position.cast<double>();
						const double cloest_distance_sqrd = (closest_led_position - source_vertex).squaredNorm();

						// Add the points to the their respective bucket...
						if (cloest_distance_sqrd <= k_icp_point_snap_distance)
						{
							bUpdateTargetVertices |= add_point_to_led_model(closest_led_index, source_vertex.cast<float>());
						}
						// ... or make a new bucket if no point at that location
						else
						{
							bUpdateTargetVertices |= add_led_model(source_vertex.cast<float>());
						}
					}

					if (bUpdateTargetVertices)
					{
						rebuildTargetVertices();
					}
				}
				else
				{
					for (auto it = m_lastTriangulatedPoints.begin(); it != m_lastTriangulatedPoints.end(); ++it)
					{
						add_led_model(*it);
					}

					rebuildTargetVertices();
				}
			}

			//TODO:
			/*
			// Create a mesh from the average of the best N buckets
			// where N is the expected tracking light count from HMD properties
			*/
		}
	}

	void render(const PSMTracker *trackerView) const
	{
		PSMVector2f projections[k_max_projection_points];
		int point_count = static_cast<int>(m_lastTriangulatedPoints.size());

		for (int point_index = 0; point_index < point_count; ++point_index)
		{
			PSMVector3f worldPosition= eigen_vector3f_to_psm_vector3f(m_lastTriangulatedPoints[point_index]);

			projections[point_index] = projectWorldPositionOnTracker(worldPosition, trackerView);
		}

		PSMVector2f tracker_size= trackerView->tracker_info.tracker_screen_dimensions;
		drawPointCloudProjection(projections, point_count, 6.f, glm::vec3(0.f, 1.f, 0.f), tracker_size.x, tracker_size.y);
	}

protected:
	bool add_point_to_led_model(const int led_index, const Eigen::Vector3f &point)
	{
		bool bAddedPoint = false;
		LEDModelSamples &ledModel = m_ledSampleSet[led_index];

		if (ledModel.add_position(point))
		{
			++m_totalLEDSampleCount;
			bAddedPoint = true;
		}

		return bAddedPoint;
	}

	bool add_led_model(const Eigen::Vector3f &initial_point)
	{
		bool bAddedLed = false;

		if (m_seenLEDCount < m_expectedLEDCount)
		{
			if (add_point_to_led_model(m_seenLEDCount, initial_point))
			{
				++m_seenLEDCount;
				bAddedLed = true;
			}
		}

		return bAddedLed;
	}

	void rebuildTargetVertices()
	{
		m_icpTargetVertices.resize(Eigen::NoChange, m_seenLEDCount);

		for (int led_index = 0; led_index < m_seenLEDCount; ++led_index)
		{
			const Eigen::Vector3f &ledSample= m_ledSampleSet[led_index].average_position;

			m_icpTargetVertices(0, led_index) = ledSample.x();
			m_icpTargetVertices(1, led_index) = ledSample.y();
			m_icpTargetVertices(2, led_index) = ledSample.z();
		}
	}

private:
	std::vector<Eigen::Vector3f> m_lastTriangulatedPoints;
	int m_expectedLEDCount;
	int m_seenLEDCount;
	int m_totalLEDSampleCount;

	LEDModelSamples *m_ledSampleSet;

	SICP::Vertices m_icpTargetVertices;
	Eigen::Affine3d m_icpTransform;
};

//-- public methods -----
AppStage_HMDModelCalibration::AppStage_HMDModelCalibration(App *app)
	: AppStage(app)
	, m_menuState(AppStage_HMDModelCalibration::inactive)
	, m_trackerPairState(new TrackerPairState)
	, m_hmdModelState(nullptr)
	, m_hmdView(nullptr)
	, m_bBypassCalibration(false)
{
	m_trackerPairState->init();
}

AppStage_HMDModelCalibration::~AppStage_HMDModelCalibration()
{
	delete m_trackerPairState;

	if (m_hmdModelState != nullptr)
	{
		delete m_hmdModelState;
	}
}

void AppStage_HMDModelCalibration::enterStageAndCalibrate(App *app, int requested_hmd_id)
{
	app->getAppStage<AppStage_HMDModelCalibration>()->m_bBypassCalibration = false;
	app->getAppStage<AppStage_HMDModelCalibration>()->m_overrideHmdId = requested_hmd_id;
	app->setAppStage(AppStage_HMDModelCalibration::APP_STAGE_NAME);
}

void AppStage_HMDModelCalibration::enterStageAndSkipCalibration(App *app, int requested_hmd_id)
{
	app->getAppStage<AppStage_HMDModelCalibration>()->m_bBypassCalibration = true;
	app->getAppStage<AppStage_HMDModelCalibration>()->m_overrideHmdId = requested_hmd_id;
	app->setAppStage(AppStage_HMDModelCalibration::APP_STAGE_NAME);
}

void AppStage_HMDModelCalibration::enter()
{
	// Kick off this async request chain with an hmd list request
	// -> hmd start request
	// -> tracker list request
	// -> tracker start request
	request_hmd_list();

	m_app->setCameraType(_cameraFixed);
}

void AppStage_HMDModelCalibration::exit()
{
	release_devices();

	setState(eMenuState::inactive);
}

void AppStage_HMDModelCalibration::update()
{
	switch (m_menuState)
	{
	case eMenuState::inactive:
		break;
	case eMenuState::pendingHmdListRequest:
	case eMenuState::pendingHmdStartRequest:
	case eMenuState::pendingTrackerListRequest:
	case eMenuState::pendingTrackerStartRequest:
		break;
	case eMenuState::failedHmdListRequest:
	case eMenuState::failedHmdStartRequest:
	case eMenuState::failedTrackerListRequest:
	case eMenuState::failedTrackerStartRequest:
		break;
	case eMenuState::verifyTrackers:
		update_tracker_video();
		break;
	case eMenuState::calibrate:
	{
		update_tracker_video();

		if (!m_hmdModelState->getIsComplete())
		{
			m_hmdModelState->recordSamples(m_hmdView, m_trackerPairState);
			if (m_hmdModelState->getIsComplete())
			{
				setState(eMenuState::test);
			}
		}
		else
		{
			request_set_hmd_led_model_calibration();
			setState(eMenuState::test);
		}
	}
	break;
	case eMenuState::test:
		//TODO
		break;
	default:
		assert(0 && "unreachable");
	}
}

void AppStage_HMDModelCalibration::render()
{
	switch (m_menuState)
	{
	case eMenuState::inactive:
		break;
	case eMenuState::pendingHmdListRequest:
	case eMenuState::pendingHmdStartRequest:
	case eMenuState::pendingTrackerListRequest:
	case eMenuState::pendingTrackerStartRequest:
		break;
	case eMenuState::failedHmdListRequest:
	case eMenuState::failedHmdStartRequest:
	case eMenuState::failedTrackerListRequest:
	case eMenuState::failedTrackerStartRequest:
		break;
	case eMenuState::verifyTrackers:
		{
			render_tracker_video();
		} break;
	case eMenuState::calibrate:
		{
			const PSMTracker *TrackerView = get_render_tracker_view();

			// Draw the video from the PoV of the current tracker
			render_tracker_video();

			// Draw the current state of the HMD model being generated
			m_hmdModelState->render(TrackerView);
		}
		break;
	case eMenuState::test:
		{
			// Draw the chaperone origin axes
			drawTransformedAxes(glm::mat4(1.0f), 100.f);

			// Draw the frustum for each tracking camera.
			// The frustums are defined in PSMove tracking space.
			// We need to transform them into chaperone space to display them along side the HMD.
			for (int tracker_index = 0; tracker_index < get_tracker_count(); ++tracker_index)
			{
				const PSMTracker *trackerView = m_trackerPairState->trackers.list[tracker_index].trackerView;
				const PSMPosef psmove_space_pose = trackerView->tracker_info.tracker_pose;
				const glm::mat4 chaperoneSpaceTransform = psm_posef_to_glm_mat4(psmove_space_pose);

				{
					PSMFrustum frustum;
					PSM_GetTrackerFrustum(trackerView->tracker_info.tracker_id, &frustum);

					// use color depending on tracking status
					glm::vec3 color;
					bool bIsTracking;
					if (PSM_GetIsHmdTracking(m_hmdView->HmdID, &bIsTracking) == PSMResult_Success)
					{
						PSMVector2f screenSample;

						if (bIsTracking && PSM_GetHmdPixelLocationOnTracker(m_hmdView->HmdID, trackerView->tracker_info.tracker_id, &screenSample) == PSMResult_Success)
						{
							color = k_psmove_frustum_color;
						}
						else
						{
							color = k_psmove_frustum_color_no_track;
						}
					}
					else
					{
						color = k_psmove_frustum_color_no_track;
					}
					drawTransformedFrustum(glm::mat4(1.f), &frustum, color);
				}

				drawTransformedAxes(chaperoneSpaceTransform, 20.f);
			}


			// Draw the Morpheus model
			{
				PSMPosef hmd_pose;
				
				if (PSM_GetHmdPose(m_hmdView->HmdID, &hmd_pose) == PSMResult_Success)
				{
					glm::mat4 hmd_transform = psm_posef_to_glm_mat4(hmd_pose);

					drawHMD(m_hmdView, hmd_transform);
					drawTransformedAxes(hmd_transform, 10.f);
				}
			}

		} break;
	default:
		assert(0 && "unreachable");
	}
}

void AppStage_HMDModelCalibration::renderUI()
{
	const float k_panel_width = 300.f;
	const char *k_window_title = "Compute HMD Model";
	const ImGuiWindowFlags window_flags =
		ImGuiWindowFlags_ShowBorders |
		ImGuiWindowFlags_NoResize |
		ImGuiWindowFlags_NoMove |
		ImGuiWindowFlags_NoScrollbar |
		ImGuiWindowFlags_NoCollapse;

	switch (m_menuState)
	{
	case eMenuState::inactive:
		break;

	case eMenuState::pendingHmdListRequest:
	case eMenuState::pendingHmdStartRequest:
	case eMenuState::pendingTrackerListRequest:
	case eMenuState::pendingTrackerStartRequest:
	{
		ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
		ImGui::SetNextWindowSize(ImVec2(k_panel_width, 80));
		ImGui::Begin(k_window_title, nullptr, window_flags);

		ImGui::Text("Pending device initialization...");

		if (ImGui::Button("Return to HMD Settings"))
		{
			request_exit_to_app_stage(AppStage_HMDModelCalibration::APP_STAGE_NAME);
		}

		ImGui::End();
	} break;

	case eMenuState::failedHmdListRequest:
	case eMenuState::failedHmdStartRequest:
	case eMenuState::failedTrackerListRequest:
	case eMenuState::failedTrackerStartRequest:
	{
		ImGui::SetNextWindowPosCenter();
		ImGui::SetNextWindowSize(ImVec2(k_panel_width, 180));
		ImGui::Begin(k_window_title, nullptr, window_flags);

		switch (m_menuState)
		{
		case eMenuState::failedHmdListRequest:
			ImGui::Text("Failed hmd list retrieval!");
			break;
		case eMenuState::failedHmdStartRequest:
			ImGui::Text("Failed hmd stream start!");
			break;
		case eMenuState::failedTrackerListRequest:
			{
				const char * szFailure = m_failureDetails.c_str();
				ImGui::Text("Failed tracker list retrieval:");
				ImGui::Text(szFailure);
			}
			break;
		case eMenuState::failedTrackerStartRequest:
			ImGui::Text("Failed tracker stream start!");
			break;
		}

		if (ImGui::Button("Ok"))
		{
			request_exit_to_app_stage(AppStage_HMDSettings::APP_STAGE_NAME);
		}

		if (ImGui::Button("Return to Main Menu"))
		{
			request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
		}

		ImGui::End();
	} break;

	case eMenuState::verifyTrackers:
	{
		ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - 500.f / 2.f, 20.f));
		ImGui::SetNextWindowSize(ImVec2(500.f, (get_tracker_count() > 0) ? 150.f : 100.f));
		ImGui::Begin(k_window_title, nullptr, window_flags);

		ImGui::Text("Verify that your tracking cameras can see your HMD");
		ImGui::Separator();

		ImGui::Text("Tracker #%d", m_trackerPairState->renderTrackerIndex + 1);

		if (ImGui::Button("Previous Tracker"))
		{
			go_previous_tracker();
		}
		ImGui::SameLine();
		if (ImGui::Button("Next Tracker"))
		{
			go_next_tracker();
		}

		if (ImGui::Button("Looks Good!"))
		{
			setState(eMenuState::calibrate);
		}

		if (ImGui::Button("Hmm... Something is wrong."))
		{
			request_exit_to_app_stage(AppStage_HMDSettings::APP_STAGE_NAME);
		}

		ImGui::End();
	} break;

	case eMenuState::calibrate:
	{
		ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
		ImGui::SetNextWindowSize(ImVec2(k_panel_width, 180));
		ImGui::Begin(k_window_title, nullptr, window_flags);

		ImGui::Text("Tracker #%d", m_trackerPairState->renderTrackerIndex + 1);

		if (ImGui::Button("Previous Tracker"))
		{
			go_previous_tracker();
		}
		ImGui::SameLine();
		if (ImGui::Button("Next Tracker"))
		{
			go_next_tracker();
		}

		ImGui::Separator();

		// TODO: Show calibration progress
		ImGui::ProgressBar(m_hmdModelState->getProgressFraction(), ImVec2(250, 20));

		// display tracking quality
		for (int tracker_index = 0; tracker_index < get_tracker_count(); ++tracker_index)
		{
			const PSMTracker *trackerView = m_trackerPairState->trackers.list[tracker_index].trackerView;
			const int tracker_id= trackerView->tracker_info.tracker_id;

			bool bIsTracking;
			if (PSM_GetIsHmdTracking(m_hmdView->HmdID, &bIsTracking) == PSMResult_Success)
			{
				PSMVector2f screenSample;

				if (bIsTracking && PSM_GetHmdPixelLocationOnTracker(m_hmdView->HmdID, tracker_id, &screenSample) == PSMResult_Success)
				{
					ImGui::Text("Tracking %d: OK", tracker_id + 1);
				}
				else
				{
					ImGui::Text("Tracking %d: FAIL", tracker_id + 1);
				}
			}
			else
			{
				ImGui::Text("Tracking %d: FAIL", tracker_id + 1);
			}
		}

		ImGui::SliderFloat("Tolerance", &m_trackerPairState->tolerance, 0.f, 1.f);

		ImGui::Separator();

		if (ImGui::Button("Restart Calibration"))
		{
			setState(eMenuState::verifyTrackers);
		}
		ImGui::SameLine();
		if (ImGui::Button("Exit"))
		{
			m_app->setAppStage(AppStage_HMDSettings::APP_STAGE_NAME);
		}

		ImGui::End();
	} break;

	case eMenuState::test:
	{
		ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
		ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
		ImGui::Begin(k_window_title, nullptr, window_flags);

		if (!m_bBypassCalibration)
		{
			ImGui::Text("Calibration Complete");

			if (ImGui::Button("Redo Calibration"))
			{
				setState(eMenuState::verifyTrackers);
			}
		}

		if (ImGui::Button("Exit"))
		{
			m_app->setAppStage(AppStage_HMDModelCalibration::APP_STAGE_NAME);
		}

		// display tracking quality
		for (int tracker_index = 0; tracker_index < get_tracker_count(); ++tracker_index)
		{
			const PSMTracker *trackerView = m_trackerPairState->trackers.list[tracker_index].trackerView;
			const int tracker_id= trackerView->tracker_info.tracker_id;

			bool bIsTracking;
			if (PSM_GetIsHmdTracking(m_hmdView->HmdID, &bIsTracking) == PSMResult_Success)
			{
				PSMVector2f screenSample;

				if (bIsTracking && PSM_GetHmdPixelLocationOnTracker(m_hmdView->HmdID, tracker_id, &screenSample) == PSMResult_Success)
				{
					ImGui::Text("Tracking %d: OK", tracker_id + 1);
				}
				else
				{
					ImGui::Text("Tracking %d: FAIL", tracker_id + 1);
				}
			}
			else
			{
				ImGui::Text("Tracking %d: FAIL", tracker_id + 1);
			}
		}
		ImGui::Text("");

		ImGui::End();
	}
	break;

	default:
		assert(0 && "unreachable");
	}
}

void AppStage_HMDModelCalibration::setState(AppStage_HMDModelCalibration::eMenuState newState)
{
	if (newState != m_menuState)
	{
		onExitState(m_menuState);
		onEnterState(newState);

		m_menuState = newState;
	}
}

void AppStage_HMDModelCalibration::onExitState(AppStage_HMDModelCalibration::eMenuState newState)
{
	switch (m_menuState)
	{
	case eMenuState::inactive:
		break;
	case eMenuState::pendingHmdListRequest:
	case eMenuState::pendingHmdStartRequest:
	case eMenuState::pendingTrackerListRequest:
	case eMenuState::pendingTrackerStartRequest:
		break;
	case eMenuState::failedHmdListRequest:
	case eMenuState::failedHmdStartRequest:
	case eMenuState::failedTrackerListRequest:
	case eMenuState::failedTrackerStartRequest:
		break;
	case eMenuState::verifyTrackers:
		break;
	case eMenuState::calibrate:
		break;
	case eMenuState::test:
		m_app->setCameraType(_cameraFixed);
		break;
	default:
		assert(0 && "unreachable");
	}
}

void AppStage_HMDModelCalibration::onEnterState(AppStage_HMDModelCalibration::eMenuState newState)
{
	switch (newState)
	{
	case eMenuState::inactive:
		break;
	case eMenuState::pendingHmdListRequest:
	case eMenuState::pendingHmdStartRequest:
	case eMenuState::pendingTrackerListRequest:
		m_trackerPairState->init();
		m_failureDetails = "";
		break;
	case eMenuState::pendingTrackerStartRequest:
		break;
	case eMenuState::failedHmdListRequest:
	case eMenuState::failedHmdStartRequest:
	case eMenuState::failedTrackerListRequest:
	case eMenuState::failedTrackerStartRequest:
		break;
	case eMenuState::verifyTrackers:
		m_trackerPairState->renderTrackerIndex = 0;
		break;
	case eMenuState::calibrate:
		// TODO
		break;
	case eMenuState::test:
		m_app->setCameraType(_cameraOrbit);
		break;
	default:
		assert(0 && "unreachable");
	}
}

void AppStage_HMDModelCalibration::update_tracker_video()
{
	// Render the latest from the currently active tracker
	TrackerState &trackerState= m_trackerPairState->trackers.list[m_trackerPairState->renderTrackerIndex];
	if (trackerState.trackerView != nullptr &&
		PSM_PollTrackerVideoStream(trackerState.trackerView->tracker_info.tracker_id))
	{
		const unsigned char *buffer= nullptr;

		if (PSM_GetTrackerVideoFrameBuffer(trackerState.trackerView->tracker_info.tracker_id, &buffer) == PSMResult_Success)
		{
			trackerState.textureAsset->copyBufferIntoTexture(buffer);
		}
	}
}

void AppStage_HMDModelCalibration::render_tracker_video()
{
	TrackerState &trackerState = m_trackerPairState->trackers.list[m_trackerPairState->renderTrackerIndex];
	if (trackerState.trackerView != nullptr &&
		trackerState.textureAsset != nullptr)
	{
		drawFullscreenTexture(trackerState.textureAsset->texture_id);
	}
}

void AppStage_HMDModelCalibration::go_next_tracker()
{
	m_trackerPairState->renderTrackerIndex = (m_trackerPairState->renderTrackerIndex + 1) % get_tracker_count();
}

void AppStage_HMDModelCalibration::go_previous_tracker()
{	
	m_trackerPairState->renderTrackerIndex = (m_trackerPairState->renderTrackerIndex + get_tracker_count() - 1) % get_tracker_count();
}

int AppStage_HMDModelCalibration::get_tracker_count() const
{
	return 2;
}

int AppStage_HMDModelCalibration::get_render_tracker_index() const
{
	return m_trackerPairState->renderTrackerIndex;
}

PSMTracker *AppStage_HMDModelCalibration::get_render_tracker_view() const
{
	return m_trackerPairState->trackers.list[m_trackerPairState->renderTrackerIndex].trackerView;
}

void AppStage_HMDModelCalibration::release_devices()
{
	//###HipsterSloth $REVIEW Do we care about canceling in-flight requests?

	if (m_hmdModelState != nullptr)
	{
		delete m_hmdModelState;
		m_hmdModelState = nullptr;
	}

	if (m_hmdView != nullptr)
	{
		PSM_StopHmdDataStreamAsync(m_hmdView->HmdID, nullptr);
		PSM_FreeHmdListener(m_hmdView->HmdID);
		m_hmdView = nullptr;
	}

	for (int tracker_index = 0; tracker_index < get_tracker_count(); ++tracker_index)
	{
		TrackerState &trackerState = m_trackerPairState->trackers.list[tracker_index];

		if (trackerState.textureAsset != nullptr)
		{
			delete trackerState.textureAsset;
		}

		if (trackerState.trackerView != nullptr)
		{
			PSM_CloseTrackerVideoStream(trackerState.trackerView->tracker_info.tracker_id);
			PSM_StopTrackerDataStreamAsync(trackerState.trackerView->tracker_info.tracker_id, nullptr);
			PSM_FreeTrackerListener(trackerState.trackerView->tracker_info.tracker_id);
		}
	}

	m_trackerPairState->init();
}

void AppStage_HMDModelCalibration::request_exit_to_app_stage(const char *app_stage_name)
{
	release_devices();

	m_app->setAppStage(app_stage_name);
}

void AppStage_HMDModelCalibration::request_hmd_list()
{
	if (m_menuState != AppStage_HMDModelCalibration::pendingHmdListRequest)
	{
		m_menuState = AppStage_HMDModelCalibration::pendingHmdListRequest;

		// Request a list of controllers back from the server
		PSMRequestID requestId;
		PSM_GetHmdListAsync(&requestId);
		PSM_RegisterCallback(requestId, AppStage_HMDModelCalibration::handle_hmd_list_response, this);
	}
}

void AppStage_HMDModelCalibration::handle_hmd_list_response(
	const PSMResponseMessage *response_message,
	void *userdata)
{
	const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_message->opaque_response_handle);
	const PSMoveProtocol::Request *request = GET_PSMOVEPROTOCOL_REQUEST(response_message->opaque_request_handle);

	AppStage_HMDModelCalibration *thisPtr = static_cast<AppStage_HMDModelCalibration *>(userdata);

	const PSMResult ResultCode = response_message->result_code;


	switch (ResultCode)
	{
	case PSMResult_Success:
	{
		assert(response_message->payload_type == PSMResponseMessage::_responsePayloadType_HmdList);
		const PSMHmdList *hmd_list = &response_message->payload.hmd_list;

		int trackedHmdId = thisPtr->m_overrideHmdId;
		int trackerLEDCount = 0;

		if (trackedHmdId == -1)
		{
			for (int list_index = 0; list_index < hmd_list->count; ++list_index)
			{
				if (hmd_list->hmd_type[list_index] == PSMHmd_Morpheus)
				{
					trackedHmdId = hmd_list->hmd_id[list_index];
					break;
				}
			}
		}

		//###HipsterSloth $TODO - This should come as part of the response payload
		trackerLEDCount = k_morpheus_led_count;

		if (trackedHmdId != -1)
		{
			// Create a model for the HMD that corresponds to the number of tracking lights
			assert(thisPtr->m_hmdModelState == nullptr);
			thisPtr->m_hmdModelState = new HMDModelState(trackerLEDCount);

			// Start streaming data for the HMD
			thisPtr->request_start_hmd_stream(trackedHmdId);
		}
		else
		{
			thisPtr->setState(AppStage_HMDModelCalibration::failedHmdListRequest);
		}
	} break;

	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
		{
			thisPtr->setState(AppStage_HMDModelCalibration::failedHmdListRequest);
		} break;
	}
}

void AppStage_HMDModelCalibration::request_start_hmd_stream(int HmdID)
{
	// Allocate a hmd view to track HMD state
	assert(m_hmdView == nullptr);
	PSM_AllocateHmdListener(HmdID);
	m_hmdView = PSM_GetHmd(HmdID);

	// Start receiving data from the controller
	setState(AppStage_HMDModelCalibration::pendingHmdStartRequest);

	PSMRequestID requestId;
	PSM_StartHmdDataStreamAsync(
		HmdID, 
		PSMStreamFlags_includePositionData |
		PSMStreamFlags_includeCalibratedSensorData | 
		PSMStreamFlags_includeRawSensorData |
		PSMStreamFlags_disableROI, 
		&requestId);
	PSM_RegisterCallback(requestId, &AppStage_HMDModelCalibration::handle_start_hmd_response, this);

}

void AppStage_HMDModelCalibration::handle_start_hmd_response(
	const PSMResponseMessage *response_message,
	void *userdata)
{
	AppStage_HMDModelCalibration *thisPtr = static_cast<AppStage_HMDModelCalibration *>(userdata);

	const PSMResult ResultCode = response_message->result_code;

	switch (ResultCode)
	{
	case PSMResult_Success:
	{
		thisPtr->request_tracker_list();
	} break;

	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
	{
		thisPtr->setState(AppStage_HMDModelCalibration::failedHmdStartRequest);
	} break;
	}
}

void AppStage_HMDModelCalibration::request_tracker_list()
{
	if (m_menuState != eMenuState::pendingTrackerListRequest)
	{
		setState(eMenuState::pendingTrackerListRequest);

		// Tell the psmove service that we we want a list of trackers connected to this machine
		PSMRequestID requestId;
		PSM_GetTrackerListAsync(&requestId);
		PSM_RegisterCallback(requestId, AppStage_HMDModelCalibration::handle_tracker_list_response, this);
	}
}

void AppStage_HMDModelCalibration::handle_tracker_list_response(
	const PSMResponseMessage *response_message,
	void *userdata)
{
	AppStage_HMDModelCalibration *thisPtr = static_cast<AppStage_HMDModelCalibration *>(userdata);

	switch (response_message->result_code)
	{
	case PSMResult_Success:
	{
		assert(response_message->payload_type == PSMResponseMessage::_responsePayloadType_TrackerList);
		const PSMTrackerList &tracker_list = response_message->payload.tracker_list;
		
		if (thisPtr->setup_tracker_pair(tracker_list))
		{
			thisPtr->setState(eMenuState::pendingTrackerStartRequest);
		}
		else
		{
			thisPtr->setState(eMenuState::failedTrackerListRequest);
		}
	} break;

	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
		{
			thisPtr->m_failureDetails = "Server Failure";
			thisPtr->setState(eMenuState::failedTrackerListRequest);
		} break;
	}
}

bool AppStage_HMDModelCalibration::setup_tracker_pair(const PSMTrackerList &tracker_list)
{
	bool bSuccess = true;

	if (tracker_list.count < 2)
	{
		m_failureDetails = "Need at least 2 cameras connected!";
		bSuccess = false;
	}

	// Find the pair of trackers within the capture constraints
	if (bSuccess)
	{
		int best_pair_a_index = -1;
		int best_pair_b_index = -1;
		float best_pair_distance = 0;

		for (int tracker_a_index = 0; tracker_a_index < tracker_list.count; ++tracker_a_index)
		{
			const PSMClientTrackerInfo *tracker_a_info = &tracker_list.trackers[tracker_a_index];

			PSMFrustum tracker_a_frustum;
			PSM_FrustumSetPose(&tracker_a_frustum, &tracker_a_info->tracker_pose);

			glm::vec3 tracker_a_pos = psm_vector3f_to_glm_vec3(tracker_a_frustum.origin);
			glm::vec3 tracker_a_forward = psm_vector3f_to_glm_vec3(tracker_a_frustum.forward);

			for (int tracker_b_index = tracker_a_index+1; tracker_b_index < tracker_list.count; ++tracker_b_index)
			{
				const PSMClientTrackerInfo *tracker_b_info = &tracker_list.trackers[tracker_b_index];

				PSMFrustum tracker_b_frustum;
				PSM_FrustumSetPose(&tracker_b_frustum, &tracker_b_info->tracker_pose);

				glm::vec3 tracker_b_pos = psm_vector3f_to_glm_vec3(tracker_b_frustum.origin);
				glm::vec3 tracker_b_forward = psm_vector3f_to_glm_vec3(tracker_b_frustum.forward);

				if (glm::dot(tracker_a_forward, tracker_b_forward) >= k_cosine_aligned_camera_angle)
				{
					const float test_distance = glm::distance(tracker_a_pos, tracker_b_pos);

					if (best_pair_distance <= 0 || test_distance < best_pair_distance)
					{
						best_pair_a_index = tracker_a_index;
						best_pair_b_index = tracker_b_index;
						best_pair_distance = test_distance;
					}
				}
			}
		}

		if (best_pair_a_index != -1 && best_pair_b_index != -1)
		{
			const PSMClientTrackerInfo *tracker_a_info = &tracker_list.trackers[best_pair_a_index];
			const PSMClientTrackerInfo *tracker_b_info = &tracker_list.trackers[best_pair_b_index];

			// T = Translation from camera A to camera B
			Eigen::Vector3f a_pos = psm_vector3f_to_eigen_vector3(tracker_a_info->tracker_pose.Position);
			Eigen::Vector3f b_pos = psm_vector3f_to_eigen_vector3(tracker_b_info->tracker_pose.Position);
			Eigen::Vector3f T = b_pos - a_pos;
			Eigen::Matrix3f S;
			S << 0.f, T.z(), -T.y(),
				-T.z(), 0.f, T.x(),
				T.y(), -T.x(), 0.f;

			// R = Rotation matrix from camera A to camera B
			Eigen::Quaternionf a_quat = psm_quatf_to_eigen_quaternionf(tracker_a_info->tracker_pose.Orientation);
			Eigen::Quaternionf b_quat = psm_quatf_to_eigen_quaternionf(tracker_b_info->tracker_pose.Orientation);
			Eigen::Quaternionf R_quat = a_quat.conjugate() * b_quat;
			Eigen::Matrix3f R = R_quat.toRotationMatrix();

			// Essential Matrix from A to B, depending on extrinsic parameters
			Eigen::Matrix3f E = R * S;

			// Get the intrinsic matrices for A and B
			PSMMatrix3f intrinsicA;
			PSMMatrix3f intrinsicB;
			PSM_GetTrackerIntrinsicMatrix(tracker_a_info->tracker_id, &intrinsicA);
			PSM_GetTrackerIntrinsicMatrix(tracker_b_info->tracker_id, &intrinsicB);
			Eigen::Matrix3f Ka= psm_matrix3f_to_eigen_matrix3(intrinsicA);
			Eigen::Matrix3f Kb= psm_matrix3f_to_eigen_matrix3(intrinsicB);

			// Compute the fundamental matrix from camera A to camera B
			m_trackerPairState->F_ab = Kb.inverse().transpose() * E * Ka.inverse();

			// Allocate tracker views for A and B
			PSM_AllocateTrackerListener(tracker_a_info->tracker_id, tracker_a_info);
			PSM_AllocateTrackerListener(tracker_b_info->tracker_id, tracker_b_info);

			m_trackerPairState->trackers.pair.a.trackerView= PSM_GetTracker(tracker_a_info->tracker_id);
			m_trackerPairState->trackers.pair.a.textureAsset = nullptr;
			m_trackerPairState->trackers.pair.b.trackerView = PSM_GetTracker(tracker_b_info->tracker_id);
			m_trackerPairState->trackers.pair.b.textureAsset = nullptr;
		}
		else
		{
			m_failureDetails = "Can't find two cameras facing roughly the same direction!";
			bSuccess = false;
		}
	}

	if (bSuccess)
	{
		for (int tracker_index = 0; tracker_index < 2; ++tracker_index)
		{
			PSMTracker *tracker_view = m_trackerPairState->trackers.list[tracker_index].trackerView;

			request_tracker_start_stream(tracker_view);
		}
	}

	return bSuccess;
}

void AppStage_HMDModelCalibration::request_tracker_start_stream(
	PSMTracker *tracker_view)
{
	setState(eMenuState::pendingTrackerStartRequest);

	// Increment the number of requests we're waiting to get back
	++m_trackerPairState->pendingTrackerStartCount;

	// Request data to start streaming to the tracker
	PSMRequestID requestID;
	PSM_StartTrackerDataStreamAsync(
		tracker_view->tracker_info.tracker_id, 
		&requestID);
	PSM_RegisterCallback(requestID, AppStage_HMDModelCalibration::handle_tracker_start_stream_response, this);
}

void AppStage_HMDModelCalibration::handle_tracker_start_stream_response(
	const PSMResponseMessage *response_message,
	void *userdata)
{
	AppStage_HMDModelCalibration *thisPtr = static_cast<AppStage_HMDModelCalibration *>(userdata);

	switch (response_message->result_code)
	{
	case PSMResult_Success:
	{
		// Get the tracker ID this request was for
		const PSMoveProtocol::Request *request = GET_PSMOVEPROTOCOL_REQUEST(response_message->opaque_request_handle);
		const int tracker_id = request->request_start_tracker_data_stream().tracker_id();

		TrackerState &tracker_A_state = thisPtr->m_trackerPairState->trackers.pair.a;
		TrackerState &tracker_B_state = thisPtr->m_trackerPairState->trackers.pair.b;

		// The context holds everything a handler needs to evaluate a response
		TrackerState &trackerState = (tracker_id == tracker_A_state.trackerView->tracker_info.tracker_id) ? tracker_A_state : tracker_B_state;

		// Open the shared memory that the video stream is being written to
		if (PSM_OpenTrackerVideoStream(trackerState.trackerView->tracker_info.tracker_id) == PSMResult_Success)
		{
			// Create a texture to render the video frame to
			trackerState.textureAsset = new TextureAsset();
			trackerState.textureAsset->init(
				static_cast<unsigned int>(trackerState.trackerView->tracker_info.tracker_screen_dimensions.x),
				static_cast<unsigned int>(trackerState.trackerView->tracker_info.tracker_screen_dimensions.y),
				GL_RGB, // texture format
				GL_BGR, // buffer format
				nullptr);

			// See if this was the last tracker we were waiting to get a response from
			--thisPtr->m_trackerPairState->pendingTrackerStartCount;
			if (thisPtr->m_trackerPairState->pendingTrackerStartCount <= 0)
			{
				thisPtr->handle_all_devices_ready();
			}
		}
        else
        {
			thisPtr->setState(eMenuState::failedTrackerStartRequest);
        }
	} break;

	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
		{
			thisPtr->setState(eMenuState::failedTrackerStartRequest);
		} break;
	}
}

void AppStage_HMDModelCalibration::request_set_hmd_led_model_calibration()
{

}

void AppStage_HMDModelCalibration::handle_all_devices_ready()
{
	if (!m_bBypassCalibration)
	{
		setState(eMenuState::verifyTrackers);
	}
	else
	{
		setState(eMenuState::test);
	}
}

//-- private methods -----
static glm::mat4 computeGLMCameraTransformMatrix(const PSMTracker *tracker_view)
{

	const PSMPosef pose = tracker_view->tracker_info.tracker_pose;
	const PSMQuatf &quat = pose.Orientation;
	const PSMVector3f &pos = pose.Position;

	const glm::quat glm_quat(quat.w, quat.x, quat.y, quat.z);
	const glm::vec3 glm_pos(pos.x, pos.y, pos.z);
	const glm::mat4 glm_camera_xform = glm_mat4_from_pose(glm_quat, glm_pos);

	return glm_camera_xform;
}

static cv::Matx34f computeOpenCVCameraExtrinsicMatrix(const PSMTracker *tracker_view)
{
	cv::Matx34f out;

	// Extrinsic matrix is the inverse of the camera pose matrix
	const glm::mat4 glm_camera_xform = computeGLMCameraTransformMatrix(tracker_view);
	const glm::mat4 glm_mat = glm::inverse(glm_camera_xform);

	out(0, 0) = glm_mat[0][0]; out(0, 1) = glm_mat[1][0]; out(0, 2) = glm_mat[2][0]; out(0, 3) = glm_mat[3][0];
	out(1, 0) = glm_mat[0][1]; out(1, 1) = glm_mat[1][1]; out(1, 2) = glm_mat[2][1]; out(1, 3) = glm_mat[3][1];
	out(2, 0) = glm_mat[0][2]; out(2, 1) = glm_mat[1][2]; out(2, 2) = glm_mat[2][2]; out(2, 3) = glm_mat[3][2];

	return out;
}

static cv::Matx33f computeOpenCVCameraIntrinsicMatrix(const PSMTracker *tracker_view)
{
	cv::Matx33f out;

	const PSMClientTrackerInfo &tracker_info = tracker_view->tracker_info;
	float F_PX = tracker_info.tracker_focal_lengths.x;
	float F_PY = tracker_info.tracker_focal_lengths.y;
	float PrincipalX = tracker_info.tracker_principal_point.x;
	float PrincipalY = tracker_info.tracker_principal_point.y;

	out(0, 0) = F_PX; out(0, 1) = 0.f; out(0, 2) = PrincipalX;
	out(1, 0) = 0.f; out(1, 1) = F_PY; out(1, 2) = PrincipalY;
	out(2, 0) = 0.f; out(2, 1) = 0.f; out(2, 2) = 1.f;

	return out;
}

static cv::Matx34f computeOpenCVCameraPinholeMatrix(const PSMTracker *tracker_view)
{
	cv::Matx34f extrinsic_matrix = computeOpenCVCameraExtrinsicMatrix(tracker_view);
	cv::Matx33f intrinsic_matrix = computeOpenCVCameraIntrinsicMatrix(tracker_view);
	cv::Matx34f pinhole_matrix = intrinsic_matrix * extrinsic_matrix;

	return pinhole_matrix;
}

static bool triangulateHMDProjections(
	PSMHeadMountedDisplay *hmd_view, 
	TrackerPairState *tracker_pair_state,
	std::vector<Eigen::Vector3f> &out_triangulated_points)
{
	const PSMTracker *TrackerViewA = tracker_pair_state->trackers.pair.a.trackerView;
	const PSMTracker *TrackerViewB = tracker_pair_state->trackers.pair.b.trackerView;

	PSMTrackingProjection trackingProjectionA;
	PSMTrackingProjection trackingProjectionB;

	out_triangulated_points.clear();

	// Triangulate tracking LEDs that both cameras can see
	bool bIsTracking= false;
	if (PSM_GetIsHmdTracking(hmd_view->HmdID, &bIsTracking) == PSMResult_Success)
	{
		if (bIsTracking &&
			PSM_GetHmdProjectionOnTracker(hmd_view->HmdID, TrackerViewA->tracker_info.tracker_id, &trackingProjectionA) &&
			PSM_GetHmdProjectionOnTracker(hmd_view->HmdID, TrackerViewB->tracker_info.tracker_id, &trackingProjectionB))
		{
			assert(trackingProjectionA.shape_type == PSMTrackingProjection::PSMShape_PointCloud);
			assert(trackingProjectionB.shape_type == PSMTrackingProjection::PSMShape_PointCloud);

			const PSMVector2f *pointsA = trackingProjectionA.shape.pointcloud.points;
			const PSMVector2f *pointsB = trackingProjectionB.shape.pointcloud.points;
			const int point_countA = trackingProjectionA.shape.pointcloud.point_count;
			const int point_countB = trackingProjectionB.shape.pointcloud.point_count;

			// See: http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
			cv::Mat projMatA = cv::Mat(computeOpenCVCameraPinholeMatrix(TrackerViewA));
			cv::Mat projMatB = cv::Mat(computeOpenCVCameraPinholeMatrix(TrackerViewB));

			// Make a set of the uncorrelated points on trackerB
			std::set<int> pointBIndices;
			for (int point_indexB = 0; point_indexB < point_countB; ++point_indexB)
			{
				pointBIndices.insert(point_indexB);
			}

			// For each point in one tracking projection A, 
			// try and find the corresponding point in the projection B
			for (int point_indexA = 0; point_indexA < point_countA; ++point_indexA)
			{
				const PSMVector2f &pointA = pointsA[point_indexA];
				const cv::Mat cvPointA = cv::Mat(cv::Point2f(pointA.x, pointA.y));

				for (auto it = pointBIndices.begin(); it != pointBIndices.end(); )
				{
					const int point_indexB = *it;
					const PSMVector2f &pointB = pointsB[point_indexB];
					cv::Mat cvPointB = cv::Mat(cv::Point2f(pointB.x, pointB.y));

					if (tracker_pair_state->do_points_correspond(cvPointA, cvPointB, tracker_pair_state->tolerance))
					{
						// Triangulate the world position from the two cameras
						cv::Mat point3D(1, 1, CV_32FC4);
						cv::triangulatePoints(projMatA, projMatB, cvPointA, cvPointB, point3D);

						// Get the world space position
						const float w = point3D.at<float>(3, 0);
						const Eigen::Vector3f triangulated_point(
							point3D.at<float>(0, 0) / w,
							point3D.at<float>(1, 0) / w,
							point3D.at<float>(2, 0) / w);

						// Add to the list of world space points we saw this frame
						out_triangulated_points.push_back(triangulated_point);

						// Remove the point index from the set of indices to consider 
						// so that it's not correlated with another point in tracker A
						it= pointBIndices.erase(it);
					}
					else
					{
						++it;
					}
				}
			}
		} 
	}

	return out_triangulated_points.size() >= 3;
}

static PSMVector2f projectWorldPositionOnTracker(
	const PSMVector3f &worldSpacePosition,
	const PSMTracker *trackerView)
{
	// Assume no distortion
	// TODO: Probably should get the distortion coefficients out of the tracker
	cv::Mat cvDistCoeffs(4, 1, cv::DataType<float>::type);
	cvDistCoeffs.at<float>(0) = 0;
	cvDistCoeffs.at<float>(1) = 0;
	cvDistCoeffs.at<float>(2) = 0;
	cvDistCoeffs.at<float>(3) = 0;

	// Use the identity transform for tracker relative positions
	cv::Mat rvec(3, 1, cv::DataType<double>::type, double(0));
	cv::Mat tvec(3, 1, cv::DataType<double>::type, double(0));

	// Convert the world space position into a tracker relative location
	PSMVector3f trackerRelativePosition = PSM_PosefInverseTransformPoint(&trackerView->tracker_info.tracker_pose, &worldSpacePosition);

	// Only one point to project
	std::vector<cv::Point3f> cvObjectPoints;
	cvObjectPoints.push_back(
		cv::Point3f(
			trackerRelativePosition.x,
			trackerRelativePosition.y,
			trackerRelativePosition.z));

	// Compute the camera intrinsic matrix in opencv format
	cv::Matx33f cvCameraMatrix = computeOpenCVCameraIntrinsicMatrix(trackerView);

	// Projected point 
	std::vector<cv::Point2f> projectedPoints;
	cv::projectPoints(cvObjectPoints, rvec, tvec, cvCameraMatrix, cvDistCoeffs, projectedPoints);

	PSMVector2f screenLocation = {projectedPoints[0].x, projectedPoints[1].y};

	return screenLocation;
}

static void drawHMD(PSMHeadMountedDisplay *hmdView, const glm::mat4 &transform)
{
	switch (hmdView->HmdType)
	{
	case PSMHmd_Morpheus:
		drawMorpheusModel(transform);
		break;
	}
}
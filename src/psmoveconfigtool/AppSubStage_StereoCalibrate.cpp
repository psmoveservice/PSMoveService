//-- includes -----
#include "AppSubStage_StereoCalibrate.h"
#include "AppStage_ComputeTrackerPoses.h"
#include "AppStage_TrackerSettings.h"
#include "App.h"
#include "AssetManager.h"
#include "Camera.h"
#include "GeometryUtility.h"
#include "Logger.h"
#include "MathAlignment.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "MathGLM.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"

#include "SDL_keycode.h"
#include "SDL_opengl.h"

#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <imgui.h>
#include <vector>

//-- constants -----
const float k_default_lens_separation= 8.5f; // cm
const float k_default_origin_offset= 75.f; // cm
const float k_max_epipolar_dist_error= 1.f; // px

const int k_desired_valid_sample_count= 100;
const int k_max_invalid_sample_count= 100;

//-- private structures -----
struct StereoPairSampleState
{
	ClientTrackerView *trackerViews[2];
	PSMovePose trackerPoses[2];
	Eigen::Matrix3f F_ab; // Fundamental matrix from tracker A to tracker B
	Eigen::Matrix3f F_ba; // Fundamental matrix from tracker B to tracker A
	bool bValid;

	int valid_ab_count;
	int valid_ba_count;
	int invalid_count;

	void init()
	{
		trackerViews[0]= nullptr;
		trackerViews[1]= nullptr;
		trackerPoses[0].Clear();
		trackerPoses[1].Clear();
		F_ab= Eigen::Matrix3f::Identity();
		F_ba= Eigen::Matrix3f::Identity();
		valid_ab_count= 0;
		valid_ba_count= 0;
		invalid_count= 0;
		bValid= false;
	}

	void compute_fundamental_matrices(
		const AppStage_ComputeTrackerPoses::t_tracker_state_map &trackerViewMap,
		const float separation,
		const float originOffset)
	{		
		int tracker_count= 0;
        for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator_const tracker_iter= trackerViewMap.begin(); 
			tracker_iter != trackerViewMap.end() && tracker_count < 2;
			++tracker_iter, ++tracker_count)
        {
            trackerViews[tracker_count] = tracker_iter->second.trackerView;
		}

		if (tracker_count == 2)
		{
			const ClientTrackerInfo &tracker_a_info = trackerViews[0]->getTrackerInfo();
			const ClientTrackerInfo &tracker_b_info = trackerViews[1]->getTrackerInfo();

			float half_seperation= separation / 2.f;
			Eigen::Vector3f Ta = Eigen::Vector3f(half_seperation, 0.f, -originOffset);
			Eigen::Vector3f Tb = Eigen::Vector3f(-half_seperation, 0.f, -originOffset);
			Eigen::Quaternionf Qa = Eigen::Quaternionf::Identity();
			Eigen::Quaternionf Qb = Eigen::Quaternionf::Identity();

			// Get the intrinsic matrices for A and B
			Eigen::Matrix3f Ka= psmove_matrix3x3_to_eigen_matrix3(tracker_a_info.getTrackerIntrinsicMatrix());
			Eigen::Matrix3f Kb= psmove_matrix3x3_to_eigen_matrix3(tracker_b_info.getTrackerIntrinsicMatrix());

			// Compute the fundamental matrix from camera A to camera B
			eigen_alignment_compute_camera_fundamental_matrix(Ta, Tb, Qa, Qb, Ka, Kb, F_ab);

			// Compute the fundamental matrix from camera B to camera A
			eigen_alignment_compute_camera_fundamental_matrix(Tb, Ta, Qb, Qa, Kb, Ka, F_ba);

			// Initially assume that tracker 0 is on the right
			trackerPoses[0].Position= PSMovePosition::create(half_seperation, 0.f, -originOffset);
			trackerPoses[1].Position= PSMovePosition::create(-half_seperation, 0.f, -originOffset);
			trackerPoses[0].Orientation= PSMoveQuaternion::identity();
			trackerPoses[1].Orientation= PSMoveQuaternion::identity();

			bValid= true;
		}
		else
		{
			bValid= false;
		}
	}

	bool getHasFailed() const
	{
		return invalid_count >= k_max_invalid_sample_count || 
				(valid_ab_count >= k_desired_valid_sample_count &&
				valid_ba_count >= k_desired_valid_sample_count);
	}

	bool getIsComplete() const
	{
		return getHasFailed() || 
				valid_ab_count >= k_desired_valid_sample_count ||
				valid_ba_count >= k_desired_valid_sample_count;
	}

	const PSMovePose *getTrackerAPose() const
	{
		 return valid_ab_count > valid_ba_count ? &trackerPoses[0] : &trackerPoses[1];
	}

	const PSMovePose *getTrackerBPose() const
	{
		 return valid_ab_count > valid_ba_count ? &trackerPoses[1] : &trackerPoses[0];
	}

	ClientTrackerView *getTrackerAViewMutable() 
	{
		return trackerViews[0];
	}

	ClientTrackerView *getTrackerBViewMutable()
	{
		return trackerViews[1];
	}

	void addSample(const ClientControllerView *controllerView)
	{
		const PSMoveRawTrackerData &trackerData= controllerView->GetRawTrackerData();
		const int trackerID0= trackerViews[0]->getTrackerId();
		const int trackerID1= trackerViews[1]->getTrackerId();

		PSMoveScreenLocation screenSample0, screenSample1;
		
		if (!getIsComplete() &&
			trackerData.GetPixelLocationOnTrackerId(trackerID0, screenSample0) &&
			trackerData.GetPixelLocationOnTrackerId(trackerID1, screenSample1))
		{
			bool bWithinTolerance= false;
			const float ab_distance= compute_ab_signed_epipolar_distance(screenSample0, screenSample1);
			const float ba_distance= compute_ba_signed_epipolar_distance(screenSample0, screenSample1);
			
			const bool bValidAB_Distance= ab_distance >= 0 && ab_distance < k_max_epipolar_dist_error;
			const bool bValidBA_Distance= ba_distance >= 0 && ba_distance < k_max_epipolar_dist_error;

			if (bValidAB_Distance && bValidBA_Distance)
			{
				if (ab_distance < ba_distance)
				{
					bWithinTolerance= true;
					++valid_ab_count;
				}
				else if (ba_distance < ab_distance)
				{
					bWithinTolerance= true;
					++valid_ba_count;
				}
				else
				{
					bWithinTolerance= true;
					++valid_ab_count;
					++valid_ba_count;
				}
			}
			else if (bValidAB_Distance)
			{
				bWithinTolerance= true;
				++valid_ba_count;
			}
			else if (bValidBA_Distance)
			{
				bWithinTolerance= true;
				++valid_ba_count;
			}

			if (!bWithinTolerance)
			{
				++invalid_count;
			}
		}
	}

	float compute_ab_signed_epipolar_distance(
		const PSMoveScreenLocation &point1,
		const PSMoveScreenLocation &point2)
	{
		const Eigen::Vector3f a(point1.x, point1.y, 1.f);
		const Eigen::Vector3f b(point2.x, point2.y, 1.f);
		const float epipolar_signed_distance = a.transpose() * F_ab * b;

		return epipolar_signed_distance;
	}

	float compute_ba_signed_epipolar_distance(
		const PSMoveScreenLocation &point1,
		const PSMoveScreenLocation &point2)
	{
		const Eigen::Vector3f a(point1.x, point1.y, 1.f);
		const Eigen::Vector3f b(point2.x, point2.y, 1.f);
		const float epipolar_signed_distance = a.transpose() * F_ba * b;

		return epipolar_signed_distance;
	}
};

//-- public methods -----
AppSubStage_StereoCalibrate::AppSubStage_StereoCalibrate(
    AppStage_ComputeTrackerPoses *parentStage)
    : m_stereoPairState(new StereoPairSampleState)
	, m_parentStage(parentStage)
    , m_menuState(AppSubStage_StereoCalibrate::eMenuState::invalid)
	, m_cameraLensSeparation(k_default_lens_separation)
	, m_cameraOriginOffset(k_default_origin_offset)
{
	m_stereoPairState->init();
}

AppSubStage_StereoCalibrate::~AppSubStage_StereoCalibrate()
{
	delete m_stereoPairState;
}

void AppSubStage_StereoCalibrate::enter()
{
    setState(AppSubStage_StereoCalibrate::eMenuState::initial);
}

void AppSubStage_StereoCalibrate::exit()
{
    setState(AppSubStage_StereoCalibrate::eMenuState::initial);
}

void AppSubStage_StereoCalibrate::update()
{
    const ClientControllerView *ControllerView= m_parentStage->get_calibration_controller_view();

    switch (m_menuState)
    {
    case AppSubStage_StereoCalibrate::eMenuState::initial:
        {
            setState(AppSubStage_StereoCalibrate::eMenuState::setCameraSeparation);
        } break;
    case AppSubStage_StereoCalibrate::eMenuState::setCameraSeparation:
    case AppSubStage_StereoCalibrate::eMenuState::setOffsetFromOrigin:
		break;
	case AppSubStage_StereoCalibrate::eMenuState::determineFundametalMatrix:
		{
			if (!m_stereoPairState->getIsComplete())
			{
				m_stereoPairState->addSample(ControllerView);
			}

			m_parentStage->update_tracker_video();
		}
		break;
    case AppSubStage_StereoCalibrate::eMenuState::calibrateStepSuccess:
    case AppSubStage_StereoCalibrate::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_StereoCalibrate::render()
{
    switch (m_menuState)
    {
    case AppSubStage_StereoCalibrate::eMenuState::initial:
        break;
    case AppSubStage_StereoCalibrate::eMenuState::setCameraSeparation:
        {
			float half_separation= m_cameraLensSeparation / 2.f;
			const float scale= 1.8f;
			glm::mat4 left = glm::scale(glm::translate(glm::mat4(1.f), glm::vec3(-half_separation, 0.f, 0.f)), glm::vec3(scale, scale, scale));
			glm::mat4 right = glm::scale(glm::translate(glm::mat4(1.f), glm::vec3(half_separation, 0.f, 0.f)), glm::vec3(scale, scale, scale));

			drawPS3EyeModel(left);
			drawPS3EyeModel(right);
			drawTransformedAxes(glm::mat4(1.f), 5.f);
        } break;
    case AppSubStage_StereoCalibrate::eMenuState::setOffsetFromOrigin:
		{
			float half_separation= m_cameraLensSeparation / 2.f;
			const float scale= 1.8f;
			glm::mat4 left = glm::scale(glm::translate(glm::mat4(1.f), glm::vec3(-half_separation, 0.f, -m_cameraOriginOffset)), glm::vec3(scale, scale, scale));
			glm::mat4 right = glm::scale(glm::translate(glm::mat4(1.f), glm::vec3(half_separation, 0.f, -m_cameraOriginOffset)), glm::vec3(scale, scale, scale));

			drawPS3EyeModel(left);
			drawPS3EyeModel(right);
			drawTransformedAxes(glm::mat4(1.f), 20.f);
		} break;
    case AppSubStage_StereoCalibrate::eMenuState::determineFundametalMatrix:
		{
			// Draw the video from the PoV of the current tracker
			m_parentStage->render_tracker_video();
		} break;
    case AppSubStage_StereoCalibrate::eMenuState::calibrateStepSuccess:
        break;
    case AppSubStage_StereoCalibrate::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_StereoCalibrate::renderUI()
{
    const float k_panel_width = 450.f;
    const char *k_window_title = "Compute Tracker Poses";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;
    const std::chrono::time_point<std::chrono::high_resolution_clock> now = 
        std::chrono::high_resolution_clock::now();

    switch (m_menuState)
    {
    case AppSubStage_StereoCalibrate::eMenuState::initial:
        break;
    case AppSubStage_StereoCalibrate::eMenuState::setCameraSeparation:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
			ImGui::Begin("Enter Lens Center Separation", nullptr, window_flags);

			ImGui::PushItemWidth(100.f);
			if (ImGui::InputFloat("Center-to-Center Separation (cm)", &m_cameraLensSeparation, 0.1f, 1.f, 1))
			{
				if (m_cameraLensSeparation < 1.f)
				{
					m_cameraLensSeparation = 1.f;
				}

				if (m_cameraLensSeparation > 25.f)
				{
					m_cameraLensSeparation = 25.f;
				}
			}
			ImGui::PopItemWidth();

			ImGui::Spacing();

            if (ImGui::Button("OK"))
            {
                setState(AppSubStage_StereoCalibrate::eMenuState::setOffsetFromOrigin);
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel"))
            {
                m_parentStage->setState(AppStage_ComputeTrackerPoses::eMenuState::verifyTrackers);
            }

            ImGui::End();
        } break;
    case AppSubStage_StereoCalibrate::eMenuState::setOffsetFromOrigin:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
			ImGui::Begin("Enter Origin Z-Offset", nullptr, window_flags);

			ImGui::PushItemWidth(100.f);
			if (ImGui::InputFloat("Z-Offset (cm)", &m_cameraOriginOffset, 0.5f, 1.f, 1))
			{
				if (m_cameraOriginOffset < 0.f)
				{
					m_cameraOriginOffset = 0.f;
				}

				if (m_cameraOriginOffset > 100.f)
				{
					m_cameraOriginOffset = 100.f;
				}
			}
			ImGui::PopItemWidth();

			ImGui::Spacing();

            if (ImGui::Button("OK"))
            {
                setState(AppSubStage_StereoCalibrate::eMenuState::determineFundametalMatrix);
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel"))
            {
                m_parentStage->setState(AppStage_ComputeTrackerPoses::eMenuState::verifyTrackers);
            }

            ImGui::End();
        } break;
    case AppSubStage_StereoCalibrate::eMenuState::determineFundametalMatrix:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 80.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 200));
            ImGui::Begin(k_window_title, nullptr, window_flags);

			if (m_stereoPairState->getIsComplete())
			{
				if (m_stereoPairState->getHasFailed())
				{
					ImGui::Text("Stereo calibration failed! Make sure the camera separation is accurate.");
				}
				else
				{
					ImGui::Text("Stereo calibration succeeded.");
				}
			}
			else
			{
				ImGui::Text("Please hold the controller in view of both cameras.");
			}

			const float ab_valid_fraction= static_cast<float>(m_stereoPairState->valid_ab_count) / static_cast<float>(k_desired_valid_sample_count);
			ImGui::Text("+X=1 -X=2");
			ImGui::SameLine();
			ImGui::ProgressBar(ab_valid_fraction, ImVec2(250, 20));

			const float ba_valid_fraction= static_cast<float>(m_stereoPairState->valid_ba_count) / static_cast<float>(k_desired_valid_sample_count);
			ImGui::Text("+X=2 -X=1");
			ImGui::SameLine();
			ImGui::ProgressBar(ba_valid_fraction, ImVec2(250, 20));

			const float invalid_fraction= static_cast<float>(m_stereoPairState->invalid_count) / static_cast<float>(k_max_invalid_sample_count);
			ImGui::Text("Unknown");
			ImGui::SameLine();
			ImGui::ProgressBar(invalid_fraction, ImVec2(250, 20));

            ImGui::Separator();

            if (m_parentStage->get_tracker_count() > 1)
            {
                ImGui::Text("Tracker #%d", m_parentStage->get_render_tracker_index() + 1);

                if (ImGui::Button("Previous Tracker"))
                {
                    m_parentStage->go_previous_tracker();
                }
                ImGui::SameLine();
                if (ImGui::Button("Next Tracker"))
                {
                    m_parentStage->go_next_tracker();
                }
            }

			ImGui::Separator();

			if (m_stereoPairState->getIsComplete())
			{
				if (m_stereoPairState->getHasFailed())
				{
					if (ImGui::Button("Trust me, it's fine"))
					{
						setState(AppSubStage_StereoCalibrate::eMenuState::calibrateStepSuccess);
					}
				}
				else
				{
					if (ImGui::Button("Continue"))
					{
						setState(AppSubStage_StereoCalibrate::eMenuState::calibrateStepSuccess);
					}
				}
				ImGui::SameLine();
				if (ImGui::Button("Restart"))
				{
					setState(AppSubStage_StereoCalibrate::eMenuState::initial);
				}
				ImGui::SameLine();
				if (ImGui::Button("Cancel"))
				{
					m_parentStage->setState(AppStage_ComputeTrackerPoses::eMenuState::verifyTrackers);
				}
			}
			else
			{
				if (ImGui::Button("Restart"))
				{
					setState(AppSubStage_StereoCalibrate::eMenuState::initial);
				}
				ImGui::SameLine();
				if (ImGui::Button("Cancel"))
				{
					m_parentStage->setState(AppStage_ComputeTrackerPoses::eMenuState::verifyTrackers);
				}
			}

            ImGui::End();
        } break;
    case AppSubStage_StereoCalibrate::eMenuState::calibrateStepSuccess:
    case AppSubStage_StereoCalibrate::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

//-- private methods -----
void AppSubStage_StereoCalibrate::setState(
    AppSubStage_StereoCalibrate::eMenuState newState)
{
    if (newState != m_menuState)
    {
        onExitState(m_menuState);
        onEnterState(newState);
        m_menuState = newState;
    }
}

void AppSubStage_StereoCalibrate::onExitState(
    AppSubStage_StereoCalibrate::eMenuState oldState)
{
    switch (oldState)
    {
    case AppSubStage_StereoCalibrate::eMenuState::invalid:
    case AppSubStage_StereoCalibrate::eMenuState::initial:
    case AppSubStage_StereoCalibrate::eMenuState::setCameraSeparation:
    case AppSubStage_StereoCalibrate::eMenuState::setOffsetFromOrigin:
    case AppSubStage_StereoCalibrate::eMenuState::determineFundametalMatrix:
    case AppSubStage_StereoCalibrate::eMenuState::calibrateStepSuccess:
    case AppSubStage_StereoCalibrate::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_StereoCalibrate::onEnterState(
    AppSubStage_StereoCalibrate::eMenuState newState)
{
    switch (newState)
    {
    case AppSubStage_StereoCalibrate::eMenuState::initial:
        {
			m_stereoPairState->init();
        }
        break;
    case AppSubStage_StereoCalibrate::eMenuState::setCameraSeparation:
		m_cameraLensSeparation= k_default_lens_separation;
		m_parentStage->m_app->getOrbitCamera()->resetOrientation();
        break;
    case AppSubStage_StereoCalibrate::eMenuState::setOffsetFromOrigin:
		m_cameraOriginOffset= k_default_origin_offset;
		m_parentStage->m_app->getOrbitCamera()->resetOrientation();
		m_parentStage->m_app->getOrbitCamera()->setCameraOrbitPitch(90.f);
		m_parentStage->m_app->getOrbitCamera()->setCameraOrbitRadius(400.f);
        break;
	case AppSubStage_StereoCalibrate::eMenuState::determineFundametalMatrix:
		m_stereoPairState->compute_fundamental_matrices(m_parentStage->m_trackerViews, m_cameraLensSeparation, m_cameraOriginOffset);
		break;
    case AppSubStage_StereoCalibrate::eMenuState::calibrateStepSuccess:
		{
			m_parentStage->request_set_tracker_pose(m_stereoPairState->getTrackerAPose(), m_stereoPairState->getTrackerAViewMutable());
			m_parentStage->request_set_tracker_pose(m_stereoPairState->getTrackerBPose(), m_stereoPairState->getTrackerBViewMutable());
		}
		break;
    case AppSubStage_StereoCalibrate::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}
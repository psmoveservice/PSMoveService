//-- inludes -----
#include "AppStage_OpticalCalibration.h"
#include "AppStage_ControllerSettings.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "ClientControllerView.h"
#include "GeometryUtility.h"
#include "Logger.h"
#include "MathAlignment.h"
#include "MathGLM.h"
#include "MathEigen.h"
#include "MathUtility.h"

#include "PSMoveProtocolInterface.h"
#include "Renderer.h"
#include "UIConstants.h"

#include "SDL_keycode.h"

#include <imgui.h>

#include <algorithm>

//-- statics ----
const char *AppStage_OpticalCalibration::APP_STAGE_NAME = "OpticalCalibration";

//-- constants -----
const double k_stabilize_wait_time_ms = 1000.f;
const int k_desired_noise_sample_count = 100;
const int k_sample_location_count = 10;

//-- definitions -----
struct PoseNoiseSamplesAtLocation
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Eigen::Vector3f position_samples[k_desired_noise_sample_count];
    Eigen::Quaternionf orientation_samples[k_desired_noise_sample_count];
	float projection_area_samples[k_desired_noise_sample_count];
	int sample_count;

	float position_variance_scalar; // cm^2
    float orientation_variance_scalar; // radians^2
	float avg_proj_area; // pixels^2

    void clear()
    {
        sample_count= 0;
		position_variance_scalar = 0;
		orientation_variance_scalar = 0.f;
		avg_proj_area = 0.f;
    }

    void computeStatistics()
    {
        const float N = static_cast<float>(sample_count);

		// Compute the mean of the projection areas
		avg_proj_area = 0.f;
		for (int sample_index = 0; sample_index < sample_count; sample_index++)
		{
			avg_proj_area += projection_area_samples[sample_index];
		}
		avg_proj_area /= N;

		// Compute the variance of distance around the mean position
		Eigen::Vector3f position_variance;
		eigen_vector3f_compute_mean_and_variance(
			position_samples, sample_count, nullptr, &position_variance);
		position_variance_scalar = fmaxf(fmaxf(position_variance.x(), position_variance.y()), position_variance.z());

		// Compute the average(mean) of the orientation samples
		Eigen::Quaternionf orientation_mean;
		eigen_quaternion_compute_normalized_weighted_average(
			orientation_samples, nullptr, sample_count, &orientation_mean);

        // Compute the variance in the angle of orientation around the mean
		orientation_variance_scalar = 0.f;
        for (int sample_index = 0; sample_index < sample_count; sample_index++)
        {
            const Eigen::Quaternionf &sample= orientation_samples[sample_index].normalized();
			float diff_from_mean = eigen_quaternion_unsigned_angle_between(sample, orientation_mean);

			orientation_variance_scalar += diff_from_mean*diff_from_mean;
        }
		orientation_variance_scalar /= (N - 1.f);
    }
};

struct PoseNoiseSampleSet
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	PoseNoiseSamplesAtLocation samplesAtLocation[k_sample_location_count];
	int completedSampleLocations;

	Eigen::Vector2f orientation_variance_curve;
	Eigen::Vector2f position_variance_curve;

	float near_proj_area;
	float far_proj_area;

	PoseNoiseSamplesAtLocation& getCurrentLocationSamples()
	{
		return samplesAtLocation[completedSampleLocations];
	}

	void clear()
	{
		for (int location_index = 0; location_index < k_sample_location_count; ++location_index)
		{
			samplesAtLocation[location_index].clear();
		}
		completedSampleLocations = 0;

		orientation_variance_curve = Eigen::Vector2f::Zero();
		position_variance_curve = Eigen::Vector2f::Zero();

		near_proj_area = 0.f;
		far_proj_area = 0.f;
	}

	void computeVarianceBestFit()
	{
		near_proj_area = -1.f;
		far_proj_area = -1.f;

		Eigen::Vector2f orientation_variance_area_samples[k_sample_location_count];
		Eigen::Vector2f position_variance_area_samples[k_sample_location_count];
		for (int location_index = 0; location_index < k_sample_location_count; ++location_index)
		{
			const float projection_area = samplesAtLocation[location_index].avg_proj_area;
			const float orientation_variance = samplesAtLocation[location_index].orientation_variance_scalar;
			const float position_variance = samplesAtLocation[location_index].position_variance_scalar;

			orientation_variance_area_samples[location_index] = Eigen::Vector2f(projection_area, orientation_variance);
			position_variance_area_samples[location_index] = Eigen::Vector2f(projection_area, position_variance);

			// Find the projection area closest for the closest sample
			if (near_proj_area < 0 || projection_area > near_proj_area)
			{
				near_proj_area = projection_area;
			}

			// Find the projection area closest for the farthest sample
			if (far_proj_area < 0 || projection_area < far_proj_area)
			{
				far_proj_area = projection_area;
			}
		}

		eigen_alignment_fit_least_squares_exponential(
			orientation_variance_area_samples, k_sample_location_count,
			&orientation_variance_curve);

		eigen_alignment_fit_least_squares_exponential(
			position_variance_area_samples, k_sample_location_count,
			&position_variance_curve);
	}

	float getOrientationVarianceForArea(float area) const
	{
		return orientation_variance_curve.y()*exp(orientation_variance_curve.x()*area);
	}

	float getPositionVarianceForArea(float area) const
	{
		return position_variance_curve.y()*exp(position_variance_curve.x()*area);
	}
};


//-- private methods -----
static bool isPressingSamplingButton(const ClientControllerView *controllerView);
static void drawController(ClientControllerView *controllerView, const glm::mat4 &transform);

//-- public methods -----
AppStage_OpticalCalibration::AppStage_OpticalCalibration(App *app)
    : AppStage(app)
    , m_menuState(AppStage_OpticalCalibration::inactive)
    , m_bBypassCalibration(false)
    , m_controllerView(nullptr)
    , m_isControllerStreamActive(false)
    , m_lastControllerSeqNum(-1)
	, m_lastProjectionArea(0.f)
	, m_bIsStableAndVisible(false)
	, m_bLastMulticamPositionValid(false)
	, m_bLastMulticamOrientationValid(false)
	, m_resetPoseButtonPressTime()
	, m_bResetPoseRequestSent(false)
    , m_poseNoiseSamplesSet(new PoseNoiseSampleSet)
	, m_bWaitForSampleButtonRelease(false)
{
	m_lastMulticamPosition = *k_psmove_position_origin;
	m_lastMulticamOrientation = PSMoveQuaternion::identity();
	m_lastControllerPose = PSMovePose::identity();
	memset(&m_trackerList, 0, sizeof(m_trackerList));
}

AppStage_OpticalCalibration::~AppStage_OpticalCalibration()
{
    delete m_poseNoiseSamplesSet;
}

void AppStage_OpticalCalibration::enter()
{
    const AppStage_ControllerSettings *controllerSettings =
        m_app->getAppStage<AppStage_ControllerSettings>();
    const AppStage_ControllerSettings::ControllerInfo *controllerInfo =
        controllerSettings->getSelectedControllerInfo();

	m_menuState = eCalibrationMenuState::inactive;

    // Reset all of the sampling state
	m_poseNoiseSamplesSet->clear();

    // Initialize the controller state
    assert(controllerInfo->ControllerID != -1);
    assert(m_controllerView == nullptr);
    m_controllerView = ClientPSMoveAPI::allocate_controller_view(controllerInfo->ControllerID);

	m_stableAndVisibleStartTime = std::chrono::time_point<std::chrono::high_resolution_clock>();
	m_bIsStableAndVisible = false;

    m_lastControllerSeqNum = -1;
	m_lastMulticamPosition = *k_psmove_position_origin;
	m_lastMulticamOrientation = PSMoveQuaternion::identity();
	m_lastControllerPose = PSMovePose::identity();
	m_bLastMulticamPositionValid = false;
	m_bLastMulticamOrientationValid = false;

	memset(&m_trackerList, 0, sizeof(m_trackerList));

	// Get a list of trackers first
	request_tracker_list();
}

void AppStage_OpticalCalibration::exit()
{
    assert(m_controllerView != nullptr);
    ClientPSMoveAPI::free_controller_view(m_controllerView);
    m_controllerView = nullptr;
    setState(eCalibrationMenuState::inactive);
}

void AppStage_OpticalCalibration::update()
{
    bool bControllerDataUpdatedThisFrame = false;
    bool bTimeDeltaValid = false;
    std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float, std::milli> sampleTimeDeltaMilli(0);

    if (m_isControllerStreamActive && m_controllerView->GetOutputSequenceNum() != m_lastControllerSeqNum)
    {
		PSMoveRawTrackerData rawTrackerData;
		rawTrackerData.Clear();

		m_bLastMulticamPositionValid = false;
		m_bLastMulticamOrientationValid = false;
		m_bLastProjectionAreaValid = false;

        switch(m_controllerView->GetControllerViewType())
        {
        case ClientControllerView::PSDualShock4:
            {
                rawTrackerData = m_controllerView->GetPSDualShock4View().GetRawTrackerData();

				if (rawTrackerData.bMulticamOrientationValid)
				{
					m_lastMulticamOrientation = rawTrackerData.MulticamOrientation;
					m_bLastMulticamOrientationValid = true;
				}
            }
            break;
        case ClientControllerView::PSMove:
            {
				rawTrackerData = m_controllerView->GetPSMoveView().GetRawTrackerData();

				m_lastMulticamOrientation = PSMoveQuaternion::identity();
				m_bLastMulticamOrientationValid = true;
            }
            break;
        }

		if (rawTrackerData.bMulticamPositionValid)
		{
			m_lastMulticamPosition = rawTrackerData.MulticamPosition;
			m_bLastMulticamPositionValid = true;
		}

		// Get the latest filtered controller pose
		m_lastControllerPose = m_controllerView->GetPose();

		// Compute the current projection area as the average of the projection areas
		// for each tracker that can see the controller
		{
			int projectionCount = 0;

			m_lastProjectionArea = 0.f;
			for (int trackerIndex = 0; trackerIndex < m_trackerList.count; ++trackerIndex)
			{
				int trackerId = m_trackerList.trackers[trackerIndex].tracker_id;

				PSMoveTrackingProjection projection;
				if (rawTrackerData.GetProjectionOnTrackerId(trackerId, projection))
				{
					m_lastProjectionArea+= projection.get_projection_area();
					++projectionCount;
				}
			}

			if (projectionCount > 0)
			{
				m_lastProjectionArea /= static_cast<float>(projectionCount);
				m_bLastProjectionAreaValid = true;
			}
		}

        m_lastControllerSeqNum = m_controllerView->GetOutputSequenceNum();
                
        bControllerDataUpdatedThisFrame = true;
    }

    switch (m_menuState)
    {
	case eCalibrationMenuState::pendingTrackerListRequest:
		{
		} break;
	case eCalibrationMenuState::waitingForStreamStartResponse:
        {
            if (bControllerDataUpdatedThisFrame)
            {
                if (m_bBypassCalibration)
                {
                    setState(AppStage_OpticalCalibration::test);
                }
                else
                {
                    setState(AppStage_OpticalCalibration::waitForStable);
                }
            }
        } break;
    case eCalibrationMenuState::failedStreamStart:
	case eCalibrationMenuState::failedTrackerListRequest:
	case eCalibrationMenuState::waitForStable:
		{
			if (m_bWaitForSampleButtonRelease && 
				!isPressingSamplingButton(m_controllerView))
			{
				m_bWaitForSampleButtonRelease = false;
			}

			if (!m_bWaitForSampleButtonRelease && 
				m_controllerView->GetIsGyroStable() && 
				m_controllerView->GetIsCurrentlyTracking() && 
				isPressingSamplingButton(m_controllerView))
			{
				if (m_bIsStableAndVisible)
				{
					std::chrono::duration<double, std::milli> stableDuration = now - m_stableAndVisibleStartTime;

					if (stableDuration.count() >= k_stabilize_wait_time_ms)
					{
						m_poseNoiseSamplesSet->getCurrentLocationSamples().clear();
						setState(eCalibrationMenuState::measureOpticalNoise);
					}
				}
				else
				{
					m_bIsStableAndVisible = true;
					m_stableAndVisibleStartTime = now;
				}
			}
			else
			{
				if (m_bIsStableAndVisible)
				{
					m_bIsStableAndVisible = false;
				}
			}
		} break;
    case eCalibrationMenuState::measureOpticalNoise:
        {
            if (m_controllerView->GetIsGyroStable() && 
				isPressingSamplingButton(m_controllerView))
            {
				PoseNoiseSamplesAtLocation &poseNoiseSamples = m_poseNoiseSamplesSet->getCurrentLocationSamples();

                // Record the next noise sample
                if (poseNoiseSamples.sample_count < k_desired_noise_sample_count &&
					m_bLastMulticamOrientationValid && m_bLastMulticamPositionValid && m_bLastProjectionAreaValid)
                {
					poseNoiseSamples.position_samples[poseNoiseSamples.sample_count] = 
						psmove_position_to_eigen_vector3(m_lastMulticamPosition);
					poseNoiseSamples.orientation_samples[poseNoiseSamples.sample_count] =
						psmove_quaternion_to_eigen_quaternionf(m_lastMulticamOrientation);
					poseNoiseSamples.projection_area_samples[poseNoiseSamples.sample_count] =
						m_lastProjectionArea;

                    ++poseNoiseSamples.sample_count;
                }

                // See if we have completed sampling at this location
                if (poseNoiseSamples.sample_count >= k_desired_noise_sample_count)
                {
                    // Compute position and orientation noise statistics
					poseNoiseSamples.computeStatistics();

					// Advance to the next location
					++m_poseNoiseSamplesSet->completedSampleLocations;

					// If all locations have been sampled,
					// compute the final calibration
					if (m_poseNoiseSamplesSet->completedSampleLocations >= k_sample_location_count)
					{
						// Compute a best fit function to the area/variance function
						m_poseNoiseSamplesSet->computeVarianceBestFit();

						// Tell the server about the new variance calibration
						if (m_controllerView->GetControllerViewType() == ClientControllerView::PSDualShock4)
						{
							request_set_optical_calibration(
								m_poseNoiseSamplesSet->position_variance_curve.y(), m_poseNoiseSamplesSet->position_variance_curve.x(),
								m_poseNoiseSamplesSet->orientation_variance_curve.y(), m_poseNoiseSamplesSet->orientation_variance_curve.x());
						}
						else
						{
							request_set_optical_calibration(
								m_poseNoiseSamplesSet->position_variance_curve.y(), m_poseNoiseSamplesSet->position_variance_curve.x(),
								0.f, 0.f);
						}

						setState(eCalibrationMenuState::measureComplete);
					}
					// Otherwise move on to the next location
					else
					{
						m_bWaitForSampleButtonRelease = true;
						setState(eCalibrationMenuState::waitForStable);
					}
                }
            }
            else
            {
                m_bIsStableAndVisible= false;
                setState(AppStage_OpticalCalibration::waitForStable);
            }
        } break;
    case eCalibrationMenuState::measureComplete:
    case eCalibrationMenuState::test:
        {
			if (m_controllerView->GetControllerViewType() == ClientControllerView::PSMove)
			{
				PSMoveButtonState resetPoseButtonState= m_controllerView->GetPSMoveView().GetButtonSelect();

				switch (resetPoseButtonState)
				{
				case PSMoveButtonState::PSMoveButton_DOWN:
					{
						m_resetPoseButtonPressTime= std::chrono::high_resolution_clock::now();
					} break;
				case PSMoveButtonState::PSMoveButton_PRESSED:
					{
						if (!m_bResetPoseRequestSent)
						{
							const float k_hold_duration_milli = 250.f;
							std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
							std::chrono::duration<float, std::milli> pressDurationMilli = now - m_resetPoseButtonPressTime;

							if (pressDurationMilli.count() >= k_hold_duration_milli)
							{
								ClientPSMoveAPI::eat_response(ClientPSMoveAPI::reset_orientation(m_controllerView, PSMoveQuaternion::identity()));
								m_bResetPoseRequestSent = true;
							}
						}
					} break;
				case PSMoveButtonState::PSMoveButton_UP:
					{
						m_bResetPoseRequestSent = false;
					} break;
				}
			}
			else if (m_controllerView->GetControllerViewType() == ClientControllerView::PSDualShock4 &&
					m_controllerView->GetPSDualShock4View().GetButtonOptions() == PSMoveButton_PRESSED)
			{
				ClientPSMoveAPI::eat_response(ClientPSMoveAPI::reset_orientation(m_controllerView, PSMoveQuaternion::identity()));
			}
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_OpticalCalibration::render()
{
    const float bigModelScale = 18.f;
    glm::mat4 scaleAndRotateModelX90= 
        glm::rotate(
            glm::scale(glm::mat4(1.f), glm::vec3(bigModelScale, bigModelScale, bigModelScale)),
            90.f, glm::vec3(1.f, 0.f, 0.f));  

	glm::mat4 controllerWorldTransform= glm::mat4(1.f);
	if (m_lastControllerSeqNum != -1 && 
		m_isControllerStreamActive &&
		m_controllerView->GetIsCurrentlyTracking())
	{
		PSMovePose psmove_space_pose = PSMovePose::create(m_lastMulticamPosition, m_lastMulticamOrientation);

		if (m_controllerView->GetControllerViewType() == ClientControllerView::PSMove)
		{
			psmove_space_pose.Orientation = m_lastControllerPose.Orientation;
		}

		controllerWorldTransform = psmove_pose_to_glm_mat4(psmove_space_pose);
	}

    switch (m_menuState)
    {
	case eCalibrationMenuState::pendingTrackerListRequest:
    case eCalibrationMenuState::waitingForStreamStartResponse:
    case eCalibrationMenuState::failedStreamStart:
	case eCalibrationMenuState::failedTrackerListRequest:
        {
        } break;
    case eCalibrationMenuState::waitForStable:
	case eCalibrationMenuState::measureOpticalNoise:
		{
			// Show the controller with optically derived pose
			if (m_controllerView->GetIsCurrentlyTracking())
			{
				drawController(m_controllerView, controllerWorldTransform);
				drawTransformedAxes(controllerWorldTransform, 200.f);
			}

			drawTransformedAxes(glm::mat4(1.f), 200.f);
			drawTrackerList(m_trackerList.trackers, m_trackerList.count);
		} break;
	case eCalibrationMenuState::measureComplete:
	case eCalibrationMenuState::test:
		{           
            // Show the controller with filtered pose
            if (m_controllerView->GetIsCurrentlyTracking() &&
                m_bLastMulticamPositionValid && m_bLastMulticamOrientationValid)
            {
                drawController(m_controllerView, controllerWorldTransform);
				drawTransformedAxes(controllerWorldTransform, 200.f);
            }

			drawTransformedAxes(glm::mat4(1.f), 200.f);
			drawTrackerList(m_trackerList.trackers, m_trackerList.count);
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_OpticalCalibration::renderUI()
{
    const float k_panel_width = 500;
    const char *k_window_title = "Controller Settings";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
	case eCalibrationMenuState::pendingTrackerListRequest:
    case eCalibrationMenuState::waitingForStreamStartResponse:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Waiting for server response...");

            ImGui::End();
        } break;
    case eCalibrationMenuState::failedStreamStart:
	case eCalibrationMenuState::failedTrackerListRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Failed server request!");

            if (ImGui::Button("Ok"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::SameLine();

            if (ImGui::Button("Return to Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::waitForStable:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 150));
            ImGui::Begin(k_window_title, nullptr, window_flags);

			ImGui::Text("Sample Location #%d / %d", m_poseNoiseSamplesSet->completedSampleLocations + 1, k_sample_location_count);
			
			if (m_bLastProjectionAreaValid)
			{
				ImGui::Text("Tracker Projection Area: %.1f pixels^2", m_lastProjectionArea);
			}
			else
			{
				ImGui::Text("Tracker Projection Area: [Not Visible]");
			}

			if (m_bWaitForSampleButtonRelease)
			{
				switch (m_controllerView->GetControllerViewType())
				{
				case ClientControllerView::PSMove:
					ImGui::Text("Release the Move button.");
					break;
				case ClientControllerView::PSDualShock4:
					ImGui::Text("Release the X button.");
					break;
				}
			}
			else
			{
				switch (m_controllerView->GetControllerViewType())
				{
				case ClientControllerView::PSMove:
					ImGui::Text("Hold the controller still and press the Move button.");
					ImGui::Text("Measurement will start once tracking light is visible to cameras.");
					break;
				case ClientControllerView::PSDualShock4:
					ImGui::TextWrapped("Hold the controller still and press the X button.");
					ImGui::TextWrapped("Measurement will start once tracking light is visible to cameras.");
					break;
				}

				if (m_bIsStableAndVisible)
				{
					std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
					std::chrono::duration<double, std::milli> stableDuration = now - m_stableAndVisibleStartTime;
					float fraction = static_cast<float>(stableDuration.count() / k_stabilize_wait_time_ms);

					ImGui::ProgressBar(fraction, ImVec2(250, 20));
					ImGui::Spacing();
				}
				else
				{
					ImGui::Text("Controller Destabilized! Waiting for stabilization..");
				}
			}

			if (ImGui::Button("Redo"))
			{
				m_poseNoiseSamplesSet->clear();
				setState(eCalibrationMenuState::waitForStable);
			}
			ImGui::SameLine();
			if (ImGui::Button("Cancel"))
			{
				request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
			}

            ImGui::End();
        } break;
    case eCalibrationMenuState::measureOpticalNoise:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 150));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            std::chrono::time_point<std::chrono::high_resolution_clock> now= std::chrono::high_resolution_clock::now();
			int sampleCount= m_poseNoiseSamplesSet->getCurrentLocationSamples().sample_count;
            float sampleFraction = static_cast<float>(sampleCount) / static_cast<float>(k_desired_noise_sample_count);

			ImGui::Text("Sample Location #%d / %d", m_poseNoiseSamplesSet->completedSampleLocations + 1, k_sample_location_count);
			ImGui::Text("Tracker Projection Area %.1f pixels^2", m_lastProjectionArea);
            ImGui::Text("[Sampling...]");
            ImGui::ProgressBar(sampleFraction, ImVec2(250, 20));

            if (ImGui::Button("Cancel"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::measureComplete:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

			ImGui::Text("Sampling complete.");
			ImGui::Text("Press OK to continue or Redo to recalibration.");

            if (ImGui::Button("Ok"))
            {
                m_controllerView->SetLEDOverride(0, 0, 0);
                setState(eCalibrationMenuState::test);
            }
            ImGui::SameLine();
            if (ImGui::Button("Redo"))
            {
				m_poseNoiseSamplesSet->clear();
                setState(eCalibrationMenuState::waitForStable);
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::test:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 160));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            if (m_bBypassCalibration)
            {
                ImGui::Text("Testing Tracking of Controller ID #%d", m_controllerView->GetControllerID());
            }
            else
            {
                ImGui::Text("Optical Calibration of Controller ID #%d complete!", m_controllerView->GetControllerID());
				ImGui::Text("Projection Area: %.1f px^2", m_lastProjectionArea);

				if (m_controllerView->GetControllerViewType() == ClientControllerView::PSDualShock4)
				{
					ImGui::Text("Position Var: %.4f cm^2, Orientation Var: %.4f rad^2",
						m_poseNoiseSamplesSet->getPositionVarianceForArea(m_lastProjectionArea),
						m_poseNoiseSamplesSet->getOrientationVarianceForArea(m_lastProjectionArea));
				}
				else
				{
					ImGui::Text("Position Var: %.4f cm^2",
						m_poseNoiseSamplesSet->getPositionVarianceForArea(m_lastProjectionArea));
				}
            }

			{
				const Eigen::Quaternionf eigen_quat = psmove_quaternion_to_eigen_quaternionf(m_controllerView->GetOrientation());
				const Eigen::EulerAnglesf euler_angles = eigen_quaternionf_to_euler_angles(eigen_quat);

				ImGui::Text("Attitude: %.2f, Heading: %.2f, Bank: %.2f", 
					euler_angles.get_attitude_degrees(), euler_angles.get_heading_degrees(), euler_angles.get_bank_degrees());
			}

			if (m_controllerView->GetControllerViewType() == ClientControllerView::PSDualShock4)
			{
				ImGui::TextWrapped(
					"[Press the Options button with controller pointed straight forward\n" \
					 "to recenter the controller]");
			}
			else if (m_controllerView->GetControllerViewType() == ClientControllerView::PSMove)
			{
				ImGui::TextWrapped(
					"[Hold the Select button with controller pointed forward\n" \
					"to recenter the controller]");
			}

            if (ImGui::Button("Ok"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::SameLine();

            if (ImGui::Button("Return to Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    default:
        assert(0 && "unreachable");
    }
}

//-- private methods -----
void AppStage_OpticalCalibration::setState(eCalibrationMenuState newState)
{
	if (newState != m_menuState)
	{
		onExitState(m_menuState);
		onEnterState(newState);

		m_menuState = newState;
	}
}

void AppStage_OpticalCalibration::onExitState(eCalibrationMenuState newState)
{
	switch (m_menuState)
	{
	case eCalibrationMenuState::inactive:
	case eCalibrationMenuState::pendingTrackerListRequest:
	case eCalibrationMenuState::failedTrackerListRequest:
	case eCalibrationMenuState::waitingForStreamStartResponse:
	case eCalibrationMenuState::failedStreamStart:
	case eCalibrationMenuState::waitForStable:
	case eCalibrationMenuState::measureOpticalNoise:
	case eCalibrationMenuState::measureComplete:
	case eCalibrationMenuState::test:
		break;
	default:
		assert(0 && "unreachable");
	}
}

void AppStage_OpticalCalibration::onEnterState(eCalibrationMenuState newState)
{
	switch (newState)
	{
	case eCalibrationMenuState::inactive:
		// Reset the orbit camera back to default orientation and scale
		m_app->getOrbitCamera()->reset();
		break;
	case eCalibrationMenuState::pendingTrackerListRequest:
		// Reset the menu state
		m_app->setCameraType(_cameraOrbit);
		m_app->getOrbitCamera()->reset();
		break;
	case eCalibrationMenuState::failedTrackerListRequest:
	case eCalibrationMenuState::waitingForStreamStartResponse:
	case eCalibrationMenuState::failedStreamStart:
		break;
	case eCalibrationMenuState::waitForStable:
		m_stableAndVisibleStartTime = std::chrono::high_resolution_clock::now();
		// Align the camera to face along the global forward
		// NOTE "0" degrees is down +Z in the ConfigTool View (rather than +X in the Service)
		m_app->getOrbitCamera()->reset();
		m_app->getOrbitCamera()->setCameraOrbitYaw(m_trackerList.global_forward_degrees - k_camera_default_forward_degrees);
		m_app->getOrbitCamera()->setCameraOrbitRadius(200);
		break;
	case eCalibrationMenuState::measureOpticalNoise:
	case eCalibrationMenuState::measureComplete:
	case eCalibrationMenuState::test:
		m_app->setCameraType(_cameraOrbit);
		m_app->getOrbitCamera()->reset();
		// Align the camera to face along the global forward
		// NOTE "0" degrees is down +Z in the ConfigTool View (rather than +X in the Service)
		m_app->getOrbitCamera()->setCameraOrbitYaw(m_trackerList.global_forward_degrees - k_camera_default_forward_degrees);
		m_app->getOrbitCamera()->setCameraOrbitRadius(200);
		break;
	default:
		assert(0 && "unreachable");
	}
}

void AppStage_OpticalCalibration::request_tracker_list()
{
	if (m_menuState != eCalibrationMenuState::pendingTrackerListRequest)
	{
	 	setState(eCalibrationMenuState::pendingTrackerListRequest);

		// Tell the psmove service that we we want a list of trackers connected to this machine
		ClientPSMoveAPI::register_callback(
			ClientPSMoveAPI::get_tracker_list(),
			AppStage_OpticalCalibration::handle_tracker_list_response, this);
	}
}

void AppStage_OpticalCalibration::handle_tracker_list_response(
	const ClientPSMoveAPI::ResponseMessage *response_message,
	void *userdata)
{
	AppStage_OpticalCalibration *thisPtr = static_cast<AppStage_OpticalCalibration *>(userdata);

	switch (response_message->result_code)
	{
	case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
		{
			assert(response_message->payload_type == ClientPSMoveAPI::_responsePayloadType_TrackerList);

			// Save the controller list state (used in rendering)
			thisPtr->m_trackerList = response_message->payload.tracker_list;

			// Start streaming in controller data
			assert(!thisPtr->m_isControllerStreamActive);
			ClientPSMoveAPI::register_callback(
				ClientPSMoveAPI::start_controller_data_stream(
					thisPtr->m_controllerView,
					ClientPSMoveAPI::includePositionData |
					ClientPSMoveAPI::includeRawTrackerData),
				&AppStage_OpticalCalibration::handle_acquire_controller, thisPtr);

			thisPtr->setState(eCalibrationMenuState::waitingForStreamStartResponse);
		} break;

	case ClientPSMoveAPI::_clientPSMoveResultCode_error:
	case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
		{
			thisPtr->setState(eCalibrationMenuState::failedTrackerListRequest);
		} break;
	}
}

void AppStage_OpticalCalibration::request_set_optical_calibration(
	const float position_var_exp_fit_a, const float position_var_exp_fit_b,
	const float orientation_var_exp_fit_a, const float orientation_var_exp_fit_b)
{
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_OPTICAL_NOISE_CALIBRATION);

    PSMoveProtocol::Request_RequestSetOpticalNoiseCalibration *calibration =
        request->mutable_request_set_optical_noise_calibration();

    calibration->set_controller_id(m_controllerView->GetControllerID());
	calibration->set_position_variance_exp_fit_a(position_var_exp_fit_a);
	calibration->set_position_variance_exp_fit_b(position_var_exp_fit_b);
	calibration->set_orientation_variance_exp_fit_a(orientation_var_exp_fit_a);
	calibration->set_orientation_variance_exp_fit_b(orientation_var_exp_fit_b);

    ClientPSMoveAPI::eat_response(ClientPSMoveAPI::send_opaque_request(&request));
}

void AppStage_OpticalCalibration::handle_acquire_controller(
    const ClientPSMoveAPI::ResponseMessage *response,
    void *userdata)
{
    AppStage_OpticalCalibration *thisPtr = reinterpret_cast<AppStage_OpticalCalibration *>(userdata);

    if (response->result_code == ClientPSMoveAPI::_clientPSMoveResultCode_ok)
    {
        thisPtr->m_isControllerStreamActive = true;
        thisPtr->m_lastControllerSeqNum = -1;
        // Wait for the first controller packet to show up...
    }
    else
    {
        thisPtr->setState(AppStage_OpticalCalibration::failedStreamStart);
    }
}

void AppStage_OpticalCalibration::request_exit_to_app_stage(const char *app_stage_name)
{
    ClientPSMoveAPI::eat_response(ClientPSMoveAPI::stop_controller_data_stream(m_controllerView));
    m_isControllerStreamActive= false;
    m_app->setAppStage(app_stage_name);
}

//-- private methods -----
static bool isPressingSamplingButton(const ClientControllerView *controllerView)
{
	bool bIsPressingButton = false;

	switch (controllerView->GetControllerViewType())
	{
	case ClientControllerView::PSMove:
		bIsPressingButton = controllerView->GetPSMoveView().GetButtonMove() == PSMoveButton_DOWN;
		break;
	case ClientControllerView::PSDualShock4:
		bIsPressingButton = controllerView->GetPSDualShock4View().GetButtonCross() == PSMoveButton_DOWN;
		break;
	}

	return bIsPressingButton;
}

static void drawController(ClientControllerView *controllerView, const glm::mat4 &transform)
{
    switch(controllerView->GetControllerViewType())
    {
    case ClientControllerView::PSMove:
        drawPSMoveModel(transform, glm::vec3(1.f, 1.f, 1.f));
        break;
    case ClientControllerView::PSDualShock4:
        drawPSDualShock4Model(transform, glm::vec3(1.f, 1.f, 1.f));
        break;
    }
}

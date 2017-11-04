//-- includes -----
#include "AppSubStage_CalibrateWithMat.h"
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
// Sample 5 points - The psmove standing on the 4 corners and the center of a sheet of paper
static const int k_mat_sample_location_count = 5;

// Take 60 samples at each location
static const int k_mat_calibration_sample_count = 60;

static const glm::vec3 k_psmove_frustum_color = glm::vec3(0.1f, 0.7f, 0.3f);

static const double k_stabilize_wait_time_ms = 1000.f;

static const float k_height_to_psmove_bulb_center = 17.7f; // cm - measured base to bulb center distance
static const float k_sample_x_location_offset = 14.f; // cm - Half the length of a 8.5'x11' sheet of paper
static const float k_sample_z_location_offset = 10.75f; // cm - Half the length of a 8.5'x11' sheet of paper

static const PSMVector3f k_sample_3d_locations[k_mat_sample_location_count] = {
    { k_sample_x_location_offset, k_height_to_psmove_bulb_center, k_sample_z_location_offset },
    { -k_sample_x_location_offset, k_height_to_psmove_bulb_center, k_sample_z_location_offset },
    { 0.f, k_height_to_psmove_bulb_center, 0.f },
    { -k_sample_x_location_offset, k_height_to_psmove_bulb_center, -k_sample_z_location_offset },
    { k_sample_x_location_offset, k_height_to_psmove_bulb_center, -k_sample_z_location_offset }
};
static const char *k_sample_location_names[k_mat_sample_location_count] = {
    "+X+Z Corner",
    "-X+Z Corner",
    "Center",
    "-X-Z Corner",
    "+X-Z Corner"
};

//-- private definitions -----
struct TrackerRelativePoseStatistics
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// N samples for the current sampling location
	PSMVector2f screenSpacePoints[k_mat_calibration_sample_count];
	Eigen::Vector3f trackerSpacePoints[k_mat_calibration_sample_count];
	int sampleCount;

	// Average for each sampling location
	PSMVector2f avgScreenSpacePointAtLocation[k_mat_sample_location_count];
	Eigen::Vector3f avgTrackerSpacePointAtLocation[k_mat_sample_location_count];

	PSMPosef trackerPose;
	float reprojectionError;
	bool bValidTrackerPose;

	TrackerRelativePoseStatistics()
	{
		clearAll();
	}

	bool getIsComplete() const
	{
		return sampleCount >= k_mat_calibration_sample_count;
	}

	void clearLastSampleBatch()
	{
		memset(screenSpacePoints, 0, sizeof(PSMVector2f)*k_mat_calibration_sample_count);
		memset(trackerSpacePoints, 0, sizeof(Eigen::Vector3f)*k_mat_calibration_sample_count);
		sampleCount = 0;
	}

	void clearAll()
	{
		clearLastSampleBatch();

		memset(avgScreenSpacePointAtLocation, 0, sizeof(PSMVector2f)*k_mat_sample_location_count);
		memset(avgTrackerSpacePointAtLocation, 0, sizeof(Eigen::Vector3f)*k_mat_sample_location_count);

		trackerPose = *k_psm_pose_identity;
		reprojectionError = 0.f;
		bValidTrackerPose = false;
	}

	void addControllerSample(const PSMTracker *trackerView, const PSMController *controllerView, const int sampleLocationIndex)
	{
		const int sampleTrackerID= trackerView->tracker_info.tracker_id;
        int streamTrackerID= -1;

		PSMVector2f screenSample;
		PSMVector3f trackerRelativePosition;
		
		if (!getIsComplete() &&
			PSM_GetControllerPixelLocationOnTracker(controllerView->ControllerID, &streamTrackerID, &screenSample) == PSMResult_Success &&
			PSM_GetControllerPositionOnTracker(controllerView->ControllerID, &streamTrackerID, &trackerRelativePosition) == PSMResult_Success &&
            streamTrackerID == sampleTrackerID)
		{
			screenSpacePoints[sampleCount] = screenSample;
			trackerSpacePoints[sampleCount] = psm_vector3f_to_eigen_vector3(trackerRelativePosition);
			++sampleCount;

			if (getIsComplete())
			{
				const float N = static_cast<float>(k_mat_calibration_sample_count);

				// Compute the average screen space location
				{
					PSMVector2f avg = {0, 0};

					// Average together all the samples we captured
					for (int sampleIndex = 0; sampleIndex < k_mat_calibration_sample_count; ++sampleIndex)
					{
						const PSMVector2f &sample = screenSpacePoints[sampleIndex];

						avg = PSM_Vector2fAdd(&avg, &sample);
					}
					avg = PSM_Vector2fUnsafeScalarDivide(&avg, N);

					// Save the average sample for this tracker at this location
					avgScreenSpacePointAtLocation[sampleLocationIndex] = avg;
				}
			}
		}
	}

	void addHmdSample(const PSMTracker *trackerView, const PSMHeadMountedDisplay *hmdView, const int sampleLocationIndex)
	{
		const int sampleTrackerID= trackerView->tracker_info.tracker_id;
        int streamTrackerID= -1;

		PSMVector2f screenSample;
		PSMVector3f trackerRelativePosition;
		
		if (!getIsComplete() &&
			PSM_GetHmdPixelLocationOnTracker(hmdView->HmdID, &streamTrackerID, &screenSample) == PSMResult_Success &&
			PSM_GetHmdPositionOnTracker(hmdView->HmdID, &streamTrackerID, &trackerRelativePosition) == PSMResult_Success &&
            streamTrackerID == sampleTrackerID)
		{
			screenSpacePoints[sampleCount] = screenSample;
			trackerSpacePoints[sampleCount] = psm_vector3f_to_eigen_vector3(trackerRelativePosition);
			++sampleCount;

			if (getIsComplete())
			{
				const float N = static_cast<float>(k_mat_calibration_sample_count);

				// Compute the average screen space location
				{
					PSMVector2f avg = {0, 0};

					// Average together all the samples we captured
					for (int sampleIndex = 0; sampleIndex < k_mat_calibration_sample_count; ++sampleIndex)
					{
						const PSMVector2f &sample = screenSpacePoints[sampleIndex];

						avg = PSM_Vector2fAdd(&avg, &sample);
					}
					avg = PSM_Vector2fUnsafeScalarDivide(&avg, N);

					// Save the average sample for this tracker at this location
					avgScreenSpacePointAtLocation[sampleLocationIndex] = avg;
				}
			}
		}
	}
};

//-- private methods -----
static bool computeTrackerCameraPose(
    const PSMTracker *trackerView,
    TrackerRelativePoseStatistics &trackerCoregData);

//-- public methods -----
AppSubStage_CalibrateWithMat::AppSubStage_CalibrateWithMat(
    AppStage_ComputeTrackerPoses *parentStage)
    : m_parentStage(parentStage)
    , m_menuState(AppSubStage_CalibrateWithMat::eMenuState::invalid)
    , m_bIsStable(false)
	, m_sampleLocationIndex(0)
    , m_bNeedMoreSamplesAtLocation(false)
{
	for (int location_index = 0; location_index < PSMOVESERVICE_MAX_TRACKER_COUNT; ++location_index)
	{
		m_deviceTrackerPoseStats[location_index] = new TrackerRelativePoseStatistics;
	}
}

AppSubStage_CalibrateWithMat::~AppSubStage_CalibrateWithMat()
{
	for (int location_index = 0; location_index < PSMOVESERVICE_MAX_TRACKER_COUNT; ++location_index)
	{
		delete m_deviceTrackerPoseStats[location_index];
	}
}

void AppSubStage_CalibrateWithMat::enter()
{
    setState(AppSubStage_CalibrateWithMat::eMenuState::initial);
}

void AppSubStage_CalibrateWithMat::exit()
{
    setState(AppSubStage_CalibrateWithMat::eMenuState::initial);
}

void AppSubStage_CalibrateWithMat::update()
{
    switch (m_menuState)
    {
    case AppSubStage_CalibrateWithMat::eMenuState::initial:
        {
            if (m_parentStage->get_calibration_controller_view() != nullptr)
            {
                // Go immediately to the initial place controller stage
                setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceController);
            }
            else if (m_parentStage->get_calibration_hmd_view() != nullptr)
            {
                // Go immediately to the initial place HMD stage
                setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD);
            }
            else
            {
                setState(AppSubStage_CalibrateWithMat::eMenuState::calibrateStepFailed);
            }
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceController:
        {
            const PSMController *ControllerView= m_parentStage->get_calibration_controller_view();

			bool bIsStable= false;
			bool bCanBeStable= PSM_GetIsControllerStable(ControllerView->ControllerID, &bIsStable) == PSMResult_Success;

            if ((bCanBeStable && bIsStable) || m_bForceStable)
            {
                std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();

                if (m_bIsStable)
                {
                    std::chrono::duration<double, std::milli> stableDuration = now - m_stableStartTime;

                    if (stableDuration.count() >= k_stabilize_wait_time_ms)
                    {
                        setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordController);
                    }
                }
                else
                {
                    m_bIsStable = true;
                    m_stableStartTime = now;
                }
            }
            else
            {
                if (m_bIsStable)
                {
                    m_bIsStable = false;
                }
            }

            // Poll the next video frame from the tracker rendering
            m_parentStage->update_tracker_video();
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordController:
        {
            const PSMController *ControllerView= m_parentStage->get_calibration_controller_view();

			bool bIsStable= false;
			bool bCanBeStable= PSM_GetIsControllerStable(ControllerView->ControllerID, &bIsStable) == PSMResult_Success;

            // See if any tracker needs more samples
            if (m_bNeedMoreSamplesAtLocation)
            {
                m_bNeedMoreSamplesAtLocation= false;

                for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                    iter != m_parentStage->m_trackerViews.end(); 
                    ++iter)
                {
                    const int trackerIndex = iter->second.listIndex;
                    const PSMTracker *trackerView = iter->second.trackerView;

                    if (m_deviceTrackerPoseStats[trackerIndex]->getIsComplete())
                    {
                        if (trackerView->tracker_info.tracker_id == m_sampleTrackerId)
                        {
                            m_sampleTrackerId= (m_sampleTrackerId + 1) % m_parentStage->m_trackerViews.size();

                            PSMRequestID requestId;
                            PSM_SetControllerDataStreamTrackerIndexAsync(ControllerView->ControllerID, m_sampleTrackerId, &requestId);
                            PSM_EatResponse(requestId);
                        }
                    }
                    else
                    {
                        m_bNeedMoreSamplesAtLocation = true;
                    }
                }
            }

            if (m_bNeedMoreSamplesAtLocation)
            {
                // Only record samples when the controller is stable
                if ((bCanBeStable && bIsStable) || m_bForceStable)
                {
                    for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                        iter != m_parentStage->m_trackerViews.end();
                        ++iter)
                    {
                        const int trackerIndex = iter->second.listIndex;
                        const PSMTracker *trackerView = iter->second.trackerView;

						bool bIsTracking= false;
						bool bCanBeTracked= PSM_GetIsControllerTracking(ControllerView->ControllerID, &bIsTracking) == PSMResult_Success;

                        if (bCanBeTracked && bIsTracking)
                        {
							m_deviceTrackerPoseStats[trackerIndex]->addControllerSample(trackerView, ControllerView, m_sampleLocationIndex);
                        }
                    }
                }
                else
                {
                    // Whoops! The controller got moved.
                    // Reset the sample count at this location for all trackers and wait for it 
                    setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceController);
                }
            }
            else
            {
                // If we have completed sampling at this location, wait until the controller is picked up
                if (!bIsStable)
                {
                    // Move on to next sample location
                    ++m_sampleLocationIndex;

                    if (m_sampleLocationIndex < k_mat_sample_location_count)
                    {
                        // If there are more sample locations
                        // wait until the controller stabilizes at the new location
                        setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceController);
                    }
                    else
                    {
                        setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepComputeTrackerPoses);
                    }
                }
            }

            // Poll the next video frame from the tracker rendering
            m_parentStage->update_tracker_video();
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD:
        {
            const PSMHeadMountedDisplay *HmdView= m_parentStage->get_calibration_hmd_view();

			bool bIsStable= false;
			bool bCanBeStable= PSM_GetIsHmdStable(HmdView->HmdID, &bIsStable) == PSMResult_Success;

            if ((bCanBeStable && bIsStable) || m_bForceStable)
            {
                std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();

                if (m_bIsStable)
                {
                    std::chrono::duration<double, std::milli> stableDuration = now - m_stableStartTime;

                    if (stableDuration.count() >= k_stabilize_wait_time_ms)
                    {
                        setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordHMD);
                    }
                }
                else
                {
                    m_bIsStable = true;
                    m_stableStartTime = now;
                }
            }
            else
            {
                if (m_bIsStable)
                {
                    m_bIsStable = false;
                }
            }

            // Poll the next video frame from the tracker rendering
            m_parentStage->update_tracker_video();
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordHMD:
        {
            const PSMHeadMountedDisplay *HmdView= m_parentStage->get_calibration_hmd_view();

			bool bIsStable= false;
			bool bCanBeStable= PSM_GetIsHmdStable(HmdView->HmdID, &bIsStable) == PSMResult_Success;

            // See if any tracker needs more samples
            if (m_bNeedMoreSamplesAtLocation)
            {
                m_bNeedMoreSamplesAtLocation = false;

                for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                    iter != m_parentStage->m_trackerViews.end(); 
                    ++iter)
                {
                    const int trackerIndex = iter->second.listIndex;
                    const PSMTracker *trackerView = iter->second.trackerView;

                    if (m_deviceTrackerPoseStats[trackerIndex]->getIsComplete())
                    {
                        if (trackerView->tracker_info.tracker_id == m_sampleTrackerId)
                        {
                            m_sampleTrackerId= (m_sampleTrackerId + 1) % m_parentStage->m_trackerViews.size();

                            PSMRequestID requestId;
                            PSM_SetHmdDataStreamTrackerIndexAsync(HmdView->HmdID, m_sampleTrackerId, &requestId);
                            PSM_EatResponse(requestId);
                        }
                    }
                    else
                    {
                        m_bNeedMoreSamplesAtLocation = true;
                    }
                }
            }

            if (m_bNeedMoreSamplesAtLocation)
            {
                // Only record samples when the controller is stable
                if ((bCanBeStable && bIsStable) || m_bForceStable)
                {
                    for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                        iter != m_parentStage->m_trackerViews.end();
                        ++iter)
                    {
                        const int trackerIndex = iter->second.listIndex;
                        const PSMTracker *trackerView = iter->second.trackerView;

						bool bIsTracking= false;
						bool bCanBeTracked= PSM_GetIsHmdTracking(HmdView->HmdID, &bIsTracking) == PSMResult_Success;

                        if (bCanBeTracked && bIsTracking)
                        {
							m_deviceTrackerPoseStats[trackerIndex]->addHmdSample(trackerView, HmdView, m_sampleLocationIndex);
                        }
                    }
                }
                else
                {
                    // Whoops! The HMD got moved.
                    // Reset the sample count at this location for all trackers and wait for it 
                    setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD);
                }
            }
            else
            {
                // If we have completed sampling at this location, wait until the HMD is picked up
                if (!bIsStable)
                {
                    // Move on to next sample location
                    ++m_sampleLocationIndex;

                    if (m_sampleLocationIndex < k_mat_sample_location_count)
                    {
                        // If there are more sample locations
                        // wait until the controller stabilizes at the new location
                        setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD);
                    }
                    else
                    {
                        setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepComputeTrackerPoses);
                    }
                }
            }

            // Poll the next video frame from the tracker rendering
            m_parentStage->update_tracker_video();
        }
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepComputeTrackerPoses:
        {
            bool bSuccess = true;

            // Compute and the pose transform for each tracker
            for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                bSuccess && iter != m_parentStage->m_trackerViews.end();
                ++iter)
            {
                const int trackerIndex = iter->second.listIndex;
                const PSMTracker *trackerView = iter->second.trackerView;
                TrackerRelativePoseStatistics &trackerSampleData = *m_deviceTrackerPoseStats[trackerIndex];

                bSuccess&= computeTrackerCameraPose(trackerView, trackerSampleData);
            }

            if (bSuccess)
            {
                // Update the poses on each local tracker view and notify the service of the new pose
                for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                    bSuccess && iter != m_parentStage->m_trackerViews.end();
                    ++iter)
                {
                    const int trackerIndex = iter->second.listIndex;
                    const TrackerRelativePoseStatistics &trackerSampleData = *m_deviceTrackerPoseStats[trackerIndex];
                    const PSMPosef trackerPose = trackerSampleData.trackerPose;

                    PSMTracker *trackerView = iter->second.trackerView;

                    m_parentStage->request_set_tracker_pose(&trackerPose, trackerView);
                }
            }

            if (bSuccess)
            {
                setState(AppSubStage_CalibrateWithMat::eMenuState::calibrateStepSuccess);
            }
            else
            {
                setState(AppSubStage_CalibrateWithMat::eMenuState::calibrateStepFailed);
            }
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepSuccess:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_CalibrateWithMat::render()
{
    switch (m_menuState)
    {
    case AppSubStage_CalibrateWithMat::eMenuState::initial:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceController:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordController:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordHMD:
        {
            // Draw the video from the PoV of the current tracker
            m_parentStage->render_tracker_video();
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepComputeTrackerPoses:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepSuccess:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_CalibrateWithMat::renderUI()
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
    case AppSubStage_CalibrateWithMat::eMenuState::initial:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceController:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            if (m_menuState == AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceController)
            {
                ImGui::Text("Stand the Controller upright on location #%d (%s)",
                    m_sampleLocationIndex + 1, k_sample_location_names[m_sampleLocationIndex]);
            }
            else
            {
                ImGui::Text("Stand the HMD upright on location #%d (%s)",
                    m_sampleLocationIndex + 1, k_sample_location_names[m_sampleLocationIndex]);
            }

            if (m_bIsStable)
            {
                std::chrono::duration<double, std::milli> stableDuration = now - m_stableStartTime;

                ImGui::Text("[stable for %d/%dms]", 
                    static_cast<int>(stableDuration.count()),
                    static_cast<int>(k_stabilize_wait_time_ms));
            }
            else
            {
                ImGui::Text("[Not stable and upright]");
            }

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

            if (ImGui::Button("Trust me, it's stable"))
            {
                m_bForceStable= true;
            }
            ImGui::SameLine();
            if (ImGui::Button("Restart Calibration"))
            {
                setState(AppSubStage_CalibrateWithMat::eMenuState::initial);
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel"))
            {
                m_parentStage->setState(AppStage_ComputeTrackerPoses::eMenuState::verifyTrackers);
            }

            ImGui::End();
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordController:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordHMD:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 200));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            if (m_menuState == AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordController)
            {
                ImGui::Text("Recording Controller samples at location #%d (%s)",
                    m_sampleLocationIndex + 1, k_sample_location_names[m_sampleLocationIndex]);
            }
            else
            {
                ImGui::Text("Recording HMD samples at location #%d (%s)",
                    m_sampleLocationIndex + 1, k_sample_location_names[m_sampleLocationIndex]);
            }

            bool bAnyTrackersSampling = false;
            for (int tracker_index = 0; tracker_index < m_parentStage->get_tracker_count(); ++tracker_index)
            {
                const int sampleCount = m_deviceTrackerPoseStats[tracker_index]->sampleCount;

                if (sampleCount < k_mat_calibration_sample_count)
                {
                    ImGui::Text("Tracker %d: sample %d/%d", tracker_index + 1, sampleCount, k_mat_calibration_sample_count);
                    bAnyTrackersSampling = true;
                }
                else
                {
                    ImGui::Text("Tracker %d: COMPLETE", tracker_index + 1);
                }
            }

            if (!bAnyTrackersSampling)
            {
                if (m_menuState == AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordController)
                {
                    ImGui::Text("Location sampling complete. Please pick up the controller.");
                }
                else
                {
                    ImGui::Text("Location sampling complete. Please pick up the HMD.");
                }
            }

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

            if (ImGui::Button("Cancel"))
            {
                m_parentStage->setState(AppStage_ComputeTrackerPoses::eMenuState::verifyTrackers);
            }

            ImGui::End();
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepComputeTrackerPoses:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepSuccess:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

//-- private methods -----
void AppSubStage_CalibrateWithMat::setState(
    AppSubStage_CalibrateWithMat::eMenuState newState)
{
    if (newState != m_menuState)
    {
        onExitState(m_menuState);
        onEnterState(newState);
        m_menuState = newState;
    }
}

void AppSubStage_CalibrateWithMat::onExitState(
    AppSubStage_CalibrateWithMat::eMenuState oldState)
{
    switch (oldState)
    {
    case AppSubStage_CalibrateWithMat::eMenuState::invalid:
    case AppSubStage_CalibrateWithMat::eMenuState::initial:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceController:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordController:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordHMD:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepComputeTrackerPoses:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepSuccess:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_CalibrateWithMat::onEnterState(
    AppSubStage_CalibrateWithMat::eMenuState newState)
{
    switch (newState)
    {
    case AppSubStage_CalibrateWithMat::eMenuState::initial:
        {
            for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                iter != m_parentStage->m_trackerViews.end();
                ++iter)
            {
                const int trackerIndex = iter->second.listIndex;

                m_deviceTrackerPoseStats[trackerIndex]->clearAll();
            }

            m_sampleLocationIndex = 0;
            m_bIsStable = false;
            m_bForceStable = false;
            m_sampleTrackerId= 0;
        }
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceController:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD:
        {
            for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                iter != m_parentStage->m_trackerViews.end();
                ++iter)
            {
                const int trackerIndex = iter->second.listIndex;

                m_deviceTrackerPoseStats[trackerIndex]->clearLastSampleBatch();
            }

            m_bIsStable = false;
            m_bForceStable= false;
            m_bNeedMoreSamplesAtLocation= true;
            m_sampleTrackerId= 0;

            // Start off getting getting projection data from tracker 0
            if (newState == AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceController)
            {
                const PSMController *ControllerView= m_parentStage->get_calibration_controller_view();
                PSMRequestID requestId;

                PSM_SetControllerDataStreamTrackerIndexAsync(ControllerView->ControllerID, 0, &requestId);
                PSM_EatResponse(requestId);
            }
            else if (newState == AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD)
            {
                const PSMHeadMountedDisplay *HmdView= m_parentStage->get_calibration_hmd_view();
                PSMRequestID requestId;

                PSM_SetHmdDataStreamTrackerIndexAsync(HmdView->HmdID, 0, &requestId);
                PSM_EatResponse(requestId);
            }
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordController:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordHMD:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepComputeTrackerPoses:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepSuccess:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

//-- math helper functions -----
static bool
computeTrackerCameraPose(
    const PSMTracker *trackerView,
    TrackerRelativePoseStatistics &trackerCoregData)
{
    // Get the pixel width and height of the tracker image
    const PSMVector2f trackerPixelDimensions = trackerView->tracker_info.tracker_screen_dimensions;

    // Get the tracker "intrinsic" matrix that encodes the camera FOV
    PSMMatrix3f cameraMatrix;
	PSM_GetTrackerIntrinsicMatrix(trackerView->tracker_info.tracker_id, &cameraMatrix);
    cv::Matx33f cvCameraMatrix = psmove_matrix3x3_to_cv_mat33f(cameraMatrix);

    // Copy the object/image point mappings into OpenCV format
    std::vector<cv::Point3f> cvObjectPoints;
    std::vector<cv::Point2f> cvImagePoints;
    for (int locationIndex = 0; locationIndex < k_mat_sample_location_count; ++locationIndex)
    {
        const PSMVector2f &screenPoint =
            trackerCoregData.avgScreenSpacePointAtLocation[locationIndex];
        const PSMVector3f &worldPoint =
            k_sample_3d_locations[locationIndex];

        // Add in the psmove calibration origin offset
        cvObjectPoints.push_back(cv::Point3f(worldPoint.x, worldPoint.y, worldPoint.z));

		//###HipsterSloth $TODO for some reason I need to invert the y points to get the correct tracker locations
		// I suspect this has something to do with how I am constructing the intrinsic matrix
        cvImagePoints.push_back(cv::Point2f(screenPoint.x, trackerPixelDimensions.y - screenPoint.y));
    }

    // Assume no distortion
    // TODO: Probably should get the distortion coefficients out of the tracker
    cv::Mat cvDistCoeffs(4, 1, cv::DataType<float>::type);
    cvDistCoeffs.at<float>(0) = 0;
    cvDistCoeffs.at<float>(1) = 0;
    cvDistCoeffs.at<float>(2) = 0;
    cvDistCoeffs.at<float>(3) = 0;

    // Solve the Project N-Point problem:
    // Given a set of 3D points and their corresponding 2D pixel projections,
    // solve for the cameras position and orientation that would allow
    // us to re-project the 3D points back onto the 2D pixel locations
    cv::Mat rvec(3, 1, cv::DataType<double>::type);
    cv::Mat tvec(3, 1, cv::DataType<double>::type);
    trackerCoregData.bValidTrackerPose = cv::solvePnP(cvObjectPoints, cvImagePoints, cvCameraMatrix, cvDistCoeffs, rvec, tvec);

    // Compute the re-projection error
    if (trackerCoregData.bValidTrackerPose)
    {
        std::vector<cv::Point2f> projectedPoints;
        cv::projectPoints(cvObjectPoints, rvec, tvec, cvCameraMatrix, cvDistCoeffs, projectedPoints);

        trackerCoregData.reprojectionError = 0.f;
        for (unsigned int i = 0; i < projectedPoints.size(); ++i)
        {
            const float xError = cvImagePoints[i].x - projectedPoints[i].x;
            const float yError = cvImagePoints[i].y - projectedPoints[i].y;
            const float squaredError = xError*xError + yError*yError;

            trackerCoregData.reprojectionError = squaredError;
        }
    }

    // Covert the rotation vector and translation into a GLM 4x4 transform
    if (trackerCoregData.bValidTrackerPose)
    {
        // Convert rvec to a rotation matrix
        cv::Mat R;
        cv::Rodrigues(rvec, R);

        float rotMat[9];
        for (int i = 0; i < 9; i++)
        {
            rotMat[i] = static_cast<float>(R.at<double>(i));
        }

        cv::Mat R_inv = R.t();
        cv::Mat tvecInv = -R_inv * tvec; // translation of the inverse R|t transform
        float tv[3];
        for (int i = 0; i < 3; i++)
        {
            tv[i] = static_cast<float>(tvecInv.at<double>(i));
        }

        float RTMat[] = {
            rotMat[0], rotMat[1], rotMat[2], 0.0f,
            rotMat[3], rotMat[4], rotMat[5], 0.0f,
            rotMat[6], rotMat[7], rotMat[8], 0.0f,
            tv[0], tv[1], tv[2], 1.0f };

        glm::mat4 trackerXform = glm::make_mat4(RTMat);

        // Save off the tracker pose in MultiCam Tracking space
        trackerCoregData.trackerPose = glm_mat4_to_psm_posef(trackerXform);
    }

    return trackerCoregData.bValidTrackerPose;
}
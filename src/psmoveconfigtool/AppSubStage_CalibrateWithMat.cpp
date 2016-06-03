//-- includes -----
#include "AppSubStage_CalibrateWithMat.h"
#include "AppStage_ComputeTrackerPoses.h"
#include "AppStage_TrackerSettings.h"
#include "App.h"
#include "AssetManager.h"
#include "Camera.h"
#include "ClientHMDView.h"
#include "GeometryUtility.h"
#include "Logger.h"
#include "OpenVRContext.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "MathGLM.h"

#include "SDL_keycode.h"
#include "SDL_opengl.h"

#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <imgui.h>
#include <vector>

//-- constants -----
static const glm::vec3 k_hmd_frustum_color = glm::vec3(1.f, 0.788f, 0.055f);
static const glm::vec3 k_psmove_frustum_color = glm::vec3(0.1f, 0.7f, 0.3f);

static const double k_stabilize_wait_time_ms = 1000.f;

static const float k_height_to_psmove_bulb_center = 17.7f; // cm - measured base to bulb center distance
static const float k_sample_x_location_offset = 14.f; // cm - Half the length of a 8.5'x11' sheet of paper
static const float k_sample_z_location_offset = 10.75f; // cm - Half the length of a 8.5'x11' sheet of paper

static const PSMovePosition k_sample_3d_locations[k_mat_sample_location_count] = {
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

//-- private methods -----
static glm::mat4 computePSMoveTrackerToHMDTrackerSpaceTransform(
    const ClientHMDView *hmdContext,
    const PSMovePose &psmoveCalibrationOffset,
    const HMDTrackerPoseContext &hmdTrackerPoseContext);
static bool computeTrackerCameraPose(
    const ClientTrackerView *trackerView,
    PS3EYETrackerPoseContext &trackerCoregData);

//-- public methods -----
AppSubStage_CalibrateWithMat::AppSubStage_CalibrateWithMat(
    AppStage_ComputeTrackerPoses *parentStage)
    : m_parentStage(parentStage)
    , m_menuState(AppSubStage_CalibrateWithMat::eMenuState::invalid)
    , m_bIsStable(false)
    , m_bForceHMDStable(false)
{
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
    const ClientControllerView *ControllerView= m_parentStage->m_controllerView;
    const ClientHMDView *HMDView = m_parentStage->m_hmdView;

    switch (m_menuState)
    {
    case AppSubStage_CalibrateWithMat::eMenuState::initial:
        {
            // Go immediately to the initial place PSMove stage
            setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove);
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove:
        {
            if (ControllerView->GetPSMoveView().GetIsStableAndAlignedWithGravity())
            {
                std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();

                if (m_bIsStable)
                {
                    std::chrono::duration<double, std::milli> stableDuration = now - m_stableStartTime;

                    if (stableDuration.count() >= k_stabilize_wait_time_ms)
                    {
                        setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordPSMove);
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
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordPSMove:
        {
            const ClientPSMoveView &PSMoveView= ControllerView->GetPSMoveView();
            const bool bIsStable = PSMoveView.GetIsStableAndAlignedWithGravity();

            // See if any tracker needs more samples
            bool bNeedMoreSamples = false;
            for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                iter != m_parentStage->m_trackerViews.end(); 
                ++iter)
            {
                const int trackerIndex = iter->second.listIndex;

                if (m_psmoveTrackerPoseContexts[trackerIndex].screenSpacePointCount < k_mat_calibration_sample_count)
                {
                    bNeedMoreSamples = true;
                    break;
                }
            }

            if (bNeedMoreSamples)
            {
                // Only record samples when the controller is stable
                if (bIsStable)
                {
                    for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                        iter != m_parentStage->m_trackerViews.end();
                        ++iter)
                    {
                        const int trackerIndex = iter->second.listIndex;
                        const ClientTrackerView *trackerView = iter->second.trackerView;

                        PSMoveScreenLocation screenSample;

                        if (PSMoveView.GetIsCurrentlyTracking() &&
                            PSMoveView.GetRawTrackerData().GetPixelLocationOnTrackerId(trackerView->getTrackerId(), screenSample) &&
                            m_psmoveTrackerPoseContexts[trackerIndex].screenSpacePointCount < k_mat_calibration_sample_count)
                        {
                            const int sampleCount = m_psmoveTrackerPoseContexts[trackerIndex].screenSpacePointCount;

                            m_psmoveTrackerPoseContexts[trackerIndex].screenSpacePoints[sampleCount] = screenSample;
                            ++m_psmoveTrackerPoseContexts[trackerIndex].screenSpacePointCount;

                            // See if we just read the last sample
                            if (m_psmoveTrackerPoseContexts[trackerIndex].screenSpacePointCount >= k_mat_calibration_sample_count)
                            {
                                const float N = static_cast<float>(k_mat_calibration_sample_count);
                                PSMoveFloatVector2 avg = PSMoveFloatVector2::create( 0, 0 );

                                // Average together all the samples we captured
                                for (int sampleIndex = 0; sampleIndex < k_mat_calibration_sample_count; ++sampleIndex)
                                {
                                    const PSMoveScreenLocation &sample =
                                        m_psmoveTrackerPoseContexts[trackerIndex].screenSpacePoints[sampleCount];

                                    avg = avg + sample.toPSMoveFloatVector2();
                                }
                                avg= avg.unsafe_divide(N);

                                // Save the average sample for this tracker at this location
                                m_psmoveTrackerPoseContexts[trackerIndex].avgScreenSpacePointAtLocation[m_sampleLocationIndex] = 
                                    PSMoveScreenLocation::create(avg.i, avg.j);
                            }
                        }
                    }
                }
                else
                {
                    // Whoops! The controller got moved.
                    // Reset the sample count at this location for all trackers and wait for it 
                    setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove);
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
                        setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove);
                    }
                    else
                    {
                        // Otherwise we are done with all of the PSMove sample locations.
                        // Move onto the next phase.
                        if (m_parentStage->m_hmdView != nullptr)
                        {
                            setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD);
                        }
                        else
                        {
                            setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepComputeTrackerPoses);
                        }
                    }
                }
            }

            // Poll the next video frame from the tracker rendering
            m_parentStage->update_tracker_video();
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD:
        {
            if (HMDView->getIsHMDStableAndAlignedWithGravity() || m_bForceHMDStable)
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
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordHMD:
        {
            // Only record samples when the controller is stable
            if (HMDView->getIsHMDStableAndAlignedWithGravity() || m_bForceHMDStable)
            {
                if (HMDView->getIsHMDTracking() &&
                    m_hmdTrackerPoseContext.worldSpaceSampleCount < k_mat_sample_location_count)
                {
                    const int sampleCount = m_hmdTrackerPoseContext.worldSpaceSampleCount;
                    const PSMovePose pose = HMDView->getHmdPose();

                    m_hmdTrackerPoseContext.worldSpacePoints[sampleCount] = pose.Position;
                    m_hmdTrackerPoseContext.worldSpaceOrientations[sampleCount] = pose.Orientation;
                    ++m_hmdTrackerPoseContext.worldSpaceSampleCount;

                    // See if we just read the last sample
                    if (m_hmdTrackerPoseContext.worldSpaceSampleCount >= k_mat_sample_location_count)
                    {
                        const float N = static_cast<float>(k_mat_sample_location_count);
                        PSMoveFloatVector3 avgPosition = *k_psmove_float_vector3_zero;
                        PSMoveQuaternion avgOrientation = *k_psmove_quaternion_identity;

                        // Average together all the samples we captured
                        for (int sampleIndex = 0; sampleIndex < k_mat_sample_location_count; ++sampleIndex)
                        {
                            const PSMovePosition &posSample =
                                m_hmdTrackerPoseContext.worldSpacePoints[sampleCount];
                            const PSMoveQuaternion &orientationSample =
                                m_hmdTrackerPoseContext.worldSpaceOrientations[sampleCount];

                            avgPosition = avgPosition + posSample.toPSMoveFloatVector3();
                            avgOrientation = avgOrientation + orientationSample;
                        }

                        // Save the average sample for the HMD
                        m_hmdTrackerPoseContext.avgHMDWorldSpacePoint = 
                            avgPosition.unsafe_divide(N).castToPSMovePosition();
                        m_hmdTrackerPoseContext.avgHMDWorldSpaceOrientation = 
                            avgOrientation.unsafe_divide(N).normalize_with_default(*k_psmove_quaternion_identity);

                        // Otherwise we are done with the HMD sampling.
                        // Move onto the next phase.
                        setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepComputeTrackerPoses);
                    }
                }
            }
            else
            {
                // Whoops! The HMD got moved.
                // Reset the sample count and wait for it to stabilize again
                setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD);
            }
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepComputeTrackerPoses:
        {
            bool bSuccess = true;

            // Compute and the pose transform for each tracker
            for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                bSuccess && iter != m_parentStage->m_trackerViews.end();
                ++iter)
            {
                const int trackerIndex = iter->second.listIndex;
                const ClientTrackerView *trackerView = iter->second.trackerView;
                PS3EYETrackerPoseContext &trackerSampleData = m_psmoveTrackerPoseContexts[trackerIndex];

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
                    const PS3EYETrackerPoseContext &trackerSampleData = m_psmoveTrackerPoseContexts[trackerIndex];
                    const PSMovePose trackerPose = trackerSampleData.trackerPose;

                    ClientTrackerView *trackerView = iter->second.trackerView;

                    m_parentStage->request_set_tracker_pose(&trackerPose, trackerView);
                }

                if (HMDView != nullptr)
                {
                    PSMovePose hmdPose;
                    hmdPose.Orientation = m_hmdTrackerPoseContext.avgHMDWorldSpaceOrientation;
                    hmdPose.Position = m_hmdTrackerPoseContext.avgHMDWorldSpacePoint;

                    m_parentStage->request_set_hmd_tracking_space_origin(&hmdPose);
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
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordPSMove:
        {
            // Draw the video from the PoV of the current tracker
            m_parentStage->render_tracker_video();

            // Draw the projection shape of the controller in the pov of the current tracker being rendered
            {
                const ClientTrackerView *TrackerView = m_parentStage->get_render_tracker_view();
                const ClientControllerView *ControllerView = m_parentStage->m_controllerView;
                const ClientPSMoveView &PSMoveView = ControllerView->GetPSMoveView();
                const PSMoveRawTrackerData &RawTrackerData= PSMoveView.GetRawTrackerData();
                const int TrackerID = TrackerView->getTrackerId();

                PSMoveTrackingProjection trackingProjection;
                PSMoveScreenLocation centerProjection;

                if (PSMoveView.GetIsCurrentlyTracking() &&
                    RawTrackerData.GetPixelLocationOnTrackerId(TrackerID, centerProjection) &&
                    RawTrackerData.GetProjectionOnTrackerId(TrackerID, trackingProjection))
                {
                    const PSMoveFloatVector2 screenSize= TrackerView->getTrackerInfo().tracker_screen_dimensions;

                    drawTrackingProjection(
                        &centerProjection,
                        &trackingProjection,
                        screenSize.i, screenSize.j,
                        glm::vec3(1.f, 1.f, 1.f));
                }
            }
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordHMD:
        {
            const ClientHMDView *HMDView = m_parentStage->m_hmdView;
            const glm::mat4 transform = psmove_pose_to_glm_mat4(HMDView->getHmdPose());

            drawDK2Model(transform);

            if (m_menuState == AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordHMD)
            {
                drawTransformedAxes(transform, 10.f);
            }
        }
        break;
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
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Stand the PSMove upright on location #%d (%s)",
                m_sampleLocationIndex + 1, k_sample_location_names[m_sampleLocationIndex]);

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

            if (ImGui::Button("Restart Calibration"))
            {
                setState(AppSubStage_CalibrateWithMat::eMenuState::initial);
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel"))
            {
                m_parentStage->setState(AppStage_ComputeTrackerPoses::eMenuState::selectCalibrationType);
            }

            ImGui::End();
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordPSMove:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 200));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Recording PSMove samples at location #%d (%s)",
                m_sampleLocationIndex + 1, k_sample_location_names[m_sampleLocationIndex]);

            bool bAnyTrackersSampling = false;
            for (int tracker_index = 0; tracker_index < m_parentStage->get_tracker_count(); ++tracker_index)
            {
                const int sampleCount = m_psmoveTrackerPoseContexts[tracker_index].screenSpacePointCount;

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
                ImGui::Text("Location sampling complete. Please pick up the controller.");
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
                m_parentStage->setState(AppStage_ComputeTrackerPoses::eMenuState::selectCalibrationType);
            }

            ImGui::End();
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Set the HMD at the tracking origin");

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

            if (ImGui::Button("Trust me, it's stable"))
            {
                m_bForceHMDStable= true;
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel"))
            {
                m_parentStage->setState(AppStage_ComputeTrackerPoses::eMenuState::selectCalibrationType);
            }

            ImGui::End();
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordHMD:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Recording HMD sample %d/%d",
                m_hmdTrackerPoseContext.worldSpaceSampleCount, 
                k_mat_calibration_sample_count);

            ImGui::Separator();

            if (ImGui::Button("Cancel"))
            {
                m_parentStage->setState(AppStage_ComputeTrackerPoses::eMenuState::selectCalibrationType);
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
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordPSMove:
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

                m_psmoveTrackerPoseContexts[trackerIndex].screenSpacePointCount = 0;
            }

            m_sampleLocationIndex = 0;
            m_bIsStable = false;
            m_hmdTrackerPoseContext.clear();
        }
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove:
        {
            for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                iter != m_parentStage->m_trackerViews.end();
                ++iter)
            {
                const int trackerIndex = iter->second.listIndex;

                m_psmoveTrackerPoseContexts[trackerIndex].screenSpacePointCount = 0;
            }

            m_bIsStable = false;
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordPSMove:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD:
        {
            m_bForceHMDStable = false;
            m_bIsStable = false;
            m_hmdTrackerPoseContext.worldSpaceSampleCount = 0;
        }
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordHMD:
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
    const ClientTrackerView *trackerView,
    PS3EYETrackerPoseContext &trackerCoregData)
{
    // Get the pixel width and height of the tracker image
    const PSMoveFloatVector2 trackerPixelDimensions = trackerView->getTrackerPixelExtents();

    // Get the tracker "intrinsic" matrix that encodes the camera FOV
    const PSMoveMatrix3x3 cameraMatrix = trackerView->getTrackerIntrinsicMatrix();
    cv::Matx33f cvCameraMatrix = psmove_matrix3x3_to_cv_mat33f(cameraMatrix);

    // Copy the object/image point mappings into OpenCV format
    std::vector<cv::Point3f> cvObjectPoints;
    std::vector<cv::Point2f> cvImagePoints;
    for (int locationIndex = 0; locationIndex < k_mat_sample_location_count; ++locationIndex)
    {
        const PSMoveScreenLocation &screenPoint =
            trackerCoregData.avgScreenSpacePointAtLocation[locationIndex];
        const PSMovePosition &worldPoint =
            k_sample_3d_locations[locationIndex];

        // Add in the psmove calibration origin offset
        cvObjectPoints.push_back(cv::Point3f(worldPoint.x, worldPoint.y, worldPoint.z));

        // Convert the tracker screen locations in PSMoveScreenLocation space
        // i.e. [-frameWidth/2, -frameHeight/2]x[frameWidth/2, frameHeight/2] 
        // into OpenCV pixel space
        // i.e. [0, 0]x[frameWidth, frameHeight]
        cvImagePoints.push_back(
            cv::Point2f(
                screenPoint.x + (trackerPixelDimensions.i / 2), 
                screenPoint.y + (trackerPixelDimensions.j / 2)));
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
        trackerCoregData.trackerPose = glm_mat4_to_psmove_pose(trackerXform);
    }

    return trackerCoregData.bValidTrackerPose;
}
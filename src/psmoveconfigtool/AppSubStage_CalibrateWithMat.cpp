//-- inludes -----
#include "AppSubStage_CalibrateWithMat.h"
#include "AppStage_ComputeTrackerPoses.h"
#include "App.h"
#include "AssetManager.h"
#include "Camera.h"
#include "ClientHMDView.h"
#include "Logger.h"
#include "OpenVRContext.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "MathGLM.h"

#include "SDL_keycode.h"
#include "SDL_opengl.h"

//-- constants -----
const double k_stabilize_wait_time_ms = 1000.f;

static const float k_height_to_psmove_bulb_center = 17.7; // cm - measured base to bulb center distance
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

//-- public methods -----
AppSubStage_CalibrateWithMat::AppSubStage_CalibrateWithMat(
    AppStage_ComputeTrackerPoses *parentStage)
    : m_parentStage(parentStage)
    , m_menuState(AppSubStage_CalibrateWithMat::eMenuState::inactive)
{
}

void AppSubStage_CalibrateWithMat::enter()
{
    setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepOriginPlacement);
}

void AppSubStage_CalibrateWithMat::exit()
{
    setState(AppSubStage_CalibrateWithMat::eMenuState::inactive);
}

void AppSubStage_CalibrateWithMat::update()
{
    const ClientControllerView *ControllerView= m_parentStage->m_controllerView;

    switch (m_menuState)
    {
    case AppSubStage_CalibrateWithMat::eMenuState::inactive:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepOriginPlacement:
        break;
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
                            PSMoveView.GetRawTrackerData().GetLocationForTrackerId(trackerIndex, screenSample) &&
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
                            setState(AppSubStage_CalibrateWithMat::eMenuState::calibrateStepComplete);
                        }
                    }
                }
            }
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordHMD:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepComplete:
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
    case AppSubStage_CalibrateWithMat::eMenuState::inactive:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepOriginPlacement:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordPSMove:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordHMD:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepComplete:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_CalibrateWithMat::renderUI()
{
    switch (m_menuState)
    {
    case AppSubStage_CalibrateWithMat::eMenuState::inactive:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepOriginPlacement:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordPSMove:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordHMD:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepComplete:
        break;
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
    case AppSubStage_CalibrateWithMat::eMenuState::inactive:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepOriginPlacement:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordPSMove:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordHMD:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepComplete:
        break;
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
    case AppSubStage_CalibrateWithMat::eMenuState::inactive:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepOriginPlacement:
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
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordHMD:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepComplete:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}
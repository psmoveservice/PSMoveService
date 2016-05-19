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

#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <vector>

//-- constants -----
const double k_stabilize_wait_time_ms = 1000.f;

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
static glm::mat4
computePSMoveTrackerToHMDTrackerSpaceTransform(
    const ClientHMDView *hmdContext,
    const HMDTrackerPoseContext &hmdTrackerPoseContext);
static bool computeTrackerCameraPose(
    const ClientTrackerView *trackerView, const glm::mat4 &psmoveTrackerToHmdTrackerSpace, 
    PS3EYETrackerPoseContext &trackerCoregData);
static cv::Matx33f PSMoveMoveMatrix3x3ToCvMat33f(const PSMoveMatrix3x3 &in);

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
    const ClientHMDView *HMDView = m_parentStage->m_hmdView;

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
                            setState(AppSubStage_CalibrateWithMat::eMenuState::calibrateStepSuccess);
                        }
                    }
                }
            }
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD:
        {
            if (HMDView->getIsStableAndAlignedWithGravity())
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
            if (HMDView->getIsStableAndAlignedWithGravity())
            {
                if (HMDView->getIsTracking() &&
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

            // If the hmd is valid,
            // compute a transform that puts the psmove trackers in the space of the hmd tracker
            glm::mat4 psmoveTrackerToHmdTrackerSpace = glm::mat4(1.f);
            if (HMDView != nullptr)
            {
                psmoveTrackerToHmdTrackerSpace =
                    computePSMoveTrackerToHMDTrackerSpaceTransform(
                        HMDView,
                        m_hmdTrackerPoseContext);
            }

            // Compute and the pose transform for each tracker
            for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                bSuccess && iter != m_parentStage->m_trackerViews.end();
                ++iter)
            {
                const int trackerIndex = iter->second.listIndex;
                const ClientTrackerView *trackerView = iter->second.trackerView;
                PS3EYETrackerPoseContext &trackerSampleData = m_psmoveTrackerPoseContexts[trackerIndex];

                bSuccess&= computeTrackerCameraPose(trackerView, psmoveTrackerToHmdTrackerSpace, trackerSampleData);
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
        {
            m_bIsStable = false;
            m_hmdTrackerPoseContext.worldSpaceSampleCount = 0;
        }
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordHMD:
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

//-- math helper functions -----
// Compute a transform that take a pose in PSMove tracking space 
// and converts it into a pose in HMD camera space
static glm::mat4
computePSMoveTrackerToHMDTrackerSpaceTransform(
    const ClientHMDView *hmdContext,
    const HMDTrackerPoseContext &hmdTrackerPoseContext)
{
    // Some useful definitions:
    // "PSMove Tracking Space"
    //   - The coordinate system that contains the PS3EYE tracking camera and poses
    //   - PS Move controller poses are converted into this space via 
    //     psmove_fusion_get_multicam_tracking_space_location()
    // "PSMove Calibration Space"
    //   - Inside of the "PSMove Tracking Space"
    //   - Represents locations relative to the PS3EYE Calibration Origin
    // "HMD Tracking Space"
    //   - The coordinate system that contains the HMD tracking camera and HMD poses
    // "HMD Camera Space"
    //   - Inside of the "HMD Camera space"
    //   - Represents locations relative to the HMD tracking camera

    //###HipsterSloth $TODO
#if 0
    // Compute a transform that goes from the HMD tracking space to the HMD camera space
    const OVR::Matrix4f hmdCameraToHmdTrackingSpace = hmdContext->getCameraTransform();
    const OVR::Matrix4f hmdTrackingToHmdCameraSpace = hmdCameraToHmdTrackingSpace.InvertedHomogeneousTransform();

    // During calibration we record the HMD pose at the PSMove calibration origin.
    // This pose represents the psmove calibration origin in HMD tracking space.
    const PSMovePosition &hmdCalibrationPos = hmdTrackerPoseContext.avgHMDWorldSpacePoint;
    const PSMoveQuaternion &hmdCalibrationOrientation = hmdTrackerPoseContext.avgHMDWorldSpaceOrientation;
    const OVR::Posef hmdCalibrationPose(
        OVR::Quatf(
        hmdCalibrationOrientation.x,
        hmdCalibrationOrientation.y,
        hmdCalibrationOrientation.z,
        hmdCalibrationOrientation.w).Normalized(),
        OVR::Vector3f(hmdCalibrationPos.x, hmdCalibrationPos.y, hmdCalibrationPos.z));
    const OVR::Matrix4f psmoveCalibrationToHmdTrackingSpace(hmdCalibrationPose);

    // The calibration target might be manually offset from origin of psmove tracking space.
    // Compute the transform that goes from psmove tracking space to calibration origin space.
    const OVR::Matrix4f psmoveCalibrationToPSMoveTrackingSpace =
        OVR::Matrix4f::Translation(psmove3AxisVectorToOvrVector3(psmoveCalibrationOffset));
    const OVR::Matrix4f psmoveTrackingToPSMoveCalibrationSpace =
        psmoveCalibrationToPSMoveTrackingSpace.InvertedHomogeneousTransform();

    // Compute the final transform that goes from PSMove tracking space to HMD Camera space
    // NOTE: Transforms are applied right to left
    const OVR::Matrix4f ovr_transform =
        hmdTrackingToHmdCameraSpace *
        psmoveCalibrationToHmdTrackingSpace *
        psmoveTrackingToPSMoveCalibrationSpace;
    const glm::mat4 glm_transform = ovrMatrix4fToGlmMat4(ovr_transform);
#endif
    const glm::mat4 glm_transform = glm::mat4(1.f);

    return glm_transform;
}

static bool
computeTrackerCameraPose(
    const ClientTrackerView *trackerView,
    const glm::mat4 &psmoveTrackerToHmdTrackerSpace,
    PS3EYETrackerPoseContext &trackerCoregData)
{
    // Get the pixel width and height of the tracker image
    const PSMoveFloatVector2 trackerPixelDimensions = trackerView->getTrackerPixelExtents();

    // Get the tracker "intrinsic" matrix that encodes the camera FOV
    const PSMoveMatrix3x3 cameraMatrix = trackerView->getTrackerIntrinsicMatrix();
    cv::Matx33f cvCameraMatrix = PSMoveMoveMatrix3x3ToCvMat33f(cameraMatrix);

    // Copy the object/image point mappings into OpenCV format
    std::vector<cv::Point3f> cvObjectPoints;
    std::vector<cv::Point2f> cvImagePoints;
    for (int locationIndex = 0; locationIndex < k_mat_calibration_sample_count; ++locationIndex)
    {
        const PSMoveScreenLocation &screenPoint =
            trackerCoregData.avgScreenSpacePointAtLocation[locationIndex];
        const PSMovePosition &worldPoint =
            k_sample_3d_locations[locationIndex];

        // Add in the psmove calibration origin offset
        cvObjectPoints.push_back(cv::Point3f(worldPoint.x, worldPoint.y, worldPoint.z));

        // Flip the pixel y coordinates
        cvImagePoints.push_back(cv::Point2f(screenPoint.x, trackerPixelDimensions.j - screenPoint.y));
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
        trackerCoregData.trackerPose = trackerXform;

        // Also save off the tracker pose relative to the HMD tracking camera.
        // NOTE: With GLM matrix multiplication the operation you want applied first
        // should be last in the multiplication.
        trackerCoregData.hmdCameraRelativeTrackerPose = psmoveTrackerToHmdTrackerSpace * trackerXform;
    }

    return trackerCoregData.bValidTrackerPose;
}

static cv::Matx33f
PSMoveMoveMatrix3x3ToCvMat33f(const PSMoveMatrix3x3 &in)
{
    // Both OpenCV and PSMoveMatrix3x3 matrices are stored row-major
    cv::Matx33f out; 
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            out(i, j) = in.m[i][j];
        }
    }

    return out;
}

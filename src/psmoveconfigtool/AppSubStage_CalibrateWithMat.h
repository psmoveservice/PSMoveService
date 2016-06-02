#ifndef APP_STAGE_COREGISTER_WITH_MAT_H
#define APP_STAGE_COREGISTER_WITH_MAT_H

//-- includes -----
#include "ClientGeometry.h"
#include "ClientConstants.h"
#include <chrono>

//-- constants -----
// Sample 5 points - The psmove standing on the 4 corners and the center of a sheet of paper
static const int k_mat_sample_location_count = 5;

// Take 60 samples at each location
static const int k_mat_calibration_sample_count = 60;

//-- definitions -----
struct PS3EYETrackerPoseContext
{
    PSMoveScreenLocation screenSpacePoints[k_mat_calibration_sample_count];
    int screenSpacePointCount;

    PSMoveScreenLocation avgScreenSpacePointAtLocation[k_mat_sample_location_count];

    PSMovePose trackerPose;
    float reprojectionError;
    bool bValidTrackerPose;

    void clear()
    {
        memset(screenSpacePoints, 0, sizeof(PSMoveScreenLocation)*k_mat_calibration_sample_count);
        memset(avgScreenSpacePointAtLocation, 0, sizeof(PSMoveScreenLocation)*k_mat_sample_location_count);
        screenSpacePointCount = 0;
        trackerPose= *k_psmove_pose_identity;
        reprojectionError = 0.f;
        bValidTrackerPose = false;
    }
};

struct HMDTrackerPoseContext
{
    PSMovePosition worldSpacePoints[k_mat_calibration_sample_count];
    PSMoveQuaternion worldSpaceOrientations[k_mat_calibration_sample_count];
    int worldSpaceSampleCount;

    PSMovePosition avgHMDWorldSpacePoint;
    PSMoveQuaternion avgHMDWorldSpaceOrientation;

    void clear()
    {
        memset(this, 0, sizeof(HMDTrackerPoseContext));
    }
};

class AppSubStage_CalibrateWithMat
{
public:
    enum eMenuState
    {
        invalid,

        initial,
        calibrationStepPlacePSMove,
        calibrationStepRecordPSMove,
        calibrationStepPlaceHMD,
        calibrationStepRecordHMD,
        calibrationStepComputeTrackerPoses,

        calibrateStepSuccess,
        calibrateStepFailed,
    };

    AppSubStage_CalibrateWithMat(class AppStage_ComputeTrackerPoses *parentStage);

    void enter();
    void exit();
    void update();
    void render();

    void renderUI();

    inline eMenuState getMenuState() const
    {
        return m_menuState;
    }

protected:
    void setState(eMenuState newState);
    void onExitState(eMenuState newState);
    void onEnterState(eMenuState newState);

private:
    class AppStage_ComputeTrackerPoses *m_parentStage;
    eMenuState m_menuState;

    std::chrono::time_point<std::chrono::high_resolution_clock> m_stableStartTime;
    bool m_bIsStable;

    HMDTrackerPoseContext m_hmdTrackerPoseContext;
    PS3EYETrackerPoseContext m_psmoveTrackerPoseContexts[PSMOVESERVICE_MAX_TRACKER_COUNT];

    int m_sampleLocationIndex;
};

#endif // APP_STAGE_COREGISTER_WITH_MAT_H
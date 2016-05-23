#ifndef APP_STAGE_COREGISTER_WITH_HMD_H
#define APP_STAGE_COREGISTER_WITH_HMD_H

//-- includes -----
#include "ClientGeometry.h"
#include "ClientConstants.h"

//-- constants -----
#define NPOSES 300

//-- definitions -----
struct FrustumBounds
{
    PSMovePosition origin;
    PSMoveFloatVector3 forward, left, up;
    float hAngleMin, hAngleMax; // radians
    float vAngleMin, vAngleMax; // radians
    float zNear, zFar;

    void clear();
    void init(const PSMoveFrustum &frustum);
    void enclosePoint(const PSMovePosition &point);
};

struct TrackerCoregistrationData
{
    FrustumBounds sampleBounds;
    PSMovePose hmd_poses[NPOSES];
    PSMovePosition psmoveposes[NPOSES];
    int poseCount;
    bool bComputedCoregTransform;

    PSMovePose hmdCameraRelativeTrackerPose;

    void clear()
    {
        sampleBounds.clear();
        poseCount = 0;
        hmdCameraRelativeTrackerPose = *k_psmove_pose_identity;
        bComputedCoregTransform = false;
    }
};

class AppSubStage_CalibrateWithHMD
{
public:
    enum eMenuState
    {
        initial,

        calibrationStepAttachPSMove,
        calibrationStepRecordHmdPSMove,
        calibrationStepComputeTrackerPoses,

        calibrateStepSuccess,
        calibrateStepFailed
    };

    AppSubStage_CalibrateWithHMD(class AppStage_ComputeTrackerPoses *parentStage);

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
    void onExitState(eMenuState oldState);
    void onEnterState(eMenuState newState);

private:
    class AppStage_ComputeTrackerPoses *m_parentStage;
    eMenuState m_menuState;

    TrackerCoregistrationData m_trackerCoreg[PSMOVESERVICE_MAX_TRACKER_COUNT];

    PSMovePose m_hmd_tracker_pose;
    bool m_sampled_hmd_tracker_pose;
};

#endif // APP_STAGE_COREGISTER_WITH_HMD_H
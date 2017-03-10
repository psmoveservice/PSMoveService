#ifndef APP_STAGE_COREGISTER_WITH_MAT_H
#define APP_STAGE_COREGISTER_WITH_MAT_H

//-- includes -----
#include "ClientGeometry_CAPI.h"
#include "ClientConstants.h"
#include <chrono>
#include <string.h>  // Required for memset in Xcode

//-- definitions -----
class AppSubStage_CalibrateWithMat
{
public:
    enum eMenuState
    {
        invalid,

        initial,
        calibrationStepPlacePSMove,
        calibrationStepRecordPSMove,
        calibrationStepComputeTrackerPoses,

        calibrateStepSuccess,
        calibrateStepFailed,
    };

    AppSubStage_CalibrateWithMat(class AppStage_ComputeTrackerPoses *parentStage);
	virtual ~AppSubStage_CalibrateWithMat();

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
    bool m_bForceControllerStable;

	struct TrackerRelativePoseStatistics *m_psmoveTrackerPoseStats[PSMOVESERVICE_MAX_TRACKER_COUNT];

    int m_sampleLocationIndex;
};

#endif // APP_STAGE_COREGISTER_WITH_MAT_H
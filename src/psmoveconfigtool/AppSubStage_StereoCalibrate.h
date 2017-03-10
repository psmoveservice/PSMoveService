#ifndef APP_STAGE_STEREO_CALIBRATE_H
#define APP_STAGE_STEREO_CALIBRATE_H

//-- includes -----
#include "ClientGeometry_CAPI.h"
#include "ClientConstants.h"
#include <chrono>
#include <string.h>  // Required for memset in Xcode

//-- definitions -----
class AppSubStage_StereoCalibrate
{
public:
    enum eMenuState
    {
        invalid,

        initial,
        setCameraSeparation,
		setOffsetFromOrigin,
		determineFundametalMatrix,

        calibrateStepSuccess,
        calibrateStepFailed,
    };

    AppSubStage_StereoCalibrate(class AppStage_ComputeTrackerPoses *parentStage);
	~AppSubStage_StereoCalibrate();

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
	struct StereoPairSampleState *m_stereoPairState;
    class AppStage_ComputeTrackerPoses *m_parentStage;
    eMenuState m_menuState;
	float m_cameraLensSeparation;
	float m_cameraOriginOffset;
};

#endif // APP_STAGE_STEREO_CALIBRATE_H
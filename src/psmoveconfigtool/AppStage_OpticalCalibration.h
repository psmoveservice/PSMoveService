#ifndef APP_STAGE_OPTICAL_CALIBRATION_H
#define APP_STAGE_OPTICAL_CALIBRATION_H

//-- includes -----
#include "AppStage.h"
#include "ClientGeometry_CAPI.h"
#include "PSMoveClient_CAPI.h"

#include <deque>
#include <chrono>

//-- pre-declarations -----

//-- definitions -----
class AppStage_OpticalCalibration : public AppStage
{
public:
	AppStage_OpticalCalibration(class App *app);
    virtual ~AppStage_OpticalCalibration();

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

    inline void setBypassCalibrationFlag(bool bFlag)
    {
        m_bBypassCalibration = bFlag;
    }

protected:
	enum eCalibrationMenuState
	{
		inactive,

		pendingTrackerListRequest,
		failedTrackerListRequest,
		waitingForStreamStartResponse,
		failedStreamStart,
		waitForStable,
		measureOpticalNoise,
		measureComplete,
		test
	};

	void setState(eCalibrationMenuState newState);
	void onExitState(eCalibrationMenuState newState);
	void onEnterState(eCalibrationMenuState newState);

	void request_tracker_list();
	static void handle_tracker_list_response(
		const PSMResponseMessage *response_message,
		void *userdata);
    void request_set_optical_calibration(
		const float position_var_exp_fit_a, const float position_var_exp_fit_b,
		const float orientation_var_exp_fit_a, const float orientation_var_exp_fit_b);
    static void handle_acquire_controller(
        const PSMResponseMessage *response,
        void *userdata);
    void request_exit_to_app_stage(const char *app_stage_name);

private:

    eCalibrationMenuState m_menuState;
    bool m_bBypassCalibration;

    PSMController *m_controllerView;
    bool m_isControllerStreamActive;
    int m_lastControllerSeqNum;

    PSMVector3f m_lastMulticamPositionCm;
	PSMQuatf m_lastMulticamOrientation;
	PSMPosef m_lastControllerPose;
	float m_lastProjectionArea;
	bool m_bLastMulticamPositionValid;
	bool m_bLastMulticamOrientationValid;
	bool m_bLastProjectionAreaValid;

	std::chrono::time_point<std::chrono::high_resolution_clock> m_stableAndVisibleStartTime;
	bool m_bIsStableAndVisible;

    struct PoseNoiseSampleSet *m_poseNoiseSamplesSet;
	bool m_bWaitForSampleButtonRelease;

	PSMTrackerList m_trackerList;
};

#endif // APP_STAGE_OPTICAL_CALIBRATION_H
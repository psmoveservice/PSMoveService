#ifndef APP_STAGE_HMD_MODEL_CALIBRATION_H
#define APP_STAGE_HMD_MODEL_CALIBRATION_H

//-- includes -----
#include "AppStage.h"
#include "ClientGeometry_CAPI.h"
#include "PSMoveClient_CAPI.h"

#include <deque>
#include <chrono>

//-- definitions -----
class AppStage_HMDModelCalibration : public AppStage
{
public:
	enum eMenuState
	{
		inactive,

		pendingHmdListRequest,
		failedHmdListRequest,

		pendingHmdStartRequest,
		failedHmdStartRequest,

		pendingTrackerListRequest,
		failedTrackerListRequest,

		pendingTrackerStartRequest,
		failedTrackerStartRequest,

		verifyTrackers,
		calibrate,
		test
	};

	AppStage_HMDModelCalibration(class App *app);
	virtual ~AppStage_HMDModelCalibration();

	static void enterStageAndCalibrate(App *app, int reqeusted_hmd_id);
	static void enterStageAndSkipCalibration(App *app, int reqeusted_hmd_id);

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
	void setState(eMenuState newState);
	void onExitState(eMenuState newState);
	void onEnterState(eMenuState newState);

	void update_tracker_video();
	void render_tracker_video();
	void go_next_tracker();
	void go_previous_tracker();
	int get_tracker_count() const;
	int get_render_tracker_index() const;
	PSMTracker *get_render_tracker_view() const;

	void request_hmd_list();
	static void handle_hmd_list_response(
		const PSMResponseMessage *response,
		void *userdata);

	void request_start_hmd_stream(int HmdID);
	static void handle_start_hmd_response(
		const PSMResponseMessage *response_message,
		void *userdata);

	void request_tracker_list();
	static void handle_tracker_list_response(
		const PSMResponseMessage *response_message,
		void *userdata);
	bool setup_tracker_pair(const PSMTrackerList &tracker_list);

	void request_tracker_start_stream(PSMTracker *tracker_view);
	static void handle_tracker_start_stream_response(
		const PSMResponseMessage *response,
		void *userdata);

	void request_set_hmd_led_model_calibration();

	void handle_all_devices_ready();

	void release_devices();
	void request_exit_to_app_stage(const char *app_stage_name);

private:
	eMenuState m_menuState;
	bool m_bBypassCalibration;

	struct TrackerPairState *m_trackerPairState;
	class HMDModelState *m_hmdModelState;

	PSMHeadMountedDisplay *m_hmdView;
	int m_overrideHmdId;

	std::string m_failureDetails;
};

#endif // APP_STAGE_HMD_MODEL_CALIBRATION_H

#ifndef APP_STAGE_COLOR_CALIBRATION_H
#define APP_STAGE_COLOR_CALIBRATION_H

//-- includes -----
#include "AppStage.h"
#include "ClientPSMoveAPI.h"

#include <vector>
#include <string>

//-- definitions -----
class AppStage_ColorCalibration : public AppStage
{
public:
	AppStage_ColorCalibration(class App *app);

	virtual void enter() override;
	virtual void exit() override;
	virtual void update() override;
	virtual void render() override;

	virtual void renderUI() override;

	static const char *APP_STAGE_NAME;

	void request_tracker_start_stream(int trackerID);
	void request_tracker_stop_stream(int trackerID);
	void request_tracker_set_exposure(int trackerID, double value);
	void request_tracker_get_settings(int trackerID);

protected:
	static void handle_tracker_start_stream_response(
		const ClientPSMoveAPI::ResponseMessage *response,
		void *userdata);
	void open_shared_memory_stream();

	static void handle_tracker_stop_stream_response(
		const ClientPSMoveAPI::ResponseMessage *response,
		void *userdata);
	void close_shared_memory_stream();

	static void handle_tracker_set_exposure_response(
		const ClientPSMoveAPI::ResponseMessage *response,
		void *userdata);
	static void handle_tracker_get_settings_response(
		const ClientPSMoveAPI::ResponseMessage *response,
		void *userdata);

private:
	struct TrackerOption
	{
		std::string option_name;
		std::vector<std::string> option_strings;
		int option_index;
	};

	enum eTrackerMenuState
	{
		inactive,
		idle,

		pendingTrackerStartStreamRequest,
		failedTrackerStartStreamRequest,

		pendingTrackerStopStreamRequest,
		failedTrackerStopStreamRequest,
	};

	enum eVideoDisplayMode
	{
		mode_bgr,
		mode_hsv,
		mode_hsv_range
	};

	eTrackerMenuState m_menuState;
	bool m_bStreamIsActive;
	class ClientTrackerView *m_trackerView;
	class TextureAsset *m_videoTexture;
	eVideoDisplayMode m_videoDisplayMode;

	double m_trackerExposure;
	double m_trackerGain;
	std::vector<TrackerOption> m_trackerOptions;
};

#endif // APP_STAGE_COLOR_CALIBRATION_H
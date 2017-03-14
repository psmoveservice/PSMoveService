#ifndef APP_STAGE_COLOR_CALIBRATION_H
#define APP_STAGE_COLOR_CALIBRATION_H

//-- includes -----
#include "AppStage.h"
#include "PSMoveClient_CAPI.h"

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

	inline void set_override_controller_id(int controller_id)
	{ m_overrideControllerId= controller_id; }

	inline void set_override_hmd_id(int hmd_id)
	{ m_overrideHmdId = hmd_id; }

	inline void set_override_tracking_color(PSMTrackingColorType tracking_color) {
		m_masterTrackingColorType = tracking_color;
	}
	
	inline void set_autoConfig(bool colour, bool controller, bool tracker) {
		m_bAutoChangeColor = colour;
		m_bAutoChangeController = controller;
		m_bAutoChangeTracker = tracker;
	}

protected:
    enum eMenuState
    {
        inactive,
        waitingForStreamStartResponse,
        manualConfig,
		autoConfig,
		blank1,
		blank2,
		blank3,
		changeController,
		changeTracker,

        pendingControllerStartRequest,
        failedControllerStartRequest,

		pendingHmdStartRequest,
		failedHmdStartRequest,

        pendingTrackerStartStreamRequest,
        failedTrackerStartStreamRequest,
    };

    enum eVideoDisplayMode
    {
        mode_bgr,
        mode_hsv,
        mode_masked,

        MAX_VIDEO_DISPLAY_MODES
    };

    struct TrackerOption
    {
        std::string option_name;
        std::vector<std::string> option_strings;
        int option_index;
    };

    struct TrackerColorPreset
    {
        float hue_center;
        float hue_range;
        float saturation_center;
        float saturation_range;
        float value_center;
        float value_range;
    };

    void setState(eMenuState newState);

    void request_start_controller_streams();
    static void handle_start_controller_response(
        const PSMResponseMessage *response_message,
        void *userdata);

	void request_start_hmd_stream();
	static void handle_start_hmd_response(
		const PSMResponseMessage *response_message,
		void *userdata);

    void request_set_controller_tracking_color(PSMController *controllerView, PSMTrackingColorType tracking_color);

    void request_tracker_start_stream();
    static void handle_tracker_start_stream_response(
        const PSMResponseMessage *response,
        void *userdata);

	void request_tracker_set_frame_rate(double value);
	static void handle_tracker_set_frame_rate_response(
		const PSMResponseMessage *response,
		void *userdata);

    void request_tracker_set_exposure(double value);
    static void handle_tracker_set_exposure_response(
        const PSMResponseMessage *response,
        void *userdata);

    void request_tracker_set_gain(double value);
    static void handle_tracker_set_gain_response(
        const PSMResponseMessage *response,
        void *userdata);

    void request_tracker_set_option(TrackerOption &option, int new_option_index);
    static void handle_tracker_set_option_response(
        const PSMResponseMessage *response,
        void *userdata);

    void request_tracker_set_color_preset(PSMTrackingColorType color_type, TrackerColorPreset &color_preset);
    static void handle_tracker_set_color_preset_response(
        const PSMResponseMessage *response,
        void *userdata);

    void request_tracker_get_settings();
    static void handle_tracker_get_settings_response(
        const PSMResponseMessage *response,
        void *userdata);

    void request_save_default_tracker_profile();
    void request_apply_default_tracker_profile();

    void allocate_video_buffers();
    void release_video_buffers();

    void release_devices();
    void request_exit_to_app_stage(const char *app_stage_name);

	void request_turn_on_all_tracking_bulbs(bool bEnabled);

	void request_change_controller(int step);
	void request_change_tracker(int step);

    inline TrackerColorPreset getColorPreset()
    { return m_colorPresets[m_masterTrackingColorType]; }

private:
    // ClientPSMoveAPI state
	int m_overrideControllerId;	
    PSMController *m_masterControllerView;
	std::vector<PSMController *> m_controllerViews;
	std::vector<PSMTrackingColorType> m_controllerTrackingColorTypes;
	int m_pendingControllerStartCount;
    bool m_areAllControllerStreamsActive;
    int m_lastMasterControllerSeqNum;
	int m_overrideHmdId;
	PSMHeadMountedDisplay *m_hmdView;
	bool m_isHmdStreamActive;
	int m_lastHmdSeqNum;

    PSMTracker *m_trackerView;

    // Menu state
    eMenuState m_menuState;
    class VideoBufferState *m_video_buffer_state;
    eVideoDisplayMode m_videoDisplayMode;

    // Tracker Settings state
	double m_trackerFramerate;
    double m_trackerExposure;
    double m_trackerGain;
    std::vector<TrackerOption> m_trackerOptions;
    TrackerColorPreset m_colorPresets[PSMTrackingColorType_MaxColorTypes];
	int tracker_count;
	int tracker_index;

    // Color Settings
	bool m_bTurnOnAllControllers;
    PSMTrackingColorType m_masterTrackingColorType;

	// Auto Calibration options
	bool m_bAutoChangeController;
	bool m_bAutoChangeColor;
	bool m_bAutoChangeTracker;

	// Setting Windows visability
	bool m_bShowWindows;
};

#endif // APP_STAGE_COLOR_CALIBRATION_H
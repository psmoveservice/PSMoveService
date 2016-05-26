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

protected:
    enum eMenuState
    {
        inactive,
        idle,

        pendingControllerStartRequest,
        failedControllerStartRequest,

        pendingTrackerStartStreamRequest,
        failedTrackerStartStreamRequest,
    };

    enum eVideoDisplayMode
    {
        mode_bgr,
        mode_hsv,
        mode_hsv_range,

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
        float hue_min;
        float hue_max;
        float saturation_min;
        float saturation_max;
        float value_min;
        float value_max;
    };

    void setState(eMenuState newState);

    void request_start_controller_stream();
    static void handle_start_controller_response(
        const ClientPSMoveAPI::ResponseMessage *response_message,
        void *userdata);

    void request_set_controller_tracking_color(PSMoveTrackingColorType tracking_color);

    void request_tracker_start_stream();
    static void handle_tracker_start_stream_response(
        const ClientPSMoveAPI::ResponseMessage *response,
        void *userdata);

    void request_tracker_set_exposure(double value);
    static void handle_tracker_set_exposure_response(
        const ClientPSMoveAPI::ResponseMessage *response,
        void *userdata);

    void request_tracker_set_gain(double value);
    static void handle_tracker_set_gain_response(
        const ClientPSMoveAPI::ResponseMessage *response,
        void *userdata);

    void request_tracker_set_option(TrackerOption &option, int new_option_index);
    static void handle_tracker_set_option_response(
        const ClientPSMoveAPI::ResponseMessage *response,
        void *userdata);

    void request_tracker_set_color_preset(PSMoveTrackingColorType color_type, TrackerColorPreset &color_preset);
    static void handle_tracker_set_color_preset_response(
        const ClientPSMoveAPI::ResponseMessage *response,
        void *userdata);

    void request_tracker_get_settings();
    static void handle_tracker_get_settings_response(
        const ClientPSMoveAPI::ResponseMessage *response,
        void *userdata);

    void release_devices();
    void request_exit_to_app_stage(const char *app_stage_name);

private:
    // ClientPSMoveAPI state
    class ClientControllerView *m_controllerView;
    class ClientTrackerView *m_trackerView;

    // Menu state
    eMenuState m_menuState;
    class TextureAsset *m_videoTexture;
    eVideoDisplayMode m_videoDisplayMode;

    // Tracker Settings state
    double m_trackerExposure;
    double m_trackerGain;
    std::vector<TrackerOption> m_trackerOptions;
    TrackerColorPreset m_colorPresets[PSMoveTrackingColorType::MAX_PSMOVE_COLOR_TYPES];

    // Color Settings
    PSMoveTrackingColorType m_trackingColorType;
};

#endif // APP_STAGE_COLOR_CALIBRATION_H
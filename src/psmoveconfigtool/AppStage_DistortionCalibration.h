#ifndef APP_STAGE_DISTORTION_CALIBRATION_H
#define APP_STAGE_DISTORTION_CALIBRATION_H

//-- includes -----
#include "AppStage.h"
#include "PSMoveClient_CAPI.h"

#include <vector>

//-- definitions -----
class AppStage_DistortionCalibration : public AppStage
{
public:
    AppStage_DistortionCalibration(class App *app);

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

    void request_tracker_start_stream();
    void request_tracker_stop_stream();
    void request_tracker_set_temp_gain(float gain);
    void request_tracker_set_temp_exposure(float exposure);
    void request_tracker_set_intrinsic(
        float focalLengthX, float focalLengthY,
        float principalX, float principalY,
        float distortionK1, float distortionK2, float distortionK3,
        float distortionP1, float distortionP2);
    void request_tracker_reload_settings();
    void request_exit();

protected:
    static void handle_tracker_start_stream_response(
        const PSMResponseMessage *response,
        void *userdata);
    void open_shared_memory_stream();

    static void handle_tracker_stop_stream_response(
        const PSMResponseMessage *response,
        void *userdata);
    void close_shared_memory_stream();
    
private:
    enum eMenuState
    {
        inactive,
		showWarning,
		enterBoardSettings,
        capture,
        complete,

        pendingTrackerStartStreamRequest,
        failedTrackerStartStreamRequest,
        failedTrackerOpenStreamRequest,

        pendingTrackerStopStreamRequest,
        failedTrackerStopStreamRequest,
    };

    enum eVideoDisplayMode
    {
        mode_bgr,
        mode_grayscale,
        mode_undistored,

        MAX_VIDEO_DISPLAY_MODES
    };

    // Menu state
    eMenuState m_menuState;
    eVideoDisplayMode m_videoDisplayMode;

	// Board Settings
	float m_square_length_mm;

    // Tracker Settings state
    float m_trackerExposure;
    float m_trackerGain;

    bool m_bStreamIsActive;
    PSMTracker *m_tracker_view;
    class TextureAsset *m_video_texture;
    class OpenCVBufferState *m_opencv_state;
};

#endif // APP_STAGE_DISTORTION_CALIBRATION_H
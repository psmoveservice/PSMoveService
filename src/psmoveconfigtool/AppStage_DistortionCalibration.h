#ifndef APP_STAGE_DISTORTION_CALIBRATION_H
#define APP_STAGE_DISTORTION_CALIBRATION_H

//-- includes -----
#include "AppStage.h"
#include "ClientPSMoveAPI.h"

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

protected:
    static void handle_tracker_start_stream_response(
        const ClientPSMoveAPI::ResponseMessage *response,
        void *userdata);
    void open_shared_memory_stream();

    static void handle_tracker_stop_stream_response(
        const ClientPSMoveAPI::ResponseMessage *response,
        void *userdata);
    void close_shared_memory_stream();
    
private:
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
        mode_grayscale,
        mode_undistored,

        MAX_VIDEO_DISPLAY_MODES
    };

    // Menu state
    eTrackerMenuState m_menuState;
    eVideoDisplayMode m_videoDisplayMode;

    bool m_bStreamIsActive;
    class ClientTrackerView *m_tracker_view;
    class TextureAsset *m_video_texture;
    class OpenCVBufferState *m_opencv_state;
};

#endif // APP_STAGE_DISTORTION_CALIBRATION_H
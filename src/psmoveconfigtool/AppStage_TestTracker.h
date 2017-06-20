#ifndef APP_STAGE_TEST_TRACKER_H
#define APP_STAGE_TEST_TRACKER_H

//-- includes -----
#include "AppStage.h"
#include "PSMoveClient_CAPI.h"

#include <vector>

//-- definitions -----
class AppStage_TestTracker : public AppStage
{
public:
    AppStage_TestTracker(class App *app);

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
        const PSMResponseMessage *response,
        void *userdata);
    void open_shared_memory_stream();

    static void handle_tracker_stop_stream_response(
        const PSMResponseMessage *response,
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

    eTrackerMenuState m_menuState;
    bool m_bStreamIsActive;
    PSMTracker *m_tracker_view;
    class TextureAsset *m_video_texture;
};

#endif // APP_STAGE_TEST_TRACKER_H
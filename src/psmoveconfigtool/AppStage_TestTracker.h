#ifndef APP_STAGE_TEST_TRACKER_H
#define APP_STAGE_TEST_TRACKER_H

//-- includes -----
#include "AppStage.h"
#include "ClientPSMoveAPI.h"

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
    int m_trackerID;
    bool m_bStreamIsActive;
    class SharedVideoFrameReadOnlyAccessor *m_shared_memory_accesor;
    double m_trackerExposure;
};

#endif // APP_STAGE_TEST_TRACKER_H
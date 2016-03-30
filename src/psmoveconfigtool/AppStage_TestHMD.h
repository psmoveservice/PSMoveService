#ifndef APP_STAGE_TEST_HMD_H
#define APP_STAGE_TEST_HMD_H

//-- includes -----
#include "AppStage.h"
#include "ClientPSMoveAPI.h"

#include <vector>

//-- definitions -----
class AppStage_TestHMD : public AppStage
{
public:
    AppStage_TestHMD(class App *app);

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

protected:
    static void handle_hmd_start_stream_response(
        ClientPSMoveAPI::eClientPSMoveResultCode ResultCode,
        const ClientPSMoveAPI::t_request_id request_id,
        ClientPSMoveAPI::t_response_handle response_handle,
        void *userdata);
    void request_exit_to_app_stage(const char *app_stage_name);
    static void handle_hmd_stop_stream_response(
        ClientPSMoveAPI::eClientPSMoveResultCode ResultCode,
        const ClientPSMoveAPI::t_request_id request_id,
        ClientPSMoveAPI::t_response_handle response_handle,
        void *userdata);

private:
    enum eHmdMenuState
    {
        inactive,
        idle,

        pendingHmdStartStreamRequest,
        failedHmdStartStreamRequest,

        pendingHmdStopStreamRequest,
        failedHmdStopStreamRequest,
    };

    eHmdMenuState m_menuState;
    const char *m_pendingAppStage;

    ClientHMDView *m_hmdView;
    bool m_isHmdStreamActive;
    int m_lastHmdSeqNum;
};

#endif // APP_STAGE_TEST_HMD_H
#ifndef APP_STAGE_COMPUTE_TRACKER_POSES_H
#define APP_STAGE_COMPUTE_TRACKER_POSES_H

//-- includes -----
#include "AppStage.h"
#include "ClientPSMoveAPI.h"

//-- definitions -----
class AppStage_ComputeTrackerPoses : public AppStage
{
public:
    AppStage_ComputeTrackerPoses(class App *app);

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

protected:
    void request_exit_to_app_stage(const char *app_stage_name);

private:
    enum eMenuState
    {
        inactive,
        idle,

        pendingHmdStartStreamRequest,
        failedHmdStartStreamRequest,
    };

    eMenuState m_menuState;
    class ClientHMDView *m_hmdView;
    int m_lastHmdSeqNum;
};

#endif // APP_STAGE_COMPUTE_TRACKER_POSES_H
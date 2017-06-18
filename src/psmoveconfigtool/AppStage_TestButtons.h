#ifndef APP_STAGE_TEST_BUTTONS_H
#define APP_STAGE_TEST_BUTTONS_H

//-- includes -----
#include "AppStage.h"

//-- definitions -----
class AppStage_TestButtons : public AppStage
{
public:
    AppStage_TestButtons(class App *app);
    virtual ~AppStage_TestButtons();

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

protected:
    void request_exit_to_app_stage(const char *app_stage_name);
    static void handle_acquire_controller(
        const PSMResponseMessage *response,
        void *userdata);
    static void handle_release_controller(
        const PSMResponseMessage *response,
        void *userdata);

private:
    enum eMenuState
    {
        inactive,

        waitingForStreamStartResponse,
        failedStreamStart,
        idle,
    };

    eMenuState m_menuState;
    PSMController *m_controllerView;
};

#endif // APP_STAGE_TEST_BUTTONS_H
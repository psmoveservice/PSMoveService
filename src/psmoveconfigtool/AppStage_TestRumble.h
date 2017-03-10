#ifndef APP_STAGE_TEST_RUMBLE_H
#define APP_STAGE_TEST_RUMBLE_H

//-- includes -----
#include "AppStage.h"

//-- definitions -----
class AppStage_TestRumble : public AppStage
{
public:
    AppStage_TestRumble(class App *app);
    virtual ~AppStage_TestRumble();

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
    float get_left_trigger() const;
	float get_right_trigger() const;
    float get_big_rumble_amount() const;
	float get_small_rumble_amount() const;
    void set_rumble_amounts(float big_rumble, float small_rumble);

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

#endif // APP_STAGE_TEST_RUMBLE_H
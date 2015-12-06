#ifndef APP_STAGE_INTRO_SCREEN_H
#define APP_STAGE_INTRO_SCREEN_H

//-- includes -----
#include "AppStage.h"

//-- definitions -----
class AppStage_IntroScreen : public AppStage
{
public:
    AppStage_IntroScreen(class App *app);

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    virtual void onKeyDown(int keyCode) override;

    static const char *APP_STAGE_NAME;
};

#endif // APP_STAGE_INTRO_SCREEN_H
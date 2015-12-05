#ifndef APP_STAGE_SELECT_CONTROLLER_H
#define APP_STAGE_SELECT_CONTROLLER_H

//-- includes -----
#include "AppStage.h"

//-- definitions -----
class AppStage_SelectController : public AppStage
{
public:
    AppStage_SelectController(class App *app);

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    virtual void onKeyDown(int keyCode) override;

    static const char *APP_STAGE_NAME;
};

#endif // APP_STAGE_SELECT_CONTROLLER_H
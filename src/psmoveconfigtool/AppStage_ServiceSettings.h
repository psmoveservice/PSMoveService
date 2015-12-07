#ifndef APP_STAGE_SERVICE_SETTINGS_H
#define APP_STAGE_SERVICE_SETTINGS_H

//-- includes -----
#include "AppStage.h"

//-- definitions -----
class AppStage_ServiceSettings : public AppStage
{
public:
    AppStage_ServiceSettings(class App *app);

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;
};

#endif // APP_STAGE_SERVICE_SETTINGS_H
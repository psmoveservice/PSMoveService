#ifndef APP_STAGE_CAMERA_SETTINGS_H
#define APP_STAGE_CAMERA_SETTINGS_H

//-- includes -----
#include "AppStage.h"

//-- definitions -----
class AppStage_CameraSettings : public AppStage
{
public:
    AppStage_CameraSettings(class App *app);

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;
};

#endif // APP_STAGE_CAMERA_SETTINGS_H
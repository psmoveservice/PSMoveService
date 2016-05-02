#ifndef APP_STAGE_HMD_SETTINGS_H
#define APP_STAGE_HMD_SETTINGS_H

//-- includes -----
#include "AppStage.h"

//-- definitions -----
class AppStage_HMDSettings : public AppStage
{
public:

    AppStage_HMDSettings(class App *app);
    virtual ~AppStage_HMDSettings();

    const struct OpenVRHmdInfo *getSelectedHmdInfo() const;

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

protected:
    void request_hmd_list();

private:
    enum eHmdMenuState
    {
        inactive,
        idle,

        failedHmdListRequest,
    };
    eHmdMenuState m_menuState;

    struct OpenVRHmdInfo *m_hmdInfos;
    int m_hmdListCount;

    int m_selectedHmdIndex;
};

#endif // APP_STAGE_HMD_SETTINGS_H
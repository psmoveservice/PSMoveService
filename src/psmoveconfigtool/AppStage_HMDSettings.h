#ifndef APP_STAGE_HMD_SETTINGS_H
#define APP_STAGE_HMD_SETTINGS_H

//-- includes -----
#include "AppStage.h"

//-- definitions -----
class AppStage_HMDSettings : public AppStage
{
public:
    enum eHMDType
    {
        OculusDK2
    };

    struct HMDInfo
    {
        int HmdID;
        eHMDType HmdType;
        std::string DevicePath;
    };


    AppStage_HMDSettings(class App *app);

    inline const HMDInfo *getSelectedHmdInfo() const
    {
        return
            (m_selectedHmdIndex != -1)
            ? &m_hmdInfos[m_selectedHmdIndex]
            : nullptr;
    }

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

protected:
    virtual bool onClientAPIEvent(
        ClientPSMoveAPI::eClientPSMoveAPIEvent event,
        ClientPSMoveAPI::t_event_data_handle opaque_event_handle) override;

    void request_hmd_list();
    static void handle_hmd_list_response(
        ClientPSMoveAPI::eClientPSMoveResultCode ResultCode,
        const ClientPSMoveAPI::t_request_id request_id,
        ClientPSMoveAPI::t_response_handle response_handle,
        void *userdata);

private:
    enum eHmdMenuState
    {
        inactive,
        idle,

        pendingHmdListRequest,
        failedHmdListRequest,
    };
    eHmdMenuState m_menuState;

    std::vector<HMDInfo> m_hmdInfos;

    int m_selectedHmdIndex;
};

#endif // APP_STAGE_HMD_SETTINGS_H
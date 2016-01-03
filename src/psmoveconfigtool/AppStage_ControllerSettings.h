#ifndef APP_STAGE_CONTROLLER_SETTINGS_H
#define APP_STAGE_CONTROLLER_SETTINGS_H

//-- includes -----
#include "AppStage.h"
#include "ClientPSMoveAPI.h"

#include <vector>

//-- definitions -----
class AppStage_ControllerSettings : public AppStage
{
public:
    enum eControllerType
    {
        PSMove,
        PSNavi
    };

    enum eConnectionType
    {
        Bluetooth,
        USB
    };

    struct ControllerInfo
    {
        int ControllerID;
        eControllerType ControllerType;
        eConnectionType ConnectionType;
        std::string DevicePath;
        std::string DeviceSerial;
        std::string HostSerial;
    };

    AppStage_ControllerSettings(class App *app);

    inline const ControllerInfo *getSelectedControllerInfo() const
    { 
        return 
            (m_selectedControllerIndex != -1) 
            ? &m_pairedControllerInfos[m_selectedControllerIndex] 
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

    void request_controller_list();
    static void handle_controller_list_response(
        ClientPSMoveAPI::eClientPSMoveResultCode ResultCode, 
        const ClientPSMoveAPI::t_request_id request_id, 
        ClientPSMoveAPI::t_response_handle response_handle, 
        void *userdata);

private:
    enum eControllerMenuState
    {
        inactive,
        idle,

        pendingControllerListRequest,
        failedControllerListRequest,
    };
    eControllerMenuState m_menuState;

    std::vector<ControllerInfo> m_pairedControllerInfos;
    std::vector<ControllerInfo> m_unpairedControllerInfos;

    int m_selectedControllerIndex;
};

#endif // APP_STAGE_SELECT_CONTROLLER_H
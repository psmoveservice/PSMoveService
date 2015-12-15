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
    AppStage_ControllerSettings(class App *app);

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

    void request_controller_unpair(int controllerId);
    static void handle_controller_unpair_start_response(
        ClientPSMoveAPI::eClientPSMoveResultCode ResultCode, 
        const ClientPSMoveAPI::t_request_id request_id, 
        ClientPSMoveAPI::t_response_handle response_handle, 
        void *userdata);
    void handle_controller_unpair_end_event(const PSMoveProtocol::Response *event);

    void request_controller_pair(int controllerId);
    static void handle_controller_pair_start_response(
        ClientPSMoveAPI::eClientPSMoveResultCode ResultCode, 
        const ClientPSMoveAPI::t_request_id request_id, 
        ClientPSMoveAPI::t_response_handle response_handle, 
        void *userdata);
    void handle_controller_pair_end_event(const PSMoveProtocol::Response *event);

    void handle_bluetooth_request_progress_event(const PSMoveProtocol::Response *event);

private:
    enum eControllerMenuState
    {
        inactive,
        idle,
        
        pendingControllerListRequest,
        failedControllerListRequest,
        
        pendingControllerUnpairRequest,
        failedControllerUnpairRequest,

        pendingControllerPairRequest,
        failedControllerPairRequest
    };
    eControllerMenuState m_menuState;

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
    };

    std::vector<ControllerInfo> m_pairedControllerInfos;
    std::vector<ControllerInfo> m_unpairedControllerInfos;

    int m_selectedControllerIndex;

};

#endif // APP_STAGE_SELECT_CONTROLLER_H
#ifndef APP_STAGE_PAIR_CONTROLLER_H
#define APP_STAGE_PAIR_CONTROLLER_H

//-- includes -----
#include "AppStage.h"
#include "ClientPSMoveAPI.h"

#include <vector>

//-- definitions -----
class AppStage_PairController : public AppStage
{
public:
    AppStage_PairController(class App *app);

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

    void request_controller_unpair(int controllerId);
    void request_controller_pair(int controllerId);
    void request_cancel_bluetooth_operation(int controllerID);

protected:
    virtual bool onClientAPIEvent(
        ClientPSMoveAPI::eClientPSMoveAPIEvent event, 
        ClientPSMoveAPI::t_event_data_handle opaque_event_handle) override;

    static void handle_controller_unpair_start_response(
        ClientPSMoveAPI::eClientPSMoveResultCode ResultCode, 
        const ClientPSMoveAPI::t_request_id request_id, 
        ClientPSMoveAPI::t_response_handle response_handle, 
        void *userdata);
    void handle_controller_unpair_end_event(const PSMoveProtocol::Response *event);

    static void handle_controller_pair_start_response(
        ClientPSMoveAPI::eClientPSMoveResultCode ResultCode, 
        const ClientPSMoveAPI::t_request_id request_id, 
        ClientPSMoveAPI::t_response_handle response_handle, 
        void *userdata);
    void handle_controller_pair_end_event(const PSMoveProtocol::Response *event);

    void handle_bluetooth_request_progress_event(const PSMoveProtocol::Response *event);

    static void handle_cancel_bluetooth_operation_response(
        ClientPSMoveAPI::eClientPSMoveResultCode ResultCode, 
        const ClientPSMoveAPI::t_request_id request_id, 
        ClientPSMoveAPI::t_response_handle response_handle, 
        void *userdata);

private:
    enum eControllerMenuState
    {
        inactive,

        pendingControllerUnpairRequest,
        failedControllerUnpairRequest,

        pendingControllerPairRequest,
        failedControllerPairRequest,

        pendingCancelBluetoothRequest,
        failedCancelBluetoothRequest
    };
    eControllerMenuState m_menuState;

    int m_pendingBluetoothOpControllerIndex;
    
    int m_pair_steps_completed;
    int m_pair_steps_total;
};

#endif // APP_STAGE_PAIR_CONTROLLER_H
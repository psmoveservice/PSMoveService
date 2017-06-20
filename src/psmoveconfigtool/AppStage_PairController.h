#ifndef APP_STAGE_PAIR_CONTROLLER_H
#define APP_STAGE_PAIR_CONTROLLER_H

//-- includes -----
#include "AppStage.h"
#include "PSMoveClient_CAPI.h"
#include "PSMoveProtocolInterface.h"

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

    void request_controller_unpair(int controllerId, PSMControllerType controllerType);
    void request_controller_pair(int controllerId, PSMControllerType controllerType);
    void request_cancel_bluetooth_operation(int controllerID);

protected:
    virtual bool onClientAPIEvent(
        PSMEventMessage::eEventType event, 
        PSMEventDataHandle opaque_event_handle) override;

    static void handle_controller_unpair_start_response(
        const PSMResponseMessage *response,
        void *userdata);
    void handle_controller_unpair_end_event(const PSMoveProtocol::Response *event);

    static void handle_controller_pair_start_response(
        const PSMResponseMessage *response,
        void *userdata);
    void handle_controller_pair_end_event(const PSMoveProtocol::Response *event);

    void handle_bluetooth_request_progress_event(const PSMoveProtocol::Response *event);

    static void handle_cancel_bluetooth_operation_response(
        const PSMResponseMessage *response,
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

	int m_controllerID;
	PSMControllerType m_controllerType;
};

#endif // APP_STAGE_PAIR_CONTROLLER_H
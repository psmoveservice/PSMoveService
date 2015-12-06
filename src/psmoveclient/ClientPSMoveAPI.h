#ifndef CLIENT_PSMOVE_API_H
#define CLIENT_PSMOVE_API_H

//-- includes -----
#include "ClientConfig.h"
#include "ClientLog.h"

//-- pre-declarations -----
class ClientControllerView;

//-- interface -----
class CLIENTPSMOVEAPI ClientPSMoveAPI
{
public:
    enum eClientPSMoveAPIEvent
    {
        connectedToService,
        failedToConnectToService,
        disconnectedFromService,
    };

    enum eClientPSMoveResultCode
    {
        _clientPSMoveResultCode_ok,
        _clientPSMoveResultCode_error,
        _clientPSMoveResultCode_canceled
    };

    typedef void(*t_event_callback)(eClientPSMoveAPIEvent event, void *userdata);
    typedef void(*t_response_callback)(eClientPSMoveResultCode ResultCode, void *userdata);

    static bool startup(
        const std::string &host,
        const std::string &port,
        t_event_callback callback,
        void *callback_userdata,
        e_log_severity_level log_level = _log_severity_level_info);
    static void update();
    static void shutdown();

    static ClientControllerView *allocate_controller_view(int PSMoveID);
    static void free_controller_view(ClientControllerView *view);

    static void fetch_controller_list(t_response_callback callback, void *callback_userdata);
    static void start_controller_data_stream(ClientControllerView *view, t_response_callback callback, void *callback_userdata);
    static void stop_controller_data_stream(ClientControllerView *view, t_response_callback callback, void *callback_userdata);
    static void set_controller_rumble(
        ClientControllerView *view, float rumble_amount, t_response_callback callback, void *callback_userdata);
    static void reset_pose(ClientControllerView *view, t_response_callback callback, void *callback_userdata);

private:
    // Not allowed to instantiate
    ClientPSMoveAPI();

    // Singleton private implementation - same lifetime as the ClientPSMoveAPI
    static class ClientPSMoveAPIImpl *m_implementation_ptr;
};

#endif // CLIENT_PSMOVE_API_H

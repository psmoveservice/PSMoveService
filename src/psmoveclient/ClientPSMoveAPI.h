#ifndef CLIENT_PSMOVE_API_H
#define CLIENT_PSMOVE_API_H

//-- includes -----
#include "ClientConfig.h"
#include "ClientLog.h"
#include <functional>
#include <memory>

//-- pre-declarations -----
class ClientControllerView;
typedef std::shared_ptr<ClientControllerView> ClientControllerViewPtr;

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

	typedef std::function<void(eClientPSMoveAPIEvent)> event_callback;
    typedef std::function<void(eClientPSMoveResultCode ResultCode)> response_callback;

	static bool startup(
        const std::string &host, 
        const std::string &port, 
        event_callback callback,
        e_log_severity_level log_level=_log_severity_level_info);
	static void update();
	static void shutdown();

    static ClientControllerViewPtr allocate_controller_view(int PSMoveID);
    static void free_controller_view(ClientControllerViewPtr view);

    static void start_controller_data_stream(ClientControllerViewPtr view, ClientPSMoveAPI::response_callback callback);
    static void stop_controller_data_stream(ClientControllerViewPtr view, ClientPSMoveAPI::response_callback callback);
	static void set_controller_rumble(ClientControllerViewPtr view, float rumble_amount, response_callback callback);
	static void reset_pose(ClientControllerViewPtr view, response_callback callback);

private:
	// Not allowed to instantiate
	ClientPSMoveAPI();

	// Singleton private implementation - same lifetime as the ClientPSMoveAPI
	static class ClientPSMoveAPIImpl *m_implementation_ptr;
};

#endif // CLIENT_PSMOVE_API_H

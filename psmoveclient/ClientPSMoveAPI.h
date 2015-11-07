#ifndef CLIENT_PSMOVE_API_H
#define CLIENT_PSMOVE_API_H

//-- includes -----
#include "DataFrameInterface.h"
#include <boost/function.hpp>

//-- pre-declarations -----
class ClientControllerView;
typedef boost::shared_ptr<ClientControllerView> ClientControllerViewPtr;

//-- interface -----
class ClientPSMoveAPI
{
public:
	enum eClientPSMoveAPIEvent
	{
		connectedToService,
        failedToConnectToService,
		disconnectedFromService,
	};

	typedef boost::function<void(eClientPSMoveAPIEvent)> event_callback;
    typedef boost::function<void(RequestPtr, ResponsePtr)> response_callback;

	static bool startup(const std::string &host, const std::string &port, event_callback callback);
	static void update();
	static void shutdown();

    static ClientControllerViewPtr allocate_controller_view(int PSMoveID);
    static void free_controller_view(ClientControllerViewPtr view);

	static void get_controller_count(response_callback callback);
    static void start_controller_data_stream(ClientControllerViewPtr view, ClientPSMoveAPI::response_callback callback);
    static void stop_controller_data_stream(ClientControllerViewPtr view, ClientPSMoveAPI::response_callback callback);
	static void set_controller_rumble(ClientControllerViewPtr view, float rumble_amount, response_callback callback);
	static void cycle_tracking_color(ClientControllerViewPtr view, float rumble_amount, response_callback callback);
	static void reset_pose(ClientControllerViewPtr view, response_callback callback);

private:
	// Not allowed to instantiate
	ClientPSMoveAPI();

	// Singleton private implementation - same lifetime as the ClientPSMoveAPI
	static class ClientPSMoveAPIImpl *m_implementation_ptr;
};

#endif // CLIENT_PSMOVE_API_H

#ifndef CLIENT_PSMOVE_API_H
#define CLIENT_PSMOVE_API_H

//-- includes -----
#include "DataFrameInterface.h"
#include <boost/function.hpp>

//-- pre-declarations -----
class ClientControllerView;

//-- interface -----
class ClientPSMoveAPI
{
public:
	enum eClientPSMoveAPIEvent
	{
		connectedToService,
		disconnectedFromService,
	};

	typedef boost::function<void(eClientPSMoveAPIEvent)> event_callback;
	typedef boost::function<void(ResponsePtr)> response_callback;

	static bool startup(const std::string &host, const std::string &port, event_callback callback);
	static void update();
	static void shutdown();

	static void get_controller_count(response_callback callback);
	static void acquire_controller_view(int controller_id, ClientControllerView *out_view, response_callback callback);
	static void release_controller_view(ClientControllerView *view, response_callback callback);
	static void set_controller_rumble(ClientControllerView *view, float rumble_amount, response_callback callback);
	static void cycle_tracking_color(ClientControllerView *view, float rumble_amount, response_callback callback);
	static void reset_pose(ClientControllerView *view, response_callback callback);

private:
	// Not allowed to instantiate
	ClientPSMoveAPI();

	// Singleton private implementation - same lifetime as the ClientPSMoveAPI
	static class ClientPSMoveAPIImpl *m_implementation_ptr;
};

#endif // CLIENT_PSMOVE_API_H

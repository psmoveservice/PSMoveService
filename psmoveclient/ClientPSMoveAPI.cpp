//-- includes -----
#include "ClientPSMoveAPI.h"
#include "ClientRequestManager.h"
#include "ClientNetworkManager.h"
#include <iostream>

//-- public interface -----
class ClientPSMoveAPIImpl : public IClientNetworkEventListener
{
public:
	ClientPSMoveAPIImpl(const std::string &host, const std::string &port, ClientPSMoveAPI::event_callback callback)
		: request_manager()
		, network_manager(host, port, &request_manager, this)
		, m_event_callback(callback)
	{
	}

	// -- ClientPSMoveAPI System -----
	bool startup()
	{
		bool success = true;

		// Attempt to connect to the server
		if (success)
		{
			if (!network_manager.startup())
			{
				std::cerr << "Failed to initialize the client network manager" << std::endl;
				success = false;
			}
		}

		return success;
	}

	void update()
	{
		// Process incoming/outgoing networking requests
		network_manager.update();
	}

	void shutdown()
	{
		// Close all active network connections
		network_manager.shutdown();
	}

	// -- ClientPSMoveAPI Requests -----
	void get_controller_count(ClientPSMoveAPI::response_callback callback)
	{

	}

	void acquire_controller_view(int controller_id, ClientControllerView *out_view, ClientPSMoveAPI::response_callback callback)
	{

	}

	void release_controller_view(ClientControllerView *view, ClientPSMoveAPI::response_callback callback)
	{

	}

	void set_controller_rumble(ClientControllerView *view, float rumble_amount, ClientPSMoveAPI::response_callback callback)
	{

	}

	void cycle_tracking_color(ClientControllerView *view, float rumble_amount, ClientPSMoveAPI::response_callback callback)
	{

	}

	void reset_pose(ClientControllerView *view, ClientPSMoveAPI::response_callback callback)
	{

	}

	// IClientNetworkEventListener
	virtual void handle_server_connection_opened() override
	{

	}

	virtual void handle_server_connection_open_failed(const boost::system::error_code& ec) override
	{

	}

	virtual void handle_server_connection_closed() override
	{

	}

	virtual void handle_server_connection_close_failed(const boost::system::error_code& ec) override
	{

	}

	virtual void handle_server_connection_socket_error(const boost::system::error_code& ec) override
	{
	}

private:
	ClientRequestManager request_manager;
	ClientNetworkManager network_manager;
	ClientPSMoveAPI::event_callback m_event_callback;
};

//-- ClientPSMoveAPI -----
class ClientPSMoveAPIImpl *ClientPSMoveAPI::m_implementation_ptr = NULL;

bool ClientPSMoveAPI::startup(
	const std::string &host, 
	const std::string &port, 
	ClientPSMoveAPI::event_callback callback)
{
	bool success= true;

	if (ClientPSMoveAPI::m_implementation_ptr == NULL)
	{
		ClientPSMoveAPI::m_implementation_ptr = new ClientPSMoveAPIImpl(host, port, callback);
		success= ClientPSMoveAPI::m_implementation_ptr->startup();
	}

	return success;
}

void ClientPSMoveAPI::update()
{
	if (ClientPSMoveAPI::m_implementation_ptr != NULL)
	{
		ClientPSMoveAPI::m_implementation_ptr->update();
	}
}

void ClientPSMoveAPI::shutdown()
{
	if (ClientPSMoveAPI::m_implementation_ptr != NULL)
	{
		ClientPSMoveAPI::m_implementation_ptr->shutdown();
		
		delete ClientPSMoveAPI::m_implementation_ptr;
		ClientPSMoveAPI::m_implementation_ptr = NULL;
	}
}

void ClientPSMoveAPI::get_controller_count(
	ClientPSMoveAPI::response_callback callback)
{
	if (ClientPSMoveAPI::m_implementation_ptr != NULL)
	{
		ClientPSMoveAPI::m_implementation_ptr->get_controller_count(callback);
	}
}

void ClientPSMoveAPI::acquire_controller_view(
	int controller_id, 
	ClientControllerView *out_view, 
	ClientPSMoveAPI::response_callback callback)
{
	if (ClientPSMoveAPI::m_implementation_ptr != NULL)
	{
		ClientPSMoveAPI::m_implementation_ptr->acquire_controller_view(controller_id, out_view, callback);
	}
}

void ClientPSMoveAPI::release_controller_view(
	ClientControllerView *view, 
	ClientPSMoveAPI::response_callback callback)
{
	if (ClientPSMoveAPI::m_implementation_ptr != NULL)
	{
		ClientPSMoveAPI::m_implementation_ptr->release_controller_view(view, callback);
	}
}

void ClientPSMoveAPI::set_controller_rumble(
	ClientControllerView *view, 
	float rumble_amount, 
	ClientPSMoveAPI::response_callback callback)
{
	if (ClientPSMoveAPI::m_implementation_ptr != NULL)
	{
		ClientPSMoveAPI::m_implementation_ptr->set_controller_rumble(view, rumble_amount, callback);
	}
}

void ClientPSMoveAPI::cycle_tracking_color(
	ClientControllerView *view, 
	float rumble_amount, 
	ClientPSMoveAPI::response_callback callback)
{
	if (ClientPSMoveAPI::m_implementation_ptr != NULL)
	{
		ClientPSMoveAPI::m_implementation_ptr->cycle_tracking_color(view, rumble_amount, callback);
	}
}

void ClientPSMoveAPI::reset_pose(
	ClientControllerView *view, 
	ClientPSMoveAPI::response_callback callback)
{
	if (ClientPSMoveAPI::m_implementation_ptr != NULL)
	{
		ClientPSMoveAPI::m_implementation_ptr->reset_pose(view, callback);
	}
}
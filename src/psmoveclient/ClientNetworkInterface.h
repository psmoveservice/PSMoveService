#ifndef CLIENT_NETWORK_INTERFACE_H
#define CLIENT_NETWORK_INTERFACE_H

//-- includes -----
#include "PSMoveClient_export.h"
#include <boost/system/error_code.hpp>

//-- interface -----
class PSM_CPP_PRIVATE_CLASS IClientNetworkEventListener
{
public:
	virtual void handle_server_connection_opened() = 0;
	virtual void handle_server_connection_open_failed(const boost::system::error_code& ec) = 0;
	virtual void handle_server_connection_closed() = 0;
	virtual void handle_server_connection_close_failed(const boost::system::error_code& ec) = 0;
	virtual void handle_server_connection_socket_error(const boost::system::error_code& ec) = 0;
};

#endif // CLIENT_NETWORK_INTERFACE_H

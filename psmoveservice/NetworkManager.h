#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <boost/asio.hpp>
#include "RequestHandler.h"

// -Network Manager-
// Maintains TCP/UDP connection state with PSMoveClients.
// Routes requests to the given request handler.
class NetworkManager 
{
public:
    NetworkManager(boost::asio::io_service& io_service, unsigned port, RequestHandler &request_handler);
    virtual ~NetworkManager();

private:
    // Must use the overloaded constructor
    NetworkManager();

    // private implementation - same lifetime as the NetworkManager
    class NetworkManagerImpl *implementation_ptr;
};

#endif  // NETWORK_MANAGER_H


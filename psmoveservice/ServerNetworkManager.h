#ifndef SERVER_NETWORK_MANAGER_H
#define SERVER_NETWORK_MANAGER_H

#include "RequestHandler.h"

// -Server Network Manager-
// Maintains TCP/UDP connection state with PSMoveClients.
// Routes requests to the given request handler.
class ServerNetworkManager 
{
public:
    ServerNetworkManager(unsigned port, RequestHandler &request_handler);
    virtual ~ServerNetworkManager();

    bool startup();
    void update();
    void shutdown();

private:
    // Must use the overloaded constructor
    ServerNetworkManager();

    // private implementation - same lifetime as the NetworkManager
    class ServerNetworkManagerImpl *implementation_ptr;
};

#endif  // SERVER_NETWORK_MANAGER_H


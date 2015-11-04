#ifndef CLIENT_NETWORK_MANAGER_H
#define CLIENT_NETWORK_MANAGER_H

#include "ResponseHandler.h"

// -Server Network Manager-
// Maintains TCP/UDP connection state with PSMoveService.
// Routes requests to the given request handler.
class ClientNetworkManager 
{
public:
    ClientNetworkManager(const std::string &host, const std::string &port, ResponseHandler &responseHandler);
    virtual ~ClientNetworkManager();

    bool startup();
    void send_request(RequestPtr request);
    void update();
    void shutdown();

private:
    // Must use the overloaded constructor
    ClientNetworkManager();

    // private implementation - same lifetime as the ClientNetworkManager
    class ClientNetworkManagerImpl *implementation_ptr;
};

#endif  // CLIENT_NETWORK_MANAGER_H
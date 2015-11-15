#ifndef SERVER_NETWORK_MANAGER_H
#define SERVER_NETWORK_MANAGER_H

//-- includes -----
#include "DataFrameInterface.h"

//-- pre-declarations -----
class ServerRequestHandler;

namespace boost {
    namespace asio {
        class io_service;
    }
}

//-- definitions -----
// -Server Network Manager-
// Maintains TCP/UDP connection state with PSMoveClients.
// Routes requests to the given request handler.
class ServerNetworkManager 
{
public:
    ServerNetworkManager(boost::asio::io_service *io_service, unsigned port, ServerRequestHandler *request_handler);
    virtual ~ServerNetworkManager();

    static ServerNetworkManager *get_instance() { return m_instance; }

    bool startup();
    void update();
    void shutdown();

    void send_notification(int connection_id, ResponsePtr response);
    void send_notification_to_all_clients(ResponsePtr response);
    void send_controller_data_frame(int connection_id, ControllerDataFramePtr data_frame);

private:
    // Must use the overloaded constructor
    ServerNetworkManager();

    // private implementation - same lifetime as the NetworkManager
    class ServerNetworkManagerImpl *implementation_ptr;

    // Singleton instance of the class
    // Assigned in startup, cleared in teardown
    static ServerNetworkManager *m_instance;
};

#endif  // SERVER_NETWORK_MANAGER_H


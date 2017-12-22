#ifndef SERVER_NETWORK_MANAGER_H
#define SERVER_NETWORK_MANAGER_H

//-- includes -----
#include "PSMoveProtocolInterface.h"
#include "PSMoveConfig.h"

//-- pre-declarations -----
class ServerRequestHandler;

namespace boost {
    namespace asio {
        class io_service;
    }
}

//-- definitions -----
class NetworkManagerConfig : public PSMoveConfig
{
public:
    static const int CONFIG_VERSION;

    NetworkManagerConfig(const std::string &fnamebase = "NetworkManagerConfig");

    virtual const boost::property_tree::ptree config2ptree();
    virtual void ptree2config(const boost::property_tree::ptree &pt);

    long version;
	int server_port;
};

// -Server Network Manager-
/// Maintains TCP/UDP connection state with PSMoveClients.
/// Routes requests to the given request handler.
class ServerNetworkManager 
{
public:
    /// Used in PSMoveService::m_network_manager
    /**
     \param io_service Uses default initializer of boost::asio::io_service
     \param request_handler Default ServerRequestHandler(ControllerManager)
     */
    ServerNetworkManager();
    virtual ~ServerNetworkManager();

    static ServerNetworkManager *get_instance() { return m_instance; }

    /// Called first by PSMoveService::startup()
    /**
     Calls ServerNetworkManagerImpl::start_connection_accept()
     */
    bool startup(boost::asio::io_service *io_service, ServerRequestHandler *request_handler);
    
    /// Called last by PSMoveService::update()
    /**
     Calls ServerNetworkManagerImpl::poll()
     */
    void update();
    
    /// Called last by PSMoveService::shutdown()
    /**
     Calls ServerNetworkManagerImpl::close_all_connections()
     */
    void shutdown();

    void send_notification(int connection_id, ResponsePtr response);
    
    void send_notification_to_all_clients(ResponsePtr response);
    
    void send_device_data_frame(int connection_id, DeviceOutputDataFramePtr data_frame);

private:   
	/// Configuration settings used by the network manager
	NetworkManagerConfig m_cfg;

    /// private implementation - same lifetime as the NetworkManager
    class ServerNetworkManagerImpl *implementation_ptr;

    /// Singleton instance of the class
    /// Assigned in startup, cleared in teardown
    static ServerNetworkManager *m_instance;
};

#endif  // SERVER_NETWORK_MANAGER_H


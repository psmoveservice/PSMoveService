#ifndef CLIENT_NETWORK_MANAGER_H
#define CLIENT_NETWORK_MANAGER_H

//-- includes ----
#include "PSMoveClient_export.h"
#include "PSMoveProtocolInterface.h"
#include "ClientNetworkInterface.h"

//-- definitions ------
// -Server Network Manager-
// Maintains TCP/UDP connection state with PSMoveService.
// Routes requests to the given request handler.
class PSM_CPP_PRIVATE_CLASS ClientNetworkManager 
{
public:
    ClientNetworkManager(
        const std::string &host, const std::string &port, 
        IDataFrameListener *dataFrameListener,
        INotificationListener *notificationListener,
        IResponseListener *responseListener,
        IClientNetworkEventListener *netEventListener);
    virtual ~ClientNetworkManager();

    static ClientNetworkManager *get_instance() { return m_instance; }

    bool startup();
    void send_request(RequestPtr request);
    void send_device_data_frame(DeviceInputDataFramePtr data_frame);
    void update();
    void shutdown();

private:
    // Must use the overloaded constructor
    ClientNetworkManager();

    // private implementation - same lifetime as the ClientNetworkManager
    class ClientNetworkManagerImpl *m_implementation_ptr;

    // Singleton instance of the class
    // Assigned in startup, cleared in teardown
    static ClientNetworkManager *m_instance;
};

#endif  // CLIENT_NETWORK_MANAGER_H

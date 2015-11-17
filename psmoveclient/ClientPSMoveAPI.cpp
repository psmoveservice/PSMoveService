//-- includes -----
#include "ClientPSMoveAPI.h"
#include "ClientRequestManager.h"
#include "ClientNetworkManager.h"
#include "ClientControllerView.h"
#include "PSMoveProtocol.pb.h"
#include <iostream>
#include <map>

//-- typedefs -----
typedef std::map<int, ClientControllerViewPtr> t_controller_view_map;
typedef std::map<int, ClientControllerViewPtr>::iterator t_controller_view_map_iterator;
typedef std::pair<int, ClientControllerViewPtr> t_id_controller_view_pair;

//-- internal implementation -----
class ClientPSMoveAPIImpl : 
    public IDataFrameListener,
    public INotificationListener,
    public IClientNetworkEventListener
{
public:
    ClientPSMoveAPIImpl(const std::string &host, const std::string &port, ClientPSMoveAPI::event_callback callback)
        : m_request_manager()
        , m_network_manager(
            host, port, 
            this, // IDataFrameListener
            this, // INotificationListener
            &m_request_manager, // IResonseListener
            this) // IClientNetworkEventListener
        , m_event_callback(callback)
        , m_controller_view_map()
    {
    }

    // -- ClientPSMoveAPI System -----
    bool startup(e_log_severity_level log_level)
    {
        bool success = true;

        log_init(log_level);

        // Attempt to connect to the server
        if (success)
        {
            if (!m_network_manager.startup())
            {
                CLIENT_LOG_ERROR("ClientPSMoveAPI") << "Failed to initialize the client network manager" << std::endl;
                success = false;
            }
        }

        if (success)
        {
            CLIENT_LOG_INFO("ClientPSMoveAPI") << "Successfully initialized ClientPSMoveAPI" << std::endl;
        }

        return success;
    }

    void update()
    {
        // Process incoming/outgoing networking requests
        m_network_manager.update();
    }

    void shutdown()
    {
        // Close all active network connections
        m_network_manager.shutdown();
    }

    // -- ClientPSMoveAPI Requests -----
    ClientControllerViewPtr allocate_controller_view(int PSMoveID)
    {
        ClientControllerViewPtr view;

        // Use the same view if one already exists for the given controller id
        t_controller_view_map_iterator view_entry= m_controller_view_map.find(PSMoveID);
        if (view_entry != m_controller_view_map.end())
        {
            view= view_entry->second;
        }
        else
        {
            // Create a new initialized controller view
            view= ClientControllerViewPtr(new ClientControllerView(PSMoveID));

            // Add it to the map of controller
            m_controller_view_map.insert(t_id_controller_view_pair(PSMoveID, view));
        }

        // Keep track of how many clients are listening to this view
        view->IncListenerCount();        
        
        return view;
    }

    void free_controller_view(ClientControllerViewPtr view)
    {
        t_controller_view_map_iterator view_entry= m_controller_view_map.find(view->GetPSMoveID());
        assert(view_entry != m_controller_view_map.end());

        // Decrease the number of listeners to this view
        view->DecListenerCount();

        // If no one is listening to this controller anymore, free it from the map
        if (view->GetListenerCount() <= 0)
        {
            m_controller_view_map.erase(view_entry);
        }
    }

    void start_controller_data_stream(ClientControllerViewPtr view, ClientPSMoveAPI::response_callback callback)
    {
        CLIENT_LOG_INFO("start_controller_data_stream") << "requesting controller stream start for PSMoveID: " << view->GetPSMoveID() << std::endl;

        // Tell the psmove service that we are acquiring this controller
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_START_PSMOVE_DATA_STREAM);
        request->mutable_request_start_psmove_data_stream()->set_psmove_id(view->GetPSMoveID());

        m_request_manager.send_request(request, callback);
    }

    void stop_controller_data_stream(ClientControllerViewPtr view, ClientPSMoveAPI::response_callback callback)
    {
        CLIENT_LOG_INFO("stop_controller_data_stream") << "requesting controller stream stop for PSMoveID: " << view->GetPSMoveID() << std::endl;

        // Tell the psmove service that we are releasing this controller
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_STOP_PSMOVE_DATA_STREAM);
        request->mutable_request_stop_psmove_data_stream()->set_psmove_id(view->GetPSMoveID());

        m_request_manager.send_request(request, callback);
    }

    void set_controller_rumble(ClientControllerViewPtr view, float rumble_amount, ClientPSMoveAPI::response_callback callback)
    {
        CLIENT_LOG_INFO("set_controller_rumble") << "request set rumble to " << rumble_amount << " for PSMoveID: " << view->GetPSMoveID() << std::endl;

        assert(m_controller_view_map.find(view->GetPSMoveID()) != m_controller_view_map.end());

        // Tell the psmove service to set the rumble controller
        // Internally rumble values are in the range [0, 255]
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_SET_RUMBLE);
        request->mutable_request_rumble()->set_psmove_id(view->GetPSMoveID());
        request->mutable_request_rumble()->set_rumble(static_cast<int>(rumble_amount * 255.f));

        m_request_manager.send_request(request, callback);
    }

    void reset_pose(ClientControllerViewPtr view, ClientPSMoveAPI::response_callback callback)
    {
        CLIENT_LOG_INFO("set_controller_rumble") << "requesting pose reset for PSMoveID: " << view->GetPSMoveID() << std::endl;

        // Tell the psmove service to set the current orientation of the given controller as the identity pose
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_RESET_POSE);
        request->mutable_reset_pose()->set_psmove_id(view->GetPSMoveID());
        
        m_request_manager.send_request(request, callback);
    }

    // IDataFrameListener
    virtual void handle_data_frame(ControllerDataFramePtr data_frame) override
    {
        CLIENT_LOG_TRACE("handle_data_frame") << "received data frame for PSMoveID: " << data_frame->psmove_id() << std::endl;

        t_controller_view_map_iterator view_entry= m_controller_view_map.find(data_frame->psmove_id());

        if (view_entry != m_controller_view_map.end())
        {
            ClientControllerViewPtr view= view_entry->second;

            view->ApplyControllerDataFrame(data_frame.get());
        }
    }

    // INotificationListener
    virtual void handle_notification(ResponsePtr notification) override
    {
        assert(notification->request_id() == -1);

        //###bwalker $TODO: controller connected
        //###bwalker $TODO: controller disconnected
        //###bwalker $TODO: tracker connected
        //###bwalker $TODO: tracker disconnected
        CLIENT_LOG_ERROR("handle_notification") << "Unknown notification type received: " << notification->type() << std::endl;
    }

    // IClientNetworkEventListener
    virtual void handle_server_connection_opened() override
    {
        CLIENT_LOG_INFO("handle_server_connection_opened") << "Connected to service" << std::endl;

        if (m_event_callback)
        {
            m_event_callback(ClientPSMoveAPI::connectedToService);
        }
    }

    virtual void handle_server_connection_open_failed(const boost::system::error_code& ec) override
    {
        CLIENT_LOG_ERROR("handle_server_connection_open_failed") << "Failed to connect to service: " << ec.message() << std::endl;

        if (m_event_callback)
        {
            m_event_callback(ClientPSMoveAPI::failedToConnectToService);
        }
    }

    virtual void handle_server_connection_closed() override
    {
        CLIENT_LOG_INFO("handle_server_connection_closed") << "Disconnected from service" << std::endl;

        if (m_event_callback)
        {
            m_event_callback(ClientPSMoveAPI::disconnectedFromService);
        }
    }

    virtual void handle_server_connection_close_failed(const boost::system::error_code& ec) override
    {
        CLIENT_LOG_ERROR("handle_server_connection_close_failed") << "Error disconnecting from service: " << ec.message() << std::endl;
    }

    virtual void handle_server_connection_socket_error(const boost::system::error_code& ec) override
    {
        CLIENT_LOG_ERROR("handle_server_connection_close_failed") << "Socket error: " << ec.message() << std::endl;
    }

private:
    ClientRequestManager m_request_manager;
    ClientNetworkManager m_network_manager;
    ClientPSMoveAPI::event_callback m_event_callback;
    t_controller_view_map m_controller_view_map;    
};

//-- ClientPSMoveAPI -----
class ClientPSMoveAPIImpl *ClientPSMoveAPI::m_implementation_ptr = NULL;

bool ClientPSMoveAPI::startup(
    const std::string &host, 
    const std::string &port, 
    ClientPSMoveAPI::event_callback callback,
    e_log_severity_level log_level)
{
    bool success= true;

    if (ClientPSMoveAPI::m_implementation_ptr == NULL)
    {
        ClientPSMoveAPI::m_implementation_ptr = new ClientPSMoveAPIImpl(host, port, callback);
        success= ClientPSMoveAPI::m_implementation_ptr->startup(log_level);
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

ClientControllerViewPtr ClientPSMoveAPI::allocate_controller_view(int PSMoveID)
{
    ClientControllerViewPtr view;

    if (ClientPSMoveAPI::m_implementation_ptr != NULL)
    {
        view= ClientPSMoveAPI::m_implementation_ptr->allocate_controller_view(PSMoveID);
    }

    return view;
}

void ClientPSMoveAPI::free_controller_view(ClientControllerViewPtr view)
{
    if (ClientPSMoveAPI::m_implementation_ptr != NULL)
    {
        ClientPSMoveAPI::m_implementation_ptr->free_controller_view(view);
    }
}

void ClientPSMoveAPI::start_controller_data_stream(ClientControllerViewPtr view, ClientPSMoveAPI::response_callback callback)
{
    if (ClientPSMoveAPI::m_implementation_ptr != NULL)
    {
        ClientPSMoveAPI::m_implementation_ptr->start_controller_data_stream(view, callback);
    }
}

void ClientPSMoveAPI::stop_controller_data_stream(ClientControllerViewPtr view, ClientPSMoveAPI::response_callback callback)
{
    if (ClientPSMoveAPI::m_implementation_ptr != NULL)
    {
        ClientPSMoveAPI::m_implementation_ptr->stop_controller_data_stream(view, callback);
    }
}

void ClientPSMoveAPI::set_controller_rumble(
    ClientControllerViewPtr view, 
    float rumble_amount, 
    ClientPSMoveAPI::response_callback callback)
{
    if (ClientPSMoveAPI::m_implementation_ptr != NULL)
    {
        ClientPSMoveAPI::m_implementation_ptr->set_controller_rumble(view, rumble_amount, callback);
    }
}

void ClientPSMoveAPI::reset_pose(
    ClientControllerViewPtr view,
    ClientPSMoveAPI::response_callback callback)
{
    if (ClientPSMoveAPI::m_implementation_ptr != NULL)
    {
        ClientPSMoveAPI::m_implementation_ptr->reset_pose(view, callback);
    }
}
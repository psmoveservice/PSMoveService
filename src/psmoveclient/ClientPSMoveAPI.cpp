//-- includes -----
#include "ClientPSMoveAPI.h"
#include "ClientRequestManager.h"
#include "ClientNetworkManager.h"
#include "ClientControllerView.h"
#include "PSMoveProtocol.pb.h"
#include <iostream>
#include <map>

//-- typedefs -----
typedef std::map<int, ClientControllerView *> t_controller_view_map;
typedef std::map<int, ClientControllerView *>::iterator t_controller_view_map_iterator;
typedef std::pair<int, ClientControllerView *> t_id_controller_view_pair;

//-- internal implementation -----
class ClientPSMoveAPIImpl : 
    public IDataFrameListener,
    public INotificationListener,
    public IClientNetworkEventListener
{
public:
    ClientPSMoveAPIImpl(
        const std::string &host, 
        const std::string &port, 
        ClientPSMoveAPI::t_event_callback callback,
        void *callback_userdata)
        : m_request_manager()
        , m_network_manager(
            host, port, 
            this, // IDataFrameListener
            this, // INotificationListener
            &m_request_manager, // IResonseListener
            this) // IClientNetworkEventListener
        , m_event_callback(callback)
        , m_event_callback_userdata(callback_userdata)
        , m_controller_view_map()
    {
    }

    virtual ~ClientPSMoveAPIImpl()
    {
        // Without this we get a warning for deletion:
        // "Delete called on 'class ClientPSMoveAPIImpl' that has virtual functions but non-virtual destructor"
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
    ClientControllerView * allocate_controller_view(int ControllerID)
    {
        ClientControllerView * view;

        // Use the same view if one already exists for the given controller id
        t_controller_view_map_iterator view_entry= m_controller_view_map.find(ControllerID);
        if (view_entry != m_controller_view_map.end())
        {
            view= view_entry->second;
        }
        else
        {
            // Create a new initialized controller view
            view= new ClientControllerView(ControllerID);

            // Add it to the map of controller
            m_controller_view_map.insert(t_id_controller_view_pair(ControllerID, view));
        }

        // Keep track of how many clients are listening to this view
        view->IncListenerCount();        
        
        return view;
    }

    void free_controller_view(ClientControllerView * view)
    {
        t_controller_view_map_iterator view_entry= m_controller_view_map.find(view->GetControllerID());
        assert(view_entry != m_controller_view_map.end());

        // Decrease the number of listeners to this view
        view->DecListenerCount();

        // If no one is listening to this controller anymore, free it from the map
        if (view->GetListenerCount() <= 0)
        {
            // Free the controller view allocated in allocate_controller_view
            delete view_entry->second;
            view_entry->second= nullptr;

            // Remove the entry from the map
            m_controller_view_map.erase(view_entry);
        }
    }

    ClientPSMoveAPI::t_request_id start_controller_data_stream(
        ClientControllerView * view, ClientPSMoveAPI::t_response_callback callback, void *userdata)
    {
        CLIENT_LOG_INFO("start_controller_data_stream") << "requesting controller stream start for PSMoveID: " << view->GetControllerID() << std::endl;

        // Tell the psmove service that we are acquiring this controller
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_START_CONTROLLER_DATA_STREAM);
        request->mutable_request_start_psmove_data_stream()->set_controller_id(view->GetControllerID());

        m_request_manager.send_request(request, callback, userdata);

        return request->request_id();
    }

    ClientPSMoveAPI::t_request_id stop_controller_data_stream(
        ClientControllerView * view, ClientPSMoveAPI::t_response_callback callback, void *userdata)
    {
        CLIENT_LOG_INFO("stop_controller_data_stream") << "requesting controller stream stop for PSMoveID: " << view->GetControllerID() << std::endl;

        // Tell the psmove service that we are releasing this controller
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_STOP_CONTROLLER_DATA_STREAM);
        request->mutable_request_stop_psmove_data_stream()->set_controller_id(view->GetControllerID());

        m_request_manager.send_request(request, callback, userdata);

        return request->request_id();
    }

    ClientPSMoveAPI::t_request_id set_controller_rumble(
        ClientControllerView * view, float rumble_amount, ClientPSMoveAPI::t_response_callback callback, void *userdata)
    {
        CLIENT_LOG_INFO("set_controller_rumble") << "request set rumble to " << rumble_amount << " for PSMoveID: " << view->GetControllerID() << std::endl;

        assert(m_controller_view_map.find(view->GetControllerID()) != m_controller_view_map.end());

        // Tell the psmove service to set the rumble controller
        // Internally rumble values are in the range [0, 255]
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_SET_RUMBLE);
        request->mutable_request_rumble()->set_controller_id(view->GetControllerID());
        request->mutable_request_rumble()->set_rumble(static_cast<int>(rumble_amount * 255.f));

        m_request_manager.send_request(request, callback, userdata);

        return request->request_id();
    }

    ClientPSMoveAPI::t_request_id set_led_color(
        ClientControllerView *view, 
        unsigned char r, unsigned char g, unsigned b,
        ClientPSMoveAPI::t_response_callback callback, void *callback_userdata)
    {
        CLIENT_LOG_INFO("set_controller_rumble") << "request set color to " << r << "," << g << "," << b << 
            " for PSMoveID: " << view->GetControllerID() << std::endl;

        assert(m_controller_view_map.find(view->GetControllerID()) != m_controller_view_map.end());

        // Tell the psmove service to set the rumble controller
        // Internally rumble values are in the range [0, 255]
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_SET_LED_COLOR);
        request->mutable_set_led_color_request()->set_controller_id(view->GetControllerID());
        request->mutable_set_led_color_request()->set_r(static_cast<int>(r));
        request->mutable_set_led_color_request()->set_g(static_cast<int>(g));
        request->mutable_set_led_color_request()->set_b(static_cast<int>(b));

        m_request_manager.send_request(request, callback, callback_userdata);

        return request->request_id();
    }

    ClientPSMoveAPI::t_request_id reset_pose(ClientControllerView * view, ClientPSMoveAPI::t_response_callback callback, void *userdata)
    {
        CLIENT_LOG_INFO("set_controller_rumble") << "requesting pose reset for PSMoveID: " << view->GetControllerID() << std::endl;

        // Tell the psmove service to set the current orientation of the given controller as the identity pose
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_RESET_POSE);
        request->mutable_reset_pose()->set_controller_id(view->GetControllerID());
        
        m_request_manager.send_request(request, callback, userdata);

        return request->request_id();
    }

    ClientPSMoveAPI::t_request_id send_opaque_request(
        ClientPSMoveAPI::t_request_handle request_handle,
        ClientPSMoveAPI::t_response_callback callback, 
        void *callback_userdata)
    {
        RequestPtr &request= *reinterpret_cast<RequestPtr *>(request_handle);

        m_request_manager.send_request(request, callback, callback_userdata);

        return request->request_id();
    }

    // IDataFrameListener
    virtual void handle_data_frame(ControllerDataFramePtr data_frame) override
    {
        CLIENT_LOG_TRACE("handle_data_frame") << "received data frame for ControllerID: " << data_frame->controller_id() << std::endl;

        t_controller_view_map_iterator view_entry= m_controller_view_map.find(data_frame->controller_id());

        if (view_entry != m_controller_view_map.end())
        {
            ClientControllerView * view= view_entry->second;

            view->ApplyControllerDataFrame(data_frame.get());
        }
    }

    // INotificationListener
    virtual void handle_notification(ResponsePtr notification) override
    {
        assert(notification->request_id() == -1);

        if (m_event_callback)
        {
            ClientPSMoveAPI::eClientPSMoveAPIEvent specificEventType= ClientPSMoveAPI::opaqueServiceEvent;

            // See if we can translate this to an event type a client without protocol access can see
            switch(notification->type())
            {
            case PSMoveProtocol::Response_ResponseType_CONTROLLER_LIST_UPDATED:
                specificEventType= ClientPSMoveAPI::controllerListUpdated;
                break;
            }

            m_event_callback(
                specificEventType,
                static_cast<ClientPSMoveAPI::t_event_data_handle>(notification.get()),
                m_event_callback_userdata);
        }
    }

    // IClientNetworkEventListener
    virtual void handle_server_connection_opened() override
    {
        CLIENT_LOG_INFO("handle_server_connection_opened") << "Connected to service" << std::endl;

        if (m_event_callback)
        {
            m_event_callback(
                ClientPSMoveAPI::connectedToService, 
                static_cast<ClientPSMoveAPI::t_event_data_handle>(nullptr),
                m_event_callback_userdata);
        }
    }

    virtual void handle_server_connection_open_failed(const boost::system::error_code& ec) override
    {
        CLIENT_LOG_ERROR("handle_server_connection_open_failed") << "Failed to connect to service: " << ec.message() << std::endl;

        if (m_event_callback)
        {
            m_event_callback(
                ClientPSMoveAPI::failedToConnectToService, 
                static_cast<ClientPSMoveAPI::t_event_data_handle>(nullptr),
                m_event_callback_userdata);
        }
    }

    virtual void handle_server_connection_closed() override
    {
        CLIENT_LOG_INFO("handle_server_connection_closed") << "Disconnected from service" << std::endl;

        if (m_event_callback)
        {
            m_event_callback(
                ClientPSMoveAPI::disconnectedFromService, 
                static_cast<ClientPSMoveAPI::t_event_data_handle>(nullptr),
                m_event_callback_userdata);
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
    ClientPSMoveAPI::t_event_callback m_event_callback;
    void *m_event_callback_userdata;
    t_controller_view_map m_controller_view_map;
};

//-- ClientPSMoveAPI -----
class ClientPSMoveAPIImpl *ClientPSMoveAPI::m_implementation_ptr = nullptr;

bool ClientPSMoveAPI::startup(
    const std::string &host, 
    const std::string &port, 
    ClientPSMoveAPI::t_event_callback callback,
    void *callback_userdata,
    e_log_severity_level log_level)
{
    bool success= true;

    if (ClientPSMoveAPI::m_implementation_ptr == nullptr)
    {
        ClientPSMoveAPI::m_implementation_ptr = new ClientPSMoveAPIImpl(host, port, callback, callback_userdata);
        success= ClientPSMoveAPI::m_implementation_ptr->startup(log_level);
    }

    return success;
}

bool ClientPSMoveAPI::has_started()
{
    return ClientPSMoveAPI::m_implementation_ptr != nullptr;
}

void ClientPSMoveAPI::update()
{
    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        ClientPSMoveAPI::m_implementation_ptr->update();
    }
}

void ClientPSMoveAPI::shutdown()
{
    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        ClientPSMoveAPI::m_implementation_ptr->shutdown();
        
        delete ClientPSMoveAPI::m_implementation_ptr;

        ClientPSMoveAPI::m_implementation_ptr = nullptr;
    }
}

ClientControllerView * ClientPSMoveAPI::allocate_controller_view(int ControllerID)
{
    ClientControllerView * view;

    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        view= ClientPSMoveAPI::m_implementation_ptr->allocate_controller_view(ControllerID);
    }

    return view;
}

void ClientPSMoveAPI::free_controller_view(ClientControllerView * view)
{
    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        ClientPSMoveAPI::m_implementation_ptr->free_controller_view(view);
    }
}

ClientPSMoveAPI::t_request_id 
ClientPSMoveAPI::start_controller_data_stream(
    ClientControllerView * view, 
    ClientPSMoveAPI::t_response_callback callback, 
    void *callback_userdata)
{
    ClientPSMoveAPI::t_request_id request_id= ClientPSMoveAPI::INVALID_REQUEST_ID;

    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        request_id= ClientPSMoveAPI::m_implementation_ptr->start_controller_data_stream(view, callback, callback_userdata);
    }

    return request_id;
}

ClientPSMoveAPI::t_request_id 
ClientPSMoveAPI::stop_controller_data_stream(
    ClientControllerView * view, 
    ClientPSMoveAPI::t_response_callback callback,
    void *callback_userdata)
{
    ClientPSMoveAPI::t_request_id request_id= ClientPSMoveAPI::INVALID_REQUEST_ID;

    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        request_id= ClientPSMoveAPI::m_implementation_ptr->stop_controller_data_stream(view, callback, callback_userdata);
    }

    return request_id;
}

ClientPSMoveAPI::t_request_id 
ClientPSMoveAPI::set_controller_rumble(
    ClientControllerView * view, 
    float rumble_amount, 
    ClientPSMoveAPI::t_response_callback callback,
    void *callback_userdata)
{
    ClientPSMoveAPI::t_request_id request_id= ClientPSMoveAPI::INVALID_REQUEST_ID;

    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        request_id= ClientPSMoveAPI::m_implementation_ptr->set_controller_rumble(view, rumble_amount, callback, callback_userdata);
    }

    return request_id;
}

ClientPSMoveAPI::t_request_id 
ClientPSMoveAPI::set_led_color(
    ClientControllerView *view, 
    unsigned char r, unsigned char g, unsigned b,
    t_response_callback callback, void *callback_userdata)
{
    ClientPSMoveAPI::t_request_id request_id= ClientPSMoveAPI::INVALID_REQUEST_ID;

    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        request_id= ClientPSMoveAPI::m_implementation_ptr->set_led_color(view, r, g, b, callback, callback_userdata);
    }

    return request_id;
}

ClientPSMoveAPI::t_request_id 
ClientPSMoveAPI::reset_pose(
    ClientControllerView * view,
    ClientPSMoveAPI::t_response_callback callback,
    void *callback_userdata)
{
    ClientPSMoveAPI::t_request_id request_id= ClientPSMoveAPI::INVALID_REQUEST_ID;

    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        request_id= ClientPSMoveAPI::m_implementation_ptr->reset_pose(view, callback, callback_userdata);
    }

    return request_id;
}

ClientPSMoveAPI::t_request_id 
ClientPSMoveAPI::send_opaque_request(
    ClientPSMoveAPI::t_request_handle request_handle,
    ClientPSMoveAPI::t_response_callback callback, 
    void *callback_userdata)
{
    ClientPSMoveAPI::t_request_id request_id= ClientPSMoveAPI::INVALID_REQUEST_ID;

    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        request_id= ClientPSMoveAPI::m_implementation_ptr->send_opaque_request(
            request_handle, callback, callback_userdata);
    }

    return request_id;
}

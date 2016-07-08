//-- includes -----
#include "ClientPSMoveAPI.h"
#include "ClientRequestManager.h"
#include "ClientNetworkManager.h"
#include "ClientControllerView.h"
#include "PSMoveProtocol.pb.h"
#include <iostream>
#include <map>
#include <deque>

//-- typedefs -----
typedef std::map<int, ClientControllerView *> t_controller_view_map;
typedef std::map<int, ClientControllerView *>::iterator t_controller_view_map_iterator;
typedef std::pair<int, ClientControllerView *> t_id_controller_view_pair;

typedef std::map<int, ClientTrackerView *> t_tracker_view_map;
typedef std::map<int, ClientTrackerView *>::iterator t_tracker_view_map_iterator;
typedef std::pair<int, ClientTrackerView *> t_id_tracker_view_pair;

typedef std::deque<ClientPSMoveAPI::Message> t_message_queue;
typedef std::vector<ResponsePtr> t_event_reference_cache;

//-- internal implementation -----
class ClientPSMoveAPIImpl : 
    public IDataFrameListener,
    public INotificationListener,
    public IClientNetworkEventListener
{
public:
    ClientPSMoveAPIImpl(
        const std::string &host, 
        const std::string &port)
        : m_request_manager(
            this, // IDataFrameListener
            ClientPSMoveAPIImpl::handle_response_message, 
            this) // ClientPSMoveAPIImpl::handle_response_message userdata
        , m_network_manager(
            host, port, 
            this, // IDataFrameListener
            this, // INotificationListener
            &m_request_manager, // IResponseListener
            this) // IClientNetworkEventListener
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
        // Drop an unread messages from the previous call to update
        m_message_queue.clear();

        // Drop all of the message parameters
        // NOTE: std::vector::clear() calls the destructor on each element in the vector
        // This will decrement the last ref count to the parameter data, causing them to get cleaned up.
        m_request_manager.flush_response_cache();
        m_event_reference_cache.clear();

        // Publish modified device state back to the service
        publish();

        // Process incoming/outgoing networking requests
        m_network_manager.update();
    }

    void publish()
    {
        // Publish all of the modified controller state
        for (t_controller_view_map_iterator view_entry = m_controller_view_map.begin();
            view_entry != m_controller_view_map.end();
            ++view_entry)
        {
            ClientControllerView *controllerView= view_entry->second;

            controllerView->Publish();
        }
    }

    bool poll_next_message(ClientPSMoveAPI::Message *message, size_t message_size)
    {
        bool bHasMessage = false;

        if (m_message_queue.size() > 0)
        {
            const ClientPSMoveAPI::Message &first = m_message_queue.front();

            assert(sizeof(ClientPSMoveAPI::Message) == message_size);
            assert(message != nullptr);
            memcpy(message, &first, sizeof(ClientPSMoveAPI::Message));

            m_message_queue.pop_front();

            // NOTE: We intentionally keep the message parameters around in the 
            // m_response_reference_cache and m_event_reference_cache since the
            // messages contain raw void pointers to the parameters, which
            // become invalid after the next call to update.

            bHasMessage = true;
        }

        return bHasMessage;
    }

    void shutdown()
    {
        // Close all active network connections
        m_network_manager.shutdown();

        // Drop an unread messages from the previous call to update
        m_message_queue.clear();

        // Drop all of the message parameters
        // NOTE: std::vector::clear() calls the destructor on each element in the vector
        // This will decrement the last ref count to the parameter data, causing them to get cleaned up.
        m_request_manager.flush_response_cache();
        m_event_reference_cache.clear();

        // No more pending requests
        m_pending_request_map.clear();
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

        // Increment the number of objects that are registered to listen in on this controller on this client
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
//            stop_controller_data_stream(view_entry->second);
            // Free the controller view allocated in allocate_controller_view
            delete view_entry->second;
            view_entry->second= nullptr;

            // Remove the entry from the map
            m_controller_view_map.erase(view_entry);
        }
    }
    
    ClientControllerView* get_controller_view(int controller_id)
    {
        t_controller_view_map_iterator view_entry= m_controller_view_map.find(controller_id);
        assert(view_entry != m_controller_view_map.end());
        return view_entry->second;
    }

    ClientPSMoveAPI::t_request_id get_controller_list()
    {
        CLIENT_LOG_INFO("get_controller_list") << "requesting controller list" << std::endl;

        // Tell the psmove service that we want a list of all connected controllers
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_GET_CONTROLLER_LIST);

        // Don't include controllers connected via USB for normal controller list requests
        request->mutable_request_get_controller_list()->set_include_usb_controllers(false);

        m_request_manager.send_request(request);

        return request->request_id();
    }

    ClientPSMoveAPI::t_request_id start_controller_data_stream(ClientControllerView * view, unsigned int flags)
    {
        CLIENT_LOG_INFO("start_controller_data_stream") << "requesting controller stream start for ControllerID: " << view->GetControllerID() << std::endl;

        // Tell the psmove service that we are acquiring this controller
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_START_CONTROLLER_DATA_STREAM);
        request->mutable_request_start_psmove_data_stream()->set_controller_id(view->GetControllerID());

        if ((flags & ClientPSMoveAPI::includePositionData) > 0)
        {
            request->mutable_request_start_psmove_data_stream()->set_include_position_data(true);
        }

        if ((flags & ClientPSMoveAPI::includeRawSensorData) > 0)
        {
            request->mutable_request_start_psmove_data_stream()->set_include_raw_sensor_data(true);
        }

        if ((flags & ClientPSMoveAPI::includeRawTrackerData) > 0)
        {
            request->mutable_request_start_psmove_data_stream()->set_include_raw_tracker_data(true);
        }

        if ((flags & ClientPSMoveAPI::includePhysicsData) > 0)
        {
            request->mutable_request_start_psmove_data_stream()->set_include_physics_data(true);
        }

        m_request_manager.send_request(request);

        return request->request_id();
    }

    ClientPSMoveAPI::t_request_id stop_controller_data_stream(ClientControllerView * view)
    {
        CLIENT_LOG_INFO("stop_controller_data_stream") << "requesting controller stream stop for ControllerID: " << view->GetControllerID() << std::endl;

        // Tell the psmove service that we are releasing this controller
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_STOP_CONTROLLER_DATA_STREAM);
        request->mutable_request_stop_psmove_data_stream()->set_controller_id(view->GetControllerID());

        m_request_manager.send_request(request);

        return request->request_id();
    }

    ClientPSMoveAPI::t_request_id set_led_tracking_color(
        ClientControllerView *view,
        PSMoveTrackingColorType tracking_color)
    {
        CLIENT_LOG_INFO("set_controller_rumble") << "request set tracking color to " << tracking_color <<
            " for PSMoveID: " << view->GetControllerID() << std::endl;

        assert(m_controller_view_map.find(view->GetControllerID()) != m_controller_view_map.end());

        // Tell the psmove service to set the led color by tracking preset
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_SET_LED_TRACKING_COLOR);
        request->mutable_set_led_tracking_color_request()->set_controller_id(view->GetControllerID());
        request->mutable_set_led_tracking_color_request()->set_color_type(
            static_cast<PSMoveProtocol::TrackingColorType>(tracking_color));

        m_request_manager.send_request(request);

        return request->request_id();
    }

    ClientPSMoveAPI::t_request_id reset_pose(ClientControllerView * view)
    {
        CLIENT_LOG_INFO("set_controller_rumble") << "requesting pose reset for PSMoveID: " << view->GetControllerID() << std::endl;

        // Tell the psmove service to set the current orientation of the given controller as the identity pose
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_RESET_POSE);
        request->mutable_reset_pose()->set_controller_id(view->GetControllerID());
        
        m_request_manager.send_request(request);

        return request->request_id();
    }

    ClientTrackerView *allocate_tracker_view(const ClientTrackerInfo &trackerInfo)
    {
        ClientTrackerView * view;

        // Use the same view if one already exists for the given tracker id
        t_tracker_view_map_iterator view_entry = m_tracker_view_map.find(trackerInfo.tracker_id);
        if (view_entry != m_tracker_view_map.end())
        {
            view = view_entry->second;
        }
        else
        {
            // Create a new initialized tracker view
            view = new ClientTrackerView(trackerInfo);

            // Add it to the map of tracker
            m_tracker_view_map.insert(t_id_tracker_view_pair(trackerInfo.tracker_id, view));
        }

        // Keep track of how many clients are listening to this view
        view->incListenerCount();

        return view;
    }

    void free_tracker_view(ClientTrackerView *view)
    {
        t_tracker_view_map_iterator view_entry = m_tracker_view_map.find(view->getTrackerId());
        assert(view_entry != m_tracker_view_map.end());

        // Decrease the number of listeners to this view
        view->decListenerCount();

        // If no one is listening to this tracker anymore, free it from the map
        if (view->getListenerCount() <= 0)
        {
            // Free the tracker view allocated in allocate_tracker_view
//            stop_tracker_data_stream(view_entry->second);
            delete view_entry->second;
            view_entry->second = nullptr;

            // Remove the entry from the map
            m_tracker_view_map.erase(view_entry);
        }
    }

    ClientPSMoveAPI::t_request_id get_tracker_list()
    {
        CLIENT_LOG_INFO("get_tracker_list") << "requesting tracker list" << std::endl;

        // Tell the psmove service that we want a list of all connected trackers
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_GET_TRACKER_LIST);

        m_request_manager.send_request(request);

        return request->request_id();
    }

    ClientPSMoveAPI::t_request_id start_tracker_data_stream(ClientTrackerView *view)
    {
        CLIENT_LOG_INFO("start_tracker_data_stream") << "requesting tracker stream start for TrackerID: " << view->getTrackerId() << std::endl;

        // Tell the psmove service that we are acquiring this tracker
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_START_TRACKER_DATA_STREAM);
        request->mutable_request_start_tracker_data_stream()->set_tracker_id(view->getTrackerId());

        m_request_manager.send_request(request);

        return request->request_id();
    }

    ClientPSMoveAPI::t_request_id stop_tracker_data_stream(ClientTrackerView *view)
    {
        CLIENT_LOG_INFO("stop_tracker_data_stream") << "requesting tracker stream stop for TrackerID: " << view->getTrackerId() << std::endl;

        // Tell the psmove service that we want to stop streaming data from the tracker
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_STOP_TRACKER_DATA_STREAM);
        request->mutable_request_stop_tracker_data_stream()->set_tracker_id(view->getTrackerId());

        m_request_manager.send_request(request);

        return request->request_id();
    }

    ClientPSMoveAPI::t_request_id get_hmd_tracking_space_settings()
    {
        CLIENT_LOG_INFO("get_hmd_tracking_space_settings") << "requesting hmd tracking space settings: " << std::endl;

        // Tell the psmove service that we want the hmd tracking space settings defined during tracker config
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_GET_HMD_TRACKING_SPACE_SETTINGS);

        m_request_manager.send_request(request);

        return request->request_id();
    }

    ClientPSMoveAPI::t_request_id send_opaque_request(
        ClientPSMoveAPI::t_request_handle request_handle)
    {
        RequestPtr &request= *reinterpret_cast<RequestPtr *>(request_handle);

        m_request_manager.send_request(request);

        return request->request_id();
    }

    // IDataFrameListener
    virtual void handle_data_frame(const PSMoveProtocol::DeviceOutputDataFrame *data_frame) override
    {
        switch (data_frame->device_category())
        {
        case PSMoveProtocol::DeviceOutputDataFrame::CONTROLLER:
            {
                const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket& controller_packet= data_frame->controller_data_packet();

                CLIENT_LOG_TRACE("handle_data_frame") 
                    << "received data frame for ControllerID: " 
                    << controller_packet.controller_id() << std::endl;

                t_controller_view_map_iterator view_entry = m_controller_view_map.find(controller_packet.controller_id());

                if (view_entry != m_controller_view_map.end())
                {
                    ClientControllerView * view = view_entry->second;

                    view->ApplyControllerDataFrame(&controller_packet);
                }
            } break;
        case PSMoveProtocol::DeviceOutputDataFrame::TRACKER:
            {
                const PSMoveProtocol::DeviceOutputDataFrame_TrackerDataPacket& tracker_packet = data_frame->tracker_data_packet();

                CLIENT_LOG_TRACE("handle_data_frame")
                    << "received data frame for TrackerID: "
                    << tracker_packet.tracker_id() << std::endl;

                t_tracker_view_map_iterator view_entry = m_tracker_view_map.find(tracker_packet.tracker_id());

                if (view_entry != m_tracker_view_map.end())
                {
                    ClientTrackerView * view = view_entry->second;

                    view->applyTrackerDataFrame(&tracker_packet);
                }
            } break;
        }
    }

    // INotificationListener
    virtual void handle_notification(ResponsePtr notification) override
    {
        assert(notification->request_id() == -1);

        ClientPSMoveAPI::eEventType specificEventType= ClientPSMoveAPI::opaqueServiceEvent;

        // See if we can translate this to an event type a client without protocol access can see
        switch(notification->type())
        {
        case PSMoveProtocol::Response_ResponseType_CONTROLLER_LIST_UPDATED:
            specificEventType= ClientPSMoveAPI::controllerListUpdated;
            break;
        case PSMoveProtocol::Response_ResponseType_TRACKER_LIST_UPDATED:
            specificEventType = ClientPSMoveAPI::trackerListUpdated;
            break;

        }

        enqueue_event_message(specificEventType, notification);
    }

    // IClientNetworkEventListener
    virtual void handle_server_connection_opened() override
    {
        CLIENT_LOG_INFO("handle_server_connection_opened") << "Connected to service" << std::endl;

        enqueue_event_message(ClientPSMoveAPI::connectedToService, ResponsePtr());
    }

    virtual void handle_server_connection_open_failed(const boost::system::error_code& ec) override
    {
        CLIENT_LOG_ERROR("handle_server_connection_open_failed") << "Failed to connect to service: " << ec.message() << std::endl;

        enqueue_event_message(ClientPSMoveAPI::failedToConnectToService, ResponsePtr());
    }

    virtual void handle_server_connection_closed() override
    {
        CLIENT_LOG_INFO("handle_server_connection_closed") << "Disconnected from service" << std::endl;

        enqueue_event_message(ClientPSMoveAPI::disconnectedFromService, ResponsePtr());
    }

    virtual void handle_server_connection_close_failed(const boost::system::error_code& ec) override
    {
        CLIENT_LOG_ERROR("handle_server_connection_close_failed") << "Error disconnecting from service: " << ec.message() << std::endl;
    }

    virtual void handle_server_connection_socket_error(const boost::system::error_code& ec) override
    {
        CLIENT_LOG_ERROR("handle_server_connection_close_failed") << "Socket error: " << ec.message() << std::endl;
    }

    // Request Manager Callback
    static void handle_response_message(
        const ClientPSMoveAPI::ResponseMessage *response_message,
        void *userdata)
    {
        ClientPSMoveAPIImpl *this_ptr = reinterpret_cast<ClientPSMoveAPIImpl *>(userdata);

        if (response_message->request_id != ClientPSMoveAPI::INVALID_REQUEST_ID)
        {
            // If there is a callback waiting to be called for this request,
            // then go ahead and execute it now.
            if (!this_ptr->execute_callback(response_message))
            {
                // Otherwise go ahead and enqueue a message that can be picked up
                // in poll_next_message() this frame.
                this_ptr->enqueue_response_message(response_message);
            }
        }
    }

    // Message Helpers
    //-----------------
    void enqueue_event_message(
        ClientPSMoveAPI::eEventType event_type,
        ResponsePtr event)
    {
        ClientPSMoveAPI::Message message;

        memset(&message, 0, sizeof(ClientPSMoveAPI::Message));
        message.payload_type = ClientPSMoveAPI::_messagePayloadType_Event;
        message.event_data.event_type= event_type;

        // Maintain a reference to the event until the next update
        if (event)
        {
            // Create a smart pointer to a new copy of the event.
            // If we just add the given event smart pointer to the reference cache
            // we'll be storing a reference to the shared m_packed_response on the client network manager
            // which gets constantly overwritten with new incoming events.
            ResponsePtr eventCopy(new PSMoveProtocol::Response(*event.get()));

            //NOTE: This pointer is only safe until the next update call to update is made
            message.event_data.event_data_handle = static_cast<const void *>(eventCopy.get());

            m_event_reference_cache.push_back(eventCopy);
        }
        else
        {
            message.event_data.event_data_handle = nullptr;
        }

        // Add the message to the message queue
        m_message_queue.push_back(message);
    }

    bool register_callback(
        ClientPSMoveAPI::t_request_id request_id,
        ClientPSMoveAPI::t_response_callback callback,
        void *callback_userdata)
    {
        bool bSuccess = false;

        if (request_id != ClientPSMoveAPI::INVALID_REQUEST_ID)
        {
            PendingRequest pendingRequest;

            assert(m_pending_request_map.find(request_id) == m_pending_request_map.end());
            memset(&pendingRequest, 0, sizeof(PendingRequest));
            pendingRequest.request_id = request_id;
            pendingRequest.response_callback = callback;
            pendingRequest.response_userdata = callback_userdata;

            m_pending_request_map.insert(t_pending_request_map_entry(request_id, pendingRequest));
            bSuccess = true;
        }

        return bSuccess;
    }

    bool execute_callback(
        const ClientPSMoveAPI::ResponseMessage *response_message)
    {
        const ClientPSMoveAPI::t_request_id request_id = response_message->request_id;
        bool bExecutedCallback = false;

        if (request_id != ClientPSMoveAPI::INVALID_REQUEST_ID)
        {
            t_pending_request_map::iterator iter = m_pending_request_map.find(request_id);

            if (iter != m_pending_request_map.end())
            {
                const PendingRequest &pendingRequest = iter->second;

                if (pendingRequest.response_callback != nullptr)
                {
                    pendingRequest.response_callback(
                        response_message,
                        pendingRequest.response_userdata);

                    bExecutedCallback = true;
                }

                m_pending_request_map.erase(iter);
            }
        }

        return bExecutedCallback;
    }

    void enqueue_response_message(
        const ClientPSMoveAPI::ResponseMessage *response_message)
    {
        ClientPSMoveAPI::Message message;

        memset(&message, 0, sizeof(ClientPSMoveAPI::Message));
        message.payload_type = ClientPSMoveAPI::_messagePayloadType_Response;
        message.response_data= *response_message;

        // Add the message to the message queue
        m_message_queue.push_back(message);
    }

    bool cancel_callback(ClientPSMoveAPI::t_request_id request_id)
    {
        bool bSuccess = false;

        if (request_id != ClientPSMoveAPI::INVALID_REQUEST_ID)
        {
            t_pending_request_map::iterator iter= m_pending_request_map.find(request_id);

            if (iter != m_pending_request_map.end())
            {
                m_pending_request_map.erase(iter);
                bSuccess = true;
            }
        }

        return bSuccess;
    }

private:
    //-- Pending requests -----
    ClientRequestManager m_request_manager;
    
    //-- Session Management -----
    ClientNetworkManager m_network_manager;
    
    //-- Controller Views -----
    t_controller_view_map m_controller_view_map;

    //-- Tracker Views -----
    t_tracker_view_map m_tracker_view_map;

    struct PendingRequest
    {
        ClientPSMoveAPI::t_request_id request_id;
        ClientPSMoveAPI::t_response_callback response_callback;
        void *response_userdata;
    };
    typedef std::map<ClientPSMoveAPI::t_request_id, PendingRequest> t_pending_request_map;
    typedef std::pair<ClientPSMoveAPI::t_request_id, PendingRequest> t_pending_request_map_entry;

    t_pending_request_map m_pending_request_map;

    //-- Messages -----
    // Queue of message received from the most recent call to update()
    // This queue will be emptied automatically at the next call to update().
    t_message_queue m_message_queue;

    // These vectors are used solely to keep the ref counted pointers to the 
    // response and event parameter data valid until the next update call.
    // The message queue contains raw void pointers to the response and event data.
    t_event_reference_cache m_event_reference_cache;
};

//-- ClientPSMoveAPI -----
class ClientPSMoveAPIImpl *ClientPSMoveAPI::m_implementation_ptr = nullptr;

bool ClientPSMoveAPI::startup(
    const std::string &host, 
    const std::string &port,
    e_log_severity_level log_level)
{
    bool success= true;

    if (ClientPSMoveAPI::m_implementation_ptr == nullptr)
    {
        ClientPSMoveAPI::m_implementation_ptr = new ClientPSMoveAPIImpl(host, port);
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

bool ClientPSMoveAPI::poll_next_message(ClientPSMoveAPI::Message *message, size_t message_size)
{
    bool bResult = false;

    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        bResult= ClientPSMoveAPI::m_implementation_ptr->poll_next_message(message, message_size);
    }

    return bResult;
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
    ClientControllerView * view = nullptr;

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

ClientControllerView * ClientPSMoveAPI::get_controller_view(int controller_id)
{
    ClientControllerView * view = nullptr;
    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        view = ClientPSMoveAPI::m_implementation_ptr->get_controller_view(controller_id);
    }
    return view;
}

ClientPSMoveAPI::t_request_id 
ClientPSMoveAPI::get_controller_list()
{
    ClientPSMoveAPI::t_request_id request_id = ClientPSMoveAPI::INVALID_REQUEST_ID;

    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        request_id = ClientPSMoveAPI::m_implementation_ptr->get_controller_list();
    }

    return request_id;
}

ClientPSMoveAPI::t_request_id 
ClientPSMoveAPI::start_controller_data_stream(
    ClientControllerView * view, 
    unsigned int flags)
{
    ClientPSMoveAPI::t_request_id request_id= ClientPSMoveAPI::INVALID_REQUEST_ID;

    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        request_id= ClientPSMoveAPI::m_implementation_ptr->start_controller_data_stream(view, flags);
    }

    return request_id;
}

ClientPSMoveAPI::t_request_id 
ClientPSMoveAPI::stop_controller_data_stream(
    ClientControllerView * view)
{
    ClientPSMoveAPI::t_request_id request_id= ClientPSMoveAPI::INVALID_REQUEST_ID;

    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        request_id= ClientPSMoveAPI::m_implementation_ptr->stop_controller_data_stream(view);
    }

    return request_id;
}

ClientPSMoveAPI::t_request_id
ClientPSMoveAPI::set_led_tracking_color(
    ClientControllerView *view,
    PSMoveTrackingColorType tracking_color)
{
    ClientPSMoveAPI::t_request_id request_id = ClientPSMoveAPI::INVALID_REQUEST_ID;

    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        request_id = ClientPSMoveAPI::m_implementation_ptr->set_led_tracking_color(view, tracking_color);
    }

    return request_id;
}

ClientPSMoveAPI::t_request_id 
ClientPSMoveAPI::reset_pose(
    ClientControllerView * view)
{
    ClientPSMoveAPI::t_request_id request_id= ClientPSMoveAPI::INVALID_REQUEST_ID;

    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        request_id= ClientPSMoveAPI::m_implementation_ptr->reset_pose(view);
    }

    return request_id;
}

ClientTrackerView *
ClientPSMoveAPI::allocate_tracker_view(const ClientTrackerInfo &trackerInfo)
{
    ClientTrackerView * view = nullptr;

    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        view = ClientPSMoveAPI::m_implementation_ptr->allocate_tracker_view(trackerInfo);
    }

    return view;
}

void 
ClientPSMoveAPI::free_tracker_view(ClientTrackerView *view)
{
    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        ClientPSMoveAPI::m_implementation_ptr->free_tracker_view(view);
    }
}

ClientPSMoveAPI::t_request_id
ClientPSMoveAPI::get_tracker_list()
{
    ClientPSMoveAPI::t_request_id request_id = ClientPSMoveAPI::INVALID_REQUEST_ID;

    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        request_id = ClientPSMoveAPI::m_implementation_ptr->get_tracker_list();
    }

    return request_id;
}

ClientPSMoveAPI::t_request_id 
ClientPSMoveAPI::start_tracker_data_stream(ClientTrackerView *view)
{
    ClientPSMoveAPI::t_request_id request_id = ClientPSMoveAPI::INVALID_REQUEST_ID;

    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        request_id = ClientPSMoveAPI::m_implementation_ptr->start_tracker_data_stream(view);
    }

    return request_id;
}

ClientPSMoveAPI::t_request_id 
ClientPSMoveAPI::stop_tracker_data_stream(ClientTrackerView *view)
{
    ClientPSMoveAPI::t_request_id request_id = ClientPSMoveAPI::INVALID_REQUEST_ID;

    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        request_id = ClientPSMoveAPI::m_implementation_ptr->stop_tracker_data_stream(view);
    }

    return request_id;
}

ClientPSMoveAPI::t_request_id
ClientPSMoveAPI::get_hmd_tracking_space_settings()
{
    ClientPSMoveAPI::t_request_id request_id = ClientPSMoveAPI::INVALID_REQUEST_ID;

    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        request_id = ClientPSMoveAPI::m_implementation_ptr->get_hmd_tracking_space_settings();
    }

    return request_id;
}

ClientPSMoveAPI::t_request_id 
ClientPSMoveAPI::send_opaque_request(
    ClientPSMoveAPI::t_request_handle request_handle)
{
    ClientPSMoveAPI::t_request_id request_id= ClientPSMoveAPI::INVALID_REQUEST_ID;

    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        request_id= ClientPSMoveAPI::m_implementation_ptr->send_opaque_request(request_handle);
    }

    return request_id;
}

bool ClientPSMoveAPI::register_callback(
    ClientPSMoveAPI::t_request_id request_id,
    ClientPSMoveAPI::t_response_callback callback, 
    void *callback_userdata)
{
    bool bSuccess = false;

    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        bSuccess= ClientPSMoveAPI::m_implementation_ptr->register_callback(request_id, callback, callback_userdata);
    }

    return bSuccess;
}

bool ClientPSMoveAPI::cancel_callback(
    ClientPSMoveAPI::t_request_id request_id)
{
    bool bSuccess = false;

    if (ClientPSMoveAPI::m_implementation_ptr != nullptr)
    {
        bSuccess= ClientPSMoveAPI::m_implementation_ptr->cancel_callback(request_id);
    }

    return bSuccess;
}

bool ClientPSMoveAPI::eat_response(ClientPSMoveAPI::t_request_id request_id)
{
    return ClientPSMoveAPI::register_callback(request_id, ClientPSMoveAPI::null_response_callback, nullptr);
}

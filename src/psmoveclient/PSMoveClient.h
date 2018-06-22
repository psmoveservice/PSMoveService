#ifndef PSMOVE_CLIENT_H
#define PSMOVE_CLIENT_H

//-- includes -----
#include "PSMoveClient_CAPI.h"
#include "PSMoveProtocolInterface.h"
#include "ClientNetworkInterface.h"
#include "ClientLog.h"
#include <deque>
#include <map>
#include <vector>

//-- typedefs -----
typedef std::deque<PSMMessage> t_message_queue;
typedef std::vector<ResponsePtr> t_event_reference_cache;

//-- definitions -----
class PSMoveClient : 
    public IDataFrameListener,
    public INotificationListener,
    public IClientNetworkEventListener
{
public:
    PSMoveClient(
        const std::string &host, 
        const std::string &port);
    virtual ~PSMoveClient();

	// -- State Queries ----
	inline bool getIsConnected() const { return m_bIsConnected; }
	bool pollHasConnectionStatusChanged();
	bool pollHasControllerListChanged();
	bool pollHasTrackerListChanged();
	bool pollHasHMDListChanged();
	bool pollWasSystemButtonPressed();

    // -- ClientPSMoveAPI System -----
    bool startup(e_log_severity_level log_level);
    void update();
	void process_messages();
    bool poll_next_message(PSMMessage *message, size_t message_size);
    void shutdown();

	// -- System Requests ----
    PSMRequestID get_service_version();

    // -- ClientPSMoveAPI Requests -----
    bool allocate_controller_listener(PSMControllerID controller_id);
    void free_controller_listener(PSMControllerID controller_id);   
    PSMController* get_controller_view(PSMControllerID controller_id);
    PSMRequestID get_controller_list();
    PSMRequestID start_controller_data_stream(PSMControllerID controller_id, unsigned int flags);
    PSMRequestID stop_controller_data_stream(PSMControllerID controller_id);
    PSMRequestID set_led_tracking_color(PSMControllerID controller_id, PSMTrackingColorType tracking_color);
    PSMRequestID reset_orientation(PSMControllerID controller_id, const PSMQuatf& q_pose);
    PSMRequestID set_controller_data_stream_tracker_index(PSMControllerID controller_id, PSMTrackerID tracker_id);
	PSMRequestID set_controller_hand(PSMControllerID controller_id, PSMControllerHand controller_hand);

    bool allocate_tracker_listener(const PSMClientTrackerInfo &trackerInfo);
    void free_tracker_listener(PSMTrackerID tracker_id);
    PSMTracker* get_tracker_view(PSMTrackerID tracker_id);
	PSMRequestID get_tracking_space_settings();
    PSMRequestID get_tracker_list();
    PSMRequestID start_tracker_data_stream(PSMTrackerID tracker_id);
    PSMRequestID stop_tracker_data_stream(PSMTrackerID tracker_id);
	bool open_video_stream(PSMTrackerID tracker_id);
	bool poll_video_stream(PSMTrackerID tracker_id);
	void close_video_stream(PSMTrackerID tracker_id);
	const unsigned char *get_video_frame_buffer(PSMTrackerID tracker_id) const;

    bool allocate_hmd_listener(PSMHmdID HmdID);
    void free_hmd_listener(PSMHmdID HmdID);   
	PSMHeadMountedDisplay* get_hmd_view(PSMHmdID tracker_id);
    PSMRequestID get_hmd_list();    
    PSMRequestID start_hmd_data_stream(PSMHmdID hmd_id, unsigned int flags);
    PSMRequestID stop_hmd_data_stream(PSMHmdID hmd_id);
    PSMRequestID set_hmd_data_stream_tracker_index(PSMHmdID hmd_id, PSMTrackerID tracker_id);
    
    PSMRequestID send_opaque_request(PSMRequestHandle request_handle);

    // -- Callback API --
    bool register_callback(PSMRequestID request_id, PSMResponseCallback callback, void *callback_userdata);
    bool cancel_callback(PSMRequestID request_id);
    
protected:
    void publish();

    // IDataFrameListener
    virtual void handle_data_frame(const PSMoveProtocol::DeviceOutputDataFrame *data_frame) override;

    // INotificationListener
    virtual void handle_notification(ResponsePtr notification) override;

    // IClientNetworkEventListener
    virtual void handle_server_connection_opened() override;
    virtual void handle_server_connection_open_failed(const boost::system::error_code& ec) override;
    virtual void handle_server_connection_closed() override;
    virtual void handle_server_connection_close_failed(const boost::system::error_code& ec) override;
    virtual void handle_server_connection_socket_error(const boost::system::error_code& ec) override;

    // Request Manager Callback
    static void handle_response_message(const PSMResponseMessage *response_message, void *userdata);

    // Message Helpers
    //-----------------
	void process_event_message(const PSMEventMessage *event_message);
    void enqueue_event_message(PSMEventMessage::eEventType event_type, ResponsePtr event);
    bool execute_callback(const PSMResponseMessage *response_message);
    void enqueue_response_message(const PSMResponseMessage *response_message);

private:
    //-- Pending requests -----
    class ClientRequestManager *m_request_manager;
    
    //-- Session Management -----
    class ClientNetworkManager *m_network_manager;
    
    //-- Controller Views -----
	PSMController m_controllers[PSMOVESERVICE_MAX_CONTROLLER_COUNT];

    //-- Tracker Views -----
	PSMTracker m_trackers[PSMOVESERVICE_MAX_TRACKER_COUNT];
    
    //-- HMD Views -----
	PSMHeadMountedDisplay m_HMDs[PSMOVESERVICE_MAX_HMD_COUNT];

	bool m_bIsConnected;
	bool m_bHasConnectionStatusChanged;
	bool m_bHasControllerListChanged;
	bool m_bHasTrackerListChanged;
	bool m_bHasHMDListChanged;
	bool m_bWasSystemButtonPressed;

    struct PendingRequest
    {
        PSMRequestID request_id;
        PSMResponseCallback response_callback;
        void *response_userdata;
    };
    typedef std::map<PSMRequestID, PendingRequest> t_pending_request_map;
    typedef std::pair<PSMRequestID, PendingRequest> t_pending_request_map_entry;

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


#endif // PSMOVE_CLIENT_H
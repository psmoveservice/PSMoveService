//-- includes -----
#include "PSMoveClient.h"
#include "ClientRequestManager.h"
#include "ClientNetworkManager.h"
#include "ClientLog.h"
#include "PSMoveProtocol.pb.h"
#include "SharedTrackerState.h"
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <algorithm>
#include <iostream>
#include <thread>
#include <memory>

#ifdef _MSC_VER
	#pragma warning(disable:4996)  // ignore strncpy warning
#endif

//-- typedefs -----
typedef std::deque<PSMMessage> t_message_queue;
typedef std::vector<ResponsePtr> t_event_reference_cache;

// -- macros -----
#define IS_VALID_CONTROLLER_INDEX(x) ((x) >= 0 && (x) < PSMOVESERVICE_MAX_CONTROLLER_COUNT)
#define IS_VALID_TRACKER_INDEX(x) ((x) >= 0 && (x) < PSMOVESERVICE_MAX_TRACKER_COUNT)
#define IS_VALID_HMD_INDEX(x) ((x) >= 0 && (x) < PSMOVESERVICE_MAX_HMD_COUNT)

// -- prototypes -----
static void processPSMoveRecenterAction(PSMController *controller);
static void processDualShock4RecenterAction(PSMController *controller);

static void applyControllerDataFrame(const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket& controller_packet, PSMController *controller);
static void applyPSMoveDataFrame(const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket& controller_packet, PSMPSMove *psmove);
static void applyPSNaviDataFrame(const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket& controller_packet, PSMPSNavi *psnavi);
static void applyDualShock4DataFrame(const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket& controller_packet, PSMDualShock4 *ds4);
static void applyVirtualControllerDataFrame(const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket& controller_packet, PSMVirtualController *virtual_controller);
static void applyPSMButtonState(PSMButtonState &button, unsigned int button_bitmask, unsigned int button_bit);
static void applyTrackerDataFrame(const PSMoveProtocol::DeviceOutputDataFrame_TrackerDataPacket& tracker_packet, PSMTracker *tracker);
static void applyHmdDataFrame(const PSMoveProtocol::DeviceOutputDataFrame_HMDDataPacket& hmd_packet, PSMHeadMountedDisplay *hmd);
static void applyMorpheusDataFrame(const PSMoveProtocol::DeviceOutputDataFrame_HMDDataPacket& hmd_packet, PSMMorpheus *morpheus);
static void applyVirtualHMDDataFrame(const PSMoveProtocol::DeviceOutputDataFrame_HMDDataPacket& hmd_packet, PSMVirtualHMD *virtualHMD);

// -- private definitions -----
class SharedVideoFrameReadOnlyAccessor
{
public:
    SharedVideoFrameReadOnlyAccessor()
        : m_shared_memory_object(nullptr)
        , m_region(nullptr)
        , m_bgr_frame_buffer(nullptr)
        , m_frame_width(0)
        , m_frame_height(0)
        , m_frame_stride(0)
        , m_last_frame_index(0)
    {}

    ~SharedVideoFrameReadOnlyAccessor()
    {
        dispose();
    }

    bool initialize(const char *shared_memory_name)
    {
        bool bSuccess = false;

        try
        {
            CLIENT_LOG_INFO("SharedMemory::initialize()") << "Opening shared memory: " << shared_memory_name;

            // Remember the name of the shared memory
            strncpy(m_shared_memory_name, shared_memory_name, sizeof(m_shared_memory_name)-1);
            m_shared_memory_name[sizeof(m_shared_memory_name) - 1] = '\0';

            // Create the shared memory object
            m_shared_memory_object =
                new boost::interprocess::shared_memory_object(
                boost::interprocess::open_only,
                shared_memory_name,
                boost::interprocess::read_write);

            // Map all of the shared memory for read/write access
            m_region = new boost::interprocess::mapped_region(*m_shared_memory_object, boost::interprocess::read_write);

            bSuccess = true;
        }
        catch (boost::interprocess::interprocess_exception &ex)
        {
            dispose();
            CLIENT_LOG_ERROR("SharedMemory::initialize()") << "Failed to allocated shared memory: " << m_shared_memory_name
                << ", reason: " << ex.what();
        }
        catch (std::exception &ex)
        {
            dispose();
            CLIENT_LOG_ERROR("SharedMemory::initialize()") << "Failed to allocated shared memory: " << m_shared_memory_name
                << ", reason: " << ex.what();
        }

        return bSuccess;
    }

    void dispose()
    {
        if (m_region != nullptr)
        {
            delete m_region;
            m_region = nullptr;
        }

        if (m_shared_memory_object != nullptr)
        {
            delete m_shared_memory_object;
            m_shared_memory_object = nullptr;
        }

        if (m_bgr_frame_buffer != nullptr)
        {
            delete[] m_bgr_frame_buffer;
            m_bgr_frame_buffer = nullptr;
        }
    }

    bool readVideoFrame()
    {
        bool bNewFrame = false;
        SharedVideoFrameHeader *sharedFrameState = getFrameHeader();
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(sharedFrameState->mutex);

        // Make sure the target buffer is big enough to read the video frame into
        size_t buffer_size =
            SharedVideoFrameHeader::computeVideoBufferSize(sharedFrameState->stride, sharedFrameState->height);

        // Make sure the shared memory is the size we expect
        size_t total_shared_mem_size =
            SharedVideoFrameHeader::computeTotalSize(sharedFrameState->stride, sharedFrameState->height);
        assert(m_region->get_size() >= total_shared_mem_size);

        // Re-allocate the buffer if any of the video properties changed
        if (m_frame_width != sharedFrameState->width ||
            m_frame_height != sharedFrameState->height ||
            m_frame_stride != sharedFrameState->stride)
        {
            freeVideoBuffer();

            m_frame_width = sharedFrameState->width;
            m_frame_height = sharedFrameState->height;
            m_frame_stride = sharedFrameState->stride;

            allocateVideoBuffer();
        }

        // Copy over the video frame if the frame index changed
        if (m_last_frame_index != sharedFrameState->frame_index)
        {
            if (buffer_size > 0)
            {
                std::memcpy(m_bgr_frame_buffer, sharedFrameState->getBufferMutable(), buffer_size);
            }

            m_last_frame_index = sharedFrameState->frame_index;

            bNewFrame = true;
        }

        return bNewFrame;
    }

    void allocateVideoBuffer()
    {
        size_t buffer_size = SharedVideoFrameHeader::computeVideoBufferSize(m_frame_stride, m_frame_height);

        if (buffer_size > 0)
        {
            // Allocate the buffer to copy the video frame into
            m_bgr_frame_buffer = new unsigned char[buffer_size];
        }
    }

    void freeVideoBuffer()
    {
        // free the video frame buffer
        if (m_bgr_frame_buffer != nullptr)
        {
            delete[] m_bgr_frame_buffer;
            m_bgr_frame_buffer = 0;
        }
    }

    inline const unsigned char *getVideoFrameBuffer() const { return m_bgr_frame_buffer; }
    inline int getVideoFrameWidth() const { return m_frame_width; }
    inline int getVideoFrameHeight() const { return m_frame_height; }
    inline int getVideoFrameStride() const { return m_frame_stride; }
    inline int getLastVideoFrameIndex() const { return m_last_frame_index; }

protected:
    SharedVideoFrameHeader *getFrameHeader()
    {
        return reinterpret_cast<SharedVideoFrameHeader *>(m_region->get_address());
    }

private:
    char m_shared_memory_name[256];
    boost::interprocess::shared_memory_object *m_shared_memory_object;
    boost::interprocess::mapped_region *m_region;
    unsigned char *m_bgr_frame_buffer;
    int m_frame_width, m_frame_height, m_frame_stride;
    int m_last_frame_index;
};

// -- methods -----
PSMoveClient::PSMoveClient(
    const std::string &host, 
    const std::string &port)
    : m_request_manager(nullptr)  // ClientPSMoveAPIImpl::handle_response_message userdata
    , m_network_manager(nullptr) // IClientNetworkEventListener
	, m_bIsConnected(false)
	, m_bHasConnectionStatusChanged(false)
	, m_bHasControllerListChanged(false)
	, m_bHasTrackerListChanged(false)
	, m_bHasHMDListChanged(false)
{
	m_request_manager=
		new ClientRequestManager(
            this,  // IDataFrameListener
            PSMoveClient::handle_response_message,
            this);  // ClientPSMoveAPIImpl::handle_response_message userdata
    m_network_manager=
		new ClientNetworkManager(
			host, port, 
			this, // IDataFrameListener
			this, // INotificationListener
			m_request_manager, // IResponseListener
			this); // IClientNetworkEventListener
}

PSMoveClient::~PSMoveClient()
{
	delete m_network_manager;
	delete m_request_manager;
}

// -- State Queries ----
bool PSMoveClient::pollHasConnectionStatusChanged()
{ 
	bool bHasConnectionStatusChanged= m_bHasConnectionStatusChanged;

	m_bHasConnectionStatusChanged= false;

	return bHasConnectionStatusChanged; 
}

bool PSMoveClient::pollHasControllerListChanged()
{
	bool bHasControllerListChanged= m_bHasControllerListChanged;

	m_bHasControllerListChanged= false;

	return bHasControllerListChanged;
}

bool PSMoveClient::pollHasTrackerListChanged()
{ 
	bool bHasTrackerListChanged= m_bHasTrackerListChanged;

	m_bHasTrackerListChanged= false;

	return bHasTrackerListChanged; 
}

bool PSMoveClient::pollHasHMDListChanged()
{
	bool bHasHMDListChanged= m_bHasHMDListChanged;

	m_bHasHMDListChanged= false;

	return bHasHMDListChanged; 
}

bool PSMoveClient::pollWasSystemButtonPressed()
{
	bool bWasSystemButtonPressed= m_bWasSystemButtonPressed;

	m_bWasSystemButtonPressed= false;

	return bWasSystemButtonPressed; 
}

// -- ClientPSMoveAPI System -----
bool PSMoveClient::startup(e_log_severity_level log_level)
{
    bool success = true;

    log_init(log_level);

	// Reset status flags
	m_bIsConnected= false;
	m_bHasConnectionStatusChanged= false;
	m_bHasControllerListChanged= false;
	m_bHasTrackerListChanged= false;
	m_bHasHMDListChanged= false;
	m_bWasSystemButtonPressed = false;

    // Attempt to connect to the server
    if (success)
    {
        if (!m_network_manager->startup())
        {
            CLIENT_LOG_ERROR("ClientPSMoveAPI") << "Failed to initialize the client network manager" << std::endl;
            success = false;
        }
    }

	if (success)
	{
        CLIENT_LOG_INFO("ClientPSMoveAPI") << "Successfully initialized ClientPSMoveAPI" << std::endl;

		memset(&m_controllers, 0, sizeof(PSMController)*PSMOVESERVICE_MAX_CONTROLLER_COUNT);
		for (PSMControllerID controller_id= 0; controller_id < PSMOVESERVICE_MAX_CONTROLLER_COUNT; ++controller_id)    
		{
			m_controllers[controller_id].ControllerID= controller_id;
			m_controllers[controller_id].ControllerType= PSMController_None;
		}

		memset(m_trackers, 0, sizeof(PSMTracker)*PSMOVESERVICE_MAX_TRACKER_COUNT);
		for (PSMTrackerID tracker_id= 0; tracker_id < PSMOVESERVICE_MAX_TRACKER_COUNT; ++tracker_id)    
		{
			m_trackers[tracker_id].tracker_info.tracker_id= tracker_id;
			m_trackers[tracker_id].tracker_info.tracker_type= PSMTracker_None;
		}

		memset(m_HMDs, 0, sizeof(PSMHeadMountedDisplay)*PSMOVESERVICE_MAX_HMD_COUNT);
		for (PSMHmdID hmd_id= 0; hmd_id < PSMOVESERVICE_MAX_HMD_COUNT; ++hmd_id)    
		{
			m_HMDs[hmd_id].HmdID= hmd_id;
			m_HMDs[hmd_id].HmdType= PSMHmd_None;
		}
	}

    return success;
}

void PSMoveClient::update()
{
	// If this system button pressed flag wasn't checked last frame, 
	// then drop it so we don't have a stale flag
	m_bWasSystemButtonPressed = false;

    // Drop an unread messages from the previous call to update
    m_message_queue.clear();

    // Drop all of the message parameters
    // NOTE: std::vector::clear() calls the destructor on each element in the vector
    // This will decrement the last ref count to the parameter data, causing them to get cleaned up.
    m_request_manager->flush_response_cache();
    m_event_reference_cache.clear();

    // Publish modified device state back to the service
    publish();

    // Process incoming/outgoing networking requests
    m_network_manager->update();
}

void PSMoveClient::process_messages()
{
    PSMMessage message;
    while(poll_next_message(&message, sizeof(message)))
    {
        switch(message.payload_type)
        {
            case PSMMessage::_messagePayloadType_Event:
                // Only handle events
                process_event_message(&message.event_data);
                break;
            case PSMMessage::_messagePayloadType_Response:
                // Any response that didn't get a callback executed get dropped on the floor
                CLIENT_LOG_INFO("process_messages") << "Dropping response to request id: " << message.response_data.request_id;
                break;
            default:
                assert(0 && "unreachable");
                break;
        }
    }
}

void PSMoveClient::publish()
{
    // Publish all of the modified controller state
	for (PSMControllerID controller_id= 0; controller_id < PSMOVESERVICE_MAX_CONTROLLER_COUNT; ++controller_id)    
	{
		PSMController *Controller= &m_controllers[controller_id];
			
		if (Controller->bValid)
		{
			bool bHasUnpublishedState = false;

			switch (Controller->ControllerType)
			{
			case PSMController_Move:
				bHasUnpublishedState = Controller->ControllerState.PSMoveState.bHasUnpublishedState;
				break;
			case PSMController_Navi:
				bHasUnpublishedState = false;
				break;
			case PSMController_DualShock4:
				bHasUnpublishedState = Controller->ControllerState.PSDS4State.bHasUnpublishedState;
				break;
			case PSMController_Virtual:
				bHasUnpublishedState = false;
				break;
			}

			if (bHasUnpublishedState)
			{
				DeviceInputDataFramePtr data_frame(new PSMoveProtocol::DeviceInputDataFrame);
				data_frame->set_device_category(PSMoveProtocol::DeviceInputDataFrame_DeviceCategory_CONTROLLER);

				auto *controller_data_packet= data_frame->mutable_controller_data_packet();
				controller_data_packet->set_controller_id(Controller->ControllerID);
				controller_data_packet->set_sequence_num(++Controller->InputSequenceNum);

				switch (Controller->ControllerType)
				{
				case PSMController_Move:
					{
						PSMPSMove *psmove_state= &Controller->ControllerState.PSMoveState;
						auto *psmove_packet = controller_data_packet->mutable_psmove_state();

						controller_data_packet->set_controller_type(PSMoveProtocol::PSMOVE);
						psmove_packet->set_led_r(psmove_state->LED_r);
						psmove_packet->set_led_g(psmove_state->LED_g);
						psmove_packet->set_led_b(psmove_state->LED_b);
						psmove_packet->set_rumble_value(psmove_state->Rumble);

						psmove_state->bHasUnpublishedState = false;
					}
					break;
				case PSMController_Navi:
					{
						controller_data_packet->set_controller_type(PSMoveProtocol::PSNAVI);
					}
					break;
				case PSMController_DualShock4:
					{
						PSMDualShock4 *ds4_state= &Controller->ControllerState.PSDS4State;
						auto *ds4_packet = controller_data_packet->mutable_psdualshock4_state();

						controller_data_packet->set_controller_type(PSMoveProtocol::PSDUALSHOCK4);
						ds4_packet->set_led_r(ds4_state->LED_r);
						ds4_packet->set_led_g(ds4_state->LED_g);
						ds4_packet->set_led_b(ds4_state->LED_b);
						ds4_packet->set_big_rumble_value(ds4_state->BigRumble);
						ds4_packet->set_small_rumble_value(ds4_state->SmallRumble);

						ds4_state->bHasUnpublishedState= false;
					}
					break;
				case PSMController_Virtual:
					{
						controller_data_packet->set_controller_type(PSMoveProtocol::VIRTUALCONTROLLER);
					}
					break;
				default:
					assert(0 && "Unhandled controller type");
				}

				// Send the controller data frame over the network
				m_network_manager->send_device_data_frame(data_frame);
			}
		}
	}

    // Send any pending re-center controller actions
	for (PSMControllerID controller_id= 0; controller_id < PSMOVESERVICE_MAX_CONTROLLER_COUNT; ++controller_id)    
	{
		PSMController *Controller= &m_controllers[controller_id];
			
		if (Controller->bValid)
		{
			switch (Controller->ControllerType)
			{
			case PSMController_Move:
				{
					processPSMoveRecenterAction(Controller);
				}
				break;
			case PSMController_Navi:
				break;
			case PSMController_DualShock4:
				{
					processDualShock4RecenterAction(Controller);
				}
				break;
			case PSMController_Virtual:
				break;
			default:
				assert(0 && "Unhandled controller type");
			}
		}
	}
}

static void processPSMoveRecenterAction(PSMController *controller)
{
	PSMPSMove *psmove= &controller->ControllerState.PSMoveState;

	if (psmove->bPoseResetButtonEnabled)
	{
		long long now =
			std::chrono::duration_cast< std::chrono::milliseconds >(
				std::chrono::system_clock::now().time_since_epoch()).count();

		PSMButtonState resetPoseButtonState = psmove->SelectButton;

		switch (resetPoseButtonState)
		{
		case PSMButtonState_PRESSED:
			{
				psmove->ResetPoseButtonPressTime = now;
			} break;
		case PSMButtonState_DOWN:
			{
				if (!psmove->bResetPoseRequestSent)
				{
					const long long k_hold_duration_milli = 250;
					long long pressDurationMilli = now - psmove->ResetPoseButtonPressTime;

					if (pressDurationMilli >= k_hold_duration_milli)
					{
                        PSMRequestID request_id;
						PSM_ResetControllerOrientationAsync(controller->ControllerID, k_psm_quaternion_identity, &request_id);
                        PSM_EatResponse(request_id);

						psmove->bResetPoseRequestSent = true;
					}
				}
			} break;
		case PSMButtonState_RELEASED:
			{
				psmove->bResetPoseRequestSent = false;
			} break;
		}
	}
}

static void processDualShock4RecenterAction(PSMController *controller)
{
	PSMDualShock4 *ds4= &controller->ControllerState.PSDS4State;

	if (ds4->bPoseResetButtonEnabled)
	{
		long long now =
			std::chrono::duration_cast< std::chrono::milliseconds >(
				std::chrono::system_clock::now().time_since_epoch()).count();

		PSMButtonState resetPoseButtonState = ds4->OptionsButton;

		switch (resetPoseButtonState)
		{
		case PSMButtonState_PRESSED:
			{
				ds4->ResetPoseButtonPressTime = now;
			} break;
		case PSMButtonState_DOWN:
			{
				if (!ds4->bResetPoseRequestSent)
				{
					const long long k_hold_duration_milli = 250;
					long long pressDurationMilli = now - ds4->ResetPoseButtonPressTime;

					if (pressDurationMilli >= k_hold_duration_milli)
					{
                        PSMRequestID request_id;
						PSM_ResetControllerOrientationAsync(controller->ControllerID, k_psm_quaternion_identity, &request_id);
                        PSM_EatResponse(request_id);

						ds4->bResetPoseRequestSent = true;
					}
				}
			} break;
		case PSMButtonState_RELEASED:
			{
				ds4->bResetPoseRequestSent = false;
			} break;
		}
	}
}

bool PSMoveClient::poll_next_message(PSMMessage *message, size_t message_size)
{
    bool bHasMessage = false;

    if (m_message_queue.size() > 0)
    {
        const PSMMessage &first = m_message_queue.front();

        assert(sizeof(PSMMessage) == message_size);
        assert(message != nullptr);
        memcpy(message, &first, sizeof(PSMMessage));

        m_message_queue.pop_front();

        // NOTE: We intentionally keep the message parameters around in the 
        // m_response_reference_cache and m_event_reference_cache since the
        // messages contain raw void pointers to the parameters, which
        // become invalid after the next call to update.

        bHasMessage = true;
    }

    return bHasMessage;
}

void PSMoveClient::shutdown()
{
    // Close all active network connections
    m_network_manager->shutdown();

    // Drop an unread messages from the previous call to update
    m_message_queue.clear();

    // Drop all of the message parameters
    // NOTE: std::vector::clear() calls the destructor on each element in the vector
    // This will decrement the last ref count to the parameter data, causing them to get cleaned up.
    m_request_manager->flush_response_cache();
    m_event_reference_cache.clear();

    // No more pending requests
    m_pending_request_map.clear();
}

// -- System Requests ----
PSMRequestID PSMoveClient::get_service_version()
{
    CLIENT_LOG_INFO("get_service_version") << "requesting service version" << std::endl;

    // Tell the psmove service that we want the version string
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_GET_SERVICE_VERSION);

    m_request_manager->send_request(request);

    return request->request_id();
}

// -- ClientPSMoveAPI Requests -----
bool PSMoveClient::allocate_controller_listener(PSMControllerID ControllerID)
{
	bool bSuccess= false;

	if (IS_VALID_CONTROLLER_INDEX(ControllerID))
	{
		PSMController *controller= &m_controllers[ControllerID];

		if (controller->ListenerCount == 0)
		{
			memset(controller, 0, sizeof(PSMController));
			controller->ControllerID= ControllerID;
			controller->ControllerType = PSMController_None;
		}

		++controller->ListenerCount;
		bSuccess= true;
	}

	return bSuccess;
}

void PSMoveClient::free_controller_listener(PSMControllerID ControllerID)
{
	if (IS_VALID_CONTROLLER_INDEX(ControllerID))
	{
		PSMController *controller= &m_controllers[ControllerID];

		assert(controller->ListenerCount > 0);
		--controller->ListenerCount;

		if (controller->ListenerCount <= 0)
		{
			memset(controller, 0, sizeof(PSMController));
			controller->ControllerID= ControllerID;
			controller->ControllerType= PSMController_None;
		}
	}
}
    
PSMController* PSMoveClient::get_controller_view(PSMControllerID controller_id)
{
	return IS_VALID_CONTROLLER_INDEX(controller_id) ? &m_controllers[controller_id] : nullptr;
}

PSMRequestID PSMoveClient::get_controller_list()
{
    CLIENT_LOG_INFO("get_controller_list") << "requesting controller list" << std::endl;

    // Tell the psmove service that we want a list of all connected controllers
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_GET_CONTROLLER_LIST);

    // Include controllers connected via USB
	// We'll filter out the usb controllers we don't care (i.e. non-navi) about on the client
    request->mutable_request_get_controller_list()->set_include_usb_controllers(true);

    m_request_manager->send_request(request);

    return request->request_id();
}

PSMRequestID PSMoveClient::start_controller_data_stream(PSMControllerID controller_id, unsigned int flags)
{
	PSMRequestID requestID= PSM_INVALID_REQUEST_ID;

    CLIENT_LOG_INFO("start_controller_data_stream") << "requesting controller stream start for ControllerID: " << controller_id << std::endl;

	if (IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		// Tell the psmove service that we are acquiring this controller
		RequestPtr request(new PSMoveProtocol::Request());
		request->set_type(PSMoveProtocol::Request_RequestType_START_CONTROLLER_DATA_STREAM);
		request->mutable_request_start_psmove_data_stream()->set_controller_id(controller_id);

		if ((flags & PSMStreamFlags_includePositionData) > 0)
		{
			request->mutable_request_start_psmove_data_stream()->set_include_position_data(true);
		}

		if ((flags & PSMStreamFlags_includeRawSensorData) > 0)
		{
			request->mutable_request_start_psmove_data_stream()->set_include_raw_sensor_data(true);
		}

		if ((flags & PSMStreamFlags_includeCalibratedSensorData) > 0)
		{
			request->mutable_request_start_psmove_data_stream()->set_include_calibrated_sensor_data(true);
		}

		if ((flags & PSMStreamFlags_includeRawTrackerData) > 0)
		{
			request->mutable_request_start_psmove_data_stream()->set_include_raw_tracker_data(true);
		}

		if ((flags & PSMStreamFlags_includePhysicsData) > 0)
		{
			request->mutable_request_start_psmove_data_stream()->set_include_physics_data(true);
		}

		if ((flags & PSMStreamFlags_disableROI) > 0)
		{
			request->mutable_request_start_psmove_data_stream()->set_disable_roi(true);
		}

		m_request_manager->send_request(request);

		requestID= request->request_id();
	}

    return requestID;
}

PSMRequestID PSMoveClient::stop_controller_data_stream(PSMControllerID controller_id)
{
	PSMRequestID requestID= PSM_INVALID_REQUEST_ID;

    CLIENT_LOG_INFO("stop_controller_data_stream") << "requesting controller stream stop for ControllerID: " << controller_id << std::endl;

	if (IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		// Tell the psmove service that we are releasing this controller
		RequestPtr request(new PSMoveProtocol::Request());
		request->set_type(PSMoveProtocol::Request_RequestType_STOP_CONTROLLER_DATA_STREAM);
		request->mutable_request_stop_psmove_data_stream()->set_controller_id(controller_id);

		m_request_manager->send_request(request);

		requestID= request->request_id();
	}

	return requestID;
}

PSMRequestID PSMoveClient::set_led_tracking_color(
    PSMControllerID controller_id,
    PSMTrackingColorType tracking_color)
{
	PSMRequestID requestID= PSM_INVALID_REQUEST_ID;

    CLIENT_LOG_INFO("set_led_tracking_color") << "request set tracking color to " << tracking_color <<
        " for PSMoveID: " << controller_id << std::endl;

	if (IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		// Tell the psmove service to set the led color by tracking preset
		RequestPtr request(new PSMoveProtocol::Request());
		request->set_type(PSMoveProtocol::Request_RequestType_SET_LED_TRACKING_COLOR);
		request->mutable_set_led_tracking_color_request()->set_controller_id(controller_id);
		request->mutable_set_led_tracking_color_request()->set_color_type(
			static_cast<PSMoveProtocol::TrackingColorType>(tracking_color));

		m_request_manager->send_request(request);

		requestID= request->request_id();
	}

	return requestID;
}

PSMRequestID PSMoveClient::reset_orientation(PSMControllerID controller_id, const PSMQuatf& q_pose)
{
	PSMRequestID requestID= PSM_INVALID_REQUEST_ID;

    CLIENT_LOG_INFO("reset_orientation") << "requesting pose reset for PSMoveID: " << controller_id << std::endl;

	if (IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		// Tell the psmove service to set the current orientation of the given controller as the identity pose
		RequestPtr request(new PSMoveProtocol::Request());
		request->set_type(PSMoveProtocol::Request_RequestType_RESET_ORIENTATION);
		request->mutable_reset_orientation()->set_controller_id(controller_id);
		request->mutable_reset_orientation()->mutable_orientation()->set_w(q_pose.w);
		request->mutable_reset_orientation()->mutable_orientation()->set_x(q_pose.x);
		request->mutable_reset_orientation()->mutable_orientation()->set_y(q_pose.y);
		request->mutable_reset_orientation()->mutable_orientation()->set_z(q_pose.z);
        
		m_request_manager->send_request(request);

		requestID= request->request_id();
	}

	return requestID;
}

PSMRequestID PSMoveClient::set_controller_data_stream_tracker_index(PSMControllerID controller_id, PSMTrackerID tracker_id)
{
	PSMRequestID requestID= PSM_INVALID_REQUEST_ID;

    CLIENT_LOG_INFO("set_controller_data_stream_tracker_index") << "set TrackerID: " << tracker_id << " for ControllerID: " << controller_id << std::endl;

	if (IS_VALID_CONTROLLER_INDEX(controller_id) && IS_VALID_TRACKER_INDEX(tracker_id))
	{
		RequestPtr request(new PSMoveProtocol::Request());
		request->set_type(PSMoveProtocol::Request_RequestType_SET_CONTROLLER_DATA_STREAM_TRACKER_INDEX);
		request->mutable_request_set_controller_data_stream_tracker_index()->set_controller_id(controller_id);
        request->mutable_request_set_controller_data_stream_tracker_index()->set_tracker_id(tracker_id);
        
		m_request_manager->send_request(request);

		requestID= request->request_id();
	}

	return requestID;
}

PSMRequestID PSMoveClient::set_controller_hand(PSMControllerID controller_id, PSMControllerHand controller_hand)
{
	PSMRequestID requestID= PSM_INVALID_REQUEST_ID;

    CLIENT_LOG_INFO("set_controller_hand") << "set hand id: " << controller_hand << " for ControllerID: " << controller_id << std::endl;

	if (IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		RequestPtr request(new PSMoveProtocol::Request());
		request->set_type(PSMoveProtocol::Request_RequestType_SET_CONTROLLER_HAND);
		request->mutable_request_set_controller_hand()->set_controller_id(controller_id);
        request->mutable_request_set_controller_hand()->set_controller_hand(
			static_cast<PSMoveProtocol::ControllerHand>(controller_hand));
        
		m_request_manager->send_request(request);

		requestID= request->request_id();
	}

	return requestID;
}

bool PSMoveClient::allocate_tracker_listener(const PSMClientTrackerInfo &trackerInfo)
{
    bool bSuccess= false;

    if (IS_VALID_TRACKER_INDEX(trackerInfo.tracker_id))
    {
        PSMTracker *tracker= &m_trackers[trackerInfo.tracker_id];

		if (tracker->listener_count == 0)
		{
			memset(tracker, 0, sizeof(PSMTracker));
            tracker->tracker_info= trackerInfo;
        }

        ++tracker->listener_count;
        bSuccess= true;
    }
    
    return bSuccess;
}

void PSMoveClient::free_tracker_listener(PSMTrackerID tracker_id)
{
	if (IS_VALID_TRACKER_INDEX(tracker_id))
	{
		PSMTracker *tracker= &m_trackers[tracker_id];

		assert(tracker->listener_count > 0);
		--tracker->listener_count;

		if (tracker->listener_count <= 0)
		{
			memset(tracker, 0, sizeof(PSMTracker));
			tracker->tracker_info.tracker_id= tracker_id;
			tracker->tracker_info.tracker_type= PSMTracker_None;
		}
	}
}

PSMTracker* PSMoveClient::get_tracker_view(PSMTrackerID tracker_id)
{
	return IS_VALID_TRACKER_INDEX(tracker_id) ? &m_trackers[tracker_id] : nullptr;
}

PSMRequestID PSMoveClient::get_tracking_space_settings()
{
	CLIENT_LOG_INFO("get_tracking_space_settings") << "requesting tracking space settings" << std::endl;

	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_GET_TRACKING_SPACE_SETTINGS);

	m_request_manager->send_request(request);

	return request->request_id();
}

PSMRequestID PSMoveClient::get_tracker_list()
{
    CLIENT_LOG_INFO("get_tracker_list") << "requesting tracker list" << std::endl;

    // Tell the psmove service that we want a list of all connected trackers
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_GET_TRACKER_LIST);

    m_request_manager->send_request(request);

    return request->request_id();
}

PSMRequestID PSMoveClient::start_tracker_data_stream(PSMTrackerID tracker_id)
{
    CLIENT_LOG_INFO("start_tracker_data_stream") << "requesting tracker stream start for TrackerID: " << tracker_id << std::endl;

    // Tell the psmove service that we are acquiring this tracker
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_START_TRACKER_DATA_STREAM);
    request->mutable_request_start_tracker_data_stream()->set_tracker_id(tracker_id);

    m_request_manager->send_request(request);

    return request->request_id();
}

PSMRequestID PSMoveClient::stop_tracker_data_stream(PSMTrackerID tracker_id)
{
    CLIENT_LOG_INFO("stop_tracker_data_stream") << "requesting tracker stream stop for TrackerID: " << tracker_id << std::endl;

    // Tell the psmove service that we want to stop streaming data from the tracker
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_STOP_TRACKER_DATA_STREAM);
    request->mutable_request_stop_tracker_data_stream()->set_tracker_id(tracker_id);

    m_request_manager->send_request(request);

    return request->request_id();
}

bool PSMoveClient::open_video_stream(PSMTrackerID tracker_id)
{
    bool bSuccess = false;

	if (IS_VALID_TRACKER_INDEX(tracker_id))
	{
		PSMTracker *tracker= &m_trackers[tracker_id];

		if (tracker->opaque_shared_memory_accesor == nullptr)
		{
			SharedVideoFrameReadOnlyAccessor *shared_memory_accesor = new SharedVideoFrameReadOnlyAccessor();

			if (shared_memory_accesor->initialize(tracker->tracker_info.shared_memory_name))
			{
				static const int k_max_read_attempt_count = 10;
				int read_attempt = 0;

				bSuccess = false;
				while (!bSuccess && read_attempt < k_max_read_attempt_count)
				{
					bSuccess = shared_memory_accesor->readVideoFrame();
					++read_attempt;

					if (!bSuccess)
					{
						std::this_thread::sleep_for(std::chrono::milliseconds(10));
					}
				}

				tracker->opaque_shared_memory_accesor= shared_memory_accesor;
			}
		}
		else
		{
			// Already open
			bSuccess = true;
		}

		if (!bSuccess)
		{
			close_video_stream(tracker_id);
		}
	}

    return bSuccess;
}

bool PSMoveClient::poll_video_stream(PSMTrackerID tracker_id)
{
    bool bNewFrame = false;

	if (IS_VALID_TRACKER_INDEX(tracker_id))
	{
		PSMTracker *tracker= &m_trackers[tracker_id];

		if (tracker->opaque_shared_memory_accesor != nullptr)
		{
			SharedVideoFrameReadOnlyAccessor *shared_memory_accesor = 
				reinterpret_cast<SharedVideoFrameReadOnlyAccessor *>(tracker->opaque_shared_memory_accesor);

			bNewFrame= shared_memory_accesor->readVideoFrame();
		}
	}

    return bNewFrame;
}

void PSMoveClient::close_video_stream(PSMTrackerID tracker_id)
{
	if (IS_VALID_TRACKER_INDEX(tracker_id))
	{
		PSMTracker *tracker= &m_trackers[tracker_id];

		if (tracker->opaque_shared_memory_accesor != nullptr)
		{
			SharedVideoFrameReadOnlyAccessor *shared_memory_accesor = 
				reinterpret_cast<SharedVideoFrameReadOnlyAccessor *>(tracker->opaque_shared_memory_accesor);

			delete shared_memory_accesor;
			tracker->opaque_shared_memory_accesor = nullptr;
		}
	}
}

const unsigned char *PSMoveClient::get_video_frame_buffer(PSMTrackerID tracker_id) const
{
	const unsigned char *buffer= nullptr;

	if (IS_VALID_TRACKER_INDEX(tracker_id))
	{
		const PSMTracker *tracker= &m_trackers[tracker_id];

		if (tracker->opaque_shared_memory_accesor != nullptr)
		{
			SharedVideoFrameReadOnlyAccessor *shared_memory_accesor = 
				reinterpret_cast<SharedVideoFrameReadOnlyAccessor *>(tracker->opaque_shared_memory_accesor);

			buffer= shared_memory_accesor->getVideoFrameBuffer();
		}
	}

	return buffer;
}
    
bool PSMoveClient::allocate_hmd_listener(PSMHmdID hmd_id)
{
    bool bSuccess= false;

    if (IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= &m_HMDs[hmd_id];

        if (hmd->ListenerCount == 0)
        {
			memset(hmd, 0, sizeof(PSMHeadMountedDisplay));
            hmd->HmdID= hmd_id;
            hmd->HmdType = PSMHmd_None;
        }

        ++hmd->ListenerCount;
        bSuccess= true;
    }
    
    return bSuccess;
}

void PSMoveClient::free_hmd_listener(PSMHmdID hmd_id)
{
    if (IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= &m_HMDs[hmd_id];

        assert(hmd->ListenerCount > 0);
        --hmd->ListenerCount;

        if (hmd->ListenerCount <= 0)
        {
            memset(hmd, 0, sizeof(PSMHeadMountedDisplay));
            hmd->HmdID= hmd_id;
            hmd->HmdType= PSMHmd_None;
        }
    }
}

PSMHeadMountedDisplay* PSMoveClient::get_hmd_view(PSMHmdID hmd_id)
{
	return IS_VALID_HMD_INDEX(hmd_id) ? &m_HMDs[hmd_id] : nullptr;
}

PSMRequestID PSMoveClient::get_hmd_list()
{
    CLIENT_LOG_INFO("get_hmd_list") << "requesting hmd list" << std::endl;

    // Tell the psmove service that we want a list of all connected HMDs
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_GET_HMD_LIST);

    m_request_manager->send_request(request);

    return request->request_id();
}    

    
PSMRequestID PSMoveClient::start_hmd_data_stream(
    PSMHmdID hmd_id,
    unsigned int flags)
{
    CLIENT_LOG_INFO("start_hmd_data_stream") << "requesting HMD stream start for HmdID: " << hmd_id << std::endl;

    // Tell the service that we are acquiring this HMD
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_START_HMD_DATA_STREAM);
    request->mutable_request_start_hmd_data_stream()->set_hmd_id(hmd_id);

	if ((flags & PSMStreamFlags_includePositionData) > 0)
	{
		request->mutable_request_start_hmd_data_stream()->set_include_position_data(true);
	}

	if ((flags & PSMStreamFlags_includePhysicsData) > 0)
	{
		request->mutable_request_start_hmd_data_stream()->set_include_physics_data(true);
	}

    if ((flags & PSMStreamFlags_includeRawSensorData) > 0)
    {
        request->mutable_request_start_hmd_data_stream()->set_include_raw_sensor_data(true);
    }

	if ((flags & PSMStreamFlags_includeCalibratedSensorData) > 0)
	{
		request->mutable_request_start_hmd_data_stream()->set_include_calibrated_sensor_data(true);
	}

	if ((flags & PSMStreamFlags_includeRawTrackerData) > 0)
	{
		request->mutable_request_start_hmd_data_stream()->set_include_raw_tracker_data(true);
	}

	if ((flags & PSMStreamFlags_disableROI) > 0)
	{
		request->mutable_request_start_hmd_data_stream()->set_disable_roi(true);
	}

    m_request_manager->send_request(request);

    return request->request_id();
}

PSMRequestID PSMoveClient::stop_hmd_data_stream(
    PSMHmdID hmd_id)
{
    CLIENT_LOG_INFO("stop_hmd_data_stream") << "requesting HMD stream stop for HmdID: " << hmd_id << std::endl;

    // Tell the service that we are releasing this HMD
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_STOP_HMD_DATA_STREAM);
    request->mutable_request_stop_hmd_data_stream()->set_hmd_id(hmd_id);

    m_request_manager->send_request(request);

    return request->request_id();
}

PSMRequestID PSMoveClient::set_hmd_data_stream_tracker_index(PSMHmdID hmd_id, PSMTrackerID tracker_id)
{
    CLIENT_LOG_INFO("set_hmd_data_stream_tracker_index") << "setting TrackerID: " << tracker_id << " for HmdID: " << hmd_id << std::endl;

    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_HMD_DATA_STREAM_TRACKER_INDEX);
    request->mutable_request_set_hmd_data_stream_tracker_index()->set_hmd_id(hmd_id);
    request->mutable_request_set_hmd_data_stream_tracker_index()->set_tracker_id(tracker_id);

    m_request_manager->send_request(request);

    return request->request_id();
}
    
PSMRequestID PSMoveClient::send_opaque_request(
    PSMRequestHandle request_handle)
{
    RequestPtr &request= *reinterpret_cast<RequestPtr *>(request_handle);

    m_request_manager->send_request(request);

    return request->request_id();
}    
    
// IDataFrameListener
void PSMoveClient::handle_data_frame(const PSMoveProtocol::DeviceOutputDataFrame *data_frame)
{
    switch (data_frame->device_category())
    {
    case PSMoveProtocol::DeviceOutputDataFrame::CONTROLLER:
        {
            const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket& controller_packet= data_frame->controller_data_packet();
			const PSMControllerID controller_id= controller_packet.controller_id();

            CLIENT_LOG_TRACE("handle_data_frame") 
                << "received data frame for ControllerID: " 
                << controller_id << std::endl;

			if (IS_VALID_CONTROLLER_INDEX(controller_id))
			{
				PSMController *controller= get_controller_view(controller_id);

				applyControllerDataFrame(controller_packet, controller);
			}
        } break;
    case PSMoveProtocol::DeviceOutputDataFrame::TRACKER:
        {
            const PSMoveProtocol::DeviceOutputDataFrame_TrackerDataPacket& tracker_packet = data_frame->tracker_data_packet();
			const PSMTrackerID tracker_id= tracker_packet.tracker_id();

            CLIENT_LOG_TRACE("handle_data_frame")
                << "received data frame for TrackerID: "
                << tracker_id << std::endl;

			if (IS_VALID_TRACKER_INDEX(tracker_id))
			{
				PSMTracker *tracker= get_tracker_view(tracker_id);

				applyTrackerDataFrame(tracker_packet, tracker);
			}
        } break;
    case PSMoveProtocol::DeviceOutputDataFrame::HMD:
        {
            const PSMoveProtocol::DeviceOutputDataFrame_HMDDataPacket& hmd_packet = data_frame->hmd_data_packet();
			const PSMHmdID hmd_id= hmd_packet.hmd_id();

            CLIENT_LOG_TRACE("handle_data_frame")
                << "received data frame for HmdID: "
                << hmd_packet.hmd_id()
                << ". Ignoring." << std::endl;

			if (IS_VALID_HMD_INDEX(hmd_id))
			{
				PSMHeadMountedDisplay *hmd= get_hmd_view(hmd_id);

				applyHmdDataFrame(hmd_packet, hmd);
			}
        } break;            
    }
}

static void applyControllerDataFrame(
	const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket& controller_packet, 
	PSMController *controller)
{    
	// Ignore old packets
	if (controller_packet.sequence_num() <= controller->OutputSequenceNum)
		return;

    // Set the generic items
    controller->bValid = controller_packet.controller_id() != -1;
    controller->ControllerType = static_cast<PSMControllerType>(controller_packet.controller_type());
    controller->OutputSequenceNum = controller_packet.sequence_num();
    controller->IsConnected = controller_packet.isconnected();

    // Compute the data frame receive window statistics if we have received enough samples
    {
        long long now = 
            std::chrono::duration_cast< std::chrono::milliseconds >(
                std::chrono::system_clock::now().time_since_epoch()).count();
        long long diff= now - controller->DataFrameLastReceivedTime;

        if (diff > 0)
        {
            float seconds= static_cast<float>(diff) / 1000.f;
            float fps= 1.f / seconds;

            controller->DataFrameAverageFPS= (0.9f)*controller->DataFrameAverageFPS + (0.1f)*fps;
        }

        controller->DataFrameLastReceivedTime= now;
    }
   
	// Don't bother updating the rest of the controller state if it's not connected
	if (!controller->IsConnected)
		return;

    switch (controller->ControllerType) 
	{
        case PSMController_Move:
			applyPSMoveDataFrame(controller_packet, &controller->ControllerState.PSMoveState);
            break;
            
        case PSMController_Navi:		
			applyPSNaviDataFrame(controller_packet, &controller->ControllerState.PSNaviState);
            break;

        case PSMController_DualShock4:
			applyDualShock4DataFrame(controller_packet, &controller->ControllerState.PSDS4State);            
            break;

        case PSMController_Virtual:
			applyVirtualControllerDataFrame(controller_packet, &controller->ControllerState.VirtualController);
            break;
        default:
            break;
    }
}

static void applyPSMoveDataFrame(
	const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket& controller_packet,
	PSMPSMove *psmove)
{
	const auto &psmove_packet= controller_packet.psmove_state();

    psmove->bHasValidHardwareCalibration = psmove_packet.validhardwarecalibration();
    psmove->bIsTrackingEnabled = psmove_packet.istrackingenabled();
    psmove->bIsCurrentlyTracking = psmove_packet.iscurrentlytracking();
	psmove->bIsOrientationValid = psmove_packet.isorientationvalid();
	psmove->bIsPositionValid = psmove_packet.ispositionvalid();
            
    psmove->Pose.Orientation.w= psmove_packet.orientation().w();
    psmove->Pose.Orientation.x= psmove_packet.orientation().x();
    psmove->Pose.Orientation.y= psmove_packet.orientation().y();
    psmove->Pose.Orientation.z= psmove_packet.orientation().z();

    psmove->Pose.Position.x= psmove_packet.position_cm().x();
    psmove->Pose.Position.y= psmove_packet.position_cm().y();
    psmove->Pose.Position.z= psmove_packet.position_cm().z();
            
    if (psmove_packet.has_physics_data())
    {
        const auto &raw_physics_data = psmove_packet.physics_data();

        psmove->PhysicsData.LinearVelocityCmPerSec.x = raw_physics_data.velocity_cm_per_sec().i();
        psmove->PhysicsData.LinearVelocityCmPerSec.y = raw_physics_data.velocity_cm_per_sec().j();
        psmove->PhysicsData.LinearVelocityCmPerSec.z = raw_physics_data.velocity_cm_per_sec().k();

        psmove->PhysicsData.LinearAccelerationCmPerSecSqr.x = raw_physics_data.acceleration_cm_per_sec_sqr().i();
        psmove->PhysicsData.LinearAccelerationCmPerSecSqr.y = raw_physics_data.acceleration_cm_per_sec_sqr().j();
        psmove->PhysicsData.LinearAccelerationCmPerSecSqr.z = raw_physics_data.acceleration_cm_per_sec_sqr().k();

        psmove->PhysicsData.AngularVelocityRadPerSec.x = raw_physics_data.angular_velocity_rad_per_sec().i();
        psmove->PhysicsData.AngularVelocityRadPerSec.y = raw_physics_data.angular_velocity_rad_per_sec().j();
        psmove->PhysicsData.AngularVelocityRadPerSec.z = raw_physics_data.angular_velocity_rad_per_sec().k();

        psmove->PhysicsData.AngularAccelerationRadPerSecSqr.x = raw_physics_data.angular_acceleration_rad_per_sec_sqr().i();
        psmove->PhysicsData.AngularAccelerationRadPerSecSqr.y = raw_physics_data.angular_acceleration_rad_per_sec_sqr().j();
        psmove->PhysicsData.AngularAccelerationRadPerSecSqr.z = raw_physics_data.angular_acceleration_rad_per_sec_sqr().k();

		//###HipsterSloth $TODO - pass down the physics data timestamp
		psmove->PhysicsData.TimeInSeconds= -1.0;
    }
    else
    {
        memset(&psmove->PhysicsData, 0, sizeof(PSMPhysicsData));
    }
            
    if (psmove_packet.has_raw_sensor_data())
    {
        const auto &raw_sensor_data = psmove_packet.raw_sensor_data();

        psmove->RawSensorData.Magnetometer.x= raw_sensor_data.magnetometer().i();
        psmove->RawSensorData.Magnetometer.y= raw_sensor_data.magnetometer().j();
        psmove->RawSensorData.Magnetometer.z= raw_sensor_data.magnetometer().k();

        psmove->RawSensorData.Accelerometer.x= raw_sensor_data.accelerometer().i();
        psmove->RawSensorData.Accelerometer.y= raw_sensor_data.accelerometer().j();
        psmove->RawSensorData.Accelerometer.z= raw_sensor_data.accelerometer().k();

        psmove->RawSensorData.Gyroscope.x= raw_sensor_data.gyroscope().i();
        psmove->RawSensorData.Gyroscope.y= raw_sensor_data.gyroscope().j();
        psmove->RawSensorData.Gyroscope.z= raw_sensor_data.gyroscope().k();

		//###HipsterSloth $TODO - pass down the raw sensor data timestamp
		psmove->RawSensorData.TimeInSeconds= -1.0;
    }
    else
    {
		memset(&psmove->RawSensorData, 0, sizeof(PSMPSMoveRawSensorData));
	}

	if (psmove_packet.has_calibrated_sensor_data())
	{
		const auto &calibrated_sensor_data = psmove_packet.calibrated_sensor_data();

		psmove->CalibratedSensorData.Magnetometer.x = calibrated_sensor_data.magnetometer().i();
		psmove->CalibratedSensorData.Magnetometer.y = calibrated_sensor_data.magnetometer().j();
		psmove->CalibratedSensorData.Magnetometer.z = calibrated_sensor_data.magnetometer().k();

		psmove->CalibratedSensorData.Accelerometer.x = calibrated_sensor_data.accelerometer().i();
		psmove->CalibratedSensorData.Accelerometer.y = calibrated_sensor_data.accelerometer().j();
		psmove->CalibratedSensorData.Accelerometer.z = calibrated_sensor_data.accelerometer().k();

		psmove->CalibratedSensorData.Gyroscope.x = calibrated_sensor_data.gyroscope().i();
		psmove->CalibratedSensorData.Gyroscope.y = calibrated_sensor_data.gyroscope().j();
		psmove->CalibratedSensorData.Gyroscope.z = calibrated_sensor_data.gyroscope().k();

		//###HipsterSloth $TODO - pass down the raw sensor data timestamp
		psmove->CalibratedSensorData.TimeInSeconds = -1.0;
	}
	else
	{
		memset(&psmove->RawSensorData, 0, sizeof(PSMPSMoveCalibratedSensorData));
	}

	if (psmove_packet.has_raw_tracker_data())
	{
		const auto &raw_tracker_data = psmove_packet.raw_tracker_data();

		const PSMoveProtocol::Pixel &locationOnTracker = raw_tracker_data.screen_location();
		const PSMoveProtocol::Position &positionOnTracker = raw_tracker_data.relative_position_cm();

		psmove->RawTrackerData.TrackerID = raw_tracker_data.tracker_id();
		psmove->RawTrackerData.ScreenLocation = { locationOnTracker.x(), locationOnTracker.y() };
		psmove->RawTrackerData.RelativePositionCm = { positionOnTracker.x(), positionOnTracker.y(), positionOnTracker.z() };
		psmove->RawTrackerData.RelativeOrientation = *k_psm_quaternion_identity;
		psmove->RawTrackerData.ValidTrackerBitmask = raw_tracker_data.valid_tracker_bitmask();

        if (raw_tracker_data.has_projected_sphere())
		{
			const PSMoveProtocol::Ellipse &protocolEllipse = raw_tracker_data.projected_sphere();
			PSMTrackingProjection &projection = psmove->RawTrackerData.TrackingProjection;

			projection.shape.ellipse.center.x = protocolEllipse.center().x();
			projection.shape.ellipse.center.y = protocolEllipse.center().y();
			projection.shape.ellipse.half_x_extent = protocolEllipse.half_x_extent();
			projection.shape.ellipse.half_y_extent = protocolEllipse.half_y_extent();
			projection.shape.ellipse.angle = protocolEllipse.angle();
			projection.shape_type = PSMTrackingProjection::PSMShape_Ellipse;
		}
        else
		{
			PSMTrackingProjection &projection = psmove->RawTrackerData.TrackingProjection;

			projection.shape_type = PSMTrackingProjection::PSMShape_INVALID_PROJECTION;
		}


		if (raw_tracker_data.has_multicam_position_cm())
		{
			const PSMoveProtocol::Position &multicam_position = raw_tracker_data.multicam_position_cm();

			psmove->RawTrackerData.MulticamPositionCm.x = multicam_position.x();
			psmove->RawTrackerData.MulticamPositionCm.y = multicam_position.y();
			psmove->RawTrackerData.MulticamPositionCm.z = multicam_position.z();
			psmove->RawTrackerData.bMulticamPositionValid = true;
		}

		// No optical orientation from sphere projection
		psmove->RawTrackerData.bMulticamOrientationValid = false;
		psmove->RawTrackerData.MulticamOrientation = *k_psm_quaternion_identity;
	}
	else
	{
		memset(&psmove->RawTrackerData, 0, sizeof(PSMRawTrackerData));
	}

	unsigned int button_bitmask = controller_packet.button_down_bitmask();
	applyPSMButtonState(psmove->TriangleButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_TRIANGLE);
	applyPSMButtonState(psmove->CircleButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_CIRCLE);
	applyPSMButtonState(psmove->CrossButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_CROSS);
	applyPSMButtonState(psmove->SquareButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_SQUARE);
	applyPSMButtonState(psmove->SelectButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_SELECT);
	applyPSMButtonState(psmove->StartButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_START);
	applyPSMButtonState(psmove->PSButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_PS);
	applyPSMButtonState(psmove->MoveButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_MOVE);
	applyPSMButtonState(psmove->TriggerButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_TRIGGER);

	// Trigger value in range [0,255]
	psmove->TriggerValue = static_cast<unsigned char>(psmove_packet.trigger_value());

	// Battery level range [0, 5] - EE charging & EF full
	psmove->BatteryValue = static_cast<PSMBatteryState>(psmove_packet.battery_value());
}

static void applyPSNaviDataFrame(
	const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket& controller_packet,
	PSMPSNavi *psnavi)
{
    const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_PSNaviState &psnavi_packet= controller_packet.psnavi_state();

    unsigned int button_bitmask= controller_packet.button_down_bitmask();
    applyPSMButtonState(psnavi->L1Button, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_L1);
    applyPSMButtonState(psnavi->L2Button, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_L2);
    applyPSMButtonState(psnavi->L3Button, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_L3);
    applyPSMButtonState(psnavi->CircleButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_CIRCLE);
    applyPSMButtonState(psnavi->CrossButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_CROSS);
    applyPSMButtonState(psnavi->PSButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_PS);
    applyPSMButtonState(psnavi->TriggerButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_TRIGGER);
    applyPSMButtonState(psnavi->DPadUpButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_UP);
    applyPSMButtonState(psnavi->DPadRightButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_RIGHT);
    applyPSMButtonState(psnavi->DPadDownButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_DOWN);
    applyPSMButtonState(psnavi->DPadLeftButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_LEFT);

    psnavi->TriggerValue= static_cast<unsigned char>(psnavi_packet.trigger_value());
    psnavi->Stick_XAxis= static_cast<unsigned char>(psnavi_packet.stick_xaxis());
    psnavi->Stick_YAxis= static_cast<unsigned char>(psnavi_packet.stick_yaxis());
}

static void applyDualShock4DataFrame(
	const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket& controller_packet,
	PSMDualShock4 *ds4)
{
	const auto &ds4_packet= controller_packet.psdualshock4_state();

    ds4->bHasValidHardwareCalibration = ds4_packet.validhardwarecalibration();
    ds4->bIsTrackingEnabled = ds4_packet.istrackingenabled();
    ds4->bIsCurrentlyTracking = ds4_packet.iscurrentlytracking();
	ds4->bIsOrientationValid = ds4_packet.isorientationvalid();
	ds4->bIsPositionValid = ds4_packet.ispositionvalid();
            
    ds4->Pose.Orientation.w= ds4_packet.orientation().w();
    ds4->Pose.Orientation.x= ds4_packet.orientation().x();
    ds4->Pose.Orientation.y= ds4_packet.orientation().y();
    ds4->Pose.Orientation.z= ds4_packet.orientation().z();

    ds4->Pose.Position.x= ds4_packet.position_cm().x();
    ds4->Pose.Position.y= ds4_packet.position_cm().y();
    ds4->Pose.Position.z= ds4_packet.position_cm().z();
            
    if (ds4_packet.has_physics_data())
    {
        const auto &raw_physics_data = ds4_packet.physics_data();

        ds4->PhysicsData.LinearVelocityCmPerSec.x = raw_physics_data.velocity_cm_per_sec().i();
        ds4->PhysicsData.LinearVelocityCmPerSec.y = raw_physics_data.velocity_cm_per_sec().j();
        ds4->PhysicsData.LinearVelocityCmPerSec.z = raw_physics_data.velocity_cm_per_sec().k();

        ds4->PhysicsData.LinearAccelerationCmPerSecSqr.x = raw_physics_data.acceleration_cm_per_sec_sqr().i();
        ds4->PhysicsData.LinearAccelerationCmPerSecSqr.y = raw_physics_data.acceleration_cm_per_sec_sqr().j();
        ds4->PhysicsData.LinearAccelerationCmPerSecSqr.z = raw_physics_data.acceleration_cm_per_sec_sqr().k();

        ds4->PhysicsData.AngularVelocityRadPerSec.x = raw_physics_data.angular_velocity_rad_per_sec().i();
        ds4->PhysicsData.AngularVelocityRadPerSec.y = raw_physics_data.angular_velocity_rad_per_sec().j();
        ds4->PhysicsData.AngularVelocityRadPerSec.z = raw_physics_data.angular_velocity_rad_per_sec().k();

        ds4->PhysicsData.AngularAccelerationRadPerSecSqr.x = raw_physics_data.angular_acceleration_rad_per_sec_sqr().i();
        ds4->PhysicsData.AngularAccelerationRadPerSecSqr.y = raw_physics_data.angular_acceleration_rad_per_sec_sqr().j();
        ds4->PhysicsData.AngularAccelerationRadPerSecSqr.z = raw_physics_data.angular_acceleration_rad_per_sec_sqr().k();

		//###HipsterSloth $TODO - pass down the physics data timestamp
		ds4->PhysicsData.TimeInSeconds= -1.0;
    }
    else
    {
        memset(&ds4->PhysicsData, 0, sizeof(PSMPhysicsData));
    }
            
    if (ds4_packet.has_raw_sensor_data())
    {
        const auto &raw_sensor_data = ds4_packet.raw_sensor_data();

        ds4->RawSensorData.Accelerometer.x= raw_sensor_data.accelerometer().i();
        ds4->RawSensorData.Accelerometer.y= raw_sensor_data.accelerometer().j();
        ds4->RawSensorData.Accelerometer.z= raw_sensor_data.accelerometer().k();

        ds4->RawSensorData.Gyroscope.x= raw_sensor_data.gyroscope().i();
        ds4->RawSensorData.Gyroscope.y= raw_sensor_data.gyroscope().j();
        ds4->RawSensorData.Gyroscope.z= raw_sensor_data.gyroscope().k();

		//###HipsterSloth $TODO - pass down the raw sensor data timestamp
		ds4->RawSensorData.TimeInSeconds= -1.0;
    }
    else
    {
		memset(&ds4->RawSensorData, 0, sizeof(PSMPSMoveRawSensorData));
	}

	if (ds4_packet.has_calibrated_sensor_data())
	{
		const auto &calibrated_sensor_data = ds4_packet.calibrated_sensor_data();

		ds4->CalibratedSensorData.Accelerometer.x = calibrated_sensor_data.accelerometer().i();
		ds4->CalibratedSensorData.Accelerometer.y = calibrated_sensor_data.accelerometer().j();
		ds4->CalibratedSensorData.Accelerometer.z = calibrated_sensor_data.accelerometer().k();

		ds4->CalibratedSensorData.Gyroscope.x = calibrated_sensor_data.gyroscope().i();
		ds4->CalibratedSensorData.Gyroscope.y = calibrated_sensor_data.gyroscope().j();
		ds4->CalibratedSensorData.Gyroscope.z = calibrated_sensor_data.gyroscope().k();

		//###HipsterSloth $TODO - pass down the raw sensor data timestamp
		ds4->CalibratedSensorData.TimeInSeconds = -1.0;
	}
	else
	{
		memset(&ds4->RawSensorData, 0, sizeof(PSMPSMoveCalibratedSensorData));
	}

	if (ds4_packet.has_raw_tracker_data())
	{
		const auto &raw_tracker_data = ds4_packet.raw_tracker_data();

		const PSMoveProtocol::Pixel &locationOnTracker = raw_tracker_data.screen_location();
		const PSMoveProtocol::Position &positionOnTracker = raw_tracker_data.relative_position_cm();
		const PSMoveProtocol::Orientation &orientationOnTracker = raw_tracker_data.relative_orientation();

		ds4->RawTrackerData.TrackerID = raw_tracker_data.tracker_id();
		ds4->RawTrackerData.ScreenLocation = { locationOnTracker.x(), locationOnTracker.y() };
		ds4->RawTrackerData.RelativePositionCm = { positionOnTracker.x(), positionOnTracker.y(), positionOnTracker.z() };
		ds4->RawTrackerData.RelativeOrientation = 
			PSM_QuatfCreate(orientationOnTracker.w(), orientationOnTracker.x(), orientationOnTracker.y(), orientationOnTracker.z());
        ds4->RawTrackerData.ValidTrackerBitmask = raw_tracker_data.valid_tracker_bitmask();

		if (raw_tracker_data.has_projected_blob())
		{
            const PSMoveProtocol::Polygon &protocolPolygon = raw_tracker_data.projected_blob();
            PSMTrackingProjection &projection = ds4->RawTrackerData.TrackingProjection;

            assert (protocolPolygon.vertices_size() == 7);
            projection.shape_type = PSMTrackingProjection::PSMShape_LightBar;

            for (int vert_index = 0; vert_index < 3; ++vert_index)
            {
                const PSMoveProtocol::Pixel &pixel = protocolPolygon.vertices(vert_index);

                projection.shape.lightbar.triangle[vert_index].x = pixel.x();
                projection.shape.lightbar.triangle[vert_index].y = pixel.y();
            }
            for (int vert_index = 0; vert_index < 4; ++vert_index)
            {
                const PSMoveProtocol::Pixel &pixel = protocolPolygon.vertices(vert_index+3);

                projection.shape.lightbar.quad[vert_index].x = pixel.x();
                projection.shape.lightbar.quad[vert_index].y = pixel.y();
            }
		}
		else
		{
			PSMTrackingProjection &projection = ds4->RawTrackerData.TrackingProjection;

			projection.shape_type = PSMTrackingProjection::PSMShape_INVALID_PROJECTION;
		}

		if (raw_tracker_data.has_multicam_position_cm())
		{
			const PSMoveProtocol::Position &multicam_position = raw_tracker_data.multicam_position_cm();

			ds4->RawTrackerData.MulticamPositionCm.x = multicam_position.x();
			ds4->RawTrackerData.MulticamPositionCm.y = multicam_position.y();
			ds4->RawTrackerData.MulticamPositionCm.z = multicam_position.z();
			ds4->RawTrackerData.bMulticamPositionValid = true;
		}

		if (raw_tracker_data.has_multicam_orientation())
		{
			const PSMoveProtocol::Orientation &multicam_orientation = raw_tracker_data.multicam_orientation();

			ds4->RawTrackerData.MulticamOrientation.w = multicam_orientation.w();
			ds4->RawTrackerData.MulticamOrientation.x = multicam_orientation.x();
			ds4->RawTrackerData.MulticamOrientation.y = multicam_orientation.y();
			ds4->RawTrackerData.MulticamOrientation.z = multicam_orientation.z();
			ds4->RawTrackerData.bMulticamOrientationValid = true;
		}
	}
	else
	{
		memset(&ds4->RawTrackerData, 0, sizeof(PSMRawTrackerData));
	}

	unsigned int button_bitmask = controller_packet.button_down_bitmask();

	applyPSMButtonState(ds4->DPadUpButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_UP);
	applyPSMButtonState(ds4->DPadDownButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_DOWN);
	applyPSMButtonState(ds4->DPadLeftButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_LEFT);
	applyPSMButtonState(ds4->DPadRightButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_RIGHT);

	applyPSMButtonState(ds4->L1Button, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_L1);
	applyPSMButtonState(ds4->L2Button, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_L2);
	applyPSMButtonState(ds4->L3Button, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_L3);
	applyPSMButtonState(ds4->R1Button, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_R1);
	applyPSMButtonState(ds4->R2Button, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_R2);
	applyPSMButtonState(ds4->R3Button, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_R3);

	applyPSMButtonState(ds4->TriangleButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_TRIANGLE);
	applyPSMButtonState(ds4->CircleButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_CIRCLE);
	applyPSMButtonState(ds4->CrossButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_CROSS);
	applyPSMButtonState(ds4->SquareButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_SQUARE);

	applyPSMButtonState(ds4->ShareButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_SHARE);
	applyPSMButtonState(ds4->OptionsButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_OPTIONS);

	applyPSMButtonState(ds4->PSButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_PS);
	applyPSMButtonState(ds4->TrackPadButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_TRACKPAD);

    ds4->LeftAnalogX = ds4_packet.left_thumbstick_x();
    ds4->LeftAnalogY = ds4_packet.left_thumbstick_y();
    ds4->RightAnalogX = ds4_packet.right_thumbstick_x();
    ds4->RightAnalogY = ds4_packet.right_thumbstick_y();
    ds4->LeftTriggerValue = ds4_packet.left_trigger_value();
    ds4->RightTriggerValue = ds4_packet.right_trigger_value();
}

static void applyVirtualControllerDataFrame(
    const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket& controller_packet,
    PSMVirtualController *virtual_controller)
{
	const auto &virtual_controller_packet= controller_packet.virtualcontroller_state();

    virtual_controller->bIsTrackingEnabled = virtual_controller_packet.istrackingenabled();
    virtual_controller->bIsCurrentlyTracking = virtual_controller_packet.iscurrentlytracking();
	virtual_controller->bIsPositionValid = virtual_controller_packet.ispositionvalid();
            
    virtual_controller->Pose.Orientation.w= 1.f;
    virtual_controller->Pose.Orientation.x= 0.f;
    virtual_controller->Pose.Orientation.y= 0.f;
    virtual_controller->Pose.Orientation.z= 0.f;

    virtual_controller->Pose.Position.x= virtual_controller_packet.position_cm().x();
    virtual_controller->Pose.Position.y= virtual_controller_packet.position_cm().y();
    virtual_controller->Pose.Position.z= virtual_controller_packet.position_cm().z();

    virtual_controller->vendorID= virtual_controller_packet.vendorid();
    virtual_controller->productID= virtual_controller_packet.productid();

    virtual_controller->numAxes = virtual_controller_packet.axisstates_size();
    virtual_controller->numButtons = virtual_controller_packet.numbuttons();

    unsigned int button_bitmask = controller_packet.button_down_bitmask();
    memset(virtual_controller->buttonStates, PSMButtonState_UP, sizeof(virtual_controller->buttonStates));
    for (int button_index = 0; button_index < virtual_controller->numButtons; ++button_index)
    {
    	applyPSMButtonState(virtual_controller->buttonStates[button_index], button_bitmask, button_index);
    }

    memset(virtual_controller->axisStates, 0x7f, sizeof(virtual_controller->axisStates));
    for (int axis_index = 0; axis_index < virtual_controller->numAxes; ++axis_index)
    {
        virtual_controller->axisStates[axis_index]= virtual_controller_packet.axisstates(axis_index);
    }

    if (virtual_controller_packet.has_physics_data())
    {
        const auto &raw_physics_data = virtual_controller_packet.physics_data();

        virtual_controller->PhysicsData.LinearVelocityCmPerSec.x = raw_physics_data.velocity_cm_per_sec().i();
        virtual_controller->PhysicsData.LinearVelocityCmPerSec.y = raw_physics_data.velocity_cm_per_sec().j();
        virtual_controller->PhysicsData.LinearVelocityCmPerSec.z = raw_physics_data.velocity_cm_per_sec().k();

        virtual_controller->PhysicsData.LinearAccelerationCmPerSecSqr.x = raw_physics_data.acceleration_cm_per_sec_sqr().i();
        virtual_controller->PhysicsData.LinearAccelerationCmPerSecSqr.y = raw_physics_data.acceleration_cm_per_sec_sqr().j();
        virtual_controller->PhysicsData.LinearAccelerationCmPerSecSqr.z = raw_physics_data.acceleration_cm_per_sec_sqr().k();

        virtual_controller->PhysicsData.AngularVelocityRadPerSec.x = 0.f;
        virtual_controller->PhysicsData.AngularVelocityRadPerSec.y = 0.f;
        virtual_controller->PhysicsData.AngularVelocityRadPerSec.z = 0.f;

        virtual_controller->PhysicsData.AngularAccelerationRadPerSecSqr.x = 0.f;
        virtual_controller->PhysicsData.AngularAccelerationRadPerSecSqr.y = 0.f;
        virtual_controller->PhysicsData.AngularAccelerationRadPerSecSqr.z = 0.f;

		//###HipsterSloth $TODO - pass down the physics data timestamp
		virtual_controller->PhysicsData.TimeInSeconds= -1.0;
    }
    else
    {
        memset(&virtual_controller->PhysicsData, 0, sizeof(PSMPhysicsData));
    }

	if (virtual_controller_packet.has_raw_tracker_data())
	{
		const auto &raw_tracker_data = virtual_controller_packet.raw_tracker_data();

		const PSMoveProtocol::Pixel &locationOnTracker = raw_tracker_data.screen_location();
		const PSMoveProtocol::Position &positionOnTracker = raw_tracker_data.relative_position_cm();

		virtual_controller->RawTrackerData.TrackerID = raw_tracker_data.tracker_id();
		virtual_controller->RawTrackerData.ScreenLocation = { locationOnTracker.x(), locationOnTracker.y() };
		virtual_controller->RawTrackerData.RelativePositionCm = { positionOnTracker.x(), positionOnTracker.y(), positionOnTracker.z() };
        virtual_controller->RawTrackerData.ValidTrackerBitmask = raw_tracker_data.valid_tracker_bitmask();

		if (raw_tracker_data.has_projected_sphere())
		{
			const PSMoveProtocol::Ellipse &protocolEllipse = raw_tracker_data.projected_sphere();
			PSMTrackingProjection &projection = virtual_controller->RawTrackerData.TrackingProjection;

			projection.shape.ellipse.center.x = protocolEllipse.center().x();
			projection.shape.ellipse.center.y = protocolEllipse.center().y();
			projection.shape.ellipse.half_x_extent = protocolEllipse.half_x_extent();
			projection.shape.ellipse.half_y_extent = protocolEllipse.half_y_extent();
			projection.shape.ellipse.angle = protocolEllipse.angle();
			projection.shape_type = PSMTrackingProjection::PSMShape_Ellipse;
		}
		else
		{
			PSMTrackingProjection &projection = virtual_controller->RawTrackerData.TrackingProjection;

			projection.shape_type = PSMTrackingProjection::PSMShape_INVALID_PROJECTION;
		}

		if (raw_tracker_data.has_multicam_position_cm())
		{
			const PSMoveProtocol::Position &multicam_position = raw_tracker_data.multicam_position_cm();

			virtual_controller->RawTrackerData.MulticamPositionCm.x = multicam_position.x();
			virtual_controller->RawTrackerData.MulticamPositionCm.y = multicam_position.y();
			virtual_controller->RawTrackerData.MulticamPositionCm.z = multicam_position.z();
			virtual_controller->RawTrackerData.bMulticamPositionValid = true;
		}
	}
	else
	{
		memset(&virtual_controller->RawTrackerData, 0, sizeof(PSMRawTrackerData));
	}
}

static void applyPSMButtonState(
    PSMButtonState &button,
    unsigned int button_bitmask,
    unsigned int button_bit)
{
    const bool is_down= (button_bitmask & (1 << button_bit)) > 0;

    switch (button)
    {
    case PSMButtonState_UP:
        button= is_down ? PSMButtonState_PRESSED : PSMButtonState_UP;
        break;
    case PSMButtonState_PRESSED:
        button= is_down ? PSMButtonState_DOWN : PSMButtonState_RELEASED;
        break;
    case PSMButtonState_DOWN:
        button= is_down ? PSMButtonState_DOWN : PSMButtonState_RELEASED;
        break;
    case PSMButtonState_RELEASED:
        button= is_down ? PSMButtonState_PRESSED : PSMButtonState_UP;
        break;
    };
}

static void applyTrackerDataFrame(
	const PSMoveProtocol::DeviceOutputDataFrame_TrackerDataPacket& tracker_packet, 
	PSMTracker *tracker)
{
	assert(tracker_packet.tracker_id() == tracker->tracker_info.tracker_id);

    // Compute the data frame receive window statistics if we have received enough samples
    {
        long long now =
            std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()).count();
        long long diff = now - tracker->data_frame_last_received_time;

        if (diff > 0)
        {
            float seconds = static_cast<float>(diff) / 1000.f;
            float fps = 1.f / seconds;

            tracker->data_frame_average_fps = (0.9f)*tracker->data_frame_average_fps + (0.1f)*fps;
        }

        tracker->data_frame_last_received_time = now;
    }

    if (tracker_packet.sequence_num() > tracker->sequence_num)
    {
        tracker->sequence_num = tracker_packet.sequence_num();
        tracker->is_connected = tracker_packet.isconnected();
    }
}

static void applyHmdDataFrame(
	const PSMoveProtocol::DeviceOutputDataFrame_HMDDataPacket& hmd_packet, 
	PSMHeadMountedDisplay *hmd)
{
	// Ignore old packets
	if (hmd_packet.sequence_num() <= hmd->OutputSequenceNum)
		return;

    // Set the generic items
    hmd->bValid = hmd_packet.hmd_id() != -1;
    hmd->HmdType = static_cast<PSMHmdType>(hmd_packet.hmd_type());
    hmd->OutputSequenceNum = hmd_packet.sequence_num();
    hmd->IsConnected = hmd_packet.isconnected();

    // Compute the data frame receive window statistics if we have received enough samples
    {
        long long now = 
            std::chrono::duration_cast< std::chrono::milliseconds >(
                std::chrono::system_clock::now().time_since_epoch()).count();
        long long diff= now - hmd->DataFrameLastReceivedTime;

        if (diff > 0)
        {
            float seconds= static_cast<float>(diff) / 1000.f;
            float fps= 1.f / seconds;

            hmd->DataFrameAverageFPS= (0.9f)*hmd->DataFrameAverageFPS + (0.1f)*fps;
        }

        hmd->DataFrameLastReceivedTime= now;
    }

	// Don't bother updating the rest of the hmd state if it's not connected
	if (!hmd->IsConnected)
		return;

    switch (hmd->HmdType) 
	{
        case PSMHmd_Morpheus:
			applyMorpheusDataFrame(hmd_packet, &hmd->HmdState.MorpheusState);
            break;
        case PSMHmd_Virtual:
			applyVirtualHMDDataFrame(hmd_packet, &hmd->HmdState.VirtualHMDState);
            break;            
        default:
            break;
    }
}

static void applyMorpheusDataFrame(
	const PSMoveProtocol::DeviceOutputDataFrame_HMDDataPacket& hmd_packet,
	PSMMorpheus *morpheus)
{
    const auto &morpheus_data_frame = hmd_packet.morpheus_state();

	morpheus->bIsTrackingEnabled = morpheus_data_frame.istrackingenabled();
	morpheus->bIsCurrentlyTracking = morpheus_data_frame.iscurrentlytracking();
	morpheus->bIsOrientationValid = morpheus_data_frame.isorientationvalid();
	morpheus->bIsPositionValid = morpheus_data_frame.ispositionvalid();

	morpheus->Pose.Orientation.w = morpheus_data_frame.orientation().w();
	morpheus->Pose.Orientation.x = morpheus_data_frame.orientation().x();
	morpheus->Pose.Orientation.y = morpheus_data_frame.orientation().y();
	morpheus->Pose.Orientation.z = morpheus_data_frame.orientation().z();

	morpheus->Pose.Position.x = morpheus_data_frame.position_cm().x();
	morpheus->Pose.Position.y = morpheus_data_frame.position_cm().y();
	morpheus->Pose.Position.z = morpheus_data_frame.position_cm().z();

	if (morpheus_data_frame.has_raw_sensor_data())
	{
		const auto &raw_sensor_data = morpheus_data_frame.raw_sensor_data();

		morpheus->RawSensorData.Accelerometer.x = raw_sensor_data.accelerometer().i();
		morpheus->RawSensorData.Accelerometer.y = raw_sensor_data.accelerometer().j();
		morpheus->RawSensorData.Accelerometer.z = raw_sensor_data.accelerometer().k();

		morpheus->RawSensorData.Gyroscope.x = raw_sensor_data.gyroscope().i();
		morpheus->RawSensorData.Gyroscope.y = raw_sensor_data.gyroscope().j();
		morpheus->RawSensorData.Gyroscope.z = raw_sensor_data.gyroscope().k();
	}
	else
	{
		memset(&morpheus->RawSensorData, 0, sizeof(PSMMorpheusRawSensorData));
	}

	if (morpheus_data_frame.has_calibrated_sensor_data())
	{
		const auto &calibrated_sensor_data = morpheus_data_frame.calibrated_sensor_data();

		morpheus->CalibratedSensorData.Accelerometer.x = calibrated_sensor_data.accelerometer().i();
		morpheus->CalibratedSensorData.Accelerometer.y = calibrated_sensor_data.accelerometer().j();
		morpheus->CalibratedSensorData.Accelerometer.z = calibrated_sensor_data.accelerometer().k();

		morpheus->CalibratedSensorData.Gyroscope.x = calibrated_sensor_data.gyroscope().i();
		morpheus->CalibratedSensorData.Gyroscope.y = calibrated_sensor_data.gyroscope().j();
		morpheus->CalibratedSensorData.Gyroscope.z = calibrated_sensor_data.gyroscope().k();
	}
	else
	{
		memset(&morpheus->CalibratedSensorData, 0, sizeof(PSMMorpheusCalibratedSensorData));
	}

	if (morpheus_data_frame.has_raw_tracker_data())
	{
		const auto &raw_tracker_data = morpheus_data_frame.raw_tracker_data();

		const PSMoveProtocol::Pixel &locationOnTracker = raw_tracker_data.screen_location();
		const PSMoveProtocol::Position &positionOnTrackerCm = raw_tracker_data.relative_position_cm();

		morpheus->RawTrackerData.TrackerID = raw_tracker_data.tracker_id();
		morpheus->RawTrackerData.ScreenLocation = {locationOnTracker.x(), locationOnTracker.y()};
		morpheus->RawTrackerData.RelativePositionCm = 
			{positionOnTrackerCm.x(), positionOnTrackerCm.y(), positionOnTrackerCm.z()};
        morpheus->RawTrackerData.ValidTrackerBitmask = raw_tracker_data.valid_tracker_bitmask();

		if (raw_tracker_data.has_projected_point_cloud())
		{
			const PSMoveProtocol::Polygon &protocolPointCloud = raw_tracker_data.projected_point_cloud();
			PSMTrackingProjection &projection = morpheus->RawTrackerData.TrackingProjection;

			projection.shape.pointcloud.point_count = std::min(protocolPointCloud.vertices_size(), 7);
			for (int point_index = 0; point_index < projection.shape.pointcloud.point_count; ++point_index)
			{
				const PSMoveProtocol::Pixel &point= protocolPointCloud.vertices(point_index);

				projection.shape.pointcloud.points[point_index].x = point.x();
				projection.shape.pointcloud.points[point_index].y = point.y();
			}					
			projection.shape_type = PSMTrackingProjection::PSMShape_PointCloud;
		}
		else
		{
			PSMTrackingProjection &projection = morpheus->RawTrackerData.TrackingProjection;

			projection.shape_type = PSMTrackingProjection::PSMShape_INVALID_PROJECTION;
		}
	}
	else
	{
		memset(&morpheus->RawTrackerData, 0, sizeof(PSMRawTrackerData));
	}

	if (morpheus_data_frame.has_physics_data())
	{
		const auto &raw_physics_data = morpheus_data_frame.physics_data();

		morpheus->PhysicsData.LinearVelocityCmPerSec.x = raw_physics_data.velocity_cm_per_sec().i();
		morpheus->PhysicsData.LinearVelocityCmPerSec.y = raw_physics_data.velocity_cm_per_sec().j();
		morpheus->PhysicsData.LinearVelocityCmPerSec.z = raw_physics_data.velocity_cm_per_sec().k();

		morpheus->PhysicsData.LinearAccelerationCmPerSecSqr.x = raw_physics_data.acceleration_cm_per_sec_sqr().i();
		morpheus->PhysicsData.LinearAccelerationCmPerSecSqr.y = raw_physics_data.acceleration_cm_per_sec_sqr().j();
		morpheus->PhysicsData.LinearAccelerationCmPerSecSqr.z = raw_physics_data.acceleration_cm_per_sec_sqr().k();

		morpheus->PhysicsData.AngularVelocityRadPerSec.x = raw_physics_data.angular_velocity_rad_per_sec().i();
		morpheus->PhysicsData.AngularVelocityRadPerSec.y = raw_physics_data.angular_velocity_rad_per_sec().j();
		morpheus->PhysicsData.AngularVelocityRadPerSec.z = raw_physics_data.angular_velocity_rad_per_sec().k();

		morpheus->PhysicsData.AngularAccelerationRadPerSecSqr.x = raw_physics_data.angular_acceleration_rad_per_sec_sqr().i();
		morpheus->PhysicsData.AngularAccelerationRadPerSecSqr.y = raw_physics_data.angular_acceleration_rad_per_sec_sqr().j();
		morpheus->PhysicsData.AngularAccelerationRadPerSecSqr.z = raw_physics_data.angular_acceleration_rad_per_sec_sqr().k();
	}
	else
	{
		memset(&morpheus->PhysicsData, 0, sizeof(PSMPhysicsData));
	}
}

static void applyVirtualHMDDataFrame(
	const PSMoveProtocol::DeviceOutputDataFrame_HMDDataPacket& hmd_packet,
	PSMVirtualHMD *virtualHMD)
{
    const auto &virtual_hmd_data_frame = hmd_packet.virtual_hmd_state();

	virtualHMD->bIsTrackingEnabled = virtual_hmd_data_frame.istrackingenabled();
	virtualHMD->bIsCurrentlyTracking = virtual_hmd_data_frame.iscurrentlytracking();
	virtualHMD->bIsPositionValid = virtual_hmd_data_frame.ispositionvalid();

	virtualHMD->Pose.Orientation.w = 1.f;
	virtualHMD->Pose.Orientation.x = 0.f;
	virtualHMD->Pose.Orientation.y = 0.f;
	virtualHMD->Pose.Orientation.z = 0.f;

	virtualHMD->Pose.Position.x = virtual_hmd_data_frame.position_cm().x();
	virtualHMD->Pose.Position.y = virtual_hmd_data_frame.position_cm().y();
	virtualHMD->Pose.Position.z = virtual_hmd_data_frame.position_cm().z();

	if (virtual_hmd_data_frame.has_raw_tracker_data())
	{
		const auto &raw_tracker_data = virtual_hmd_data_frame.raw_tracker_data();

		const PSMoveProtocol::Pixel &locationOnTracker = raw_tracker_data.screen_location();
		const PSMoveProtocol::Position &positionOnTrackerCm = raw_tracker_data.relative_position_cm();

		virtualHMD->RawTrackerData.TrackerID = raw_tracker_data.tracker_id();
		virtualHMD->RawTrackerData.ScreenLocation = {locationOnTracker.x(), locationOnTracker.y()};
		virtualHMD->RawTrackerData.RelativePositionCm = 
			{positionOnTrackerCm.x(), positionOnTrackerCm.y(), positionOnTrackerCm.z()};
        virtualHMD->RawTrackerData.ValidTrackerBitmask = raw_tracker_data.valid_tracker_bitmask();

		if (raw_tracker_data.has_projected_sphere())
		{
			const PSMoveProtocol::Ellipse &protocolEllipse = raw_tracker_data.projected_sphere();
			PSMTrackingProjection &projection = virtualHMD->RawTrackerData.TrackingProjection;

			projection.shape.ellipse.center.x = protocolEllipse.center().x();
			projection.shape.ellipse.center.y = protocolEllipse.center().y();
			projection.shape.ellipse.half_x_extent = protocolEllipse.half_x_extent();
			projection.shape.ellipse.half_y_extent = protocolEllipse.half_y_extent();
			projection.shape.ellipse.angle = protocolEllipse.angle();
			projection.shape_type = PSMTrackingProjection::PSMShape_Ellipse;
		}
		else
		{
			PSMTrackingProjection &projection = virtualHMD->RawTrackerData.TrackingProjection;

			projection.shape_type = PSMTrackingProjection::PSMShape_INVALID_PROJECTION;
		}
	}
	else
	{
		memset(&virtualHMD->RawTrackerData, 0, sizeof(PSMRawTrackerData));
	}

	if (virtual_hmd_data_frame.has_physics_data())
	{
		const auto &raw_physics_data = virtual_hmd_data_frame.physics_data();

		virtualHMD->PhysicsData.LinearVelocityCmPerSec.x = raw_physics_data.velocity_cm_per_sec().i();
		virtualHMD->PhysicsData.LinearVelocityCmPerSec.y = raw_physics_data.velocity_cm_per_sec().j();
		virtualHMD->PhysicsData.LinearVelocityCmPerSec.z = raw_physics_data.velocity_cm_per_sec().k();

		virtualHMD->PhysicsData.LinearAccelerationCmPerSecSqr.x = raw_physics_data.acceleration_cm_per_sec_sqr().i();
		virtualHMD->PhysicsData.LinearAccelerationCmPerSecSqr.y = raw_physics_data.acceleration_cm_per_sec_sqr().j();
		virtualHMD->PhysicsData.LinearAccelerationCmPerSecSqr.z = raw_physics_data.acceleration_cm_per_sec_sqr().k();

		virtualHMD->PhysicsData.AngularVelocityRadPerSec.x = 0.f;
		virtualHMD->PhysicsData.AngularVelocityRadPerSec.y = 0.f;
		virtualHMD->PhysicsData.AngularVelocityRadPerSec.z = 0.f;

		virtualHMD->PhysicsData.AngularAccelerationRadPerSecSqr.x = 0.f;
		virtualHMD->PhysicsData.AngularAccelerationRadPerSecSqr.y = 0.f;
		virtualHMD->PhysicsData.AngularAccelerationRadPerSecSqr.z = 0.f;
	}
	else
	{
		memset(&virtualHMD->PhysicsData, 0, sizeof(PSMPhysicsData));
	}
}

// INotificationListener
void PSMoveClient::handle_notification(ResponsePtr notification)
{
    assert(notification->request_id() == -1);

    PSMEventMessage::eEventType specificEventType= PSMEventMessage::PSMEvent_opaqueServiceEvent;

    // See if we can translate this to an event type a client without protocol access can see
    switch(notification->type())
    {
    case PSMoveProtocol::Response_ResponseType_CONTROLLER_LIST_UPDATED:
        specificEventType= PSMEventMessage::PSMEvent_controllerListUpdated;
        break;
    case PSMoveProtocol::Response_ResponseType_TRACKER_LIST_UPDATED:
        specificEventType = PSMEventMessage::PSMEvent_trackerListUpdated;
        break;
    case PSMoveProtocol::Response_ResponseType_HMD_LIST_UPDATED:
        specificEventType = PSMEventMessage::PSMEvent_hmdListUpdated;
        break;
	case PSMoveProtocol::Response_ResponseType_SYSTEM_BUTTON_PRESSED:
		specificEventType = PSMEventMessage::PSMEvent_systemButtonPressed;
		break;
    }

    enqueue_event_message(specificEventType, notification);
}

// IClientNetworkEventListener
void PSMoveClient::handle_server_connection_opened()
{
    CLIENT_LOG_INFO("handle_server_connection_opened") << "Connected to service" << std::endl;

    enqueue_event_message(PSMEventMessage::PSMEvent_connectedToService, ResponsePtr());
}

void PSMoveClient::handle_server_connection_open_failed(const boost::system::error_code& ec)
{
    CLIENT_LOG_ERROR("handle_server_connection_open_failed") << "Failed to connect to service: " << ec.message() << std::endl;

    enqueue_event_message(PSMEventMessage::PSMEvent_failedToConnectToService, ResponsePtr());
}

void PSMoveClient::handle_server_connection_closed()
{
    CLIENT_LOG_INFO("handle_server_connection_closed") << "Disconnected from service" << std::endl;

    enqueue_event_message(PSMEventMessage::PSMEvent_disconnectedFromService, ResponsePtr());
}

void PSMoveClient::handle_server_connection_close_failed(const boost::system::error_code& ec)
{
    CLIENT_LOG_ERROR("handle_server_connection_close_failed") << "Error disconnecting from service: " << ec.message() << std::endl;
}

void PSMoveClient::handle_server_connection_socket_error(const boost::system::error_code& ec)
{
    CLIENT_LOG_ERROR("handle_server_connection_close_failed") << "Socket error: " << ec.message() << std::endl;
}

// Request Manager Callback
void PSMoveClient::handle_response_message(
    const PSMResponseMessage *response_message,
    void *userdata)
{
    PSMoveClient *this_ptr = reinterpret_cast<PSMoveClient *>(userdata);

    if (response_message->request_id != PSM_INVALID_REQUEST_ID)
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
void PSMoveClient::process_event_message(
	const PSMEventMessage *event_message)
{
    switch (event_message->event_type)
    {
    // Client Events
    case PSMEventMessage::PSMEvent_connectedToService:
        m_bIsConnected= true;
        m_bHasConnectionStatusChanged= true;
        break;
    case PSMEventMessage::PSMEvent_failedToConnectToService:
        m_bIsConnected= false;
        m_bHasConnectionStatusChanged= true;
        break;
    case PSMEventMessage::PSMEvent_disconnectedFromService:
        m_bIsConnected= false;
        m_bHasConnectionStatusChanged= true;
        break;

    // Service Events
    case PSMEventMessage::PSMEvent_opaqueServiceEvent:
        // Need to have protocol access to see what kind of event this is
        CLIENT_LOG_INFO("PSM_Update") << "Dropping opaque service event";
        break;
    case PSMEventMessage::PSMEvent_controllerListUpdated:
        m_bHasControllerListChanged= true;
        break;
    case PSMEventMessage::PSMEvent_trackerListUpdated:
        m_bHasTrackerListChanged= true;
        break;
    case PSMEventMessage::PSMEvent_hmdListUpdated:
        m_bHasHMDListChanged= true;
        break;
    case PSMEventMessage::PSMEvent_systemButtonPressed:
        m_bWasSystemButtonPressed= true;
        break;
    default:
        assert(0 && "unreachable");
        break;
    }
}

void PSMoveClient::enqueue_event_message(
    PSMEventMessage::eEventType event_type,
    ResponsePtr event)
{
    PSMMessage message;

    memset(&message, 0, sizeof(PSMMessage));
    message.payload_type = PSMMessage::_messagePayloadType_Event;
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

bool PSMoveClient::register_callback(
    PSMRequestID request_id,
    PSMResponseCallback callback,
    void *callback_userdata)
{
    bool bSuccess = false;

    if (request_id != PSM_INVALID_REQUEST_ID)
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

bool PSMoveClient::execute_callback(
    const PSMResponseMessage *response_message)
{
    const PSMRequestID request_id = response_message->request_id;
    bool bExecutedCallback = false;

    if (request_id != PSM_INVALID_REQUEST_ID)
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

void PSMoveClient::enqueue_response_message(
    const PSMResponseMessage *response_message)
{
    PSMMessage message;

    memset(&message, 0, sizeof(PSMMessage));
    message.payload_type = PSMMessage::_messagePayloadType_Response;
    message.response_data= *response_message;

    // Add the message to the message queue
    m_message_queue.push_back(message);
}

bool PSMoveClient::cancel_callback(PSMRequestID request_id)
{
    bool bSuccess = false;

    if (request_id != PSM_INVALID_REQUEST_ID)
    {
        t_pending_request_map::iterator iter= m_pending_request_map.find(request_id);

        if (iter != m_pending_request_map.end())
        {
            const PendingRequest &pendingRequest = iter->second;
                
            // Notify the response callback that the request was canceled
            if (pendingRequest.response_callback != nullptr)
            {
                PSMResponseMessage response;
                memset(&response, 0, sizeof(PSMResponseMessage));
                response.result_code= PSMResult_Canceled;
                response.request_id= request_id;
                response.payload_type= PSMResponseMessage::_responsePayloadType_HmdList;
                pendingRequest.response_callback(&response, pendingRequest.response_userdata);
            }
            m_pending_request_map.erase(iter);
            bSuccess = true;
        }
    }

    return bSuccess;
}

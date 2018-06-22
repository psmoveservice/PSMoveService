//-- includes -----
#include "ClientRequestManager.h"
#include "ClientNetworkManager.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include <cassert>
#include <map>
#include <utility>

//-- definitions -----
struct RequestContext
{
    RequestPtr request;  // std::shared_ptr<PSMoveProtocol::Request>
};
typedef std::map<int, RequestContext> t_request_context_map;
typedef std::map<int, RequestContext>::iterator t_request_context_map_iterator;
typedef std::pair<int, RequestContext> t_id_request_context_pair;
typedef std::vector<ResponsePtr> t_response_reference_cache;
typedef std::vector<RequestPtr> t_request_reference_cache;

class ClientRequestManagerImpl
{
public:
    ClientRequestManagerImpl(
        IDataFrameListener *dataFrameListener,
        PSMResponseCallback callback,
        void *userdata)
        : m_dataFrameListener(dataFrameListener)
        , m_callback(callback)
        , m_callback_userdata(userdata)
        , m_pending_requests()
        , m_next_request_id(0)
    {
    }

    void flush_response_cache()
    {
        // Drop all of the request/response references,
        // NOTE: std::vector::clear() calls the destructor on each element in the vector
        // This will decrement the last ref count to the parameter data, causing them to get cleaned up.
        m_request_reference_cache.clear();
        m_response_reference_cache.clear();
    }

    void send_request(RequestPtr request)
    {
        RequestContext context;

        context.request = request;

        request->set_request_id(m_next_request_id);
        ++m_next_request_id;

        // Add the request to the pending request map.
        // Requests should never be double registered.
        assert(m_pending_requests.find(request->request_id()) == m_pending_requests.end());
        m_pending_requests.insert(t_id_request_context_pair(request->request_id(), context));

        // Send the request off to the network manager to get sent to the server
        ClientNetworkManager::get_instance()->send_request(request);
    }

    void handle_request_canceled(RequestPtr request)
    {
        // Create a general canceled result
        ResponsePtr response(new PSMoveProtocol::Response);

        response->set_type(PSMoveProtocol::Response_ResponseType_GENERAL_RESULT);
        response->set_request_id(request->request_id());
        response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_CANCELED);

        handle_response(response);
    }

    void handle_response(ResponsePtr response)
    {
        // Get the request awaiting completion
        t_request_context_map_iterator pending_request_entry= m_pending_requests.find(response->request_id());
        assert(pending_request_entry != m_pending_requests.end());

        // The context holds everything a handler needs to evaluate a response
        const RequestContext &context= pending_request_entry->second;

        // Notify the callback of the response
        if (m_callback != nullptr)
        {
            PSMResponseMessage response_message;

            // Generate a client API response message from 
            build_response_message(context.request, response, &response_message);

            m_callback(&response_message, m_callback_userdata);
        }

        // Remove the pending request from the map
        m_pending_requests.erase(pending_request_entry);
    }

    void build_response_message(
        RequestPtr request,
        ResponsePtr response,
        PSMResponseMessage *out_response_message)
    {        
        memset(out_response_message, 0, sizeof(PSMResponseMessage));

        // Let the response know what request the result came from
        out_response_message->request_id = request->request_id();

        // Translate internal result codes into public facing result codes
        switch (response->result_code())
        {
        case PSMoveProtocol::Response_ResultCode_RESULT_OK:
            out_response_message->result_code = PSMResult_Success;
            break;
        case PSMoveProtocol::Response_ResultCode_RESULT_ERROR:
            out_response_message->result_code = PSMResult_Error;
            break;
        case PSMoveProtocol::Response_ResultCode_RESULT_CANCELED:
            out_response_message->result_code = PSMResult_Canceled;
            break;
        default:
            assert(false && "Unknown response result code");
        }

        // Attach an opaque pointer to the PSMoveProtocol request.
        // Client code that has linked against PSMoveProtocol library
        // can access this pointer via the GET_PSMOVEPROTOCOL_REQUEST() macro.
        out_response_message->opaque_request_handle = static_cast<const void*>(request.get());

        // The opaque request pointer will only remain valid until the next call to update()
        // at which time the request reference cache gets cleared out.
        m_request_reference_cache.push_back(request);

        {
            // Create a smart pointer to a new copy of the response.
            // If we just add the given event smart pointer to the reference cache
            // we'll be storing a reference to the shared m_packed_response on the client network manager
            // which gets constantly overwritten with new incoming responses.
            ResponsePtr responseCopy(new PSMoveProtocol::Response(*response.get()));

            // Attach an opaque pointer to the PSMoveProtocol response.
            // Client code that has linked against PSMoveProtocol library
            // can access this pointer via the GET_PSMOVEPROTOCOL_RESPONSE() macro.
            out_response_message->opaque_response_handle = static_cast<const void*>(responseCopy.get());

            // The opaque response pointer will only remain valid until the next call to update()
            // at which time the response reference cache gets cleared out.
            m_response_reference_cache.push_back(responseCopy);
        }

        // Write response specific data
        if (response->result_code() == PSMoveProtocol::Response_ResultCode_RESULT_OK)
        {
            switch (response->type())
            {
            case PSMoveProtocol::Response_ResponseType_CONTROLLER_STREAM_STARTED:
                {
                    const PSMoveProtocol::DeviceOutputDataFrame *dataFrame= &response->result_controller_stream_started().initial_data_frame();
                    m_dataFrameListener->handle_data_frame(dataFrame);
                } break;
			case PSMoveProtocol::Response_ResponseType_SERVICE_VERSION:
                build_service_version_response_message(response, &out_response_message->payload.service_version);
                out_response_message->payload_type = PSMResponseMessage::_responsePayloadType_ServiceVersion;
                break;
            case PSMoveProtocol::Response_ResponseType_CONTROLLER_LIST:
                build_controller_list_response_message(response, &out_response_message->payload.controller_list);
                out_response_message->payload_type = PSMResponseMessage::_responsePayloadType_ControllerList;
                break;
            case PSMoveProtocol::Response_ResponseType_TRACKER_LIST:
                build_tracker_list_response_message(response, &out_response_message->payload.tracker_list);
                out_response_message->payload_type = PSMResponseMessage::_responsePayloadType_TrackerList;
                break;
			case PSMoveProtocol::Response_ResponseType_HMD_LIST:
				build_hmd_list_response_message(response, &out_response_message->payload.hmd_list);
				out_response_message->payload_type = PSMResponseMessage::_responsePayloadType_HmdList;
				break;
			case PSMoveProtocol::Response_ResponseType_TRACKING_SPACE_SETTINGS:
				build_tracking_space_response_message(response, &out_response_message->payload.tracking_space);
				out_response_message->payload_type = PSMResponseMessage::_responsePayloadType_TrackingSpace;
				break;
            default:
                out_response_message->payload_type = PSMResponseMessage::_responsePayloadType_Empty;
                break;
            }
        }
    }

	void build_service_version_response_message(
		ResponsePtr response,
		PSMServiceVersion *service_version)
	{
		const auto &VersionResponse = response->result_service_version();

		strncpy(service_version->version_string, VersionResponse.version().c_str(), PSMOVESERVICE_MAX_VERSION_STRING_LEN);
	}

    void build_controller_list_response_message(
        ResponsePtr response,
        PSMControllerList *controller_list)
    {
        int src_controller_count = 0;
        int dest_controller_count= 0;

        // Copy the controller entries into the response payload
        while (src_controller_count < response->result_controller_list().controllers_size()
                && src_controller_count < PSMOVESERVICE_MAX_CONTROLLER_COUNT)
        {
            const auto &ControllerResponse = response->result_controller_list().controllers(src_controller_count);
            const bool bIsBluetooth=
				ControllerResponse.connection_type() == PSMoveProtocol::Response_ResultControllerList_ControllerInfo_ConnectionType_BLUETOOTH;

			// For some controllers, we only include them in the public facing controller list
			// if the controller is currently connected via bluetooth
			bool bIsPublicFacingController= false;

            // Convert the PSMoveProtocol controller enum to the public ClientControllerView enum
            PSMControllerType controllerType;
            switch (ControllerResponse.controller_type())
            {
            case PSMoveProtocol::PSMOVE:
				bIsPublicFacingController= bIsBluetooth;
                controllerType = PSMController_Move;
                break;
            case PSMoveProtocol::PSNAVI:
				bIsPublicFacingController= true;
                controllerType = PSMController_Navi;
                break;
            case PSMoveProtocol::PSDUALSHOCK4:
				bIsPublicFacingController= bIsBluetooth;
                controllerType = PSMController_DualShock4;
                break;
            case PSMoveProtocol::VIRTUALCONTROLLER:
                bIsPublicFacingController= true;
                controllerType= PSMController_Virtual;
                break;
            default:
                assert(0 && "unreachable");
                controllerType = PSMController_None;
            }

			if (bIsPublicFacingController)
			{
                // Add an entry to the controller list
                controller_list->controller_type[dest_controller_count] = controllerType;
                controller_list->controller_id[dest_controller_count] = ControllerResponse.controller_id();
				controller_list->controller_hand[dest_controller_count] = (PSMControllerHand)ControllerResponse.controller_hand();
				strncpy(controller_list->controller_serial[dest_controller_count], ControllerResponse.device_serial().c_str(), PSMOVESERVICE_CONTROLLER_SERIAL_LEN);
				strncpy(controller_list->parent_controller_serial[dest_controller_count], ControllerResponse.parent_controller_serial().c_str(), PSMOVESERVICE_CONTROLLER_SERIAL_LEN);
                ++dest_controller_count;
            }

            ++src_controller_count;
        }

        // Record how many controllers we copied into the payload
        controller_list->count = dest_controller_count;
    }

    inline PSMPosef protocol_pose_to_psmove_pose(const PSMoveProtocol::Pose &pose)
    {
        PSMPosef result;

        result.Orientation.w = pose.orientation().w();
        result.Orientation.x = pose.orientation().x();
        result.Orientation.y = pose.orientation().y();
        result.Orientation.z = pose.orientation().z();

        result.Position.x = pose.position().x();
        result.Position.y = pose.position().y();
        result.Position.z = pose.position().z();

        return result;
    }

    void build_tracker_list_response_message(
        ResponsePtr response,
        PSMTrackerList *tracker_list)
    {
        int tracker_count = 0;

        // Copy the controller entries into the response payload
        while (tracker_count < response->result_tracker_list().trackers_size()
            && tracker_count < PSMOVESERVICE_MAX_TRACKER_COUNT)
        {
            const auto &TrackerResponse = response->result_tracker_list().trackers(tracker_count);
            PSMClientTrackerInfo &TrackerInfo= tracker_list->trackers[tracker_count];

            TrackerInfo.tracker_id = TrackerResponse.tracker_id();

            switch (TrackerResponse.tracker_type())
            {
            case PSMoveProtocol::TrackerType::PS3EYE:
                TrackerInfo.tracker_type = PSMTracker_PS3Eye;
                break;
            default:
                assert(0 && "unreachable");
            }

            switch (TrackerResponse.tracker_driver())
            {
            case PSMoveProtocol::TrackerDriver::LIBUSB:
                TrackerInfo.tracker_driver = PSMDriver_LIBUSB;
                break;
            case PSMoveProtocol::TrackerDriver::CL_EYE:
                TrackerInfo.tracker_driver = PSMDriver_CL_EYE;
                break;
            case PSMoveProtocol::TrackerDriver::CL_EYE_MULTICAM:
                TrackerInfo.tracker_driver = PSMDriver_CL_EYE_MULTICAM;
                break;
            case PSMoveProtocol::TrackerDriver::GENERIC_WEBCAM:
                TrackerInfo.tracker_driver = PSMDriver_GENERIC_WEBCAM;
                break;
            default:
                assert(0 && "unreachable");
            }

            TrackerInfo.tracker_focal_lengths = 
                { TrackerResponse.tracker_focal_lengths().x(), TrackerResponse.tracker_focal_lengths().y() };
            TrackerInfo.tracker_principal_point =
                { TrackerResponse.tracker_principal_point().x(), TrackerResponse.tracker_principal_point().y() };
            TrackerInfo.tracker_screen_dimensions =
                { TrackerResponse.tracker_screen_dimensions().x(), TrackerResponse.tracker_screen_dimensions().y() };

            TrackerInfo.tracker_hfov = TrackerResponse.tracker_hfov();
            TrackerInfo.tracker_vfov = TrackerResponse.tracker_vfov();

            TrackerInfo.tracker_znear = TrackerResponse.tracker_znear();
            TrackerInfo.tracker_zfar = TrackerResponse.tracker_zfar();

            TrackerInfo.tracker_k1 = TrackerResponse.tracker_k1();
            TrackerInfo.tracker_k2 = TrackerResponse.tracker_k2();
            TrackerInfo.tracker_k3 = TrackerResponse.tracker_k3();
            TrackerInfo.tracker_p1 = TrackerResponse.tracker_p1();
            TrackerInfo.tracker_p2 = TrackerResponse.tracker_p2();

            strncpy(TrackerInfo.device_path, TrackerResponse.device_path().c_str(), sizeof(TrackerInfo.device_path));
            strncpy(TrackerInfo.shared_memory_name, TrackerResponse.shared_memory_name().c_str(), sizeof(TrackerInfo.shared_memory_name));

            TrackerInfo.tracker_pose= protocol_pose_to_psmove_pose(TrackerResponse.tracker_pose());

            ++tracker_count;
        }

        // Record how many trackers we copied into the payload
        tracker_list->count = tracker_count;

		// Copy over the tracking space properties
		tracker_list->global_forward_degrees= response->result_tracker_list().global_forward_degrees();
    }

	void build_hmd_list_response_message(
		ResponsePtr response,
		PSMHmdList *hmd_list)
	{
		int src_hmd_count = 0;
		int dest_hmd_count = 0;

		// Copy the controller entries into the response payload
		while (src_hmd_count < response->result_hmd_list().hmd_entries_size()
			&& src_hmd_count < PSMOVESERVICE_MAX_HMD_COUNT)
		{
			const auto &HMDResponse = response->result_hmd_list().hmd_entries(src_hmd_count);

			// Convert the PSMoveProtocol HMD enum to the public ClientHMDView enum
			PSMHmdType hmdType;
			switch (HMDResponse.hmd_type())
			{
			case PSMoveProtocol::Morpheus:
				hmdType = PSMHmd_Morpheus;
				break;
			case PSMoveProtocol::VirtualHMD:
				hmdType = PSMHmd_Virtual;
				break;
			default:
				assert(0 && "unreachable");
				hmdType = PSMHmd_None;
			}

			// Add an entry to the hmd list
			hmd_list->hmd_type[dest_hmd_count] = hmdType;
			hmd_list->hmd_id[dest_hmd_count] = HMDResponse.hmd_id();
			++dest_hmd_count;
			++src_hmd_count;
		}

		// Record how many controllers we copied into the payload
		hmd_list->count = dest_hmd_count;
	}

	void build_tracking_space_response_message(
		ResponsePtr response,
		PSMTrackingSpace *tracking_space)
	{
		tracking_space->global_forward_degrees = response->result_tracking_space_settings().global_forward_degrees();
	}

private:
    IDataFrameListener *m_dataFrameListener;
    PSMResponseCallback m_callback;
    void *m_callback_userdata;
    t_request_context_map m_pending_requests;
    int m_next_request_id;

    // These vectors is used solely to keep the ref counted pointers to the 
    // request/response parameter data valid until the next update call.
    // The ClientAPI message queue contains raw void pointers to the request/response and event data.
    t_request_reference_cache m_request_reference_cache;
    t_response_reference_cache m_response_reference_cache;
};

//-- public methods -----
ClientRequestManager::ClientRequestManager(
    IDataFrameListener *dataFrameListener,
    PSMResponseCallback callback,
    void *userdata)
{
    m_implementation_ptr = new ClientRequestManagerImpl(dataFrameListener, callback, userdata);
}

ClientRequestManager::~ClientRequestManager()
{
    delete m_implementation_ptr;
}

void ClientRequestManager::flush_response_cache()
{
    m_implementation_ptr->flush_response_cache();
}

void ClientRequestManager::send_request(
    RequestPtr request)
{
    m_implementation_ptr->send_request(request);
}

void ClientRequestManager::handle_request_canceled(RequestPtr request)
{
    m_implementation_ptr->handle_request_canceled(request);
}

void ClientRequestManager::handle_response(ResponsePtr response)
{
    m_implementation_ptr->handle_response(response);
}

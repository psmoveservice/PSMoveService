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

class ClientRequestManagerImpl
{
public:
    ClientRequestManagerImpl(
        ClientPSMoveAPI::t_response_callback callback, 
        void *userdata)
        : m_callback(callback)
        , m_callback_userdata(userdata)
        , m_pending_requests()
        , m_next_request_id(0)
    {
    }

    void flush_response_cache()
    {
        // Drop all of the response references,
        // NOTE: std::vector::clear() calls the destructor on each element in the vector
        // This will decrement the last ref count to the parameter data, causing them to get cleaned up.
        m_response_reference_cache.clear();
    }

    void send_request(RequestPtr request)
    {
        RequestContext context;

        request->set_request_id(m_next_request_id);
        ++m_next_request_id;

        context.request= request;

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
            ClientPSMoveAPI::ResponseMessage response_message;

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
        ClientPSMoveAPI::ResponseMessage *out_response_message)
    {        
        memset(out_response_message, 0, sizeof(ClientPSMoveAPI::ResponseMessage));

        // Let the response know what request the result came from
        out_response_message->request_id = request->request_id();

        // Translate internal result codes into public facing result codes
        switch (response->result_code())
        {
        case PSMoveProtocol::Response_ResultCode_RESULT_OK:
            out_response_message->result_code = ClientPSMoveAPI::_clientPSMoveResultCode_ok;
            break;
        case PSMoveProtocol::Response_ResultCode_RESULT_ERROR:
            out_response_message->result_code = ClientPSMoveAPI::_clientPSMoveResultCode_error;
            break;
        case PSMoveProtocol::Response_ResultCode_RESULT_CANCELED:
            out_response_message->result_code = ClientPSMoveAPI::_clientPSMoveResultCode_canceled;
            break;
        default:
            assert(false && "Unknown response result code");
        }

        // Attach an opaque pointer to the PSMoveProtocol response.
        // Client code that has linked against PSMoveProtocol library
        // can access this pointer via the GET_PSMOVEPROTOCOL_RESPONSE() macro.
        out_response_message->opaque_response_handle = static_cast<const void*>(response.get());

        // The opaque response pointer will only remain valid until the next call to update()
        // at which time the response reference cache gets cleared out.
        m_response_reference_cache.push_back(response);

        // Write response specific data
        switch (response->type())
        {        
        case PSMoveProtocol::Response_ResponseType_CONTROLLER_LIST:
            build_controller_list_response_message(response, &out_response_message->payload.controller_list);
            out_response_message->payload_type = ClientPSMoveAPI::_responsePayloadType_ControllerCount;
            break;
        default:
            out_response_message->payload_type = ClientPSMoveAPI::_responsePayloadType_Empty;
            break;
        }
    }

    void build_controller_list_response_message(
        ResponsePtr response,
        ClientPSMoveAPI::ResponsePayload_ControllerList *controller_list)
    {
        int controller_count = 0;

        // Copy the controller entries into the response payload
        while (controller_count < response->result_controller_list().controllers_size()
                && controller_count < PSMOVESERVICE_MAX_CONTROLLER_COUNT)
        {
            const auto &ControllerResponse = response->result_controller_list().controllers(controller_count);

            // Convert the PSMoveProtocol controller enum to the public ClientControllerView enum
            ClientControllerView::eControllerType controllerType;
            switch (ControllerResponse.controller_type())
            {
            case PSMoveProtocol::PSMOVE:
                controllerType = ClientControllerView::PSMove;
                break;
            case PSMoveProtocol::PSNAVI:
                controllerType = ClientControllerView::PSNavi;
                break;
            default:
                assert(0 && "unreachable");
                controllerType = ClientControllerView::PSMove;
            }

            // Add an entry to the controller list
            controller_list->controller_type[controller_count] = controllerType;
            controller_list->controller_id[controller_count] = ControllerResponse.controller_id();
            ++controller_count;
        }

        // Record how many controllers we copied into the payload
        controller_list->count = controller_count;
    }

private:
    ClientPSMoveAPI::t_response_callback m_callback;
    void *m_callback_userdata;
    t_request_context_map m_pending_requests;
    int m_next_request_id;

    // This vector is used solely to keep the ref counted pointers to the 
    // response parameter data valid until the next update call.
    // The ClientAPI message queue contains raw void pointers to the response and event data.
    t_response_reference_cache m_response_reference_cache;
};

//-- public methods -----
ClientRequestManager::ClientRequestManager(ClientPSMoveAPI::t_response_callback callback, void *userdata)
{
    m_implementation_ptr = new ClientRequestManagerImpl(callback, userdata);
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
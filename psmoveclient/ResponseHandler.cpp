//-- includes -----
#include "ResponseHandler.h"
#include "PSMoveDataFrame.pb.h"
#include <cassert>

//-- definitions -----
struct ResponseContext
{
    RequestPtr request;
    ResponsePtr response;
};

//-- prototypes -----
static void handle_response__get_active_psmove_count(const ResponseContext &context);
static void handle_response__general(const ResponseContext &context);

//-- public methods -----
ResponseHandler::ResponseHandler() 
    : pending_requests()
{
}

void ResponseHandler::register_pending_request(RequestPtr request)
{
    // Requests should never be double registered
    assert(pending_requests.find(request->request_id()) == pending_requests.end());
    pending_requests.insert(t_id_request_pair(request->request_id(), request));
}

void ResponseHandler::pending_request_canceled(RequestPtr request)
{
    // Get the request awaiting completion
    t_request_map_iterator pending_request_entry= pending_requests.find(request->request_id());
    assert(pending_request_entry != pending_requests.end());

    // Remove the pending request from the map
    pending_requests.erase(pending_request_entry);

    // TODO: Notify any other systems about the cancellation based on request type?
}

void ResponseHandler::handle_response(ResponsePtr response)
{
    // Get the request awaiting completion
    t_request_map_iterator pending_request_entry= pending_requests.find(response->request_id());
    assert(pending_request_entry != pending_requests.end());

    // The context holds everything a handler needs to evaluate a response
    ResponseContext context;
    context.request= pending_request_entry->second;
    context.response= response;

    // Evaluate the response
    switch (response->type())
    {
        case PSMoveDataFrame::Response_ResponseType_ACTIVE_PSMOVE_COUNT:
            handle_response__get_active_psmove_count(context);
            break;
        case PSMoveDataFrame::Response_ResponseType_GENERAL_RESULT:
            handle_response__general(context);
            break;
        default:
            assert(0 && "Whoops, bad response!");
            break;
    }

    // Remove the pending request from the map
    pending_requests.erase(pending_request_entry);
}

//-- private request handlers -----
static void handle_response__get_active_psmove_count(
    const ResponseContext &context)
{
    // TODO
    int psmove_count= context.response->response_psmove_count().count();
}

static void handle_response__general(
    const ResponseContext &context)
{
    // TODO
    switch(context.response->response_general().code())
    {
    case PSMoveDataFrame::Response_ResultCode_RESULT_OK:
        break;
    case PSMoveDataFrame::Response_ResultCode_RESULT_ERROR:
        break;
    default:
        assert(0 && "Whoops, bad response!");
        break;
    }
}
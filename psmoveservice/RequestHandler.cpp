//-- includes -----
#include "RequestHandler.h"
#include "PSMoveDataFrame.pb.h"
#include <cassert>

//-- definitions -----
struct RequestContext
{
    RequestPtr request;
};

//-- prototypes -----
static void handle_request__get_active_psmove_count(const RequestContext &context, PSMoveDataFrame::Response *response);
static void handle_request__start_psmove_data_stream(const RequestContext &context, PSMoveDataFrame::Response *response);
static void handle_request__stop_psmove_data_stream(const RequestContext &context, PSMoveDataFrame::Response *response);
static void handle_request__set_rumble(const RequestContext &context, PSMoveDataFrame::Response *response);
static void handle_request__cycle_tracking_color(const RequestContext &context, PSMoveDataFrame::Response *response);
static void handle_request__reset_pose(const RequestContext &context, PSMoveDataFrame::Response *response);

//-- public methods -----
RequestHandler::RequestHandler()
{
}

ResponsePtr RequestHandler::handle_request(RequestPtr request)
{
    // The context holds everything a handler needs to evaluate a request
    RequestContext context;
    context.request= request;

    // All responses track which request they came from
    PSMoveDataFrame::Response *response= new PSMoveDataFrame::Response;
    response->set_request_id(request->request_id());

    switch (request->type())
    {
        case PSMoveDataFrame::Request_RequestType_GET_ACTIVE_PSMOVE_COUNT:
            handle_request__get_active_psmove_count(context, response);
            break;
        case PSMoveDataFrame::Request_RequestType_START_PSMOVE_DATA_STREAM:
            handle_request__start_psmove_data_stream(context, response);
            break;
        case PSMoveDataFrame::Request_RequestType_STOP_PSMOVE_DATA_STREAM:
            handle_request__stop_psmove_data_stream(context, response);
            break;
        case PSMoveDataFrame::Request_RequestType_SET_RUMBLE:
            handle_request__set_rumble(context, response);
            break;
        case PSMoveDataFrame::Request_RequestType_CYCLE_TRACKING_COLOR:
            handle_request__cycle_tracking_color(context, response);
            break;
        case PSMoveDataFrame::Request_RequestType_RESET_POSE:
            handle_request__reset_pose(context, response);
            break;
        default:
            assert(0 && "Whoops, bad request!");
            break;
    }

    return ResponsePtr(response);
}

//-- private request handlers -----
static void handle_request__get_active_psmove_count(
    const RequestContext &context, 
    PSMoveDataFrame::Response *response)
{
    // TODO
    response->mutable_response_psmove_count()->set_count(1);
}

static void handle_request__start_psmove_data_stream(
    const RequestContext &context, 
    PSMoveDataFrame::Response *response)
{
    // TODO
    response->set_result_code(PSMoveDataFrame::Response_ResultCode_RESULT_OK);
}

static void handle_request__stop_psmove_data_stream(
    const RequestContext &context,
    PSMoveDataFrame::Response *response)
{
    // TODO
    response->set_result_code(PSMoveDataFrame::Response_ResultCode_RESULT_OK);
}

static void handle_request__set_rumble(
    const RequestContext &context,
    PSMoveDataFrame::Response *response)
{
    // TODO
    response->set_result_code(PSMoveDataFrame::Response_ResultCode_RESULT_OK);
}

static void handle_request__cycle_tracking_color(
    const RequestContext &context, 
    PSMoveDataFrame::Response *response)
{
    // TODO
    response->set_result_code(PSMoveDataFrame::Response_ResultCode_RESULT_OK);
}

static void handle_request__reset_pose(
    const RequestContext &context, 
    PSMoveDataFrame::Response *response)
{
    // TODO
    response->set_result_code(PSMoveDataFrame::Response_ResultCode_RESULT_OK);
}
//-- includes -----
#include "ServerRequestHandler.h"
#include "ServerNetworkManager.h"
#include "PSMoveDataFrame.pb.h"
#include <cassert>
#include <bitset>
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

//-- pre-declarations -----
class ServerRequestHandlerImpl;
typedef boost::shared_ptr<ServerRequestHandlerImpl> ServerRequestHandlerImplPtr;

//-- definitions -----
struct RequestContext
{
    RequestPtr request;
};

//-- private implementation -----
class ServerRequestHandlerImpl
{
public:
    ServerRequestHandlerImpl() 
        : m_sequence_number(0)
        , m_active_controller_streams()
        , m_last_publish_time()
    {
    }

    void update()
    {
        boost::posix_time::ptime now= boost::posix_time::second_clock::local_time();
        boost::posix_time::time_duration diff = now - m_last_publish_time;

        //###bwalker $TODO This is a hacky way to simulate controller data frame updates
        if (diff.total_milliseconds() >= 1000)
        {
            for (size_t psmove_id= 0; psmove_id < k_max_psmove_controllers; ++psmove_id)
            {
                if (m_active_controller_streams.test(psmove_id))
                {
                    publish_controller_data_frame(psmove_id);
                }
            }

            m_last_publish_time= now;
        }
    }

    ResponsePtr handle_request(int connection_id, RequestPtr request)
    {
        // The context holds everything a handler needs to evaluate a request
        RequestContext context;
        context.request= request;

        // All responses track which request they came from
        PSMoveDataFrame::Response *response= new PSMoveDataFrame::Response;
        response->set_request_id(request->request_id());

        switch (request->type())
        {
            case PSMoveDataFrame::Request_RequestType_START_PSMOVE_DATA_STREAM:
                handle_request__start_psmove_data_stream(context, response);
                break;
            case PSMoveDataFrame::Request_RequestType_STOP_PSMOVE_DATA_STREAM:
                handle_request__stop_psmove_data_stream(context, response);
                break;
            case PSMoveDataFrame::Request_RequestType_SET_RUMBLE:
                handle_request__set_rumble(context, response);
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

protected:
    void publish_controller_data_frame(size_t psmove_id)
    {
        //###bwalker $TODO This is a hacky way to simulate controller data frame updates
        ControllerDataFramePtr data_frame(new PSMoveDataFrame::ControllerDataFrame);

        data_frame->set_psmove_id(static_cast<int>(psmove_id));
        data_frame->set_sequence_num(m_sequence_number);
        m_sequence_number++;
        
        data_frame->set_isconnected(true);
        data_frame->set_iscurrentlytracking(true);
        data_frame->set_istrackingenabled(true);

        data_frame->mutable_orientation()->set_w(1.f);
        data_frame->mutable_orientation()->set_x(0.f);
        data_frame->mutable_orientation()->set_y(0.f);
        data_frame->mutable_orientation()->set_z(0.f);

        data_frame->mutable_position()->set_x(0.f);
        data_frame->mutable_position()->set_y(0.f);
        data_frame->mutable_position()->set_z(0.f);

        data_frame->set_button_down_bitmask(0);
        data_frame->set_trigger_value(0);

        ServerNetworkManager::get_instance()->send_controller_data_frame(0, data_frame);
    }

    void handle_request__start_psmove_data_stream(
        const RequestContext &context, 
        PSMoveDataFrame::Response *response)
    {
        // TODO
        size_t psmove_id= static_cast<size_t>(context.request->request_start_psmove_data_stream().psmove_id());

        if (psmove_id >= 0 && psmove_id < k_max_psmove_controllers)
        {
            m_active_controller_streams.set(psmove_id, true);

            response->set_result_code(PSMoveDataFrame::Response_ResultCode_RESULT_OK);
        }
        else
        {
            response->set_result_code(PSMoveDataFrame::Response_ResultCode_RESULT_ERROR);
        }
    }

    void handle_request__stop_psmove_data_stream(
        const RequestContext &context,
        PSMoveDataFrame::Response *response)
    {
        // TODO
        size_t psmove_id= static_cast<size_t>(context.request->request_start_psmove_data_stream().psmove_id());

        if (psmove_id >= 0 && psmove_id < k_max_psmove_controllers)
        {
            m_active_controller_streams.set(psmove_id, false);

            response->set_result_code(PSMoveDataFrame::Response_ResultCode_RESULT_OK);
        }
        else
        {
            response->set_result_code(PSMoveDataFrame::Response_ResultCode_RESULT_ERROR);
        }
    }

    void handle_request__set_rumble(
        const RequestContext &context,
        PSMoveDataFrame::Response *response)
    {
        // TODO
        response->set_result_code(PSMoveDataFrame::Response_ResultCode_RESULT_OK);
    }

    void handle_request__reset_pose(
        const RequestContext &context, 
        PSMoveDataFrame::Response *response)
    {
        // TODO
        response->set_result_code(PSMoveDataFrame::Response_ResultCode_RESULT_OK);
    }

private:
    static const size_t k_max_psmove_controllers= 5;

    //###bwalker $TODO This is a hacky way to simulate controller data frame updates
    int m_sequence_number;
    std::bitset<k_max_psmove_controllers> m_active_controller_streams;
    boost::posix_time::ptime m_last_publish_time;
};

//-- public interface -----
ServerRequestHandler::ServerRequestHandler()
    : m_implementation_ptr(new ServerRequestHandlerImpl)
{

}

ServerRequestHandler::~ServerRequestHandler()
{
    delete m_implementation_ptr;
}

void ServerRequestHandler::update()
{
    m_implementation_ptr->update();
}

ResponsePtr ServerRequestHandler::handle_request(int connection_id, RequestPtr request)
{
    return m_implementation_ptr->handle_request(connection_id, request);
}
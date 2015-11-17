//-- includes -----
#include "ServerRequestHandler.h"
#include "ControllerManager.h"
#include "ServerNetworkManager.h"
#include "PSMoveProtocol.pb.h"
#include <cassert>
#include <bitset>
#include <map>
#include <boost/shared_ptr.hpp>

//-- pre-declarations -----
class ServerRequestHandlerImpl;
typedef boost::shared_ptr<ServerRequestHandlerImpl> ServerRequestHandlerImplPtr;

//-- definitions -----
struct RequestConnectionState
{
    int connection_id;
    std::bitset<k_max_psmove_controllers> active_controller_streams;

    RequestConnectionState()
        : connection_id(-1)
        , active_controller_streams()
    {
    }
};
typedef boost::shared_ptr<RequestConnectionState> RequestConnectionStatePtr;
typedef std::map<int, RequestConnectionStatePtr> t_connection_state_map;
typedef std::map<int, RequestConnectionStatePtr>::iterator t_connection_state_iter;
typedef std::pair<int, RequestConnectionStatePtr> t_id_connection_state_pair;

struct RequestContext
{
    RequestConnectionStatePtr connection_state;
    RequestPtr request;
};

//-- private implementation -----
class ServerRequestHandlerImpl
{
public:
    ServerRequestHandlerImpl(ControllerManager &controllerManager) 
        : m_controller_manager(controllerManager)
        , m_sequence_number(0)
        , m_connection_state_map()
    {
    }

    ResponsePtr handle_request(int connection_id, RequestPtr request)
    {
        // The context holds everything a handler needs to evaluate a request
        RequestContext context;
        context.request= request;
        context.connection_state= FindOrCreateConnectionState(connection_id);

        // All responses track which request they came from
        PSMoveProtocol::Response *response= new PSMoveProtocol::Response;
        response->set_request_id(request->request_id());

        switch (request->type())
        {
            case PSMoveProtocol::Request_RequestType_START_PSMOVE_DATA_STREAM:
                handle_request__start_psmove_data_stream(context, response);
                break;
            case PSMoveProtocol::Request_RequestType_STOP_PSMOVE_DATA_STREAM:
                handle_request__stop_psmove_data_stream(context, response);
                break;
            case PSMoveProtocol::Request_RequestType_SET_RUMBLE:
                handle_request__set_rumble(context, response);
                break;
            case PSMoveProtocol::Request_RequestType_RESET_POSE:
                handle_request__reset_pose(context, response);
                break;
            default:
                assert(0 && "Whoops, bad request!");
                break;
        }

        return ResponsePtr(response);
    }

    void handle_client_connection_stopped(int connection_id)
    {
        t_connection_state_iter iter= m_connection_state_map.find(connection_id);

        if (iter != m_connection_state_map.end())
        {
            m_connection_state_map.erase(iter);
        }
    }

    void publish_controller_data_frame(ControllerDataFramePtr data_frame)
    {
        int psmove_id= data_frame->psmove_id();

        // Notify any connections that care about the controller update
        for (t_connection_state_iter iter= m_connection_state_map.begin(); iter != m_connection_state_map.end(); ++iter)
        {
            int connection_id= iter->first;
            RequestConnectionStatePtr connection_state= iter->second;

            for (size_t psmove_id= 0; psmove_id < k_max_psmove_controllers; ++psmove_id)
            {
                if (connection_state->active_controller_streams.test(psmove_id))
                {
                    ServerNetworkManager::get_instance()->send_controller_data_frame(connection_id, data_frame);
                }
            }
        }
    }

protected:
    RequestConnectionStatePtr FindOrCreateConnectionState(int connection_id)
    {
        t_connection_state_iter iter= m_connection_state_map.find(connection_id);
        RequestConnectionStatePtr connection_state;

        if (iter == m_connection_state_map.end())
        {
            connection_state= RequestConnectionStatePtr(new RequestConnectionState());
            connection_state->connection_id= connection_id;

            m_connection_state_map.insert(t_id_connection_state_pair(connection_id, connection_state));
        }
        else
        {
            connection_state= iter->second;
        }

        return connection_state;
    }

    void handle_request__start_psmove_data_stream(
        const RequestContext &context, 
        PSMoveProtocol::Response *response)
    {
        size_t psmove_id= static_cast<size_t>(context.request->request_start_psmove_data_stream().psmove_id());

        if (psmove_id >= 0 && psmove_id < k_max_psmove_controllers)
        {
            // The controller manager will always publish updates regardless of who is listening.
            // All we have to do is keep track of which connections care about the updates.
            context.connection_state->active_controller_streams.set(psmove_id, true);

            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);
        }
        else
        {
            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
        }
    }

    void handle_request__stop_psmove_data_stream(
        const RequestContext &context,
        PSMoveProtocol::Response *response)
    {
        size_t psmove_id= static_cast<size_t>(context.request->request_start_psmove_data_stream().psmove_id());

        if (psmove_id >= 0 && psmove_id < k_max_psmove_controllers)
        {
            context.connection_state->active_controller_streams.set(psmove_id, false);

            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);
        }
        else
        {
            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
        }
    }

    void handle_request__set_rumble(
        const RequestContext &context,
        PSMoveProtocol::Response *response)
    {
        const int psmove_id= context.request->request_rumble().psmove_id();
        const int rumble_amount= context.request->request_rumble().rumble();

        if (m_controller_manager.setControllerRumble(psmove_id, rumble_amount))
        {
            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);
        }
        else
        {
            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
        }
    }

    void handle_request__reset_pose(
        const RequestContext &context, 
        PSMoveProtocol::Response *response)
    {
        const int psmove_id= context.request->reset_pose().psmove_id();

        if (m_controller_manager.resetPose(psmove_id))
        {
            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);
        }
        else
        {
            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
        }
    }

private:
    ControllerManager &m_controller_manager;
    int m_sequence_number;    
    t_connection_state_map m_connection_state_map;
};

//-- public interface -----
ServerRequestHandler *ServerRequestHandler::m_instance = NULL;

ServerRequestHandler::ServerRequestHandler(ControllerManager *controllerManager)
    : m_implementation_ptr(new ServerRequestHandlerImpl(*controllerManager))
{

}

ServerRequestHandler::~ServerRequestHandler()
{
    assert(m_instance == NULL);
    delete m_implementation_ptr;
}

bool ServerRequestHandler::startup()
{
    m_instance= this;
    return true;
}

void ServerRequestHandler::shutdown()
{
    m_instance= NULL;
}

ResponsePtr ServerRequestHandler::handle_request(int connection_id, RequestPtr request)
{
    return m_implementation_ptr->handle_request(connection_id, request);
}

void ServerRequestHandler::handle_client_connection_stopped(int connection_id)
{
    return m_implementation_ptr->handle_client_connection_stopped(connection_id);
}

void ServerRequestHandler::publish_controller_data_frame(ControllerDataFramePtr data_frame)
{
    return m_implementation_ptr->publish_controller_data_frame(data_frame);
}
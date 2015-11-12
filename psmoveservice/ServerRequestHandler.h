#ifndef SERVER_REQUEST_HANDLER_H
#define SERVER_REQUEST_HANDLER_H

#include <boost/shared_ptr.hpp>
#include "PSMoveDataFrame.pb.h"

//-- pre-declarations -----
namespace PSMoveDataFrame
{
    class ControllerDataFrame;
    class Request;
    class Response;
};
typedef boost::shared_ptr<PSMoveDataFrame::ControllerDataFrame> ControllerDataFramePtr;
typedef boost::shared_ptr<PSMoveDataFrame::Request> RequestPtr;
typedef boost::shared_ptr<PSMoveDataFrame::Response> ResponsePtr;

// -ServerRequestHandler-
class ServerRequestHandler 
{
public:
    ServerRequestHandler();
    virtual ~ServerRequestHandler();

    void update();
    ResponsePtr handle_request(int connection_id, RequestPtr request);
    void handle_client_connection_stopped(int connection_id);

private:
    // private implementation - same lifetime as the ServerRequestHandler
    class ServerRequestHandlerImpl *m_implementation_ptr;
};

#endif  // SERVER_REQUEST_HANDLER_H

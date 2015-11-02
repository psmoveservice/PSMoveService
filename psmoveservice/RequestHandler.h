#ifndef REQUEST_HANDLER_H
#define REQUEST_HANDLER_H

#include <boost/shared_ptr.hpp>
#include "PSMoveDataFrame.pb.h"

//-- pre-declarations -----
namespace PSMoveDataFrame
{
    class Request;
    class Response;
};
typedef boost::shared_ptr<PSMoveDataFrame::Request> RequestPtr;
typedef boost::shared_ptr<PSMoveDataFrame::Response> ResponsePtr;

// -RequestHandler-
class RequestHandler 
{
public:
    RequestHandler();

    ResponsePtr handle_request(RequestPtr request);
};

#endif  // REQUEST_HANDLER_H

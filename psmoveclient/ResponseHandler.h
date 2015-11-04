#ifndef RESPONSE_HANDLER_H
#define RESPONSE_HANDLER_H

//-- includes -----
#include "PSMoveDataFrame.pb.h"
#include <boost/shared_ptr.hpp>
#include <map>

//-- pre-declarations -----
namespace PSMoveDataFrame
{
    class Request;
    class Response;
};
typedef boost::shared_ptr<PSMoveDataFrame::Request> RequestPtr;
typedef boost::shared_ptr<PSMoveDataFrame::Response> ResponsePtr;

typedef std::map<int,RequestPtr> t_request_map;
typedef std::map<int,RequestPtr>::iterator t_request_map_iterator;
typedef std::pair<int,RequestPtr> t_id_request_pair;

//-- definitions -----
class ResponseHandler 
{
public:
    ResponseHandler();

    void register_pending_request(RequestPtr request);
    void pending_request_canceled(RequestPtr request);
    void handle_response(ResponsePtr response);

private:
    t_request_map pending_requests;
};

#endif  // RESPONSE_HANDLER_H

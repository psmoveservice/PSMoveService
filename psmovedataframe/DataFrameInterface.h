#ifndef DATAFRAME_INTERFACE_H
#define DATAFRAME_INTERFACE_H

//-- includes -----
#include <boost/shared_ptr.hpp>
#include <map>
#include <utility>

//-- pre-declarations -----
namespace PSMoveDataFrame
{
	class Request;
	class Response;
};
typedef boost::shared_ptr<PSMoveDataFrame::Request> RequestPtr;
typedef boost::shared_ptr<PSMoveDataFrame::Response> ResponsePtr;

typedef std::map<int, RequestPtr> t_request_map;
typedef std::map<int, RequestPtr>::iterator t_request_map_iterator;
typedef std::pair<int, RequestPtr> t_id_request_pair;

//-- interface -----
class IDataFrameEventListener
{
public:
	virtual void handle_request_canceled(RequestPtr request) = 0;
	virtual void handle_response(ResponsePtr response) = 0;
};

#endif  // DATAFRAME_INTERFACE_H

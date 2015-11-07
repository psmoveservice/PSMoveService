#ifndef DATAFRAME_INTERFACE_H
#define DATAFRAME_INTERFACE_H

//-- includes -----
#include <boost/shared_ptr.hpp>

//-- pre-declarations -----
namespace PSMoveDataFrame
{
	class Request;
	class Response;
};
typedef boost::shared_ptr<PSMoveDataFrame::Request> RequestPtr;
typedef boost::shared_ptr<PSMoveDataFrame::Response> ResponsePtr;

//-- interface -----
class IDataFrameEventListener
{
public:
	virtual void handle_request_canceled(RequestPtr request) = 0;
	virtual void handle_response(ResponsePtr response) = 0;
};

#endif  // DATAFRAME_INTERFACE_H

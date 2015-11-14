#ifndef DATAFRAME_INTERFACE_H
#define DATAFRAME_INTERFACE_H

//-- includes -----
#include <memory>

//-- constants -----
#define MAX_DATA_FRAME_MESSAGE_SIZE 24
 
//-- pre-declarations -----
namespace PSMoveDataFrame
{
    class ControllerDataFrame;
	class Request;
	class Response;
};
typedef std::shared_ptr<PSMoveDataFrame::ControllerDataFrame> ControllerDataFramePtr;
typedef std::shared_ptr<PSMoveDataFrame::Request> RequestPtr;
typedef std::shared_ptr<PSMoveDataFrame::Response> ResponsePtr;

//-- interface -----
class INotificationListener
{
public:
    virtual void handle_notification(ResponsePtr response) = 0;
};

class IDataFrameListener
{
public:
    virtual void handle_data_frame(ControllerDataFramePtr data_frame) = 0;
};

class IResponseListener
{
public:
	virtual void handle_request_canceled(RequestPtr request) = 0;
	virtual void handle_response(ResponsePtr response) = 0;
};

#endif  // DATAFRAME_INTERFACE_H

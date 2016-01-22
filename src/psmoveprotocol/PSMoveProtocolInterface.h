#ifndef PSMOVEPROTOCOL_INTERFACE_H
#define PSMOVEPROTOCOL_INTERFACE_H

//-- includes -----
#include <memory>

//-- constants -----
#define MAX_DATA_FRAME_MESSAGE_SIZE 128
 
//-- pre-declarations -----
namespace PSMoveProtocol
{
    class ControllerDataFrame;
	class Request;
	class Response;
};
typedef std::shared_ptr<PSMoveProtocol::ControllerDataFrame> ControllerDataFramePtr;
typedef std::shared_ptr<PSMoveProtocol::Request> RequestPtr;
typedef std::shared_ptr<PSMoveProtocol::Response> ResponsePtr;

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

#endif  // PSMOVEPROTOCOL_INTERFACE_H

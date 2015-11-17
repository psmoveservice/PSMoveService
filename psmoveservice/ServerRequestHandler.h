#ifndef SERVER_REQUEST_HANDLER_H
#define SERVER_REQUEST_HANDLER_H

// -- includes -----
#include "PSMoveProtocolInterface.h"

// -- pre-declarations -----
class ControllerManager;

// -- definitions -----
class ServerRequestHandler 
{
public:
    ServerRequestHandler(ControllerManager *controllerManager);
    virtual ~ServerRequestHandler();

    static ServerRequestHandler *get_instance() { return m_instance; }

    bool startup();
    void shutdown();

    ResponsePtr handle_request(int connection_id, RequestPtr request);
    void handle_client_connection_stopped(int connection_id);
    void publish_controller_data_frame(ControllerDataFramePtr data_frame);

private:
    // private implementation - same lifetime as the ServerRequestHandler
    class ServerRequestHandlerImpl *m_implementation_ptr;

    // Singleton instance of the class
    // Assigned in startup, cleared in teardown
    static ServerRequestHandler *m_instance;
};

#endif  // SERVER_REQUEST_HANDLER_H

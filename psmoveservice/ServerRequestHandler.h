#ifndef SERVER_REQUEST_HANDLER_H
#define SERVER_REQUEST_HANDLER_H

#include "DataFrameInterface.h"

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

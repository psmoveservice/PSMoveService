#ifndef REQUEST_MANAGER_H
#define REQUEST_MANAGER_H

//-- includes -----
#include "PSMoveProtocolInterface.h"
#include "ClientPSMoveAPI.h"
#include <functional>

//-- definitions -----
class ClientRequestManager : public IResponseListener
{
public:
    typedef std::function<void(ClientPSMoveAPI::eClientPSMoveResultCode)> response_callback;

    ClientRequestManager();
    virtual ~ClientRequestManager();

    void send_request(RequestPtr request, response_callback callback);

    virtual void handle_request_canceled(RequestPtr request) override;
    virtual void handle_response(ResponsePtr response) override;

private:
    // private implementation - same lifetime as the ClientRequestManager
    class ClientRequestManagerImpl *m_implementation_ptr;
};

#endif  // REQUEST_MANAGER_H

#ifndef REQUEST_MANAGER_H
#define REQUEST_MANAGER_H

//-- includes -----
#include "PSMoveDataFrame.pb.h"
#include "DataFrameInterface.h"

//-- definitions -----
class ClientRequestManager : public IDataFrameEventListener
{
public:
    ClientRequestManager();

    void register_pending_request(RequestPtr request);

    virtual void handle_request_canceled(RequestPtr request) override;
    virtual void handle_response(ResponsePtr response) override;

private:
    t_request_map pending_requests;
};

#endif  // REQUEST_MANAGER_H

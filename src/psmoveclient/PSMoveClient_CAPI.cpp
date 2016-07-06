#include <map>
#include "PSMoveClient_CAPI.h"
#include "ClientPSMoveAPI.h"
#include "PSMoveProtocol.pb.h"


PSMResult PSM_Initialize(const char* host, const char* port)
{
    std::string s_host(host);
    std::string s_port(port);
    e_log_severity_level log_level = _log_severity_level_info;
    PSMResult result = ClientPSMoveAPI::startup(s_host, s_port, log_level) ? PSMResult_Success : PSMResult_Error;
    return result;
}

PSMResult PSM_Shutdown()
{
    ClientPSMoveAPI::shutdown();
    return PSMResult_Success;
}

PSMResult findMessageOfType(PSMoveProtocol::Request_RequestType request_type, unsigned int timeout_msec)
{
    bool response_found = false;
    // Start the timer
    
    // Look to see if there's a pending response of the same request_type
    // if yes, retrieve the message
    // Look to see if there's a pending request of the same request_type
    // need access to `ClientPSMoveAPIImpl`'s `m_request_manager`'s `m_implementation_ptr`'s `m_pending_requests`
    // If no, create a new request
    while (!response_found)  // && timer.since < timeout_msec
    {
        ClientPSMoveAPI::update();
        // Sleep a tick.
        // search for responses, copy message to input
        
        //m_pending_requests
    }
    return response_found ? PSMResult_Success : PSMResult_Timeout;
}

PSMResult PSM_GetControllerList(PSMController** controllers, unsigned int timeout_msec)
{
    bool bReceived= false;
    ClientPSMoveAPI::ResponseMessage out_response;
    
    ClientPSMoveAPI::register_callback(
                      ClientPSMoveAPI::get_controller_list(),
                      // C++11 lambda that receives callback when response arrived
                      // &bReceived and &out_response are local variables that the lambda can change
                      [&bReceived, &out_response](const ClientPSMoveAPI::ResponseMessage *response, void *userdata) mutable {
                          bReceived= false;
                          out_response= *response;
                      },
                      nullptr);
    
    while (!bReceived)
    {
//        sleep(10);
        ClientPSMoveAPI::update();
    }
    
    //TODO: handle out_response
    
    return PSMResult_Error;
}

PSMResult PSM_RegisterAsControllerListener(PSMController *controller)
{
    
    return PSMResult_Error;
}

PSMResult PSM_StartControllerDataStream(PSMController *controller, unsigned int data_stream_flags)
{
    
    return PSMResult_Error;
}

PSMResult PSM_StopControllerDataStream(PSMController *controller)
{
    
    return PSMResult_Error;
}

PSMResult PSM_DeregisterAsControllerListener(PSMController *controller)

{
    
    return PSMResult_Error;
}

PSMResult PSM_SetControllerLEDColor(PSMController *controller, PSMTrackingColorType tracking_color)
{
    
    return PSMResult_Error;
}

PSMResult PSM_ResetControllerPose(PSMController *controller)
{
    
    return PSMResult_Error;
}

PSMResult PSM_UpdateController(PSMController *controller)
{
    
    return PSMResult_Error;
}

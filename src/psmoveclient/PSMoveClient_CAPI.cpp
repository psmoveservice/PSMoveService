// -- includes -----
#include <map>
#include "PSMoveClient_CAPI.h"
#include "ClientPSMoveAPI.h"
#include "ClientControllerView.h"
#include "MathUtility.h"
#include "ProtocolVersion.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include <assert.h>

// -- macros -----
#define IS_VALID_CONTROLLER_INDEX(x) ((x) >= 0 && (x) < PSMOVESERVICE_MAX_CONTROLLER_COUNT)
#define IS_VALID_TRACKER_INDEX(x) ((x) >= 0 && (x) < PSMOVESERVICE_MAX_TRACKER_COUNT)
#define IS_VALID_HMD_INDEX(x) ((x) >= 0 && (x) < PSMOVESERVICE_MAX_HMD_COUNT)

// -- constants ----
const PSMVector3f k_identity_gravity_calibration_direction= {0.f, 1.f, 0.f};

// -- private methods -----
static PSMResult blockUntilResponse(ClientPSMoveAPI::t_request_id req_id, int timeout_ms);
static void extractResponseMessage(const ClientPSMoveAPI::ResponseMessage *response_internal, PSMResponseMessage *response);
static void extractControllerState(const ClientControllerView *view, PSMController *controller);
static void extractTrackerState(const ClientTrackerView *view, PSMTracker *tracker);
static void extractHmdState(const ClientHMDView *view, PSMHeadMountedDisplay *hmd);
static void processEvent(ClientPSMoveAPI::EventMessage *event_message);

// -- private definitions -----
struct CallbackResultCapture
{
    bool bReceived= false;
    ClientPSMoveAPI::ResponseMessage out_response;
    
    static void response_callback(const ClientPSMoveAPI::ResponseMessage *response, void *userdata)
    {
        CallbackResultCapture *result= reinterpret_cast<CallbackResultCapture *>(userdata);
        
        result->out_response= *response;
        result->bReceived= true;
    }
};

struct CallbackResultAdapter
{   
    PSMResponseCallback callback;
    void *callback_userdata;
        
    static void response_callback(const ClientPSMoveAPI::ResponseMessage *response_internal, void *userdata)
    {
        CallbackResultAdapter *adapter= reinterpret_cast<CallbackResultAdapter *>(userdata);
        
        PSMResponseMessage response;
        extractResponseMessage(response_internal, &response);

        adapter->callback(&response, adapter->callback_userdata);

        delete adapter;
    }
};

class CallbackTimeout
{
public:
    CallbackTimeout(int timeout_ms) 
        : m_startTime(std::chrono::high_resolution_clock::now())
        , m_duration(static_cast<float>(timeout_ms))
    {        
    }

    bool HasElapsed() const
    {
        std::chrono::time_point<std::chrono::high_resolution_clock> now= std::chrono::high_resolution_clock::now();
        std::chrono::duration<float, std::milli> timeSinceStart= now - m_startTime;

        return timeSinceStart > m_duration;
    }

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> m_startTime;
    std::chrono::duration<float, std::milli> m_duration;
};

// -- private data ---
PSMController g_controllers[PSMOVESERVICE_MAX_CONTROLLER_COUNT];
PSMTracker g_trackers[PSMOVESERVICE_MAX_TRACKER_COUNT];
PSMHeadMountedDisplay g_HMDs[PSMOVESERVICE_MAX_HMD_COUNT];

ClientControllerView *g_controller_views[PSMOVESERVICE_MAX_CONTROLLER_COUNT];
ClientTrackerView *g_tracker_views[PSMOVESERVICE_MAX_TRACKER_COUNT];
ClientHMDView *g_hmd_views[PSMOVESERVICE_MAX_HMD_COUNT];

bool g_bIsConnected= false;
bool g_bHasConnectionStatusChanged= false;
bool g_bHasControllerListChanged= false;
bool g_bHasTrackerListChanged= false;
bool g_bHasHMDListChanged= false;

// -- public interface -----
const char* PSM_GetClientVersionString()
{
    const char *version_string= PSM_DETAILED_VERSION_STRING;

    return version_string;
}

bool PSM_GetIsInitialized()
{
	return ClientPSMoveAPI::has_started();
}

bool PSM_GetIsConnected()
{
    return g_bIsConnected;
}

bool PSM_HasConnectionStatusChanged()
{
    bool result= g_bHasConnectionStatusChanged;

    g_bHasConnectionStatusChanged= false;

    return result;
}

bool PSM_HasControllerListChanged()
{
    bool result= g_bHasControllerListChanged;

    g_bHasControllerListChanged= false;

    return result;
}

bool PSM_HasTrackerListChanged()
{
    bool result= g_bHasTrackerListChanged;

    g_bHasTrackerListChanged= false;

    return result;
}

bool PSM_HasHMDListChanged()
{
	bool result= g_bHasHMDListChanged;

	g_bHasHMDListChanged= false;

	return result;
}

PSMResult PSM_Initialize(const char* host, const char* port, int timeout_ms)
{
    PSMResult result = PSMResult_Error;

    if (PSM_InitializeAsync(host, port) == PSMResult_RequestSent)
    {
        CallbackTimeout timeout(timeout_ms);

        while (!PSM_HasConnectionStatusChanged() && !timeout.HasElapsed())
        {
            _PAUSE(10);
            PSM_Update();            
        }

        if (!timeout.HasElapsed())
        {
            result= PSM_GetIsConnected() ? PSMResult_Success : PSMResult_Error;
        }
        else
        {
            result= PSMResult_Timeout;
        }
    }

    return result;
}

PSMResult PSM_InitializeAsync(const char* host, const char* port)
{
    std::string s_host(host);
    std::string s_port(port);
    e_log_severity_level log_level = _log_severity_level_info;
    PSMResult result = ClientPSMoveAPI::startup(s_host, s_port, log_level) ? PSMResult_RequestSent : PSMResult_Error;

    memset(&g_controllers, 0, sizeof(PSMController)*PSMOVESERVICE_MAX_CONTROLLER_COUNT);
    memset(&g_controller_views, 0, sizeof(ClientControllerView *)*PSMOVESERVICE_MAX_CONTROLLER_COUNT);
    for (PSMControllerID controller_id= 0; controller_id < PSMOVESERVICE_MAX_CONTROLLER_COUNT; ++controller_id)    
    {
        g_controllers[controller_id].ControllerID= controller_id;
        g_controllers[controller_id].ControllerType= PSMController_None;
    }

    memset(g_trackers, 0, sizeof(PSMTracker)*PSMOVESERVICE_MAX_TRACKER_COUNT);
    memset(&g_tracker_views, 0, sizeof(ClientTrackerView *)*PSMOVESERVICE_MAX_TRACKER_COUNT);
    for (PSMTrackerID tracker_id= 0; tracker_id < PSMOVESERVICE_MAX_TRACKER_COUNT; ++tracker_id)    
    {
        g_trackers[tracker_id].tracker_info.tracker_id= tracker_id;
        g_trackers[tracker_id].tracker_info.tracker_type= PSMTracker_None;
    }

    memset(g_HMDs, 0, sizeof(PSMHeadMountedDisplay)*PSMOVESERVICE_MAX_HMD_COUNT);
    memset(&g_hmd_views, 0, sizeof(ClientHMDView *)*PSMOVESERVICE_MAX_HMD_COUNT);
    for (PSMHmdID hmd_id= 0; hmd_id < PSMOVESERVICE_MAX_HMD_COUNT; ++hmd_id)    
    {
        g_HMDs[hmd_id].HmdID= hmd_id;
        g_HMDs[hmd_id].HmdType= PSMHmd_None;
    }

    g_bIsConnected= false;
    g_bHasConnectionStatusChanged= false;
    g_bHasControllerListChanged= false;
    g_bHasTrackerListChanged= false;
	g_bHasHMDListChanged= false;

    return result;
}

PSMResult PSM_GetServiceVersionString(char *out_version_string, size_t max_version_string, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    CallbackResultCapture resultState;
    ClientPSMoveAPI::t_request_id request_id= ClientPSMoveAPI::get_service_version();
    ClientPSMoveAPI::register_callback(request_id,
                                       CallbackResultCapture::response_callback,
                                       &resultState);

    CallbackTimeout timeout(timeout_ms);
    
    while (!resultState.bReceived && !timeout.HasElapsed())
    {
        _PAUSE(10);
        PSM_Update();
    }
    
    if (timeout.HasElapsed())
    {
        ClientPSMoveAPI::cancel_callback(request_id);
        result= PSMResult_Timeout;
    }
    else if (resultState.out_response.result_code == ClientPSMoveAPI::_clientPSMoveResultCode_ok)
    {
        assert(resultState.out_response.payload_type == ClientPSMoveAPI::eResponsePayloadType::_responsePayloadType_HMDList);
        
        PSMResponseMessage response;
        extractResponseMessage(&resultState.out_response, &response);
		strncpy(out_version_string, response.payload.service_version.version_string, max_version_string);

        result= PSMResult_Success;
    }
    
    return result;
}

PSMResult PSM_GetServiceVersionStringAsync(PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    ClientPSMoveAPI::t_request_id req_id = ClientPSMoveAPI::get_service_version();

    if (out_request_id != nullptr)
    {
        *out_request_id= static_cast<PSMRequestID>(req_id);
    }

    result= (req_id > 0) ? PSMResult_RequestSent : PSMResult_Error;

    return result;
}

PSMResult PSM_Shutdown()
{
    for (PSMControllerID controller_id= 0; controller_id < PSMOVESERVICE_MAX_CONTROLLER_COUNT; ++controller_id)    
    {
        if (g_controller_views[controller_id] != nullptr)
        {
            ClientPSMoveAPI::free_controller_view(g_controller_views[controller_id]);
            g_controller_views[controller_id]= nullptr;
        }
    }

    for (PSMTrackerID tracker_id= 0; tracker_id < PSMOVESERVICE_MAX_TRACKER_COUNT; ++tracker_id)    
    {
        if (g_tracker_views[tracker_id] != nullptr)
        {
            ClientPSMoveAPI::free_tracker_view(g_tracker_views[tracker_id]);
            g_tracker_views[tracker_id]= nullptr;
        }
    }

    for (PSMHmdID hmd_id= 0; hmd_id < PSMOVESERVICE_MAX_HMD_COUNT; ++hmd_id)    
    {
        if (g_hmd_views[hmd_id] != nullptr)
        {
            ClientPSMoveAPI::free_hmd_view(g_hmd_views[hmd_id]);
            g_hmd_views[hmd_id]= nullptr;
        }
    }

    ClientPSMoveAPI::shutdown();

    g_bIsConnected= false;
    g_bHasConnectionStatusChanged= false;
    g_bHasControllerListChanged= false;
    g_bHasTrackerListChanged= false;
	g_bHasHMDListChanged= false;

    return PSMResult_Success;
}

PSMResult PSM_Update()
{
    PSMResult result = PSMResult_Error;

    if (PSM_UpdateNoPollMessages() == PSMResult_Success)
    {
        ClientPSMoveAPI::Message message;
        while(ClientPSMoveAPI::poll_next_message(&message, sizeof(message)))
        {
            switch(message.payload_type)
            {
                case ClientPSMoveAPI::eMessagePayloadType::_messagePayloadType_Event:
                    // Only handle events
                    processEvent(&message.event_data);
                    break;
                case ClientPSMoveAPI::eMessagePayloadType::_messagePayloadType_Response:
                    // Any response that didn't get a callback executed get dropped on the floor
                    CLIENT_LOG_INFO("PSM_Update") << "Dropping response to request id: " << message.response_data.request_id;
                    break;
                default:
                    assert(0 && "unreachable");
                    break;
            }
        }

        result= PSMResult_Success;
    }

    return result;
}

PSMResult PSM_UpdateNoPollMessages()
{
    PSMResult result= PSMResult_Error;

    if (ClientPSMoveAPI::has_started())
    {
        ClientPSMoveAPI::update();

        for (PSMControllerID controller_id= 0; controller_id < PSMOVESERVICE_MAX_CONTROLLER_COUNT; ++controller_id)
        {
            PSMController *controller= &g_controllers[controller_id];
            ClientControllerView * view = g_controller_views[controller_id];

            if (view != nullptr)
            {
                extractControllerState(view, controller);
            }
        }

        for (PSMTrackerID tracker_id= 0; tracker_id < PSMOVESERVICE_MAX_TRACKER_COUNT; ++tracker_id)
        {
            PSMTracker *tracker= &g_trackers[tracker_id];
            ClientTrackerView * view = g_tracker_views[tracker_id];

            if (view != nullptr)
            {
                extractTrackerState(view, tracker);
            }
        }

        for (PSMHmdID hmd_id= 0; hmd_id < PSMOVESERVICE_MAX_HMD_COUNT; ++hmd_id)
        {
            PSMHeadMountedDisplay *hmd= &g_HMDs[hmd_id];
            ClientHMDView * view = g_hmd_views[hmd_id];

            if (view != nullptr)
            {
                extractHmdState(view, hmd);
            }
        }

        result= PSMResult_Success;
    }

    return result;
}

PSMController *PSM_GetController(PSMControllerID controller_id)
{
    PSMController *controller= nullptr;

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        controller= &g_controllers[controller_id];
    }

    return controller;
}

PSMResult PSM_GetControllerListAsync(PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    ClientPSMoveAPI::t_request_id req_id = ClientPSMoveAPI::get_controller_list();

    if (out_request_id != nullptr)
    {
        *out_request_id= static_cast<PSMRequestID>(req_id);
    }

    result= (req_id > 0) ? PSMResult_RequestSent : PSMResult_Error;

    return result;
}

PSMResult PSM_GetControllerList(PSMControllerList *out_controller_list, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    CallbackResultCapture resultState;
    ClientPSMoveAPI::t_request_id request_id= ClientPSMoveAPI::get_controller_list();
    ClientPSMoveAPI::register_callback(request_id,
                                       CallbackResultCapture::response_callback,
                                       &resultState);

    CallbackTimeout timeout(timeout_ms);
    
    while (!resultState.bReceived && !timeout.HasElapsed())
    {
        _PAUSE(10);
        PSM_Update();
    }
    
    if (timeout.HasElapsed())
    {
        ClientPSMoveAPI::cancel_callback(request_id);
        result= PSMResult_Timeout;
    }
    else if (resultState.out_response.result_code == ClientPSMoveAPI::_clientPSMoveResultCode_ok)
    {
        assert(resultState.out_response.payload_type == ClientPSMoveAPI::eResponsePayloadType::_responsePayloadType_ControllerList);
        
        PSMResponseMessage response;
        extractResponseMessage(&resultState.out_response, &response);

        *out_controller_list= response.payload.controller_list;
        result= PSMResult_Success;
    }
    
    return result;
}

PSMResult PSM_AllocateControllerListener(PSMControllerID controller_id)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= &g_controllers[controller_id];

        if (g_controller_views[controller_id] == nullptr)
        {
            ClientControllerView *view= ClientPSMoveAPI::allocate_controller_view(controller_id);

            g_controller_views[controller_id]= view;

            controller->ControllerID= controller_id;
            controller->ControllerType = static_cast<PSMControllerType>(view->GetControllerViewType());
            controller->IsConnected = view->GetIsConnected();
            controller->InputSequenceNum = view->GetInputSequenceNum();
            controller->OutputSequenceNum = view->GetOutputSequenceNum();
            assert(controller->ListenerCount == 0);
        }

        ++controller->ListenerCount;
        result= PSMResult_Success;
    }
    
    return result;
}

PSMResult PSM_FreeControllerListener(PSMControllerID controller_id)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= &g_controllers[controller_id];

        assert(controller->ListenerCount > 0);
        --controller->ListenerCount;

        if (controller->ListenerCount <= 0)
        {
            ClientPSMoveAPI::free_controller_view(g_controller_views[controller_id]);
            g_controller_views[controller_id]= nullptr;

            memset(controller, 0, sizeof(PSMController));
            controller->ControllerID= controller_id;
            controller->ControllerType= PSMController_None;
            controller->ListenerCount= 0;
        }

        result= PSMResult_Success;
    }

    return result;
}

PSMResult PSM_StartControllerDataStreamAsync(PSMControllerID controller_id, unsigned int data_stream_flags, PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= &g_controllers[controller_id];

        if (controller->ListenerCount > 0)
        {
            ClientControllerView * view = g_controller_views[controller_id];
            assert(view != nullptr);

            ClientPSMoveAPI::t_request_id req_id = ClientPSMoveAPI::start_controller_data_stream(view, data_stream_flags);

            if (out_request_id != nullptr)
            {
                *out_request_id= static_cast<PSMRequestID>(req_id);
            }

            result= (req_id > 0) ? PSMResult_RequestSent : PSMResult_Error;
        }
    }

    return result;
}

PSMResult PSM_StartControllerDataStream(PSMControllerID controller_id, unsigned int data_stream_flags, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= &g_controllers[controller_id];

        if (controller->ListenerCount > 0)
        {
            ClientControllerView *view = g_controller_views[controller_id];
            assert (view != nullptr);

            result= blockUntilResponse(ClientPSMoveAPI::start_controller_data_stream(view, data_stream_flags), timeout_ms);
        
            if (result == PSMResult_Success)
            {
                extractControllerState(view, controller);
            }
        }
    }

    return result;
}

PSMResult PSM_StopControllerDataStreamAsync(PSMControllerID controller_id, PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= &g_controllers[controller_id];        

        if (controller->ListenerCount > 0)
        {
            ClientControllerView *view = g_controller_views[controller_id];
            assert(view != nullptr);

            ClientPSMoveAPI::t_request_id req_id = ClientPSMoveAPI::stop_controller_data_stream(view);

            if (out_request_id != nullptr)
            {
                *out_request_id= static_cast<PSMRequestID>(req_id);
            }

            result= (req_id > 0) ? PSMResult_RequestSent : PSMResult_Error;
        }
    }

    return result;
}

PSMResult PSM_StopControllerDataStream(PSMControllerID controller_id, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= &g_controllers[controller_id];        

        if (controller->ListenerCount > 0)
        {
            ClientControllerView *view = g_controller_views[controller_id];
            assert(view != nullptr);

            result= blockUntilResponse(ClientPSMoveAPI::stop_controller_data_stream(view), timeout_ms);
        }
    }

    return result;
}

PSMResult PSM_SetControllerLEDColorAsync(PSMControllerID controller_id, PSMTrackingColorType tracking_color, PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= &g_controllers[controller_id];

        if (controller->ListenerCount > 0)
        {
            ClientControllerView *view = g_controller_views[controller_id];
            assert(view != nullptr);

            ClientPSMoveAPI::t_request_id req_id = ClientPSMoveAPI::set_led_tracking_color(view, static_cast<PSMoveTrackingColorType>(tracking_color));

            if (out_request_id != nullptr)
            {
                *out_request_id= static_cast<PSMRequestID>(req_id);
            }

            result= (req_id > 0) ? PSMResult_RequestSent : PSMResult_Error;
        }
    }

    return result;
}

PSMResult PSM_SetControllerLEDTrackingColor(PSMControllerID controller_id, PSMTrackingColorType tracking_color, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= &g_controllers[controller_id];

        if (controller->ListenerCount > 0)
        {
            ClientControllerView *view = g_controller_views[controller_id];
            assert(view != nullptr);

            result= blockUntilResponse(
                ClientPSMoveAPI::set_led_tracking_color(view, static_cast<PSMoveTrackingColorType>(tracking_color)), 
                timeout_ms);
        }
    }

    return result;
}

PSMResult PSM_SetControllerLEDOverrideColor(PSMControllerID controller_id, unsigned char r, unsigned char g, unsigned char b)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= &g_controllers[controller_id];
        
        switch (controller->ControllerType)
        {
        case PSMController_Move:
            {
                PSMPSMove *psmove= &controller->ControllerState.PSMoveState;

                if (r != psmove->LED_r || g != psmove->LED_g || b != psmove->LED_b)
                {
                    psmove->LED_r = r;
                    psmove->LED_g = g;
                    psmove->LED_b = b;

                    psmove->bHasUnpublishedState = true;
                }
            } break;
        case PSMController_Navi:
            break;
        case PSMController_DualShock4:
            {
                PSMDualShock4 *ds4= &controller->ControllerState.PSDS4State;

                if (r != ds4->LED_r || g != ds4->LED_g || b != ds4->LED_b)
                {
                    ds4->LED_r = r;
                    ds4->LED_g = g;
                    ds4->LED_b = b;

                    ds4->bHasUnpublishedState = true;
                }
            } break;
        }

        if (controller->ListenerCount > 0)
        {
            ClientControllerView *view = g_controller_views[controller_id];
            assert(view != nullptr);

            view->SetLEDOverride(r, g, b);
        }

        result= PSMResult_Success;
    }

    return result;
}

PSMResult PSM_GetControllerRumble(PSMControllerID controller_id, PSMControllerRumbleChannel channel, float *out_rumbleFraction)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= &g_controllers[controller_id];
        unsigned char rumbleByte= 0;
        
        switch (controller->ControllerType)
        {
        case PSMController_Move:
            {
                rumbleByte= controller->ControllerState.PSMoveState.Rumble;
            } break;
        case PSMController_Navi:
            break;
        case PSMController_DualShock4:
            {                
                if (channel == PSMControllerRumbleChannel_Left)
                {
                    rumbleByte= controller->ControllerState.PSDS4State.BigRumble;
                }
                else if (channel == PSMControllerRumbleChannel_Right)
                {
                    rumbleByte= controller->ControllerState.PSDS4State.SmallRumble;
                }
            } break;
        }

        *out_rumbleFraction= clampf01(static_cast<float>(rumbleByte / 255.f));
        result= PSMResult_Success;
    }

    return result;
}

PSMResult PSM_SetControllerRumble(PSMControllerID controller_id, PSMControllerRumbleChannel channel, float rumbleFraction)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= &g_controllers[controller_id];
        const unsigned char rumbleByte= static_cast<unsigned char>(clampf01(rumbleFraction)*255.f);
        
        switch (controller->ControllerType)
        {
        case PSMController_Move:
            {
                PSMPSMove *psmove= &controller->ControllerState.PSMoveState;

                if (psmove->Rumble != rumbleByte)
                {
                    psmove->Rumble = rumbleByte;
                    psmove->bHasUnpublishedState = true;
                }

                if (controller->ListenerCount > 0)
                {
                    ClientControllerView *view = g_controller_views[controller_id];
                    assert(view != nullptr);

                    view->GetPSMoveViewMutable().SetRumble(rumbleFraction);
                }
            } break;
        case PSMController_Navi:
            break;
        case PSMController_DualShock4:
            {
                PSMDualShock4 *ds4= &controller->ControllerState.PSDS4State;
                
                if ((channel == PSMControllerRumbleChannel_All || channel == PSMControllerRumbleChannel_Left) &&
                    ds4->BigRumble != rumbleByte)
                {
                    ds4->BigRumble = rumbleByte;
                    ds4->bHasUnpublishedState = true;

                    if (controller->ListenerCount > 0)
                    {
                        ClientControllerView *view = g_controller_views[controller_id];
                        assert(view != nullptr);

                        view->GetPSDualShock4ViewMutable().SetBigRumble(rumbleFraction);
                    }
                }

                if ((channel == PSMControllerRumbleChannel_All || channel == PSMControllerRumbleChannel_Right) &&
                    ds4->SmallRumble != rumbleByte)
                {
                    ds4->SmallRumble = rumbleByte;
                    ds4->bHasUnpublishedState = true;

                    if (controller->ListenerCount > 0)
                    {
                        ClientControllerView *view = g_controller_views[controller_id];
                        assert(view != nullptr);

                        view->GetPSDualShock4ViewMutable().SetSmallRumble(rumbleFraction);
                    }
                }
            } break;
        }

        result= PSMResult_Success;
    }

    return result;
}

PSMResult PSM_GetControllerOrientation(PSMControllerID controller_id, PSMQuatf *out_orientation)
{
    PSMResult result= PSMResult_Error;
	assert(out_orientation);

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= &g_controllers[controller_id];
        
        switch (controller->ControllerType)
        {
        case PSMController_Move:
            {
				PSMPSMove State= controller->ControllerState.PSMoveState;
				*out_orientation = State.Pose.Orientation;

				result= State.bIsOrientationValid ? PSMResult_Success : PSMResult_Error;
            } break;
        case PSMController_Navi:
            break;
        case PSMController_DualShock4:
            {
				PSMDualShock4 State= controller->ControllerState.PSDS4State;
				*out_orientation = State.Pose.Orientation;

				result= State.bIsOrientationValid ? PSMResult_Success : PSMResult_Error;
            } break;
        }
    }

    return result;
}

PSMResult PSM_GetControllerPosition(PSMControllerID controller_id, PSMVector3f *out_position)
{
    PSMResult result= PSMResult_Error;
	assert(out_position);

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= &g_controllers[controller_id];
        
        switch (controller->ControllerType)
        {
        case PSMController_Move:
            {
				PSMPSMove State= controller->ControllerState.PSMoveState;
				*out_position = State.Pose.Position;

				result= State.bIsOrientationValid ? PSMResult_Success : PSMResult_Error;
            } break;
        case PSMController_Navi:
            break;
        case PSMController_DualShock4:
            {
				PSMDualShock4 State= controller->ControllerState.PSDS4State;
				*out_position = State.Pose.Position;

				result= State.bIsOrientationValid ? PSMResult_Success : PSMResult_Error;
            } break;
        }
    }

    return result;
}

PSMResult PSM_GetControllerPose(PSMControllerID controller_id, PSMPosef *out_pose)
{
    PSMResult result= PSMResult_Error;
	assert(out_pose);

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= &g_controllers[controller_id];
        
        switch (controller->ControllerType)
        {
        case PSMController_Move:
            {
				PSMPSMove State= controller->ControllerState.PSMoveState;
				*out_pose = State.Pose;

				result= (State.bIsOrientationValid && State.bIsPositionValid) ? PSMResult_Success : PSMResult_Error;
            } break;
        case PSMController_Navi:
            break;
        case PSMController_DualShock4:
            {
				PSMDualShock4 State= controller->ControllerState.PSDS4State;
				*out_pose = State.Pose;

				result= (State.bIsOrientationValid && State.bIsPositionValid) ? PSMResult_Success : PSMResult_Error;
            } break;
        }
    }

    return result;
}

PSMResult PSM_GetIsControllerStable(PSMControllerID controller_id, bool *out_is_stable)
{
    PSMResult result= PSMResult_Error;
	assert(out_is_stable);

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= &g_controllers[controller_id];
        
        switch (controller->ControllerType)
        {
        case PSMController_Move:
            {
				const float k_cosine_10_degrees = 0.984808f;

				// Get the direction the gravity vector should be pointing 
				// while the controller is in cradle pose.
				const PSMVector3f acceleration_direction = controller->ControllerState.PSMoveState.CalibratedSensorData.Accelerometer;
				float acceleration_magnitude;
				PSM_Vector3fNormalizeWithDefaultGetLength(&acceleration_direction, k_psm_float_vector3_zero, &acceleration_magnitude);

				*out_is_stable =
					is_nearly_equal(1.f, acceleration_magnitude, 0.1f) &&
					PSM_Vector3fDot(&k_identity_gravity_calibration_direction, &acceleration_direction) >= k_cosine_10_degrees;

				result= PSMResult_Success;
            } break;
        case PSMController_Navi:
            break;
        case PSMController_DualShock4:
            {
                PSMVector3f gyro= controller->ControllerState.PSDS4State.CalibratedSensorData.Gyroscope;
                
				const float k_gyro_noise= 10.f*k_degrees_to_radians; // noise threshold in rad/sec
				const float worst_rotation_rate = fabsf(PSM_Vector3fMaxValue(&gyro));

				*out_is_stable = worst_rotation_rate < k_gyro_noise;

				result= PSMResult_Success;
            } break;
        }
    }

    return result;
}

PSMResult PSM_GetIsControllerTracking(PSMControllerID controller_id, bool *out_is_tracking)
{
    PSMResult result= PSMResult_Error;
	assert(out_is_tracking);

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= &g_controllers[controller_id];
        
        switch (controller->ControllerType)
        {
        case PSMController_Move:
            {
				*out_is_tracking = controller->ControllerState.PSMoveState.bIsCurrentlyTracking;
				result= PSMResult_Success;
            } break;
        case PSMController_Navi:
            break;
        case PSMController_DualShock4:
            {
				*out_is_tracking = controller->ControllerState.PSDS4State.bIsCurrentlyTracking;
				result= PSMResult_Success;
            } break;
        }
    }

    return result;
}

PSMResult PSM_GetControllerPixelLocationOnTracker(PSMControllerID controller_id, PSMTrackerID tracker_id, PSMVector2f *outLocation)
{
	assert(outLocation);

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= &g_controllers[controller_id];
		PSMRawTrackerData *trackerData= nullptr;
        
        switch (controller->ControllerType)
        {
        case PSMController_Move:
			trackerData= &controller->ControllerState.PSMoveState.RawTrackerData;
            break;
        case PSMController_DualShock4:
			trackerData= &controller->ControllerState.PSDS4State.RawTrackerData;
            break;
        }

		if (trackerData != nullptr)
		{
			for (int listIndex = 0; listIndex < trackerData->ValidTrackerLocations; ++listIndex)
			{
				if (trackerData->TrackerIDs[listIndex] == tracker_id)
				{
					*outLocation = trackerData->ScreenLocations[listIndex];
					return PSMResult_Success;
				}
			}
		}
	}

    return PSMResult_Error;
}

PSMResult PSM_GetControllerPositionOnTracker(PSMControllerID controller_id, PSMTrackerID tracker_id, PSMVector3f *outPosition)
{
	assert(outPosition);

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= &g_controllers[controller_id];
		PSMRawTrackerData *trackerData= nullptr;
        
        switch (controller->ControllerType)
        {
        case PSMController_Move:
			trackerData= &controller->ControllerState.PSMoveState.RawTrackerData;
            break;
        case PSMController_DualShock4:
			trackerData= &controller->ControllerState.PSDS4State.RawTrackerData;
            break;
        }

		if (trackerData != nullptr)
		{
			for (int listIndex = 0; listIndex < trackerData->ValidTrackerLocations; ++listIndex)
			{
				if (trackerData->TrackerIDs[listIndex] == tracker_id)
				{
					*outPosition = trackerData->RelativePositionsCm[listIndex];
					return PSMResult_Success;
				}
			}
        }
	}

    return PSMResult_Error;
}

PSMResult PSM_GetControllerOrientationOnTracker(PSMControllerID controller_id, PSMTrackerID tracker_id, PSMQuatf *outOrientation)
{
	assert(outOrientation);

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= &g_controllers[controller_id];
		PSMRawTrackerData *trackerData= nullptr;
        
        switch (controller->ControllerType)
        {
        case PSMController_Move:
			trackerData= &controller->ControllerState.PSMoveState.RawTrackerData;
            break;
        case PSMController_DualShock4:
			trackerData= &controller->ControllerState.PSDS4State.RawTrackerData;
            break;
        }

		if (trackerData != nullptr)
		{
			for (int listIndex = 0; listIndex < trackerData->ValidTrackerLocations; ++listIndex)
			{
				if (trackerData->TrackerIDs[listIndex] == tracker_id)
				{
					*outOrientation = trackerData->RelativeOrientations[listIndex];
					return PSMResult_Success;
				}
			}
        }
	}

    return PSMResult_Error;
}

PSMResult PSM_GetControllerProjectionOnTracker(PSMControllerID controller_id, PSMTrackerID tracker_id, PSMTrackingProjection *outProjection)
{
	assert(outProjection);

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= &g_controllers[controller_id];
		PSMRawTrackerData *trackerData= nullptr;
        
        switch (controller->ControllerType)
        {
        case PSMController_Move:
			trackerData= &controller->ControllerState.PSMoveState.RawTrackerData;
            break;
        case PSMController_DualShock4:
			trackerData= &controller->ControllerState.PSDS4State.RawTrackerData;
            break;
        }

		if (trackerData != nullptr)
		{
			for (int listIndex = 0; listIndex < trackerData->ValidTrackerLocations; ++listIndex)
			{
				if (trackerData->TrackerIDs[listIndex] == tracker_id)
				{
					*outProjection = trackerData->TrackingProjections[listIndex];
					return PSMResult_Success;
				}
			}
        }
	}

    return PSMResult_Error;
}

PSMResult PSM_ResetControllerOrientationAsync(PSMControllerID controller_id, const PSMQuatf *q_pose, PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= &g_controllers[controller_id];

        if (controller->ListenerCount > 0)
        {
            ClientControllerView *view = g_controller_views[controller_id];
            assert(view != nullptr);

			PSMoveQuaternion q= PSMoveQuaternion::create(q_pose->w, q_pose->x, q_pose->y, q_pose->z);
            ClientPSMoveAPI::t_request_id req_id = ClientPSMoveAPI::reset_orientation(view, q);

            if (out_request_id != nullptr)
            {
                *out_request_id= static_cast<PSMRequestID>(req_id);
            }

            result= (req_id > 0) ? PSMResult_RequestSent : PSMResult_Error;
        }
    }

    return result;
}

PSMResult PSM_ResetControllerOrientation(PSMControllerID controller_id, PSMQuatf *q_pose, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= &g_controllers[controller_id];

        if (controller->ListenerCount > 0)
        {
            ClientControllerView *view = g_controller_views[controller_id];
            assert(view != nullptr);

            PSMoveQuaternion q= PSMoveQuaternion::create(q_pose->w, q_pose->x, q_pose->y, q_pose->z);

            result= blockUntilResponse(ClientPSMoveAPI::reset_orientation(view, q), timeout_ms);
        }
    }

    return result;
}

/// Tracker Pool
PSMTracker *PSM_GetTracker(PSMTrackerID tracker_id)
{
    PSMTracker *tracker= nullptr;

    if (IS_VALID_TRACKER_INDEX(tracker_id))
    {
        tracker= &g_trackers[tracker_id];
    }

    return tracker;
}

PSMResult PSM_AllocateTrackerListener(PSMTrackerID tracker_id, const PSMClientTrackerInfo *tracker_info)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_TRACKER_INDEX(tracker_id))
    {
        PSMTracker *tracker= &g_trackers[tracker_id];

        if (g_tracker_views[tracker_id] == nullptr)
        {
            ClientTrackerView *view= ClientPSMoveAPI::allocate_tracker_view(*reinterpret_cast<const ClientTrackerInfo *>(tracker_info));

            g_tracker_views[tracker_id]= view;

            tracker->tracker_info= *tracker_info;
            tracker->opaque_shared_memory_accesor= view->getSharedMemoryAccessor();
            extractTrackerState(view, tracker);

            assert(tracker->listener_count == 0);
        }

        ++tracker->listener_count;
        result= PSMResult_Success;
    }
    
    return result;
}

PSMResult PSM_FreeTrackerListener(PSMTrackerID tracker_id)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_TRACKER_INDEX(tracker_id))
    {
        PSMTracker *tracker= &g_trackers[tracker_id];

        assert(tracker->listener_count > 0);
        --tracker->listener_count;

        if (tracker->listener_count <= 0)
        {
            ClientPSMoveAPI::free_tracker_view(g_tracker_views[tracker_id]);
            g_tracker_views[tracker_id]= nullptr;

            memset(tracker, 0, sizeof(PSMTracker));
            tracker->tracker_info.tracker_id= tracker_id;
            tracker->tracker_info.tracker_type= PSMTracker_None;
            tracker->listener_count= 0;
        }

        result= PSMResult_Success;
    }

    return result;
}

/// Tracker State Methods
PSMResult PSM_GetTrackerIntrinsicMatrix(PSMTrackerID tracker_id, PSMMatrix3f *out_matrix)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_TRACKER_INDEX(tracker_id))
    {
		PSMTracker *tracker= &g_trackers[tracker_id];
		PSMClientTrackerInfo *tracker_info= &tracker->tracker_info;

		PSMVector3f basis_x= {tracker_info->tracker_focal_lengths.x, 0.f, tracker_info->tracker_principal_point.x};
		PSMVector3f basis_y= {0.f, tracker_info->tracker_focal_lengths.y, tracker_info->tracker_principal_point.y};
		PSMVector3f basis_z= {0.f, 0.f, 1.f};

		*out_matrix= PSM_Matrix3fCreate(&basis_x, &basis_y, &basis_z);
		result= PSMResult_Success;
	}

	return result;
}

/// Blocking Tracker Methods
PSMResult PSM_GetTrackerList(PSMTrackerList *out_tracker_list, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    CallbackResultCapture resultState;
    ClientPSMoveAPI::t_request_id request_id= ClientPSMoveAPI::get_tracker_list();
    ClientPSMoveAPI::register_callback(
                                       request_id,
                                       CallbackResultCapture::response_callback,
                                       &resultState);

    CallbackTimeout timeout(timeout_ms);
    
    while (!resultState.bReceived && !timeout.HasElapsed())
    {
        _PAUSE(10);
        PSM_Update();
    }
    
    if (timeout.HasElapsed())
    {
        ClientPSMoveAPI::cancel_callback(request_id);
        result= PSMResult_Timeout;
    }
    else if (resultState.out_response.result_code == ClientPSMoveAPI::_clientPSMoveResultCode_ok)
    {
        assert(resultState.out_response.payload_type == ClientPSMoveAPI::eResponsePayloadType::_responsePayloadType_TrackerList);
        
        PSMResponseMessage response;
        extractResponseMessage(&resultState.out_response, &response);

        *out_tracker_list= response.payload.tracker_list;
        result= PSMResult_Success;
    }
    
    return result;
}

PSMResult PSM_StartTrackerDataStream(PSMTrackerID tracker_id, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_TRACKER_INDEX(tracker_id))
    {
        ClientTrackerView *view = g_tracker_views[tracker_id];

        result= blockUntilResponse(ClientPSMoveAPI::start_tracker_data_stream(view), timeout_ms);
    }

    return result;
}

PSMResult PSM_StopTrackerDataStream(PSMTrackerID tracker_id, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_TRACKER_INDEX(tracker_id))
    {
        ClientTrackerView *view = g_tracker_views[tracker_id];

        result= blockUntilResponse(ClientPSMoveAPI::stop_tracker_data_stream(view), timeout_ms);
    }

    return result;
}

PSMResult PSM_GetTrackingSpaceSettings(PSMTrackingSpace *out_tracking_space, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    CallbackResultCapture resultState;
    ClientPSMoveAPI::t_request_id request_id= ClientPSMoveAPI::get_tracking_space_settings();
    ClientPSMoveAPI::register_callback(request_id,
                                       CallbackResultCapture::response_callback,
                                       &resultState);
    CallbackTimeout timeout(timeout_ms);
    
    while (!resultState.bReceived && !timeout.HasElapsed())
    {
        _PAUSE(10);
        PSM_Update();
    }
    
    if (timeout.HasElapsed())
    {
        ClientPSMoveAPI::cancel_callback(request_id);
        result= PSMResult_Timeout;
    }
    else if (resultState.out_response.result_code == ClientPSMoveAPI::_clientPSMoveResultCode_ok)
    {
        assert(resultState.out_response.payload_type == ClientPSMoveAPI::eResponsePayloadType::_responsePayloadType_ControllerList);
        
        PSMResponseMessage response;
        extractResponseMessage(&resultState.out_response, &response);

        *out_tracking_space= response.payload.tracking_space;
        result= PSMResult_Success;
    }
    
    return result;
}

PSMResult PSM_OpenTrackerVideoStream(PSMTrackerID tracker_id)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_TRACKER_INDEX(tracker_id))
    {
        ClientTrackerView *view = g_tracker_views[tracker_id];

        result= view->openVideoStream() ? PSMResult_Success : PSMResult_Error;
    }

    return result;
}

PSMResult PSM_PollTrackerVideoStream(PSMTrackerID tracker_id)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_TRACKER_INDEX(tracker_id))
    {
        ClientTrackerView *view = g_tracker_views[tracker_id];

        result= view->pollVideoStream() ? PSMResult_Success : PSMResult_NoData;
    }

    return result;
}

PSMResult PSM_CloseTrackerVideoStream(PSMTrackerID tracker_id)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_TRACKER_INDEX(tracker_id))
    {
        ClientTrackerView *view = g_tracker_views[tracker_id];

        view->closeVideoStream();
		result= PSMResult_Success;
    }

    return result;
}

PSMResult PSM_GetTrackerVideoFrameBuffer(PSMTrackerID tracker_id, const unsigned char **out_buffer)
{
    PSMResult result= PSMResult_Error;
	assert(out_buffer != nullptr);

    if (IS_VALID_TRACKER_INDEX(tracker_id))
    {
        ClientTrackerView *view = g_tracker_views[tracker_id];

        const unsigned char *buffer= view->getVideoFrameBuffer();
		if (buffer != nullptr)
		{
			*out_buffer= buffer;
			result= PSMResult_Success;
		}
    }

    return result;
}

PSMResult PSM_GetTrackerFrustum(PSMTrackerID tracker_id, PSMFrustum *out_frustum)
{
    PSMResult result= PSMResult_Error;
	assert(out_frustum != nullptr);

    if (IS_VALID_TRACKER_INDEX(tracker_id))
    {
		const PSMTracker *tracker= &g_trackers[tracker_id];
		const PSMClientTrackerInfo *tracker_info= &tracker->tracker_info;
		PSM_FrustumSetPose(out_frustum, &tracker_info->tracker_pose);

		// Convert the FOV angles to radians for rendering purposes
		out_frustum->HFOV = tracker_info->tracker_hfov * k_degrees_to_radians;
		out_frustum->VFOV = tracker_info->tracker_vfov * k_degrees_to_radians;
		out_frustum->zNear = tracker_info->tracker_znear;
		out_frustum->zFar = tracker_info->tracker_zfar;

		result= PSMResult_Success;
	}

    return result;
}

/// Async Tracker Methods
PSMResult PSM_GetTrackerListAsync(PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    ClientPSMoveAPI::t_request_id req_id = ClientPSMoveAPI::get_tracker_list();

    if (out_request_id != nullptr)
    {
        *out_request_id= static_cast<PSMRequestID>(req_id);
    }

    result= (req_id > 0) ? PSMResult_RequestSent : PSMResult_Error;

    return result;
}

PSMResult PSM_StartTrackerDataStreamAsync(PSMTrackerID tracker_id, PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_TRACKER_INDEX(tracker_id))
    {
        ClientTrackerView *view = g_tracker_views[tracker_id];

        if (view != nullptr)
        {
            ClientPSMoveAPI::t_request_id req_id = ClientPSMoveAPI::start_tracker_data_stream(view);

            if (out_request_id != nullptr)
            {
                *out_request_id= static_cast<PSMRequestID>(req_id);
            }

            result= (req_id > 0) ? PSMResult_RequestSent : PSMResult_Error;
        }
    }

    return result;
}

PSMResult PSM_StopTrackerDataStreamAsync(PSMTrackerID tracker_id, PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_TRACKER_INDEX(tracker_id))
    {
        ClientTrackerView *view = g_tracker_views[tracker_id];

        if (view != nullptr)
        {
            ClientPSMoveAPI::t_request_id req_id = ClientPSMoveAPI::stop_tracker_data_stream(view);

            if (out_request_id != nullptr)
            {
                *out_request_id= static_cast<PSMRequestID>(req_id);
            }

            result= (req_id > 0) ? PSMResult_RequestSent : PSMResult_Error;
        }
    }

    return result;
}

PSMResult PSM_GetTrackingSpaceSettingsAsync(PSMRequestID *out_request_id)
{
    ClientPSMoveAPI::t_request_id req_id = ClientPSMoveAPI::get_tracking_space_settings();

    if (out_request_id != nullptr)
    {
        *out_request_id= static_cast<PSMRequestID>(req_id);
    }

    return (req_id > 0) ? PSMResult_RequestSent : PSMResult_Error;
}

/// HMD Pool
PSMHeadMountedDisplay *PSM_GetHmd(PSMHmdID hmd_id)
{
    PSMHeadMountedDisplay *hmd= nullptr;

    if (IS_VALID_HMD_INDEX(hmd_id))
    {
        hmd= &g_HMDs[hmd_id];
    }

    return hmd;
}

PSMResult PSM_AllocateHmdListener(PSMHmdID hmd_id)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= &g_HMDs[hmd_id];

        if (g_hmd_views[hmd_id] == nullptr)
        {
            ClientHMDView *view= ClientPSMoveAPI::allocate_hmd_view(hmd_id);

            g_hmd_views[hmd_id]= view;

            hmd->HmdID= hmd_id;
            hmd->HmdType = static_cast<PSMHmdType>(view->GetHmdViewType());
            hmd->IsConnected = view->GetIsConnected();
            hmd->OutputSequenceNum = view->GetSequenceNum();
            assert(hmd->ListenerCount == 0);
        }

        ++hmd->ListenerCount;
        result= PSMResult_Success;
    }
    
    return result;
}

PSMResult PSM_FreeHmdListener(PSMHmdID hmd_id)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= &g_HMDs[hmd_id];

        assert(hmd->ListenerCount > 0);
        --hmd->ListenerCount;

        if (hmd->ListenerCount <= 0)
        {
            ClientPSMoveAPI::free_hmd_view(g_hmd_views[hmd_id]);
            g_hmd_views[hmd_id]= nullptr;

            memset(hmd, 0, sizeof(PSMHeadMountedDisplay));
            hmd->HmdID= hmd_id;
            hmd->HmdType= PSMHmd_None;
            hmd->ListenerCount= 0;
        }

        result= PSMResult_Success;
    }

    return result;

}

/// HMD State Methods
PSMResult PSM_GetHmdOrientation(PSMHmdID hmd_id, PSMQuatf *out_orientation)
{
    PSMResult result= PSMResult_Error;
	assert(out_orientation);

    if (IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= &g_HMDs[hmd_id];
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				PSMMorpheus State= hmd->HmdState.MorpheusState;
				*out_orientation = State.Pose.Orientation;

				result= State.bIsOrientationValid ? PSMResult_Success : PSMResult_Error;
            } break;
        }
    }

    return result;
}

PSMResult PSM_GetHmdPosition(PSMHmdID hmd_id, PSMVector3f *out_position)
{
    PSMResult result= PSMResult_Error;
	assert(out_position);

    if (IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= &g_HMDs[hmd_id];
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				PSMMorpheus State= hmd->HmdState.MorpheusState;
				*out_position = State.Pose.Position;

				result= State.bIsPositionValid ? PSMResult_Success : PSMResult_Error;
            } break;
        }
    }

    return result;
}

PSMResult PSM_GetHmdPose(PSMHmdID hmd_id, PSMPosef *out_pose)
{
    PSMResult result= PSMResult_Error;
	assert(out_pose);

    if (IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= &g_HMDs[hmd_id];
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				PSMMorpheus State= hmd->HmdState.MorpheusState;
				*out_pose = State.Pose;

				result= (State.bIsOrientationValid && State.bIsPositionValid) ? PSMResult_Success : PSMResult_Error;
            } break;
        }
    }

    return result;
}

PSMResult PSM_GetIsHmdStable(PSMHmdID hmd_id, bool *out_is_stable)
{
    PSMResult result= PSMResult_Error;
	assert(out_is_stable);

    if (IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= &g_HMDs[hmd_id];
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				const float k_cosine_20_degrees = 0.9396926f;

				// Get the direction the gravity vector should be pointing 
				// while the controller is in cradle pose.
				const PSMVector3f acceleration_direction = hmd->HmdState.MorpheusState.CalibratedSensorData.Accelerometer;
				float acceleration_magnitude;
				PSM_Vector3fNormalizeWithDefaultGetLength(&acceleration_direction, k_psm_float_vector3_zero, &acceleration_magnitude);

				*out_is_stable =
					is_nearly_equal(1.f, acceleration_magnitude, 0.1f) &&
					PSM_Vector3fDot(&k_identity_gravity_calibration_direction, &acceleration_direction) >= k_cosine_20_degrees;

				result= PSMResult_Success;
            } break;
        }
    }

    return result;
}

PSMResult PSM_GetIsHmdTracking(PSMHmdID hmd_id, bool *out_is_tracking)
{
    PSMResult result= PSMResult_Error;
	assert(out_is_tracking);

    if (IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= &g_HMDs[hmd_id];
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				*out_is_tracking = hmd->HmdState.MorpheusState.bIsCurrentlyTracking;
				result= PSMResult_Success;
            } break;
        }
    }

    return result;
}

PSMResult PSM_GetHmdPixelLocationOnTracker(PSMHmdID hmd_id, PSMTrackerID tracker_id, PSMVector2f *outLocation)
{
	assert(outLocation);

    if (IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= &g_HMDs[hmd_id];
		PSMRawTrackerData *trackerData= nullptr;
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				trackerData= &hmd->HmdState.MorpheusState.RawTrackerData;
            } break;
        }

		if (trackerData != nullptr)
		{
			for (int listIndex = 0; listIndex < trackerData->ValidTrackerLocations; ++listIndex)
			{
				if (trackerData->TrackerIDs[listIndex] == tracker_id)
				{
					*outLocation = trackerData->ScreenLocations[listIndex];
					return PSMResult_Success;
				}
			}
		}
	}

    return PSMResult_Error;
}

PSMResult PSM_GetHmdPositionOnTracker(PSMHmdID hmd_id, PSMTrackerID tracker_id, PSMVector3f *outPosition)
{
	assert(outPosition);

    if (IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= &g_HMDs[hmd_id];
		PSMRawTrackerData *trackerData= nullptr;
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				trackerData= &hmd->HmdState.MorpheusState.RawTrackerData;
            } break;
        }

		if (trackerData != nullptr)
		{
			for (int listIndex = 0; listIndex < trackerData->ValidTrackerLocations; ++listIndex)
			{
				if (trackerData->TrackerIDs[listIndex] == tracker_id)
				{
					*outPosition = trackerData->RelativePositionsCm[listIndex];
					return PSMResult_Success;
				}
			}
        }
	}

    return PSMResult_Error;
}

PSMResult PSM_GetHmdOrientationOnTracker(PSMHmdID hmd_id, PSMTrackerID tracker_id, PSMQuatf *outOrientation)
{
	assert(outOrientation);

    if (IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= &g_HMDs[hmd_id];
		PSMRawTrackerData *trackerData= nullptr;
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				trackerData= &hmd->HmdState.MorpheusState.RawTrackerData;
            } break;
        }

		if (trackerData != nullptr)
		{
			for (int listIndex = 0; listIndex < trackerData->ValidTrackerLocations; ++listIndex)
			{
				if (trackerData->TrackerIDs[listIndex] == tracker_id)
				{
					*outOrientation = trackerData->RelativeOrientations[listIndex];
					return PSMResult_Success;
				}
			}
        }
	}

    return PSMResult_Error;
}

PSMResult PSM_GetHmdProjectionOnTracker(PSMHmdID hmd_id, PSMTrackerID tracker_id, PSMTrackingProjection *outProjection)
{
	assert(outProjection);

    if (IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= &g_HMDs[hmd_id];
		PSMRawTrackerData *trackerData= nullptr;
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				trackerData= &hmd->HmdState.MorpheusState.RawTrackerData;
            } break;
        }

		if (trackerData != nullptr)
		{
			for (int listIndex = 0; listIndex < trackerData->ValidTrackerLocations; ++listIndex)
			{
				if (trackerData->TrackerIDs[listIndex] == tracker_id)
				{
					*outProjection = trackerData->TrackingProjections[listIndex];
					return PSMResult_Success;
				}
			}
        }
	}

    return PSMResult_Error;
}

/// Blocking HMD Methods
PSMResult PSM_GetHmdList(PSMHmdList *out_hmd_list, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    CallbackResultCapture resultState;
    ClientPSMoveAPI::t_request_id request_id= ClientPSMoveAPI::get_hmd_list();
    ClientPSMoveAPI::register_callback(request_id,
                                       CallbackResultCapture::response_callback,
                                       &resultState);

    CallbackTimeout timeout(timeout_ms);
    
    while (!resultState.bReceived && !timeout.HasElapsed())
    {
        _PAUSE(10);
        PSM_Update();
    }
    
    if (timeout.HasElapsed())
    {
        ClientPSMoveAPI::cancel_callback(request_id);
        result= PSMResult_Timeout;
    }
    else if (resultState.out_response.result_code == ClientPSMoveAPI::_clientPSMoveResultCode_ok)
    {
        assert(resultState.out_response.payload_type == ClientPSMoveAPI::eResponsePayloadType::_responsePayloadType_HMDList);
        
        PSMResponseMessage response;
        extractResponseMessage(&resultState.out_response, &response);

        *out_hmd_list= response.payload.hmd_list;
        result= PSMResult_Success;
    }
    
    return result;
}

PSMResult PSM_StartHmdDataStream(PSMHmdID hmd_id, unsigned int data_stream_flags, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= &g_HMDs[hmd_id];

		if (hmd->ListenerCount > 0)
		{
	        ClientHMDView *view = g_hmd_views[hmd_id];
			assert (view != nullptr);

			result= blockUntilResponse(ClientPSMoveAPI::start_hmd_data_stream(view, data_stream_flags), timeout_ms);
        
			if (result == PSMResult_Success)
			{
				extractHmdState(view, hmd);
			}
		}
    }

    return result;
}

PSMResult PSM_StopHmdDataStream(PSMHmdID hmd_id, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_HMD_INDEX(hmd_id))
    {
        ClientHMDView *view = g_hmd_views[hmd_id];
        assert (view != nullptr);

        result= blockUntilResponse(ClientPSMoveAPI::stop_hmd_data_stream(view), timeout_ms);
    }

    return result;
}

/// Async HMD Methods
PSMResult PSM_GetHmdListAsync(PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    ClientPSMoveAPI::t_request_id req_id = ClientPSMoveAPI::get_hmd_list();

    if (out_request_id != nullptr)
    {
        *out_request_id= static_cast<PSMRequestID>(req_id);
    }

    result= (req_id > 0) ? PSMResult_RequestSent : PSMResult_Error;

    return result;
}

PSMResult PSM_StartHmdDataStreamAsync(PSMHmdID hmd_id, unsigned int data_stream_flags, PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= &g_HMDs[hmd_id];

        if (hmd->ListenerCount > 0)
        {
            ClientHMDView * view = g_hmd_views[hmd_id];
            assert(view != nullptr);

            ClientPSMoveAPI::t_request_id req_id = ClientPSMoveAPI::start_hmd_data_stream(view, data_stream_flags);

            if (out_request_id != nullptr)
            {
                *out_request_id= static_cast<PSMRequestID>(req_id);
            }

            result= (req_id > 0) ? PSMResult_RequestSent : PSMResult_Error;
        }
    }

    return result;
}

PSMResult PSM_StopHmdDataStreamAsync(PSMHmdID hmd_id, PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    if (IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= &g_HMDs[hmd_id];        

        if (hmd->ListenerCount > 0)
        {
            ClientHMDView *view = g_hmd_views[hmd_id];
            assert(view != nullptr);

            ClientPSMoveAPI::t_request_id req_id = ClientPSMoveAPI::stop_hmd_data_stream(view);

            if (out_request_id != nullptr)
            {
                *out_request_id= static_cast<PSMRequestID>(req_id);
            }

            result= (req_id > 0) ? PSMResult_RequestSent : PSMResult_Error;
        }
    }

    return result;
}

PSMResult PSM_PollNextMessage(PSMMessage *message, size_t message_size)
{
    PSMResult result= PSMResult_Error;

    // Poll events queued up by the call to ClientPSMoveAPI::update()
    ClientPSMoveAPI::Message message_internal;
    if (ClientPSMoveAPI::poll_next_message(&message_internal, sizeof(message_internal)))
    {
        assert(sizeof(PSMMessage) == message_size);
        assert(message != nullptr);

        switch (message_internal.payload_type)
        {
        case ClientPSMoveAPI::_messagePayloadType_Response:
            {
                message->payload_type= PSMMessage::_messagePayloadType_Response;
                extractResponseMessage(&message_internal.response_data, &message->response_data);
            } break;
        case ClientPSMoveAPI::_messagePayloadType_Event:
            {
                // Update event flags before handling off the event
                processEvent(&message_internal.event_data);

                // Package up the event
                message->payload_type= PSMMessage::_messagePayloadType_Event;
                message->event_data.event_type= static_cast<PSMEventMessage::eEventType>(message_internal.event_data.event_type);
                message->event_data.event_data_handle= static_cast<PSMEventDataHandle>(message_internal.event_data.event_data_handle);

            } break;
        default:
            assert(0 && "unreachable");
        }

        result= PSMResult_Success;
    }

    return result;
}

PSMResult PSM_SendOpaqueRequest(PSMRequestHandle request_handle, PSMRequestID *out_request_id)
{
    ClientPSMoveAPI::t_request_id request_id= ClientPSMoveAPI::send_opaque_request(static_cast<ClientPSMoveAPI::t_request_handle>(request_handle));
    PSMResult result= PSMResult_Error;

    if (request_id != -1)
    {
        if (out_request_id != nullptr)
        {
            *out_request_id= static_cast<PSMRequestID>(request_id);
        }

        result= PSMResult_RequestSent;
    }

    return result;
}

PSMResult PSM_RegisterCallback(PSMRequestID request_id, PSMResponseCallback callback, void *callback_userdata)
{
    PSMResult result= PSMResult_Error;

    CallbackResultAdapter *adapter = new CallbackResultAdapter;
    adapter->callback= callback;
    adapter->callback_userdata= callback_userdata;

    if (ClientPSMoveAPI::register_callback(static_cast<ClientPSMoveAPI::t_request_id>(request_id), CallbackResultAdapter::response_callback, adapter))
    {
        result= PSMResult_Success;
    }
    else
    {
        delete adapter;
    }

    return result;
}

PSMResult PSM_CancelCallback(PSMRequestID request_id)
{
    return ClientPSMoveAPI::cancel_callback(static_cast<ClientPSMoveAPI::t_request_id>(request_id)) ? PSMResult_Success : PSMResult_Error;
}

PSMResult PSM_EatResponse(PSMRequestID request_id)
{
    return ClientPSMoveAPI::eat_response(static_cast<ClientPSMoveAPI::t_request_id>(request_id)) ? PSMResult_Success : PSMResult_Error;
}

// -- Async Messaging Helpers -----
static PSMResult blockUntilResponse(ClientPSMoveAPI::t_request_id req_id, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    if (req_id > 0)
    {
        CallbackResultCapture resultState;
        ClientPSMoveAPI::register_callback(req_id, CallbackResultCapture::response_callback, &resultState);
    
        CallbackTimeout timeout(timeout_ms);

        while (!resultState.bReceived && !timeout.HasElapsed())
        {
            _PAUSE(10);

            // Process responses, events and controller updates from the service
            PSM_Update();
        }

        if (timeout.HasElapsed())
        {
            ClientPSMoveAPI::cancel_callback(req_id);
            result= PSMResult_Timeout;
        }
        else if (resultState.out_response.result_code == ClientPSMoveAPI::_clientPSMoveResultCode_ok)
        {
            result= PSMResult_Success;
        }
    }

    return result;
}

static void extractResponseMessage(const ClientPSMoveAPI::ResponseMessage *response_internal, PSMResponseMessage *response)
{
    response->request_id= static_cast<PSMRequestID>(response_internal->request_id);

    switch(response_internal->result_code)
    {
    case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
        response->result_code= PSMResult_Success;
        break;
    case ClientPSMoveAPI::_clientPSMoveResultCode_error:
        response->result_code= PSMResult_Error;
        break;
    case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
        response->result_code= PSMResult_Canceled;
        break;
    default:
        assert(0 && "unreachable");
    }

    response->opaque_request_handle= static_cast<PSMResponseHandle>(response_internal->opaque_request_handle);
    response->opaque_response_handle= static_cast<PSMResponseHandle>(response_internal->opaque_response_handle);

    switch (response_internal->payload_type)
    {
    case ClientPSMoveAPI::_responsePayloadType_Empty:
        response->payload_type= PSMResponseMessage::_responsePayloadType_Empty;
        break;
    case ClientPSMoveAPI::_responsePayloadType_ServiceVersion:
        response->payload_type= PSMResponseMessage::_responsePayloadType_ServiceVersion;
        static_assert(sizeof(PSMServiceVersion) == sizeof(ClientPSMoveAPI::ResponsePayload_ServiceVersion), "Response payload types changed!");
        memcpy(&response->payload.service_version, &response_internal->payload.service_version, sizeof(PSMServiceVersion));
        break;
    case ClientPSMoveAPI::_responsePayloadType_ControllerList:
        response->payload_type= PSMResponseMessage::_responsePayloadType_ControllerList;
        static_assert(sizeof(PSMControllerList) == sizeof(ClientPSMoveAPI::ResponsePayload_ControllerList), "Response payload types changed!");
        memcpy(&response->payload.controller_list, &response_internal->payload.controller_list, sizeof(PSMControllerList));
        break;
    case ClientPSMoveAPI::_responsePayloadType_TrackerList:
        response->payload_type= PSMResponseMessage::_responsePayloadType_TrackerList;
        static_assert(sizeof(PSMTrackerList) == sizeof(ClientPSMoveAPI::ResponsePayload_TrackerList), "Response payload types changed!");
        memcpy(&response->payload.tracker_list, &response_internal->payload.tracker_list, sizeof(PSMTrackerList));
        break;
    case ClientPSMoveAPI::_responsePayloadType_TrackingSpace:
        response->payload_type= PSMResponseMessage::_responsePayloadType_TrackingSpace;
        static_assert(sizeof(PSMTrackingSpace) == sizeof(ClientPSMoveAPI::ResponsePayload_TrackingSpace), "Response payload types changed!");
        memcpy(&response->payload.tracking_space, &response_internal->payload.tracking_space, sizeof(PSMTrackingSpace));
        break;
	case ClientPSMoveAPI::_responsePayloadType_HMDList:
        response->payload_type= PSMResponseMessage::_responsePayloadType_HmdList;
        static_assert(sizeof(PSMHmdList) == sizeof(ClientPSMoveAPI::ResponsePayload_HMDList), "Response payload types changed!");
        memcpy(&response->payload.hmd_list, &response_internal->payload.hmd_list, sizeof(PSMHmdList));
        break;
    default:
        assert(0 && "unreachable");
    }
}

static void extractControllerState(const ClientControllerView *view, PSMController *controller)
{    
    // Set the generic items
    controller->bValid = view->IsValid();
    controller->ControllerType = static_cast<PSMControllerType>(view->GetControllerViewType());
    controller->InputSequenceNum = view->GetInputSequenceNum();
    controller->OutputSequenceNum = view->GetOutputSequenceNum();
    controller->IsConnected = view->GetIsConnected();
//    controller->DataFrameLastReceivedTime =
    controller->DataFrameAverageFPS = view->GetDataFrameFPS();
    
    // Have to declare some variables in case they are used in the switches
    ClientPSMoveView psmview;
	ClientPSNaviView psnview;
	ClientPSDualShock4View ds4view;
    PSMovePose pose;
    PSMovePhysicsData phydat;
    PSMoveRawSensorData psm_raw_sens;
	PSMoveCalibratedSensorData psm_calib_sens;
	PSDualShock4RawSensorData ds4_raw_sens;
	PSDualShock4CalibratedSensorData ds4_calib_sens;
    PSMoveRawTrackerData raw_track;
    
    switch (view->GetControllerViewType()) {
        case ClientControllerView::eControllerType::PSMove:
            psmview = view->GetPSMoveView();
            
            // Copy to PSMController
//            char                    DevicePath[256];
//            char                    DeviceSerial[128];
//            char                    AssignedHostSerial[128];
//            PSMBool                 PairedToHost;
//            PSMConnectionType       ConnectionType;
            controller->ControllerState.PSMoveState.bHasValidHardwareCalibration = psmview.GetHasValidHardwareCalibration();
            controller->ControllerState.PSMoveState.bIsTrackingEnabled = psmview.GetIsTrackingEnabled();
            controller->ControllerState.PSMoveState.bIsCurrentlyTracking = psmview.GetIsCurrentlyTracking();
			controller->ControllerState.PSMoveState.bIsOrientationValid = psmview.GetIsOrientationValid();
			controller->ControllerState.PSMoveState.bIsPositionValid = psmview.GetIsPositionValid();
            controller->ControllerState.PSMoveState.bHasUnpublishedState = psmview.GetHasUnpublishedState();
//            is_stable = psmview.GetIsStableAndAlignedWithGravity();
//            PSMTrackingColorType    TrackingColorType;
            
            pose = psmview.GetPose();
            controller->ControllerState.PSMoveState.Pose.Position = {pose.Position.x, pose.Position.y, pose.Position.z};
            controller->ControllerState.PSMoveState.Pose.Orientation = PSM_QuatfCreate(pose.Orientation.w, pose.Orientation.x, pose.Orientation.y, pose.Orientation.z);
            
            phydat = psmview.GetPhysicsData();
            controller->ControllerState.PSMoveState.PhysicsData.LinearAccelerationCmPerSecSqr = {phydat.AccelerationCmPerSecSqr.i, phydat.AccelerationCmPerSecSqr.j, phydat.AccelerationCmPerSecSqr.k};
            controller->ControllerState.PSMoveState.PhysicsData.LinearVelocityCmPerSec = {phydat.VelocityCmPerSec.i, phydat.VelocityCmPerSec.j, phydat.VelocityCmPerSec.k};
            controller->ControllerState.PSMoveState.PhysicsData.AngularAccelerationRadPerSecSqr = {phydat.AngularAccelerationRadPerSecSqr.i, phydat.AngularAccelerationRadPerSecSqr.j, phydat.AngularAccelerationRadPerSecSqr.k};
            controller->ControllerState.PSMoveState.PhysicsData.AngularVelocityRadPerSec = {phydat.AngularVelocityRadPerSec.i, phydat.AngularVelocityRadPerSec.j, phydat.AngularVelocityRadPerSec.k};
            
            psm_raw_sens = psmview.GetRawSensorData();
            controller->ControllerState.PSMoveState.RawSensorData.Accelerometer = {psm_raw_sens.Accelerometer.i, psm_raw_sens.Accelerometer.j, psm_raw_sens.Accelerometer.k};
            controller->ControllerState.PSMoveState.RawSensorData.Gyroscope = {psm_raw_sens.Gyroscope.i, psm_raw_sens.Gyroscope.j, psm_raw_sens.Gyroscope.k};
            controller->ControllerState.PSMoveState.RawSensorData.Magnetometer = {psm_raw_sens.Magnetometer.i, psm_raw_sens.Magnetometer.j, psm_raw_sens.Magnetometer.k};

            psm_calib_sens = psmview.GetCalibratedSensorData();
            controller->ControllerState.PSMoveState.CalibratedSensorData.Accelerometer = {psm_calib_sens.Accelerometer.i, psm_calib_sens.Accelerometer.j, psm_calib_sens.Accelerometer.k};
            controller->ControllerState.PSMoveState.CalibratedSensorData.Gyroscope = {psm_calib_sens.Gyroscope.i, psm_calib_sens.Gyroscope.j, psm_calib_sens.Gyroscope.k};
            controller->ControllerState.PSMoveState.CalibratedSensorData.Magnetometer = {psm_calib_sens.Magnetometer.i, psm_calib_sens.Magnetometer.j, psm_calib_sens.Magnetometer.k};
            
            raw_track = psmview.GetRawTrackerData();
            std::copy(std::begin(raw_track.TrackerIDs), std::end(raw_track.TrackerIDs), std::begin(controller->ControllerState.PSMoveState.RawTrackerData.TrackerIDs));
            controller->ControllerState.PSMoveState.RawTrackerData.ValidTrackerLocations = raw_track.ValidTrackerLocations;
            for(auto & track_id : raw_track.TrackerIDs)
            {
                controller->ControllerState.PSMoveState.RawTrackerData.ScreenLocations[track_id] = {
                    raw_track.ScreenLocations[track_id].x, raw_track.ScreenLocations[track_id].y
                };
                controller->ControllerState.PSMoveState.RawTrackerData.RelativePositionsCm[track_id] = {
                    raw_track.RelativePositionsCm[track_id].x, raw_track.RelativePositionsCm[track_id].y, raw_track.RelativePositionsCm[track_id].z
                };
                controller->ControllerState.PSMoveState.RawTrackerData.RelativeOrientations[track_id] = {
                    raw_track.RelativeOrientations[track_id].x, raw_track.RelativeOrientations[track_id].y, 
                    raw_track.RelativeOrientations[track_id].z, raw_track.RelativeOrientations[track_id].w
                };
                controller->ControllerState.PSMoveState.RawTrackerData.TrackingProjections[track_id].shape_type = PSMTrackingProjection::eShapeType::PSMShape_Ellipse;
                controller->ControllerState.PSMoveState.RawTrackerData.TrackingProjections[track_id].shape.ellipse.center = {
                    raw_track.TrackingProjections[track_id].shape.ellipse.center.x, raw_track.TrackingProjections[track_id].shape.ellipse.center.y
                };
                controller->ControllerState.PSMoveState.RawTrackerData.TrackingProjections[track_id].shape.ellipse.angle = raw_track.TrackingProjections[track_id].shape.ellipse.angle;
                controller->ControllerState.PSMoveState.RawTrackerData.TrackingProjections[track_id].shape.ellipse.half_x_extent = raw_track.TrackingProjections[track_id].shape.ellipse.half_x_extent;
                controller->ControllerState.PSMoveState.RawTrackerData.TrackingProjections[track_id].shape.ellipse.half_y_extent = raw_track.TrackingProjections[track_id].shape.ellipse.half_y_extent;
            }
            controller->ControllerState.PSMoveState.RawTrackerData.MulticamPositionCm = 
                {raw_track.MulticamPositionCm.x, raw_track.MulticamPositionCm.y, raw_track.MulticamPositionCm.z};
            controller->ControllerState.PSMoveState.RawTrackerData.MulticamOrientation = 
                {raw_track.MulticamOrientation.w, raw_track.MulticamOrientation.x, raw_track.MulticamOrientation.y, raw_track.MulticamOrientation.z};
            controller->ControllerState.PSMoveState.RawTrackerData.bMulticamOrientationValid= raw_track.bMulticamOrientationValid;
            controller->ControllerState.PSMoveState.RawTrackerData.bMulticamPositionValid= raw_track.bMulticamPositionValid;
            
            controller->ControllerState.PSMoveState.TriangleButton = static_cast<PSMButtonState>(psmview.GetButtonTriangle());
            controller->ControllerState.PSMoveState.CircleButton = static_cast<PSMButtonState>(psmview.GetButtonCircle());
            controller->ControllerState.PSMoveState.CrossButton = static_cast<PSMButtonState>(psmview.GetButtonCross());
            controller->ControllerState.PSMoveState.SquareButton = static_cast<PSMButtonState>(psmview.GetButtonSquare());
            controller->ControllerState.PSMoveState.SelectButton = static_cast<PSMButtonState>(psmview.GetButtonSelect());
            controller->ControllerState.PSMoveState.StartButton = static_cast<PSMButtonState>(psmview.GetButtonStart());
            controller->ControllerState.PSMoveState.PSButton = static_cast<PSMButtonState>(psmview.GetButtonPS());
            controller->ControllerState.PSMoveState.MoveButton = static_cast<PSMButtonState>(psmview.GetButtonMove());
            controller->ControllerState.PSMoveState.TriggerButton = static_cast<PSMButtonState>(psmview.GetButtonTrigger());
            controller->ControllerState.PSMoveState.BatteryValue = static_cast<PSMBatteryState>(psmview.GetBatteryValue());
            controller->ControllerState.PSMoveState.TriggerValue = static_cast<unsigned char>(psmview.GetTriggerValue());
            controller->ControllerState.PSMoveState.Rumble = static_cast<unsigned char>(psmview.GetRumble() * 255.f);
            break;
            
        case ClientControllerView::eControllerType::PSNavi:		
			psnview = view->GetPSNaviView();	

			controller->ControllerState.PSNaviState.L1Button = static_cast<PSMButtonState>(psnview.GetButtonL1());
			controller->ControllerState.PSNaviState.L2Button = static_cast<PSMButtonState>(psnview.GetButtonL2());
			controller->ControllerState.PSNaviState.L3Button = static_cast<PSMButtonState>(psnview.GetButtonL3());
			controller->ControllerState.PSNaviState.CircleButton = static_cast<PSMButtonState>(psnview.GetButtonCircle());
			controller->ControllerState.PSNaviState.CrossButton = static_cast<PSMButtonState>(psnview.GetButtonCross());
			controller->ControllerState.PSNaviState.PSButton = static_cast<PSMButtonState>(psnview.GetButtonPS());
			controller->ControllerState.PSNaviState.TriggerButton = static_cast<PSMButtonState>(psnview.GetButtonTrigger());
			controller->ControllerState.PSNaviState.DPadUpButton = static_cast<PSMButtonState>(psnview.GetButtonDPadUp());
			controller->ControllerState.PSNaviState.DPadRightButton = static_cast<PSMButtonState>(psnview.GetButtonDPadRight());
			controller->ControllerState.PSNaviState.DPadDownButton = static_cast<PSMButtonState>(psnview.GetButtonDPadDown());
			controller->ControllerState.PSNaviState.DPadLeftButton = static_cast<PSMButtonState>(psnview.GetButtonDPadLeft());
			controller->ControllerState.PSNaviState.TriggerValue = static_cast<unsigned char>(psnview.GetTriggerValue());
			controller->ControllerState.PSNaviState.Stick_XAxis= psnview.GetStickXAxis();
			controller->ControllerState.PSNaviState.Stick_YAxis= psnview.GetStickYAxis();;
            break;

        case ClientControllerView::eControllerType::PSDualShock4:
			ds4view = view->GetPSDualShock4View();
            // Copy to PSMController
//            char                    DevicePath[256];
//            char                    DeviceSerial[128];
//            char                    AssignedHostSerial[128];
//            PSMBool                 PairedToHost;
//            PSMConnectionType       ConnectionType;
            controller->ControllerState.PSDS4State.bHasValidHardwareCalibration = ds4view.GetHasValidHardwareCalibration();
            controller->ControllerState.PSDS4State.bIsTrackingEnabled = ds4view.GetIsTrackingEnabled();
            controller->ControllerState.PSDS4State.bIsCurrentlyTracking = ds4view.GetIsCurrentlyTracking();
			controller->ControllerState.PSDS4State.bIsOrientationValid = psmview.GetIsOrientationValid();
			controller->ControllerState.PSDS4State.bIsPositionValid = psmview.GetIsPositionValid();
            controller->ControllerState.PSDS4State.bHasUnpublishedState = ds4view.GetHasUnpublishedState();
//            is_stable = ds4view.GetIsStableAndAlignedWithGravity();
//            PSMTrackingColorType    TrackingColorType;
            
            pose = ds4view.GetPose();
            controller->ControllerState.PSDS4State.Pose.Position = {pose.Position.x, pose.Position.y, pose.Position.z};
            controller->ControllerState.PSDS4State.Pose.Orientation = PSM_QuatfCreate(pose.Orientation.w, pose.Orientation.x, pose.Orientation.y, pose.Orientation.z);
            
            phydat = ds4view.GetPhysicsData();
            controller->ControllerState.PSDS4State.PhysicsData.LinearAccelerationCmPerSecSqr = {phydat.AccelerationCmPerSecSqr.i, phydat.AccelerationCmPerSecSqr.j, phydat.AccelerationCmPerSecSqr.k};
            controller->ControllerState.PSDS4State.PhysicsData.LinearVelocityCmPerSec = {phydat.VelocityCmPerSec.i, phydat.VelocityCmPerSec.j, phydat.VelocityCmPerSec.k};
            controller->ControllerState.PSDS4State.PhysicsData.AngularAccelerationRadPerSecSqr = {phydat.AngularAccelerationRadPerSecSqr.i, phydat.AngularAccelerationRadPerSecSqr.j, phydat.AngularAccelerationRadPerSecSqr.k};
            controller->ControllerState.PSDS4State.PhysicsData.AngularVelocityRadPerSec = {phydat.AngularVelocityRadPerSec.i, phydat.AngularVelocityRadPerSec.j, phydat.AngularVelocityRadPerSec.k};
            
            ds4_raw_sens = ds4view.GetRawSensorData();
            controller->ControllerState.PSDS4State.RawSensorData.Accelerometer = {ds4_raw_sens.Accelerometer.i, ds4_raw_sens.Accelerometer.j, ds4_raw_sens.Accelerometer.k};
            controller->ControllerState.PSDS4State.RawSensorData.Gyroscope = {ds4_raw_sens.Gyroscope.i, ds4_raw_sens.Gyroscope.j, ds4_raw_sens.Gyroscope.k};

            ds4_calib_sens = ds4view.GetCalibratedSensorData();
            controller->ControllerState.PSDS4State.CalibratedSensorData.Accelerometer = {ds4_calib_sens.Accelerometer.i, ds4_calib_sens.Accelerometer.j, ds4_calib_sens.Accelerometer.k};
            controller->ControllerState.PSDS4State.CalibratedSensorData.Gyroscope = {ds4_calib_sens.Gyroscope.i, ds4_calib_sens.Gyroscope.j, ds4_calib_sens.Gyroscope.k};
            
            raw_track = ds4view.GetRawTrackerData();
            std::copy(std::begin(raw_track.TrackerIDs), std::end(raw_track.TrackerIDs), std::begin(controller->ControllerState.PSMoveState.RawTrackerData.TrackerIDs));
            controller->ControllerState.PSDS4State.RawTrackerData.ValidTrackerLocations = raw_track.ValidTrackerLocations;
            for(auto & track_id : raw_track.TrackerIDs)
            {
                controller->ControllerState.PSDS4State.RawTrackerData.ScreenLocations[track_id] = {
                    raw_track.ScreenLocations[track_id].x, raw_track.ScreenLocations[track_id].y
                };
                controller->ControllerState.PSDS4State.RawTrackerData.RelativePositionsCm[track_id] = {
                    raw_track.RelativePositionsCm[track_id].x, raw_track.RelativePositionsCm[track_id].y, raw_track.RelativePositionsCm[track_id].z
                };
                controller->ControllerState.PSDS4State.RawTrackerData.RelativeOrientations[track_id] = {
                    raw_track.RelativeOrientations[track_id].x, raw_track.RelativeOrientations[track_id].y, 
					raw_track.RelativeOrientations[track_id].z, raw_track.RelativeOrientations[track_id].w
                };
                controller->ControllerState.PSDS4State.RawTrackerData.TrackingProjections[track_id].shape_type = PSMTrackingProjection::eShapeType::PSMShape_LightBar;
				for (int index = 0; index < 3; ++index)
				{
					const PSMoveScreenLocation &pixel= raw_track.TrackingProjections[track_id].shape.lightbar.triangle[index];

					controller->ControllerState.PSDS4State.RawTrackerData.TrackingProjections[track_id].shape.lightbar.triangle[index]=
						{pixel.x, pixel.y};
				}
				for (int index = 0; index < 4; ++index)
				{
					const PSMoveScreenLocation &pixel= raw_track.TrackingProjections[track_id].shape.lightbar.quad[index];

					controller->ControllerState.PSDS4State.RawTrackerData.TrackingProjections[track_id].shape.lightbar.quad[index]=
						{pixel.x, pixel.y};
				}
            }
            
			controller->ControllerState.PSDS4State.DPadUpButton = static_cast<PSMButtonState>(ds4view.GetButtonDPadUp());
			controller->ControllerState.PSDS4State.DPadDownButton = static_cast<PSMButtonState>(ds4view.GetButtonDPadDown());
			controller->ControllerState.PSDS4State.DPadLeftButton = static_cast<PSMButtonState>(ds4view.GetButtonDPadLeft());
			controller->ControllerState.PSDS4State.DPadRightButton = static_cast<PSMButtonState>(ds4view.GetButtonDPadRight());

            controller->ControllerState.PSDS4State.TriangleButton = static_cast<PSMButtonState>(ds4view.GetButtonTriangle());
            controller->ControllerState.PSDS4State.CircleButton = static_cast<PSMButtonState>(ds4view.GetButtonCircle());
            controller->ControllerState.PSDS4State.CrossButton = static_cast<PSMButtonState>(ds4view.GetButtonCross());
            controller->ControllerState.PSDS4State.SquareButton = static_cast<PSMButtonState>(ds4view.GetButtonSquare());

			controller->ControllerState.PSDS4State.L1Button = static_cast<PSMButtonState>(ds4view.GetButtonL1());
			controller->ControllerState.PSDS4State.L2Button = static_cast<PSMButtonState>(ds4view.GetButtonL2());
			controller->ControllerState.PSDS4State.L3Button = static_cast<PSMButtonState>(ds4view.GetButtonL3());
			controller->ControllerState.PSDS4State.R1Button = static_cast<PSMButtonState>(ds4view.GetButtonR1());
			controller->ControllerState.PSDS4State.R2Button = static_cast<PSMButtonState>(ds4view.GetButtonR2());
			controller->ControllerState.PSDS4State.R3Button = static_cast<PSMButtonState>(ds4view.GetButtonR3());

			controller->ControllerState.PSDS4State.ShareButton = static_cast<PSMButtonState>(ds4view.GetButtonShare());
			controller->ControllerState.PSDS4State.OptionsButton = static_cast<PSMButtonState>(ds4view.GetButtonOptions());

            controller->ControllerState.PSDS4State.PSButton = static_cast<PSMButtonState>(ds4view.GetButtonPS());
			controller->ControllerState.PSDS4State.TrackPadButton = static_cast<PSMButtonState>(ds4view.GetButtonTrackpad());

			controller->ControllerState.PSDS4State.LeftAnalogX = ds4view.GetLeftAnalogX();
			controller->ControllerState.PSDS4State.LeftAnalogY = ds4view.GetLeftAnalogY();
			controller->ControllerState.PSDS4State.RightAnalogX = ds4view.GetRightAnalogX();
			controller->ControllerState.PSDS4State.RightAnalogY = ds4view.GetRightAnalogY();
			controller->ControllerState.PSDS4State.RightTriggerValue = static_cast<unsigned char>(ds4view.GetRightTriggerValue() * 255.f);
            controller->ControllerState.PSDS4State.LeftTriggerValue = static_cast<unsigned char>(ds4view.GetLeftTriggerValue() * 255.f);
            controller->ControllerState.PSDS4State.BigRumble = static_cast<unsigned char>(ds4view.GetBigRumble() * 255.f);
			controller->ControllerState.PSDS4State.SmallRumble = static_cast<unsigned char>(ds4view.GetSmallRumble() * 255.f);
            break;
            
        default:
            break;
    }
}

static void extractTrackerState(const ClientTrackerView *view, PSMTracker *tracker)
{
    tracker->is_connected= view->getIsConnected();
    tracker->data_frame_average_fps= view->GetDataFrameFPS();
    tracker->data_frame_last_received_time= view->GetDataFrameLastReceivedTime();
    tracker->sequence_num= view->getSequenceNum();
}

static void extractHmdState(const ClientHMDView *view, PSMHeadMountedDisplay *hmd)
{
    // Set the generic items
    hmd->bValid = view->IsValid();
    hmd->HmdType = static_cast<PSMHmdType>(view->GetHmdViewType());
    hmd->OutputSequenceNum = view->GetSequenceNum();
    hmd->IsConnected = view->GetIsConnected();
//    hmd->DataFrameLastReceivedTime =
    hmd->DataFrameAverageFPS = view->GetDataFrameFPS();
    
    // Have to declare some variables in case they are used in the switches
    ClientMorpheusView morpheus_view;
    PSMovePose pose;
    MorpheusPhysicsData phydat;
    MorpheusRawSensorData morpheus_raw_sens;
    MorpheusCalibratedSensorData morpheus_calib_sens;
    MorpheusRawTrackerData raw_track;
    
    switch (view->GetHmdViewType()) {
        case ClientHMDView::eHMDViewType::Morpheus:
            morpheus_view = view->GetMorpheusView();
            // Copy to PSMHeadMountedDisplay
//            char                    DevicePath[256];
//            char                    DeviceSerial[128];
            hmd->HmdState.MorpheusState.bIsTrackingEnabled = morpheus_view.GetIsTrackingEnabled();
            hmd->HmdState.MorpheusState.bIsCurrentlyTracking = morpheus_view.GetIsCurrentlyTracking();
            hmd->HmdState.MorpheusState.bIsOrientationValid = morpheus_view.GetIsOrientationValid();
            hmd->HmdState.MorpheusState.bIsPositionValid = morpheus_view.GetIsPositionValid();
            
            pose = morpheus_view.GetPose();
            hmd->HmdState.MorpheusState.Pose.Position = {pose.Position.x, pose.Position.y, pose.Position.z};
            hmd->HmdState.MorpheusState.Pose.Orientation = PSM_QuatfCreate(pose.Orientation.w, pose.Orientation.x, pose.Orientation.y, pose.Orientation.z);
            
            phydat = morpheus_view.GetPhysicsData();
            hmd->HmdState.MorpheusState.PhysicsData.LinearAccelerationCmPerSecSqr = {phydat.AccelerationCmPerSecSqr.i, phydat.AccelerationCmPerSecSqr.j, phydat.AccelerationCmPerSecSqr.k};
            hmd->HmdState.MorpheusState.PhysicsData.LinearVelocityCmPerSec = {phydat.VelocityCmPerSec.i, phydat.VelocityCmPerSec.j, phydat.VelocityCmPerSec.k};
            hmd->HmdState.MorpheusState.PhysicsData.AngularAccelerationRadPerSecSqr = {phydat.AngularAccelerationRadPerSecSqr.i, phydat.AngularAccelerationRadPerSecSqr.j, phydat.AngularAccelerationRadPerSecSqr.k};
            hmd->HmdState.MorpheusState.PhysicsData.AngularVelocityRadPerSec = {phydat.AngularVelocityRadPerSec.i, phydat.AngularVelocityRadPerSec.j, phydat.AngularVelocityRadPerSec.k};
            
            morpheus_raw_sens = morpheus_view.GetRawSensorData();
            hmd->HmdState.MorpheusState.RawSensorData.Accelerometer = {morpheus_raw_sens.Accelerometer.i, morpheus_raw_sens.Accelerometer.j, morpheus_raw_sens.Accelerometer.k};
            hmd->HmdState.MorpheusState.RawSensorData.Gyroscope = {morpheus_raw_sens.Gyroscope.i, morpheus_raw_sens.Gyroscope.j, morpheus_raw_sens.Gyroscope.k};

            morpheus_calib_sens = morpheus_view.GetCalibratedSensorData();
            hmd->HmdState.MorpheusState.CalibratedSensorData.Accelerometer = {morpheus_calib_sens.Accelerometer.i, morpheus_calib_sens.Accelerometer.j, morpheus_calib_sens.Accelerometer.k};
            hmd->HmdState.MorpheusState.CalibratedSensorData.Gyroscope = {morpheus_calib_sens.Gyroscope.i, morpheus_calib_sens.Gyroscope.j, morpheus_calib_sens.Gyroscope.k};
            
            raw_track = morpheus_view.GetRawTrackerData();
            std::copy(std::begin(raw_track.TrackerIDs), std::end(raw_track.TrackerIDs), std::begin(hmd->HmdState.MorpheusState.RawTrackerData.TrackerIDs));
            hmd->HmdState.MorpheusState.RawTrackerData.ValidTrackerLocations = raw_track.ValidTrackerLocations;
            for(auto & track_id : raw_track.TrackerIDs)
            {
                hmd->HmdState.MorpheusState.RawTrackerData.ScreenLocations[track_id] = {
                    raw_track.ScreenLocations[track_id].x, raw_track.ScreenLocations[track_id].y
                };
                hmd->HmdState.MorpheusState.RawTrackerData.RelativePositionsCm[track_id] = {
                    raw_track.RelativePositionsCm[track_id].x, raw_track.RelativePositionsCm[track_id].y, raw_track.RelativePositionsCm[track_id].z
                };
                hmd->HmdState.MorpheusState.RawTrackerData.RelativeOrientations[track_id] = {
                    raw_track.RelativeOrientations[track_id].x, raw_track.RelativeOrientations[track_id].y, 
                    raw_track.RelativeOrientations[track_id].z, raw_track.RelativeOrientations[track_id].w
                };
                hmd->HmdState.MorpheusState.RawTrackerData.TrackingProjections[track_id].shape_type = PSMTrackingProjection::eShapeType::PSMShape_PointCloud;
                hmd->HmdState.MorpheusState.RawTrackerData.TrackingProjections[track_id].shape.pointcloud.point_count =
                    raw_track.TrackingProjections[track_id].shape.pointcloud.point_count;
                for (int index = 0; index < raw_track.TrackingProjections[track_id].shape.pointcloud.point_count; ++index)
                {
                    const PSMoveScreenLocation &pixel= raw_track.TrackingProjections[track_id].shape.pointcloud.points[index];

                    hmd->HmdState.MorpheusState.RawTrackerData.TrackingProjections[track_id].shape.pointcloud.points[index]=
                        {pixel.x, pixel.y};
                }
            }
            break;
            
        default:
            break;
    }
}

static void processEvent(ClientPSMoveAPI::EventMessage *event_message)
{
    switch (event_message->event_type)
    {
    // Client Events
    case PSMEventMessage::eEventType::PSMEvent_connectedToService:
        g_bIsConnected= true;
        g_bHasConnectionStatusChanged= true;
        break;
    case PSMEventMessage::eEventType::PSMEvent_failedToConnectToService:
        g_bIsConnected= false;
        g_bHasConnectionStatusChanged= true;
        break;
    case PSMEventMessage::eEventType::PSMEvent_disconnectedFromService:
        g_bIsConnected= false;
        g_bHasConnectionStatusChanged= true;
        break;

    // Service Events
    case PSMEventMessage::eEventType::PSMEvent_opaqueServiceEvent:
        // Need to have protocol access to see what kind of event this is
        CLIENT_LOG_INFO("PSM_Update") << "Dropping opaque service event";
        break;
    case PSMEventMessage::eEventType::PSMEvent_controllerListUpdated:
        g_bHasControllerListChanged= true;
        break;
    case PSMEventMessage::eEventType::PSMEvent_trackerListUpdated:
        g_bHasTrackerListChanged= true;
        break;
    case PSMEventMessage::eEventType::PSMEvent_hmdListUpdated:
        g_bHasHMDListChanged= true;
        break;
    default:
        assert(0 && "unreachable");
        break;
    }
}

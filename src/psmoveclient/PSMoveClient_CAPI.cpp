#include <map>
#include "PSMoveClient_CAPI.h"
#include "ClientPSMoveAPI.h"
#include "PSMoveProtocol.pb.h"

struct ResultState
{
    bool bReceived= false;
    ClientPSMoveAPI::ResponseMessage out_response;
    
    static void response_callback(const ClientPSMoveAPI::ResponseMessage *response, void *userdata)
    {
        ResultState *result= reinterpret_cast<ResultState *>(userdata);
        
        result->out_response= *response;
        result->bReceived= true;
    }
};

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

int PSM_GetControllerList(PSMController** controllers)
{
    int controller_count = -1;
    ResultState resultState;
    ClientPSMoveAPI::register_callback(
                                       ClientPSMoveAPI::get_controller_list(),
                                       ResultState::response_callback,
                                       &resultState);
    
    while (!resultState.bReceived)
    {
        _PAUSE(10);
        ClientPSMoveAPI::update();
    }
    
    if (resultState.out_response.payload_type == ClientPSMoveAPI::eResponsePayloadType::_responsePayloadType_ControllerList)
    {
        ClientPSMoveAPI::ResponsePayload_ControllerList controller_list = resultState.out_response.payload.controller_list;
        controller_count = controller_list.count;
        for (int list_index = 0; list_index < controller_list.count; ++list_index)
        {
            PSMController *thisController = (PSMController *)calloc(1, sizeof(PSMController));
            thisController->ControllerID = controller_list.controller_id[list_index];
            thisController->ControllerType = static_cast<PSMController::eControllerType>(controller_list.controller_type[list_index]);
            std::cout << "Set ControllerID and ControllerType on *thisController from input" << std::endl;
            switch (thisController->ControllerType)
            {
                case PSMController::eControllerType::PSMove:
                    break;
                    //TODO
                case PSMController::eControllerType::PSNavi:
                    break;
                //case ClientControllerView::DualShock4:
                //break;
                default:
                    break;
            }
            controllers[list_index] = thisController;
        }
    }
    else
    {
        controller_count = -1;
    }
    
    return controller_count;
}

PSMResult PSM_RegisterAsControllerListener(PSMController *controller)
{
    ClientControllerView* view = ClientPSMoveAPI::allocate_controller_view(controller->ControllerID);
    controller->ControllerType = static_cast<PSMController::eControllerType>(view->GetControllerViewType());
    controller->IsConnected = view->GetIsConnected();
    controller->InputSequenceNum = view->GetInputSequenceNum();
    controller->OutputSequenceNum = view->GetOutputSequenceNum();
    
    return PSMResult_Success;
}

PSMResult PSM_StartControllerDataStream(PSMController *controller, unsigned int data_stream_flags)
{
    ClientControllerView * view = new ClientControllerView(controller->ControllerID);
    ClientPSMoveAPI::t_request_id req_id = ClientPSMoveAPI::start_controller_data_stream(view, data_stream_flags);
    return (req_id > 0) ? PSMResult_RequestSent : PSMResult_Error;
}

PSMResult PSM_StopControllerDataStream(PSMController *controller)
{
    ClientControllerView * view = new ClientControllerView(controller->ControllerID);
    return (ClientPSMoveAPI::stop_controller_data_stream(view) > 0) ? PSMResult_RequestSent : PSMResult_Error;
}

PSMResult PSM_DeregisterAsControllerListener(PSMController *controller)

{
    ClientControllerView * view = new ClientControllerView(controller->ControllerID);
    ClientPSMoveAPI::free_controller_view(view);
    return PSMResult_Success;
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
    ClientPSMoveAPI::update();
    ClientControllerView *view = ClientPSMoveAPI::get_controller_view(controller->ControllerID);
    
    // Set the generic items
    controller->bValid = view->IsValid();
    controller->ControllerType = static_cast<PSMController::eControllerType>(view->GetControllerViewType());
    controller->InputSequenceNum = view->GetInputSequenceNum();
    controller->OutputSequenceNum = view->GetOutputSequenceNum();
    controller->IsConnected = view->GetIsConnected();
//    controller->DataFrameLastReceivedTime =
    controller->DataFrameAverageFPS = view->GetDataFrameFPS();
    
    // Have to declare some variables in case they are used in the switches
    ClientPSMoveView psmview;
    PSMovePose pose;
    PSMovePhysicsData phydat;
    PSMoveRawSensorData raw_sens;
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
            controller->ControllerState.PSMoveState.bHasUnpublishedState = psmview.GetHasUnpublishedState();
//            is_stable = psmview.GetIsStableAndAlignedWithGravity();
//            PSMTrackingColorType    TrackingColorType;
            
            pose = psmview.GetPose();
            controller->ControllerState.PSMoveState.Pose.Position = {pose.Position.x, pose.Position.y, pose.Position.z};
            controller->ControllerState.PSMoveState.Pose.Orientation = {pose.Orientation.x, pose.Orientation.y, pose.Orientation.z, pose.Orientation.w};
            
            phydat = psmview.GetPhysicsData();
            controller->ControllerState.PSMoveState.PhysicsData.LinearAcceleration = {phydat.Acceleration.i, phydat.Acceleration.j, phydat.Acceleration.k};
            controller->ControllerState.PSMoveState.PhysicsData.LinearVelocity = {phydat.Velocity.i, phydat.Velocity.j, phydat.Velocity.k};
            controller->ControllerState.PSMoveState.PhysicsData.AngularAcceleration = {phydat.AngularAcceleration.i, phydat.AngularAcceleration.j, phydat.AngularAcceleration.k};
            controller->ControllerState.PSMoveState.PhysicsData.AngularVelocity = {phydat.AngularVelocity.i, phydat.AngularVelocity.j, phydat.AngularVelocity.k};
            
            raw_sens = psmview.GetRawSensorData();
            controller->ControllerState.PSMoveState.RawSensorData.Accelerometer = {raw_sens.Accelerometer.i, raw_sens.Accelerometer.j, raw_sens.Accelerometer.k};
            controller->ControllerState.PSMoveState.RawSensorData.Gyroscope = {raw_sens.Gyroscope.i, raw_sens.Gyroscope.j, raw_sens.Gyroscope.k};
            controller->ControllerState.PSMoveState.RawSensorData.Magnetometer = {raw_sens.Magnetometer.i, raw_sens.Magnetometer.j, raw_sens.Magnetometer.k};
            
            raw_track = psmview.GetRawTrackerData();
            std::copy(std::begin(raw_track.TrackerIDs), std::end(raw_track.TrackerIDs), std::begin(controller->ControllerState.PSMoveState.RawTrackerData.TrackerIDs));
            controller->ControllerState.PSMoveState.RawTrackerData.ValidTrackerLocations = raw_track.ValidTrackerLocations;
            for(auto & track_id : raw_track.TrackerIDs)
            {
                controller->ControllerState.PSMoveState.RawTrackerData.ScreenLocations[track_id] = {
                    raw_track.ScreenLocations[track_id].x, raw_track.ScreenLocations[track_id].y
                };
                controller->ControllerState.PSMoveState.RawTrackerData.RelativePositions[track_id] = {
                    raw_track.RelativePositions[track_id].x, raw_track.RelativePositions[track_id].y, raw_track.RelativePositions[track_id].z
                };
                controller->ControllerState.PSMoveState.RawTrackerData.TrackingProjections[track_id].shape_type = PSMTrackingProjection::eShapeType::Ellipse;
                controller->ControllerState.PSMoveState.RawTrackerData.TrackingProjections[track_id].shape.ellipse.center = {
                    raw_track.TrackingProjections[track_id].shape.ellipse.center.x, raw_track.TrackingProjections[track_id].shape.ellipse.center.y
                };
                controller->ControllerState.PSMoveState.RawTrackerData.TrackingProjections[track_id].shape.ellipse.angle = raw_track.TrackingProjections[track_id].shape.ellipse.angle;
                controller->ControllerState.PSMoveState.RawTrackerData.TrackingProjections[track_id].shape.ellipse.half_x_extent = raw_track.TrackingProjections[track_id].shape.ellipse.half_x_extent;
                controller->ControllerState.PSMoveState.RawTrackerData.TrackingProjections[track_id].shape.ellipse.half_y_extent = raw_track.TrackingProjections[track_id].shape.ellipse.half_y_extent;
            }
            
            controller->ControllerState.PSMoveState.TriangleButton = static_cast<PSMButtonState>(psmview.GetButtonTriangle());
            controller->ControllerState.PSMoveState.CircleButton = static_cast<PSMButtonState>(psmview.GetButtonCircle());
            controller->ControllerState.PSMoveState.CrossButton = static_cast<PSMButtonState>(psmview.GetButtonCross());
            controller->ControllerState.PSMoveState.SquareButton = static_cast<PSMButtonState>(psmview.GetButtonSquare());
            controller->ControllerState.PSMoveState.SelectButton = static_cast<PSMButtonState>(psmview.GetButtonSelect());
            controller->ControllerState.PSMoveState.StartButton = static_cast<PSMButtonState>(psmview.GetButtonStart());
            controller->ControllerState.PSMoveState.PSButton = static_cast<PSMButtonState>(psmview.GetButtonPS());
            controller->ControllerState.PSMoveState.MoveButton = static_cast<PSMButtonState>(psmview.GetButtonMove());
            controller->ControllerState.PSMoveState.TriggerButton = static_cast<PSMButtonState>(psmview.GetButtonTrigger());
            controller->ControllerState.PSMoveState.TriggerValue = psmview.GetTriggerValue();
            controller->ControllerState.PSMoveState.Rumble = psmview.GetRumble();
//            unsigned char           LED_r, LED_g, LED_b;
            break;
            
        case ClientControllerView::eControllerType::PSNavi:
            break;
            
        default:
            break;
    }
    return PSMResult_Success;
}

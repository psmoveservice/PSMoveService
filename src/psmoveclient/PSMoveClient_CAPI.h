#ifndef __PSMOVECLIENT_CAPI_H
#define __PSMOVECLIENT_CAPI_H

#include "PSMoveClient_export.h"
#include "ClientConstants.h"

typedef enum _PSMResult
{
    PSMResult_Error                 = -1,
    PSMResult_Success               = 0
} PSMResult;

typdef enum _PSMControllerType
{
    PSMControllerType_PSMove        = 0,
    PSMControllerType_PSNavi        = 1,
    PSMControllerType_DS4           = 2
} PSMControllerType;


//PSM_PUBLIC_FUNCTION(const char*) PSM_GetVersionString();
PSM_PUBLIC_FUNCTION(PSMResult) PSM_Initialize(const char* host, const char* port);  //"localhost", "9512"
PSM_PUBLIC_FUNCTION(PSMresult) PSM_Shutdown();
PSM_PUBLIC_FUNCTION(PSMResult) PSM_Update();
PSM_PUBLIC_FUNCTION(PSMResult) PSM_PollNextMessage();  //*message, sizeof(message)

/// Controller Methods
//static ClientControllerView *allocate_controller_view(int ControllerID);
//static void free_controller_view(ClientControllerView *view);
PSM_PUBLIC_FUNCTION(PSMResult) get_controller_list();
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StartControllerDataStream(PSMController *controller, unsigned int data_stream_flags);

static t_request_id get_controller_list();
static t_request_id start_controller_data_stream(ClientControllerView *view, unsigned int data_stream_flags);
static t_request_id stop_controller_data_stream(ClientControllerView *view);
static t_request_id set_led_tracking_color(ClientControllerView *view, PSMoveTrackingColorType tracking_color);
static t_request_id reset_pose(ClientControllerView *view);

/// Tracker Methods
static ClientTrackerView *allocate_tracker_view(const ClientTrackerInfo &trackerInfo);
static void free_tracker_view(ClientTrackerView *view);

static t_request_id get_tracker_list();
static t_request_id start_tracker_data_stream(ClientTrackerView *view);
static t_request_id stop_tracker_data_stream(ClientTrackerView *view);
static t_request_id get_hmd_tracking_space_settings();


#endif
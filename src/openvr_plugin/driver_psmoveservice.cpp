//
// driver_psmoveservice.cpp : Defines the client and server interfaces used by the SteamVR runtime.
//

//==================================================================================================
// Includes
//==================================================================================================
#include "driver_psmoveservice.h"
#include <sstream>

#define _USE_MATH_DEFINES
#include <math.h>

#if defined( _WIN32 )
#include <windows.h>
#else
#include <unistd.h>
#endif

//==================================================================================================
// Macros
//==================================================================================================

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(GNUC) || defined(COMPILER_GCC) || defined(__GNUC__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C" 
#else
#error "Unsupported Platform."
#endif

#if _MSC_VER
#define strcasecmp(a, b) stricmp(a,b)
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

#define LOG_TOUCHPAD_EMULATION 0
#define LOG_REALIGN_TO_HMD 1

//==================================================================================================
// Constants
//==================================================================================================
static const float k_fScalePSMoveAPIToMeters = 0.01f;  // psmove driver in cm
static const float k_fRadiansToDegrees = 180.f / 3.14159265f;

static const char *k_PSButtonNames[CPSMoveControllerLatest::k_EPSButtonID_Count] = {
    "ps",
    "left",
    "up",
    "down",
    "right",
    "move",
    "trackpad",
    "trigger",
    "triangle",
    "square",
    "circle",
    "cross",
    "select",
    "share",
    "start",
    "options",
    "l1",
    "l2",
    "l3",
    "r1",
    "r2",
    "r3"
};

static const int k_max_vr_buttons = 37;
static const char *k_VRButtonNames[k_max_vr_buttons] = {
    "system",               // k_EButton_System
    "application_menu",     // k_EButton_ApplicationMenu
    "grip",                 // k_EButton_Grip
    "dpad_left",            // k_EButton_DPad_Left
    "dpad_up",              // k_EButton_DPad_Up
    "dpad_right",           // k_EButton_DPad_Right
    "dpad_down",            // k_EButton_DPad_Down
    "a",                    // k_EButton_A
    "button_8",              // (vr::EVRButtonId)8
    "button_9",              // (vr::EVRButtonId)9
    "button_10",              // (vr::EVRButtonId)10
    "button_11",              // (vr::EVRButtonId)11
    "button_12",              // (vr::EVRButtonId)12
    "button_13",              // (vr::EVRButtonId)13
    "button_14",              // (vr::EVRButtonId)14
    "button_15",              // (vr::EVRButtonId)15
    "button_16",              // (vr::EVRButtonId)16
    "button_17",              // (vr::EVRButtonId)17
    "button_18",              // (vr::EVRButtonId)18
    "button_19",              // (vr::EVRButtonId)19
    "button_20",              // (vr::EVRButtonId)20
    "button_21",              // (vr::EVRButtonId)21
    "button_22",              // (vr::EVRButtonId)22
    "button_23",              // (vr::EVRButtonId)23
    "button_24",              // (vr::EVRButtonId)24
    "button_25",              // (vr::EVRButtonId)25
    "button_26",              // (vr::EVRButtonId)26
    "button_27",              // (vr::EVRButtonId)27
    "button_28",              // (vr::EVRButtonId)28
    "button_29",              // (vr::EVRButtonId)29
    "button_30",              // (vr::EVRButtonId)30
    "button_31",              // (vr::EVRButtonId)31
    "touchpad",               // k_EButton_Axis0, k_EButton_SteamVR_Touchpad
    "trigger",                // k_EButton_Axis1, k_EButton_SteamVR_Trigger
    "axis_2",                 // k_EButton_Axis2
    "axis_3",                 // k_EButton_Axis3
    "axis_4",                 // k_EButton_Axis4
};

static const int k_max_vr_touchpad_directions = CPSMoveControllerLatest::k_EVRTouchpadDirection_Count;
static const char *k_VRTouchpadDirectionNames[k_max_vr_touchpad_directions] = {
	"none",
	"touchpad_left",
	"touchpad_up",
	"touchpad_right",
	"touchpad_down",
};

//==================================================================================================
// Globals
//==================================================================================================

CServerDriver_PSMoveService g_ServerTrackedDeviceProvider;
CClientDriver_PSMoveService g_ClientTrackedDeviceProvider;

//==================================================================================================
// Logging helpers
//==================================================================================================

static vr::IDriverLog * s_pLogFile = NULL;

static bool InitDriverLog( vr::IDriverLog *pDriverLog )
{
    if ( s_pLogFile )
        return false;
    s_pLogFile = pDriverLog;
    return s_pLogFile != NULL;
}

static void CleanupDriverLog()
{
    s_pLogFile = NULL;
}

static void DriverLogVarArgs( const char *pMsgFormat, va_list args )
{
    char buf[1024];
#if defined( WIN32 )
    vsprintf_s( buf, pMsgFormat, args );
#else
    vsnprintf( buf, sizeof( buf ), pMsgFormat, args );
#endif

    if ( s_pLogFile )
        s_pLogFile->Log( buf );
}

/** Provides printf-style debug logging via the vr::IDriverLog interface provided by SteamVR
* during initialization.  Client logging ends up in vrclient_appname.txt and server logging
* ends up in vrserver.txt.
*/
static void DriverLog( const char *pMsgFormat, ... )
{
    va_list args;
    va_start( args, pMsgFormat );

    DriverLogVarArgs( pMsgFormat, args );

    va_end( args );
}


static std::string PsMovePositionToString( const PSMovePosition& position )
{
	std::ostringstream stringBuilder;
	stringBuilder << "(" << position.x << ", " << position.y << ", " << position.z << ")";
	return stringBuilder.str();
}


static std::string PsMoveQuaternionToString(const PSMoveQuaternion& rotation)
{
	std::ostringstream stringBuilder;
	stringBuilder << "(" << rotation.w << ", " << rotation.x << ", " << rotation.y << ", " << rotation.z << ")";
	return stringBuilder.str();
}


static std::string PsMovePoseToString(const PSMovePose& pose)
{
	std::ostringstream stringBuilder;
	stringBuilder << "[Pos: " << PsMovePositionToString(pose.Position) << ", Rot:" << PsMoveQuaternionToString(pose.Orientation) << "]";
	return stringBuilder.str();
}


//==================================================================================================
// Math Helpers
//==================================================================================================
// From: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
static PSMoveQuaternion openvrMatrixExtractPSMoveQuaternion(const vr::HmdMatrix34_t &openVRTransform)
{
	PSMoveQuaternion q;

	const float(&a)[3][4] = openVRTransform.m;
	const float trace = a[0][0] + a[1][1] + a[2][2];

	if (trace > 0)
	{
		const float s = 0.5f / sqrtf(trace + 1.0f);

		q.w = 0.25f / s;
		q.x = (a[2][1] - a[1][2]) * s;
		q.y = (a[0][2] - a[2][0]) * s;
		q.z = (a[1][0] - a[0][1]) * s;
	}
	else
	{
		if (a[0][0] > a[1][1] && a[0][0] > a[2][2])
		{
			const float s = 2.0f * sqrtf(1.0f + a[0][0] - a[1][1] - a[2][2]);

			q.w = (a[2][1] - a[1][2]) / s;
			q.x = 0.25f * s;
			q.y = (a[0][1] + a[1][0]) / s;
			q.z = (a[0][2] + a[2][0]) / s;
		}
		else if (a[1][1] > a[2][2])
		{
			const float s = 2.0f * sqrtf(1.0f + a[1][1] - a[0][0] - a[2][2]);

			q.w = (a[0][2] - a[2][0]) / s;
			q.x = (a[0][1] + a[1][0]) / s;
			q.y = 0.25f * s;
			q.z = (a[1][2] + a[2][1]) / s;
		}
		else
		{
			const float s = 2.0f * sqrtf(1.0f + a[2][2] - a[0][0] - a[1][1]);

			q.w = (a[1][0] - a[0][1]) / s;
			q.x = (a[0][2] + a[2][0]) / s;
			q.y = (a[1][2] + a[2][1]) / s;
			q.z = 0.25f * s;
		}
	}

	return q;
}

static PSMovePosition openvrMatrixExtractPSMovePosition(const vr::HmdMatrix34_t &openVRTransform)
{
	const float(&a)[3][4] = openVRTransform.m;

	return PSMovePosition::create(a[0][3], a[1][3], a[2][3]);
}

static PSMovePose openvrMatrixExtractPSMovePose(const vr::HmdMatrix34_t &openVRTransform)
{
	PSMovePose pose;
	pose.Orientation = openvrMatrixExtractPSMoveQuaternion(openVRTransform);
	pose.Position = openvrMatrixExtractPSMovePosition(openVRTransform);

	return pose;
}

//==================================================================================================
// Server Provider
//==================================================================================================

CServerDriver_PSMoveService::CServerDriver_PSMoveService()
    : m_bLaunchedPSMoveMonitor(false)
{
}

CServerDriver_PSMoveService::~CServerDriver_PSMoveService()
{
    // 10/10/2015 benj:  vrserver is exiting without calling Cleanup() to balance Init()
    // causing std::thread to call std::terminate
    Cleanup();
}

vr::EVRInitError CServerDriver_PSMoveService::Init(
    vr::IDriverLog * pDriverLog, 
    vr::IServerDriverHost * pDriverHost, 
    const char * pchUserDriverConfigDir, 
    const char * pchDriverInstallDir )
{
    vr::EVRInitError initError = vr::VRInitError_None;

    InitDriverLog( pDriverLog );
    m_pDriverHost = pDriverHost;
    m_strDriverInstallDir = pchDriverInstallDir;
    
    // By default, assume the psmove and openvr tracking spaces are the same
    m_worldFromDriverPose.Clear();
    
    // Note that reconnection is a non-blocking async request.
    // Returning true means we we're able to start trying to connect,
    // not that we are successfully connected yet.
    if (!ReconnectToPSMoveService())
    {
        initError= vr::VRInitError_Driver_Failed;
    }

    return initError;
}

bool CServerDriver_PSMoveService::ReconnectToPSMoveService()
{
    if (ClientPSMoveAPI::has_started())
    {
        ClientPSMoveAPI::shutdown();
    }

    return ClientPSMoveAPI::startup(
        PSMOVESERVICE_DEFAULT_ADDRESS, 
        PSMOVESERVICE_DEFAULT_PORT, 
        _log_severity_level_warning);
}

void CServerDriver_PSMoveService::Cleanup()
{
    ClientPSMoveAPI::shutdown();
}

const char * const *CServerDriver_PSMoveService::GetInterfaceVersions()
{
    return vr::k_InterfaceVersions;
}

uint32_t CServerDriver_PSMoveService::GetTrackedDeviceCount()
{
    return static_cast<uint32_t>(m_vecTrackedDevices.size());
}

vr::ITrackedDeviceServerDriver * CServerDriver_PSMoveService::GetTrackedDeviceDriver( 
    uint32_t unWhich)
{
    if (unWhich < m_vecTrackedDevices.size())
    {
        return m_vecTrackedDevices[unWhich];
    }

    return nullptr;
}

vr::ITrackedDeviceServerDriver * CServerDriver_PSMoveService::FindTrackedDeviceDriver(
    const char * pchId)
{
    for (auto it = m_vecTrackedDevices.begin(); it != m_vecTrackedDevices.end(); ++it)
    {
        if ( 0 == strcmp( ( *it )->GetSerialNumber(), pchId ) )
        {
            return *it;
        }
    }

    return nullptr;
}

void CServerDriver_PSMoveService::RunFrame()
{
    // Poll any events from the service
    ClientPSMoveAPI::update();

    // Poll events queued up by the call to ClientPSMoveAPI::update()
    ClientPSMoveAPI::Message message;
    while (ClientPSMoveAPI::poll_next_message(&message, sizeof(message)))
    {
        switch (message.payload_type)
        {
        case ClientPSMoveAPI::_messagePayloadType_Response:
            HandleClientPSMoveResponse(&message.response_data);
            break;
        case ClientPSMoveAPI::_messagePayloadType_Event:
            HandleClientPSMoveEvent(&message.event_data);
            break;
        }
    }

    // Update all active tracked devices
    for (auto it = m_vecTrackedDevices.begin(); it != m_vecTrackedDevices.end(); ++it)
    {
        CPSMoveTrackedDeviceLatest *pTrackedDevice = *it;

        switch (pTrackedDevice->GetTrackedDeviceClass())
        {
        case vr::TrackedDeviceClass_Controller:
            {
                CPSMoveControllerLatest *pController = static_cast<CPSMoveControllerLatest *>(pTrackedDevice);

                pController->Update();
            } break;
        case vr::TrackedDeviceClass_TrackingReference:
            {
                CPSMoveTrackerLatest *pTracker = static_cast<CPSMoveTrackerLatest *>(pTrackedDevice);

                pTracker->Update();
            } break;
        default:
            assert(0 && "unreachable");
        }
    }
}

bool CServerDriver_PSMoveService::ShouldBlockStandbyMode()
{
    return false;
}

void CServerDriver_PSMoveService::EnterStandby()
{
}

void CServerDriver_PSMoveService::LeaveStandby()
{
}

// -- Event Handling -----
void CServerDriver_PSMoveService::HandleClientPSMoveEvent(
    const ClientPSMoveAPI::EventMessage *event)
{
    ClientPSMoveAPI::eEventType event_type = event->event_type;

    switch (event_type)
    {
    // Client Events
    case ClientPSMoveAPI::connectedToService:
        HandleConnectedToPSMoveService();
        break;
    case ClientPSMoveAPI::failedToConnectToService:
        HandleFailedToConnectToPSMoveService();
        break;
    case ClientPSMoveAPI::disconnectedFromService:
        HandleDisconnectedFromPSMoveService();
        break;

    // Service Events
    case ClientPSMoveAPI::opaqueServiceEvent: 
        // We don't care about any opaque service events
        break;
    case ClientPSMoveAPI::controllerListUpdated:
        HandleControllerListChanged();
        break;
    case ClientPSMoveAPI::trackerListUpdated:
        HandleTrackerListChanged();
        break;

    //###HipsterSloth $TODO - Need a notification for when a tracker pose changes
    }
}

void CServerDriver_PSMoveService::HandleConnectedToPSMoveService()
{
    // Ask the service for a list of connected controllers
    // Response handled in HandleControllerListReponse()
    ClientPSMoveAPI::get_controller_list();

    // Ask the service for a list of connected trackers
    // Response handled in HandleTrackerListReponse()
    ClientPSMoveAPI::get_tracker_list();
}

void CServerDriver_PSMoveService::HandleFailedToConnectToPSMoveService()
{
    // Immediately attempt to reconnect to the service
    ReconnectToPSMoveService();
}

void CServerDriver_PSMoveService::HandleDisconnectedFromPSMoveService()
{
    for (auto it = m_vecTrackedDevices.begin(); it != m_vecTrackedDevices.end(); ++it)
    {
        CPSMoveTrackedDeviceLatest *pDevice = *it;

        pDevice->Deactivate();
    }

    // Immediately attempt to reconnect to the service
    ReconnectToPSMoveService();
}

void CServerDriver_PSMoveService::HandleControllerListChanged()
{
    // Ask the service for a list of connected controllers
    // Response handled in HandleControllerListReponse()
    ClientPSMoveAPI::get_controller_list();
}

void CServerDriver_PSMoveService::HandleTrackerListChanged()
{
    // Ask the service for a list of connected trackers
    // Response handled in HandleTrackerListReponse()
    ClientPSMoveAPI::get_tracker_list();
}

// -- Response Handling -----
void CServerDriver_PSMoveService::HandleClientPSMoveResponse(
    const ClientPSMoveAPI::ResponseMessage *response)
{
    switch (response->payload_type)
    {
    case ClientPSMoveAPI::_responsePayloadType_Empty:
        DriverLog("NotifyClientPSMoveResponse - request id %d returned result %s.\n",
            response->request_id, 
            (response->result_code == ClientPSMoveAPI::_clientPSMoveResultCode_ok) ? "ok" : "error");
        break;
    case ClientPSMoveAPI::_responsePayloadType_ControllerList:
        DriverLog("NotifyClientPSMoveResponse - Controller Count = %d (request id %d).\n", 
            response->payload.controller_list.count, response->request_id);
        HandleControllerListReponse(&response->payload.controller_list);
        break;
    case ClientPSMoveAPI::_responsePayloadType_TrackerList:
        DriverLog("NotifyClientPSMoveResponse - Tracker Count = %d (request id %d).\n",
            response->payload.tracker_list.count, response->request_id);
        HandleTrackerListReponse(&response->payload.tracker_list);
        break;
    default:
        DriverLog("NotifyClientPSMoveResponse - Unhandled response (request id %d).\n", response->request_id);
    }
}

void CServerDriver_PSMoveService::HandleControllerListReponse(
    const ClientPSMoveAPI::ResponsePayload_ControllerList *controller_list)
{
    for (int list_index = 0; list_index < controller_list->count; ++list_index)
    {
        int controller_id = controller_list->controller_id[list_index];
        ClientControllerView::eControllerType controller_type = controller_list->controller_type[list_index];

        switch (controller_type)
        {
        case ClientControllerView::PSMove:
            AllocateUniquePSMoveController(controller_id);
            break;
        case ClientControllerView::PSNavi:
            AllocateUniquePSNaviController(controller_id);
            break;
        case ClientControllerView::PSDualShock4:
            AllocateUniqueDualShock4Controller(controller_id);
            break;
        default:
            break;
        }
    }
}

void CServerDriver_PSMoveService::HandleTrackerListReponse(
    const ClientPSMoveAPI::ResponsePayload_TrackerList *tracker_list)
{
    for (int list_index = 0; list_index < tracker_list->count; ++list_index)
    {
        const ClientTrackerInfo &trackerInfo = tracker_list->trackers[list_index];

        AllocateUniquePSMoveTracker(trackerInfo);
    }
}

void CServerDriver_PSMoveService::SetHMDTrackingSpace(
    const PSMovePose &origin_pose)
{
	#if LOG_REALIGN_TO_HMD != 0
		DriverLog("Begin CServerDriver_PSMoveService::SetHMDTrackingSpace()\n");
	#endif

    m_worldFromDriverPose = origin_pose;

    // Tell all the devices that the relationship between the psmove and the OpenVR
    // tracking spaces changed
    for (auto it = m_vecTrackedDevices.begin(); it != m_vecTrackedDevices.end(); ++it)
    {
        CPSMoveTrackedDeviceLatest *pDevice = *it;

        pDevice->RefreshWorldFromDriverPose();
    }
}

static void GenerateControllerSerialNumber( char *p, int psize, int controller )
{
    snprintf(p, psize, "psmove_controller%d", controller);
}

void CServerDriver_PSMoveService::AllocateUniquePSMoveController(int ControllerID)
{
    char buf[256];
    GenerateControllerSerialNumber(buf, sizeof(buf), ControllerID);

    if ( !FindTrackedDeviceDriver(buf) )
    {
        DriverLog( "added new psmove controller %s\n", buf );
        m_vecTrackedDevices.push_back( new CPSMoveControllerLatest( m_pDriverHost, ControllerID ) );

        if (m_pDriverHost)
        {
            m_pDriverHost->TrackedDeviceAdded(m_vecTrackedDevices.back()->GetSerialNumber());
        }
    }
}

void CServerDriver_PSMoveService::AllocateUniquePSNaviController(int ControllerID)
{
    char buf[256];
    GenerateControllerSerialNumber(buf, sizeof(buf), ControllerID);

    if (!FindTrackedDeviceDriver(buf))
    {
        DriverLog("added new ps navi controller %s\n", buf);
        m_vecTrackedDevices.push_back(new CPSMoveControllerLatest(m_pDriverHost, ControllerID));

        if (m_pDriverHost)
        {
            m_pDriverHost->TrackedDeviceAdded(m_vecTrackedDevices.back()->GetSerialNumber());
        }
    }
}

void CServerDriver_PSMoveService::AllocateUniqueDualShock4Controller(int ControllerID)
{
    char buf[256];
    GenerateControllerSerialNumber(buf, sizeof(buf), ControllerID);

    if (!FindTrackedDeviceDriver(buf))
    {
        DriverLog("added new ps dualshock4 controller %s\n", buf);
        m_vecTrackedDevices.push_back(new CPSMoveControllerLatest(m_pDriverHost, ControllerID));

        if (m_pDriverHost)
        {
            m_pDriverHost->TrackedDeviceAdded(m_vecTrackedDevices.back()->GetSerialNumber());
        }
    }
}

static void GenerateTrackerSerialNumber(char *p, int psize, int tracker)
{
    snprintf(p, psize, "psmove_tracker%d", tracker);
}

void CServerDriver_PSMoveService::AllocateUniquePSMoveTracker(const ClientTrackerInfo &trackerInfo)
{
    char buf[256];
    GenerateTrackerSerialNumber(buf, sizeof(buf), trackerInfo.tracker_id);

    if (!FindTrackedDeviceDriver(buf))
    {
        DriverLog("added new device %s\n", buf);
        m_vecTrackedDevices.push_back(new CPSMoveTrackerLatest(m_pDriverHost, trackerInfo));

        if (m_pDriverHost)
        {
            m_pDriverHost->TrackedDeviceAdded(m_vecTrackedDevices.back()->GetSerialNumber());
        }
    }
}


// The monitor_psmove is a companion program which can display overlay prompts for us
// and tell us the pose of the HMD at the moment we want to calibrate.
void CServerDriver_PSMoveService::LaunchPSMoveMonitor( const char * pchDriverInstallDir )
{
	#if LOG_REALIGN_TO_HMD != 0
		DriverLog("Entered CServerDriver_PSMoveService::LaunchPSMoveMonitor(%s)\n", pchDriverInstallDir );
	#endif

    if ( m_bLaunchedPSMoveMonitor )
	{
        return;
	}

    m_bLaunchedPSMoveMonitor = true;

    std::ostringstream path_and_executable_string_builder;

    path_and_executable_string_builder << "\"" << pchDriverInstallDir << "\\bin\\";
#if defined( _WIN64 )
    path_and_executable_string_builder << "win64";
#elif defined( _WIN32 )
    path_and_executable_string_builder << "win32";
#elif defined(__APPLE__) 
    path_and_executable_string_builder << "osx";
#else 
    #error Do not know how to launch psmove_monitor
#endif


#if defined( _WIN32 ) || defined( _WIN64 )
	const std::string monitor_path = path_and_executable_string_builder.str() + "\\\"";

	path_and_executable_string_builder << "\\monitor_psmove.exe\"";
	const std::string monitor_path_and_exe = path_and_executable_string_builder.str();

	std::ostringstream args_string_builder;
	args_string_builder << "\"" << pchDriverInstallDir << "\\resources\"";
	const std::string monitor_args = args_string_builder.str();

	#if LOG_REALIGN_TO_HMD != 0
		DriverLog("CServerDriver_PSMoveService::LaunchPSMoveMonitor() monitor_psmove windows full path: %s\n", monitor_path_and_exe.c_str());
		DriverLog("CServerDriver_PSMoveService::LaunchPSMoveMonitor() monitor_psmove windows args: %s\n", monitor_args.c_str());
		DriverLog("CServerDriver_PSMoveService::LaunchPSMoveMonitor() monitor_psmove windows current directory: %s\n", monitor_path.c_str());
	#endif

	HINSTANCE shellExecuteResult
		= ShellExecuteA(NULL, "open", monitor_path_and_exe.c_str(), monitor_args.c_str(), monitor_path.c_str(), SW_HIDE);

	DriverLog("CServerDriver_PSMoveService::LaunchPSMoveMonitor() Start monitor_psmove ShellExecuteA() result: %d.\n", shellExecuteResult);

#elif defined(__APPLE__) 
    pid_t processId;
    if ((processId = fork()) == 0)
    {
		path_and_executable_string_builder << "\\monitor_psmove";

		const std::string monitor_exe_path = path_and_executable_string_builder.str();        
        char * const argv[] = { monitor_exe_path.c_str(), pchDriverInstallDir, NULL };
        
        if (execv(app, argv) < 0)
        {
            DriverLog( "Failed to exec child process\n");
        }
    }
    else if (processId < 0)
    {
        DriverLog( "Failed to fork child process\n");
        perror("fork error");
    }
#else 
#error Do not know how to launch psmove config tool
#endif
}

/** Launch monitor_psmove if needed (requested by devices as they activate) */
void CServerDriver_PSMoveService::LaunchPSMoveMonitor()
{
	#if LOG_REALIGN_TO_HMD != 0
		DriverLog("Entered CServerDriver_PSMoveService::LaunchPSMoveMonitor()\n");
	#endif

    LaunchPSMoveMonitor( m_strDriverInstallDir.c_str() );
}

//==================================================================================================
// Client Provider
//==================================================================================================

CClientDriver_PSMoveService::CClientDriver_PSMoveService()
{
}

CClientDriver_PSMoveService::~CClientDriver_PSMoveService()
{
}

vr::EVRInitError CClientDriver_PSMoveService::Init( 
    vr::EClientDriverMode driverMode,
    vr::IDriverLog * pDriverLog, 
    vr::IClientDriverHost * pDriverHost, 
    const char * pchUserDriverConfigDir, 
    const char * pchDriverInstallDir )
{
    vr::EVRInitError result= vr::VRInitError_Driver_Failed;

    switch(driverMode)
    {
    case vr::ClientDriverMode_Normal:
        InitDriverLog( pDriverLog );
        m_pDriverHost = pDriverHost;
        result= vr::VRInitError_None;
        break;
    case vr::ClientDriverMode_Watchdog: // client should return VRInitError_Init_LowPowerWatchdogNotSupported if it can't support this mode
        result= vr::VRInitError_Init_LowPowerWatchdogNotSupported;
        break;
    }

    return result;
}

void CClientDriver_PSMoveService::Cleanup()
{
}

bool CClientDriver_PSMoveService::BIsHmdPresent( const char * pchUserConfigDir )
{
    return false;
}

vr::EVRInitError CClientDriver_PSMoveService::SetDisplayId( const char * pchDisplayId )
{
    return vr::VRInitError_None;
    //return vr::VRInitError_Driver_HmdUnknown;
}

vr::HiddenAreaMesh_t CClientDriver_PSMoveService::GetHiddenAreaMesh( vr::EVREye eEye )
{
    vr::HiddenAreaMesh_t hiddenAreaMesh= vr::HiddenAreaMesh_t();

    return hiddenAreaMesh;
}

uint32_t CClientDriver_PSMoveService::GetMCImage( uint32_t * pImgWidth, uint32_t * pImgHeight, uint32_t * pChannels, void * pDataBuffer, uint32_t unBufferLen )
{
    uint32_t image= uint32_t();

    return image;
}

//==================================================================================================
// Tracked Device Driver
//==================================================================================================

CPSMoveTrackedDeviceLatest::CPSMoveTrackedDeviceLatest(vr::IServerDriverHost * pDriverHost)
    : m_pDriverHost(pDriverHost)
    , m_properties_dirty(false)
    , m_unSteamVRTrackedDeviceId(vr::k_unTrackedDeviceIndexInvalid)
{
    memset(&m_Pose, 0, sizeof(m_Pose));
    m_Pose.result = vr::TrackingResult_Uninitialized;

    // By default, assume that the tracked devices are in the tracking space as OpenVR
    m_Pose.qWorldFromDriverRotation.w = 1.f;
    m_Pose.qWorldFromDriverRotation.x = 0.f;
    m_Pose.qWorldFromDriverRotation.y = 0.f;
    m_Pose.qWorldFromDriverRotation.z = 0.f;
    m_Pose.vecWorldFromDriverTranslation[0] = 0.f;
    m_Pose.vecWorldFromDriverTranslation[1] = 0.f;
    m_Pose.vecWorldFromDriverTranslation[2] = 0.f;

    m_firmware_revision = 0x0001;
    m_hardware_revision = 0x0001;

	m_lastHMDPoseInMeters.Clear();
	m_lastHMDPoseTime= std::chrono::time_point<std::chrono::high_resolution_clock>();
	m_bIsLastHMDPoseValid= false;
	m_hmdResultCallback = nullptr;
	m_hmdResultUserData = nullptr;
}

CPSMoveTrackedDeviceLatest::~CPSMoveTrackedDeviceLatest()
{

}

// Shared Implementation of vr::ITrackedDeviceServerDriver
vr::EVRInitError CPSMoveTrackedDeviceLatest::Activate(uint32_t unObjectId)
{
    DriverLog("CPSMoveTrackedDeviceLatest::Activate: %s is object id %d\n", GetSerialNumber(), unObjectId);
    m_unSteamVRTrackedDeviceId = unObjectId;

    return vr::VRInitError_None;
}

void CPSMoveTrackedDeviceLatest::Deactivate() 
{
    DriverLog("CPSMoveTrackedDeviceLatest::Deactivate: %s was object id %d\n", GetSerialNumber(), m_unSteamVRTrackedDeviceId);
    m_unSteamVRTrackedDeviceId = vr::k_unTrackedDeviceIndexInvalid;
}

void CPSMoveTrackedDeviceLatest::PowerOff()
{
    //###HipsterSloth $TODO - No good way to do this at the moment
}

void *CPSMoveTrackedDeviceLatest::GetComponent(const char *pchComponentNameAndVersion)
{
    return NULL;
}

void CPSMoveTrackedDeviceLatest::DebugRequest(
    const char * pchRequest,
    char * pchResponseBuffer,
    uint32_t unResponseBufferSize)
{
	std::istringstream ss( pchRequest );
	std::string strCmd;

	ss >> strCmd;
	if (strCmd == "psmove:hmd_pose")
	{
		#if LOG_REALIGN_TO_HMD != 0
			DriverLog( "CPSMoveTrackedDeviceLatest::DebugRequest(): %s\n", strCmd.c_str() );
		#endif

		// monitor_psmove is calling us back with HMD tracking information
		vr::HmdMatrix34_t hmdTransform;


		#if LOG_REALIGN_TO_HMD != 0
			std::ostringstream matrixStringBuilder;
			matrixStringBuilder << "hmdTransform.m:\n\t";
		#endif

		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 4; ++j)
			{
				ss >> hmdTransform.m[i][j];
				#if LOG_REALIGN_TO_HMD != 0
					matrixStringBuilder << "[" << hmdTransform.m[i][j] << "] ";
				#endif
			}
			#if LOG_REALIGN_TO_HMD != 0
				matrixStringBuilder << "\n\t";
			#endif
		}

		m_lastHMDPoseInMeters = openvrMatrixExtractPSMovePose(hmdTransform);

		#if LOG_REALIGN_TO_HMD != 0
			matrixStringBuilder << "\n";
			DriverLog(matrixStringBuilder.str().c_str());

			DriverLog("Extracted pose: %s \n", PsMovePoseToString(m_lastHMDPoseInMeters).c_str() );
		#endif

		m_lastHMDPoseTime = std::chrono::high_resolution_clock::now();

		if (m_hmdResultCallback != nullptr)
		{
			m_hmdResultCallback(m_lastHMDPoseInMeters, m_hmdResultUserData);
			m_hmdResultCallback = nullptr;
			m_hmdResultUserData = nullptr;
		}
	}
}

void CPSMoveTrackedDeviceLatest::RequestLatestHMDPose(
	float maxPoseAgeMilliseconds,
	CPSMoveTrackedDeviceLatest::t_hmd_request_callback callback,
	void *userdata)
{
	#if LOG_REALIGN_TO_HMD != 0
		DriverLog("Begin CPSMoveTrackedDeviceLatest::RequestLatestHMDPose()\n");
	#endif

	assert(m_hmdResultCallback == nullptr || m_hmdResultCallback == callback);

	if (m_hmdResultCallback == nullptr)
	{
		bool bUsedCachedHMDPose;

		if (m_bIsLastHMDPoseValid)
		{
			std::chrono::duration<float, std::milli> hmdPoseAge =
				std::chrono::high_resolution_clock::now() - m_lastHMDPoseTime;

			bUsedCachedHMDPose = hmdPoseAge.count() < maxPoseAgeMilliseconds;
		}
		else
		{
			bUsedCachedHMDPose = false;
		}

		if (bUsedCachedHMDPose)
		{
			// Give the callback the cached pose immediately
			if (callback != nullptr)
			{
				callback(m_lastHMDPoseInMeters, userdata);
			}
		}
		else
		{
			static vr::VREvent_Data_t nodata = { 0 };

			// Register the callback
			m_hmdResultCallback = callback;
			m_hmdResultUserData = userdata;

			// Ask monitor_psmove to tell us the latest HMD pose
			m_pDriverHost->VendorSpecificEvent(
				m_unSteamVRTrackedDeviceId,
				(vr::EVREventType) (vr::VREvent_VendorSpecific_Reserved_Start + 0),
				nodata,
				0);
		}
	}
}

vr::DriverPose_t CPSMoveTrackedDeviceLatest::GetPose()
{
    // This is only called at startup to synchronize with the driver.
    // Future updates are driven by our thread calling TrackedDevicePoseUpdated()
    return m_Pose;
}

bool CPSMoveTrackedDeviceLatest::GetBoolTrackedDeviceProperty(
    vr::ETrackedDeviceProperty prop, 
    vr::ETrackedPropertyError * pError)
{
    bool bBoolResult = false;

    switch (prop)
    {
    // Not sure about this property yet
    //case vr::Prop_CanUnifyCoordinateSystemWithHmd_Bool:
    //    bBoolResult = true;
    //    *pError = vr::TrackedProp_Success;
    //    break;
    case vr::Prop_Firmware_UpdateAvailable_Bool:
    case vr::Prop_Firmware_ManualUpdate_Bool:
    case vr::Prop_ContainsProximitySensor_Bool:
    case vr::Prop_HasCamera_Bool:
    case vr::Prop_Firmware_ForceUpdateRequired_Bool:
    case vr::Prop_DeviceCanPowerOff_Bool:
        bBoolResult = false;
        *pError = vr::TrackedProp_Success;
        break;
    default:
        *pError = vr::TrackedProp_ValueNotProvidedByDevice;
    }

    return bBoolResult;
}

float CPSMoveTrackedDeviceLatest::GetFloatTrackedDeviceProperty(
    vr::ETrackedDeviceProperty prop, 
    vr::ETrackedPropertyError * pError)
{
    *pError = vr::TrackedProp_ValueNotProvidedByDevice;
    return 0.0f;
}

int32_t CPSMoveTrackedDeviceLatest::GetInt32TrackedDeviceProperty(
    vr::ETrackedDeviceProperty prop,
    vr::ETrackedPropertyError * pError)
{
    *pError = vr::TrackedProp_ValueNotProvidedByDevice;
    return 0;
}

uint64_t CPSMoveTrackedDeviceLatest::GetUint64TrackedDeviceProperty(
    vr::ETrackedDeviceProperty prop,
    vr::ETrackedPropertyError * pError)
{
    uint64_t ulRetVal = 0;

    switch (prop)
    {
    case vr::Prop_HardwareRevision_Uint64:
        ulRetVal = m_hardware_revision;
        *pError = vr::TrackedProp_Success;
        break;

    case vr::Prop_FirmwareVersion_Uint64:
        ulRetVal = m_firmware_revision;
        *pError = vr::TrackedProp_Success;
        break;
    default:
        *pError = vr::TrackedProp_ValueNotProvidedByDevice;
    }

    return ulRetVal;
}

vr::HmdMatrix34_t CPSMoveTrackedDeviceLatest::GetMatrix34TrackedDeviceProperty(
    vr::ETrackedDeviceProperty prop,
    vr::ETrackedPropertyError *pError)
{
    *pError = vr::TrackedProp_ValueNotProvidedByDevice;
    return vr::HmdMatrix34_t();
}

uint32_t CPSMoveTrackedDeviceLatest::GetStringTrackedDeviceProperty(
    vr::ETrackedDeviceProperty prop, 
    char * pchValue,
    uint32_t unBufferSize,
    vr::ETrackedPropertyError * pError)
{
    std::ostringstream ssRetVal;

    switch (prop)
    {
    case vr::Prop_SerialNumber_String:
        ssRetVal << m_strSerialNumber;
        break;

    case vr::Prop_ManufacturerName_String:
        ssRetVal << "Sony";
        break;

    case vr::Prop_ModelNumber_String:
        ssRetVal << "PSMove";
        break;

    case vr::Prop_TrackingFirmwareVersion_String:
        ssRetVal << "cd.firmware_revision=" << m_firmware_revision;
        break;

    case vr::Prop_HardwareRevision_String:
        ssRetVal << "cd.hardware_revision=" << m_hardware_revision;
        break;
    }

    std::string sRetVal = ssRetVal.str();
    if (sRetVal.empty())
    {
        *pError = vr::TrackedProp_ValueNotProvidedByDevice;
        return 0;
    }
    else if (sRetVal.size() + 1 > unBufferSize)
    {
        *pError = vr::TrackedProp_BufferTooSmall;
        return static_cast<uint32_t>(sRetVal.size() + 1);  // caller needs to know how to size buffer
    }
    else
    {
        snprintf(pchValue, unBufferSize, sRetVal.c_str());
        *pError = vr::TrackedProp_Success;
        return static_cast<uint32_t>(sRetVal.size() + 1);
    }
}

// CPSMoveTrackedDeviceLatest Interface
vr::ETrackedDeviceClass CPSMoveTrackedDeviceLatest::GetTrackedDeviceClass() const
{
    return vr::TrackedDeviceClass_Invalid;
}

bool CPSMoveTrackedDeviceLatest::IsActivated() const
{
    return m_unSteamVRTrackedDeviceId != vr::k_unTrackedDeviceIndexInvalid;
}

void CPSMoveTrackedDeviceLatest::Update()
{
    if (IsActivated() && m_properties_dirty)
    {
        m_pDriverHost->TrackedDevicePropertiesChanged(m_unSteamVRTrackedDeviceId);
        m_properties_dirty= false;
    }
}

void CPSMoveTrackedDeviceLatest::RefreshWorldFromDriverPose()
{
	#if LOG_REALIGN_TO_HMD != 0
		DriverLog( "Begin CServerDriver_PSMoveService::RefreshWorldFromDriverPose() for device %s\n", GetSerialNumber() );
	#endif

    const PSMovePose worldFromDriverPose = g_ServerTrackedDeviceProvider.GetWorldFromDriverPose();

	#if LOG_REALIGN_TO_HMD != 0
		DriverLog("worldFromDriverPose: %s \n", PsMovePoseToString(worldFromDriverPose).c_str());
	#endif
	

    // Transform used to convert from PSMove Tracking space to OpenVR Tracking Space
    m_Pose.qWorldFromDriverRotation.w = worldFromDriverPose.Orientation.w;
    m_Pose.qWorldFromDriverRotation.x = worldFromDriverPose.Orientation.x;
    m_Pose.qWorldFromDriverRotation.y = worldFromDriverPose.Orientation.y;
    m_Pose.qWorldFromDriverRotation.z = worldFromDriverPose.Orientation.z;
    m_Pose.vecWorldFromDriverTranslation[0] = worldFromDriverPose.Position.x;
    m_Pose.vecWorldFromDriverTranslation[1] = worldFromDriverPose.Position.y;
    m_Pose.vecWorldFromDriverTranslation[2] = worldFromDriverPose.Position.z;
}

const char *CPSMoveTrackedDeviceLatest::GetSerialNumber() const
{
    return m_strSerialNumber.c_str();
}

//==================================================================================================
// Controller Driver
//==================================================================================================

CPSMoveControllerLatest::CPSMoveControllerLatest( vr::IServerDriverHost * pDriverHost, int controllerId )
    : CPSMoveTrackedDeviceLatest(pDriverHost)
    , m_nControllerId(controllerId)
    , m_controller_view(nullptr)
    , m_nPoseSequenceNumber(0)
    , m_bIsBatteryCharging(false)
    , m_fBatteryChargeFraction(1.f)
	, m_bRumbleSuppressed(false)
    , m_pendingHapticPulseDuration(0)
    , m_lastTimeRumbleSent()
    , m_lastTimeRumbleSentValid(false)
{
    char buf[256];
    GenerateControllerSerialNumber(buf, sizeof(buf), controllerId);
    m_strSerialNumber = buf;

    // Tell psmoveapi that we are listening to this controller id
    m_controller_view = ClientPSMoveAPI::allocate_controller_view(controllerId);

    memset(&m_ControllerState, 0, sizeof(vr::VRControllerState_t));
	m_trackingStatus = vr::TrackingResult_Uninitialized;

    // Load config from steamvr.vrsettings
    vr::IVRSettings *pSettings= m_pDriverHost->GetSettings(vr::IVRSettings_Version);

    // Map every button to the system button initially
    memset(psButtonIDToVRButtonID, vr::k_EButton_System, k_EPSButtonID_Count*sizeof(vr::EVRButtonId));

	// Map every button to not be associated with any touchpad direction, initially
	memset(psButtonIDToVrTouchpadDirection, k_EVRTouchpadDirection_None, k_EPSButtonID_Count*sizeof(vr::EVRButtonId));

    // Load the button remapping from the settings for all possible controller buttons   
    LoadButtonMapping(pSettings, ePSButtonID::k_EPSButtonID_PS, vr::k_EButton_System, k_EVRTouchpadDirection_None);
    LoadButtonMapping(pSettings, ePSButtonID::k_EPSButtonID_Left, vr::k_EButton_DPad_Left, k_EVRTouchpadDirection_None);
    LoadButtonMapping(pSettings, ePSButtonID::k_EPSButtonID_Up, vr::k_EButton_DPad_Up, k_EVRTouchpadDirection_None);
    LoadButtonMapping(pSettings, ePSButtonID::k_EPSButtonID_Right, vr::k_EButton_DPad_Right, k_EVRTouchpadDirection_None);
    LoadButtonMapping(pSettings, ePSButtonID::k_EPSButtonID_Down, vr::k_EButton_DPad_Down, k_EVRTouchpadDirection_None);
    LoadButtonMapping(pSettings, ePSButtonID::k_EPSButtonID_Move, vr::k_EButton_SteamVR_Touchpad, k_EVRTouchpadDirection_None);
    LoadButtonMapping(pSettings, ePSButtonID::k_EPSButtonID_Trackpad, vr::k_EButton_SteamVR_Touchpad, k_EVRTouchpadDirection_None);
    LoadButtonMapping(pSettings, ePSButtonID::k_EPSButtonID_Trigger, vr::k_EButton_SteamVR_Trigger, k_EVRTouchpadDirection_None);
    LoadButtonMapping(pSettings, ePSButtonID::k_EPSButtonID_Triangle, (vr::EVRButtonId)8, k_EVRTouchpadDirection_None);
    LoadButtonMapping(pSettings, ePSButtonID::k_EPSButtonID_Square, (vr::EVRButtonId)9, k_EVRTouchpadDirection_None);
    LoadButtonMapping(pSettings, ePSButtonID::k_EPSButtonID_Circle, (vr::EVRButtonId)10, k_EVRTouchpadDirection_None);
    LoadButtonMapping(pSettings, ePSButtonID::k_EPSButtonID_Cross, (vr::EVRButtonId)11, k_EVRTouchpadDirection_None);
    LoadButtonMapping(pSettings, ePSButtonID::k_EPSButtonID_Select, vr::k_EButton_Grip, k_EVRTouchpadDirection_None);
    LoadButtonMapping(pSettings, ePSButtonID::k_EPSButtonID_Share, vr::k_EButton_ApplicationMenu, k_EVRTouchpadDirection_None);
    LoadButtonMapping(pSettings, ePSButtonID::k_EPSButtonID_Start, vr::k_EButton_ApplicationMenu, k_EVRTouchpadDirection_None);
    LoadButtonMapping(pSettings, ePSButtonID::k_EPSButtonID_Options, vr::k_EButton_ApplicationMenu, k_EVRTouchpadDirection_None);
    LoadButtonMapping(pSettings, ePSButtonID::k_EPSButtonID_L1, vr::k_EButton_SteamVR_Trigger, k_EVRTouchpadDirection_None);
    LoadButtonMapping(pSettings, ePSButtonID::k_EPSButtonID_L2, vr::k_EButton_SteamVR_Trigger, k_EVRTouchpadDirection_None);
    LoadButtonMapping(pSettings, ePSButtonID::k_EPSButtonID_L3, vr::k_EButton_Grip, k_EVRTouchpadDirection_None);
    LoadButtonMapping(pSettings, ePSButtonID::k_EPSButtonID_R1, vr::k_EButton_SteamVR_Trigger, k_EVRTouchpadDirection_None);
    LoadButtonMapping(pSettings, ePSButtonID::k_EPSButtonID_R2, vr::k_EButton_SteamVR_Trigger, k_EVRTouchpadDirection_None);
    LoadButtonMapping(pSettings, ePSButtonID::k_EPSButtonID_R3, vr::k_EButton_Grip, k_EVRTouchpadDirection_None);

	// Load the rumble settings
	m_bRumbleSuppressed= pSettings->GetBool("psmove", "rumble_suppressed", m_bRumbleSuppressed);

	// Grab the settings associated with mapping spatial movement to touchpad axes.
	if (pSettings != nullptr)
	{
		vr::EVRSettingsError fetchError;
		m_bUseSpatialOffsetAfterTouchpadPressAsTouchpadAxis
			= pSettings->GetBool("psmove", "use_spatial_offset_after_touchpad_press_as_touchpad_axis", true, &fetchError);

		m_fMetersPerTouchpadAxisUnits
			= pSettings->GetFloat("psmove", "meters_per_touchpad_units", 0.075f, &fetchError);

		#if LOG_TOUCHPAD_EMULATION != 0
			DriverLog("use_spatial_offset_after_touchpad_press_as_touchpad_axis: %d\n", m_bUseSpatialOffsetAfterTouchpadPressAsTouchpadAxis);
			DriverLog("meters_per_touchpad_units: %f\n", m_fMetersPerTouchpadAxisUnits);
		#endif


		m_fControllerMetersInFrontOfHmdAtCallibration
			= pSettings->GetFloat("psmove", "m_fControllerMetersInFrontOfHmdAtCallibration", 0.06f, &fetchError);

		#if LOG_REALIGN_TO_HMD != 0
			DriverLog("m_fControllerMetersInFrontOfHmdAtCallibration: %d\n", m_fControllerMetersInFrontOfHmdAtCallibration);
		#endif
	}

}

CPSMoveControllerLatest::~CPSMoveControllerLatest()
{
    ClientPSMoveAPI::free_controller_view(m_controller_view);
}

void CPSMoveControllerLatest::LoadButtonMapping(
    vr::IVRSettings *pSettings,
    const CPSMoveControllerLatest::ePSButtonID psButtonID,
    const vr::EVRButtonId defaultVRButtonID,
	const eVRTouchpadDirection defaultTouchpadDirection)
{

    vr::EVRButtonId vrButtonID = defaultVRButtonID;
	eVRTouchpadDirection vrTouchpadDirection = defaultTouchpadDirection;

    if (pSettings != nullptr)
    {
        const char *szPSButtonName = k_PSButtonNames[psButtonID];
        char remapButtonToButtonString[32];
        vr::EVRSettingsError fetchError;
        pSettings->GetString("psmove", szPSButtonName, remapButtonToButtonString, 32, "", &fetchError);

        if (fetchError == vr::VRSettingsError_None)
        {
            for (int vr_button_index = 0; vr_button_index < k_max_vr_buttons; ++vr_button_index)
            {
                if (strcasecmp(remapButtonToButtonString, k_VRButtonNames[vr_button_index]) == 0)
                {
                    vrButtonID = static_cast<vr::EVRButtonId>(vr_button_index);
                    break;
                }
            }
        }

		char remapButtonToTouchpadDirectionString[32];
		pSettings->GetString("psmove_touchpad_directions", szPSButtonName, remapButtonToTouchpadDirectionString, 32, "", &fetchError);

		if (fetchError == vr::VRSettingsError_None)
		{
			for (int vr_touchpad_direction_index = 0; vr_touchpad_direction_index < k_max_vr_touchpad_directions; ++vr_touchpad_direction_index)
			{
				if (strcasecmp(remapButtonToTouchpadDirectionString, k_VRTouchpadDirectionNames[vr_touchpad_direction_index]) == 0)
				{
					vrTouchpadDirection = static_cast<eVRTouchpadDirection>(vr_touchpad_direction_index);
					break;
				}
			}
		}
    }

    // Save the mapping
    psButtonIDToVRButtonID[psButtonID] = vrButtonID;
	psButtonIDToVrTouchpadDirection[psButtonID] = vrTouchpadDirection;
}

vr::EVRInitError CPSMoveControllerLatest::Activate(uint32_t unObjectId)
{
    vr::EVRInitError result = CPSMoveTrackedDeviceLatest::Activate(unObjectId);

    if (result == vr::VRInitError_None)
    {
		#if LOG_REALIGN_TO_HMD != 0
			DriverLog("CPSMoveControllerLatest::Activate(%d) -- calling g_ServerTrackedDeviceProvider.LaunchPSMoveMonitor()\n", unObjectId);
		#endif

		g_ServerTrackedDeviceProvider.LaunchPSMoveMonitor();

        ClientPSMoveAPI::register_callback(
            ClientPSMoveAPI::start_controller_data_stream(
                m_controller_view, 
                ClientPSMoveAPI::includePositionData | ClientPSMoveAPI::includePhysicsData),
            CPSMoveControllerLatest::start_controller_response_callback,
            this);
    }

    return result;
}

void CPSMoveControllerLatest::start_controller_response_callback(
    const ClientPSMoveAPI::ResponseMessage *response, void *userdata)
{
    CPSMoveControllerLatest *controller= reinterpret_cast<CPSMoveControllerLatest *>(userdata);

    if (response->result_code == ClientPSMoveAPI::_clientPSMoveResultCode_ok)
    {
        controller->m_properties_dirty= true;
    }
}

void CPSMoveControllerLatest::Deactivate()
{
    ClientPSMoveAPI::stop_controller_data_stream(m_controller_view);
}

void *CPSMoveControllerLatest::GetComponent(const char *pchComponentNameAndVersion)
{
    if (!strcasecmp(pchComponentNameAndVersion, vr::IVRControllerComponent_Version))
    {
        return (vr::IVRControllerComponent*)this;
    }

    return NULL;
}

bool CPSMoveControllerLatest::GetBoolTrackedDeviceProperty(
    vr::ETrackedDeviceProperty prop, 
    vr::ETrackedPropertyError * pError)
{
    bool bBoolResult = false;

    switch (prop)
    {
    case vr::Prop_WillDriftInYaw_Bool:
        bBoolResult = false;
        *pError = vr::TrackedProp_Success;
        break;
    case vr::Prop_DeviceIsWireless_Bool:
        bBoolResult = false;
        *pError = vr::TrackedProp_Success;
        break;
    case vr::Prop_DeviceIsCharging_Bool:
        bBoolResult = m_bIsBatteryCharging;
        *pError = vr::TrackedProp_Success;
        break;
    case vr::Prop_DeviceProvidesBatteryStatus_Bool:
        bBoolResult = true;
        *pError = vr::TrackedProp_Success;
        break;
    default:
        *pError = vr::TrackedProp_ValueNotProvidedByDevice;
    }

    if (*pError == vr::TrackedProp_ValueNotProvidedByDevice)
    {
        bBoolResult= CPSMoveTrackedDeviceLatest::GetBoolTrackedDeviceProperty(prop, pError);
    }

    return bBoolResult;
}

float CPSMoveControllerLatest::GetFloatTrackedDeviceProperty(
    vr::ETrackedDeviceProperty prop,
    vr::ETrackedPropertyError * pError)
{
    float floatResult = 0.f;

    switch (prop)
    {
    case vr::Prop_DeviceBatteryPercentage_Float: // 0 is empty, 1 is full
        floatResult = m_fBatteryChargeFraction;
        *pError = vr::TrackedProp_Success;
        break;
    default:
        *pError = vr::TrackedProp_ValueNotProvidedByDevice;
    }
    
    if (*pError == vr::TrackedProp_ValueNotProvidedByDevice)
    {
        floatResult = CPSMoveTrackedDeviceLatest::GetFloatTrackedDeviceProperty(prop, pError);
    }

    return floatResult;
}

int32_t CPSMoveControllerLatest::GetInt32TrackedDeviceProperty(
    vr::ETrackedDeviceProperty prop,
    vr::ETrackedPropertyError * pError)
{
    int32_t nRetVal = 0;

    switch ( prop )
    {
    case vr::Prop_DeviceClass_Int32:
        nRetVal = vr::TrackedDeviceClass_Controller;
        *pError = vr::TrackedProp_Success;
        break;

    case vr::Prop_Axis0Type_Int32:
        nRetVal = vr::k_eControllerAxis_Trigger;
        *pError = vr::TrackedProp_Success;
        break;

    default:
        *pError = vr::TrackedProp_ValueNotProvidedByDevice;
        break;
    }

    if (*pError == vr::TrackedProp_ValueNotProvidedByDevice)
    {
        nRetVal = CPSMoveTrackedDeviceLatest::GetInt32TrackedDeviceProperty(prop, pError);
    }

    return nRetVal;
}

uint64_t CPSMoveControllerLatest::GetUint64TrackedDeviceProperty( 
    vr::ETrackedDeviceProperty prop,
    vr::ETrackedPropertyError * pError)
{
    uint64_t ulRetVal = 0;

    switch ( prop )
    {
    case vr::Prop_SupportedButtons_Uint64:
		{
			for (int buttonIndex = 0; buttonIndex < static_cast<int>(k_EPSButtonID_Count); ++buttonIndex)
			{
				ulRetVal |= vr::ButtonMaskFromId( psButtonIDToVRButtonID[buttonIndex] );

				if( psButtonIDToVrTouchpadDirection[buttonIndex] != k_EVRTouchpadDirection_None )
				{
					ulRetVal |= vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Touchpad);
				}
			}
			*pError = vr::TrackedProp_Success;
			break;
		}

    default:
        *pError = vr::TrackedProp_ValueNotProvidedByDevice;
    }

    if (*pError == vr::TrackedProp_ValueNotProvidedByDevice)
    {
        ulRetVal = CPSMoveTrackedDeviceLatest::GetUint64TrackedDeviceProperty(prop, pError);
    }

    return ulRetVal;
}

uint32_t CPSMoveControllerLatest::GetStringTrackedDeviceProperty(
    vr::ETrackedDeviceProperty prop, 
    char * pchValue, 
    uint32_t unBufferSize, 
    vr::ETrackedPropertyError * pError)
{
    std::ostringstream ssRetVal;

    switch ( prop )
    {
    case vr::Prop_RenderModelName_String:
        // The {psmove} syntax lets us refer to rendermodels that are installed
        // in the driver's own resources/rendermodels directory.  The driver can
        // still refer to SteamVR models like "generic_hmd".
        switch(m_controller_view->GetControllerViewType())
        {
        case ClientControllerView::PSMove:
            ssRetVal << "{psmove}psmove_controller";
            break;
        case ClientControllerView::PSNavi:
            ssRetVal << "{psmove}navi_controller";
            break;
        case ClientControllerView::PSDualShock4:
            ssRetVal << "{psmove}dualshock4_controller";
            break;
        default:
            ssRetVal << "generic_controller";
        }
        break;
    case vr::Prop_TrackingSystemName_String:
        ssRetVal << "psmoveservice";
        break;
    }

    std::string sRetVal = ssRetVal.str();
    if ( sRetVal.empty() )
    {        
        return CPSMoveTrackedDeviceLatest::GetStringTrackedDeviceProperty(prop, pchValue, unBufferSize, pError);
    }
    else if ( sRetVal.size() + 1 > unBufferSize )
    {
        *pError = vr::TrackedProp_BufferTooSmall;
        return static_cast<uint32_t>(sRetVal.size() + 1);  // caller needs to know how to size buffer
    }
    else
    {
        snprintf( pchValue, unBufferSize, sRetVal.c_str() );
        *pError = vr::TrackedProp_Success;
        return static_cast<uint32_t>(sRetVal.size() + 1);
    }
}

vr::VRControllerState_t CPSMoveControllerLatest::GetControllerState()
{
    return m_ControllerState;
}

bool CPSMoveControllerLatest::TriggerHapticPulse( uint32_t unAxisId, uint16_t usPulseDurationMicroseconds )
{
    m_pendingHapticPulseDuration = usPulseDurationMicroseconds;
    UpdateRumbleState();

    return true;
}

void CPSMoveControllerLatest::SendButtonUpdates( ButtonUpdate ButtonEvent, uint64_t ulMask )
{
    if ( !ulMask )
        return;

    for ( int i = 0; i< vr::k_EButton_Max; i++ )
    {
        vr::EVRButtonId button = ( vr::EVRButtonId )i;

        uint64_t bit = ButtonMaskFromId( button );

        if ( bit & ulMask )
        {
            ( m_pDriverHost->*ButtonEvent )( m_unSteamVRTrackedDeviceId, button, 0.0 );
        }
    }
}

void CPSMoveControllerLatest::UpdateControllerState()
{
    assert(m_controller_view != nullptr);
    assert(m_controller_view->GetIsConnected());

    vr::VRControllerState_t NewState = { 0 };

    // Changing unPacketNum tells anyone polling state that something might have
    // changed.  We don't try to be precise about that here.
    NewState.unPacketNum = m_ControllerState.unPacketNum + 1;
   
    switch (m_controller_view->GetControllerViewType())
    {
    case ClientControllerView::eControllerType::PSMove:
        {
            const ClientPSMoveView &clientView = m_controller_view->GetPSMoveView();

            bool bStartWasPressed = (m_ControllerState.ulButtonPressed & vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Start])) > 0;
            bool bSelectWasPressed = (m_ControllerState.ulButtonPressed & vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Select])) > 0;

            bool bSquareWasPressed = (m_ControllerState.ulButtonPressed & vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Square])) > 0;
            bool bCrossWasPressed = (m_ControllerState.ulButtonPressed & vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Cross])) > 0;
            bool bTriangleWasPressed = (m_ControllerState.ulButtonPressed & vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Triangle])) > 0;
            bool bCircleWasPressed = (m_ControllerState.ulButtonPressed & vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Circle])) > 0;

            // If start was just pressed while and select was held or vice versa,
			// recenter the controller orientation pose and start the realignment of the controller to HMD tracking space.
            if ((clientView.GetButtonStart() == PSMoveButton_PRESSED && clientView.GetButtonSelect() == PSMoveButton_PRESSED) ||
				(clientView.GetButtonStart() == PSMoveButton_PRESSED && clientView.GetButtonSelect() == PSMoveButton_DOWN) ||
				(clientView.GetButtonStart() == PSMoveButton_DOWN && clientView.GetButtonSelect() == PSMoveButton_PRESSED) )
                
            {
				#if LOG_REALIGN_TO_HMD != 0
					DriverLog("CPSMoveControllerLatest::UpdateControllerState(): Calling StartRealignHMDTrackingSpace() in response to controller chord.\n");
				#endif

				PSMoveFloatVector3 controllerBallPointedUpEuler = PSMoveFloatVector3::create((float)M_PI_2, 0.0f, 0.0f);
				PSMoveQuaternion controllerBallPointedUpQuat = PSMoveQuaternion::create(controllerBallPointedUpEuler);
				ClientPSMoveAPI::reset_pose(m_controller_view, controllerBallPointedUpQuat);

				StartRealignHMDTrackingSpace();
            }

			UpdateControllerStateFromPsMoveButtonState(k_EPSButtonID_Circle, clientView.GetButtonCircle(), &NewState);
			UpdateControllerStateFromPsMoveButtonState(k_EPSButtonID_Cross, clientView.GetButtonCross(), &NewState);
			UpdateControllerStateFromPsMoveButtonState(k_EPSButtonID_Move, clientView.GetButtonMove(), &NewState);
			UpdateControllerStateFromPsMoveButtonState(k_EPSButtonID_PS, clientView.GetButtonPS(), &NewState);
			UpdateControllerStateFromPsMoveButtonState(k_EPSButtonID_Select, clientView.GetButtonSelect(), &NewState);
			UpdateControllerStateFromPsMoveButtonState(k_EPSButtonID_Square, clientView.GetButtonSquare(), &NewState);
			UpdateControllerStateFromPsMoveButtonState(k_EPSButtonID_Start, clientView.GetButtonStart(), &NewState);
			UpdateControllerStateFromPsMoveButtonState(k_EPSButtonID_Triangle, clientView.GetButtonTriangle(), &NewState);
			UpdateControllerStateFromPsMoveButtonState(k_EPSButtonID_Trigger, clientView.GetButtonTrigger(), &NewState);


			if (m_bUseSpatialOffsetAfterTouchpadPressAsTouchpadAxis)
			{
				static const uint64_t s_kTouchpadButtonMask = vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Touchpad);
				if (NewState.ulButtonPressed & s_kTouchpadButtonMask)
				{
					if (!(m_ControllerState.ulButtonPressed & s_kTouchpadButtonMask))
					{
						// Just pressed.
						const ClientPSMoveView &view = m_controller_view->GetPSMoveView();
						m_driverSpaceRotationAtTouchpadPressTime = view.GetOrientation();

						GetMetersPosInRotSpace(&m_posMetersAtTouchpadPressTime, m_driverSpaceRotationAtTouchpadPressTime);

						#if LOG_TOUCHPAD_EMULATION != 0
							DriverLog("Touchpad pressed! At (%f, %f, %f) meters relative to orientation\n",
								m_posMetersAtTouchpadPressTime.i, m_posMetersAtTouchpadPressTime.j, m_posMetersAtTouchpadPressTime.k);
						#endif
					}
					else
					{
						// Held!
						PSMoveFloatVector3 newPosMeters;
						GetMetersPosInRotSpace(&newPosMeters, m_driverSpaceRotationAtTouchpadPressTime);

						PSMoveFloatVector3 offsetMeters = newPosMeters - m_posMetersAtTouchpadPressTime;

						#if LOG_TOUCHPAD_EMULATION != 0
							DriverLog("Touchpad held! Relative position (%f, %f, %f) meters\n",
								offsetMeters.i, offsetMeters.j, offsetMeters.k);
						#endif

						NewState.rAxis[0].x = offsetMeters.i / m_fMetersPerTouchpadAxisUnits;
						NewState.rAxis[0].x = fminf( fmaxf(NewState.rAxis[0].x, -1.0f), 1.0f );

						NewState.rAxis[0].y = -offsetMeters.k / m_fMetersPerTouchpadAxisUnits;
						NewState.rAxis[0].y = fminf(fmaxf(NewState.rAxis[0].y, -1.0f), 1.0f);

						#if LOG_TOUCHPAD_EMULATION != 0
						DriverLog("Touchpad axis at (%f, %f) \n",
							NewState.rAxis[0].x, NewState.rAxis[0].y);
						#endif
					}
				}
			}


			if (NewState.rAxis[0].x != m_ControllerState.rAxis[0].x || NewState.rAxis[0].y != m_ControllerState.rAxis[0].y)
				m_pDriverHost->TrackedDeviceAxisUpdated(m_unSteamVRTrackedDeviceId, 0, NewState.rAxis[0]);


            NewState.rAxis[1].x = clientView.GetTriggerValue();
            NewState.rAxis[1].y = 0.f;

			if (NewState.rAxis[1].x != m_ControllerState.rAxis[1].x)
                m_pDriverHost->TrackedDeviceAxisUpdated(m_unSteamVRTrackedDeviceId, 1, NewState.rAxis[1]);

        } break;
    case ClientControllerView::eControllerType::PSNavi:
        {
            const ClientPSNaviView &clientView = m_controller_view->GetPSNaviView();

            if (clientView.GetButtonL1())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_L1]);
            if (clientView.GetButtonL2())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_L2]);
            if (clientView.GetButtonL3())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_L3]);
            if (clientView.GetButtonCircle())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Circle]);
            if (clientView.GetButtonCross())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Cross]);
            if (clientView.GetButtonPS())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_PS]);
            if (clientView.GetButtonTrigger())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Trigger]);
            if (clientView.GetButtonDPadUp())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Up]);
            if (clientView.GetButtonDPadDown())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Down]);
            if (clientView.GetButtonDPadLeft())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Left]);
            if (clientView.GetButtonDPadRight())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Right]);

            NewState.rAxis[0].x = clientView.GetStickXAxis();
            NewState.rAxis[0].y = clientView.GetStickYAxis();

            NewState.rAxis[1].x = clientView.GetTriggerValue();
            NewState.rAxis[1].y = 0.f;

            if (NewState.rAxis[0].x != m_ControllerState.rAxis[0].x || NewState.rAxis[0].y != m_ControllerState.rAxis[0].y)
                m_pDriverHost->TrackedDeviceAxisUpdated(m_unSteamVRTrackedDeviceId, 0, NewState.rAxis[0]);
            if (NewState.rAxis[1].x != m_ControllerState.rAxis[1].x)
                m_pDriverHost->TrackedDeviceAxisUpdated(m_unSteamVRTrackedDeviceId, 1, NewState.rAxis[1]);
        } break;
    case ClientControllerView::eControllerType::PSDualShock4:
        {
            const ClientPSDualShock4View &clientView = m_controller_view->GetPSDualShock4View();

            if (clientView.GetButtonL1())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_L1]);
            if (clientView.GetButtonL2())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_L2]);
            if (clientView.GetButtonL3())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_L3]);
            if (clientView.GetButtonR1())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_R1]);
            if (clientView.GetButtonR2())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_R2]);
            if (clientView.GetButtonR3())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_R3]);

            if (clientView.GetButtonCircle())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Circle]);
            if (clientView.GetButtonCross())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Cross]);
            if (clientView.GetButtonSquare())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Square]);
            if (clientView.GetButtonTriangle())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Triangle]);

            if (clientView.GetButtonDPadUp())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Up]);
            if (clientView.GetButtonDPadDown())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Down]);
            if (clientView.GetButtonDPadLeft())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Left]);
            if (clientView.GetButtonDPadRight())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Right]);

            if (clientView.GetButtonOptions())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Options]);
            if (clientView.GetButtonShare())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Share]);
            if (clientView.GetButtonTrackpad())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_Trackpad]);
            if (clientView.GetButtonPS())
                NewState.ulButtonPressed |= vr::ButtonMaskFromId(psButtonIDToVRButtonID[k_EPSButtonID_PS]);

            NewState.rAxis[0].x = clientView.GetLeftAnalogX();
            NewState.rAxis[0].y = -clientView.GetLeftAnalogY();

            NewState.rAxis[1].x = clientView.GetLeftTriggerValue();
            NewState.rAxis[1].y = 0.f;

            NewState.rAxis[2].x = clientView.GetRightAnalogX();
            NewState.rAxis[2].y = -clientView.GetRightAnalogY();

            NewState.rAxis[3].x = clientView.GetRightTriggerValue();
            NewState.rAxis[3].y = 0.f;

            if (NewState.rAxis[0].x != m_ControllerState.rAxis[0].x || NewState.rAxis[0].y != m_ControllerState.rAxis[0].y)
                m_pDriverHost->TrackedDeviceAxisUpdated(m_unSteamVRTrackedDeviceId, 0, NewState.rAxis[0]);
            if (NewState.rAxis[1].x != m_ControllerState.rAxis[1].x)
                m_pDriverHost->TrackedDeviceAxisUpdated(m_unSteamVRTrackedDeviceId, 1, NewState.rAxis[1]);

            if (NewState.rAxis[2].x != m_ControllerState.rAxis[2].x || NewState.rAxis[2].y != m_ControllerState.rAxis[2].y)
                m_pDriverHost->TrackedDeviceAxisUpdated(m_unSteamVRTrackedDeviceId, 2, NewState.rAxis[2]);
            if (NewState.rAxis[3].x != m_ControllerState.rAxis[3].x)
                m_pDriverHost->TrackedDeviceAxisUpdated(m_unSteamVRTrackedDeviceId, 3, NewState.rAxis[3]);
        } break;
    }

    // All pressed buttons are touched
    NewState.ulButtonTouched |= NewState.ulButtonPressed;

    uint64_t ulChangedTouched = NewState.ulButtonTouched ^ m_ControllerState.ulButtonTouched;
    uint64_t ulChangedPressed = NewState.ulButtonPressed ^ m_ControllerState.ulButtonPressed;

    SendButtonUpdates( &vr::IServerDriverHost::TrackedDeviceButtonTouched, ulChangedTouched & NewState.ulButtonTouched );
    SendButtonUpdates( &vr::IServerDriverHost::TrackedDeviceButtonPressed, ulChangedPressed & NewState.ulButtonPressed );
    SendButtonUpdates( &vr::IServerDriverHost::TrackedDeviceButtonUnpressed, ulChangedPressed & ~NewState.ulButtonPressed );
    SendButtonUpdates( &vr::IServerDriverHost::TrackedDeviceButtonUntouched, ulChangedTouched & ~NewState.ulButtonTouched );

    m_ControllerState = NewState;
}


void CPSMoveControllerLatest::UpdateControllerStateFromPsMoveButtonState(ePSButtonID buttonId, PSMoveButtonState buttonState, vr::VRControllerState_t* pControllerStateToUpdate)
{
	if (buttonState & PSMoveButton_PRESSED || buttonState & PSMoveButton_DOWN)
	{
		pControllerStateToUpdate->ulButtonPressed |= vr::ButtonMaskFromId( psButtonIDToVRButtonID[buttonId] );

		if (psButtonIDToVrTouchpadDirection[buttonId] == k_EVRTouchpadDirection_Left)
		{
			pControllerStateToUpdate->rAxis[0].x = -1.0f;
			pControllerStateToUpdate->ulButtonPressed |= vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Touchpad);
		}
		else if (psButtonIDToVrTouchpadDirection[buttonId] == k_EVRTouchpadDirection_Right)
		{
			pControllerStateToUpdate->rAxis[0].x = 1.0f;
			pControllerStateToUpdate->ulButtonPressed |= vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Touchpad);
		}
		else if (psButtonIDToVrTouchpadDirection[buttonId] == k_EVRTouchpadDirection_Up)
		{
			pControllerStateToUpdate->rAxis[0].y = 1.0f;
			pControllerStateToUpdate->ulButtonPressed |= vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Touchpad);
		}
		else if (psButtonIDToVrTouchpadDirection[buttonId] == k_EVRTouchpadDirection_Down)
		{
			pControllerStateToUpdate->rAxis[0].y = -1.0f;
			pControllerStateToUpdate->ulButtonPressed |= vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Touchpad);
		}
	}
}


PSMoveQuaternion ExtractHMDYawQuaternion(const PSMoveQuaternion &q)
{
    // Convert the quaternion to a basis matrix
    const PSMoveMatrix3x3 hmd_orientation = PSMoveMatrix3x3::create(q);

    // Extract the forward (z-axis) vector from the basis
	const PSMoveFloatVector3 &global_forward = *k_psmove_float_vector3_k;
    const PSMoveFloatVector3 forward = hmd_orientation.basis_z();
	PSMoveFloatVector3 forward2d = PSMoveFloatVector3::create(forward.i, 0.f, forward.k);
	forward2d.normalize_with_default(global_forward);

    // Compute the yaw angle (amount the z-axis has been rotated to it's current facing)
    const float cos_yaw = PSMoveFloatVector3::dot(forward, global_forward);
    float half_yaw = acosf(fminf(fmaxf(cos_yaw, -1.f), 1.f)) / 2.f;

	// Flip the sign of the yaw angle depending on if forward2d is to the left or right of global forward
	const PSMoveFloatVector3 &global_up = *k_psmove_float_vector3_j;
	PSMoveFloatVector3 yaw_axis = PSMoveFloatVector3::cross(global_forward, forward2d);
	if (PSMoveFloatVector3::dot(yaw_axis, global_up) < 0)
	{
		half_yaw = -half_yaw;
	}

    // Convert this yaw rotation back into a quaternion
    PSMoveQuaternion yaw_quaternion =
        PSMoveQuaternion::create(
            cosf(half_yaw), // w = cos(theta/2)
            0.f, sinf(half_yaw), 0.f); // (x, y, z) = sin(theta/2)*axis, where axis = (0, 1, 0)

    return yaw_quaternion;
}

PSMoveQuaternion ExtractPSMoveYawQuaternion(const PSMoveQuaternion &q)
{
	// Convert the quaternion to a basis matrix
	const PSMoveMatrix3x3 psmove_basis = PSMoveMatrix3x3::create(q);

	// Extract the forward (negative z-axis) vector from the basis
	const PSMoveFloatVector3 global_forward = PSMoveFloatVector3::create(0.f, 0.f, -1.f);
	const PSMoveFloatVector3 &forward = psmove_basis.basis_y();
	PSMoveFloatVector3 forward2d = PSMoveFloatVector3::create(forward.i, 0.f, forward.k);
	forward2d.normalize_with_default(global_forward);

	// Compute the yaw angle (amount the z-axis has been rotated to it's current facing)
	const float cos_yaw = PSMoveFloatVector3::dot(forward, global_forward);
	float yaw = acosf(fminf(fmaxf(cos_yaw, -1.f), 1.f));

	// Flip the sign of the yaw angle depending on if forward2d is to the left or right of global forward
	const PSMoveFloatVector3 &global_up = *k_psmove_float_vector3_j;
	PSMoveFloatVector3 yaw_axis = PSMoveFloatVector3::cross(global_forward, forward2d);
	if (PSMoveFloatVector3::dot(yaw_axis, global_up) < 0)
	{
		yaw = -yaw;
	}

	// Convert this yaw rotation back into a quaternion
	PSMoveQuaternion yaw_quaternion =
		PSMoveQuaternion::concat(
			PSMoveQuaternion::create(PSMoveFloatVector3::create((float)1.57079632679489661923, 0.f, 0.f)), // pitch 90 up first
			PSMoveQuaternion::create(PSMoveFloatVector3::create(0, yaw, 0))); // Then apply the yaw

	return yaw_quaternion;
}

void CPSMoveControllerLatest::GetMetersPosInRotSpace(PSMoveFloatVector3* pOutPosition, const PSMoveQuaternion& rRotation )
{
	const ClientPSMoveView &view = m_controller_view->GetPSMoveView();

	const PSMovePosition &position = view.GetPosition();

	PSMoveFloatVector3 unrotatedPositionMeters
		= PSMoveFloatVector3::create(
			position.x * k_fScalePSMoveAPIToMeters,
			position.y * k_fScalePSMoveAPIToMeters,
			position.z * k_fScalePSMoveAPIToMeters);

	PSMoveQuaternion viewOrientationInverse	= rRotation.inverse();

	*pOutPosition	= viewOrientationInverse.rotate_vector(unrotatedPositionMeters);
}

void CPSMoveControllerLatest::StartRealignHMDTrackingSpace()
{
	#if LOG_REALIGN_TO_HMD != 0
		DriverLog( "Begin CPSMoveControllerLatest::StartRealignHMDTrackingSpace()\n" );
	#endif

	if (m_trackingStatus != vr::TrackingResult_Calibrating_InProgress)
	{
		m_trackingStatus = vr::TrackingResult_Calibrating_InProgress;
		RequestLatestHMDPose(0.f, CPSMoveControllerLatest::FinishRealignHMDTrackingSpace, this);
	}
}

void CPSMoveControllerLatest::FinishRealignHMDTrackingSpace(
	const PSMovePose &hmd_pose_raw_meters, 
	void *userdata)
{

	CPSMoveControllerLatest* pThis = (CPSMoveControllerLatest*)userdata;

	#if LOG_REALIGN_TO_HMD != 0
		DriverLog("Begin CPSMoveControllerLatest::FinishRealignHMDTrackingSpace()\n");
	#endif

	PSMovePose hmd_pose_meters = hmd_pose_raw_meters;
	DriverLog("hmd_pose_meters(raw): %s \n", PsMovePoseToString(hmd_pose_meters).c_str());

	// Make the HMD orientation only contain a yaw
	hmd_pose_meters.Orientation = ExtractHMDYawQuaternion(hmd_pose_raw_meters.Orientation);
	DriverLog("hmd_pose_meters(yaw-only): %s \n", PsMovePoseToString(hmd_pose_meters).c_str());

	// We have the transform of the HMD in world space. It and the controller aren't quite
	// aligned -- the controller's local -Z axis (from the center to the glowing ball) is currently pointed 
	// in the direction of the HMD's local +Y axis, and the controller's position is a few inches ahead of
	// the HMD's on the HMD's local -Z axis. Transform the HMD's world space transform to where we expect the
	// controller's world space transform to be.

	PSMovePosition controllerLocalOffsetFromHmdPosition
		= PSMovePosition::create(0.0f, 0.0f, -1.0f * pThis->m_fControllerMetersInFrontOfHmdAtCallibration);
	PSMoveQuaternion controllerOrientationInHmdSpaceQuat =
		PSMoveQuaternion::create(PSMoveFloatVector3::create((float)M_PI_2, 0.0f, 0.0f));
	PSMovePose controllerPoseRelativeToHMD =
		PSMovePose::create(controllerLocalOffsetFromHmdPosition, controllerOrientationInHmdSpaceQuat);
	DriverLog("controllerPoseRelativeToHMD: %s \n", PsMovePoseToString(controllerPoseRelativeToHMD).c_str());

	// Compute the expected psmove controller pose in HMD tracking space (i.e. "World Space")
	PSMovePose controller_world_space_pose = PSMovePose::concat(controllerPoseRelativeToHMD, hmd_pose_meters);
	DriverLog("controller_world_space_pose: %s \n", PsMovePoseToString(controller_world_space_pose).c_str());


	// We now have the transform of the controller in world space -- controller_world_space_pose

	// We also have the transform of the controller in driver space -- psmove_pose_meters

	// We need the transform that goes from driver space to world space -- driver_pose_to_world_pose
	// psmove_pose_meters * driver_pose_to_world_pose = controller_world_space_pose
	// psmove_pose_meters.inverse() * psmove_pose_meters * driver_pose_to_world_pose = psmove_pose_meters.inverse() * controller_world_space_pose
	// driver_pose_to_world_pose = psmove_pose_meters.inverse() * controller_world_space_pose

	CPSMoveControllerLatest *controller = reinterpret_cast<CPSMoveControllerLatest *>(userdata);

	// Get the current pose from the controller view instead of using the driver's cached
	// value because the user may have triggered a pose reset, in which case the driver's
	// cached pose might not yet be up to date by the time this callback is triggered.
	PSMovePose psmove_pose_meters = controller->m_controller_view->GetPSMoveView().GetPose();
	DriverLog("psmove_pose_meters(raw): %s \n", PsMovePoseToString(psmove_pose_meters).c_str());

	// PSMove Position is in cm, but OpenVR stores position in meters
	psmove_pose_meters.Position.x *= k_fScalePSMoveAPIToMeters;
	psmove_pose_meters.Position.y *= k_fScalePSMoveAPIToMeters;
	psmove_pose_meters.Position.z *= k_fScalePSMoveAPIToMeters;

	// Snap the controller to a +90 degree pitch so that we get a clean yaw
	psmove_pose_meters.Orientation = ExtractPSMoveYawQuaternion(psmove_pose_meters.Orientation);
	DriverLog("psmove_pose_meters(yaw-only): %s \n", PsMovePoseToString(psmove_pose_meters).c_str());

	PSMovePose psmove_pose_inv = psmove_pose_meters.inverse();
	DriverLog("psmove_pose_inv: %s \n", PsMovePoseToString(psmove_pose_inv).c_str());

	PSMovePose driver_pose_to_world_pose = PSMovePose::concat(psmove_pose_inv, controller_world_space_pose);
	DriverLog("driver_pose_to_world_pose: %s \n", PsMovePoseToString(driver_pose_to_world_pose).c_str());

	//PSMovePose test_composed_controller_world_space = PSMovePose::concat(psmove_pose_meters, driver_pose_to_world_pose);
	//DriverLog("test_composed_controller_world_space: %s \n", PsMovePoseToString(test_composed_controller_world_space).c_str());

	g_ServerTrackedDeviceProvider.SetHMDTrackingSpace(driver_pose_to_world_pose);
}

void CPSMoveControllerLatest::UpdateTrackingState()
{
    assert(m_controller_view != nullptr);
    assert(m_controller_view->GetIsConnected());

	// The tracking status will be one of the following states:
    m_Pose.result = m_trackingStatus;

    m_Pose.deviceIsConnected = m_controller_view->GetIsConnected();

    // These should always be false from any modern driver.  These are for Oculus DK1-like
    // rotation-only tracking.  Support for that has likely rotted in vrserver.
    m_Pose.willDriftInYaw = false;
    m_Pose.shouldApplyHeadModel = false;

    switch (m_controller_view->GetControllerViewType())
    {
    case ClientControllerView::eControllerType::PSMove:
        {
            const ClientPSMoveView &view= m_controller_view->GetPSMoveView();

            // No prediction since that's already handled in the psmove service
            m_Pose.poseTimeOffset = 0.f;

            // No transform due to the current HMD orientation
            m_Pose.qDriverFromHeadRotation.w = 1.f;
            m_Pose.qDriverFromHeadRotation.x = 0.0f;
            m_Pose.qDriverFromHeadRotation.y = 0.0f;
            m_Pose.qDriverFromHeadRotation.z = 0.0f;
            m_Pose.vecDriverFromHeadTranslation[0] = 0.f;
            m_Pose.vecDriverFromHeadTranslation[1] = 0.f;
            m_Pose.vecDriverFromHeadTranslation[2] = 0.f;            

            // Set position
            {
                const PSMovePosition &position = view.GetPosition();

                m_Pose.vecPosition[0] = position.x * k_fScalePSMoveAPIToMeters;
                m_Pose.vecPosition[1] = position.y * k_fScalePSMoveAPIToMeters;
                m_Pose.vecPosition[2] = position.z * k_fScalePSMoveAPIToMeters;
            }

            // Set rotational coordinates
            {
                const PSMoveQuaternion &orientation = view.GetOrientation();

                m_Pose.qRotation.w = orientation.w;
                m_Pose.qRotation.x = orientation.x;
                m_Pose.qRotation.y = orientation.y;
                m_Pose.qRotation.z = orientation.z;
            }

            // Set the physics state of the controller
            {
                const PSMovePhysicsData &physicsData= view.GetPhysicsData();

                m_Pose.vecVelocity[0] = physicsData.Velocity.i * k_fScalePSMoveAPIToMeters;
                m_Pose.vecVelocity[1] = physicsData.Velocity.j * k_fScalePSMoveAPIToMeters;
                m_Pose.vecVelocity[2] = physicsData.Velocity.k * k_fScalePSMoveAPIToMeters;

                m_Pose.vecAcceleration[0] = physicsData.Acceleration.i * k_fScalePSMoveAPIToMeters;
                m_Pose.vecAcceleration[1] = physicsData.Acceleration.j * k_fScalePSMoveAPIToMeters;
                m_Pose.vecAcceleration[2] = physicsData.Acceleration.k * k_fScalePSMoveAPIToMeters;

                m_Pose.vecAngularVelocity[0] = physicsData.AngularVelocity.i;
                m_Pose.vecAngularVelocity[1] = physicsData.AngularVelocity.j;
                m_Pose.vecAngularVelocity[2] = physicsData.AngularVelocity.k;

                m_Pose.vecAngularAcceleration[0] = physicsData.AngularAcceleration.i;
                m_Pose.vecAngularAcceleration[1] = physicsData.AngularAcceleration.j;
                m_Pose.vecAngularAcceleration[2] = physicsData.AngularAcceleration.k;
            }

            m_Pose.poseIsValid = m_controller_view->GetIsPoseValid();

            // This call posts this pose to shared memory, where all clients will have access to it the next
            // moment they want to predict a pose.
            m_pDriverHost->TrackedDevicePoseUpdated(m_unSteamVRTrackedDeviceId, m_Pose);
        } break;
    case ClientControllerView::eControllerType::PSNavi:
        {
            m_Pose.poseIsValid = false;

            // Don't post anything to the driver host
        } break;
    case ClientControllerView::eControllerType::PSDualShock4:
        {
            const ClientPSDualShock4View &view = m_controller_view->GetPSDualShock4View();

            // No prediction since that's already handled in the psmove service
            m_Pose.poseTimeOffset = 0.f;

            // Rotate -90 degrees about the x-axis from the current HMD orientation
            m_Pose.qDriverFromHeadRotation.w = 0.707107;
            m_Pose.qDriverFromHeadRotation.x = -0.707107;
            m_Pose.qDriverFromHeadRotation.y = 0.0f;
            m_Pose.qDriverFromHeadRotation.z = 0.0f;
            m_Pose.vecDriverFromHeadTranslation[0] = 0.f;
            m_Pose.vecDriverFromHeadTranslation[1] = 0.f;
            m_Pose.vecDriverFromHeadTranslation[2] = 0.f;

            // Set position
            {
                const PSMovePosition &position = view.GetPosition();

                m_Pose.vecPosition[0] = position.x * k_fScalePSMoveAPIToMeters;
                m_Pose.vecPosition[1] = position.y * k_fScalePSMoveAPIToMeters;
                m_Pose.vecPosition[2] = position.z * k_fScalePSMoveAPIToMeters;
            }

            // Set rotational coordinates
            {
                const PSMoveQuaternion &orientation = view.GetOrientation();

                m_Pose.qRotation.w = orientation.w;
                m_Pose.qRotation.x = orientation.x;
                m_Pose.qRotation.y = orientation.y;
                m_Pose.qRotation.z = orientation.z;
            }

            // Set the physics state of the controller
            // TODO: Physics data is too noisy for the DS4 right now, causes jitter
            {
                const PSMovePhysicsData &physicsData = view.GetPhysicsData();

                m_Pose.vecVelocity[0] = 0.f; // physicsData.Velocity.i * k_fScalePSMoveAPIToMeters;
                m_Pose.vecVelocity[1] = 0.f; // physicsData.Velocity.j * k_fScalePSMoveAPIToMeters;
                m_Pose.vecVelocity[2] = 0.f; // physicsData.Velocity.k * k_fScalePSMoveAPIToMeters;

                m_Pose.vecAcceleration[0] = 0.f; // physicsData.Acceleration.i * k_fScalePSMoveAPIToMeters;
                m_Pose.vecAcceleration[1] = 0.f; // physicsData.Acceleration.j * k_fScalePSMoveAPIToMeters;
                m_Pose.vecAcceleration[2] = 0.f; // physicsData.Acceleration.k * k_fScalePSMoveAPIToMeters;

                m_Pose.vecAngularVelocity[0] = 0.f; // physicsData.AngularVelocity.i;
                m_Pose.vecAngularVelocity[1] = 0.f; // physicsData.AngularVelocity.j;
                m_Pose.vecAngularVelocity[2] = 0.f; // physicsData.AngularVelocity.k;

                m_Pose.vecAngularAcceleration[0] = 0.f; // physicsData.AngularAcceleration.i;
                m_Pose.vecAngularAcceleration[1] = 0.f; // physicsData.AngularAcceleration.j;
                m_Pose.vecAngularAcceleration[2] = 0.f; // physicsData.AngularAcceleration.k;
            }

            m_Pose.poseIsValid = m_controller_view->GetIsPoseValid();

            // This call posts this pose to shared memory, where all clients will have access to it the next
            // moment they want to predict a pose.
            m_pDriverHost->TrackedDevicePoseUpdated(m_unSteamVRTrackedDeviceId, m_Pose);
        } break;
    }
}

void CPSMoveControllerLatest::UpdateRumbleState()
{
	if (!m_bRumbleSuppressed)
	{
		const float k_max_rumble_update_rate = 33.f; // Don't bother trying to update the rumble faster than 30fps (33ms)
		const float k_max_pulse_microseconds = 1000.f; // Docs suggest max pulse duration of 5ms, but we'll call 1ms max

		std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
		bool bTimoutElapsed = true;

		if (m_lastTimeRumbleSentValid)
		{
			std::chrono::duration<double, std::milli> timeSinceLastSend = now - m_lastTimeRumbleSent;

			bTimoutElapsed = timeSinceLastSend.count() >= k_max_rumble_update_rate;
		}

		// See if a rumble request hasn't come too recently
		if (bTimoutElapsed)
		{
			float rumble_fraction = static_cast<float>(m_pendingHapticPulseDuration) / k_max_pulse_microseconds;

			// Unless a zero rumble intensity was explicitly set, 
			// don't rumble less than 35% (no enough to feel)
			if (m_pendingHapticPulseDuration != 0)
			{
				if (rumble_fraction < 0.35f)
				{
					// rumble values less 35% isn't noticeable
					rumble_fraction = 0.35f;
				}
			}

			// Keep the pulse intensity within reasonable bounds
			if (rumble_fraction > 1.f)
			{
				rumble_fraction = 1.f;
			}

			// Actually send the rumble to the server
			switch (m_controller_view->GetControllerViewType())
			{
			case ClientControllerView::PSMove:
				m_controller_view->GetPSMoveViewMutable().SetRumble(rumble_fraction);
				break;
			case ClientControllerView::PSNavi:
				break;
			case ClientControllerView::PSDualShock4:
				m_controller_view->GetPSDualShock4ViewMutable().SetBigRumble(rumble_fraction);
				break;
			default:
				assert(0 && "Unreachable");
			}

			// Remember the last rumble we went and when we sent it
			m_lastTimeRumbleSent = now;
			m_lastTimeRumbleSentValid = true;

			// Reset the pending haptic pulse duration.
			// If another call to TriggerHapticPulse() is made later, it will stomp this value.
			// If no call to TriggerHapticPulse() is made later, then the next call to UpdateRumbleState()
			// in k_max_rumble_update_rate milliseconds will set the rumble_fraction to 0.f
			// This effectively makes the shortest rumble pulse k_max_rumble_update_rate milliseconds.
			m_pendingHapticPulseDuration = 0;
		}
	}
	else
	{
		// Reset the pending haptic pulse duration since rumble is suppressed.
		m_pendingHapticPulseDuration = 0;
	}
}

bool CPSMoveControllerLatest::HasControllerId( int ControllerID )
{
    return ControllerID == m_nControllerId;
}

void CPSMoveControllerLatest::Update()
{
    CPSMoveTrackedDeviceLatest::Update();

    if (IsActivated() && m_controller_view->GetIsConnected())
    {
        int seq_num= m_controller_view->GetOutputSequenceNum();

        // Only other updating incoming state if it actually changed
        if (m_nPoseSequenceNumber != seq_num)
        {
            m_nPoseSequenceNumber = seq_num;

            UpdateTrackingState();
            UpdateControllerState();
        }

        // Update the outgoing state
        UpdateRumbleState();
    }
}

void CPSMoveControllerLatest::RefreshWorldFromDriverPose()
{
	CPSMoveTrackedDeviceLatest::RefreshWorldFromDriverPose();

	// Mark the calibration process as done
	// once we have setup the world from driver pose
	m_trackingStatus = vr::TrackingResult_Running_OK;
}

//==================================================================================================
// Tracker Driver
//==================================================================================================

CPSMoveTrackerLatest::CPSMoveTrackerLatest(vr::IServerDriverHost * pDriverHost, const ClientTrackerInfo &trackerInfo)
    : CPSMoveTrackedDeviceLatest(pDriverHost)
    , m_nTrackerId(trackerInfo.tracker_id)
{
    char buf[256];
    GenerateTrackerSerialNumber(buf, sizeof(buf), trackerInfo.tracker_id);
    m_strSerialNumber = buf;

    SetClientTrackerInfo(trackerInfo);

    // Load config from steamvr.vrsettings
    //vr::IVRSettings *settings_;
    //settings_ = m_pDriverHost->GetSettings(vr::IVRSettings_Version);
}

CPSMoveTrackerLatest::~CPSMoveTrackerLatest()
{
}

vr::EVRInitError CPSMoveTrackerLatest::Activate(uint32_t unObjectId)
{
    vr::EVRInitError result = CPSMoveTrackedDeviceLatest::Activate(unObjectId);

    if (result == vr::VRInitError_None)
    {
        // Poll the latest WorldFromDriverPose transform we got from the service
        // Transform used to convert from PSMove Tracking space to OpenVR Tracking Space
        RefreshWorldFromDriverPose();
    }

    return result;
}

void CPSMoveTrackerLatest::Deactivate()
{
}

float CPSMoveTrackerLatest::GetFloatTrackedDeviceProperty(
    vr::ETrackedDeviceProperty prop,
    vr::ETrackedPropertyError * pError)
{
    float floatResult = 0.f;

    switch (prop)
    {
    case vr::Prop_FieldOfViewLeftDegrees_Float:
    case vr::Prop_FieldOfViewRightDegrees_Float:
        floatResult = (m_tracker_info.tracker_hfov / 2.f);
        *pError = vr::TrackedProp_Success;
        break;
    case vr::Prop_FieldOfViewTopDegrees_Float:
    case vr::Prop_FieldOfViewBottomDegrees_Float:
        floatResult = (m_tracker_info.tracker_vfov / 2.f);
        *pError = vr::TrackedProp_Success;
        break;
    case vr::Prop_TrackingRangeMinimumMeters_Float:
        floatResult = m_tracker_info.tracker_znear * k_fScalePSMoveAPIToMeters;
        *pError = vr::TrackedProp_Success;
        break;
    case vr::Prop_TrackingRangeMaximumMeters_Float:
        floatResult = m_tracker_info.tracker_zfar * k_fScalePSMoveAPIToMeters;
        *pError = vr::TrackedProp_Success;
        break;
    default:
        *pError = vr::TrackedProp_ValueNotProvidedByDevice;
    }

    if (*pError == vr::TrackedProp_ValueNotProvidedByDevice)
    {
        floatResult = CPSMoveTrackedDeviceLatest::GetFloatTrackedDeviceProperty(prop, pError);
    }

    return floatResult;
}

void CPSMoveTrackerLatest::SetClientTrackerInfo(
    const ClientTrackerInfo &trackerInfo)
{
    m_tracker_info = trackerInfo;

    //### HipsterSloth $TODO expose on the pose state if calibration is currently active
    //m_Pose.result = vr::TrackingResult_Calibrating_InProgress;
    m_Pose.result = vr::TrackingResult_Running_OK;

    m_Pose.deviceIsConnected = true;

    // Yaw can't drift because the tracker never moves (hopefully)
    m_Pose.willDriftInYaw = false;
    m_Pose.shouldApplyHeadModel = false;

    // No prediction since that's already handled in the psmove service
    m_Pose.poseTimeOffset = 0.f;

    // Poll the latest WorldFromDriverPose transform we got from the service
    // Transform used to convert from PSMove Tracking space to OpenVR Tracking Space
    RefreshWorldFromDriverPose();

    // No transform due to the current HMD orientation
    m_Pose.qDriverFromHeadRotation.w = 1.f;
    m_Pose.qDriverFromHeadRotation.x = 0.0f;
    m_Pose.qDriverFromHeadRotation.y = 0.0f;
    m_Pose.qDriverFromHeadRotation.z = 0.0f;
    m_Pose.vecDriverFromHeadTranslation[0] = 0.f;
    m_Pose.vecDriverFromHeadTranslation[1] = 0.f;
    m_Pose.vecDriverFromHeadTranslation[2] = 0.f;

    // Set position
    {
        const PSMovePosition &position = m_tracker_info.tracker_pose.Position;

        m_Pose.vecPosition[0] = position.x * k_fScalePSMoveAPIToMeters;
        m_Pose.vecPosition[1] = position.y * k_fScalePSMoveAPIToMeters;
        m_Pose.vecPosition[2] = position.z * k_fScalePSMoveAPIToMeters;
    }

    // Set rotational coordinates
    {
        const PSMoveQuaternion &orientation = m_tracker_info.tracker_pose.Orientation;

        m_Pose.qRotation.w = orientation.w;
        m_Pose.qRotation.x = orientation.x;
        m_Pose.qRotation.y = orientation.y;
        m_Pose.qRotation.z = orientation.z;
    }

    m_Pose.poseIsValid = true;
}

void CPSMoveTrackerLatest::Update()
{
    CPSMoveTrackedDeviceLatest::Update();

    // This call posts this pose to shared memory, where all clients will have access to it the next
    // moment they want to predict a pose.
    m_pDriverHost->TrackedDevicePoseUpdated(m_unSteamVRTrackedDeviceId, m_Pose);
}

int32_t CPSMoveTrackerLatest::GetInt32TrackedDeviceProperty(
    vr::ETrackedDeviceProperty prop,
    vr::ETrackedPropertyError * pError)
{
    int32_t nRetVal = 0;

    switch (prop)
    {
    case vr::Prop_DeviceClass_Int32:
        nRetVal = vr::TrackedDeviceClass_TrackingReference;
        *pError = vr::TrackedProp_Success;
        break;

    default:
        *pError = vr::TrackedProp_ValueNotProvidedByDevice;
        break;
    }

    if (*pError == vr::TrackedProp_ValueNotProvidedByDevice)
    {
        nRetVal = CPSMoveTrackedDeviceLatest::GetInt32TrackedDeviceProperty(prop, pError);
    }

    return nRetVal;
}

uint32_t CPSMoveTrackerLatest::GetStringTrackedDeviceProperty(
    vr::ETrackedDeviceProperty prop,
    char * pchValue,
    uint32_t unBufferSize,
    vr::ETrackedPropertyError * pError)
{
    std::ostringstream ssRetVal;

    switch (prop)
    {
    case vr::Prop_RenderModelName_String:
        // The {psmove} syntax lets us refer to rendermodels that are installed
        // in the driver's own resources/rendermodels directory.  The driver can
        // still refer to SteamVR models like "generic_hmd".
        ssRetVal << "{psmove}ps3eye_tracker";
        //ssRetVal << "generic_tracker";
        break;

    case vr::Prop_ModeLabel_String:
        ssRetVal << m_tracker_info.tracker_id;
        break;
    }

    std::string sRetVal = ssRetVal.str();
    if (sRetVal.empty())
    {
        return CPSMoveTrackedDeviceLatest::GetStringTrackedDeviceProperty(prop, pchValue, unBufferSize, pError);
    }
    else if (sRetVal.size() + 1 > unBufferSize)
    {
        *pError = vr::TrackedProp_BufferTooSmall;
        return static_cast<uint32_t>(sRetVal.size() + 1);  // caller needs to know how to size buffer
    }
    else
    {
        snprintf(pchValue, unBufferSize, sRetVal.c_str());
        *pError = vr::TrackedProp_Success;
        return static_cast<uint32_t>(sRetVal.size() + 1);
    }
}

bool CPSMoveTrackerLatest::HasTrackerId(int TrackerID)
{
    return TrackerID == m_nTrackerId;
}

//==================================================================================================
// Driver Factory
//==================================================================================================

HMD_DLL_EXPORT
void *HmdDriverFactory(const char *pInterfaceName, int *pReturnCode)
{
    if (0 == strcmp(vr::IServerTrackedDeviceProvider_Version, pInterfaceName))
    {
        return &g_ServerTrackedDeviceProvider;
    }
    if (0 == strcmp(vr::IClientTrackedDeviceProvider_Version, pInterfaceName))
    {
        return &g_ClientTrackedDeviceProvider;
    }

    if (pReturnCode)
        *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;

    return NULL;
}
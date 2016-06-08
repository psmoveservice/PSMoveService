//
// driver_psmoveservice.cpp : Defines the client and server interfaces used by the SteamVR runtime.
//

//==================================================================================================
// Includes
//==================================================================================================
#include "driver_psmoveservice.h"
#include <sstream>

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

//==================================================================================================
// Constants
//==================================================================================================
static const float k_fScalePSMoveAPIToMeters = 0.01f;  // psmove driver in cm
static const float k_fRadiansToDegrees = 180.f / 3.14159265f;

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

//==================================================================================================
// Server Provider
//==================================================================================================

CServerDriver_PSMoveService::CServerDriver_PSMoveService()
    : m_bLaunchedPSMoveConfigTool(false)
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
    return m_vecTrackedDevices.size();
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
    // Ask for the HMD tracking space configuration
    // Response handled in HandleHMDTrackingSpaceReponse()
    ClientPSMoveAPI::get_hmd_tracking_space_settings();

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
    case ClientPSMoveAPI::_responsePayloadType_HMDTrackingSpace:
        DriverLog("NotifyClientPSMoveResponse - Updated HMD tracking space.\n");
        HandleHMDTrackingSpaceReponse(&response->payload.hmd_tracking_space);
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
        //TODO
        //case ClientControllerView::PSNavi:
        //    AllocateUniquePSNaviController(controller_id);
        //TODO
        //case ClientControllerView::DualShock4:
        //    AllocateUniqueDualShock4Controller(controller_id);
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

void CServerDriver_PSMoveService::HandleHMDTrackingSpaceReponse(
    const ClientPSMoveAPI::ResponsePayload_HMDTrackingSpace *hmdTrackingSpace)
{
    m_worldFromDriverPose= hmdTrackingSpace->origin_pose;

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


// The psmoveconfigtool is a companion program which can display overlay prompts for us
// and tell us the pose of the HMD at the moment we want to calibrate.
void CServerDriver_PSMoveService::LaunchPSMoveConfigTool( const char * pchDriverInstallDir )
{
    if ( m_bLaunchedPSMoveConfigTool )
        return;

    m_bLaunchedPSMoveConfigTool = true;

    std::ostringstream ss;

    ss << pchDriverInstallDir << "\\bin\\";
#if defined( _WIN64 )
    ss << "win64";
#elif defined( _WIN32 )
    ss << "win32";
#elif defined(__APPLE__) 
    ss << "osx";
#else 
    #error Do not know how to launch psmove config tool
#endif
    DriverLog( "psmoveconfigtool path: %s\n", ss.str().c_str() );

#if defined( _WIN32 )
    STARTUPINFOA sInfoProcess = { 0 };
    sInfoProcess.cb = sizeof( STARTUPINFOW );
    PROCESS_INFORMATION pInfoStartedProcess;
    BOOL okay = CreateProcessA( (ss.str() + "\\psmoveconfigtool.exe").c_str(), NULL, NULL, NULL, FALSE, 0, NULL, ss.str().c_str(), &sInfoProcess, &pInfoStartedProcess );
    DriverLog( "start psmoveconfigtool okay: %d %08x\n", okay, GetLastError() );
#elif defined(__APPLE__) 
    pid_t processId;
    if ((processId = fork()) == 0)
    {
        const char * app_path = (ss.str() + "\\psmoveconfigtool").c_str();
        char app[256];
        
        size_t app_path_len= strnlen(app_path, sizeof(app)-1);
        strncpy(app, app_path, app_path_len);
        app[app_path_len]= '\0';
        
        char * const argv[] = { app, NULL };
        
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

/** Launch hydra_monitor if needed (requested by devices as they activate) */
void CServerDriver_PSMoveService::LaunchPSMoveConfigTool()
{
    LaunchPSMoveConfigTool( m_strDriverInstallDir.c_str() );
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

vr::EVRInitError CClientDriver_PSMoveService::Init( vr::IDriverLog * pDriverLog, vr::IClientDriverHost * pDriverHost, const char * pchUserDriverConfigDir, const char * pchDriverInstallDir )
{
    InitDriverLog( pDriverLog );
    m_pDriverHost = pDriverHost;
    return vr::VRInitError_None;
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
        return sRetVal.size() + 1;  // caller needs to know how to size buffer
    }
    else
    {
        snprintf(pchValue, unBufferSize, sRetVal.c_str());
        *pError = vr::TrackedProp_Success;
        return sRetVal.size() + 1;
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

}

void CPSMoveTrackedDeviceLatest::RefreshWorldFromDriverPose()
{
    const PSMovePose worldFromDriverPose = g_ServerTrackedDeviceProvider.GetWorldFromDriverPose();

    // Transform used to convert from PSMove Tracking space to OpenVR Tracking Space
    m_Pose.qWorldFromDriverRotation.w = worldFromDriverPose.Orientation.w;
    m_Pose.qWorldFromDriverRotation.x = worldFromDriverPose.Orientation.x;
    m_Pose.qWorldFromDriverRotation.y = worldFromDriverPose.Orientation.y;
    m_Pose.qWorldFromDriverRotation.z = worldFromDriverPose.Orientation.z;
    m_Pose.vecWorldFromDriverTranslation[0] = worldFromDriverPose.Position.x * k_fScalePSMoveAPIToMeters;
    m_Pose.vecWorldFromDriverTranslation[1] = worldFromDriverPose.Position.y * k_fScalePSMoveAPIToMeters;
    m_Pose.vecWorldFromDriverTranslation[2] = worldFromDriverPose.Position.z * k_fScalePSMoveAPIToMeters;
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
    , m_pendingHapticPulseDuration(0)
    , m_sentHapticPulseDuration(0)
    , m_lastTimeRumbleSent()
    , m_lastTimeRumbleSentValid(false)
{
    char buf[256];
    GenerateControllerSerialNumber(buf, sizeof(buf), controllerId);
    m_strSerialNumber = buf;

    // Tell psmoveapi that we are listening to this controller id
    m_controller_view = ClientPSMoveAPI::allocate_controller_view(controllerId);

    memset(&m_ControllerState, 0, sizeof(vr::VRControllerState_t));

    // Load config from steamvr.vrsettings
    //vr::IVRSettings *settings_;
    //settings_ = m_pDriverHost->GetSettings(vr::IVRSettings_Version);
}

CPSMoveControllerLatest::~CPSMoveControllerLatest()
{
    ClientPSMoveAPI::free_controller_view(m_controller_view);
}

vr::EVRInitError CPSMoveControllerLatest::Activate(uint32_t unObjectId)
{
    vr::EVRInitError result = CPSMoveTrackedDeviceLatest::Activate(unObjectId);

    if (result == vr::VRInitError_None)
    {
        // Poll the latest WorldFromDriverPose transform we got from the service
        // Transform used to convert from PSMove Tracking space to OpenVR Tracking Space
        RefreshWorldFromDriverPose();

        ClientPSMoveAPI::start_controller_data_stream(
            m_controller_view, 
            ClientPSMoveAPI::includePositionData | ClientPSMoveAPI::includePhysicsData);
    }

    return result;
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
        ulRetVal = 
            vr::ButtonMaskFromId(k_EButton_PS) |
            vr::ButtonMaskFromId(k_EButton_Move) |
            vr::ButtonMaskFromId(k_EButton_Select) |
            vr::ButtonMaskFromId(k_EButton_Start) |
            vr::ButtonMaskFromId(k_EButton_Trigger) |
            vr::ButtonMaskFromId(k_EButton_Triangle) |
            vr::ButtonMaskFromId(k_EButton_Circle) |
            vr::ButtonMaskFromId(k_EButton_Square) |
            vr::ButtonMaskFromId(k_EButton_Cross);
        *pError = vr::TrackedProp_Success;
        break;

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
        //ssRetVal << "{psmove}psmove_controller";
        ssRetVal << "vr_controller_vive_1_5";
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
        return sRetVal.size() + 1;  // caller needs to know how to size buffer
    }
    else
    {
        snprintf( pchValue, unBufferSize, sRetVal.c_str() );
        *pError = vr::TrackedProp_Success;
        return sRetVal.size() + 1;
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

    const ClientPSMoveView &clientView= m_controller_view->GetPSMoveView();
    vr::VRControllerState_t NewState = { 0 };

    // Changing unPacketNum tells anyone polling state that something might have
    // changed.  We don't try to be precise about that here.
    NewState.unPacketNum = m_ControllerState.unPacketNum + 1;

    if (clientView.GetButtonCircle())
        NewState.ulButtonPressed |= vr::ButtonMaskFromId(k_EButton_Circle);
    if (clientView.GetButtonCross())
        NewState.ulButtonPressed |= vr::ButtonMaskFromId(k_EButton_Cross);
    if (clientView.GetButtonMove())
        NewState.ulButtonPressed |= vr::ButtonMaskFromId(k_EButton_Move);
    if (clientView.GetButtonPS())
        NewState.ulButtonPressed |= vr::ButtonMaskFromId(k_EButton_PS);
    if (clientView.GetButtonSelect())
        NewState.ulButtonPressed |= vr::ButtonMaskFromId(k_EButton_Select);
    if (clientView.GetButtonSquare())
        NewState.ulButtonPressed |= vr::ButtonMaskFromId(k_EButton_Square);
    if (clientView.GetButtonStart())
        NewState.ulButtonPressed |= vr::ButtonMaskFromId(k_EButton_Start);
    if (clientView.GetButtonTriangle())
        NewState.ulButtonPressed |= vr::ButtonMaskFromId(k_EButton_Triangle);
    if (clientView.GetButtonTrigger())
        NewState.ulButtonPressed |= vr::ButtonMaskFromId(k_EButton_Trigger);

    // All pressed buttons are touched
    NewState.ulButtonTouched |= NewState.ulButtonPressed;

    uint64_t ulChangedTouched = NewState.ulButtonTouched ^ m_ControllerState.ulButtonTouched;
    uint64_t ulChangedPressed = NewState.ulButtonPressed ^ m_ControllerState.ulButtonPressed;

    SendButtonUpdates( &vr::IServerDriverHost::TrackedDeviceButtonTouched, ulChangedTouched & NewState.ulButtonTouched );
    SendButtonUpdates( &vr::IServerDriverHost::TrackedDeviceButtonPressed, ulChangedPressed & NewState.ulButtonPressed );
    SendButtonUpdates( &vr::IServerDriverHost::TrackedDeviceButtonUnpressed, ulChangedPressed & ~NewState.ulButtonPressed );
    SendButtonUpdates( &vr::IServerDriverHost::TrackedDeviceButtonUntouched, ulChangedTouched & ~NewState.ulButtonTouched );

    NewState.rAxis[0].x = clientView.GetTriggerValue();
    NewState.rAxis[0].y = 0.f;

    if ( NewState.rAxis[0].x != m_ControllerState.rAxis[0].x )
        m_pDriverHost->TrackedDeviceAxisUpdated( m_unSteamVRTrackedDeviceId, 0, NewState.rAxis[0] );

    m_ControllerState = NewState;
}

void CPSMoveControllerLatest::UpdateTrackingState()
{
    assert(m_controller_view != nullptr);
    assert(m_controller_view->GetIsConnected());

    //### HipsterSloth $TODO expose on the pose state if calibration is currently active
    //m_Pose.result = vr::TrackingResult_Calibrating_InProgress;
    m_Pose.result = vr::TrackingResult_Running_OK;

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

                // The physics coming from the server is jacked up right now
                // so until I can figure out what's going on, let's just disabme it for now

                //m_Pose.vecVelocity[0] = physicsData.Velocity.i * k_fScalePSMoveAPIToMeters;
                //m_Pose.vecVelocity[1] = physicsData.Velocity.j * k_fScalePSMoveAPIToMeters;
                //m_Pose.vecVelocity[2] = physicsData.Velocity.k * k_fScalePSMoveAPIToMeters;

                //m_Pose.vecAcceleration[0] = physicsData.Acceleration.i * k_fScalePSMoveAPIToMeters;
                //m_Pose.vecAcceleration[1] = physicsData.Acceleration.j * k_fScalePSMoveAPIToMeters;
                //m_Pose.vecAcceleration[2] = physicsData.Acceleration.k * k_fScalePSMoveAPIToMeters;

                //m_Pose.vecAngularVelocity[0] = physicsData.AngularVelocity.i;
                //m_Pose.vecAngularVelocity[1] = physicsData.AngularVelocity.j;
                //m_Pose.vecAngularVelocity[2] = physicsData.AngularVelocity.k;

                //m_Pose.vecAngularAcceleration[0] = physicsData.AngularAcceleration.i;
                //m_Pose.vecAngularAcceleration[1] = physicsData.AngularAcceleration.j;
                //m_Pose.vecAngularAcceleration[2] = physicsData.AngularAcceleration.k;

                m_Pose.vecVelocity[0] = 0.f;
                m_Pose.vecVelocity[1] = 0.f;
                m_Pose.vecVelocity[2] = 0.f;

                m_Pose.vecAcceleration[0] = 0.f;
                m_Pose.vecAcceleration[1] = 0.f;
                m_Pose.vecAcceleration[2] = 0.f;

                m_Pose.vecAngularVelocity[0] = 0.f;
                m_Pose.vecAngularVelocity[1] = 0.f;
                m_Pose.vecAngularVelocity[2] = 0.f;

                m_Pose.vecAngularAcceleration[0] = 0.f;
                m_Pose.vecAngularAcceleration[1] = 0.f;
                m_Pose.vecAngularAcceleration[2] = 0.f;
            }

            m_Pose.poseIsValid = view.GetIsCurrentlyTracking();

            // This call posts this pose to shared memory, where all clients will have access to it the next
            // moment they want to predict a pose.
            m_pDriverHost->TrackedDevicePoseUpdated(m_unSteamVRTrackedDeviceId, m_Pose);
        } break;
    case ClientControllerView::eControllerType::PSNavi:
        {
            m_Pose.poseIsValid = false;

            // Don't post anything to the driver host
        } break;
    }
}

void CPSMoveControllerLatest::UpdateRumbleState()
{
    const float k_max_rumble_update_rate = 100.f; // Don't bother trying to update the rumble faster than 10fps (100ms)
    const float k_max_pulse_microseconds = 1000.f; // Docs suggest max pulse duration of 5ms, but we'll call 1ms max

    // Only bother with this if the rumble value actually changed
    if (m_pendingHapticPulseDuration != m_sentHapticPulseDuration)
    {
        std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
        bool bTimoutElapsed= true;

        if (m_lastTimeRumbleSentValid)
        {
            std::chrono::duration<double, std::milli> timeSinceLastSend = now - m_lastTimeRumbleSent;

            bTimoutElapsed= timeSinceLastSend.count() >= k_max_rumble_update_rate;
        }

        // See if a rumble request hasn't come too recently
        if (bTimoutElapsed)
        {
            float rumble_fraction = static_cast<float>(m_pendingHapticPulseDuration) / k_max_pulse_microseconds;

            // Unless a zero runble intensity was explicitly set, 
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
            ClientPSMoveAPI::eat_response(ClientPSMoveAPI::set_controller_rumble(m_controller_view, rumble_fraction));

            // Remember the last rumble we went and when we sent it
            m_sentHapticPulseDuration = m_pendingHapticPulseDuration;
            m_lastTimeRumbleSent = now;
            m_lastTimeRumbleSentValid= true;

            // Reset the pending haptic pulse duration.
            // If another call to TriggerHapticPulse() is made later, it will stomp this value.
            // If no call to TriggerHapticPulse() is made later, then the next call to UpdateRumbleState()
            // in k_max_rumble_update_rate milliseconds will set the rumble_fraction to 0.f
            // This effectively makes the shortest rumble pulse k_max_rumble_update_rate milliseconds.
            m_pendingHapticPulseDuration = 0;
        }
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
        int seq_num= m_controller_view->GetSequenceNum();

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
        //ssRetVal << "ps3eye_tracker";
        ssRetVal << "generic_tracker";
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
        return sRetVal.size() + 1;  // caller needs to know how to size buffer
    }
    else
    {
        snprintf(pchValue, unBufferSize, sRetVal.c_str());
        *pError = vr::TrackedProp_Success;
        return sRetVal.size() + 1;
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
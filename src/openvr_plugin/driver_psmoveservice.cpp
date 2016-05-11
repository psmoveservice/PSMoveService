//
// driver_psmoveservice.cpp : Defines the client and server interfaces used by the SteamVR runtime.
//

#include "driver_psmoveservice.h"
#include <sstream>

#if defined( _WIN32 )
#include <windows.h>
#else
#include <unistd.h>
#endif

#if defined( _WIN32 )
#define HMD_DLL_EXPORT extern "C" __declspec(dllexport)
#else
#define HMD_DLL_EXPORT extern "C"
#endif

#if defined( _WIN32)
#define strcasecmp(a, b) stricmp(a,b)
#define snprintf _snprintf
#endif

CServerDriver_PSMoveService g_ServerTrackedDeviceProvider;
CClientDriver_PSMoveService g_ClientTrackedDeviceProvider;

HMD_DLL_EXPORT
void *HmdDriverFactory( const char *pInterfaceName, int *pReturnCode )
{
    if ( 0 == strcmp( vr::IServerTrackedDeviceProvider_Version, pInterfaceName ) )
    {
        return &g_ServerTrackedDeviceProvider;
    }
    if ( 0 == strcmp( vr::IClientTrackedDeviceProvider_Version, pInterfaceName ) )
    {
        return &g_ClientTrackedDeviceProvider;
    }

    if ( pReturnCode )
        *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;

    return NULL;
}

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
    
    // Note that reconnection is a non-blocking async request.
    // Returning true means we we're able to start trying to connect,
    // not that we are successfully connected yet.
    if (ReconnectToPSMoveService())
    {
        // By default, allocate one tracked device slot.
        // The allocated slot won't be considered connected until we get data back from
        // the service about its connection status.
        // If we get a tracked devices list update and there turn out to be more than one
        // tracked devices, we'll allocate additional slots as needed.
        AllocateUniquePSMoveController(0, false);
    }
    else
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

uint32_t CServerDriver_PSMoveService::GetTrackedDeviceCount()
{
    return m_vecPSMoveControllers.size();
}

vr::ITrackedDeviceServerDriver * CServerDriver_PSMoveService::GetTrackedDeviceDriver( 
    uint32_t unWhich, 
    const char *pchInterfaceVersion )
{
    // don't return anything if that's not the interface version we have
    if ( 0 != strcasecmp( pchInterfaceVersion, vr::ITrackedDeviceServerDriver_Version ) )
    {
        DriverLog( "FindTrackedDeviceDriver for version %s, which we don't support.\n",
            pchInterfaceVersion );
        return nullptr;
    }

    if ( unWhich < m_vecPSMoveControllers.size() )
    {
        return m_vecPSMoveControllers[unWhich];
    }

    return nullptr;
}

vr::ITrackedDeviceServerDriver * CServerDriver_PSMoveService::FindTrackedDeviceDriver(
    const char * pchId, 
    const char *pchInterfaceVersion )
{
    // don't return anything if that's not the interface version we have
    if ( 0 != strcasecmp( pchInterfaceVersion, vr::ITrackedDeviceServerDriver_Version ) )
    {
        DriverLog( "FindTrackedDeviceDriver for version %s, which we don't support.\n",
            pchInterfaceVersion );
        return NULL;
    }

    for ( auto it = m_vecPSMoveControllers.begin(); it != m_vecPSMoveControllers.end(); ++it )
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

    // Update all active PSMove controllers
    for (auto it = m_vecPSMoveControllers.begin(); it != m_vecPSMoveControllers.end(); ++it)
    {
        CPSMoveControllerLatest *pController = *it;

        pController->Update();
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
    }
}

void CServerDriver_PSMoveService::HandleConnectedToPSMoveService()
{
    // Ask the service for a list of connected controllers
    // Response handled in HandleControllerListReponse()
    ClientPSMoveAPI::get_controller_list();

    //TODO: Ask the service for a list of connected trackers
}

void CServerDriver_PSMoveService::HandleFailedToConnectToPSMoveService()
{
    // Immediately attempt to reconnect to the service
    ReconnectToPSMoveService();
}

void CServerDriver_PSMoveService::HandleDisconnectedFromPSMoveService()
{
    // Deactivate all of the controller contexts
    for (auto it = m_vecPSMoveControllers.begin(); it != m_vecPSMoveControllers.end(); ++it)
    {
        CPSMoveControllerLatest *pController = *it;

        pController->Deactivate();
    }

    //TODO: Deactivate all of the tracker contexts

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
    //TODO: Ask the service for a list of connected trackers
}

// -- Response Handling -----
void CServerDriver_PSMoveService::HandleClientPSMoveResponse(
    const ClientPSMoveAPI::ResponseMessage *response)
{
    switch (response->payload_type)
    {
    case ClientPSMoveAPI::_responsePayloadType_ControllerCount:
        DriverLog("NotifyClientPSMoveResponse - Controller Count = %d (request id %d).\n", 
            response->payload.controller_list.count, response->request_id);
        HandleControllerListReponse(&response->payload.controller_list);
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
            AllocateUniquePSMoveController(controller_id, true);
            break;
        //TODO
        //case ClientControllerView::PSNavi:
        //    AllocateUniquePSNaviController(controller_id, true);
            break;
        default:
            break;
        }
    }
}

static void GenerateSerialNumber( char *p, int psize, int controller )
{
#if defined( WIN32 )
    _snprintf( p, psize, "psmove_controller%d", controller );
#else
    snprintf(p, psize, "psmove_controller%d", controller);
#endif
}

void CServerDriver_PSMoveService::AllocateUniquePSMoveController(int ControllerID, bool bNotifyServer)
{
    char buf[256];
    GenerateSerialNumber(buf, sizeof(buf), ControllerID);

    if ( !FindTrackedDeviceDriver( buf, vr::ITrackedDeviceServerDriver_Version ) )
    {
        DriverLog( "added new device %s\n", buf );
        m_vecPSMoveControllers.push_back( new CPSMoveControllerLatest( m_pDriverHost, ControllerID ) );

        if ( bNotifyServer && m_pDriverHost )
        {
            m_pDriverHost->TrackedDeviceAdded( m_vecPSMoveControllers.back()->GetSerialNumber() );
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
#else
    ss << "osx";
#endif
    DriverLog( "psmoveconfigtool path: %s\n", ss.str().c_str() );

#if defined( _WIN32 )
    STARTUPINFOA sInfoProcess = { 0 };
    sInfoProcess.cb = sizeof( STARTUPINFOW );
    PROCESS_INFORMATION pInfoStartedProcess;
    BOOL okay = CreateProcessA( (ss.str() + "\\psmoveconfigtool.exe").c_str(), NULL, NULL, NULL, FALSE, 0, NULL, ss.str().c_str(), &sInfoProcess, &pInfoStartedProcess );
    DriverLog( "start psmoveconfigtool okay: %d %08x\n", okay, GetLastError() );
#else
    pid_t processId;
    if ((processId = fork()) == 0)
    {
        const char * app_path = (ss.str() + "\\psmoveconfigtool.exe").c_str();
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
    return vr::HiddenAreaMesh_t();
}

uint32_t CClientDriver_PSMoveService::GetMCImage( uint32_t * pImgWidth, uint32_t * pImgHeight, uint32_t * pChannels, void * pDataBuffer, uint32_t unBufferLen )
{
    return uint32_t();
}

//==================================================================================================
// Device Driver
//==================================================================================================

const float CPSMoveControllerLatest::k_fScalePSMoveAPIToMeters = 0.01;  // psmove driver in cm

CPSMoveControllerLatest::CPSMoveControllerLatest( vr::IServerDriverHost * pDriverHost, int controllerId )
    : m_pDriverHost( pDriverHost )
    , m_nControllerId( controllerId )
    , m_nPoseSequenceNumber( 0 )
    , m_bCalibrated( false )
    , m_unSteamVRTrackedDeviceId( vr::k_unTrackedDeviceIndexInvalid )
{
    char buf[256];
    GenerateSerialNumber( buf, sizeof( buf ), controllerId );
    m_strSerialNumber = buf;

    // Tell psmoveapi that we are listening to this controller id
    m_controller_view = ClientPSMoveAPI::allocate_controller_view(controllerId);

    memset( &m_ControllerState, 0, sizeof( m_ControllerState ) );
    memset( &m_Pose, 0, sizeof( m_Pose ) );
    m_Pose.result = vr::TrackingResult_Calibrating_InProgress;

    //###HipsterSloth $TODO
    m_WorldFromDriverTranslation= *k_psmove_float_vector3_zero;
    m_WorldFromDriverRotation= *k_psmove_quaternion_identity;
    m_bCalibrated= true;

    //###HipsterSloth $TODO
    m_firmware_revision = 0;
    m_hardware_revision = 0;

    // Load config from steamvr.vrsettings
    vr::IVRSettings *settings_;
    settings_ = m_pDriverHost->GetSettings(vr::IVRSettings_Version);
    char tmp_[32];

    // renderModel: assign a rendermodel
    settings_->GetString("psmove", "renderModel", tmp_, sizeof(tmp_), "vr_controller_vive_1_5");
    m_strRenderModel.assign(tmp_, sizeof(tmp_));
}

CPSMoveControllerLatest::~CPSMoveControllerLatest()
{
    ClientPSMoveAPI::free_controller_view(m_controller_view);
}

void *CPSMoveControllerLatest::GetComponent( const char *pchComponentNameAndVersion )
{
    if ( !strcasecmp( pchComponentNameAndVersion, vr::IVRControllerComponent_Version ) )
    {
        return ( vr::IVRControllerComponent* )this;
    }
    
    return NULL;
}

vr::EVRInitError CPSMoveControllerLatest::Activate( uint32_t unObjectId )
{
    DriverLog( "CPSMoveServiceHmdLatest::Activate: %s is object id %d\n", GetSerialNumber(), unObjectId );
    m_unSteamVRTrackedDeviceId = unObjectId;

    g_ServerTrackedDeviceProvider.LaunchPSMoveConfigTool();

    return vr::VRInitError_None;
}

void CPSMoveControllerLatest::Deactivate()
{
    DriverLog( "CPSMoveServiceHmdLatest::Deactivate: %s was object id %d\n", GetSerialNumber(), m_unSteamVRTrackedDeviceId );
    m_unSteamVRTrackedDeviceId = vr::k_unTrackedDeviceIndexInvalid;
}

void CPSMoveControllerLatest::PowerOff()
{
    // TODO FIXME Implement
}

void CPSMoveControllerLatest::DebugRequest( const char * pchRequest, char * pchResponseBuffer, uint32_t unResponseBufferSize )
{
}

const char * CPSMoveControllerLatest::GetSerialNumber()
{
    return m_strSerialNumber.c_str();
}

vr::DriverPose_t CPSMoveControllerLatest::GetPose()
{
    // This is only called at startup to synchronize with the driver.
    // Future updates are driven by our thread calling TrackedDevicePoseUpdated()
    return m_Pose;
}

bool CPSMoveControllerLatest::GetBoolTrackedDeviceProperty( vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError * pError )
{
    *pError = vr::TrackedProp_ValueNotProvidedByDevice;
    return false;
}

float CPSMoveControllerLatest::GetFloatTrackedDeviceProperty( vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError * pError )
{
    *pError = vr::TrackedProp_ValueNotProvidedByDevice;
    return 0.0f;
}

int32_t CPSMoveControllerLatest::GetInt32TrackedDeviceProperty( vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError * pError )
{
    int32_t nRetVal = 0;
    vr::ETrackedPropertyError error = vr::TrackedProp_UnknownProperty;
    switch ( prop )
    {
    case vr::Prop_DeviceClass_Int32:
        nRetVal = vr::TrackedDeviceClass_Controller;
        error = vr::TrackedProp_Success;
        break;

    case vr::Prop_Axis0Type_Int32:
        nRetVal = vr::k_eControllerAxis_Trigger;
        error = vr::TrackedProp_Success;
        break;

    case vr::Prop_Axis1Type_Int32:
    case vr::Prop_Axis2Type_Int32:
    case vr::Prop_Axis3Type_Int32:
    case vr::Prop_Axis4Type_Int32:
        error = vr::TrackedProp_ValueNotProvidedByDevice;
        break;
    }

    *pError = error;
    return nRetVal;
}

uint64_t CPSMoveControllerLatest::GetUint64TrackedDeviceProperty( vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError * pError )
{
    uint64_t ulRetVal = 0;
    vr::ETrackedPropertyError error = vr::TrackedProp_ValueNotProvidedByDevice;

    switch ( prop )
    {
    case vr::Prop_CurrentUniverseId_Uint64:
    case vr::Prop_PreviousUniverseId_Uint64:
        error = vr::TrackedProp_ValueNotProvidedByDevice;
        break;

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
        error = vr::TrackedProp_Success;
        break;

    case vr::Prop_HardwareRevision_Uint64:
        ulRetVal = m_hardware_revision;
        error = vr::TrackedProp_Success;
        break;

    case vr::Prop_FirmwareVersion_Uint64:
        ulRetVal = m_firmware_revision;
        error = vr::TrackedProp_Success;
        break;

    }

    *pError = error;
    return ulRetVal;
}

vr::HmdMatrix34_t CPSMoveControllerLatest::GetMatrix34TrackedDeviceProperty( vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError * pError )
{
    return vr::HmdMatrix34_t();
}

uint32_t CPSMoveControllerLatest::GetStringTrackedDeviceProperty( vr::ETrackedDeviceProperty prop, char * pchValue, uint32_t unBufferSize, vr::ETrackedPropertyError * pError )
{
    std::ostringstream ssRetVal;


    switch ( prop )
    {
    case vr::Prop_SerialNumber_String:
        ssRetVal << m_strSerialNumber;
        break;

    case vr::Prop_RenderModelName_String:
        // The {psmove} syntax lets us refer to rendermodels that are installed
        // in the driver's own resources/rendermodels directory.  The driver can
        // still refer to SteamVR models like "generic_hmd".
        //ssRetVal << "{psmove}psmove_controller";

        // We return the user configured rendermodel here. Defaults to "{psmove}psmove_controller".
        ssRetVal << m_strRenderModel.c_str();
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
    if ( sRetVal.empty() )
    {
        *pError = vr::TrackedProp_ValueNotProvidedByDevice;
        return 0;
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
    // This is only called at startup to synchronize with the driver.
    // Future updates are driven by our thread calling TrackedDeviceButton*() and TrackedDeviceAxis*()
    return vr::VRControllerState_t();
}

bool CPSMoveControllerLatest::TriggerHapticPulse( uint32_t unAxisId, uint16_t usPulseDurationMicroseconds )
{
    const float k_max_pulse_microseconds = 5000.f; // Docs suggest max pulse duration of 5ms
    float rumble_fraction = static_cast<float>(usPulseDurationMicroseconds) / k_max_pulse_microseconds;

    if (rumble_fraction > 1.f)
    {
        rumble_fraction = 1.f;
    }

    return ClientPSMoveAPI::eat_response(ClientPSMoveAPI::set_controller_rumble(m_controller_view, rumble_fraction));
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

    const ClientPSMoveView &clientView = m_controller_view->GetPSMoveView();

    //TODO: Determine what this offset should actually be (come from psmoveapi?)
    m_Pose.poseTimeOffset = 0.f;

    m_Pose.qWorldFromDriverRotation.w = m_WorldFromDriverRotation.w;
    m_Pose.qWorldFromDriverRotation.x = m_WorldFromDriverRotation.x;
    m_Pose.qWorldFromDriverRotation.y = m_WorldFromDriverRotation.y;
    m_Pose.qWorldFromDriverRotation.z = m_WorldFromDriverRotation.z;
    m_Pose.vecWorldFromDriverTranslation[0] = m_WorldFromDriverTranslation.i;
    m_Pose.vecWorldFromDriverTranslation[1] = m_WorldFromDriverTranslation.j;
    m_Pose.vecWorldFromDriverTranslation[2] = m_WorldFromDriverTranslation.k;

    //TODO: Determine what this offset should actually be (come from psmoveapi?)
    m_Pose.qDriverFromHeadRotation.w = 1.f;
    m_Pose.qDriverFromHeadRotation.x = 0.0f;
    m_Pose.qDriverFromHeadRotation.y = 0.0f;
    m_Pose.qDriverFromHeadRotation.z = 0.0f;
    m_Pose.vecDriverFromHeadTranslation[0] = 0.f;
    m_Pose.vecDriverFromHeadTranslation[1] = 0.f;
    m_Pose.vecDriverFromHeadTranslation[2] = 0.f;

    // Set position
    {
        const PSMovePosition &position = clientView.GetPosition();

        m_Pose.vecPosition[0] = position.x * k_fScalePSMoveAPIToMeters;
        m_Pose.vecPosition[1] = position.y * k_fScalePSMoveAPIToMeters;
        m_Pose.vecPosition[2] = position.z * k_fScalePSMoveAPIToMeters;
    }

    // Set rotational coordinates
    {
        const PSMoveQuaternion &orientation = clientView.GetOrientation();

        m_Pose.qRotation.w = orientation.w;
        m_Pose.qRotation.x = orientation.x;
        m_Pose.qRotation.y = orientation.y;
        m_Pose.qRotation.z = orientation.z;
    }

    //TODO: Expose linear and angular acceleration on client view
    m_Pose.vecVelocity[0] = 0.0;
    m_Pose.vecVelocity[1] = 0.0;
    m_Pose.vecVelocity[2] = 0.0;

    m_Pose.vecAcceleration[0] = 0.0;
    m_Pose.vecAcceleration[1] = 0.0;
    m_Pose.vecAcceleration[2] = 0.0;

    m_Pose.vecAngularVelocity[0] = 0.0;
    m_Pose.vecAngularVelocity[1] = 0.0;
    m_Pose.vecAngularVelocity[2] = 0.0;

    m_Pose.vecAngularAcceleration[0] = 0.0;
    m_Pose.vecAngularAcceleration[1] = 0.0;
    m_Pose.vecAngularAcceleration[2] = 0.0;

    if ( !m_bCalibrated )
        m_Pose.result = vr::TrackingResult_Calibrating_InProgress;
    else
        m_Pose.result = vr::TrackingResult_Running_OK;

    m_Pose.poseIsValid = m_bCalibrated;
    m_Pose.deviceIsConnected = true;

    // These should always be false from any modern driver.  These are for Oculus DK1-like
    // rotation-only tracking.  Support for that has likely rotted in vrserver.
    m_Pose.willDriftInYaw = false;
    m_Pose.shouldApplyHeadModel = false;

    // This call posts this pose to shared memory, where all clients will have access to it the next
    // moment they want to predict a pose.
    m_pDriverHost->TrackedDevicePoseUpdated( m_unSteamVRTrackedDeviceId, m_Pose );
}

bool CPSMoveControllerLatest::IsActivated() const
{
    return m_unSteamVRTrackedDeviceId != vr::k_unTrackedDeviceIndexInvalid;
}

bool CPSMoveControllerLatest::HasControllerId( int ControllerID )
{
    return ControllerID == m_nControllerId;
}

void CPSMoveControllerLatest::Update()
{
    if (IsActivated() && m_controller_view->GetIsConnected())
    {
        int seq_num= m_controller_view->GetSequenceNum();

        if (m_nPoseSequenceNumber != seq_num)
        {
            m_nPoseSequenceNumber = seq_num;

            UpdateTrackingState();
            UpdateControllerState();
        }
    }
}
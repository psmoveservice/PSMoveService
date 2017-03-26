#ifndef __PSMOVECLIENT_CAPI_H
#define __PSMOVECLIENT_CAPI_H
#include "PSMoveClient_export.h"
#include "ClientConstants.h"
#include "ClientGeometry_CAPI.h"
#include <stdbool.h>
//cut_before

// Wrapper Types
//--------------
typedef int PSMRequestID;
typedef const void * PSMResponseHandle;
typedef void *PSMRequestHandle;
typedef const void *PSMEventDataHandle;

typedef int PSMControllerID;
typedef int PSMTrackerID;
typedef int PSMHmdID;

// Shared Constants
//-----------------

const PSMRequestID PSM_INVALID_REQUEST_ID = -1;

enum _PSMResult
{
    PSMResult_Error                 = -1,
    PSMResult_Success               = 0,
    PSMResult_Timeout               = 1,
    PSMResult_RequestSent           = 2,
    PSMResult_Canceled              = 3,
    PSMResult_NoData                = 4,
};
typedef enum _PSMResult PSMResult;

enum _PSMConnectionType
{
    PSMConnectionType_BLUETOOTH,
    PSMConnectionType_USB
    
};
typedef enum _PSMConnectionType PSMConnectionType;

enum _PSMButtonState {
    PSMButtonState_UP = 0x00,       // (00b) Not pressed
    PSMButtonState_PRESSED = 0x01,  // (01b) Down for one frame only
    PSMButtonState_DOWN = 0x03,     // (11b) Down for >1 frame
    PSMButtonState_RELEASED = 0x02, // (10b) Up for one frame only
};
typedef enum _PSMButtonState PSMButtonState;

typedef enum _PSMTrackingColorType
{
    PSMTrackingColorType_Magenta,    // R:0xFF, G:0x00, B:0xFF
    PSMTrackingColorType_Cyan,       // R:0x00, G:0xFF, B:0xFF
    PSMTrackingColorType_Yellow,     // R:0xFF, G:0xFF, B:0x00
    PSMTrackingColorType_Red,        // R:0xFF, G:0x00, B:0x00
    PSMTrackingColorType_Green,      // R:0x00, G:0xFF, B:0x00
    PSMTrackingColorType_Blue,       // R:0x00, G:0x00, B:0xFF
	
	PSMTrackingColorType_MaxColorTypes
} PSMTrackingColorType;

typedef enum _PSMBatteryState
{
    PSMBattery_0        = 0,
    PSMBattery_20       = 1,
    PSMBattery_40       = 2,
    PSMBattery_60       = 3,
    PSMBattery_80       = 4,
    PSMBattery_100      = 5,
    PSMBattery_Charging = 0xEE,
    PSMBattery_Charged  = 0xEF
} PSMBatteryState;

typedef enum _PSMControllerDataStreamFlags
{
    PSMStreamFlags_defaultStreamOptions = 0x00,
    PSMStreamFlags_includePositionData = 0x01,
    PSMStreamFlags_includePhysicsData = 0x02,
    PSMStreamFlags_includeRawSensorData = 0x04,
	PSMStreamFlags_includeCalibratedSensorData = 0x08,
    PSMStreamFlags_includeRawTrackerData = 0x10,
	PSMStreamFlags_disableROI = 0x20,
} PSMControllerDataStreamFlags;

typedef enum _PSMControllerRumbleChannel
{
    PSMControllerRumbleChannel_All,
    PSMControllerRumbleChannel_Left,
    PSMControllerRumbleChannel_Right
} PSMControllerRumbleChannel;

typedef enum _PSMControllerType
{
    PSMController_None= -1,
    PSMController_Move,
    PSMController_Navi,
	PSMController_DualShock4
} PSMControllerType;

typedef enum _PSMTrackerType
{
    PSMTracker_None= -1,
    PSMTracker_PS3Eye
} PSMTrackerType;

typedef enum _PSMHmdType
{
    PSMHmd_None= -1,
	PSMHmd_Morpheus= -1,
} PSMHmdType;

typedef enum _PSMTrackerDriver
{
    PSMDriver_LIBUSB,
    PSMDriver_CL_EYE,
    PSMDriver_CL_EYE_MULTICAM,
    PSMDriver_GENERIC_WEBCAM
} PSMTrackerDriver;

// Controller State
//------------------

typedef struct _PSMovePhysicsData
{
    PSMVector3f LinearVelocityCmPerSec;
    PSMVector3f LinearAccelerationCmPerSecSqr;
    PSMVector3f AngularVelocityRadPerSec;
    PSMVector3f AngularAccelerationRadPerSecSqr;
    double       TimeInSeconds;
} PSMPhysicsData;

typedef struct _PSMPSMoveRawSensorData
{
    PSMVector3i Magnetometer;
    PSMVector3i Accelerometer;
    PSMVector3i Gyroscope;
    double      TimeInSeconds;
} PSMPSMoveRawSensorData;

typedef struct _PSMPSMoveCalibratedSensorData
{
    PSMVector3f Magnetometer;
    PSMVector3f Accelerometer;
    PSMVector3f Gyroscope;
    double      TimeInSeconds;
} PSMPSMoveCalibratedSensorData;

typedef struct _PSMRawTrackerData
{
    // Parallel arrays: ScreenLocations, Positions and the TrackerID associated with them
    PSMVector2f             ScreenLocations[PSMOVESERVICE_MAX_TRACKER_COUNT];
    PSMVector3f             RelativePositionsCm[PSMOVESERVICE_MAX_TRACKER_COUNT];
    PSMQuatf                RelativeOrientations[PSMOVESERVICE_MAX_TRACKER_COUNT];
    PSMTrackingProjection   TrackingProjections[PSMOVESERVICE_MAX_TRACKER_COUNT];
    PSMTrackerID            TrackerIDs[PSMOVESERVICE_MAX_TRACKER_COUNT];
    int                     ValidTrackerLocations;

    // Multicam triangulated position and orientation, pre-filtered
    PSMVector3f             MulticamPositionCm;
    PSMQuatf                MulticamOrientation;
    bool                    bMulticamPositionValid;
    bool                    bMulticamOrientationValid;
} PSMRawTrackerData;

typedef struct _PSMPSMove
{
    bool                         bHasValidHardwareCalibration;
    bool                         bIsTrackingEnabled;
    bool                         bIsCurrentlyTracking;
    bool                         bIsOrientationValid;
    bool                         bIsPositionValid;
    bool                         bHasUnpublishedState;
    
    char                         DevicePath[256];
    char                         DeviceSerial[128];
    char                         AssignedHostSerial[128];
    bool                         PairedToHost;
    PSMConnectionType            ConnectionType;
    
    PSMTrackingColorType         TrackingColorType;
    PSMPosef                     Pose;
    PSMPhysicsData               PhysicsData;
    PSMPSMoveRawSensorData          RawSensorData;
    PSMPSMoveCalibratedSensorData   CalibratedSensorData;
    PSMRawTrackerData            RawTrackerData;
    
    PSMButtonState               TriangleButton;
    PSMButtonState               CircleButton;
    PSMButtonState               CrossButton;
    PSMButtonState               SquareButton;
    PSMButtonState               SelectButton;
    PSMButtonState               StartButton;
    PSMButtonState               PSButton;
    PSMButtonState               MoveButton;
    PSMButtonState               TriggerButton;
    PSMBatteryState              BatteryValue;
    unsigned char                TriggerValue;
    unsigned char                Rumble;
    unsigned char                LED_r, LED_g, LED_b;

    long long                    ResetPoseButtonPressTime;
    bool                         bResetPoseRequestSent;
    bool                         bPoseResetButtonEnabled;
    
} PSMPSMove;

typedef struct _PSMPSNavi
{
    PSMButtonState               L1Button;
    PSMButtonState               L2Button;
    PSMButtonState               L3Button;
    PSMButtonState               CircleButton;
    PSMButtonState               CrossButton;
    PSMButtonState               PSButton;
    PSMButtonState               TriggerButton;
    PSMButtonState               DPadUpButton;
    PSMButtonState               DPadRightButton;
    PSMButtonState               DPadDownButton;
    PSMButtonState               DPadLeftButton;
    unsigned char                TriggerValue;
    float                        Stick_XAxis;
    float                        Stick_YAxis;
} PSMPSNavi;

typedef struct _PSMDS4RawSensorData
{
    PSMVector3i Accelerometer;
    PSMVector3i Gyroscope;
    double      TimeInSeconds;
} PSMDS4RawSensorData;

typedef struct _PSMDS4CalibratedSensorData
{
    PSMVector3f Accelerometer;
    PSMVector3f Gyroscope;
    double      TimeInSeconds;
} PSMDS4CalibratedSensorData;

typedef struct _PSMDualShock4
{
    bool                         bHasValidHardwareCalibration;
    bool                         bIsTrackingEnabled;
    bool                         bIsCurrentlyTracking;
    bool                         bIsOrientationValid;
    bool                         bIsPositionValid;
    bool                         bHasUnpublishedState;
    
    char                         DevicePath[256];
    char                         DeviceSerial[128];
    char                         AssignedHostSerial[128];
    bool                         PairedToHost;
    PSMConnectionType            ConnectionType;
    
    PSMTrackingColorType         TrackingColorType;
    PSMPosef                     Pose;
    PSMPhysicsData               PhysicsData;
    PSMDS4RawSensorData           RawSensorData;
    PSMDS4CalibratedSensorData    CalibratedSensorData;
    PSMRawTrackerData            RawTrackerData;
    
    PSMButtonState               DPadUpButton;
    PSMButtonState               DPadDownButton;
    PSMButtonState               DPadLeftButton;
    PSMButtonState               DPadRightButton;

    PSMButtonState               SquareButton;
    PSMButtonState               CrossButton;
    PSMButtonState               CircleButton;
    PSMButtonState               TriangleButton;

    PSMButtonState               L1Button;
    PSMButtonState               R1Button;
    PSMButtonState               L2Button;
    PSMButtonState               R2Button;
    PSMButtonState               L3Button;
    PSMButtonState               R3Button;

    PSMButtonState               ShareButton;
    PSMButtonState               OptionsButton;

    PSMButtonState               PSButton;
    PSMButtonState               TrackPadButton;

    float                        LeftAnalogX;
    float                        LeftAnalogY;
    float                        RightAnalogX;
    float                        RightAnalogY;
    float                        LeftTriggerValue;
    float                        RightTriggerValue;

    unsigned char                BigRumble, SmallRumble;
    unsigned char                LED_r, LED_g, LED_b;

    long long                    ResetPoseButtonPressTime;
    bool                         bResetPoseRequestSent;
    bool                         bPoseResetButtonEnabled;
    
} PSMDualShock4;

typedef struct _PSMController
{
    PSMControllerID ControllerID;
    PSMControllerType ControllerType;
    union
    {
        PSMPSMove PSMoveState;
        PSMPSNavi PSNaviState;
		PSMDualShock4 PSDS4State;
    }               ControllerState;
    bool            bValid;
    int             OutputSequenceNum;
    int             InputSequenceNum;
    bool            IsConnected;
    long long       DataFrameLastReceivedTime;
    float           DataFrameAverageFPS;
    int             ListenerCount;
} PSMController;

// Tracker State
//--------------
typedef struct _PSMClientTrackerInfo
{
    // ID of the tracker in the service
    PSMTrackerID tracker_id;

    // Tracker USB properties
    PSMTrackerType tracker_type;
    PSMTrackerDriver tracker_driver;
    char device_path[128];

    // Video stream properties
    char shared_memory_name[64];

    // Camera Intrinsic properties
    PSMVector2f tracker_focal_lengths; // pixels
    PSMVector2f tracker_principal_point; // pixels
    PSMVector2f tracker_screen_dimensions; // pixels
    float tracker_hfov; // degrees
    float tracker_vfov; // degrees
    float tracker_znear; // cm
    float tracker_zfar; // cm
    float tracker_k1;
    float tracker_k2;
    float tracker_k3;
    float tracker_p1;
    float tracker_p2;

    // Camera Extrinsic properties
    PSMPosef tracker_pose;
} PSMClientTrackerInfo;

typedef struct _PSMTracker
{
    // Tracker Static Properties
    PSMClientTrackerInfo tracker_info;

    // Tracker Streaming State
    int listener_count;
    bool is_connected;
    int sequence_num;
    long long data_frame_last_received_time;
    float data_frame_average_fps;

    // SharedVideoFrameReadOnlyAccessor used by config tool
    void *opaque_shared_memory_accesor;
} PSMTracker;

// HMD State
//----------
typedef struct _PSMMorpheusRawSensorData
{
    PSMVector3i Accelerometer;
    PSMVector3i Gyroscope;
    double      TimeInSeconds;
} PSMMorpheusRawSensorData;

typedef struct _PSMMorpheusCalibratedSensorData
{
    PSMVector3f Accelerometer;
    PSMVector3f Gyroscope;
    double      TimeInSeconds;
} PSMMorpheusCalibratedSensorData;

typedef struct _PSMMorpheus
{
    bool                         bIsTrackingEnabled;
    bool                         bIsCurrentlyTracking;
    bool                         bIsOrientationValid;
    bool                         bIsPositionValid;
    
    PSMPosef                     Pose;
    PSMPhysicsData               PhysicsData;
    PSMMorpheusRawSensorData     RawSensorData;
    PSMMorpheusCalibratedSensorData CalibratedSensorData;
    PSMRawTrackerData            RawTrackerData;
} PSMMorpheus;

typedef struct _PSMHeadMountedDisplay
{
    PSMHmdID HmdID;
    PSMHmdType HmdType;
    union
    {
        PSMMorpheus  MorpheusState;
    }               HmdState;
    bool            bValid;
    int             OutputSequenceNum;
    bool            IsConnected;
    long long       DataFrameLastReceivedTime;
    float           DataFrameAverageFPS;
    int             ListenerCount;
} PSMHeadMountedDisplay;

// Service Events
//------------------
typedef struct _PSMEventMessage
{
    enum eEventType
    {
        // Client Events
        PSMEvent_connectedToService,
        PSMEvent_failedToConnectToService,
        PSMEvent_disconnectedFromService,

        // Service Events
        PSMEvent_opaqueServiceEvent, // Need to have protocol access to see what kind of event this is
        PSMEvent_controllerListUpdated,
        PSMEvent_trackerListUpdated,
        PSMEvent_hmdListUpdated,
        PSMEvent_systemButtonPressed
    } event_type;

    // Opaque handle that can be converted to a <const PSMoveProtocol::Response *> pointer
    // using GET_PSMOVEPROTOCOL_EVENT(handle) if you linked against the PSMoveProtocol lib.
    PSMEventDataHandle event_data_handle;
} PSMEventMessage;

// Service Responses
//------------------
typedef struct _PSMServiceVersion
{
	char version_string[PSMOVESERVICE_MAX_VERSION_STRING_LEN];
} PSMServiceVersion;

typedef struct _PSMControllerList
{
    PSMControllerID controller_id[PSMOVESERVICE_MAX_CONTROLLER_COUNT];
    PSMControllerType controller_type[PSMOVESERVICE_MAX_CONTROLLER_COUNT];
	char controller_serial[PSMOVESERVICE_MAX_CONTROLLER_COUNT][PSMOVESERVICE_CONTROLLER_SERIAL_LEN];
	char parent_controller_serial[PSMOVESERVICE_MAX_CONTROLLER_COUNT][PSMOVESERVICE_CONTROLLER_SERIAL_LEN];
    int count;
} PSMControllerList;

typedef struct _PSMTrackerList
{
    PSMClientTrackerInfo trackers[PSMOVESERVICE_MAX_TRACKER_COUNT];
    int count;
    float global_forward_degrees;
} PSMTrackerList;

typedef struct _PSMHmdList
{
    PSMHmdID hmd_id[PSMOVESERVICE_MAX_HMD_COUNT];
    PSMHmdType hmd_type[PSMOVESERVICE_MAX_HMD_COUNT];
    int count;
} PSMHmdList;

typedef struct _PSMTrackingSpace
{
    float global_forward_degrees;
} PSMTrackingSpace;


typedef struct _PSMResponseMessage
{
    // Fields common to all responses
    //----
    // The id of the request this response is from
    PSMRequestID request_id;

    // Whether this request succeeded, failed, or was canceled
    PSMResult result_code;

    // Opaque handle that can be converted to a <const PSMoveProtocol::Request *> pointer
    // using GET_PSMOVEPROTOCOL_REQUEST(handle) if you linked against the PSMoveProtocol lib.
    PSMResponseHandle opaque_request_handle;

    // Opaque handle that can be converted to a <const PSMoveProtocol::Response *> pointer
    // using GET_PSMOVEPROTOCOL_RESPONSE(handle) if you linked against the PSMoveProtocol lib.
    PSMResponseHandle opaque_response_handle;

    // Payload data specific to a subset of the responses
    //----
    union
    {
		PSMServiceVersion service_version;
        PSMControllerList controller_list;
        PSMTrackerList tracker_list;
		PSMHmdList hmd_list;
        PSMTrackingSpace tracking_space;
    } payload;

    enum eResponsePayloadType
    {
        _responsePayloadType_Empty,
		_responsePayloadType_ServiceVersion,
        _responsePayloadType_ControllerList,
        _responsePayloadType_TrackerList,
        _responsePayloadType_TrackingSpace,
		_responsePayloadType_HmdList,

        _responsePayloadType_Count
    } payload_type;
} PSMResponseMessage;

typedef void(*PSMResponseCallback)(const PSMResponseMessage *response, void *userdata);

// Message Container
//------------------
typedef struct _PSMMessage
{
    union{
        PSMEventMessage event_data;
        PSMResponseMessage response_data;
    };

    enum eMessagePayloadType
    {
        _messagePayloadType_Event,
        _messagePayloadType_Response,

        _messagePayloadType_Count
    } payload_type;
} PSMMessage;

// Interface
//----------

// Blocking Connection Methods
PSM_PUBLIC_FUNCTION(PSMResult) PSM_Initialize(const char* host, const char* port, int timeout_ms);  //"localhost", "9512"
PSM_PUBLIC_FUNCTION(PSMResult) PSM_Shutdown();

/// Async Connection Methods
PSM_PUBLIC_FUNCTION(PSMResult) PSM_InitializeAsync(const char* host, const char* port);  //"localhost", "9512"

/// Update
PSM_PUBLIC_FUNCTION(PSMResult) PSM_Update();
PSM_PUBLIC_FUNCTION(PSMResult) PSM_UpdateNoPollMessages();

/// System State Queries
PSM_PUBLIC_FUNCTION(const char*) PSM_GetClientVersionString();
PSM_PUBLIC_FUNCTION(bool) PSM_GetIsInitialized();
PSM_PUBLIC_FUNCTION(bool) PSM_GetIsConnected();
PSM_PUBLIC_FUNCTION(bool) PSM_HasConnectionStatusChanged();
PSM_PUBLIC_FUNCTION(bool) PSM_HasControllerListChanged();
PSM_PUBLIC_FUNCTION(bool) PSM_HasTrackerListChanged();
PSM_PUBLIC_FUNCTION(bool) PSM_HasHMDListChanged();
PSM_PUBLIC_FUNCTION(bool) PSM_WasSystemButtonPressed();

/// System Blocking Queries
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetServiceVersionString(char *out_version_string, size_t max_version_string, int timeout_ms);

/// System Async Queries
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetServiceVersionStringAsync(PSMRequestID *out_request_id);

/// Async Message Handling API
PSM_PUBLIC_FUNCTION(PSMResult) PSM_PollNextMessage(PSMMessage *message, size_t message_size);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_SendOpaqueRequest(PSMRequestHandle request_handle, PSMRequestID *out_request_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_RegisterCallback(PSMRequestID request_id, PSMResponseCallback callback, void *callback_userdata);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_CancelCallback(PSMRequestID request_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_EatResponse(PSMRequestID request_id);

/// Controller Pool
PSM_PUBLIC_FUNCTION(PSMController *) PSM_GetController(PSMControllerID controller_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_AllocateControllerListener(PSMControllerID controller_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_FreeControllerListener(PSMControllerID controller_id);

/// Blocking Controller Methods
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetControllerList(PSMControllerList *out_controller_list, int timeout_ms);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StartControllerDataStream(PSMControllerID controller_id, unsigned int data_stream_flags, int timeout_ms);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StopControllerDataStream(PSMControllerID controller_id, int timeout_ms);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_SetControllerLEDTrackingColor(PSMControllerID controller_id, PSMTrackingColorType tracking_color, int timeout_ms);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_ResetControllerOrientation(PSMControllerID controller_id, PSMQuatf *q_pose, int timeout_ms);

/// Controller State Methods
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetControllerOrientation(PSMControllerID controller_id, PSMQuatf *out_orientation);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetControllerPosition(PSMControllerID controller_id, PSMVector3f *out_position);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetControllerPose(PSMControllerID controller_id, PSMPosef *out_pose);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetControllerRumble(PSMControllerID controller_id, PSMControllerRumbleChannel channel, float *out_rumbleFraction);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetIsControllerStable(PSMControllerID controller_id, bool *out_is_stable);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetIsControllerTracking(PSMControllerID controller_id, bool *out_is_tracking);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetControllerPixelLocationOnTracker(PSMControllerID controller_id, PSMTrackerID tracker_id, PSMVector2f *outLocation);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetControllerPositionOnTracker(PSMControllerID controller_id, PSMTrackerID tracker_id, PSMVector3f *outPosition);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetControllerOrientationOnTracker(PSMControllerID controller_id, PSMTrackerID tracker_id, PSMQuatf *outOrientation);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetControllerProjectionOnTracker(PSMControllerID controller_id, PSMTrackerID tracker_id, PSMTrackingProjection *outProjection);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_SetControllerLEDOverrideColor(PSMControllerID controller_id, unsigned char r, unsigned char g, unsigned char b);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_SetControllerRumble(PSMControllerID controller_id, PSMControllerRumbleChannel channel, float rumbleFraction);

/// Async Controller Methods
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetControllerListAsync(PSMRequestID *out_request_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StartControllerDataStreamAsync(PSMControllerID controller_id, unsigned int data_stream_flags, PSMRequestID *out_request_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StopControllerDataStreamAsync(PSMControllerID controller_id, PSMRequestID *out_request_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_SetControllerLEDColorAsync(PSMControllerID controller_id, PSMTrackingColorType tracking_color, PSMRequestID *out_request_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_ResetControllerOrientationAsync(PSMControllerID controller_id, const PSMQuatf *q_pose, PSMRequestID *out_request_id);

/// Tracker Pool
PSM_PUBLIC_FUNCTION(PSMTracker *) PSM_GetTracker(PSMTrackerID tracker_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_AllocateTrackerListener(PSMTrackerID tracker_id, const PSMClientTrackerInfo *tracker_info);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_FreeTrackerListener(PSMTrackerID controller_id);

/// Tracker State Methods
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackerIntrinsicMatrix(PSMTrackerID tracker_id, PSMMatrix3f *out_matrix);

/// Blocking Tracker Methods
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackerList(PSMTrackerList *out_tracker_list, int timeout_ms);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StartTrackerDataStream(PSMTrackerID tracker_id, int timeout_ms);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StopTrackerDataStream(PSMTrackerID tracker_id, int timeout_ms);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackingSpaceSettings(PSMTrackingSpace *out_tracking_space, int timeout_ms);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_OpenTrackerVideoStream(PSMTrackerID tracker_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_PollTrackerVideoStream(PSMTrackerID tracker_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_CloseTrackerVideoStream(PSMTrackerID tracker_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackerVideoFrameBuffer(PSMTrackerID tracker_id, const unsigned char **out_buffer); 
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackerFrustum(PSMTrackerID tracker_id, PSMFrustum *out_frustum);

/// Async Tracker Methods
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackerListAsync(PSMRequestID *out_request_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StartTrackerDataStreamAsync(PSMTrackerID tracker_id, PSMRequestID *out_request_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StopTrackerDataStreamAsync(PSMTrackerID tracker_id, PSMRequestID *out_request_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackingSpaceSettingsAsync(PSMRequestID *out_request_id);

/// HMD Pool
PSM_PUBLIC_FUNCTION(PSMHeadMountedDisplay *) PSM_GetHmd(PSMHmdID hmd_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_AllocateHmdListener(PSMHmdID hmd_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_FreeHmdListener(PSMHmdID hmd_id);

/// HMD State Methods
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdOrientation(PSMHmdID hmd_id, PSMQuatf *out_orientation);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdPosition(PSMHmdID hmd_id, PSMVector3f *out_position);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdPose(PSMHmdID hmd_id, PSMPosef *out_pose);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetIsHmdStable(PSMHmdID hmd_id, bool *out_is_stable);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetIsHmdTracking(PSMHmdID hmd_id, bool *out_is_tracking);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdPixelLocationOnTracker(PSMHmdID hmd_id, PSMTrackerID tracker_id, PSMVector2f *outLocation);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdPositionOnTracker(PSMHmdID hmd_id, PSMTrackerID tracker_id, PSMVector3f *outPosition);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdOrientationOnTracker(PSMHmdID hmd_id, PSMTrackerID tracker_id, PSMQuatf *outOrientation);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdProjectionOnTracker(PSMHmdID hmd_id, PSMTrackerID tracker_id, PSMTrackingProjection *outProjection);

/// Blocking HMD Methods
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdList(PSMHmdList *out_hmd_list, int timeout_ms);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StartHmdDataStream(PSMHmdID hmd_id, unsigned int data_stream_flags, int timeout_ms);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StopHmdDataStream(PSMHmdID hmd_id, int timeout_ms);

/// Async HMD Methods
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdListAsync(PSMRequestID *out_request_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StartHmdDataStreamAsync(PSMHmdID hmd_id, unsigned int data_stream_flags, PSMRequestID *out_request_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StopHmdDataStreamAsync(PSMHmdID hmd_id, PSMRequestID *out_request_id);

//cut_after
#endif

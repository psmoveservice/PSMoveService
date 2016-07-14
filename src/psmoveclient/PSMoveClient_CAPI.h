#ifndef __PSMOVECLIENT_CAPI_H
#define __PSMOVECLIENT_CAPI_H
#include "PSMoveClient_export.h"
#include "ClientConstants.h"
#include <stdbool.h>

// Wrapper Types
//--------------
typedef int PSMRequestID;
typedef const void * PSMResponseHandle;
typedef void *PSMRequestHandle;
typedef const void *PSMEventDataHandle;

typedef int PSMControllerID;
typedef int PSMTrackerID;

// Shared Constants
//-----------------

enum _PSMResult
{
    PSMResult_Error                 = -1,
    PSMResult_Success               = 0,
    PSMResult_Timeout               = 1,
    PSMResult_RequestSent           = 2,
    PSMResult_Canceled              = 3,
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
} PSMTrackingColorType;

typedef enum _PSMControllerDataStreamFlags
{
    PSMStreamFlags_defaultStreamOptions = 0x00,
    PSMStreamFlags_includePositionData = 0x01,
    PSMStreamFlags_includePhysicsData = 0x02,
    PSMStreamFlags_includeRawSensorData = 0x04,
    PSMStreamFlags_includeRawTrackerData = 0x08
} PSMControllerDataStreamFlags;

typedef enum _PSMControllerType
{
    PSMController_None= -1,
    PSMController_Move,
    PSMController_Navi
} PSMControllerType;

typedef enum _PSMTrackerType
{
    PSMTracker_None= -1,
    PSMTracker_PS3Eye
} PSMTrackerType;

typedef enum _PSMTrackerDriver
{
    PSMDriver_LIBUSB,
    PSMDriver_CL_EYE,
    PSMDriver_CL_EYE_MULTICAM,
    PSMDriver_GENERIC_WEBCAM
} PSMTrackerDriver;

// Client Geometry
//----------------

/// A 2D vector with float components.
typedef struct _PSMVector2f
{
    float x, y;
} PSMVector2f;

/// A 3D vector with float components.
typedef struct _PSMVector3f
{
    float x, y, z;
} PSMVector3f;

/// A 3D vector with int components.
typedef struct _PSMVector3i
{
    int x, y, z;
} PSMVector3i;

/// A quaternion rotation.
typedef struct _PSMQuatf
{
    float x, y, z, w;
} PSMQuatf;

/// Position and orientation together.
typedef struct _PSMPosef
{
    PSMVector3f  Position;
    PSMQuatf     Orientation;
} PSMPosef;

typedef struct _PSMovePhysicsData
{
    PSMVector3f LinearVelocity;
    PSMVector3f LinearAcceleration;
    PSMVector3f AngularVelocity;
    PSMVector3f AngularAcceleration;
    double       TimeInSeconds;
} PSMPhysicsData;

typedef struct _PSMRawSensorData
{
    PSMVector3i Magnetometer;
    PSMVector3f Accelerometer;
    PSMVector3f Gyroscope;
    double      TimeInSeconds;
} PSMRawSensorData;

typedef struct _PSMTrackingProjection
{
    enum eShapeType
    {
        PSMShape_INVALID_PROJECTION = -1,
        PSMShape_Ellipse,
        PSMShape_Quad,
    }                               shape_type;
    union{
        struct {
            PSMVector2f center;
            float half_x_extent;
            float half_y_extent;
            float angle;
        } ellipse;
        struct {
            PSMVector2f corners[4];
        } quad;
    }                               shape;
    
} PSMTrackingProjection;

// Controller State
//------------------
typedef struct _PSMRawTrackerData
{
    // Parallel arrays: ScreenLocations, Positions and the TrackerID associated with them
    PSMVector2f             ScreenLocations[PSMOVESERVICE_MAX_TRACKER_COUNT];
    PSMVector3f             RelativePositions[PSMOVESERVICE_MAX_TRACKER_COUNT];
    PSMTrackingProjection   TrackingProjections[PSMOVESERVICE_MAX_TRACKER_COUNT];
    int                     TrackerIDs[PSMOVESERVICE_MAX_TRACKER_COUNT];
    int                     ValidTrackerLocations;
} PSMRawTrackerData;

typedef struct _PSMPSMove
{
    bool                    bHasValidHardwareCalibration;
    bool                    bIsTrackingEnabled;
    bool                    bIsCurrentlyTracking;
    bool                    bHasUnpublishedState;
    
    char                    DevicePath[256];
    char                    DeviceSerial[128];
    char                    AssignedHostSerial[128];
    bool                 PairedToHost;
    PSMConnectionType       ConnectionType;
    
    PSMTrackingColorType    TrackingColorType;
    PSMPosef                Pose;
    PSMPhysicsData          PhysicsData;
    PSMRawSensorData        RawSensorData;
    PSMRawTrackerData       RawTrackerData;
    
    PSMButtonState          TriangleButton;
    PSMButtonState          CircleButton;
    PSMButtonState          CrossButton;
    PSMButtonState          SquareButton;
    PSMButtonState          SelectButton;
    PSMButtonState          StartButton;
    PSMButtonState          PSButton;
    PSMButtonState          MoveButton;
    PSMButtonState          TriggerButton;
    unsigned char           TriggerValue;
    unsigned char           Rumble;
    unsigned char           LED_r, LED_g, LED_b;
    
} PSMPSMove;

typedef struct _PSMPSNavi
{
    PSMButtonState L1Button;
    PSMButtonState L2Button;
    PSMButtonState L3Button;
    PSMButtonState CircleButton;
    PSMButtonState CrossButton;
    PSMButtonState PSButton;
    PSMButtonState TriggerButton;
    PSMButtonState DPadUpButton;
    PSMButtonState DPadRightButton;
    PSMButtonState DPadDownButton;
    PSMButtonState DPadLeftButton;
    unsigned char TriggerValue;
    unsigned char Stick_XAxis;
    unsigned char Stick_YAxis;
} PSMPSNavi;

typedef struct _PSMController
{
    PSMControllerID ControllerID;
    PSMControllerType ControllerType;
    union
    {
        PSMPSMove PSMoveState;
        PSMPSNavi PSNaviState;
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
    } event_type;

    // Opaque handle that can be converted to a <const PSMoveProtocol::Response *> pointer
    // using GET_PSMOVEPROTOCOL_EVENT(handle) if you linked against the PSMoveProtocol lib.
    PSMEventDataHandle event_data_handle;
} PSMEventMessage;

// Service Responses
//------------------
typedef struct _PSMControllerList
{
    PSMControllerID controller_id[PSMOVESERVICE_MAX_CONTROLLER_COUNT];
    PSMControllerType controller_type[PSMOVESERVICE_MAX_CONTROLLER_COUNT];
    int count;
} PSMControllerList;

typedef struct _PSMTrackerList
{
    PSMClientTrackerInfo trackers[PSMOVESERVICE_MAX_TRACKER_COUNT];
    int count;
} PSMTrackerList;

typedef struct _PSMHMDTrackingSpace
{
    PSMPosef origin_pose;
} PSMHMDTrackingSpace;

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
        PSMControllerList controller_list;
        PSMTrackerList tracker_list;
        PSMHMDTrackingSpace hmd_tracking_space;
    } payload;

    enum eResponsePayloadType
    {
        _responsePayloadType_Empty,
        _responsePayloadType_ControllerList,
        _responsePayloadType_TrackerList,
        _responsePayloadType_HMDTrackingSpace,

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

// Async Connection Methods
PSM_PUBLIC_FUNCTION(PSMResult) PSM_InitializeAsync(const char* host, const char* port);  //"localhost", "9512"

// Update
PSM_PUBLIC_FUNCTION(PSMResult) PSM_Update();
PSM_PUBLIC_FUNCTION(PSMResult) PSM_UpdateNoPollMessages();

// System Queries
PSM_PUBLIC_FUNCTION(const char*) PSM_GetVersionString();
PSM_PUBLIC_FUNCTION(bool) PSM_GetIsConnected();
PSM_PUBLIC_FUNCTION(bool) PSM_HasConnectionStatusChanged();
PSM_PUBLIC_FUNCTION(bool) PSM_HasControllerListChanged();
PSM_PUBLIC_FUNCTION(bool) PSM_HasTrackerListChanged();

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
PSM_PUBLIC_FUNCTION(PSMResult) PSM_SetControllerLEDColor(PSMControllerID controller_id, PSMTrackingColorType tracking_color, int timeout_ms);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_ResetControllerPose(PSMControllerID controller_id, int timeout_ms);

/// Async Controller Methods
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetControllerListAsync(PSMRequestID *out_request_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StartControllerDataStreamAsync(PSMControllerID controller_id, unsigned int data_stream_flags, PSMRequestID *out_request_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StopControllerDataStreamAsync(PSMControllerID controller_id, PSMRequestID *out_request_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_SetControllerLEDColorAsync(PSMControllerID controller_id, PSMTrackingColorType tracking_color, PSMRequestID *out_request_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_ResetControllerPoseAsync(PSMControllerID controller_id, PSMRequestID *out_request_id);

/// Tracker Pool
PSM_PUBLIC_FUNCTION(PSMTracker *) PSM_GetTracker(PSMTrackerID tracker_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_AllocateTrackerListener(PSMTrackerID tracker_id, PSMClientTrackerInfo *tracker_info);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_FreeTrackerListener(PSMTrackerID controller_id);

/// Blocking Tracker Methods
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackerList(PSMTrackerList *out_tracker_list, int timeout_ms);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StartTrackerDataStream(PSMTrackerID tracker_id, int timeout_ms);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StopTrackerDataStream(PSMTrackerID tracker_id, int timeout_ms);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHMDTrackingSpaceSettings(PSMHMDTrackingSpace *out_tracking_space, int timeout_ms);

/// Async Tracker Methods
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackerListAsync(PSMRequestID *out_request_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StartTrackerDataStreamAsync(PSMTrackerID tracker_id, PSMRequestID *out_request_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StopTrackerDataStreamAsync(PSMTrackerID tracker_id, PSMRequestID *out_request_id);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHMDTrackingSpaceSettingsAsync(PSMRequestID *out_request_id);

#endif
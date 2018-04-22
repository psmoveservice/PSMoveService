/**
\file
*/ 

#ifndef __PSMOVECLIENT_CAPI_H
#define __PSMOVECLIENT_CAPI_H
#include "PSMoveClient_export.h"
#include "ClientConstants.h"
#include "ClientGeometry_CAPI.h"
#include <stdbool.h>
//cut_before

/** 
\brief Client Interface for PSMoveService
\defgroup PSMoveClient_CAPI Client Interface
\addtogroup PSMoveClient_CAPI 
@{ 
*/
 
// Wrapper Types
//--------------

/// The ID of a pending request send to PSMoveService
typedef int PSMRequestID;

/// Opaque handle to the internal protocol buffer response
typedef const void * PSMResponseHandle;

/// Opaque handle to the internal protocol buffer request
typedef void *PSMRequestHandle;

/// opaque handle to the internal protocol buffer event
typedef const void *PSMEventDataHandle;

/// The ID of a controller in the controller pool
typedef int PSMControllerID;

/// The ID of a tracker in the tracker pool
typedef int PSMTrackerID;

/// The ID of an HMD in the HMD pool
typedef int PSMHmdID;

// Shared Constants
//-----------------

/// Invalid Request ID constant
const PSMRequestID PSM_INVALID_REQUEST_ID = -1;

/// Result enum in response to a client API request
typedef enum
{
    PSMResult_Error                 = -1, 	///< General Error Result
    PSMResult_Success               = 0,	///< General Success Result
    PSMResult_Timeout               = 1,	///< Requested Timed Out
    PSMResult_RequestSent           = 2,	///< Request Successfully Sent
    PSMResult_Canceled              = 3,	///< Request Canceled 
    PSMResult_NoData                = 4,	///< Request Returned No Data
} PSMResult;

/// Connection type for a device
typedef enum
{
    PSMConnectionType_BLUETOOTH,	///< Device connected over bluetooth
    PSMConnectionType_USB			///< Device connected over USB
    
} PSMConnectionType;

/// De-bounced state of a button
typedef enum 
{
    PSMButtonState_UP = 0x00,       ///< (00b) Not pressed
    PSMButtonState_PRESSED = 0x01,  ///< (01b) Down for one frame only
    PSMButtonState_DOWN = 0x03,     ///< (11b) Down for >1 frame
    PSMButtonState_RELEASED = 0x02, ///< (10b) Up for one frame only
} PSMButtonState;

/// The available tracking color types
typedef enum
{
    PSMTrackingColorType_Magenta,    ///< R:0xFF, G:0x00, B:0xFF
    PSMTrackingColorType_Cyan,       ///< R:0x00, G:0xFF, B:0xFF
    PSMTrackingColorType_Yellow,     ///< R:0xFF, G:0xFF, B:0x00
    PSMTrackingColorType_Red,        //</ R:0xFF, G:0x00, B:0x00
    PSMTrackingColorType_Green,      ///< R:0x00, G:0xFF, B:0x00
    PSMTrackingColorType_Blue,       ///< R:0x00, G:0x00, B:0xFF
	
	PSMTrackingColorType_MaxColorTypes
} PSMTrackingColorType;

/// Battery charge state levels
typedef enum
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

/// Tracked device data stream options
typedef enum
{
    PSMStreamFlags_defaultStreamOptions = 0x00,			///< Default minimal data stream
    PSMStreamFlags_includePositionData = 0x01,			///< Add position data (turns on tracking)
    PSMStreamFlags_includePhysicsData = 0x02,			///< Add IMU physics state
    PSMStreamFlags_includeRawSensorData = 0x04,			///< Add raw IMU sensor data
	PSMStreamFlags_includeCalibratedSensorData = 0x08,	///< Add calibrated IMU sensor state
    PSMStreamFlags_includeRawTrackerData = 0x10,		///< Add raw optical tracking projection info
	PSMStreamFlags_disableROI = 0x20,					///< Disable Region-of-Interest tracking optimization
} PSMControllerDataStreamFlags;

/// The possible rumble channels available to the comtrollers
typedef enum
{
    PSMControllerRumbleChannel_All,		///< Rumble across all channels
    PSMControllerRumbleChannel_Left,	///< Rumble on the left channel
    PSMControllerRumbleChannel_Right	///< Runble on the right channel
} PSMControllerRumbleChannel;

/// The list of possible controller types tracked by PSMoveService
typedef enum
{
    PSMController_None= -1,
    PSMController_Move,
    PSMController_Navi,
	PSMController_DualShock4,
    PSMController_Virtual
} PSMControllerType;

/// Describes which hand the given device is intended for
typedef enum 
{
	PSMControllerHand_Any = 0,
	PSMControllerHand_Left = 1,
	PSMControllerHand_Right = 2,
} PSMControllerHand;

/// The list of possible camera types tracked by PSMoveService
typedef enum
{
    PSMTracker_None= -1,
    PSMTracker_PS3Eye
} PSMTrackerType;

/// The list of possible HMD types tracked by PSMoveService
typedef enum
{
    PSMHmd_None= -1,
	PSMHmd_Morpheus= 0,
    PSMHmd_Virtual= 1,
} PSMHmdType;

/// The list of possible camera drivers used by PSMoveService
typedef enum
{
    PSMDriver_LIBUSB,
    PSMDriver_CL_EYE,
    PSMDriver_CL_EYE_MULTICAM,
    PSMDriver_GENERIC_WEBCAM
} PSMTrackerDriver;

// Controller State
//------------------

/// Tracked object physics data state
typedef struct
{
    PSMVector3f LinearVelocityCmPerSec;
    PSMVector3f LinearAccelerationCmPerSecSqr;
    PSMVector3f AngularVelocityRadPerSec;
    PSMVector3f AngularAccelerationRadPerSecSqr;
    double       TimeInSeconds;
} PSMPhysicsData;

/// Raw Sensor data from the PSMove IMU
typedef struct
{
    PSMVector3i Magnetometer;
    PSMVector3i Accelerometer;
    PSMVector3i Gyroscope;
    double      TimeInSeconds;
} PSMPSMoveRawSensorData;

/// Calibrated Sensor
typedef struct
{
    PSMVector3f Magnetometer;
    PSMVector3f Accelerometer;
    PSMVector3f Gyroscope;
    double      TimeInSeconds;
} PSMPSMoveCalibratedSensorData;

/// Device projection geometry as seen by each tracker
typedef struct
{
	/// ID of the selected tracker
    PSMTrackerID            TrackerID;
	/// Pixel position of device projection centroid on each tracker
    PSMVector2f             ScreenLocation;
	/// Tracker relative device 3d position on each tracker
    PSMVector3f             RelativePositionCm;
	/// Tracker relative device 3d orientation on each tracker
    PSMQuatf                RelativeOrientation;
	/// Tracker relative device projection geometry on each tracker
    PSMTrackingProjection   TrackingProjection;
	/// A bitmask of the trackers with valid projections
    unsigned int            ValidTrackerBitmask;

    // Multicam triangulated position and orientation, pre-filtered
	/// Optically derived world space position of device in cm
    PSMVector3f             MulticamPositionCm;
	/// Optically derived world space orientation of device in cm
    PSMQuatf                MulticamOrientation;
	/// Flag if the world space optical position is valid
    bool                    bMulticamPositionValid;
	/// Flag if the world space optical orientation is valid
    bool                    bMulticamOrientationValid;
} PSMRawTrackerData;

/// PSMove Controller State in Controller Pool Entry
typedef struct
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

/// PSNavi Controller State in Controller Pool Entry
typedef struct
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
    unsigned char                Stick_XAxis;
    unsigned char                Stick_YAxis;
} PSMPSNavi;

/// DualShock4 raw IMU sensor data
typedef struct
{
    PSMVector3i Accelerometer;
    PSMVector3i Gyroscope;
    double      TimeInSeconds;
} PSMDS4RawSensorData;

/// DualShock4 calibrated IMU sensor data
typedef struct
{
    PSMVector3f Accelerometer;
    PSMVector3f Gyroscope;
    double      TimeInSeconds;
} PSMDS4CalibratedSensorData;

/// DualShock4 Controller State in Controller Pool Entry
typedef struct
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

/// Virtual Controller State in Controller Pool Entry
typedef struct
{
    bool                         bIsTrackingEnabled;
    bool                         bIsCurrentlyTracking;
    bool                         bIsPositionValid;
    
    char                         DevicePath[256];

    int                          vendorID;
    int                          productID;
    
    int                          numAxes;
    int                          numButtons;
    
    unsigned char                axisStates[PSM_MAX_VIRTUAL_CONTROLLER_AXES];
    PSMButtonState               buttonStates[PSM_MAX_VIRTUAL_CONTROLLER_BUTTONS];
    
    PSMTrackingColorType         TrackingColorType;
    PSMPosef                     Pose;
    PSMPhysicsData               PhysicsData;
    PSMRawTrackerData            RawTrackerData;   
    
} PSMVirtualController;

/// Controller Pool Entry
typedef struct
{
    PSMControllerID ControllerID;
    PSMControllerType ControllerType;
	PSMControllerHand ControllerHand;
    union
    {
        PSMPSMove PSMoveState;
        PSMPSNavi PSNaviState;
		PSMDualShock4 PSDS4State;
        PSMVirtualController VirtualController;
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

/// Static properties about a tracker
typedef struct
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
    PSMVector2f tracker_focal_lengths; ///< lens focal length in pixels
    PSMVector2f tracker_principal_point; ///< lens center point in pixels
    PSMVector2f tracker_screen_dimensions; ///< tracker image size in pixels
    float tracker_hfov; ///< tracker horizontal FOV in degrees
    float tracker_vfov; ///< tracker vertical FOV in degrees
    float tracker_znear; ///< tracker z-near plane distance in cm
    float tracker_zfar; ///< tracker z-far plane distance in cm
    float tracker_k1; ///< lens distortion coefficient k1
    float tracker_k2; ///< lens distortion coefficient k2
    float tracker_k3; ///< lens distortion coefficient k3
    float tracker_p1; ///< lens distortion coefficient p1
    float tracker_p2; ///< lens distortion coefficient p2

    // Camera Extrinsic properties
    PSMPosef tracker_pose; ///< World space location of tracker (relative to calibration mat)
} PSMClientTrackerInfo;

/// Tracker Pool Entry
typedef struct
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

/// Morpheus Raw IMU sensor data
typedef struct
{
    PSMVector3i Accelerometer;
    PSMVector3i Gyroscope;
    double      TimeInSeconds;
} PSMMorpheusRawSensorData;

/// Morpheus Calibrated IMU sensor data
typedef struct
{
    PSMVector3f Accelerometer;
    PSMVector3f Gyroscope;
    double      TimeInSeconds;
} PSMMorpheusCalibratedSensorData;

/// Morpheus HMD State in HMD Pool Entry
typedef struct
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

/// Virtual HMD State in HMD Pool Entry
typedef struct
{
    bool                         bIsTrackingEnabled;
    bool                         bIsCurrentlyTracking;
    bool                         bIsPositionValid;
    
    PSMPosef                     Pose;
    PSMPhysicsData               PhysicsData;
    PSMRawTrackerData            RawTrackerData;
} PSMVirtualHMD;

/// HMD Pool Entry
typedef struct
{
    PSMHmdID HmdID;
    PSMHmdType HmdType;
    union
    {
        PSMMorpheus  MorpheusState;
        PSMVirtualHMD VirtualHMDState;
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

/// A container for all PSMoveService events
typedef struct
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

    /// Opaque handle that can be converted to a <const PSMoveProtocol::Response *> pointer
    /// using GET_PSMOVEPROTOCOL_EVENT(handle) if you linked against the PSMoveProtocol lib.
    PSMEventDataHandle event_data_handle;
} PSMEventMessage;

// Service Responses
//------------------

/// Current version of PSMoveService
typedef struct
{
	char version_string[PSMOVESERVICE_MAX_VERSION_STRING_LEN];
} PSMServiceVersion;

/// List of controllers attached to PSMoveService
typedef struct
{
    PSMControllerID controller_id[PSMOVESERVICE_MAX_CONTROLLER_COUNT];
    PSMControllerType controller_type[PSMOVESERVICE_MAX_CONTROLLER_COUNT];
	PSMControllerHand controller_hand[PSMOVESERVICE_MAX_CONTROLLER_COUNT];
	char controller_serial[PSMOVESERVICE_MAX_CONTROLLER_COUNT][PSMOVESERVICE_CONTROLLER_SERIAL_LEN];
	char parent_controller_serial[PSMOVESERVICE_MAX_CONTROLLER_COUNT][PSMOVESERVICE_CONTROLLER_SERIAL_LEN];
    int count;
} PSMControllerList;

/// List of trackers connected to PSMoveService
typedef struct
{
    PSMClientTrackerInfo trackers[PSMOVESERVICE_MAX_TRACKER_COUNT];
    int count;
    float global_forward_degrees;
} PSMTrackerList;

/// List of HMDs connected to PSMoveSerivce
typedef struct
{
    PSMHmdID hmd_id[PSMOVESERVICE_MAX_HMD_COUNT];
    PSMHmdType hmd_type[PSMOVESERVICE_MAX_HMD_COUNT];
    int count;
} PSMHmdList;

/// Tracking Space Parameters
typedef struct
{
    float global_forward_degrees;
} PSMTrackingSpace;

/// A contrainer for all possible responses to requests sent from PSMoveService
typedef struct
{
    /// The id of the request this response is from
    PSMRequestID request_id;

    /// Whether this request succeeded, failed, or was canceled
    PSMResult result_code;

    /// Opaque handle that can be converted to a <const PSMoveProtocol::Request *> pointer
    /// using GET_PSMOVEPROTOCOL_REQUEST(handle) if you linked against the PSMoveProtocol lib.
    PSMResponseHandle opaque_request_handle;

    /// Opaque handle that can be converted to a <const PSMoveProtocol::Response *> pointer
    /// using GET_PSMOVEPROTOCOL_RESPONSE(handle) if you linked against the PSMoveProtocol lib.
    PSMResponseHandle opaque_response_handle;

    /// Payload data specific to a subset of the responses
    union
    {
		PSMServiceVersion service_version;	///< Response to service version request
        PSMControllerList controller_list;	///< Response to controller list request
        PSMTrackerList tracker_list;		///< Response to tracker list request
		PSMHmdList hmd_list;				///< Response to hmd list request
        PSMTrackingSpace tracking_space;	///< Response to tracking space request
    } payload;

	/// Type of response sent from PSMoveService
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

/// Registered response callback function for a PSMoveService request
typedef void(*PSMResponseCallback)(const PSMResponseMessage *response, void *userdata);

// Message Container
//------------------

/// Message container for responses and events sent from PSMoveService
typedef struct
{
    union{
        PSMEventMessage event_data;			///< Event sent about something changing in PSMoveService
        PSMResponseMessage response_data;	///< Response sent from PSMoveService` in reply to a request 
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
/** \brief Initializes a connection to PSMoveService.
 Attempts to connect to PSMService at the given address and port. 
 This function must be called before calling any other client functions. 
 Calling this function again after a connection is already started will return PSMResult_Success.

 \remark Blocking - Returns after either a connection is successfully established OR the timeout period is reached. 
 \param host The address that PSMoveService is running at, usually PSMOVESERVICE_DEFAULT_ADDRESS
 \param port The port that PSMoveSerive is running at, usually PSMOVESERVICE_DEFAULT_PORT
 \param timeout The conection timeout period in milliseconds, usually PSM_DEFAULT_TIMEOUT
 \returns PSMResult_Success on success, PSMResult_Timeout, or PSMResult_Error on a general connection error.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_Initialize(const char* host, const char* port, int timeout_ms); 

/** \brief Shuts down connection to PSMoveService
 Closes an active connection to PSMoveService and cleans out any pending requests. 
 This function should be called when closing down the client OR to reset a client connection.
 Calling this function again after a connection is alread closed will return PSMResult_Error.

  \returns PSMResult_Success on success or PSMResult_Error if there was no valid connection.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_Shutdown();

// Async Connection Methods
/** \brief Initializes a connection to PSMoveService.
 Starts connection process to PSMService at the given address and port. 
 This function must be called before calling any other client functions. 
 Calling this function again after a connection has already been requested will return PSMResult_RequestSent.

 \remark Async - Starts a request for login. Connection status can be tested one of two ways:
  - Call \ref PSM_Update() and then test \ref PSM_HasConnectionStatusChanged() is true
  - Call \ref PSM_UpdateNoPollMessages() and then call \ref PSM_PollNextMessage() to see if an event is received:
    - \ref PSMEvent_connectedToService
    - \ref PSMEvent_failedToConnectToService
	.   
 \param host The address that PSMoveService is running at, usually PSMOVESERVICE_DEFAULT_ADDRESS
 \param port The port that PSMoveSerive is running at, usually PSMOVESERVICE_DEFAULT_PORT
 \param timeout The conection timeout period in milliseconds, usually PSM_DEFAULT_TIMEOUT
 \returns PSMResult_RequestSent on success, PSMResult_Timeout, or PSMResult_Error on a general connection error.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_InitializeAsync(const char* host, const char* port);

// Update
/** \brief Poll the connection and process messages.
	This function will poll the connection for new messages from PSMoveService.
	If new events are received they are processed right away and the the appropriate status flag will be set.
	Any received responses that had a callback assigned to them will be called.
	The following state polling functions can be called after an update:
	  - \ref PSM_GetIsConnected()
	  - \ref PSM_HasConnectionStatusChanged()
	  - \ref PSM_HasControllerListChanged()
	  - \ref PSM_HasTrackerListChanged()
	  - \ref PSM_HasHMDListChanged()
	  - \ref PSM_WasSystemButtonPressed()
	  
	\return PSMResult_Success if there is an active connection or PSMResult_Error if there is no valid connection
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_Update();

/** \brief Poll the connection and DO NOT process messages.
	This function will poll the connection for new messages from PSMoveService.
	If new events are received they are put in a queue. The messages are extracted using \ref PSM_PollNextMessage().
	Messages not read from the queue will get cleared on the next update call. 
	
	\return PSMResult_Success if there is an active connection or PSMResult_Error if there is no valid connection
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_UpdateNoPollMessages();

// System State Queries
/** \brief Get the client API version string 
	\return A zero-terminated version string of the format "Product.Major-Phase Minor.Release.Hotfix", ex: "0.9-alpha 8.1.0"
 */
PSM_PUBLIC_FUNCTION(const char*) PSM_GetClientVersionString();

/** \brief Get the API initialization status
	\return true if the client API is initialized
 */
PSM_PUBLIC_FUNCTION(bool) PSM_GetIsInitialized();

/** \brief Get the client connection status
	\return true if the client is connected to PSMoveService
 */
PSM_PUBLIC_FUNCTION(bool) PSM_GetIsConnected();

/** \brief Get the connection status change flag
	This flag is only filled in when \ref PSM_Update() is called.
	If you instead call PSM_UpdateNoPollMessages() you'll need to process the event queue yourself to get connection
	status change events.
	
	\return true if the client is connection status changed
 */
PSM_PUBLIC_FUNCTION(bool) PSM_HasConnectionStatusChanged();

/** \brief Get the controller list change flag
	This flag is only filled in when \ref PSM_Update() is called.
	If you instead call PSM_UpdateNoPollMessages() you'll need to process the event queue yourself to get controller
	list change events.
	
	\return true if the controller list changed
 */
PSM_PUBLIC_FUNCTION(bool) PSM_HasControllerListChanged();

/** \brief Get the tracker list change flag
	This flag is only filled in when \ref PSM_Update() is called.
	If you instead call PSM_UpdateNoPollMessages() you'll need to process the event queue yourself to get tracker
	list change events.
	
	\return true if the tracker list changed
 */
PSM_PUBLIC_FUNCTION(bool) PSM_HasTrackerListChanged();

/** \brief Get the HMD list change flag
	This flag is only filled in when \ref PSM_Update() is called.
	If you instead call PSM_UpdateNoPollMessages() you'll need to process the event queue yourself to get HMD
	list change events.
	
	\return true if the HMD list changed
 */
PSM_PUBLIC_FUNCTION(bool) PSM_HasHMDListChanged();

/** \brief Get the "any system button pressed" flag
	This flag is only filled in when \ref PSM_Update() is called.
	If you instead call PSM_UpdateNoPollMessages() you'll need to process the event queue yourself to get HMD
	list change events.
	This is useful if the client need to send a "wake up" notification on system button press.
	
	\return true if the system button (i.e. the "PS" button) was pressed on any controller this update.
 */
PSM_PUBLIC_FUNCTION(bool) PSM_WasSystemButtonPressed();

// System Blocking Queries
/** \brief Get the client API version string from PSMoveService
	Sends a request to PSMoveService to get the protocol version.
	This should be compared against the version returned from \ref PSM_GetClientVersionString() as a way to verify
	that the an outdated client isn't being used with PSMoveService.
	\remark Blocking - Returns after either the version string is returned OR the timeout period is reached. 
	\param[out] out_version_string The string buffer to write the version into
	\param max_version_string The size of the output buffer
	\param timeout_ms The conection timeout period in milliseconds, usually PSM_DEFAULT_TIMEOUT
	\return PSMResult_Success upon receiving result, PSMResult_Timeoout, or PSMResult_Error on request error.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetServiceVersionString(char *out_version_string, size_t max_version_string, int timeout_ms);

// System Async Queries
/** \brief Get the client API version string from PSMoveService
	Sends a request to PSMoveService to get the protocol version.
	This should be compared against the version returned from \ref PSM_GetClientVersionString() as a way to verify
	that the an outdated client isn't being used with PSMoveService.
	\remark Async - Starts a request for version string. Result obtained in one of two ways:
	  - Register callback for request id with \ref PSM_RegisterCallback and the poll with \ref PSM_Update()
	  - Poll with \ref PSM_UpdateNoPollMessages() and then call \ref PSM_PollNextMessage() to see if 
	  \ref PSMServiceVersion result has been received.
	\param[out] out_request_id The id of the request sent to PSMoveService. Can be used to register callback with \ref PSM_RegisterCallback.
	\return PSMResult_Success upon receiving result, PSMResult_Timeoout, or PSMResult_Error on request error.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetServiceVersionStringAsync(PSMRequestID *out_request_id);

// Async Message Handling API
/** \brief Retrieve the next message from the message queue.
	A call to \ref PSM_UpdateNoPollMessages will queue messages received from PSMoveService.
	Use this function to processes the queued event and response messages one by one.
	If a response message does not have a callback registered with \ref PSM_RegisterCallback it will get returned here.	
	\param[out] out_messaage The next \ref PSMMessage read from the incoming message queue.
	\param message_size The size of the message structure. Pass in sizeof(PSMMessage).
	\return PSMResult_Success or PSMResult_NoData if no more messages are available.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_PollNextMessage(PSMMessage *out_message, size_t message_size);

/** \brief Sends a private protocol request to PSMoveService.
	If the client has linked against the PSMoveProtocol.lib and defined the HAS_PROTOCOL_ACCESS symbol then you can 
	construct a private message declared in PSMoveProtocol.pb.h. These messages are defined in the Google protobuf 
	definition file, PSMoveProtocol.proto. Typically clients won't need or want to send private messages since many of
	the lower level protocol messages are intended for calibration and other service setting options used by the 
	PSMoveConfigTool.
	\param request_handle The pointer to the protocol message
	\param[out] out_request_id The id of the request sent to PSMoveService. Can be used to register callback with \ref PSM_RegisterCallback.
	\return PSMResult_RequestSent upon successfully sending request or PSMResult_Error if connection is invalid.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_SendOpaqueRequest(PSMRequestHandle request_handle, PSMRequestID *out_request_id);

/** \brief Registers an async request callback
	A \ref PSMRequestID is issued for every request sent. 
	This request_id can be assigned a \ref PSMResponseCallback.
	The callback will get called when a responsed is fetched by a call to \ref PSMUpdate or \ref PSMUpdateNoPollMessages.
	PSMoveConfigToolback can be cancelled with a call to \ref PSM_CancelCallback.
	\param request_id The id of a pending async request
	\param callback A callback function pointer
	\param callback_userdata Userdata for a callback function (often a "this" pointer to a class that issued the request).
	\return PSMResult_Success if the request_id is valid and the connection is active
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_RegisterCallback(PSMRequestID request_id, PSMResponseCallback callback, void *callback_userdata);

/** \brief Cancels a pending async request callback.
	This can be used to unregister a callback for a pending async request.
	If the response for the request is received later, it will simply get dropped on the floor.
	\param request_id The id of a pending request to cancel
	\return PSMResult_Success if the request_id is valid and the connection is active
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_CancelCallback(PSMRequestID request_id);

/** \brief Marks a request's response to be ignored.
	If \ref PSM_UpdateNoPollMessages is called, responses that don't have a callback registered will get added to the 
	message queue which can then be fetched by \ref PSM_PollNextMessage.
	If instead \ref PSM_Update is called then it expects all requests to have a registered callback.
	If no callback is found then a warning is issued and the message is dropped on the floor.
	Calling PSM_EatResponse explicly signals that we don't care about the response and will prevent the response from
	getting added to the message queue in the first place.
	\param request_id The id of a pending request whose response we want to ignore
	\return PSMResult_Success if the request_id is valid and the connection is active
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_EatResponse(PSMRequestID request_id);

// Controller Pool
/** \brief Fetches the \ref PSMController data for the given controller
	The client API maintains a pool of controller structs. 
	We can fetch a given controller by \ref PSMControllerID.
	DO NOT DELETE the controller pointer returned by this function.
	It is safe to copy this pointer on to other structures so long as the pointer is cleared once the client API is shutdown.
	\param controller_id The id of the controler structure to fetch
	\return A pointer to a \ref PSMController
 */
PSM_PUBLIC_FUNCTION(PSMController *) PSM_GetController(PSMControllerID controller_id);

/** \brief Allocate a reference to a controller.
	This function tells the client API to increment a reference count for a given controller.
	This function should be called before fetching the controller data using \ref PSM_GetController.
	When done with the controller, make sure to call \ref PSM_FreeControllerListener.
	\param controller_id The id of the controller we want to allocate a listener for
	\return PSMResult_Success if a valid controller id is given
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_AllocateControllerListener(PSMControllerID controller_id);

/** \brief Free a reference to a controller
	This function tells the client API to decrement a reference count for a given controller.
	\param controller_id The of of the controller we want to free the listener for.
	\return PSMResult_Success if a valid controller id is given that has a non-zero ref count
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_FreeControllerListener(PSMControllerID controller_id);

// Blocking Controller Methods
/** \brief Requests a list of the streamable controllers currently connected to PSMoveService.
	Sends a request to PSMoveService to get the list of currently streamable controllers.
	\remark Blocking - Returns after either the controller list is returned OR the timeout period is reached. 
	\param[out] out_controller_list The controller list to write the result into.
	\param timeout_ms The conection timeout period in milliseconds, usually PSM_DEFAULT_TIMEOUT	
	\return PSMResult_Success upon receiving result, PSMResult_Timeoout, or PSMResult_Error on request error.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetControllerList(PSMControllerList *out_controller_list, int timeout_ms);

/** \brief Requests start of an unreliable(udp) data stream for a given controller
	Asks PSMoveService to start stream data for the given controller with the given set of stream properties.
	The data in the associated \ref PSMController state will get updated automatically in calls to \ref PSM_Update or 
	\ref PSM_UpdateNoPollMessages.
	Requests to restart an already started stream will return an error.
	\remark Blocking - Returns after either stream start response comes back OR the timeout period is reached. 
	\param controller_id The id of the controller to start the stream for.
	\param data_stream_flags One or more of the following steam:
	    - PSMStreamFlags_defaultStreamOptions = minimal controller stream info
		- PSMStreamFlags_includePositionData = add position to pose data (which turns on tracking lights)
		- PSMStreamFlags_includePhysicsData = add linear and angular velocity and acceleration
		- PSMStreamFlags_includeRawSensorData = add raw IMU sensor data values
		- PSMStreamFlags_includeCalibratedSensorData = add calibrated sensor data values
		- PSMStreamFlags_includeRawTrackerData = add tracker projection info for each tacker
		- PSMStreamFlags_disableROI = turns off RegionOfInterest optimization used to reduce CPU load when finding tracking bulb
	\param timeout_ms The conection timeout period in milliseconds, usually PSM_DEFAULT_TIMEOUT
	\return PSMResult_Success upon receiving result, PSMResult_Timeoout, or PSMResult_Error on request error.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StartControllerDataStream(PSMControllerID controller_id, unsigned int data_stream_flags, int timeout_ms);

/** \brief Requests stop of an unreliable(udp) data stream for a given controller
	Asks PSMoveService to start stream data for the given controller with the given set of stream properties.
	The data in the associated \ref PSMController state will get updated automatically in calls to \ref PSM_Update or 
	\ref PSM_UpdateNoPollMessages.
	Requests to restart an already started stream will return an error.
	\remark Blocking - Returns after either stream start response comes back OR the timeout period is reached. 
	\param controller_id The id of the controller to start the stream for.
	\param timeout_ms The conection timeout period in milliseconds, usually PSM_DEFAULT_TIMEOUT
	\return PSMResult_Success upon receiving result, PSMResult_Timeoout, or PSMResult_Error on request error. */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StopControllerDataStream(PSMControllerID controller_id, int timeout_ms);

/** \brief Requests changing the tracking color type of a given controller.
	Sends a request to PSMoveService to change the tracking color of a controller.
	If another controller already is using the color being assigned to this controller, it will be assigned an available color.
	\remark Blocking - Returns after either the new color is set OR the timeout period is reached. 
	\param timeout_ms The conection timeout period in milliseconds, usually PSM_DEFAULT_TIMEOUT
	\return PSMResult_Success upon receiving result, PSMResult_Timeoout, or PSMResult_Error on request error.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_SetControllerLEDTrackingColor(PSMControllerID controller_id, PSMTrackingColorType tracking_color, int timeout_ms);

/** \brief Requests resetting the controllers current orientation.
	This request is used to reset any drift that has occured in the pose filter's orientation from the controllers true
	orientation. Resetting the controller orientation assumes that controller is currently being held in the "identity" orientation,
	which is typically pointing down the -Z axis. 
	This request is typically sent in reponse to a certain combonation of buttons being held (usually SELECT).
	\remark Blocking - Returns after either the version string is returned OR the timeout period is reached. 
	\param controller_id The ID of the whose orientation we want to reset
	\param q_pose The pose the controller is currently being held in relative to the identity pose (like straight up).
	\param timeout_ms The conection timeout period in milliseconds, usually PSM_DEFAULT_TIMEOUT
	\return PSMResult_Success upon receiving result, PSMResult_Timeoout, or PSMResult_Error on request error.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_ResetControllerOrientation(PSMControllerID controller_id, PSMQuatf *q_pose, int timeout_ms);

/** \brief Requests setting the selected tracker index for a controller
	This request is used to set the selected tracker index on a controller data stream
    when the data stream has tracking projection data active. The projection data is
    only provided for the selected tracker.
	\remark Blocking - Returns after either the result is returned OR the timeout period is reached. 
	\param controller_id The ID of the controller whose data stream we want to modify
    \param tracker_id The ID of the tracker we want to assign as the active tracker
	\param timeout_ms The request timeout period in milliseconds, usually PSM_DEFAULT_TIMEOUT
	\return PSMResult_RequestSent on success or PSMResult_Error if there was no valid connection
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_SetControllerDataStreamTrackerIndex(PSMControllerID controller_id, PSMTrackerID tracker_id, int timeout_ms);

/** \brief Requests setting the hand assigned to a controller
	This request is used to set the suggested hand for a controller.
	Hand information is used by external APIs and not by PSMoveService.
	No restrictions are made about which hands are assigned to a given controller.
	\remark Blocking - Returns after either the result is returned OR the timeout period is reached. 
	\param controller_id The ID of the controller whose data stream we want to modify
    \param hand The hand to assign to a controller (Any, Left or Right)
	\param timeout_ms The request timeout period in milliseconds, usually PSM_DEFAULT_TIMEOUT
	\return PSMResult_RequestSent on success or PSMResult_Error if there was no valid connection
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_SetControllerHand(PSMControllerID controller_id, PSMControllerHand hand, int timeout_ms);

// Controller State Methods
/** \brief Get the current orientation of a controller
	\param controller_id The id of the controller
	\param[out] out_orientation The orientation of the controller
	\return PSMResult_Success if controller has a valid orientation
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetControllerOrientation(PSMControllerID controller_id, PSMQuatf *out_orientation);

/** \brief Get the current position of a controller
	\param controller_id The id of the controller
	\param[out] out_position The position of the controller
	\return PSMResult_Success if controller has a valid position
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetControllerPosition(PSMControllerID controller_id, PSMVector3f *out_position);

/** \brief Get the current pose (orienation and position) of a controller
	\param controller_id The id of the controller
	\param[out] out_pose The pose of the controller
	\return PSMResult_Success if controller has a valid pose
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetControllerPose(PSMControllerID controller_id, PSMPosef *out_pose);

/** \brief Get the current rumble fraction of a controller
	\param controller_id The id of the controller
	\param channel The channel to get the rumble for. The PSMove has one channel. The DualShock4 has two.
	\param[out] out_rumble_fraction The 0.0-1.0 fraction of rumble the controller is currently set to
	\return PSMResult_Success if controller has a valid rumble state.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetControllerRumble(PSMControllerID controller_id, PSMControllerRumbleChannel channel, float *out_rumble_fraction);

/** \brief Helper used to tell if the controller is upright on a level surface.
	This method is used as a calibration helper when you want to get a number of controller samples. 
	Often in this instance you want to make sure the controller is sitting upright on a table.
	\param controller_id The id of the controller
	\param[out] out_is_stable True if the controller is stable and upright.
	\return PSMResult_Success if controller can be tested for stability.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetIsControllerStable(PSMControllerID controller_id, bool *out_is_stable);

/** \brief See if the controller is currently being tracked by at least one tracking camera.
	\param controller_id The id of the controller
	\param[out] out_is_tracking True if the controller is currently tracking
	\return PSMResult_Success if controller can be tracked at all.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetIsControllerTracking(PSMControllerID controller_id, bool *out_is_tracking);

/** \brief Helper function for getting the tracking centroid for a controller on a given tracker
	Each tracking camera sees a projection of a controllers tracking light. 
	This method gets the pixel centroid of controller projection.
	\param controller_id The controller id to get the tracking projection location
	\param tracker_id The tracker id of the tracker that has the controller projection we care about.
	\param[out] out_tracker_id The id of the tracker this projection is for
	\param[out] out_location The center pixel location of the controller projection on the tracker.
	\return PSMResult_Success if controller has a valid projection on the tracker.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetControllerPixelLocationOnTracker(PSMControllerID controller_id, PSMTrackerID *out_tracker_id, PSMVector2f *out_location);

/** \brief Helper function for getting the tracker relative 3d position of the controller.
	Each tracking camera can compute a estimate of the controllers 3d position relative to the tracker.
	This method gets the 3d centroid of the tracking light in tracker relative coordinates.
	\param controller_id The controller id to get the tracking position for.
	\param[out] out_tracker_id The id of the tracker this projection is for
	\param[out] out_position Tracker relative centroid position of controller in cm.
	\return PSMResult_Success if controller has a valid position relative to the tracker.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetControllerPositionOnTracker(PSMControllerID controller_id, PSMTrackerID *out_tracker_id, PSMVector3f *outPosition);

/** \brief Helper function for getting the tracker relative 3d orientation of the controller.
	Each tracking camera can compute a estimate of the controllers 3d position relative to the tracker.
	This method gets the 3d centroid of the tracking light in tracker relative coordinates.
	\param controller_id The controller id to get the tracking position for
	\param[out] out_tracker_id The id of the tracker this projection is for
	\param[out] out_orientation Tracker relative centroid orientation of controller.
	\return PSMResult_Success if controller has a valid optical orientation relative to the tracker (PSMove can't, DS4 can).
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetControllerOrientationOnTracker(PSMControllerID controller_id, PSMTrackerID *out_tracker_id, PSMQuatf *outOrientation);

/** \brief Helper function for getting the tracker relative projection of a controller
	Each tracking camera can have a projection of the controller.
	This method gets the pixel geomtry of that projection.
	For a PSMoveController this is an ellipse.
	For a DualShock4 this is a quad.
	\param controller_id The controller id to get the tracking projection for.
	\param[out] out_tracker_id The id of the tracker this projection is for
	\param[out] out_projection The tracking projection shape of the controller.
	\return PSMResult_Success if controller has a valid projection on the tracker.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetControllerProjectionOnTracker(PSMControllerID controller_id, PSMTrackerID *out_tracker_id, PSMTrackingProjection *out_projection);

/** \brief Sets a temporary RGB override for the controller's light
	The light color override will be sent on the outbound controller UDP stream.
	Tracking will not run when a light color override is set.
	The override can be cleared by setting the override color to (0, 0, 0)
	\param controller_id The controller whose light override value we want to set.
	\param r The red color override, range [0. 255]
	\param g The green color override, range [0. 255]
	\param b The blue color override, range [0. 255]
	\return PSMResult_Success if the controller can have a light color override set
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_SetControllerLEDOverrideColor(PSMControllerID controller_id, unsigned char r, unsigned char g, unsigned char b);

/** \brief Sets the controller rumble fraction
	The controller rumble is set on the outbound UDP stream.
	The rumble for the controller stays on this setting until you change it.
	\param controller_id The id of the controller to set the rumble for
	\param channel The channel for the rumble (PSMove has one channel, DS4 has two channels)
	\param rumble_fraction A rumble value in the range 0.0 to 1.0
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_SetControllerRumble(PSMControllerID controller_id, PSMControllerRumbleChannel channel, float rumble_fraction);

// Async Controller Methods
/** \brief Sends a request for the controller list.
	\remark Async - Starts a request for the controller list. Result obtained in one of two ways:
	  - Register callback for request id with \ref PSM_RegisterCallback and the poll with \ref PSM_Update()
	  - Poll with \ref PSM_UpdateNoPollMessages() and then call \ref PSM_PollNextMessage() to see if 
	  \ref PSMControllerList result has been received.
	\param[out] out_request_id The id of the request sent to PSMoveService. Can be used to register callback with \ref PSM_RegisterCallback.
	\return PSMResult_RequestSent on success or PSMResult_Error if there was no valid connection
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetControllerListAsync(PSMRequestID *out_request_id);

/** \brief Requests start of an unreliable(udp) data stream for a given controller
	Asks PSMoveService to start stream data for the given controller with the given set of stream properties.
	The data in the associated \ref PSMController state will get updated automatically in calls to \ref PSM_Update or 
	\ref PSM_UpdateNoPollMessages.
	Requests to restart an already started stream will return an error.
	\remark Async - Starts a request for version string. Result obtained in one of two ways:
	  - Register callback for request id with \ref PSM_RegisterCallback and the poll with \ref PSM_Update()
	  - Poll with \ref PSM_UpdateNoPollMessages() and then call \ref PSM_PollNextMessage() to see if 
	  generic \ref PSMResponseMessage result has been received.
	\param controller_id The controller id we wish to start the stream for
	\param data_stream_flags One or more of the following steam:
	    - PSMStreamFlags_defaultStreamOptions = minimal controller stream info
		- PSMStreamFlags_includePositionData = add position to pose data (which turns on tracking lights)
		- PSMStreamFlags_includePhysicsData = add linear and angular velocity and acceleration
		- PSMStreamFlags_includeRawSensorData = add raw IMU sensor data values
		- PSMStreamFlags_includeCalibratedSensorData = add calibrated sensor data values
		- PSMStreamFlags_includeRawTrackerData = add tracker projection info for each tacker
		- PSMStreamFlags_disableROI = turns off RegionOfInterest optimization used to reduce CPU load when finding tracking bulb
	\param[out] out_request_id The id of the request sent to PSMoveService. Can be used to register callback with \ref PSM_RegisterCallback.
	\return PSMResult_RequestSent on success or PSMResult_Error if there was no valid connection
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StartControllerDataStreamAsync(PSMControllerID controller_id, unsigned int data_stream_flags, PSMRequestID *out_request_id);

/** \brief Requests stop of an unreliable(udp) data stream for a given controller
	Asks PSMoveService to stop stream data for the given controller.
	\remark Async - Starts a request for version string. Result obtained in one of two ways:
	  - Register callback for request id with \ref PSM_RegisterCallback and the poll with \ref PSM_Update()
	  - Poll with \ref PSM_UpdateNoPollMessages() and then call \ref PSM_PollNextMessage() to see if 
	  generic \ref PSMResponseMessage result has been received.
	\param controller_id The controller id we wish to start the stream for
	\param[out] out_request_id The id of the request sent to PSMoveService. Can be used to register callback with \ref PSM_RegisterCallback.
	\return PSMResult_RequestSent on success or PSMResult_Error if there was no valid connection
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StopControllerDataStreamAsync(PSMControllerID controller_id, PSMRequestID *out_request_id);

/** \brief Requests changing the tracking color type of a given controller.
	Sends a request to PSMoveService to change the tracking color of a controller.
	that the an outdated client isn't being used with PSMoveService.
	If another controller already is using the color being assigned to this controller, it will be assigned an available color.
	\remark Async - Starts a request for version string. Result obtained in one of two ways:
	  - Register callback for request id with \ref PSM_RegisterCallback and the poll with \ref PSM_Update()
	  - Poll with \ref PSM_UpdateNoPollMessages() and then call \ref PSM_PollNextMessage() to see if 
	  generic \ref PSMMessage result has been received.
	\param controller_id The ID of the whose orientation we want to reset
	\param tracking_color The new tracking color for the controller
	\param[out] out_request_id The id of the request sent to PSMoveService. Can be used to register callback with \ref PSM_RegisterCallback.
	\return PSMResult_RequestSent on success or PSMResult_Error if there was no valid connection
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_SetControllerLEDColorAsync(PSMControllerID controller_id, PSMTrackingColorType tracking_color, PSMRequestID *out_request_id);

/** \brief Requests resetting the controllers current orientation.
	This request is used to reset any drift that has occured in the pose filter's orientation from the controllers true
	orientation. Resetting the controller orientation assumes that controller is currently being held in the "identity" orientation,
	which is typically pointing down the -Z axis. 
	This request is typically sent in reponse to a certain combonation of buttons being held (usually SELECT).
	\remark Async - Starts a request for orientation reset. Result obtained in one of two ways:
	  - Register callback for request id with \ref PSM_RegisterCallback and the poll with \ref PSM_Update()
	  - Poll with \ref PSM_UpdateNoPollMessages() and then call \ref PSM_PollNextMessage() to see if 
	  generic \ref PSMMessage result has been received.
	\param controller_id The ID of the whose orientation we want to reset
	\param q_pose The pose the controller is currently being held in relative to the identity pose (like straight up).
	\param[out] out_request_id The id of the request sent to PSMoveService. Can be used to register callback with \ref PSM_RegisterCallback.
	\return PSMResult_RequestSent on success or PSMResult_Error if there was no valid connection
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_ResetControllerOrientationAsync(PSMControllerID controller_id, const PSMQuatf *q_pose, PSMRequestID *out_request_id);

/** \brief Requests setting the selected tracker index for a controller
	This request is used to set the selected tracker index on a controller data stream
    when the data stream has tracking projection data active. The projection data is
    only provided for the selected tracker.
	\remark Async - Starts an async request. Result obtained in one of two ways:
	  - Register callback for request id with \ref PSM_RegisterCallback and the poll with \ref PSM_Update()
	  - Poll with \ref PSM_UpdateNoPollMessages() and then call \ref PSM_PollNextMessage() to see if 
	  generic \ref PSMMessage result has been received.
	\param controller_id The ID of the controller whose data stream we want to modify
    \param tracker_id The ID of the tracker we want to assign as the active tracker
	\param[out] out_request_id The id of the request sent to PSMoveService. Can be used to register callback with \ref PSM_RegisterCallback.
	\return PSMResult_RequestSent on success or PSMResult_Error if there was no valid connection
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_SetControllerDataStreamTrackerIndexAsync(PSMControllerID controller_id, PSMTrackerID tracker_id, PSMRequestID *out_request_id);

/** \brief Requests setting the assigned hand for a controller
	This request is used to set the suggested hand for a controller.
	Hand information is used by external APIs and not by PSMoveService.
	No restrictions are made about which hands are assigned to a given controller.
	\remark Async - Starts an async request. Result obtained in one of two ways:
	  - Register callback for request id with \ref PSM_RegisterCallback and the poll with \ref PSM_Update()
	  - Poll with \ref PSM_UpdateNoPollMessages() and then call \ref PSM_PollNextMessage() to see if 
	  generic \ref PSMMessage result has been received.
	\param controller_id The ID of the controller whose data stream we want to modify
    \param hand The hand to assign to a controller (Any, Left or Right)
	\param[out] out_request_id The id of the request sent to PSMoveService. Can be used to register callback with \ref PSM_RegisterCallback.
	\return PSMResult_RequestSent on success or PSMResult_Error if there was no valid connection
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_SetControllerHandAsync(PSMControllerID controller_id, PSMControllerHand hand, PSMRequestID *out_request_id);

// Tracker Pool
/** \brief Fetches the \ref PSMTracker data for the given tracker
	The client API maintains a pool of tracker structs. 
	We can fetch a given tracker by \ref PSMTrackerID.
	DO NOT DELETE the tracker pointer returned by this function.
	It is safe to copy this pointer on to other structures so long as the pointer is cleared once the client API is shutdown.
	\param tracker_id The id of the tracker structure to fetch
	\return A pointer to a \ref PSMTracker
 */
PSM_PUBLIC_FUNCTION(PSMTracker *) PSM_GetTracker(PSMTrackerID tracker_id);

/** \brief Allocate a reference to a tracker.
	This function tells the client API to increment a reference count for a given tracker.
	This function should be called before fetching the tracker data using \ref PSM_GetTracker.
	When done with the tracker, make sure to call \ref PSM_FreeTrackerListener.
	\param tracker_id The id of the tracker we want to allocate a listener for
	\return PSMResult_Success if a valid tracker id is given
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_AllocateTrackerListener(PSMTrackerID tracker_id, const PSMClientTrackerInfo *tracker_info);

/** \brief Free a reference to a tracker
	This function tells the client API to decrement a reference count for a given tracker.
	\param tracker_id The of of the tracker we want to free the listener for.
	\return PSMResult_Success if a valid tracker id is given that has a non-zero ref count
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_FreeTrackerListener(PSMTrackerID controller_id);

// Tracker State Methods
/** \brief Extract the intrinsic matrix
	The intrinsic matrix is used to convert a camera relative 3d position to a pixel projection.
	It encodes the focal length and the center offset of the cameras lens.
	\param tracker_id The id of the tracker
	\param[out] out_matrix the 3x3 colomn major camera intrinsic matrix
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackerIntrinsicMatrix(PSMTrackerID tracker_id, PSMMatrix3f *out_matrix);

// Blocking Tracker Methods
/** \brief Requests a list of the trackers currently connected to PSMoveService.
	Sends a request to PSMoveService to get the list of trackers.
	\remark Blocking - Returns after either the tracker list is returned OR the timeout period is reached. 
	\param[out] out_tracker_list The tracker list to write the result into.
	\param timeout_ms The conection timeout period in milliseconds, usually PSM_DEFAULT_TIMEOUT	
	\return A zero-terminated version string of the format "Product.Major-Phase Minor.Release.Hotfix", ex: "0.9-alpha 8.1.0"
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackerList(PSMTrackerList *out_tracker_list, int timeout_ms);

/** \brief Requests start of a shared memory video stream for a given tracker
	Asks PSMoveService to start a video stream for the given tracker.
	PSMoveService will then start writing the video stream to a shared memory buffer.
	The data in the associated \ref PSMTracker state will get updated automatically in calls to \ref PSM_Update or 
	\ref PSM_UpdateNoPollMessages.
	Requests to restart an already started stream will return an error.
	\remark Video streams can only be started on clients that run on the same machine as PSMoveService is running on.
	\remark Blocking - Returns after either stream start response comes back OR the timeout period is reached. 
	\param tracker_id The id of the tracker to start the stream for.
	\param timeout_ms The conection timeout period in milliseconds, usually PSM_DEFAULT_TIMEOUT
	\return PSMResult_Success upon receiving result, PSMResult_Timeoout, or PSMResult_Error on request error.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StartTrackerDataStream(PSMTrackerID tracker_id, int timeout_ms);

/** \brief Requests stop of a shared memory video stream for a given tracker
	Asks PSMoveService to stop an active video stream for the given tracker.
	\remark Video streams can only be started on clients that run on the same machine as PSMoveService is running on.
	\remark Blocking - Returns after either stream start response comes back OR the timeout period is reached. 
	\param tracker_id The id of the tracker to start the stream for.
	\param timeout_ms The conection timeout period in milliseconds, usually PSM_DEFAULT_TIMEOUT
	\return PSMResult_Success upon receiving result, PSMResult_Timeoout, or PSMResult_Error on request error.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StopTrackerDataStream(PSMTrackerID tracker_id, int timeout_ms);

/** \brief Request the tracking space settings
	Sends a request to PSMoveService to get the tracking space settings for PSMoveService.
	The settings contain the direction of global forward (usually the -Z axis)
	\remark Blocking - Returns after either the tracking space settings are returned OR the timeout period is reached. 
	\param out_tracking_space The \ref PSMTrackingSpace settings for PSMoveService
	\param timeout_ms The conection timeout period in milliseconds, usually PSM_DEFAULT_TIMEOUT
	\return PSMResult_Success upon receiving result, PSMResult_Timeoout, or PSMResult_Error on request error.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackingSpaceSettings(PSMTrackingSpace *out_tracking_space, int timeout_ms);

/** \brief Opens the tracker video stream buffer on the client.
	Starts reading tracker video stream from a shared memory buffer.
	A call to \ref PSM_StartTrackerDataStream must be done first to open the video stream on PSMoveServices end.
	\param tracker_id The id of the tracker we wish to open the video stream for.
	\return PSMResult_Success if the shared memory buffer was activated by PSMoveService.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_OpenTrackerVideoStream(PSMTrackerID tracker_id);

/** \brief Poll the next video frame from an opened tracker video stream
	Copy the next video frame from the shared memory buffer.
	This should be called at least as fast as the frame rate of the video feed.
	\return PSMResult_Success if there is data available in the shared memory buffer.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_PollTrackerVideoStream(PSMTrackerID tracker_id);

/** \brief Closes the tracker video stream buffer on the client.
	Stops reading tracker video stream from a shared memory buffer.
	A call to \ref PSM_StopTrackerDataStream must be done after closing the video stream.
	\param tracker_id The id of the tracker we wish to close the video stream for.
	\return PSMResult_Success if the shared memory buffer was active.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_CloseTrackerVideoStream(PSMTrackerID tracker_id);

/** \brief Fetch the next video frame buffer from an opened tracker video stream
	\remark Make sure the video buffer is large enough to hold tracker dimension x 3 bytes.
	\param tracker_id The tracker to poll the next video frame from
	\param[out] out_buffer A pointer to the buffer to copy the video frame into
	\return PSMResult_Success if there was frame data available to read
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackerVideoFrameBuffer(PSMTrackerID tracker_id, const unsigned char **out_buffer); 

/** \brief Helper function to fetch tracking frustum properties from a tracker
	\param The id of the tracker we wish to get the tracking frustum properties for
	\param out_frustum The tracking frustum properties to write the result into
	\return PSMResult_Success if the tracker state is valid
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackerFrustum(PSMTrackerID tracker_id, PSMFrustum *out_frustum);

// Async Tracker Methods
/** \brief Sends a request for the tracker list.
	\remark Async - Starts a request for the tracker list. Result obtained in one of two ways:
	  - Register callback for request id with \ref PSM_RegisterCallback and the poll with \ref PSM_Update()
	  - Poll with \ref PSM_UpdateNoPollMessages() and then call \ref PSM_PollNextMessage() to see if 
	  \ref PSMTrackerList result has been received.
	\param[out] out_request_id The id of the request sent to PSMoveService. Can be used to register callback with \ref PSM_RegisterCallback.
	\return PSMResult_RequestSent on success or PSMResult_Error if there was no valid connection
 */
 PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackerListAsync(PSMRequestID *out_request_id);

/** \brief Requests start shared memory video stream for a given tracker
	Asks PSMoveService to start video data for the given tracker.
	The data in the associated \ref PSMTracker state will get updated automatically in calls to \ref PSM_Update or 
	\ref PSM_UpdateNoPollMessages.
	Requests to restart an already started stream will return an error.
	\remark Video streams can only be started on clients that run on the same machine as PSMoveService is running on.
	\remark Async - Result obtained in one of two ways:
	  - Register callback for request id with \ref PSM_RegisterCallback and the poll with \ref PSM_Update()
	  - Poll with \ref PSM_UpdateNoPollMessages() and then call \ref PSM_PollNextMessage() to see if 
	  generic \ref PSMResponseMessage result has been received.
	\param tracker_id The tracker id we wish to start the stream for
	\param[out] out_request_id The id of the request sent to PSMoveService. Can be used to register callback with \ref PSM_RegisterCallback.
	\return PSMResult_RequestSent on success or PSMResult_Error if there was no valid connection
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StartTrackerDataStreamAsync(PSMTrackerID tracker_id, PSMRequestID *out_request_id);

/** \brief Requests stop shared memory video stream for a given tracker
	Asks PSMoveService to stop video data for the given tracker.
	\remark Async - Result obtained in one of two ways:
	  - Register callback for request id with \ref PSM_RegisterCallback and the poll with \ref PSM_Update()
	  - Poll with \ref PSM_UpdateNoPollMessages() and then call \ref PSM_PollNextMessage() to see if 
	  generic \ref PSMResponseMessage result has been received.
	\param tracker_id The tracker id we wish to stop the stream for
	\param[out] out_request_id The id of the request sent to PSMoveService. Can be used to register callback with \ref PSM_RegisterCallback.
	\return PSMResult_RequestSent on success or PSMResult_Error if there was no valid connection
 */
 PSM_PUBLIC_FUNCTION(PSMResult) PSM_StopTrackerDataStreamAsync(PSMTrackerID tracker_id, PSMRequestID *out_request_id);

/** \brief Request the tracking space settings
	Sends a request to PSMoveService to get the tracking space settings for PSMoveService.
	The settings contain the direction of global forward (usually the -Z axis)
	\remark Async - Starts a request for tracking space settings. Result obtained in one of two ways:
	  - Register callback for request id with \ref PSM_RegisterCallback and the poll with \ref PSM_Update()
	  - Poll with \ref PSM_UpdateNoPollMessages() and then call \ref PSM_PollNextMessage() to see if 
	  \ref PSMTrackingSpace has been received.
	\param[out] out_request_id The id of the request sent to PSMoveService. Can be used to register callback with \ref PSM_RegisterCallback.
	\return PSMResult_RequestSent on success or PSMResult_Error if there was no valid connection
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackingSpaceSettingsAsync(PSMRequestID *out_request_id);

// HMD Pool
/** \brief Fetches the \ref PSMHeadMountedDisplay data for the given HMD
	The client API maintains a pool of HMD structs. 
	We can fetch a given HMD by \ref PSMHmdID
	DO NOT DELETE the HMD pointer returned by this function.
	It is safe to copy this pointer on to other structures so long as the pointer is cleared once the client API is shutdown.
	\param hmd_id The id of the hmd structure to fetch
	\return A pointer to a \ref PSMHeadMountedDisplay
 */
PSM_PUBLIC_FUNCTION(PSMHeadMountedDisplay *) PSM_GetHmd(PSMHmdID hmd_id);

/** \brief Allocate a reference to an HMD.
	This function tells the client API to increment a reference count for a given HMD.
	This function should be called before fetching the hmd data using \ref PSM_GetHmd.
	When done with the HMD, make sure to call \ref PSM_FreeHmdListener.
	\param hmd_id The id of the HMD we want to allocate a listener for
	\return PSMResult_Success if a valid controller id is given
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_AllocateHmdListener(PSMHmdID hmd_id);

/** \brief Free a reference to an HMD
	This function tells the client API to decrement a reference count for a given HMD.
	\param hmd_id The id of the HMD we want to free the listener for.
	\return PSMResult_Success if a valid HMD id is given that has a non-zero ref count
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_FreeHmdListener(PSMHmdID hmd_id);

// HMD State Methods
/** \brief Get the current orientation of an HMD
	\param hmd_id The id of the HMD
	\param[out] out_orientation The orientation of the HMD
	\return PSMResult_Success if HMD has a valid orientation
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdOrientation(PSMHmdID hmd_id, PSMQuatf *out_orientation);

/** \brief Get the current position of an HMD
	\param hmd_id The id of the HMD
	\param[out] out_position The position of the HMD
	\return PSMResult_Success if HMD has a valid position
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdPosition(PSMHmdID hmd_id, PSMVector3f *out_position);

/** \brief Get the current pose (orienation and position) of an HMD
	\param hmd_id The id of the HMD
	\param[out] out_pose The pose of the HMD
	\return PSMResult_Success if HMD has a valid pose
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdPose(PSMHmdID hmd_id, PSMPosef *out_pose);

/** \brief Helper used to tell if the HMD is upright on a level surface.
	This method is used as a calibration helper when you want to get a number of HMD samples. 
	Often in this instance you want to make sure the HMD is sitting upright on a table.
	\param hmd_id The id of the HMD
	\param[out] out_is_stable True if the HMD is stable and upright.
	\return PSMResult_Success if HMD can be tested for stability.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetIsHmdStable(PSMHmdID hmd_id, bool *out_is_stable);

/** \brief See if the HMD is currently being tracked by at least one tracking camera.
	\param hmd_id The id of the HMD
	\param[out] out_is_tracking True if the hmd is currently tracking
	\return PSMResult_Success if hmd can be tracked at all.
 */ 
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetIsHmdTracking(PSMHmdID hmd_id, bool *out_is_tracking);

/** \brief Helper function for getting the tracking centroid for an HMD on a given tracker
	Each tracking camera sees a projection of an HMD's tracking light(s). 
	This method gets the pixel centroid of the HMD projection.
	\param hmd_id The hmd id to get the tracking projection location
	\param[out] out_tracker_id The id of the tracker this projection is for
	\param[out] out_location The center pixel location of the HMD projection on the tracker.
	\return PSMResult_Success if HMD has a valid projection on the tracker.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdPixelLocationOnTracker(PSMHmdID hmd_id, PSMTrackerID *out_tracker_id, PSMVector2f *out_location);

/** \brief Helper function for getting the tracker relative 3d position of the HMD.
	Each tracking camera can compute a estimate of the HMD's 3d position relative to the tracker.
	This method gets the 3d centroid of the tracking light(s) in tracker relative coordinates.
	\param hmd_id The hmd id to get the tracking position for
	\param[out] out_tracker_id The id of the tracker this projection is for
	\param[out] out_position Tracker relative centroid position of the HMD in cm.
	\return PSMResult_Success if the HMD has a valid position relative to the tracker.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdPositionOnTracker(PSMHmdID hmd_id, PSMTrackerID *out_tracker_id, PSMVector3f *out_position);

/** \brief Helper function for getting the tracker relative 3d orientation of the HMD
	Each tracking camera can compute a estimate of an HMDs 3d position relative to the tracker.
	This method gets the 3d centroid of the tracking light in tracker relative coordinates.
	\param hmd_id The HMD id to get the tracking position for
	\param[out] out_tracker_id The id of the tracker this projection is for
	\param[out] out_orientation Tracker relative centroid orientation of the HMD.
	\return PSMResult_Success if HMD has a valid optical orientation relative to the tracker.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdOrientationOnTracker(PSMHmdID hmd_id, PSMTrackerID *out_tracker_id, PSMQuatf *out_orientation);

/** \brief Helper function for getting the tracker relative projection of an HMD
	Each tracking camera can have a projection of the HMD.
	This method gets the pixel geometry of that projection.
	\param hmd_id The hmd id to get the tracking projection for
	\param[out] out_tracker_id The id of the tracker this projection is for
	\param[out] out_projection The tracking projection shape of the HMD.
	\return PSMResult_Success if HMD has a valid projection on the tracker.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdProjectionOnTracker(PSMHmdID hmd_id, PSMTrackerID *out_tracker_id, PSMTrackingProjection *out_projection);

// Blocking HMD Methods
/** \brief Requests a list of the HMDs currently connected to PSMoveService.
	Sends a request to PSMoveService to get the list of HMDs.
	\remark Blocking - Returns after either the HMD list is returned OR the timeout period is reached. 
	\param[out] out_hmd_list The hmd list to write the result into.
	\param timeout_ms The conection timeout period in milliseconds, usually PSM_DEFAULT_TIMEOUT	
	\return PSMResult_Success upon receiving result, PSMResult_Timeoout, or PSMResult_Error on request error.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdList(PSMHmdList *out_hmd_list, int timeout_ms);

/** \brief Requests start of an unreliable(udp) data stream for a given HMD
	Asks PSMoveService to start stream data for the given HMD with the given set of stream properties.
	The data in the associated \ref PSMHeadMountedDisplay state will get updated automatically in calls to \ref PSM_Update or 
	\ref PSM_UpdateNoPollMessages.
	Requests to restart an already started stream will return an error.
	\remark Blocking - Returns after either stream start response comes back OR the timeout period is reached. 
	\param hmd_id The id of the head mounted display to start the stream for.
	\param data_stream_flags One or more of the following steam:
	    - PSMStreamFlags_defaultStreamOptions = minimal HMD stream info
		- PSMStreamFlags_includePositionData = add position to pose data (which turns on tracking lights)
		- PSMStreamFlags_includePhysicsData = add linear and angular velocity and acceleration
		- PSMStreamFlags_includeRawSensorData = add raw IMU sensor data values
		- PSMStreamFlags_includeCalibratedSensorData = add calibrated sensor data values
		- PSMStreamFlags_includeRawTrackerData = add tracker projection info for each tacker
		- PSMStreamFlags_disableROI = turns off RegionOfInterest optimization used to reduce CPU load when finding tracking bulb(s)
	\param timeout_ms The conection timeout period in milliseconds, usually PSM_DEFAULT_TIMEOUT
	\return PSMResult_Success upon receiving result, PSMResult_Timeoout, or PSMResult_Error on request error.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StartHmdDataStream(PSMHmdID hmd_id, unsigned int data_stream_flags, int timeout_ms);

/** \brief Requests stop of an unreliable(udp) data stream for a given HMD
	Asks PSMoveService to stop stream data for the given HMD.
	\remark Blocking - Returns after either stream stop response comes back OR the timeout period is reached. 
	\param hmd_id The id of the HMD to start the stream for.
	\param timeout_ms The conection timeout period in milliseconds, usually PSM_DEFAULT_TIMEOUT
	\return PSMResult_Success upon receiving result, PSMResult_Timeoout, or PSMResult_Error on request error. 
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StopHmdDataStream(PSMHmdID hmd_id, int timeout_ms);

/** \brief Requests setting the selected tracker index for an HMD
	This request is used to set the selected tracker index on an HMD data stream
    when the data stream has tracking projection data active. The projection data is
    only provided for the selected tracker.
	\remark Blocking - Returns after either the result is returned OR the timeout period is reached. 
	\param hmd_id The ID of the HMD whose data stream we want to modify
    \param tracker_id The ID of the tracker we want to assign as the active tracker
	\param timeout_ms The request timeout period in milliseconds, usually PSM_DEFAULT_TIMEOUT
	\return PSMResult_RequestSent on success or PSMResult_Error if there was no valid connection
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_SetHmdDataStreamTrackerIndex(PSMHmdID hmd_id, PSMTrackerID tracker_id, int timeout_ms);

// Async HMD Methods
/** \brief Requests a list of the HMDs currently connected to PSMoveService.
	Sends a request to PSMoveService to get the list of HMDs.
	\remark Async - Starts a request for HMD Result obtained in one of two ways:
	  - Register callback for request id with \ref PSM_RegisterCallback and the poll with \ref PSM_Update()
	  - Poll with \ref PSM_UpdateNoPollMessages() and then call \ref PSM_PollNextMessage() to see if 
	  \ref PSMHmdList result has been received.
	\param[out] out_request_id The id of the request sent to PSMoveService. Can be used to register callback with \ref PSM_RegisterCallback.
	\return PSMResult_RequestSent if request successfully sent or PSMResult_Error if connection is invalid.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdListAsync(PSMRequestID *out_request_id);

/** \brief Requests start of an unreliable(udp) data stream for a given HMD
	Asks PSMoveService to start stream data for the given HMD with the given set of stream properties.
	The data in the associated \ref PSMHeadMountedDisplay state will get updated automatically in calls to \ref PSM_Update or 
	\ref PSM_UpdateNoPollMessages.
	Requests to restart an already started stream will return an error.
	\remark Async - Sends a request for HMD stream start. Result obtained in one of two ways:
	  - Register callback for request id with \ref PSM_RegisterCallback and the poll with \ref PSM_Update()
	  - Poll with \ref PSM_UpdateNoPollMessages() and then call \ref PSM_PollNextMessage() to see if 
	  generic \ref PSMMessage result has been received.
	\param hmd_id The id of the HMD to start the stream for.
	\param data_stream_flags One or more of the following steam:
	    - PSMStreamFlags_defaultStreamOptions = minimal HMD stream info
		- PSMStreamFlags_includePositionData = add position to pose data (which turns on tracking lights)
		- PSMStreamFlags_includePhysicsData = add linear and angular velocity and acceleration
		- PSMStreamFlags_includeRawSensorData = add raw IMU sensor data values
		- PSMStreamFlags_includeCalibratedSensorData = add calibrated sensor data values
		- PSMStreamFlags_includeRawTrackerData = add tracker projection info for each tacker
		- PSMStreamFlags_disableROI = turns off RegionOfInterest optimization used to reduce CPU load when finding tracking bulb(s)
	\param[out] out_request_id The id of the request sent to PSMoveService. Can be used to register callback with \ref PSM_RegisterCallback.
	\return PSMResult_RequestSent if request successfully sent or PSMResult_Error if connection is invalid.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StartHmdDataStreamAsync(PSMHmdID hmd_id, unsigned int data_stream_flags, PSMRequestID *out_request_id);

/** \brief Requests stop of an unreliable(udp) data stream for a given HMD
	Asks PSMoveService to stop stream data for the given HMD.
	\remark Async - Sends a request for HMD stream stop. Result obtained in one of two ways:
	  - Register callback for request id with \ref PSM_RegisterCallback and the poll with \ref PSM_Update()
	  - Poll with \ref PSM_UpdateNoPollMessages() and then call \ref PSM_PollNextMessage() to see if 
	  generic \ref PSMMessage result has been received.
	\param hmd_id The id of the HMD to start the stream for.
	\param[out] out_request_id The id of the request sent to PSMoveService. Can be used to register callback with \ref PSM_RegisterCallback.
	\return PSMResult_RequestSent if request successfully sent or PSMResult_Error if connection is invalid.
 */
 PSM_PUBLIC_FUNCTION(PSMResult) PSM_StopHmdDataStreamAsync(PSMHmdID hmd_id, PSMRequestID *out_request_id);

 /** \brief Requests setting the selected tracker index for an HMD
	This request is used to set the selected tracker index on an HMD data stream
    when the data stream has tracking projection data active. The projection data is
    only provided for the selected tracker.
	\remark Async - Starts an async request. Result obtained in one of two ways:
	  - Register callback for request id with \ref PSM_RegisterCallback and the poll with \ref PSM_Update()
	  - Poll with \ref PSM_UpdateNoPollMessages() and then call \ref PSM_PollNextMessage() to see if 
	  generic \ref PSMMessage result has been received.
	\param hmd_id The ID of the hmd whose data stream we want to modify
    \param tracker_id The ID of the tracker we want to assign as the active tracker
	\param[out] out_request_id The id of the request sent to PSMoveService. Can be used to register callback with \ref PSM_RegisterCallback.
	\return PSMResult_RequestSent on success or PSMResult_Error if there was no valid connection
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_SetHmdDataStreamTrackerIndexAsync(PSMHmdID hmd_id, PSMTrackerID tracker_id, PSMRequestID *out_request_id);

/** 
@} 
*/ 

//cut_after
#endif

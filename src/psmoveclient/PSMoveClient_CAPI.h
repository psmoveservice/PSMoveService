#ifndef __PSMOVECLIENT_CAPI_H
#define __PSMOVECLIENT_CAPI_H

#include "PSMoveClient_export.h"
#include "ClientConstants.h"

typedef int PSMRequestID;
typedef char PSMBool;
#define PSMFalse 0
#define PSMTrue 1

typedef enum _PSMResult
{
    PSMResult_Error                 = -1,
    PSMResult_Success               = 0,
    PSMResult_Timeout               = 1
} PSMResult;

typedef enum _PSMConnectionType
{
    PSMConnectionType_BLUETOOTH,
    PSMConnectionType_USB
    
} PSMConnectionType;

typedef enum _PSMButtonState {
    PSMButtonState_UP = 0x00,       // (00b) Not pressed
    PSMButtonState_PRESSED = 0x01,  // (01b) Down for one frame only
    PSMButtonState_DOWN = 0x03,     // (11b) Down for >1 frame
    PSMButtonState_RELEASED = 0x02, // (10b) Up for one frame only
} PSMButtonState;

typedef enum _PSMTrackingColorType
{
    PSMTrackingColorType_Magenta,    // R:0xFF, G:0x00, B:0xFF
    PSMTrackingColorType_Cyan,       // R:0x00, G:0xFF, B:0xFF
    PSMTrackingColorType_Yellow,     // R:0xFF, G:0xFF, B:0x00
    PSMTrackingColorType_Red,        // R:0xFF, G:0x00, B:0x00
    PSMTrackingColorType_Green,      // R:0x00, G:0xFF, B:0x00
    PSMTrackingColorType_Blue,       // R:0x00, G:0x00, B:0xFF
} PSMTrackingColorType;


/// A quaternion rotation.
typedef struct _PSMQuatf
{
    float x, y, z, w;
} PSMQuatf;

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

/// Position and orientation together.
typedef struct _PSMPosef
{
    PSMQuatf     Orientation;
    PSMVector3f  Position;
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
    enum        eShapeType
    {
        INVALID_PROJECTION = -1,
        Ellipse,
        Quad,
    };
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
    }           shape;
    
    eShapeType  shape_type;
} PSMTrackingProjection;

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
    bool                    bValid;
    bool                    bHasValidHardwareCalibration;
    bool                    bIsTrackingEnabled;
    bool                    bIsCurrentlyTracking;
    bool                    bHasUnpublishedState;
    
    char                    DevicePath[256];
    char                    DeviceSerial[128];
    char                    AssignedHostSerial[128];
    PSMBool                 PairedToHost;
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
    bool bValid;
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
    enum            eControllerType
    {
        None= -1,
        PSMove,
        PSNavi
    };
    union
    {
        PSMPSMove PSMoveState;
        PSMPSNavi PSNaviState;
    }               ControllerState;
    eControllerType ControllerType;
    unsigned int    ControllerID;
    int             OutputSequenceNum;
    int             InputSequenceNum;
    int             ListenerCount;
    bool            IsConnected;
    long long       DataFrameLastReceivedTime;
    float           DataFrameAverageFPS;
} PSMController;

//PSM_PUBLIC_FUNCTION(const char*) PSM_GetVersionString();
PSM_PUBLIC_FUNCTION(PSMResult) PSM_Initialize(const char* host, const char* port);  //"localhost", "9512"
PSM_PUBLIC_FUNCTION(PSMResult) PSM_Shutdown();
//PSM_PUBLIC_FUNCTION(PSMResult) PSM_Update();
//PSM_PUBLIC_FUNCTION(PSMResult) PSM_PollNextMessage();  //*message, sizeof(message)

/// Controller Methods
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetControllerList(PSMController** controllers, unsigned int timeout_msec);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_RegisterAsControllerListener(PSMController *controller);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StartControllerDataStream(PSMController *controller, unsigned int data_stream_flags);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StopControllerDataStream(PSMController *controller);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_DeregisterAsControllerListener(PSMController *controller);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_SetControllerLEDColor(PSMController *controller, PSMTrackingColorType tracking_color);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_ResetControllerPose(PSMController *controller);
PSM_PUBLIC_FUNCTION(PSMResult) PSM_UpdateController(PSMController *controller);

/// Tracker Methods
//PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackerList(PSMTracker** controllers, unsigned int timeout_msec);
//PSM_PUBLIC_FUNCTION(PSMResult) PSM_StartTrackerDataStream(PSMTracker* tracker);
//PSM_PUBLIC_FUNCTION(PSMResult) PSM_StopTrackerDataStream(PSMTracker* tracker);
//PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHMDTrackingSpaceSettings();



#endif
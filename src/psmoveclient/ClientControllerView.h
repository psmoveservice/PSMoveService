#ifndef CLIENT_CONTROLLER_VIEW_H
#define CLIENT_CONTROLLER_VIEW_H

//-- includes -----
#include "PSMoveClient_export.h"
#include "ClientConstants.h"
#include "ClientGeometry.h"
#include <cassert>

//-- pre-declarations -----
namespace PSMoveProtocol
{
    class DeviceOutputDataFrame;
    class DeviceOutputDataFrame_ControllerDataPacket;
    class DeviceInputDataFrame;
    class DeviceInputDataFrame_ControllerDataPacket;
};

//-- constants -----
enum PSMoveButtonState {
    PSMoveButton_UP = 0x00,       // (00b) Not pressed
    PSMoveButton_PRESSED = 0x01,  // (01b) Down for one frame only
    PSMoveButton_DOWN = 0x03,     // (11b) Down for >1 frame
    PSMoveButton_RELEASED = 0x02, // (10b) Up for one frame only
};

enum PSMoveTrackingColorType {
    Magenta,    // R:0xFF, G:0x00, B:0xFF
    Cyan,       // R:0x00, G:0xFF, B:0xFF
    Yellow,     // R:0xFF, G:0xFF, B:0x00
    Red,        // R:0xFF, G:0x00, B:0x00
    Green,      // R:0x00, G:0xFF, B:0x00
    Blue,       // R:0x00, G:0x00, B:0xFF

    MAX_PSMOVE_COLOR_TYPES
};

//-- declarations -----
struct PSM_CPP_PUBLIC_CLASS PSMovePhysicsData
{
    PSMoveFloatVector3 Velocity;
    PSMoveFloatVector3 Acceleration;
    PSMoveFloatVector3 AngularVelocity;
    PSMoveFloatVector3 AngularAcceleration;

    inline void Clear()
    {
        Velocity = *k_psmove_float_vector3_zero;
        Acceleration = *k_psmove_float_vector3_zero;
        AngularVelocity = *k_psmove_float_vector3_zero;
        AngularAcceleration = *k_psmove_float_vector3_zero;
    }
};

struct PSM_CPP_PUBLIC_CLASS PSMoveRawSensorData
{
    PSMoveIntVector3 Magnetometer;
    PSMoveFloatVector3 Accelerometer;
    PSMoveFloatVector3 Gyroscope;

    inline void Clear()
    {
        Magnetometer= *k_psmove_int_vector3_zero;
        Accelerometer= *k_psmove_float_vector3_zero;
        Gyroscope= *k_psmove_float_vector3_zero;
    }
};

struct PSM_CPP_PUBLIC_CLASS PSMoveRawTrackerData
{
    // Parallel arrays: ScreenLocations, Positions and the TrackerID associated with them
    PSMoveScreenLocation ScreenLocations[PSMOVESERVICE_MAX_TRACKER_COUNT];
    PSMovePosition RelativePositions[PSMOVESERVICE_MAX_TRACKER_COUNT];
    PSMoveTrackingProjection TrackingProjections[PSMOVESERVICE_MAX_TRACKER_COUNT];
    int TrackerIDs[PSMOVESERVICE_MAX_TRACKER_COUNT];
    int ValidTrackerLocations;

    inline void Clear()
    {
        for (int index = 0; index < PSMOVESERVICE_MAX_TRACKER_COUNT; ++index)
        {
            ScreenLocations[index] = PSMoveScreenLocation::create(0, 0);
            RelativePositions[index] = *k_psmove_position_origin;
            TrackerIDs[index] = -1;
        }
        ValidTrackerLocations = 0;
    }

    inline bool GetPixelLocationOnTrackerId(int trackerId, PSMoveScreenLocation &outLocation) const
    {
        bool bFound = false;

        for (int listIndex = 0; listIndex < ValidTrackerLocations; ++listIndex)
        {
            if (TrackerIDs[listIndex] == trackerId)
            {
                outLocation = ScreenLocations[listIndex];
                bFound = true;
                break;
            }
        }

        return bFound;
    }

    inline bool GetPositionOnTrackerId(int trackerId, PSMovePosition &outPosition) const
    {
        bool bFound = false;

        for (int listIndex = 0; listIndex < ValidTrackerLocations; ++listIndex)
        {
            if (TrackerIDs[listIndex] == trackerId)
            {
                outPosition = RelativePositions[listIndex];
                bFound = true;
                break;
            }
        }

        return bFound;
    }

    inline bool GetProjectionOnTrackerId(int trackerId, PSMoveTrackingProjection &outProjection) const
    {
        bool bFound = false;

        for (int listIndex = 0; listIndex < ValidTrackerLocations; ++listIndex)
        {
            if (TrackerIDs[listIndex] == trackerId)
            {
                outProjection = TrackingProjections[listIndex];
                bFound = true;
                break;
            }
        }

        return bFound;
    }
};

struct PSM_CPP_PUBLIC_CLASS ClientPSMoveView
{
private:
    bool bValid;
    bool bHasValidHardwareCalibration;
    bool bIsTrackingEnabled;
    bool bIsCurrentlyTracking;
    bool bHasUnpublishedState;

    PSMovePose Pose;
    PSMovePhysicsData PhysicsData;
    PSMoveRawSensorData RawSensorData;
    PSMoveRawTrackerData RawTrackerData;

    PSMoveButtonState TriangleButton;
    PSMoveButtonState CircleButton;
    PSMoveButtonState CrossButton;
    PSMoveButtonState SquareButton;
    PSMoveButtonState SelectButton;
    PSMoveButtonState StartButton;
    PSMoveButtonState PSButton;
    PSMoveButtonState MoveButton;
    PSMoveButtonState TriggerButton;

    unsigned char TriggerValue;

    unsigned char Rumble;
    unsigned char LED_r, LED_g, LED_b;

public:
    void Clear();
    void ApplyControllerDataFrame(const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket *data_frame);
    void Publish(PSMoveProtocol::DeviceInputDataFrame_ControllerDataPacket *data_frame);

    void SetRumble(float rumbleFraction);
    void SetLEDOverride(unsigned char red, unsigned char g, unsigned char b);

    inline bool IsValid() const
    {
        return bValid;
    }

    inline void SetValid(bool flag)
    {
        bValid= flag;
    }

    inline bool GetHasValidHardwareCalibration() const
    {
        return IsValid() ? bHasValidHardwareCalibration : false;
    }

    inline bool GetIsCurrentlyTracking() const
    {
        return IsValid() ? bIsCurrentlyTracking : false;
    }
    
    inline bool GetIsTrackingEnabled() const
    {
        return IsValid() ? bIsTrackingEnabled : false;
    }

    inline bool GetHasUnpublishedState() const
    {
        return IsValid() ? bHasUnpublishedState : false;
    }

    inline const PSMovePose &GetPose() const
    {
        return IsValid() ? Pose : *k_psmove_pose_identity;
    }

    inline const PSMovePosition &GetPosition() const
    {
        return IsValid() ? Pose.Position : *k_psmove_position_origin;
    }

    inline const PSMoveQuaternion &GetOrientation() const
    {
        return IsValid() ? Pose.Orientation : *k_psmove_quaternion_identity;
    }

    inline PSMoveButtonState GetButtonTriangle() const
    {
        return IsValid() ? TriangleButton : PSMoveButton_UP;
    }
    
    inline PSMoveButtonState GetButtonCircle() const
    {
        return IsValid() ? CircleButton : PSMoveButton_UP;
    }
    
    inline PSMoveButtonState GetButtonCross() const
    {
        return IsValid() ? CrossButton : PSMoveButton_UP;
    }
    
    inline PSMoveButtonState GetButtonSquare() const
    {
        return IsValid() ? SquareButton : PSMoveButton_UP;
    }
    
    inline PSMoveButtonState GetButtonSelect() const
    {
        return IsValid() ? SelectButton : PSMoveButton_UP;
    }
    
    inline PSMoveButtonState GetButtonStart() const
    {
        return IsValid() ? StartButton : PSMoveButton_UP;
    }
    
    inline PSMoveButtonState GetButtonPS() const
    {
        return IsValid() ? PSButton : PSMoveButton_UP;
    }
    
    inline PSMoveButtonState GetButtonMove() const
    {
        return IsValid() ? MoveButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonTrigger() const
    {
        return IsValid() ? TriggerButton : PSMoveButton_UP;
    }

    inline float GetTriggerValue() const
    {
        return IsValid() ? ((float)TriggerValue / 255.f) : 0.f;
    }

    inline float GetRumble() const
    {
        return IsValid() ? static_cast<float>(Rumble) / 255.f : 0.f;
    }

    const PSMovePhysicsData &GetPhysicsData() const;
    const PSMoveRawSensorData &GetRawSensorData() const;
    const PSMoveFloatVector3 &GetIdentityGravityCalibrationDirection() const;
    bool GetIsStableAndAlignedWithGravity() const;

    const PSMoveRawTrackerData &GetRawTrackerData() const;
};

class PSM_CPP_PUBLIC_CLASS ClientPSNaviView
{
private:
    bool bValid;

    PSMoveButtonState L1Button;
    PSMoveButtonState L2Button;
    PSMoveButtonState L3Button;
    PSMoveButtonState CircleButton;
    PSMoveButtonState CrossButton;
    PSMoveButtonState PSButton;
    PSMoveButtonState TriggerButton;
    PSMoveButtonState DPadUpButton;
    PSMoveButtonState DPadRightButton;
    PSMoveButtonState DPadDownButton;
    PSMoveButtonState DPadLeftButton;

    unsigned char TriggerValue;
    unsigned char Stick_XAxis;
    unsigned char Stick_YAxis;

public:
    void Clear();
    void ApplyControllerDataFrame(const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket *data_frame);
    void Publish(PSMoveProtocol::DeviceInputDataFrame_ControllerDataPacket *data_frame);

    inline bool IsValid() const
    {
        return bValid;
    }

    inline bool GetHasUnpublishedState() const
    {
        return false;
    }

    inline PSMoveButtonState GetButtonL1() const
    {
        return IsValid() ? L1Button : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonL2() const
    {
        return IsValid() ? L2Button : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonL3() const
    {
        return IsValid() ? L3Button : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonCircle() const
    {
        return IsValid() ? CircleButton : PSMoveButton_UP;
    }
    
    inline PSMoveButtonState GetButtonCross() const
    {
        return IsValid() ? CrossButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonPS() const
    {
        return IsValid() ? PSButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonTrigger() const
    {
        return IsValid() ? TriggerButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonDPadUp() const
    {
        return IsValid() ? DPadUpButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonDPadRight() const
    {
        return IsValid() ? DPadRightButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonDPadDown() const
    {
        return IsValid() ? DPadDownButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonDPadLeft() const
    {
        return IsValid() ? DPadLeftButton : PSMoveButton_UP;
    }

    inline float GetTriggerValue() const
    {
        return IsValid() ? ((float)TriggerValue / 255.f) : 0.f;
    }

    inline float GetStickXAxis() const
    {
        float float_value= 0.f;

        if (IsValid())
        {
            int signed_value= static_cast<int>(Stick_XAxis) - 0x80;

            float_value= static_cast<float>(signed_value) / static_cast<float>(0x80);
        }

        return float_value;
    }

    inline float GetStickYAxis() const
    {
        float float_value= 0.f;

        if (IsValid())
        {
            int signed_value= static_cast<int>(Stick_YAxis) - 0x80;

            float_value= static_cast<float>(signed_value) / static_cast<float>(0x80);
        }

        return float_value;
    }
};

class PSM_CPP_PUBLIC_CLASS ClientControllerView
{
public:
    enum eControllerType
    {
        None= -1,
        PSMove,
        PSNavi
    };

private:
    union
    {
        ClientPSMoveView PSMoveView;
        ClientPSNaviView PSNaviView;
    } ViewState;
    eControllerType ControllerViewType;

    int ControllerID;
    int OutputSequenceNum;
    int InputSequenceNum;
    int ListenerCount;

    bool IsConnected;

    long long data_frame_last_received_time;
    float data_frame_average_fps;

public:
    ClientControllerView(int ControllerID);

    void Clear();
    void ApplyControllerDataFrame(const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket *data_frame);
    void Publish();

    // Listener State
    inline void IncListenerCount()
    {
        ++ListenerCount;
    }

    inline void DecListenerCount()
    {
        assert(ListenerCount > 0);
        --ListenerCount;
    }

    inline int GetListenerCount() const
    {
        return ListenerCount;
    }

    // Controller Data Accessors
    inline int GetControllerID() const
    {
        return ControllerID;
    }

    inline int GetOutputSequenceNum() const
    {
        return IsValid() ? OutputSequenceNum : -1;
    }

    inline int GetInputSequenceNum() const
    {
        return IsValid() ? InputSequenceNum : -1;
    }

    bool GetHasUnpublishedState() const;

    inline eControllerType GetControllerViewType() const
    {
        return ControllerViewType;
    }

    inline const ClientPSMoveView &GetPSMoveView() const
    {
        assert(ControllerViewType == PSMove);
        return ViewState.PSMoveView;
    }

    inline ClientPSMoveView &GetPSMoveViewMutable()
    {
        assert(ControllerViewType == PSMove);
        return ViewState.PSMoveView;
    }

    inline const ClientPSNaviView &GetPSNaviView() const
    {
        assert(ControllerViewType == PSNavi);
        return ViewState.PSNaviView;
    }

    inline bool IsValid() const
    {
        return ControllerID != -1;
    }

    inline bool GetIsConnected() const
    {
        return (IsValid() && IsConnected);
    }
    
    // Statistics
    inline float GetDataFrameFPS() const
    {
        return data_frame_average_fps;
    }
};

#endif
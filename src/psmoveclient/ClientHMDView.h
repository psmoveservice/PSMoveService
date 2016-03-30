#ifndef CLIENT_HMD_VIEW_H
#define CLIENT_HMD_VIEW_H

//-- includes -----
#include "ClientConfig.h"
#include "ClientGeometry.h"
#include <cassert>

//-- pre-declarations -----
namespace PSMoveProtocol
{
    class DeviceDataFrame;
    class DeviceDataFrame_HMDDataPacket;
};

//-- constants -----

//-- declarations -----
struct CLIENTPSMOVEAPI OculusDK2RawSensorData
{
    PSMoveFloatVector3 Magnetometer;
    PSMoveFloatVector3 Accelerometer;
    PSMoveFloatVector3 Gyroscope;
    float Temparature;
    float IMUSampleTime;

    inline void Clear()
    {
        Magnetometer = *k_psmove_float_vector3_zero;
        Accelerometer = *k_psmove_float_vector3_zero;
        Gyroscope = *k_psmove_float_vector3_zero;
        Temparature = 0.f;
        IMUSampleTime = 0.f;
    }
};

struct CLIENTPSMOVEAPI ClientOculusDK2View
{
private:
    bool bValid;

    PSMoveFloatVector3 AngularVelocity;
    PSMoveFloatVector3 LinearVelocity;
    PSMoveFloatVector3 AngularAcceleration;
    PSMoveFloatVector3 LinearAcceleration;
    float StateTime;

    OculusDK2RawSensorData RawSensorData;

public:
    void Clear();
    void ApplyHMDDataFrame(const PSMoveProtocol::DeviceDataFrame_HMDDataPacket *data_frame);

    inline bool IsValid() const
    {
        return bValid;
    }

    inline void SetValid(bool flag)
    {
        bValid = flag;
    }

    inline const PSMoveFloatVector3 &GetAngularVelocity() const
    {
        return IsValid() ? AngularVelocity : *k_psmove_float_vector3_zero;
    }

    inline const PSMoveFloatVector3 &GetLinearVelocity() const
    {
        return IsValid() ? LinearVelocity : *k_psmove_float_vector3_zero;
    }

    inline const PSMoveFloatVector3 &GetAngularAcceleration() const
    {
        return IsValid() ? AngularAcceleration : *k_psmove_float_vector3_zero;
    }

    inline const PSMoveFloatVector3 &GetLinearAcceleration() const
    {
        return IsValid() ? LinearAcceleration : *k_psmove_float_vector3_zero;
    }

    inline const float GetStateTime() const
    {
        return IsValid() ? StateTime : 0.f;
    }

    const OculusDK2RawSensorData &GetRawSensorData() const;
};

class CLIENTPSMOVEAPI ClientHMDView
{
public:
    enum eHMDViewType
    {
        None = -1,
        OculusDK2,
    };

private:
    PSMovePose Pose;

    union
    {
        ClientOculusDK2View OculusDK2View;
    } ViewState;
    eHMDViewType HMDViewType;

    int HmdID;
    int SequenceNum;
    int ListenerCount;

    bool IsConnected;

    long long data_frame_last_received_time;
    float data_frame_average_fps;

public:
    ClientHMDView(int ControllerID);

    void Clear();
    void ApplyHMDDataFrame(const PSMoveProtocol::DeviceDataFrame_HMDDataPacket *data_frame);

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

    // HMD Data Accessors
    inline int GetHmdID() const
    {
        return HmdID;
    }

    inline int GetSequenceNum() const
    {
        return IsValid() ? SequenceNum : -1;
    }

    inline eHMDViewType GetHmdViewType() const
    {
        return HMDViewType;
    }

    inline PSMovePose GetHmdPose() const
    {
        return Pose;
    }

    inline const ClientOculusDK2View &GetOculusDK2View() const
    {
        assert(HMDViewType == OculusDK2);
        return ViewState.OculusDK2View;
    }

    inline bool IsValid() const
    {
        return HmdID != -1;
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
#endif // CLIENT_HMD_VIEW_H
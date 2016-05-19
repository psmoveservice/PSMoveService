#ifndef CLIENT_HMD_VIEW_H
#define CLIENT_HMD_VIEW_H

//-- includes -----
#include "ClientConfig.h"
#include "ClientGeometry.h"
#include <cassert>
#include <string>

//-- pre-declarations -----
namespace vr
{
    class IVRSystem;

    struct TrackedDevicePose_t;
}

//-- declarations -----
struct OpenVRHmdInfo
{
    int DeviceIndex;
    std::string TrackingSystemName;
    std::string ModelNumber;
    std::string SerialNumber;
    std::string ManufacturerName;
    std::string TrackingFirmwareVersion;
    std::string HardwareRevision;
    int EdidVendorID;
    int EdidProductID;

    void clear();
    void rebuild(int deviceIndex, vr::IVRSystem *pVRSystem);
};

class ClientHMDView
{
private:
    OpenVRHmdInfo m_hmdInfo;

    PSMovePose m_pose;
    PSMoveFloatVector3 m_angularVelocity;
    PSMoveFloatVector3 m_linearVelocity;

    PSMoveFloatVector3 m_xBasisVector; // +x
    PSMoveFloatVector3 m_yBasisVector; // +y
    PSMoveFloatVector3 m_zBasisVector; // -z

    int m_sequenceNum;
    int m_listenerCount;

    bool m_bIsConnected;
    bool m_bIsTracking;

    long long m_dataFrameLastReceivedTime;
    float m_dataFrameAverageFps;

public:
    ClientHMDView(int deviceIndex);

    void clear();
    void notifyConnected(vr::IVRSystem *pVRSystem);
    void notifyDisconnected(vr::IVRSystem *pVRSystem);
    void notifyPropertyChanged(vr::IVRSystem *pVRSystem);
    void applyHMDDataFrame(const vr::TrackedDevicePose_t *data_frame);

    // Listener State
    inline void incListenerCount()
    {
        ++m_listenerCount;
    }

    inline void decListenerCount()
    {
        assert(m_listenerCount > 0);
        --m_listenerCount;
    }

    inline int getListenerCount() const
    {
        return m_listenerCount;
    }

    // HMD Data Accessors
    inline const OpenVRHmdInfo &getHmdInfo() const
    {
        return m_hmdInfo;
    }

    inline int getSequenceNum() const
    {
        return getIsConnected() ? m_sequenceNum : -1;
    }

    inline PSMovePose getHmdPose() const
    {
        return m_pose;
    }

    inline const PSMoveFloatVector3 &getAngularVelocity() const
    {
        return getIsConnected() ? m_angularVelocity : *k_psmove_float_vector3_zero;
    }

    inline const PSMoveFloatVector3 &getLinearVelocity() const
    {
        return getIsConnected() ? m_linearVelocity : *k_psmove_float_vector3_zero;
    }

    inline bool getIsConnected() const
    {
        return m_bIsConnected;
    }

    inline bool getIsTracking() const
    {
        return m_bIsTracking;
    }

    bool getIsStableAndAlignedWithGravity() const;

    // Statistics
    inline float getDataFrameFPS() const
    {
        return m_dataFrameAverageFps;
    }
};
#endif // CLIENT_HMD_VIEW_H
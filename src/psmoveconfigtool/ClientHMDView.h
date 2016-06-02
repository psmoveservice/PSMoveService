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
    void rebuild(vr::IVRSystem *pVRSystem);
};

class ClientHMDView
{
private:
    OpenVRHmdInfo m_hmdInfo;

    PSMovePose m_hmdPose;
    PSMoveFloatVector3 m_hmdXBasisVector;
    PSMoveFloatVector3 m_hmdYBasisVector;
    PSMoveFloatVector3 m_hmdZBasisVector;

    PSMoveFloatVector3 m_hmdAngularVelocity;
    PSMoveFloatVector3 m_hmdLinearVelocity;

    int m_hmdSequenceNum;
    int m_trackerSequenceNum;
    int m_listenerCount;

    bool m_bIsHMDConnected;
    bool m_bIsHMDTracking;

    long long m_dataFrameLastReceivedTime;
    float m_dataFrameAverageFps;

public:
    ClientHMDView(int hmdDeviceIndex);

    void clear();
    void notifyConnected(vr::IVRSystem *pVRSystem, int deviceIndex);
    void notifyDisconnected(vr::IVRSystem *pVRSystem, int deviceIndex);
    void notifyPropertyChanged(vr::IVRSystem *pVRSystem, int deviceIndex);
    void applyHMDDataFrame(const vr::TrackedDevicePose_t *data_frame);
    void applyTrackerDataFrame(const vr::TrackedDevicePose_t *data_frame);

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

    inline int getHMDSequenceNum() const
    {
        return getIsHMDConnected() ? m_hmdSequenceNum : -1;
    }

    inline PSMovePose getHmdPose() const
    {
        return m_hmdPose;
    }

    inline const PSMoveFloatVector3 &getHMDAngularVelocity() const
    {
        return getIsHMDConnected() ? m_hmdAngularVelocity : *k_psmove_float_vector3_zero;
    }

    inline const PSMoveFloatVector3 &getHMDLinearVelocity() const
    {
        return getIsHMDConnected() ? m_hmdLinearVelocity : *k_psmove_float_vector3_zero;
    }

    inline bool getIsHMDConnected() const
    {
        return m_bIsHMDConnected;
    }

    inline bool getIsHMDTracking() const
    {
        return m_bIsHMDTracking;
    }

    bool getIsHMDStableAndAlignedWithGravity() const;

    // Statistics
    inline float getDataFrameFPS() const
    {
        return m_dataFrameAverageFps;
    }
};
#endif // CLIENT_HMD_VIEW_H
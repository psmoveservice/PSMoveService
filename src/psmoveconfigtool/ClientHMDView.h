#ifndef CLIENT_HMD_VIEW_H
#define CLIENT_HMD_VIEW_H

//-- includes -----
#include "PSMoveClient_export.h"
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

    PSMovePose m_rawHmdPose;
    PSMoveFloatVector3 m_rawHmdXBasisVector;
    PSMoveFloatVector3 m_rawHmdYBasisVector;
    PSMoveFloatVector3 m_rawHmdZBasisVector;
    PSMoveFloatVector3 m_rawHmdAngularVelocity;
    PSMoveFloatVector3 m_rawHmdLinearVelocity;

    PSMovePose m_ChaperoneSpaceHmdPose;

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
    void applyHMDDataFrame(
        const vr::TrackedDevicePose_t *raw_data_frame, 
        const vr::TrackedDevicePose_t *chaperone_data_frame);

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

    // This is the raw HMD pose that comes from the driver.
    // You want to use this pose when computing calibration state.
    inline PSMovePose getRawHmdPose() const
    {
        return m_rawHmdPose;
    }

    // This is the HMD pose relative to the chaperone tracking space in OpenVR.
    // You want to use this when rendering the HMD and tracking space.
    inline PSMovePose getChaperoneSpaceHmdPose() const
    {
        return m_ChaperoneSpaceHmdPose;
    }

    inline const PSMoveFloatVector3 &getRawHMDAngularVelocity() const
    {
        return getIsHMDConnected() ? m_rawHmdAngularVelocity : *k_psmove_float_vector3_zero;
    }

    inline const PSMoveFloatVector3 &getRawHMDLinearVelocity() const
    {
        return getIsHMDConnected() ? m_rawHmdLinearVelocity : *k_psmove_float_vector3_zero;
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
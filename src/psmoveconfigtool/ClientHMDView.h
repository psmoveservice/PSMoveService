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

struct OpenVRTrackerInfo
{
    int DeviceIndex;
    std::string TrackingSystemName;
    std::string ModelNumber;
    std::string SerialNumber;
    std::string ManufacturerName;
    std::string TrackingFirmwareVersion;
    std::string HardwareRevision;
    std::string ModeLabel;
    int EdidVendorID;
    int EdidProductID;

    float FieldOfViewLeftDegrees;
    float FieldOfViewRightDegrees;
    float FieldOfViewTopDegrees;
    float FieldOfViewBottomDegrees;
    float TrackingRangeMinimumMeters;
    float TrackingRangeMaximumMeters;

    void clear();
    void rebuild(vr::IVRSystem *pVRSystem);
};

class ClientHMDView
{
private:
    OpenVRHmdInfo m_hmdInfo;
    OpenVRTrackerInfo m_trackerInfo;

    PSMovePose m_trackerPose;
    PSMoveFloatVector3 m_trackerXBasisVector;
    PSMoveFloatVector3 m_trackerYBasisVector;
    PSMoveFloatVector3 m_trackerZBasisVector;

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

    bool m_bIsTrackerTracking;
    bool m_bIsTrackerConnected;

    long long m_dataFrameLastReceivedTime;
    float m_dataFrameAverageFps;

public:
    ClientHMDView(int hmdDeviceIndex, int trackerDeviceIndex);

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

    // Tracker Data Accessors
    inline const OpenVRTrackerInfo &getHmdTrackerInfo() const
    {
        return m_trackerInfo;
    }
    
    inline int getTrackerSequenceNum() const
    {
        return getIsTrackerConnected() ? m_trackerSequenceNum : -1;
    }

    inline PSMovePose getTrackerPose() const
    {
        return m_hmdPose;
    }

    inline bool getIsTrackerConnected() const
    {
        return m_bIsTrackerConnected;
    }

    inline bool getIsTrackerTracking() const
    {
        return m_bIsTrackerTracking;
    }

    PSMoveFrustum getTrackerFrustum() const;

    // Statistics
    inline float getDataFrameFPS() const
    {
        return m_dataFrameAverageFps;
    }
};
#endif // CLIENT_HMD_VIEW_H
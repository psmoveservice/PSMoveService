#ifndef OPENVR_CONTEXT_H
#define OPENVR_CONTEXT_H

//-- includes -----
#include "ClientGeometry.h"
#include <vector>

//-- typedefs -----
namespace vr
{
    class IVRSystem;
    class IVRCompositor;
    class IVRRenderModels;
    class IVRChaperone;

    struct VREvent_t;
    struct TrackedDevicePose_t;
}

class ClientHMDView;

//-- definitions -----

class OpenVRContext
{
public:
    OpenVRContext();
    virtual ~OpenVRContext();

    bool init();
    void update();
    void destroy();

    inline bool getIsInitialized() const
    {
        return m_bIsInitialized;
    }

    int getHmdList(struct OpenVRHmdInfo *outHmdList, int maxListSize);

    ClientHMDView *allocateHmdView();
    void freeHmdView(ClientHMDView *view);

    void setHMDTrackingSpaceOrigin(const PSMovePose &pose);
    PSMovePose getHMDPoseAtPSMoveTrackingSpaceOrigin() const;
    bool getHMDTrackingSpaceSize(float &outWidth, float &outHeight) const;
    bool getHMDTrackingVolume(PSMoveVolume &volume) const;

private:
    bool m_bIsInitialized;
    PSMovePose m_hmdOriginPose;

    vr::IVRSystem *m_pVRSystem;
    vr::IVRRenderModels *m_pRenderModels;
    vr::IVRChaperone *m_pChaperone;
    vr::TrackedDevicePose_t *m_pTrackedDevicePoseArray;
    ClientHMDView *m_hmdView;

    void processVREvent(const vr::VREvent_t & event);
};

#endif // OPENVR_CONTEXT_H
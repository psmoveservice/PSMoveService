#ifndef OPENVR_CONTEXT_H
#define OPENVR_CONTEXT_H

//-- includes -----
#include <vector>

//-- typedefs -----
namespace vr
{
    class IVRSystem;
    class IVRCompositor;
    class IVRRenderModels;

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
    int getHmdTrackerList(struct OpenVRTrackerInfo *outTrackerList, int maxListSize);

    ClientHMDView *allocateHmdView();
    void freeHmdView(ClientHMDView *view);

private:
    bool m_bIsInitialized;

    vr::IVRSystem *m_pVRSystem;
    vr::IVRRenderModels *m_pRenderModels;
    vr::TrackedDevicePose_t *m_pTrackedDevicePoseArray;
    ClientHMDView *m_hmdView;

    void processVREvent(const vr::VREvent_t & event);
};

#endif // OPENVR_CONTEXT_H
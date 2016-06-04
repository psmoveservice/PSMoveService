//-- includes -----
#include "OpenVRContext.h"
#include "ClientHMDView.h"
#include "Logger.h"
#include "GeometryUtility.h"

#include "openvr.h"

//-- constants -----
const float k_meters_to_centimenters = 100.f;
const float k_chaperone_height_centimenters = 200.f;

//-- methods -----
OpenVRContext::OpenVRContext()
    : m_bIsInitialized(false)
    , m_hmdOriginPose(*k_psmove_pose_identity)
    , m_pVRSystem(nullptr)
    , m_pRenderModels(nullptr)
    , m_pTrackedDevicePoseArray(new vr::TrackedDevicePose_t[vr::k_unMaxTrackedDeviceCount])
    , m_hmdView(nullptr)
{
}

OpenVRContext::~OpenVRContext()
{
    delete[] m_pTrackedDevicePoseArray;
}

bool OpenVRContext::init()
{
    bool bSuccess = true;
    vr::EVRInitError eError = vr::VRInitError_None;

    // Loading the SteamVR Runtime
    m_pVRSystem = vr::VR_Init(&eError, vr::VRApplication_Scene);
    if (eError != vr::VRInitError_None)
    {
        m_pVRSystem = nullptr;
        Log_ERROR("OpenVRContext::startup", "Failed to initialize OpenVR: %s!", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
        bSuccess = false;
    }

    if (bSuccess)
    {
        m_pChaperone = (vr::IVRChaperone *)vr::VR_GetGenericInterface(vr::IVRChaperone_Version, &eError);
        if (m_pChaperone == nullptr)
        {
            Log_ERROR("OpenVRContext::startup", "Unable to get chaperone interface: %s!", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
            bSuccess = false;
        }
    }

    if (bSuccess)
    {
        m_pRenderModels = (vr::IVRRenderModels *)vr::VR_GetGenericInterface(vr::IVRRenderModels_Version, &eError);
        if (m_pRenderModels == nullptr)
        {
            Log_ERROR("OpenVRContext::startup", "Unable to get render model interface: %s!", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
            bSuccess = false;
        }
    }

    if (bSuccess)
    {
        m_bIsInitialized = true;
    }
    else
    {
        destroy();
    }

    return bSuccess;
}

void OpenVRContext::destroy()
{
    if (m_hmdView != nullptr)
    {
        delete m_hmdView;
        m_hmdView = nullptr;
    }

    if (m_pVRSystem != nullptr)
    {
        vr::VR_Shutdown();
        m_pChaperone= nullptr;
        m_pRenderModels = nullptr;
        m_pVRSystem = nullptr;
    }

    m_bIsInitialized = false;
}

void OpenVRContext::update()
{
    if (getIsInitialized())
    {
        // Fetch the latest tracking data on all tracked devices
        m_pVRSystem->GetDeviceToAbsoluteTrackingPose(
            vr::TrackingUniverseStanding,
            0.f, // no prediction needed
            m_pTrackedDevicePoseArray,
            vr::k_unMaxTrackedDeviceCount);

        // Update the HMD pose
        if (m_hmdView != nullptr)
        {
            m_hmdView->applyHMDDataFrame(&m_pTrackedDevicePoseArray[vr::k_unTrackedDeviceIndex_Hmd]);
        }

        // Process OpenVR events
        vr::VREvent_t event;
        while (m_pVRSystem->PollNextEvent(&event, sizeof(event)))
        {
            processVREvent(event);
        }
    }
}

void OpenVRContext::processVREvent(const vr::VREvent_t & event)
{
    if (m_hmdView != nullptr)
    {
        switch (event.eventType)
        {
        case vr::VREvent_TrackedDeviceActivated:
            {
                //SetupRenderModelForTrackedDevice(event.trackedDeviceIndex);
                Log_INFO("OpenVRContext::processVREvent", "Device %u attached. Setting up render model.\n", event.trackedDeviceIndex);
                m_hmdView->notifyConnected(m_pVRSystem, event.trackedDeviceIndex);
            }
            break;
        case vr::VREvent_TrackedDeviceDeactivated:
            {
                Log_INFO("OpenVRContext::processVREvent", "Device %u detached.\n", event.trackedDeviceIndex);
                m_hmdView->notifyDisconnected(m_pVRSystem, event.trackedDeviceIndex);
            }
            break;
        case vr::VREvent_TrackedDeviceUpdated:
            {
                Log_INFO("OpenVRContext::processVREvent", "Device %u updated.\n", event.trackedDeviceIndex);
                m_hmdView->notifyPropertyChanged(m_pVRSystem, event.trackedDeviceIndex);
            }
            break;
        }
    }
}

int OpenVRContext::getHmdList(OpenVRHmdInfo *outHmdList, int maxListSize)
{
    int listCount = 0;

    if (getIsInitialized())
    {
        for (vr::TrackedDeviceIndex_t deviceIndex = 0; 
            deviceIndex < vr::k_unMaxTrackedDeviceCount && listCount < maxListSize;
            ++deviceIndex)
        {
            if (m_pVRSystem->IsTrackedDeviceConnected(deviceIndex) && 
                m_pVRSystem->GetTrackedDeviceClass(deviceIndex) == vr::TrackedDeviceClass_HMD)
            {
                OpenVRHmdInfo &entry = outHmdList[listCount];

                entry.clear();
                entry.DeviceIndex = deviceIndex;
                entry.rebuild(m_pVRSystem);
                ++listCount;
            }
        }
    }

    return listCount;
}

ClientHMDView *OpenVRContext::allocateHmdView()
{
    ClientHMDView * result = nullptr;

    if (getIsInitialized())
    {
        if (m_hmdView == nullptr)
        {
            OpenVRHmdInfo hmdList[1];

            int hmdCount = getHmdList(hmdList, sizeof(hmdList));

            if (hmdCount > 0)
            {
                m_hmdView = new ClientHMDView(hmdList[0].DeviceIndex);

                if (m_pVRSystem->IsTrackedDeviceConnected(hmdList[0].DeviceIndex))
                {
                    m_hmdView->notifyConnected(m_pVRSystem, hmdList[0].DeviceIndex);
                }
            }
        }

        if (m_hmdView != nullptr)
        {
            m_hmdView->incListenerCount();
        }

        result = m_hmdView;
    }

    return result;
}

void OpenVRContext::freeHmdView(ClientHMDView *view)
{
    if (getIsInitialized())
    {
        assert(m_hmdView != nullptr);
        m_hmdView->decListenerCount();

        if (m_hmdView->getListenerCount() <= 0)
        {
            delete m_hmdView;
            m_hmdView = nullptr;
        }
    }
}

void OpenVRContext::setHMDTrackingSpaceOrigin(const struct PSMovePose &pose)
{
    m_hmdOriginPose = pose;
}

PSMovePose OpenVRContext::getHMDPoseAtPSMoveTrackingSpaceOrigin() const
{
    return m_hmdOriginPose;
}

bool OpenVRContext::getHMDTrackingSpaceSize(float &outSizeX, float &outSizeZ) const
{
    bool bSuccess= false;

    if (m_pChaperone->GetPlayAreaSize(&outSizeX, &outSizeZ))
    {
        outSizeX *= k_meters_to_centimenters;
        outSizeZ *= k_meters_to_centimenters;

        bSuccess = true;
    }

    return bSuccess;
}

bool OpenVRContext::getHMDTrackingVolume(PSMoveVolume &volume) const
{
    bool bSuccess = false;

    vr::HmdQuad_t rect;
    if (m_pChaperone->GetPlayAreaRect(&rect))
    {
        for (int index = 0; index < 4; ++index)
        {
            const vr::HmdVector3_t &openvr_point= rect.vCorners[index];
            const glm::vec4 glm_point=
                glm::vec4(
                    openvr_point.v[0] * k_meters_to_centimenters, 
                    openvr_point.v[1] * k_meters_to_centimenters,
                    openvr_point.v[2] * k_meters_to_centimenters,
                    1.f);

            volume.vertices[index] = PSMovePosition::create(glm_point.x, glm_point.y, glm_point.z);
        }
        volume.vertex_count = 4;
        volume.up_height = k_chaperone_height_centimenters; // Pick the y value for any corner to get the height

        bSuccess = true;
    }

    return bSuccess;
}
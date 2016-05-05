//-- includes -----
#include "ClientHMDView.h"
#include "Logger.h"
#include "math.h"
#include "openvr.h"

//-- constants -----
const float k_meters_to_centimenters = 100.f;

//-- prototypes -----
static std::string trackedDeviceGetString(
    vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice,
    vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = nullptr);

static PSMoveQuaternion openvrMatrixExtractPSMoveQuaternion(const vr::HmdMatrix34_t &openVRTransform);
static PSMovePosition openvrMatrixExtractPSMovePosition(const vr::HmdMatrix34_t &openVRTransform);
static PSMoveFloatVector3 openvrVectorToPSMoveVector(const vr::HmdVector3_t &openVRVector);

//-- methods -----
//-- OpenVRHmdInfo --
void OpenVRHmdInfo::clear()
{
    DeviceIndex= -1;
    TrackingSystemName = "";
    ModelNumber = "";
    SerialNumber = "";
    ManufacturerName = "";
    TrackingFirmwareVersion = "";
    HardwareRevision = "";
    EdidVendorID = -1;
    EdidProductID = -1;
}

void OpenVRHmdInfo::rebuild(int deviceIndex, vr::IVRSystem *pVRSystem)
{
    DeviceIndex = deviceIndex;
    TrackingSystemName = trackedDeviceGetString(pVRSystem, deviceIndex, vr::Prop_TrackingSystemName_String);
    ModelNumber = trackedDeviceGetString(pVRSystem, deviceIndex, vr::Prop_ModelNumber_String);
    SerialNumber = trackedDeviceGetString(pVRSystem, deviceIndex, vr::Prop_SerialNumber_String);
    ManufacturerName = trackedDeviceGetString(pVRSystem, deviceIndex, vr::Prop_ManufacturerName_String);
    HardwareRevision = trackedDeviceGetString(pVRSystem, deviceIndex, vr::Prop_HardwareRevision_String);
    EdidProductID = pVRSystem->GetInt32TrackedDeviceProperty(deviceIndex, vr::Prop_EdidProductID_Int32);
    EdidVendorID = pVRSystem->GetInt32TrackedDeviceProperty(deviceIndex, vr::Prop_EdidVendorID_Int32);
}

// -- ClientHMDView --
ClientHMDView::ClientHMDView(int deviceIndex) 
{
    clear();
    m_hmdInfo.DeviceIndex = deviceIndex;
}

void ClientHMDView::clear()
{
    m_hmdInfo.clear();
    m_pose.Clear();
    m_angularVelocity = *k_psmove_float_vector3_zero;
    m_linearVelocity = *k_psmove_float_vector3_zero;

    m_sequenceNum= 0;
    m_listenerCount= 0;

    m_bIsConnected= false;

    m_dataFrameLastReceivedTime = -1LL;
    m_dataFrameAverageFps= 0.f;
}

void ClientHMDView::notifyConnected(vr::IVRSystem *pVRSystem)
{
    m_bIsConnected = true;
    m_hmdInfo.rebuild(m_hmdInfo.DeviceIndex, pVRSystem);
}

void ClientHMDView::notifyDisconnected(vr::IVRSystem *pVRSystem)
{
    m_bIsConnected = false;
}

void ClientHMDView::notifyPropertyChanged(vr::IVRSystem *pVRSystem)
{
    m_hmdInfo.rebuild(m_hmdInfo.DeviceIndex, pVRSystem);
}

void ClientHMDView::applyHMDDataFrame(const vr::TrackedDevicePose_t *data_frame)
{
    if (data_frame->bPoseIsValid)
    {
        // OpenVR uses right-handed coordinate system:
        // +y is up
        // +x is to the right
        // -z is going away from you
        // Distance unit is meters

        // PSMoveAPI also uses the same right-handed coordinate system
        // Distance unit is centimeters
        m_pose.Orientation = openvrMatrixExtractPSMoveQuaternion(data_frame->mDeviceToAbsoluteTracking);
        m_pose.Position = openvrMatrixExtractPSMovePosition(data_frame->mDeviceToAbsoluteTracking) *k_meters_to_centimenters;
        m_angularVelocity= openvrVectorToPSMoveVector(data_frame->vAngularVelocity);
        m_linearVelocity = openvrVectorToPSMoveVector(data_frame->vVelocity) *k_meters_to_centimenters;

        // Increment the sequence number now that that we have new data
        ++m_sequenceNum;
    }
}

// From: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
static PSMoveQuaternion openvrMatrixExtractPSMoveQuaternion(const vr::HmdMatrix34_t &openVRTransform)
{
    PSMoveQuaternion q;

    const float(&a)[3][4] = openVRTransform.m;
    const float trace = a[0][0] + a[1][1] + a[2][2]; 

    if (trace > 0) 
    {
        const float s = 0.5f / sqrtf(trace + 1.0f);

        q.w = 0.25f / s;
        q.x = (a[2][1] - a[1][2]) * s;
        q.y = (a[0][2] - a[2][0]) * s;
        q.z = (a[1][0] - a[0][1]) * s;
    }
    else
    {
        if (a[0][0] > a[1][1] && a[0][0] > a[2][2]) 
        {
            const float s = 2.0f * sqrtf(1.0f + a[0][0] - a[1][1] - a[2][2]);

            q.w = (a[2][1] - a[1][2]) / s;
            q.x = 0.25f * s;
            q.y = (a[0][1] + a[1][0]) / s;
            q.z = (a[0][2] + a[2][0]) / s;
        }
        else if (a[1][1] > a[2][2]) 
        {
            const float s = 2.0f * sqrtf(1.0f + a[1][1] - a[0][0] - a[2][2]);

            q.w = (a[0][2] - a[2][0]) / s;
            q.x = (a[0][1] + a[1][0]) / s;
            q.y = 0.25f * s;
            q.z = (a[1][2] + a[2][1]) / s;
        }
        else 
        {
            const float s = 2.0f * sqrtf(1.0f + a[2][2] - a[0][0] - a[1][1]);

            q.w = (a[1][0] - a[0][1]) / s;
            q.x = (a[0][2] + a[2][0]) / s;
            q.y = (a[1][2] + a[2][1]) / s;
            q.z = 0.25f * s;
        }
    }

    return q;
}

static PSMovePosition openvrMatrixExtractPSMovePosition(const vr::HmdMatrix34_t &openVRTransform)
{
    const float(&a)[3][4] = openVRTransform.m;

    return PSMovePosition::create(a[0][3], a[1][3], a[2][3]);
}

static PSMoveFloatVector3 openvrVectorToPSMoveVector(const vr::HmdVector3_t &openVRVector)
{
    const float (&v)[3] = openVRVector.v;

    return PSMoveFloatVector3::create(v[0], v[1], v[2]);
}

//-- private helpers -----
static std::string trackedDeviceGetString(
    vr::IVRSystem *pVRSystem,
    vr::TrackedDeviceIndex_t unDevice,
    vr::TrackedDeviceProperty prop,
    vr::TrackedPropertyError *peError)
{
    uint32_t unRequiredBufferLen = pVRSystem->GetStringTrackedDeviceProperty(unDevice, prop, nullptr, 0, peError);
    if (unRequiredBufferLen == 0)
        return "";

    char *pchBuffer = new char[unRequiredBufferLen];
    unRequiredBufferLen = pVRSystem->GetStringTrackedDeviceProperty(unDevice, prop, pchBuffer, unRequiredBufferLen, peError);
    std::string sResult = pchBuffer;
    delete[] pchBuffer;
    return sResult;
}
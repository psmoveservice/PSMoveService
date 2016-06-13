//-- includes -----
#include "ClientHMDView.h"
#include "ClientGeometry.h"
#include "Logger.h"
#include "MathUtility.h"
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

void OpenVRHmdInfo::rebuild(vr::IVRSystem *pVRSystem)
{
    if (DeviceIndex != -1)
    {
        TrackingSystemName = trackedDeviceGetString(pVRSystem, DeviceIndex, vr::Prop_TrackingSystemName_String);
        ModelNumber = trackedDeviceGetString(pVRSystem, DeviceIndex, vr::Prop_ModelNumber_String);
        SerialNumber = trackedDeviceGetString(pVRSystem, DeviceIndex, vr::Prop_SerialNumber_String);
        ManufacturerName = trackedDeviceGetString(pVRSystem, DeviceIndex, vr::Prop_ManufacturerName_String);
        HardwareRevision = trackedDeviceGetString(pVRSystem, DeviceIndex, vr::Prop_HardwareRevision_String);
        EdidProductID = pVRSystem->GetInt32TrackedDeviceProperty(DeviceIndex, vr::Prop_EdidProductID_Int32);
        EdidVendorID = pVRSystem->GetInt32TrackedDeviceProperty(DeviceIndex, vr::Prop_EdidVendorID_Int32);
    }
}

// -- ClientHMDView --
ClientHMDView::ClientHMDView(int hmdDeviceIndex)
{
    clear();
    m_hmdInfo.DeviceIndex = hmdDeviceIndex;
}

void ClientHMDView::clear()
{
    m_hmdInfo.clear();

    m_rawHmdPose.Clear();
    m_rawHmdAngularVelocity = *k_psmove_float_vector3_zero;
    m_rawHmdLinearVelocity = *k_psmove_float_vector3_zero;
    m_rawHmdXBasisVector = *k_psmove_float_vector3_i;
    m_rawHmdYBasisVector = *k_psmove_float_vector3_j;
    m_rawHmdZBasisVector = *k_psmove_float_vector3_k;

    m_ChaperoneSpaceHmdPose.Clear();

    m_hmdSequenceNum= 0;
    m_trackerSequenceNum = 0;
    m_listenerCount= 0;

    m_bIsHMDConnected= false;
    m_bIsHMDTracking = false;

    m_dataFrameLastReceivedTime = -1LL;
    m_dataFrameAverageFps= 0.f;
}

void ClientHMDView::notifyConnected(vr::IVRSystem *pVRSystem, int deviceIndex)
{
    if (deviceIndex == m_hmdInfo.DeviceIndex)
    {
        m_hmdInfo.rebuild(pVRSystem);
        m_bIsHMDConnected = true;
    }
}

void ClientHMDView::notifyDisconnected(vr::IVRSystem *pVRSystem, int deviceIndex)
{
    if (deviceIndex == m_hmdInfo.DeviceIndex)
    {
        m_bIsHMDConnected = false;
    }
}

void ClientHMDView::notifyPropertyChanged(vr::IVRSystem *pVRSystem, int deviceIndex)
{
    if (deviceIndex == m_hmdInfo.DeviceIndex)
    {
        m_hmdInfo.rebuild(pVRSystem);
    }
}

void ClientHMDView::applyHMDDataFrame(
    const vr::TrackedDevicePose_t *raw_data_frame,
    const vr::TrackedDevicePose_t *chaperone_data_frame)
{
    if (raw_data_frame->bPoseIsValid && chaperone_data_frame->bPoseIsValid)
    {
        {
            const float(&m)[3][4] = raw_data_frame->mDeviceToAbsoluteTracking.m;

            // OpenVR uses right-handed coordinate system:
            // +y is up
            // +x is to the right
            // -z is going away from you
            // Distance unit is meters
            m_rawHmdXBasisVector = PSMoveFloatVector3::create(m[0][0], m[1][0], m[2][0]);
            m_rawHmdYBasisVector = PSMoveFloatVector3::create(m[0][1], m[1][1], m[2][1]);
            m_rawHmdZBasisVector = PSMoveFloatVector3::create(m[0][2], m[1][2], m[2][2]);

            // PSMoveAPI also uses the same right-handed coordinate system
            // Distance unit is centimeters
            m_rawHmdPose.Orientation = openvrMatrixExtractPSMoveQuaternion(raw_data_frame->mDeviceToAbsoluteTracking);
            m_rawHmdPose.Position = openvrMatrixExtractPSMovePosition(raw_data_frame->mDeviceToAbsoluteTracking) * k_meters_to_centimenters;
            m_rawHmdAngularVelocity = openvrVectorToPSMoveVector(raw_data_frame->vAngularVelocity);
            m_rawHmdLinearVelocity = openvrVectorToPSMoveVector(raw_data_frame->vVelocity) *k_meters_to_centimenters;
        }

        {
            m_ChaperoneSpaceHmdPose.Orientation = 
                openvrMatrixExtractPSMoveQuaternion(chaperone_data_frame->mDeviceToAbsoluteTracking);
            m_ChaperoneSpaceHmdPose.Position = 
                openvrMatrixExtractPSMovePosition(chaperone_data_frame->mDeviceToAbsoluteTracking) * k_meters_to_centimenters;
        }

        // Increment the sequence number now that that we have new data
        ++m_hmdSequenceNum;

        m_bIsHMDTracking = true;
    }
    else
    {
        m_bIsHMDTracking = false;
    }
}

bool ClientHMDView::getIsHMDStableAndAlignedWithGravity() const
{
    // Other HMDs don't have a completely flat bottom like the DK2 does so they won't lie level.
    // Thus the 30 degree tolerance.
    const float k_cosine_30_degrees = 0.866025f;
    const float velocity_magnitude = m_rawHmdLinearVelocity.length();

    const bool isOk =
        is_nearly_equal(velocity_magnitude, 0.f, 0.5f) &&
        PSMoveFloatVector3::dot(m_rawHmdYBasisVector, PSMoveFloatVector3::create(0.f, 1.f, 0.f)) >= k_cosine_30_degrees;

    return isOk;
}

//-- private helpers -----
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
//-- includes -----
#include "OculusHMD.h"
#include "DeviceInterface.h"
#include "DeviceManager.h"
#include "HMDDeviceEnumerator.h"
#include "MathUtility.h"
#include "ServerLog.h"
#include "ServerUtility.h"
#include "OVR_CAPI.h"
#include <vector>
#include <cstdlib>
#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif
#include <math.h>

//-- constants -----
#define OCULUS_HMD_STATE_BUFFER_MAX 4
#define METERS_TO_CENTIMETERS 100

// -- private definitions -----
class OculusHIDDetails 
{
public:
    std::string Device_path;
    std::string Device_serial;
    ovrHmdDesc HmdDesc;
#if defined(OVR_OS_WIN32)
    ovrSession SessionHandle;
    ovrGraphicsLuid Luid;
#elif defined(OVR_OS_MAC)
    ovrHmd HmdHandle;
#endif

    OculusHIDDetails()
    {
        Reset();
    }

    void Reset()
    {
        Device_path = "";
        Device_serial = "";
        memset(&HmdDesc, 0, sizeof(ovrHmdDesc));
#if defined(OVR_OS_WIN32)
        memset(&SessionHandle, 0, sizeof(ovrSession));
        memset(&Luid, 0, sizeof(ovrGraphicsLuid));
#elif defined(OVR_OS_MAC)
        HmdHandle = nullptr;
#endif
    }
};

class OculusDataInput
{
public:
    ovrTrackingState TrackingState;

    OculusDataInput()
    {
        Reset();
    }

    void Reset()
    {
        memset(&TrackingState, 0, sizeof(ovrTrackingState));
    }
};

// -- private prototypes -----
static CommonDevicePosition ovrVector3f_to_CommonDevicePosition(const ovrVector3f &p, const float scale = 1.f);
static CommonDeviceVector ovrVector3f_to_CommonDeviceVector(const ovrVector3f &v, const float scale = 1.f);
static CommonDeviceQuaternion ovrQuatf_to_CommonDeviceQuaternion(const ovrQuatf &q);

// -- public methods

// -- PSMove Controller Config
const int OculusHMDConfig::CONFIG_VERSION = 1;

const boost::property_tree::ptree
OculusHMDConfig::config2ptree()
{
    boost::property_tree::ptree pt;

    pt.put("is_valid", is_valid);
    pt.put("version", OculusHMDConfig::CONFIG_VERSION);

    pt.put("max_poll_failure_count", max_poll_failure_count);

    return pt;
}

void
OculusHMDConfig::ptree2config(const boost::property_tree::ptree &pt)
{
    version = pt.get<int>("version", 0);

    if (version == OculusHMDConfig::CONFIG_VERSION)
    {
        is_valid = pt.get<bool>("is_valid", false);
        max_poll_failure_count = pt.get<long>("max_poll_failure_count", 100);
    }
    else
    {
        SERVER_LOG_WARNING("OculusHMDConfig") <<
            "Config version " << version << " does not match expected version " <<
            OculusHMDConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

// -- PSMove Controller -----
OculusHMD::OculusHMD()
    : cfg()
    , HIDDetails(nullptr)
    , NextPollSequenceNumber(0)
    , InData(nullptr)
    , HMDStates()
{
    HIDDetails = new OculusHIDDetails;
    InData = new OculusDataInput;

    HMDStates.clear();
}

OculusHMD::~OculusHMD()
{
    if (getIsOpen())
    {
        SERVER_LOG_ERROR("~OculusHMD") << "Controller deleted without calling close() first!";
    }

    delete InData;
    delete HIDDetails;
}

bool OculusHMD::open()
{
    HMDDeviceEnumerator enumerator(CommonDeviceState::OculusDK2);
    bool success = false;

    if (enumerator.is_valid())
    {
        success = open(&enumerator);
    }

    return success;
}

bool OculusHMD::open(
    const DeviceEnumerator *enumerator)
{
    const HMDDeviceEnumerator *pEnum = static_cast<const HMDDeviceEnumerator *>(enumerator);

    const char *cur_dev_path = pEnum->get_path();
    bool success = false;

    if (getIsOpen())
    {
        SERVER_LOG_WARNING("OculusHMD::open") << "OculusHMD(" << cur_dev_path << ") already open. Ignoring request.";
        success = true;
    }
    else
    {
        SERVER_LOG_INFO("OculusHMD::open") << "Opening OculusHMD(" << cur_dev_path << ")";
        HIDDetails->Device_path = cur_dev_path;

        char cur_dev_serial_number[256];
        if (pEnum->get_serial_number(cur_dev_serial_number, sizeof(cur_dev_serial_number)))
        {
            SERVER_LOG_INFO("OculusHMD::open") << "  with serial_number: " << cur_dev_serial_number;
        }
        else
        {
            cur_dev_serial_number[0] = '\0';
            SERVER_LOG_INFO("OculusHMD::open") << "  with EMPTY serial_number";
        }
        HIDDetails->Device_serial = cur_dev_serial_number;

#if defined(OVR_OS_WIN32)
        if (ovr_Create(&HIDDetails->SessionHandle, &HIDDetails->Luid) == ovrSuccess)
        {
            SERVER_LOG_INFO("OculusHMD::open") << "Successfully created Oculus tracking session";
            HIDDetails->HmdDesc = ovr_GetHmdDesc(HIDDetails->SessionHandle);

            if (ovr_ConfigureTracking(
                    HIDDetails->SessionHandle,
                    ovrTrackingCap_Orientation | ovrTrackingCap_MagYawCorrection | ovrTrackingCap_Position,
                    0) == ovrSuccess)
            {
                SERVER_LOG_INFO("OculusHMD::open") << "Successfully set orientation and positional tracking";
                success = true;
            }
            else
            {
                SERVER_LOG_ERROR("OculusHMD::open") << "Failed to configure tracking";
            }
        }
#elif defined(OVR_OS_MAC)
        HIDDetails->HmdHandle = ovrHmd_Create(0);
            
        if (HIDDetails->HmdHandle != NULL)
        {
            SERVER_LOG_INFO("OculusHMD::open") << "Successfully created Oculus HMD";

            // A ovrHmd is a pointer to a ovrHmdDesc
            HIDDetails->HmdDesc = *HIDDetails->HmdHandle;

            if (ovr_ConfigureTracking(
                    HIDDetails->HmdHandle,
                    ovrTrackingCap_Orientation | ovrTrackingCap_MagYawCorrection | ovrTrackingCap_Position,
                    0) == ovrSuccess)
            {
                SERVER_LOG_INFO("OculusHMD::open") << "Successfully set orientation and positional tracking";
                success = true;
            }
            else
            {
                SERVER_LOG_ERROR("OculusHMD::open") << "Failed to configure tracking";
            }
        }
#endif

        if (getIsOpen())  // Controller was opened and has an index
        {
            // Reset the polling sequence counter
            NextPollSequenceNumber = 0;
        }
        else
        {
            SERVER_LOG_ERROR("OculusHMD::open") << "Failed to open OculusHMD(" << cur_dev_path << ")";
            success = false;
        }
    }

    return success;
}

void OculusHMD::close()
{
    if (getIsOpen())
    {
        SERVER_LOG_INFO("OculusHMD::close") << "Closing OculusHMD(" << HIDDetails->Device_path << ")";

#if defined(OVR_OS_WIN32)
        if (HIDDetails->SessionHandle != nullptr)
        {
            ovr_Destroy(HIDDetails->SessionHandle);
        }
#elif defined(OVR_OS_MAC)
        if (HIDDetails->HmdHandle != nullptr)
        {
            ovrHmd_Destroy(HIDDetails->HmdHandle);
        }
#endif

        HIDDetails->Reset();
        InData->Reset();
    }
    else
    {
        SERVER_LOG_INFO("OculusHMD::close") << "OculusHMD(" << HIDDetails->Device_path << ") already closed. Ignoring request.";
    }
}

// Getters
bool
OculusHMD::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    // Down-cast the enumerator so we can use the correct get_path.
    const HMDDeviceEnumerator *pEnum = static_cast<const HMDDeviceEnumerator *>(enumerator);

    bool matches = false;

    if (pEnum->get_device_type() == getDeviceType())
    {
        const char *enumerator_path = pEnum->get_path();
        const char *dev_path = HIDDetails->Device_path.c_str();

#ifdef _WIN32
        matches = _stricmp(dev_path, enumerator_path) == 0;
#else
        matches = strcmp(dev_path, enumerator_path) == 0;
#endif
    }

    return matches;
}

bool
OculusHMD::getIsReadyToPoll() const
{
    return (getIsOpen());
}

std::string
OculusHMD::getUSBDevicePath() const
{
    return HIDDetails->Device_path;
}

std::string
OculusHMD::getSerial() const
{
    return HIDDetails->Device_serial;
}

bool
OculusHMD::getIsOpen() const
{
#if defined(OVR_OS_WIN32)
    return (HIDDetails->SessionHandle != nullptr);
#elif defined(OVR_OS_MAC)
    return (HIDDetails->HmdHandle != nullptr);
#else
    return false;
#endif
}

IControllerInterface::ePollResult
OculusHMD::poll()
{
    IControllerInterface::ePollResult result = IControllerInterface::_PollResultFailure;

    if (getIsOpen())
    {
#if defined(OVR_OS_WIN32)
        InData->TrackingState = ovr_GetTrackingState(HIDDetails->SessionHandle, 0.f, ovrFalse);
#elif defined(OVR_OS_MAC)
        InData->TrackingState= ovrHmd_GetTrackingState(HIDDetails->HmdHandle, 0.f);
#else
        InData->Reset();
#endif

        if ((InData->TrackingState.StatusFlags & ovrStatus_HmdConnected) != 0)
        {
            OculusHMDState newState;

            // Increment the sequence for every new polling packet
            newState.PollSequenceNumber = NextPollSequenceNumber;
            ++NextPollSequenceNumber;

            // Copy over the pose
            {
                const ovrPoseStatef &ovrPoseState = InData->TrackingState.HeadPose;
                ovrPosef ovrPose = ovrPoseState.ThePose;

                newState.Pose.Position = ovrVector3f_to_CommonDevicePosition(ovrPose.Position, METERS_TO_CENTIMETERS);
                newState.Pose.Orientation = ovrQuatf_to_CommonDeviceQuaternion(ovrPose.Orientation);

                newState.AngularVelocity = 
                    ovrVector3f_to_CommonDeviceVector(ovrPoseState.AngularVelocity);
                newState.LinearVelocity =
                    ovrVector3f_to_CommonDeviceVector(ovrPoseState.LinearVelocity, METERS_TO_CENTIMETERS); // cm/s
                newState.AngularAcceleration =
                    ovrVector3f_to_CommonDeviceVector(ovrPoseState.AngularAcceleration);
                newState.LinearAcceleration =
                    ovrVector3f_to_CommonDeviceVector(ovrPoseState.LinearAcceleration, METERS_TO_CENTIMETERS); // cm/s^2
                newState.StateTime = InData->TrackingState.HeadPose.TimeInSeconds;
            }

            // Copy over the sensor data
            // TODO: What space is this in?
            newState.Accelerometer = 
                ovrVector3f_to_CommonDeviceVector(InData->TrackingState.RawSensorData.Accelerometer, METERS_TO_CENTIMETERS); // cm/s^2
            newState.Gyro = 
                ovrVector3f_to_CommonDeviceVector(InData->TrackingState.RawSensorData.Gyro); // rad/s
            newState.Magnetometer = 
                ovrVector3f_to_CommonDeviceVector(InData->TrackingState.RawSensorData.Magnetometer); // gauss
            newState.Temperature = InData->TrackingState.RawSensorData.Temperature; // Celcius
            newState.IMUSampleTime = InData->TrackingState.RawSensorData.TimeInSeconds;

            // Make room for new entry if at the max queue size
            if (HMDStates.size() >= OCULUS_HMD_STATE_BUFFER_MAX)
            {
                HMDStates.erase(HMDStates.begin(),
                    HMDStates.begin() + HMDStates.size() - OCULUS_HMD_STATE_BUFFER_MAX);
            }

            HMDStates.push_back(newState);

            result = IHMDInterface::_PollResultSuccessNewData;
        }
        else
        {
            SERVER_LOG_ERROR("OculusHMD::poll") << "HMD disconnected";

            result = IHMDInterface::_PollResultFailure;
        }
    }

    return result;
}

const CommonDeviceState *
OculusHMD::getState(
int lookBack) const
{
    const int queueSize = static_cast<int>(HMDStates.size());
    const CommonDeviceState * result =
        (lookBack < queueSize) ? &HMDStates.at(queueSize - lookBack - 1) : nullptr;

    return result;
}

float
OculusHMD::getTempCelsius() const
{
    const OculusHMDState *lastState = static_cast<const OculusHMDState *>(getState());

    return lastState->Temperature;
}

long OculusHMD::getMaxPollFailureCount() const
{
    return cfg.max_poll_failure_count;
}

// -- private helper functions -----
static CommonDevicePosition ovrVector3f_to_CommonDevicePosition(
    const ovrVector3f &p,
    const float scale)
{
    CommonDevicePosition result;

    result.x = p.x * scale;
    result.y = p.y * scale;
    result.z = p.z * scale;

    return result;
}

static CommonDeviceVector ovrVector3f_to_CommonDeviceVector(
    const ovrVector3f &v,
    const float scale)
{
    CommonDeviceVector result;

    result.i = v.x * scale;
    result.j = v.y * scale;
    result.k = v.z * scale;

    return result;
}

static CommonDeviceQuaternion ovrQuatf_to_CommonDeviceQuaternion(
    const ovrQuatf &q)
{
    CommonDeviceQuaternion result;
    const float length = sqrtf(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);

    result.w = safe_divide_with_default(q.w, length, 1.f);
    result.x = safe_divide_with_default(q.x, length, 0.f);
    result.y = safe_divide_with_default(q.y, length, 0.f);
    result.z = safe_divide_with_default(q.z, length, 0.f);

    return result;
}
//-- includes -----
#include "MorpheusHMD.h"
#include "DeviceInterface.h"
#include "DeviceManager.h"
#include "HMDDeviceEnumerator.h"
#include "MathUtility.h"
#include "ServerLog.h"
#include "ServerUtility.h"
#include <vector>
#include <cstdlib>
#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif
#include <math.h>

//-- constants -----
#define MORPHEUS_HMD_STATE_BUFFER_MAX 4
#define METERS_TO_CENTIMETERS 100

// -- private definitions -----
class MorpheusHIDDetails 
{
public:
    std::string Device_path;

    MorpheusHIDDetails()
    {
        Reset();
    }

    void Reset()
    {
        Device_path = "";
    }
};

class MorpheusDataInput
{
public:
    //ovrTrackingState TrackingState;

    MorpheusDataInput()
    {
        Reset();
    }

    void Reset()
    {
        //memset(&TrackingState, 0, sizeof(ovrTrackingState));
    }
};

// -- public methods

// -- Morpheus HMD Config
const int MorpheusHMDConfig::CONFIG_VERSION = 1;

const boost::property_tree::ptree
MorpheusHMDConfig::config2ptree()
{
    boost::property_tree::ptree pt;

	pt.put("is_valid", is_valid);
	pt.put("version", MorpheusHMDConfig::CONFIG_VERSION);

	pt.put("Calibration.Accel.X.k", accelerometer_gain.i);
	pt.put("Calibration.Accel.Y.k", accelerometer_gain.j);
	pt.put("Calibration.Accel.Z.k", accelerometer_gain.k);
	pt.put("Calibration.Accel.X.b", accelerometer_bias.i);
	pt.put("Calibration.Accel.Y.b", accelerometer_bias.j);
	pt.put("Calibration.Accel.Z.b", accelerometer_bias.k);
	pt.put("Calibration.Accel.NoiseRadius", accelerometer_noise_radius);
	pt.put("Calibration.Gyro.Gain", gyro_gain);
	pt.put("Calibration.Gyro.Variance", gyro_variance);
	pt.put("Calibration.Gyro.Drift", gyro_drift);
	pt.put("Calibration.Identity.Gravity.X", identity_gravity_direction.i);
	pt.put("Calibration.Identity.Gravity.Y", identity_gravity_direction.j);
	pt.put("Calibration.Identity.Gravity.Z", identity_gravity_direction.k);

	pt.put("OrientationFilter.MinQualityScreenArea", min_orientation_quality_screen_area);
	pt.put("OrientationFilter.MaxQualityScreenArea", max_orientation_quality_screen_area);

	pt.put("PositionFilter.MinQualityScreenArea", min_position_quality_screen_area);
	pt.put("PositionFilter.MaxQualityScreenArea", max_position_quality_screen_area);

	pt.put("PositionFilter.MaxVelocity", max_velocity);

	pt.put("prediction_time", prediction_time);
	pt.put("max_poll_failure_count", max_poll_failure_count);

	writeTrackingColor(pt, tracking_color_id);

    return pt;
}

void
MorpheusHMDConfig::ptree2config(const boost::property_tree::ptree &pt)
{
    version = pt.get<int>("version", 0);

    if (version == MorpheusHMDConfig::CONFIG_VERSION)
    {
		is_valid = pt.get<bool>("is_valid", false);
		prediction_time = pt.get<float>("prediction_time", 0.f);
		max_poll_failure_count = pt.get<long>("max_poll_failure_count", 100);

		// Use the current accelerometer values (constructor defaults) as the default values
		accelerometer_gain.i = pt.get<float>("Calibration.Accel.X.k", accelerometer_gain.i);
		accelerometer_gain.j = pt.get<float>("Calibration.Accel.Y.k", accelerometer_gain.j);
		accelerometer_gain.k = pt.get<float>("Calibration.Accel.Z.k", accelerometer_gain.k);
		accelerometer_bias.i = pt.get<float>("Calibration.Accel.X.b", accelerometer_bias.i);
		accelerometer_bias.j = pt.get<float>("Calibration.Accel.Y.b", accelerometer_bias.j);
		accelerometer_bias.k = pt.get<float>("Calibration.Accel.Z.b", accelerometer_bias.k);
		accelerometer_noise_radius = pt.get<float>("Calibration.Accel.NoiseRadius", 0.0f);

		// Use the current gyroscope values (constructor defaults) as the default values
		gyro_gain = pt.get<float>("Calibration.Gyro.Gain", gyro_gain);
		gyro_variance = pt.get<float>("Calibration.Gyro.Variance", gyro_variance);
		gyro_drift = pt.get<float>("Calibration.Gyro.Drift", gyro_drift);

		// Get the orientation filter parameters
		min_orientation_quality_screen_area = pt.get<float>("OrientationFilter.MinQualityScreenArea", min_orientation_quality_screen_area);
		max_orientation_quality_screen_area = pt.get<float>("OrientationFilter.MaxQualityScreenArea", max_orientation_quality_screen_area);

		// Get the position filter parameters
		min_position_quality_screen_area = pt.get<float>("PositionFilter.MinQualityScreenArea", min_position_quality_screen_area);
		max_position_quality_screen_area = pt.get<float>("PositionFilter.MaxQualityScreenArea", max_position_quality_screen_area);
		max_velocity = pt.get<float>("PositionFilter.MaxVelocity", max_velocity);

		// Get the calibration direction for "down"
		identity_gravity_direction.i = pt.get<float>("Calibration.Identity.Gravity.X", identity_gravity_direction.i);
		identity_gravity_direction.j = pt.get<float>("Calibration.Identity.Gravity.Y", identity_gravity_direction.j);
		identity_gravity_direction.k = pt.get<float>("Calibration.Identity.Gravity.Z", identity_gravity_direction.k);

		// Read the tracking color
		tracking_color_id = static_cast<eCommonTrackingColorID>(readTrackingColor(pt));
    }
    else
    {
        SERVER_LOG_WARNING("MorpheusHMDConfig") <<
            "Config version " << version << " does not match expected version " <<
            MorpheusHMDConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

// -- Morpheus HMD -----
MorpheusHMD::MorpheusHMD()
    : cfg()
    , HIDDetails(nullptr)
    , NextPollSequenceNumber(0)
    , InData(nullptr)
    , HMDStates()
{
    HIDDetails = new MorpheusHIDDetails;
    InData = new MorpheusDataInput;

    HMDStates.clear();
}

MorpheusHMD::~MorpheusHMD()
{
    if (getIsOpen())
    {
        SERVER_LOG_ERROR("~MorpheusHMD") << "HMD deleted without calling close() first!";
    }

    delete InData;
    delete HIDDetails;
}

bool MorpheusHMD::open()
{
    HMDDeviceEnumerator enumerator(CommonDeviceState::Morpheus);
    bool success = false;

    if (enumerator.is_valid())
    {
        success = open(&enumerator);
    }

    return success;
}

bool MorpheusHMD::open(
    const DeviceEnumerator *enumerator)
{
    const HMDDeviceEnumerator *pEnum = static_cast<const HMDDeviceEnumerator *>(enumerator);

    const char *cur_dev_path = pEnum->get_path();
    bool success = false;

    if (getIsOpen())
    {
        SERVER_LOG_WARNING("MorpheusHMD::open") << "MorpheusHMD(" << cur_dev_path << ") already open. Ignoring request.";
        success = true;
    }
    else
    {
        SERVER_LOG_INFO("MorpheusHMD::open") << "Opening MorpheusHMD(" << cur_dev_path << ")";
        HIDDetails->Device_path = cur_dev_path;

        // TODO

        if (getIsOpen())  // Controller was opened and has an index
        {
            // Reset the polling sequence counter
            NextPollSequenceNumber = 0;
        }
        else
        {
            SERVER_LOG_ERROR("MorpheusHMD::open") << "Failed to open MorpheusHMD(" << cur_dev_path << ")";
            success = false;
        }
    }

    return success;
}

void MorpheusHMD::close()
{
    if (getIsOpen())
    {
        SERVER_LOG_INFO("MorpheusHMD::close") << "Closing MorpheusHMD(" << HIDDetails->Device_path << ")";

        HIDDetails->Reset();
        InData->Reset();
    }
    else
    {
        SERVER_LOG_INFO("MorpheusHMD::close") << "MorpheusHMD(" << HIDDetails->Device_path << ") already closed. Ignoring request.";
    }
}

// Getters
bool
MorpheusHMD::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
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
MorpheusHMD::getIsReadyToPoll() const
{
    return (getIsOpen());
}

std::string
MorpheusHMD::getUSBDevicePath() const
{
    return HIDDetails->Device_path;
}

bool
MorpheusHMD::getIsOpen() const
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
MorpheusHMD::poll()
{
    IControllerInterface::ePollResult result = IControllerInterface::_PollResultFailure;

    if (getIsOpen())
    {        
        InData->Reset();

        //TODO: Read state
        /*
        if ((InData->TrackingState.StatusFlags & ovrStatus_HmdConnected) != 0)
        {
            MorpheusHMDState newState;

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
            SERVER_LOG_ERROR("MorpheusHMD::poll") << "HMD disconnected";

            result = IHMDInterface::_PollResultFailure;
        }
        */
    }

    return result;
}

void
MorpheusHMD::getTrackingShape(CommonDeviceTrackingShape &outTrackingShape) const
{
	outTrackingShape.shape_type = eCommonTrackingShapeType::PointCloud;
	//###HipsterSloth $TODO - Fill in points
	outTrackingShape.shape.point_cloud.point[0].set(0.f, 0.f, 0.f);
	outTrackingShape.shape.point_cloud.point[1].set(0.f, 0.f, 0.f);
	outTrackingShape.shape.point_cloud.point[2].set(0.f, 0.f, 0.f);
	outTrackingShape.shape.point_cloud.point[3].set(0.f, 0.f, 0.f);
	outTrackingShape.shape.point_cloud.point[4].set(0.f, 0.f, 0.f);
	outTrackingShape.shape.point_cloud.point[5].set(0.f, 0.f, 0.f);
	outTrackingShape.shape.point_cloud.point[6].set(0.f, 0.f, 0.f);
}

bool 
MorpheusHMD::getTrackingColorID(eCommonTrackingColorID &out_tracking_color_id) const
{
	out_tracking_color_id = eCommonTrackingColorID::Blue;
	return true;
}

const CommonDeviceState *
MorpheusHMD::getState(
    int lookBack) const
{
    const int queueSize = static_cast<int>(HMDStates.size());
    const CommonDeviceState * result =
        (lookBack < queueSize) ? &HMDStates.at(queueSize - lookBack - 1) : nullptr;

    return result;
}

long MorpheusHMD::getMaxPollFailureCount() const
{
    return cfg.max_poll_failure_count;
}

// -- private helper functions -----
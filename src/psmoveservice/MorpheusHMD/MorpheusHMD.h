#ifndef MORPHEUS_HMD_H
#define MORPHEUS_HMD_H

#include "PSMoveConfig.h"
#include "DeviceEnumerator.h"
#include "DeviceInterface.h"
#include "MathUtility.h"
#include <string>
#include <vector>
#include <deque>
#include <array>

// The angle the accelerometer reading will be pitched by
// if the Morpheus is held such that the face plate is perpendicular to the ground
// i.e. where what we consider the "identity" pose
#define ACCELEROMETER_IDENTITY_PITCH_DEGREES 0.0f

class MorpheusHMDConfig : public PSMoveConfig
{
public:
    static const int CONFIG_VERSION;

    MorpheusHMDConfig(const std::string &fnamebase = "MorpheusHMDConfig")
        : PSMoveConfig(fnamebase)
		, is_valid(false)
		, version(CONFIG_VERSION)
		, accelerometer_noise_radius(0.f)
		, max_velocity(1.f)
		, gyro_gain(0.f)
		, gyro_variance(0.f)
		, gyro_drift(0.f)
		, max_poll_failure_count(100)
		, prediction_time(0.f)
		, min_orientation_quality_screen_area(150.f*34.f*.1f)
		, max_orientation_quality_screen_area(150.f*34.f) // light bar at ideal range looking straight on is about 150px by 34px 
		, min_position_quality_screen_area(75.f*17.f*.25f)
		, max_position_quality_screen_area(75.f*17.f)
		, tracking_color_id(eCommonTrackingColorID::Blue)
    {
		// The Morpheus I suspect uses the BMI160 IMU Chip: 
		// https://d3nevzfk7ii3be.cloudfront.net/igi/hnlrYUv5BUb6lMoW.huge
		// http://www.mouser.com/ds/2/783/BST-BMI160-DS000-07-786474.pdf
		//
		// The Accelerometer can operate in one of 4 modes: 
		//   ±2g, ±4g, ±8g, ±16g
		// The Gyroscope can operate in one of 5 modes: 
		//   ±125°/s, ±250°/s, ±500°/s, ±1000°/s, ±2000°/s
		//   (or ±2.18 rad/s, ±4.36 rad/s, ±8.72 rad/s, ±17.45 rad/s, ±34.9 rad/s)
		//
		// I haven't seen any indication that suggests the Morpheus changes modes.
		// It also appears that the raw accelerometer and gyroscope values are pre-calibrated
		// (there is no sensor calibration report with biases and gains that I can find)

		// Accelerometer gain computed from accelerometer calibration in the config tool is really close to 1/8192.
		// and is just in a 2.13 fixed point value (+1 sign bit)
		// This agrees with the stack exchange article
		accelerometer_gain.i = 1.f / 8192.f;
		accelerometer_gain.j = 1.f / 8192.f;
		accelerometer_gain.k = 1.f / 8192.f;

		// Accelerometer bias computed from accelerometer calibration in the config tool is really close to 0
		// This is because the raw gyro readings are likely pre-calibrated
		accelerometer_bias.i = 0.f;
		accelerometer_bias.j = 0.f;
		accelerometer_bias.k = 0.f;

		// Empirical testing of the of the gyro gain looks best at 1/2048.
		// This implies that gyroscope is returned from the controller is pre-calibrated 
		// and is just in a 4.11 fixed point value (+1 sign bit).
		// This is twice what the stack exchange article recommends.
		gyro_gain = 1.f / 2048.f;

		// This is the variance of the calibrated gyro value recorded for 100 samples
		// Units in rad/s^2
		gyro_variance = 1.33875039e-006f;

		// This is the drift of the raw gyro value recorded for 60 seconds
		// Units rad/s
		gyro_drift = 0.00110168592f;

		// This is the ideal accelerometer reading you get when the DS4 is held such that 
		// the light bar facing is perpendicular to gravity.        
		identity_gravity_direction.i = 0.f;
		identity_gravity_direction.j = cosf(ACCELEROMETER_IDENTITY_PITCH_DEGREES*k_degrees_to_radians);
		identity_gravity_direction.k = -sinf(ACCELEROMETER_IDENTITY_PITCH_DEGREES*k_degrees_to_radians);
    };

    virtual const boost::property_tree::ptree config2ptree();
    virtual void ptree2config(const boost::property_tree::ptree &pt);

    bool is_valid;
    long version;

	// calibrated_acc= raw_acc*acc_gain + acc_bias
	CommonDeviceVector accelerometer_gain;
	CommonDeviceVector accelerometer_bias;
	float accelerometer_noise_radius;

	// Maximum velocity for the controller physics (meters/second)
	float max_velocity;

	// The calibrated "down" direction
	CommonDeviceVector identity_gravity_direction;

	// calibrated_gyro= raw_gyro*gyro_gain
	float gyro_gain;
	// The variance of the raw gyro readings in rad/sec^2
	float gyro_variance;
	// The drift raw gyro readings in rad/second
	float gyro_drift;

	// The pixel area of the tracking projection at which the orientation quality is 0
	float min_orientation_quality_screen_area;
	// The pixel area of the tracking projection at which the orientation quality is 1
	float max_orientation_quality_screen_area;

	// The pixel area of the tracking projection at which the position quality is 0
	float min_position_quality_screen_area;
	// The pixel area of the tracking projection at which the position quality is 1
	float max_position_quality_screen_area;

    long max_poll_failure_count;
	float prediction_time;

	eCommonTrackingColorID tracking_color_id;
};

struct MorpheusHMDState : public CommonHMDState
{
    CommonDeviceVector AngularVelocity;        /// The body's angular velocity in radians per second.
    CommonDeviceVector LinearVelocity;         /// The body's velocity in meters per second.
    CommonDeviceVector AngularAcceleration;    /// The body's angular acceleration in radians per second per second.
    CommonDeviceVector LinearAcceleration;     /// The body's acceleration in meters per second per second.

	std::array< std::array<int, 3>, 2> RawAccel;    // Two frames of 3 dimensions
	std::array< std::array<int, 3>, 2> RawGyro;     // Two frames of 3 dimensions

	std::array< std::array<float, 3>, 2> CalibratedAccel;    // Two frames of 3 dimensions
	std::array< std::array<float, 3>, 2> CalibratedGyro;     // Two frames of 3 dimensions

    MorpheusHMDState()
    {
        clear();
    }

    void clear()
    {
        CommonHMDState::clear();

        DeviceType = Morpheus;
    }
};

class MorpheusHMD : public IHMDInterface 
{
public:
    MorpheusHMD();
    ~MorpheusHMD();

    // MorpheusHMD
    bool open(); // Opens the first HID device for the controller

    // -- IDeviceInterface
    bool matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const override;
    bool open(const DeviceEnumerator *enumerator) override;
    bool getIsOpen() const override;
    bool getIsReadyToPoll() const override;
    IDeviceInterface::ePollResult poll() override;
    void close() override;
    long getMaxPollFailureCount() const override;
    CommonDeviceState::eDeviceType getDeviceType() const override
    {
        return CommonDeviceState::Morpheus;
    }
    static CommonDeviceState::eDeviceType getDeviceTypeStatic()
    {
        return CommonDeviceState::Morpheus;
    }
    const CommonDeviceState * getState(int lookBack = 0) const override;

    // -- IHMDInterface
    std::string getUSBDevicePath() const override;
	void getTrackingShape(CommonDeviceTrackingShape &outTrackingShape) const override;
	bool getTrackingColorID(eCommonTrackingColorID &out_tracking_color_id) const override;

    // -- Getters
    inline const MorpheusHMDConfig *getConfig() const
    {
        return &cfg;
    }
    inline MorpheusHMDConfig *getConfigMutable()
    {
        return &cfg;
    }

    // -- Setters

private:
    // Constant while the HMD is open
    MorpheusHMDConfig cfg;
    class MorpheusHIDDetails *HIDDetails;                    // Buffer that holds static MorpheusAPI HMD description

    // Read HMD State
    int NextPollSequenceNumber;
    class MorpheusDataInput *InData;                        // Buffer to hold most recent MorpheusAPI tracking state
    std::deque<MorpheusHMDState> HMDStates;
};

#endif // MORPHEUS_HMD_H
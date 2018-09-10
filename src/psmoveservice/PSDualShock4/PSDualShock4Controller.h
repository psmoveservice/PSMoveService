#ifndef PSDUALSHOCK4_CONTROLLER_H
#define PSDUALSHOCK4_CONTROLLER_H

#include "PSMoveConfig.h"
#include "DeviceEnumerator.h"
#include "DeviceInterface.h"
#include "MathUtility.h"
#include "hidapi.h"
#include <string>
#include <vector>
#include <chrono>

// The angle the accelerometer reading is pitched forward when the DS4 is on a flat surface
// The value comes from the accelerometer calibration utility
#define FLAT_SURFACE_ACCELEROMETER_PITCH_DEGREES 12.661f

// The angle between the lightbar and a flat surface it rests on
#define FLAT_SURFACE_LIGHTBAR_PITCH_DEGREES 51.8f

// The angle the accelerometer reading will be pitched by
// if the DS4 is held such that the light bar is perpendicular to the ground
// i.e. where what we consider the "identity" pose
#define ACCELEROMETER_IDENTITY_PITCH_DEGREES 22.667f

struct DualShock4HIDDetails {
	int vendor_id;
	int product_id;
    std::string Device_path;
    hid_device *Handle;
    std::string Bt_addr;      // The bluetooth address of the controller
    std::string Host_bt_addr; // The bluetooth address of the adapter registered with the controller
};

struct DualShock4DataInput;   // See .cpp for declaration
struct DualShock4DataOutput;  // See .cpp for declaration

class PSDualShock4ControllerConfig : public PSMoveConfig
{
public:
    static const int CONFIG_VERSION;

    PSDualShock4ControllerConfig(const std::string &fnamebase = "PSDualShock4ControllerConfig")
        : PSMoveConfig(fnamebase)
        , is_valid(false)
        , version(CONFIG_VERSION)
		, position_filter_type("ComplimentaryOpticalIMU")
		, orientation_filter_type("ComplementaryOpticalARG")
        , max_poll_failure_count(100)
        , prediction_time(0.f)
        , accelerometer_noise_radius(0.015f) // rounded value from config tool measurement (g-units)
		, accelerometer_variance(1.45e-05f) // rounded value from config tool measurement (g-units^2)
        , max_velocity(1.f)
        , gyro_variance(4.75e-06f) // rounded value from config tool measurement (rad^2/s^2)
        , gyro_drift(0.00071f) // rounded value from config tool measurement (rad/s)
		, mean_update_time_delta(0.016667f)
        , min_screen_projection_area(100.f)
		, position_use_linear_acceleration(false)
		, position_apply_gravity_mask(false)
		, position_variance_exp_fit_a(0.0219580978f)
		, position_variance_exp_fit_b(-0.00079152541f)
		, orientation_variance_exp_fit_a(0.119878575f)
		, orientation_variance_exp_fit_b(-0.00267515215f)
		, tracking_color_id(eCommonTrackingColorID::INVALID_COLOR)
		, hand("Any")
    {
        // The DS4 uses the BMI055 IMU Chip: 
        // https://www.bosch-sensortec.com/bst/products/all_products/bmi055
        //
        // The Accelerometer can operate in one of 4 modes: 
        //   ±2g, ±4g, ±8g, ±16g
		// The raw sensor bits can be converted to units of 'g' by dividing with the following "Sensitivity" values
		//   1024, 512, 256, 128 [units=LSB/g]

        // The Gyroscope can operate in one of 5 modes: 
        //   ±125°/s, ±250°/s, ±500°/s, ±1000°/s, ±2000°/s
        //   (or ±2.18 rad/s, ±4.36 rad/s, ±8.72 rad/s, ±17.45 rad/s, ±34.9 rad/s)
		// The raw sensor bits can be converted to units of '°/s' by dividing with following "Sensitivity" values
		//   262.4, 131.2, 65.6, 32.8, 16.4 [units=LSB/°/s]
		// or converted to units of 'rad/s' with these "Sensitivity" values
		//   15034.4, 7517.2, 3758.6, 1879.3, 939.7 [units=LSB/rad/s]
        
        // Accelerometer gain computed from accelerometer calibration in the config tool is really close to 1/8192.
        // Since the accelerometer is 12-bit we have to >> 4 bits (divide by 16) to get the true raw sensor bits
		// That means the "Sensitivity" is 512 and thus the accelerometer mode is ±4g
		accelerometer_gain.i = 1.f / 512.f;
		accelerometer_gain.j = 1.f / 512.f;
		accelerometer_gain.k = 1.f / 512.f;
        
        // Accelerometer bias computed from accelerometer calibration in the config tool is really close to 0
        // This is because the raw gyro readings are likely pre-calibrated
        accelerometer_bias.i = 0.f;
        accelerometer_bias.j = 0.f;
        accelerometer_bias.k = 0.f;

		// Gyro gain mode can vary from controller to controller
		// Initially assume that this controller is using the '±2000°/s' mode 
		// and use the appropriate LSB/rad/s "Sensitivity"
        gyro_gain= 1.f / (16.4f / k_degrees_to_radians);

        // This is the ideal accelerometer reading you get when the DS4 is held such that 
        // the light bar facing is perpendicular to gravity.        
        identity_gravity_direction.i= 0.f;
        identity_gravity_direction.j= cosf(ACCELEROMETER_IDENTITY_PITCH_DEGREES*k_degrees_to_radians);
        identity_gravity_direction.k= -sinf(ACCELEROMETER_IDENTITY_PITCH_DEGREES*k_degrees_to_radians);
    };

    virtual const boost::property_tree::ptree config2ptree();
    virtual void ptree2config(const boost::property_tree::ptree &pt);

    bool is_valid;
    long version;

	// The type of position filter to use
	std::string position_filter_type;

	// The type of orientation filter to use
	std::string orientation_filter_type;

	// The max number of polling failures before we consider the controller disconnected
    long max_poll_failure_count;
	// The amount of prediction to apply to the controller pose after filtering
    float prediction_time;

    // calibrated_acc= raw_acc*acc_gain + acc_bias
    CommonDeviceVector accelerometer_gain;
    CommonDeviceVector accelerometer_bias;

	// The bounding radius of the accelerometer measurements 
    float accelerometer_noise_radius;

	// The variance of the accelerometer readings
	float accelerometer_variance; // g-units^2

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

	// The average time between updates in seconds
    float mean_update_time_delta;

	// Below this projection area size we just consider the projection area 0 for the purpose of filtering
	float min_screen_projection_area;

	// When enabled attempt to extract linear acceleration from the accelerometer (Kalman filter only)
	bool position_use_linear_acceleration;

	// Dampen linear accelerations near the predicted gravity measurement (Kalman filter only)
	bool position_apply_gravity_mask;

	// The variance(cm^2) of the controller position as a function of projection area
    float position_variance_exp_fit_a; 
    float position_variance_exp_fit_b;

	// The variance of the controller orientation as a function of projection area
    float orientation_variance_exp_fit_a;
	float orientation_variance_exp_fit_b;

	inline float get_position_variance(float projection_area) const {
		return position_variance_exp_fit_a*exp(position_variance_exp_fit_b*projection_area);
	}

	inline float get_orientation_variance(float projection_area) const {
		return orientation_variance_exp_fit_a*exp(orientation_variance_exp_fit_b*projection_area);
	}

	eCommonTrackingColorID tracking_color_id;

	// The assigned hand for this controller
	std::string hand;
};

struct DualShock4ControllerInputState : public CommonControllerState
{
    int RawSequence;                               // 6-bit  (counts up by 1 per report)

    unsigned int RawTimeStamp;                     // 16-bit (time since ?, units?)

    float LeftAnalogX;  // [-1.f, 1.f]
    float LeftAnalogY;  // [-1.f, 1.f]
    float RightAnalogX;  // [-1.f, 1.f]
    float RightAnalogY;  // [-1.f, 1.f]
    float LeftTrigger;  // [0.f, 1.f]
    float RightTrigger;  // [0.f, 1.f]

    ButtonState DPad_Up;
    ButtonState DPad_Down;
    ButtonState DPad_Left;
    ButtonState DPad_Right;

    ButtonState Square;
    ButtonState Cross;
    ButtonState Circle;
    ButtonState Triangle;

    ButtonState L1;
    ButtonState R1;
    ButtonState L2;
    ButtonState R2;
    ButtonState L3;
    ButtonState R3;

    ButtonState Share;
    ButtonState Options;

    ButtonState PS;
    ButtonState TrackPadButton;

    int RawAccelerometer[3]; // Raw 12-bit Accelerometer Value
    int RawGyro[3]; // Raw 16-bit Gyroscope Value

    CommonDeviceVector CalibratedAccelerometer; // units of g (where 1g = 9.8m/s^2)
    CommonDeviceVector CalibratedGyro; // rad/s

    DualShock4ControllerInputState();

    void clear();
	void parseDataInput(
		const PSDualShock4ControllerConfig *config, 
		const struct DualShock4DataInput *previous_hid_packet,
		const struct DualShock4DataInput *new_hid_packet);
};

struct DualShock4ControllerOutputState 
{
    unsigned char r;        // red value, 0x00..0xff
    unsigned char g;        // green value, 0x00..0xff
    unsigned char b;        // blue value, 0x00..0xff

    unsigned char rumble_left;   // rumble value, 0x00..0xff
	unsigned char rumble_right;  // rumble value, 0x00..0xff

	DualShock4ControllerOutputState();

	void clear();
};

class PSDualShock4Controller : public IControllerInterface {
public:
    PSDualShock4Controller();
    virtual ~PSDualShock4Controller();

    // PSMoveController
    bool open(); // Opens the first HID device for the controller

    // -- IDeviceInterface
    virtual bool matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const override;
    virtual bool open(const DeviceEnumerator *enumerator) override;
    virtual bool getIsOpen() const override;
    virtual bool getIsReadyToPoll() const override;
    virtual IDeviceInterface::ePollResult poll() override;
    virtual void close() override;
	virtual long getMaxPollFailureCount() const override;
    CommonDeviceState::eDeviceType getDeviceType() const override
    {
        return CommonDeviceState::PSDualShock4;
    }
    static CommonDeviceState::eDeviceType getDeviceTypeStatic()
    {
        return CommonDeviceState::PSDualShock4;
    }
	virtual const CommonDeviceState * getState(int lookBack = 0) const override;

    // -- IControllerInterface
    virtual bool setHostBluetoothAddress(const std::string &address) override;
	virtual bool setTrackingColorID(const eCommonTrackingColorID tracking_color_id) override;
    virtual bool getIsBluetooth() const override;
    virtual std::string getUSBDevicePath() const override;
	virtual int getVendorID() const override;
	virtual int getProductID() const override;
    virtual std::string getAssignedHostBluetoothAddress() const override;
    virtual std::string getSerial() const override;
    virtual const std::tuple<unsigned char, unsigned char, unsigned char> getColour() const override;
    virtual void getTrackingShape(CommonDeviceTrackingShape &outTrackingShape) const override;
	virtual bool getTrackingColorID(eCommonTrackingColorID &out_tracking_color_id) const override;
	virtual float getIdentityForwardDegrees() const override;
	virtual float getPredictionTime() const override;
    virtual bool getWasSystemButtonPressed() const override;

    // -- Getters
    inline const PSDualShock4ControllerConfig *getConfig() const
    {
        return &cfg;
    }
    inline PSDualShock4ControllerConfig *getConfigMutable()
    {
        return &cfg;
    }

    // -- Setters
    bool setLED(unsigned char r, unsigned char g, unsigned char b);
    bool setLeftRumbleIntensity(unsigned char value);
    bool setRightRumbleIntensity(unsigned char value);
	void setControllerListener(IControllerListener *listener) override;

private:
    bool getBTAddressesViaUSB(std::string& host, std::string& controller);

    // Constant while a controller is open
    PSDualShock4ControllerConfig cfg;
    DualShock4HIDDetails HIDDetails;
    bool IsBluetooth;                               // true if valid serial number on device opening

    // Cached MainThread Controller State
	DualShock4ControllerInputState m_cachedInputState;
    DualShock4ControllerOutputState m_cachedOutputState;

    // HID Packet Processing
	class DualShock4HidPacketProcessor* m_HIDPacketProcessor;
	IControllerListener* m_controllerListener;

};
#endif // PSDUALSHOCK4_CONTROLLER_H

#ifndef PSMOVE_CONTROLLER_H
#define PSMOVE_CONTROLLER_H

#include "PSMoveConfig.h"
#include "DeviceEnumerator.h"
#include "DeviceInterface.h"
#include "MathUtility.h"
#include "hidapi.h"
#include <string>
#include <array>
#include <deque>
#include <chrono>

struct PSMoveHIDDetails {
	int vendor_id;
	int product_id;
    std::string Device_path;
    hid_device *Handle;
    std::string Device_path_addr; // only needed by Win > 8.1, otherwise ignored.
    hid_device *Handle_addr; // only needed by Win > 8.1, otherwise ignored.
    std::string Bt_addr;      // The bluetooth address of the controller
    std::string Host_bt_addr; // The bluetooth address of the adapter registered with the controller
};

struct PSMoveDataInput;  // See .cpp for full declaration

class PSMoveControllerConfig : public PSMoveConfig
{
public:
    static const int CONFIG_VERSION;

    PSMoveControllerConfig(const std::string &fnamebase = "PSMoveControllerConfig")
        : PSMoveConfig(fnamebase)
        , is_valid(false)
        , version(CONFIG_VERSION)
		, firmware_version(0)
		, bt_firmware_version(0)
		, firmware_revision(0)
        , max_poll_failure_count(100) 
        , prediction_time(0.f)
		, position_filter_type("LowPassExponential")
		, orientation_filter_type("ComplementaryMARG")
        , cal_ag_xyz_kb({{ 
            {{ {{0, 0}}, {{0, 0}}, {{0, 0}} }},
            {{ {{0, 0}}, {{0, 0}}, {{0, 0}} }} 
        }})
        , magnetometer_fit_error(0.f)
		, magnetometer_variance(0.00059f) // rounded value from config tool measurement
		, accelerometer_variance(7.2e-06f) // rounded value from config tool measurement
        , accelerometer_noise_radius(0.014f) // rounded value from config tool measurement
        , gyro_variance(0.00035f) // rounded value from config tool measurement (rad/s)^2
        , gyro_drift(0.027f) // rounded value from config tool measurement (rad/s)
        , max_velocity(1.f)
		, mean_update_time_delta(0.008333f)
		, position_variance_exp_fit_a(0.0994158462f)
		, position_variance_exp_fit_b(-0.000567041978f)
		, orientation_variance(18.75f)
		, tracking_color_id(eCommonTrackingColorID::INVALID_COLOR)
    {
        magnetometer_identity.clear();
        magnetometer_center.clear();
        magnetometer_basis_x.clear();
        magnetometer_basis_y.clear();
        magnetometer_basis_z.clear();
        magnetometer_extents.clear();
        magnetometer_identity.clear();
    };

    virtual const boost::property_tree::ptree config2ptree();
    virtual void ptree2config(const boost::property_tree::ptree &pt);

    void getMegnetometerEllipsoid(struct EigenFitEllipsoid *out_ellipsoid);

    bool is_valid;
    long version;

	// Move's firmware version number
	unsigned short firmware_version;

	// Move Bluetooth module's firmware version number
	unsigned short bt_firmware_version; 

	// Move's firmware revision number
	unsigned short firmware_revision;


	// The max number of polling failures before we consider the controller disconnected
    long max_poll_failure_count;

	// The amount of prediction to apply to the controller pose after filtering
    float prediction_time;

	// The type of position filter to use
	std::string position_filter_type;

	// The type of orientation filter to use
	std::string orientation_filter_type;

	// The accelerometer and gyroscope scale and bias values read from the USB calibration packet
    std::array<std::array<std::array<float, 2>, 3>, 2> cal_ag_xyz_kb;

	// The direction of the magnetometer when in the identity pose
    CommonDeviceVector magnetometer_identity;

	// The bias of the magnetometer readings
    CommonDeviceVector magnetometer_center;

	// The basis vectors of the best fit ellipsoid (typically i,j,k)
    CommonDeviceVector magnetometer_basis_x;
    CommonDeviceVector magnetometer_basis_y;
    CommonDeviceVector magnetometer_basis_z;
    CommonDeviceVector magnetometer_extents;

	// The squared error of the magnetometer fit ellipsoid
    float magnetometer_fit_error;

	/// The variance of the magnetometer sensor
	float magnetometer_variance; // units^2

	// The variance of the accelerometer readings
	float accelerometer_variance; // g-units^2

    // The bounding radius of the accelerometer noise
    float accelerometer_noise_radius;
    
    // The variance of the calibrated gyro readings in (rad/s)^2
    float gyro_variance;

    // The drift of the calibrated gyro readings in rad/s
    float gyro_drift;

    // The maximum velocity allowed in the position filter in cm/s
    float max_velocity;

	// The average time between updates in seconds
    float mean_update_time_delta;

	// The variance(cm^2) of the controller position (meters^2) as a function of pixel area
    float position_variance_exp_fit_a; 
    float position_variance_exp_fit_b;

	// The variance of the controller orientation (when sitting still) in rad^2
    float orientation_variance;

	inline float get_position_variance(float projection_area) const {
		return position_variance_exp_fit_a*exp(position_variance_exp_fit_b*projection_area);
	}

	// The assigned tracking color for this controller
	eCommonTrackingColorID tracking_color_id;
};

// https://code.google.com/p/moveonpc/wiki/InputReport
struct PSMoveControllerState : public CommonControllerState
{
    int RawSequence;                               // 4-bit (1..16).
                                                // Sometimes frames are dropped.
    
    unsigned int RawTimeStamp;                     // 16-bit (time since ?, units?)
                                                // About 1150 between in-order frames.

    ButtonState Triangle;
    ButtonState Circle;
    ButtonState Cross;
    ButtonState Square;
    ButtonState Select;
    ButtonState Start;
    ButtonState PS;
    ButtonState Move;
    ButtonState Trigger;

    unsigned char TriggerValue;  // 0-255. Average of last two frames.

    std::array< std::array<int, 3>, 2> RawAccel;    // Two frames of 3 dimensions
    std::array< std::array<int, 3>, 2> RawGyro;     // Two frames of 3 dimensions
    std::array<int, 3> RawMag;                      // One frame of 3 dimensions

    std::array< std::array<float, 3>, 2> CalibratedAccel;    // Two frames of 3 dimensions
    std::array< std::array<float, 3>, 2> CalibratedGyro;     // Two frames of 3 dimensions
    std::array<float, 3> CalibratedMag;                       // One frame of 3 dimensions

    int TempRaw;

    //TODO: high-precision timestamp. Need to do in hidapi?
    
    PSMoveControllerState()
    {
        clear();
    }

    void clear()
    {
        CommonControllerState::clear();

        RawSequence = 0;
        RawTimeStamp = 0;

        DeviceType = PSMove;

        Triangle = Button_UP;
        Circle = Button_UP;
        Cross = Button_UP;
        Square = Button_UP;
        Select = Button_UP;
        Start = Button_UP;
        PS = Button_UP;
        Move = Button_UP;
        Trigger = Button_UP;

        TriggerValue= 0;

        CalibratedAccel = {{
            {{0, 0, 0}}, 
            {{0, 0, 0}} 
        }};
        CalibratedGyro = {{
            {{0, 0, 0}}, 
            {{0, 0, 0}}
        }};
        CalibratedMag = {{0, 0, 0}};

        TempRaw= 0;
    }
};

class PSMoveController : public IControllerInterface {
public:
    PSMoveController();
    virtual ~PSMoveController();

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
    virtual CommonDeviceState::eDeviceType getDeviceType() const override;
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

    // -- Getters
    inline const PSMoveControllerConfig *getConfig() const
    { return &cfg; }
    inline PSMoveControllerConfig *getConfigMutable()
    { return &cfg; }
    float getTempCelsius() const;
    static CommonDeviceState::eDeviceType getDeviceTypeStatic()
    { return CommonDeviceState::PSMove; }
	bool getSupportsMagnetometer() const
	{ return SupportsMagnetometer; }        
    const unsigned long getLEDPWMFrequency() const
    { return LedPWMF; }
    
    // -- Setters
    bool setLED(unsigned char r, unsigned char g, unsigned char b); // 0x00..0xff. TODO: vec3
    bool setLEDPWMFrequency(unsigned long freq);    // 733..24e6
    bool setRumbleIntensity(unsigned char value);
	bool enableDFUMode(); // Device Firmware Update mode

private:    
    bool getBTAddress(std::string& host, std::string& controller);
    void loadCalibration();                         // Use USB or file if on BT
	bool loadFirmwareInfo();
    
    bool writeDataOut();                            // Setters will call this
    
    // Constant while a controller is open
    PSMoveControllerConfig cfg;
    PSMoveHIDDetails HIDDetails;
    bool IsBluetooth;                               // true if valid serial number on device opening
	bool SupportsMagnetometer;                      // true if controller emits valid magnetometer data

    // Cached Setter State
    unsigned char LedR, LedG, LedB;
    unsigned char Rumble;
    unsigned long LedPWMF;
    bool bWriteStateDirty;
    std::chrono::time_point<std::chrono::high_resolution_clock> lastWriteStateTime;

    // Read Controller State
    int NextPollSequenceNumber;
    std::deque<PSMoveControllerState> ControllerStates;
    PSMoveDataInput* InData;                        // Buffer to copy hidapi reports into
};
#endif // PSMOVE_CONTROLLER_H
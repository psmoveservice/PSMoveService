#ifndef PSDUALSHOCK4_CONTROLLER_H
#define PSDUALSHOCK4_CONTROLLER_H

#include "PSMoveConfig.h"
#include "DeviceEnumerator.h"
#include "DeviceInterface.h"
#include "MathUtility.h"
#include "hidapi.h"
#include <string>
#include <vector>
#include <deque>
#include <chrono>

#define ACCELEROMETER_PITCH_DEGREES 12.661f

struct PSDualShock4HIDDetails {
    std::string Device_path;
    hid_device *Handle;
    std::string Bt_addr;      // The bluetooth address of the controller
    std::string Host_bt_addr; // The bluetooth address of the adapter registered with the controller
};

struct PSDualShock4DataInput;   // See .cpp for declaration
struct PSDualShock4DataOutput;  // See .cpp for declaration

class PSDualShock4ControllerConfig : public PSMoveConfig
{
public:
    static const int CONFIG_VERSION;

    PSDualShock4ControllerConfig(const std::string &fnamebase = "PSDualShock4ControllerConfig")
        : PSMoveConfig(fnamebase)
        , is_valid(false)
        , version(CONFIG_VERSION)
        , accelerometer_fit_error(0.f)
        , gyro_gain(0.f)
        , gyro_variance(0.f)
        , gyro_drift(0.f)
        , max_poll_failure_count(100)
        , prediction_time(0.f)
        , min_quality_screen_area(150.f*34.f*.1f)
        , max_quality_screen_area(150.f*34.f) // light bar at ideal range looking straight on is about 150px by 34px 
    {
        // The DS4 uses the BMI055 IMU Chip: 
        // https://www.bosch-sensortec.com/bst/products/all_products/bmi055
        //
        // The Accelerometer can operate in one of 4 modes: 
        //   ±2g, ±4g, ±8g, ±16g
        // The Gyroscope can operate in one of 5 modes: 
        //   ±125°/s, ±250°/s, ±500°/s, ±1000°/s, ±2000°/s
        //   (or ±2.18 rad/s, ±4.36 rad/s, ±8.72 rad/s, ±17.45 rad/s, ±34.9 rad/s)
        //
        // I haven't seen any indication that suggests the DS4 changes modes.
        // It also appears that the raw accelerometer and gyroscope values are pre-calibrated
        // (there is no sensor calibration report with biases and gains that I can find)

        // The following guide: 
        // http://gamedev.stackexchange.com/questions/87106/accessing-dualshock-4-motion-sensor-in-windows-ideally-unity/87178#87178
        // suggests that:
        //  -raw accelerometer value should be divided by 8192 to get g/s
        //  -raw gyroscope value should be divided by 1024 to get rad/s

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

        // Empirical testing of the of the gyro gain looks best at 1/1024.
        // This implies that gyroscope is returned from the controller is pre-calibrated 
        // and is just in a 5.10 fixed point value (+1 sign bit).
        // This agrees with stack exchange article.
        gyro_gain= 1.f / 1024.f;

        // This is the variance of the calibrated gyro value recorded for 100 samples
        // Units in rad/s^2
        gyro_variance= 0.033769f;

        // This is the drift of the raw gyro value recorded for 60 seconds
        // Units rad/s
        gyro_drift= 0.130851f;

        // This is the ideal accelerometer reading you get when you rest the DS4 on a flat surface
        // The ACCELEROMETER_PITCH_DEGREES comes from the calibration utility
        identity_gravity_direction.i= 0.f;
        identity_gravity_direction.j= cosf(ACCELEROMETER_PITCH_DEGREES*k_degrees_to_radians);
        identity_gravity_direction.k= sinf(ACCELEROMETER_PITCH_DEGREES*k_degrees_to_radians);
    };

    virtual const boost::property_tree::ptree config2ptree();
    virtual void ptree2config(const boost::property_tree::ptree &pt);

    bool is_valid;
    long version;

    // calibrated_acc= raw_acc*acc_gain + acc_bias
    CommonDeviceVector accelerometer_gain;
    CommonDeviceVector accelerometer_bias;
    float accelerometer_fit_error;

    // The calibrated "down" direction
    CommonDeviceVector identity_gravity_direction;

    // calibrated_gyro= raw_gyro*gyro_gain
    float gyro_gain;
    // The variance of the raw gyro readings in rad/sec^2
    float gyro_variance;
    // The drift raw gyro readings in rad/second
    float gyro_drift;

    // The pixel area of the tracking projection at which the orientation quality is 0
    float min_quality_screen_area;
    // The pixel area of the tracking projection at which the orientation quality is 0
    float max_quality_screen_area;

    long max_poll_failure_count;
    float prediction_time;
};

struct PSDualShock4ControllerState : public CommonControllerState
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

    PSDualShock4ControllerState()
    {
        clear();
    }

    void clear()
    {
        CommonControllerState::clear();

        RawSequence = 0;
        RawTimeStamp = 0;

        DeviceType = PSDualShock4;

        LeftAnalogX = 0.f;
        LeftAnalogY = 0.f;
        RightAnalogX = 0.f;
        RightAnalogY = 0.f;
        LeftTrigger= 0.f;
        RightTrigger= 0.f;

        DPad_Up = Button_UP;
        DPad_Down = Button_UP;
        DPad_Left = Button_UP;
        DPad_Right = Button_UP;

        Square = Button_UP;
        Cross = Button_UP;
        Circle = Button_UP;
        Triangle = Button_UP;

        L1 = Button_UP;
        R1 = Button_UP;
        L2 = Button_UP;
        R2 = Button_UP;
        L3 = Button_UP;
        R3 = Button_UP;

        Share = Button_UP;
        Options = Button_UP;

        PS = Button_UP;
        TrackPadButton = Button_UP;

        memset(RawAccelerometer, 0, sizeof(int) * 3);
        memset(RawGyro, 0, sizeof(int) * 3);

        CalibratedAccelerometer.clear();
        CalibratedGyro.clear();
    }
};

class PSDualShock4Controller : public IControllerInterface {
public:
    PSDualShock4Controller();
    ~PSDualShock4Controller();

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
    virtual bool getIsBluetooth() const override;
    virtual std::string getUSBDevicePath() const override;
    virtual std::string getAssignedHostBluetoothAddress() const override;
    virtual std::string getSerial() const override;
    virtual const std::tuple<unsigned char, unsigned char, unsigned char> getColour() const override;
    virtual void getTrackingShape(CommonDeviceTrackingShape &outTrackingShape) const override;

    // -- Getters
    inline const PSDualShock4ControllerConfig *getConfig() const
    {
        return &cfg;
    }
    inline PSDualShock4ControllerConfig *getConfigMutable()
    {
        return &cfg;
    }
    static CommonDeviceState::eDeviceType getDeviceTypeStatic()
    {
        return CommonDeviceState::PSDualShock4;
    }

    // -- Setters
    bool setLED(unsigned char r, unsigned char g, unsigned char b);
    bool setLeftRumbleIntensity(unsigned char value);
    bool setRightRumbleIntensity(unsigned char value);

private:
    bool getBTAddressesViaUSB(std::string& host, std::string& controller);
    void clearAndWriteDataOut();
    bool writeDataOut();                            // Setters will call this

    // Constant while a controller is open
    PSDualShock4ControllerConfig cfg;
    PSDualShock4HIDDetails HIDDetails;
    bool IsBluetooth;                               // true if valid serial number on device opening

    // Cached Setter State
    unsigned char LedR, LedG, LedB;
    unsigned char RumbleRight; // Weak
    unsigned char RumbleLeft; // Strong
    bool bWriteStateDirty;
    std::chrono::time_point<std::chrono::high_resolution_clock> lastWriteStateTime;

    // Read Controller State
    int NextPollSequenceNumber;
    std::deque<PSDualShock4ControllerState> ControllerStates;
    PSDualShock4DataInput* InData;                        // Buffer to read hidapi reports into
    PSDualShock4DataOutput* OutData;                      // Buffer to write hidapi reports out from
};
#endif // PSDUALSHOCK4_CONTROLLER_H
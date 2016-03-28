#ifndef OCULUS_HMD_H
#define OCULUS_HMD_H

#include "PSMoveConfig.h"
#include "DeviceEnumerator.h"
#include "DeviceInterface.h"
#include <string>
#include <vector>
#include <deque>

class OculusHMDConfig : public PSMoveConfig
{
public:
    static const int CONFIG_VERSION;

    OculusHMDConfig(const std::string &fnamebase = "OculusHMDConfig")
        : PSMoveConfig(fnamebase)
        , is_valid(false)
        , version(CONFIG_VERSION)
        , max_poll_failure_count(100)
    {
    };

    virtual const boost::property_tree::ptree config2ptree();
    virtual void ptree2config(const boost::property_tree::ptree &pt);

    bool is_valid;
    long version;
    long max_poll_failure_count;
};

struct OculusHMDState : public CommonHMDState
{
    CommonDeviceVector AngularVelocity;        /// The body's angular velocity in radians per second.
    CommonDeviceVector LinearVelocity;         /// The body's velocity in meters per second.
    CommonDeviceVector AngularAcceleration;    /// The body's angular acceleration in radians per second per second.
    CommonDeviceVector LinearAcceleration;     /// The body's acceleration in meters per second per second.
    double StateTime;                          /// Absolute time of this state sample.

    CommonDeviceVector Accelerometer;    /// Acceleration reading in cm/s^2.
    CommonDeviceVector Gyro;             /// Rotation rate in rad/s.
    CommonDeviceVector Magnetometer;     /// Magnetic field in Gauss.
    float Temperature;                   /// Temperature of the sensor in degrees Celsius.
    float IMUSampleTime;                 /// Time when the reported IMU reading took place, in seconds.

    OculusHMDState()
    {
        clear();
    }

    void clear()
    {
        CommonHMDState::clear();

        DeviceType = OculusHMD;
        Accelerometer.clear();
        Gyro.clear();
        Magnetometer.clear();
        Temperature = 0.f;
    }
};

// Base class for all Oculus HMDs (All use the same Oculus API)
class OculusHMD : public IHMDInterface 
{
public:
    OculusHMD();
    ~OculusHMD();

    // OculusHMD
    bool open(); // Opens the first HID device for the controller

    // -- IDeviceInterface
    virtual bool matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const override;
    virtual bool open(const DeviceEnumerator *enumerator) override;
    virtual bool getIsOpen() const override;
    virtual bool getIsReadyToPoll() const override;
    virtual IDeviceInterface::ePollResult poll() override;
    virtual void close() override;
    virtual long getMaxPollFailureCount() const override;
    virtual CommonDeviceState::eDeviceType getDeviceType() const override
    {
        return CommonDeviceState::OculusHMD;
    }
    static CommonDeviceState::eDeviceType getDeviceTypeStatic()
    {
        return CommonDeviceState::OculusHMD;
    }
    virtual const CommonDeviceState * getState(int lookBack = 0) const override;

    // -- IHMDInterface
    virtual std::string getUSBDevicePath() const override;
    virtual std::string getSerial() const override;

    // -- Getters
    inline const OculusHMDConfig *getConfig() const
    {
        return &cfg;
    }
    inline OculusHMDConfig *getConfigMutable()
    {
        return &cfg;
    }
    float getTempCelsius() const;

    // -- Setters

private:
    // Constant while the HMD is open
    OculusHMDConfig cfg;
    class OculusHIDDetails *HIDDetails;                    // Buffer that holds static OculusAPI HMD description

    // Read HMD State
    int NextPollSequenceNumber;
    class OculusDataInput *InData;                        // Buffer to hold most recent OculusAPI tracking state
    std::deque<OculusHMDState> HMDStates;
};

class OculusDevKit2 : public OculusHMD
{
public:
    // -- IDeviceInterface
    virtual CommonDeviceState::eDeviceType getDeviceType() const override
    {
        return CommonDeviceState::OculusDK2;
    }
    static CommonDeviceState::eDeviceType getDeviceTypeStatic()
    {
        return CommonDeviceState::OculusDK2;
    }
};

// TODO:
// DevKit1
// CrescentBay
// EngineeringSample06
// EngineeringSample09
// ConsumerVersion1

#endif // OCULUS_HMD_H
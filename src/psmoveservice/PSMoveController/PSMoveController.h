#ifndef PSMOVE_CONTROLLER_H
#define PSMOVE_CONTROLLER_H

#include "PSMoveConfig.h"
#include "DeviceEnumerator.h"
#include "DeviceInterface.h"
#include "MathAlignment.h"
#include "hidapi.h"
#include <string>
#include <vector>
#include <deque>
#include <chrono>

struct PSMoveHIDDetails {
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
        , max_poll_failure_count(100) 
        , cal_ag_xyz_kb(2, std::vector<std::vector<float>>(3, std::vector<float>(2, 0.f)))
        , prediction_time(0.f)
    {
        magnetometer_ellipsoid.clear();
        magnetometer_identity = Eigen::Vector3f::Zero();
    };

    virtual const boost::property_tree::ptree config2ptree();
    virtual void ptree2config(const boost::property_tree::ptree &pt);

    bool is_valid;
    long version;
    long max_poll_failure_count;
    std::vector<std::vector<std::vector<float>>> cal_ag_xyz_kb;
    EigenFitEllipsoid magnetometer_ellipsoid;
    Eigen::Vector3f magnetometer_identity;
    float prediction_time;
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

    std::vector< std::vector<float> > Accel;    // Two frames of 3 dimensions
    std::vector< std::vector<float> > Gyro;     // Two frames of 3 dimensions
    std::vector<int> Mag;                       // One frame of 3 dimensions

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

        Accel = { {0, 0, 0}, {0, 0, 0} };
        Gyro = { {0, 0, 0}, {0, 0, 0} };
        Mag = {0, 0, 0};

        TempRaw= 0;
    }
};

class PSMoveController : public IControllerInterface {
public:
    PSMoveController();
    ~PSMoveController();

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
    inline const PSMoveControllerConfig *getConfig() const
    { return &cfg; }
    inline PSMoveControllerConfig *getConfigMutable()
    { return &cfg; }
    float getTempCelsius() const;
    static CommonDeviceState::eDeviceType getDeviceTypeStatic()
    { return CommonDeviceState::PSMove; }
    
    
    const unsigned long getLEDPWMFrequency() const
    {
        return LedPWMF;
    }
    

    // -- Setters
    bool setLED(unsigned char r, unsigned char g, unsigned char b); // 0x00..0xff. TODO: vec3
    bool setLEDPWMFrequency(unsigned long freq);    // 733..24e6
    bool setRumbleIntensity(unsigned char value);

private:    
    bool getBTAddress(std::string& host, std::string& controller);
    void loadCalibration();                         // Use USB or file if on BT
    
    bool writeDataOut();                            // Setters will call this
    
    // Constant while a controller is open
    PSMoveControllerConfig cfg;
    PSMoveHIDDetails HIDDetails;
    bool IsBluetooth;                               // true if valid serial number on device opening

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
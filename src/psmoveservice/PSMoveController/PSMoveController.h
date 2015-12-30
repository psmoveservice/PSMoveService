#ifndef PSMOVE_CONTROLLER_H
#define PSMOVE_CONTROLLER_H

#include "PSMoveDataFrame.h"
#include "PSMoveConfig.h"
#include "../DeviceEnumerator.h"
#include "../DeviceInterface.h"
#include "hidapi.h"
#include <string>
#include <vector>
#include <deque>

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
    PSMoveControllerConfig(const std::string &fnamebase = "PSMoveControllerConfig")
        : PSMoveConfig(fnamebase)
        , data_timeout(1000) // ms
        , cal_ag_xyz_kb(2, std::vector<std::vector<float>>(3, std::vector<float>(2, 0.f)))
    {};

    virtual const boost::property_tree::ptree config2ptree();
    virtual void ptree2config(const boost::property_tree::ptree &pt);

    long data_timeout;
    std::vector<std::vector<std::vector<float>>> cal_ag_xyz_kb;
};

// https://code.google.com/p/moveonpc/wiki/InputReport
struct PSMoveControllerState : public CommonControllerState
{
    ButtonState Triangle;
    ButtonState Circle;
    ButtonState Cross;
    ButtonState Square;
    ButtonState Select;
    ButtonState Start;
    ButtonState PS;
    ButtonState Move;

    unsigned char Trigger;  // 0-255. Average of last two frames.

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

        DeviceType = PSMove;

        Triangle = Button_UP;
        Circle = Button_UP;
        Cross = Button_UP;
        Square = Button_UP;
        Select = Button_UP;
        Start = Button_UP;
        PS = Button_UP;
        Move = Button_UP;

        Trigger= 0;

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

    // -- Getters
    inline const PSMoveControllerConfig &getConfig() const
    { return cfg; }
    float getTempCelsius() const;

    // -- Setters
    bool setLED(unsigned char r, unsigned char g, unsigned char b); // 0x00..0xff. TODO: vec3
    bool setLEDPWMFrequency(unsigned long freq);    // 733..24e6
    bool setRumbleIntensity(unsigned char value);

    // IControllerInterface
    virtual bool matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const override;
    virtual bool open(const DeviceEnumerator *enumerator) override;
    virtual IDeviceInterface::ePollResult poll() override;
    virtual void close() override;
    virtual bool setHostBluetoothAddress(const std::string &address) override;

    // -- Getters
    virtual bool getIsBluetooth() const override;
    virtual bool getIsReadyToPoll() const override;
    virtual std::string getUSBDevicePath() const override;
    virtual std::string getSerial() const override;
    virtual std::string getHostBluetoothAddress() const override;
    virtual bool getIsOpen() const override;
    virtual CommonDeviceState::eDeviceType getDeviceType() const override;
    virtual void getState(CommonDeviceState *out_state, int lookBack = 0) const override;
    virtual long getDataTimeout() const override;

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

    // Read Controller State
    std::deque<PSMoveControllerState> ControllerStates;
    PSMoveDataInput* InData;                        // Buffer to copy hidapi reports into
};
#endif // PSMOVE_CONTROLLER_H
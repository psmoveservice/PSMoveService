#ifndef PSNAVI_CONTROLLER_H
#define PSNAVI_CONTROLLER_H

#include "PSMoveConfig.h"
#include "DeviceEnumerator.h"
#include "DeviceInterface.h"
#include "hidapi.h"
#include <string>
#include <vector>
#include <deque>

struct PSNaviHIDDetails {
    std::string Device_path;
    hid_device *Handle;
    std::string Device_path_addr; // only needed by Win > 8.1, otherwise ignored.
    hid_device *Handle_addr; // only needed by Win > 8.1, otherwise ignored.
    std::string Bt_addr;
    std::string Host_bt_addr;
};

struct PSNaviDataInput;  // See .cpp for full declaration

class PSNaviControllerConfig : public PSMoveConfig
{
public:
    PSNaviControllerConfig(const std::string &fnamebase = "PSNaviControllerConfig")
        : PSMoveConfig(fnamebase)
        , max_poll_failure_count(1000) // ms
    {};

    virtual const boost::property_tree::ptree config2ptree();
    virtual void ptree2config(const boost::property_tree::ptree &pt);

    long max_poll_failure_count;
};

// https://code.google.com/p/moveonpc/wiki/NavigationInputReport
struct PSNaviControllerState : public CommonControllerState
{
    ButtonState L1;
    ButtonState L2;
    ButtonState L3;
    ButtonState Circle;
    ButtonState Cross;
    ButtonState PS;
    ButtonState DPad_Up;
    ButtonState DPad_Right;
    ButtonState DPad_Down;
    ButtonState DPad_Left;

    unsigned char Trigger;  // 0-255. 
    unsigned char Stick_XAxis;  // 0-255. Subtract 0x80 to obtain signed values
    unsigned char Stick_YAxis;  // 0-255.  Subtract 0x80 to obtain signed values

    PSNaviControllerState()
    {
        clear();
    }

    void clear()
    {
        CommonControllerState::clear();

        DeviceType = PSNavi;

        L1 = Button_UP;
        L2 = Button_UP;
        L3 = Button_UP;
        Circle = Button_UP;
        Cross = Button_UP;
        PS = Button_UP;
        DPad_Up = Button_UP;
        DPad_Right = Button_UP;
        DPad_Down = Button_UP;
        DPad_Left = Button_UP;

        Trigger= 0;
        Stick_XAxis= 0x80;
        Stick_YAxis= 0x80;
    }
};


class PSNaviController : public IControllerInterface {
public:
    PSNaviController();
    ~PSNaviController();

    // PSNaviController
    bool open();                                             // Opens the first HID device for the controller

    // -- Getters
    inline const PSNaviControllerConfig &getConfig() const
    { return cfg; }

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
    virtual std::string getAssignedHostBluetoothAddress() const override;
    virtual bool getIsOpen() const override;
    static CommonDeviceState::eDeviceType getDeviceTypeStatic() 
    { return CommonDeviceState::PSNavi; }
    virtual CommonDeviceState::eDeviceType getDeviceType() const override;
    virtual const CommonDeviceState * getState(int lookBack = 0) const override;
    virtual long getMaxPollFailureCount() const override;
    virtual const std::tuple<unsigned char, unsigned char, unsigned char> getColour() const override;
    virtual void getTrackingShape(CommonDeviceTrackingShape &outTrackingShape) const override;
        
private:    
    bool getBTAddress(std::string& host, std::string& controller);
    
    // Constant while a controller is open
    PSNaviControllerConfig cfg;
    PSNaviHIDDetails HIDDetails;
    bool IsBluetooth;                               // true if valid serial number on device opening

    // Read Controller State
    int NextPollSequenceNumber;
    std::deque<PSNaviControllerState> ControllerStates;
    PSNaviDataInput* InData;                        // Buffer to copy hidapi reports into
};
#endif // PSMOVE_CONTROLLER_H
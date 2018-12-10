#ifndef PSNAVI_CONTROLLER_H
#define PSNAVI_CONTROLLER_H

#include "PSMoveConfig.h"
#include "DeviceEnumerator.h"
#include "DeviceInterface.h"
#include <string>
#include <vector>

class PSNaviControllerConfig : public PSMoveConfig
{
public:
    PSNaviControllerConfig(const std::string &fnamebase = "PSNaviControllerConfig")
        : PSMoveConfig(fnamebase)
        , max_poll_failure_count(1000) // ms
		, attached_to_controller("")
    {};

    virtual const boost::property_tree::ptree config2ptree();
    virtual void ptree2config(const boost::property_tree::ptree &pt);

    long max_poll_failure_count;
	std::string attached_to_controller;
};

// https://code.google.com/p/moveonpc/wiki/NavigationInputReport
struct PSNaviControllerInputState : public CommonControllerState
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

    PSNaviControllerInputState()
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
    virtual ~PSNaviController();

    // PSNaviController
    bool open();                                             // Opens the first HID device for the controller

    // -- Getters
    inline const PSNaviControllerConfig &getConfig() const
    { return cfg; }
    inline PSNaviControllerConfig &getConfigMutable()
    { return cfg; }

    // IControllerInterface
    virtual bool matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const override;
    virtual bool open(const DeviceEnumerator *enumerator) override;
    virtual IDeviceInterface::ePollResult poll() override;
    virtual void close() override;
    virtual bool setHostBluetoothAddress(const std::string &address) override;
	virtual bool setTrackingColorID(const eCommonTrackingColorID tracking_color_id) override;
	virtual void setControllerListener(IControllerListener *listener) override;

    // -- Getters
    virtual bool getIsBluetooth() const override;
    virtual bool getIsReadyToPoll() const override;
    virtual std::string getUSBDevicePath() const override;
	virtual int getVendorID() const override;
	virtual int getProductID() const override;
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
	virtual bool getTrackingColorID(eCommonTrackingColorID &out_tracking_color_id) const override;
	virtual float getIdentityForwardDegrees() const override;
	virtual float getPredictionTime() const override;
    virtual bool getWasSystemButtonPressed() const override;
        
private:    
	bool setInputStreamEnabledOverUSB();
    bool getHostBTAddressOverUSB(std::string& out_host);
	bool getControllerBTAddressOverUSB(std::string& out_controller);

	IControllerInterface::ePollResult pollUSB();
	IControllerInterface::ePollResult pollHid();
	IControllerInterface::ePollResult pollGamepad();
    
    // Constant while a controller is open
    PSNaviControllerConfig cfg;
	class PSNaviAPIContext *APIContext;
    bool IsBluetooth;                               // true if valid serial number on device opening

    // Cached Controller State
    int NextPollSequenceNumber;
    PSNaviControllerInputState m_cachedInputState;

    // HID Packet Processing
	class PSNaviHidPacketProcessor* m_HIDPacketProcessor;
};
#endif // PSMOVE_CONTROLLER_H
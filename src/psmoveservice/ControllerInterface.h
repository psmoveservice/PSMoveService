#ifndef CONTROLLER_INTERFACE_H
#define CONTROLLER_INTERFACE_H

#include <string>

struct CommonControllerState 
{
    enum eControllerDeviceType
    {
        PSMove,

        SUPPORTED_CONTROLLER_TYPE_COUNT,
        PSNavi, // NYI
    };

    enum ButtonState {
        Button_UP = 0x00,       // (00b) Not pressed
        Button_PRESSED = 0x01,  // (01b) Down for one frame only
        Button_DOWN = 0x03,     // (11b) Down for >1 frame
        Button_RELEASED = 0x02, // (10b) Up for one frame only
    };

    enum BatteryLevel {
        Batt_MIN = 0x00, /*!< Battery is almost empty (< 20%) */
        Batt_20Percent = 0x01, /*!< Battery has at least 20% remaining */
        Batt_40Percent = 0x02, /*!< Battery has at least 40% remaining */
        Batt_60Percent = 0x03, /*!< Battery has at least 60% remaining */
        Batt_80Percent = 0x04, /*!< Battery has at least 80% remaining */
        Batt_MAX = 0x05, /*!< Battery is fully charged (not on charger) */
        Batt_CHARGING = 0xEE, /*!< Battery is currently being charged */
        Batt_CHARGING_DONE = 0xEF, /*!< Battery is fully charged (on charger) */
    };

    eControllerDeviceType DeviceType;

    int Sequence;                               // 4-bit (1..16).
                                                // Sometimes frames are dropped.
    enum BatteryLevel Battery;
    unsigned int TimeStamp;                     // 16-bit (time since ?, units?)
                                                // About 1150 between in-order frames.
    unsigned int AllButtons;                    // all-buttons, used to detect changes

    //TODO: high-precision timestamp. Need to do in hidapi?
    
    inline CommonControllerState()
    {
        clear();
    }

    inline void clear()
    {
        DeviceType= SUPPORTED_CONTROLLER_TYPE_COUNT; // invalid
        Sequence= 0;
        Battery= Batt_MAX;
        TimeStamp= 0;
        AllButtons= 0;
    }
};

class IControllerInterface
{
public:
    enum ePollResult
    {
        _PollResultSuccessNoData,
        _PollResultSuccessNewData,
        _PollResultFailure,
    };

    // Return true if device path matches
    virtual bool matchesDeviceEnumerator(const class ControllerDeviceEnumerator *enumerator) const = 0;

    // Opens the HID device for the controller at the given enumerator
    virtual bool open(const class ControllerDeviceEnumerator *enumerator) = 0;

    // Polls for new controller data
    virtual ePollResult poll() = 0;

    // Closes the HID device for the controller  
    virtual void close() = 0;

    // Sets the address of the bluetooth adapter on the host PC with the controller
    virtual bool setHostBluetoothAddress(const std::string &address) = 0;

    // -- Getters
    // Returns true if the device is connected via Bluetooth, false if by USB
    virtual bool getIsBluetooth() const = 0;

    // Returns the full usb device path for the controller
    virtual std::string getUSBDevicePath() const = 0;

    // Returns the serial number for the controller
    virtual std::string getSerial() const  = 0;

    // Returns true if hidapi opened successfully
    virtual bool getIsOpen() const  = 0;     

    // Returns what type of controller 
    virtual CommonControllerState::eControllerDeviceType getControllerDeviceType() const = 0;

    // Fetch the controller state at the given sample index.
    // A lookBack of 0 corresponds to the most recent data.
    virtual void getState(CommonControllerState *out_state, int lookBack = 0) const = 0;

    // Get the number of milliseconds we're willing to accept no data from the controller before we disconnect it
    virtual long getDataTimeout() const = 0;
};

#endif // CONTROLLER_INTERFACE_H
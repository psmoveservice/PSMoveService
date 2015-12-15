#ifndef SERVER_CONTROLLER_VIEW_H
#define SERVER_CONTROLLER_VIEW_H

//-- includes -----
#include "ControllerInterface.h"
#include "PSMoveDataFrame.h"

// -- declarations -----
class ServerControllerView
{
public:
    ServerControllerView(const int controller_id);
    virtual ~ServerControllerView();

    bool matchesDeviceEnumerator(const class ControllerDeviceEnumerator *enumerator) const;
    bool open(const class ControllerDeviceEnumerator *enumerator);
    bool update();
    void close();

    // Registers the address of the bluetooth adapter on the host PC with the controller
    bool setHostBluetoothAddress(const std::string &address);

    // getters
    inline int getControllerID() const 
    { return m_controllerID; }

    // Estimate the given pose if the controller
    // Positive time values estimate into the future
    // Negative time values get pose values from the past
    psmovePosef getPose(int msec_time = 0) const;

    // Returns true if the device is connected via Bluetooth, false if by USB
    bool getIsBluetooth() const;

    // Returns the full usb device path for the controller
    std::string getUSBDevicePath() const;

    // Returns the serial number for the controller
    std::string getSerial() const;

    // Gets the host bluetooth address registered with the 
    std::string getHostBluetoothAddress() const;

    // Returns true if hidapi opened successfully
    bool getIsOpen() const;

    // Returns what type of controller this controller view represents
    CommonControllerState::eControllerDeviceType getControllerDeviceType() const;

    // Fetch the controller state at the given sample index.
    // A lookBack of 0 corresponds to the most recent data.
    void getState(struct CommonControllerState *out_state, int lookBack = 0) const;

    // setters
    inline void setControllerID(int id)
    { m_controllerID= id; }

    // Set the rumble value between 0-255
    bool setControllerRumble(int rumble_amount);

protected:
    void publish_controller_data_frame();

private:
    int m_controllerID;
    int m_sequence_number;
    long long m_last_updated_tick;
    IControllerInterface *m_controller;
};

#endif // SERVER_CONTROLLER_VIEW_H

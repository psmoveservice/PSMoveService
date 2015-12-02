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
    void update();
    void close();

    // getters
    inline int getControllerID() const 
    { return m_controllerID; }

    // Estimate the given pose if the controller
    // Positive time values estimate into the future
    // Negative time values get pose values from the past
    psmovePosef getPose(int msec_time = 0) const;

    // Returns true if the device is connected via Bluetooth, false if by USB
    bool getIsBluetooth() const;

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
    IControllerInterface *m_controller;
};

#endif // SERVER_CONTROLLER_VIEW_H

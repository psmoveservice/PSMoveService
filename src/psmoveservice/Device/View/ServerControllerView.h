#ifndef SERVER_CONTROLLER_VIEW_H
#define SERVER_CONTROLLER_VIEW_H

//-- includes -----
#include "DeviceInterface.h"
#include "ServerDeviceView.h"
#include "PSMoveDataFrame.h"
#include "PSMoveProtocolInterface.h"
#include "TrackerManager.h"

// -- declarations -----
class ServerControllerView : public ServerDeviceView
{
public:
    ServerControllerView(const int device_id);
    virtual ~ServerControllerView();

    bool open(const class DeviceEnumerator *enumerator) override;
    void close() override;

    // Compute pose/prediction of tracking blob+IMU state
    void updatePositionEstimation(TrackerManager* tracker_manager);
    void updateStateAndPredict();

    // Registers the address of the bluetooth adapter on the host PC with the controller
    bool setHostBluetoothAddress(const std::string &address);
    
    IDeviceInterface* getDevice() const override {return m_device;}
    inline class OrientationFilter * getOrientationFilter() { return m_orientation_filter; }

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

    // Returns what type of controller this controller view represents
    CommonDeviceState::eDeviceType getControllerDeviceType() const;

    // Fetch the controller state at the given sample index.
    // A lookBack of 0 corresponds to the most recent data.
    const CommonControllerState * getState(int lookBack = 0) const;

    // Get the currently assigned tracking color ID for the controller
    inline eCommonTrackingColorID getTrackingColorID() const { return m_tracking_color_id; }

    // Set the rumble value between 0-255
    bool setControllerRumble(int rumble_amount);

protected:
    bool allocate_device_interface(const class DeviceEnumerator *enumerator) override;
    void free_device_interface() override;
    void publish_device_data_frame() override;
    static void generate_controller_data_frame_for_stream(
        const ServerControllerView *controller_view,
        const struct ControllerStreamInfo *stream_info,
        DeviceDataFramePtr &data_frame);

private:
    eCommonTrackingColorID m_tracking_color_id;
    IControllerInterface *m_device;
    class OrientationFilter *m_orientation_filter;
    class PositionFilter *m_position_filter;
    int m_lastPollSeqNumProcessed;
};

#endif // SERVER_CONTROLLER_VIEW_H
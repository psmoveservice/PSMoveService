#ifndef SERVER_CONTROLLER_VIEW_H
#define SERVER_CONTROLLER_VIEW_H

//-- includes -----
#include "DeviceInterface.h"
#include "ServerDeviceView.h"
#include "PSMoveProtocolInterface.h"
#include "TrackerManager.h"
#include <chrono>

// -- declarations -----
struct ControllerPositionEstimation
{
    std::chrono::time_point<std::chrono::high_resolution_clock> last_update_timestamp;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_visible_timestamp;
    bool bValidTimestamps;

    CommonDevicePosition position;
    CommonDeviceTrackingProjection projection;
    bool bCurrentlyTracking;

    inline void clear()
    {
        last_update_timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>();
        last_visible_timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>();
        bValidTimestamps= false;

        position.clear();
        bCurrentlyTracking= false;
    }
};

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

    // Estimate the given pose if the controller at some point into the future
    CommonDevicePose getFilteredPose(float time= 0.f) const;

    // Get the current physics from the filter position and orientation
    CommonDevicePhysics getFilteredPhysics() const;

    // Returns true if the device is connected via Bluetooth, false if by USB
    bool getIsBluetooth() const;

    // Returns the full usb device path for the controller
    std::string getUSBDevicePath() const;

    // Returns the serial number for the controller
    std::string getSerial() const;

    // Gets the host bluetooth address registered with the 
    std::string getAssignedHostBluetoothAddress() const;

    // Returns what type of controller this controller view represents
    CommonDeviceState::eDeviceType getControllerDeviceType() const;

    // Fetch the controller state at the given sample index.
    // A lookBack of 0 corresponds to the most recent data.
    const CommonControllerState * getState(int lookBack = 0) const;

    // Sets the bulb LED color to some new override color
    // If tracking was active this likely will affect controller tracking
    void setLEDOverride(unsigned char r, unsigned char g, unsigned char b);

    // Removes the over led color and restores the tracking color
    // of the controller is currently being tracked
    void clearLEDOverride();

    // Returns true 
    inline bool getIsLEDOverrideActive() const { return m_LED_override_active; }

    // Get the currently assigned tracking color ID for the controller
    inline eCommonTrackingColorID getTrackingColorID() const { return m_tracking_color_id; }

    // Set the assigned tracking color ID for the controller
    void setTrackingColorID(eCommonTrackingColorID colorID);

    // Get the tracking is enabled on this controller
    inline bool getIsTrackingEnabled() const { return m_tracking_enabled && m_multicam_position_estimation != nullptr; }

    // Increment the position tracking listener count
    // Starts position tracking this controller if the count was zero
    void startTracking();

    // Decrements the position tracking listener count
    // Stop tracking this controller if this count becomes zero
    void stopTracking();

    // Get the tracking shape for the controller
    bool getTrackingShape(CommonDeviceTrackingShape &outTrackingShape);

    // Get the position estimate relative to the given tracker id
    inline const ControllerPositionEstimation *getTrackerPositionEstimate(int trackerId) const {
        return (m_tracker_position_estimation != nullptr) ? &m_tracker_position_estimation[trackerId] : nullptr;
    }

    // Get the position estimate derived from multicam positional tracking
    inline const ControllerPositionEstimation *getMulticamPositionEstimate() const { 
        return m_multicam_position_estimation; 
    }

    // return true if one or more cameras saw this controller last update
    inline bool getIsCurrentlyTracking() const { 
        return getIsTrackingEnabled() ? m_multicam_position_estimation->bCurrentlyTracking : false;
    }

    // Set the rumble value between 0-255
    bool setControllerRumble(int rumble_amount);

    // Helper used to publish the current controller state to the given output data frame
    static void generate_controller_data_frame_for_stream(
        const ServerControllerView *controller_view,
        const struct ControllerStreamInfo *stream_info,
        class PSMoveProtocol::DeviceOutputDataFrame* data_frame);

protected:
    void set_tracking_enabled_internal(bool bEnabled);
    void update_LED_color_internal();
    bool allocate_device_interface(const class DeviceEnumerator *enumerator) override;
    void free_device_interface() override;
    void publish_device_data_frame() override;

private:
    // Tracking color state
    std::tuple<unsigned char, unsigned char, unsigned char> m_tracking_color;
    eCommonTrackingColorID m_tracking_color_id;
    int m_tracking_listener_count;
    bool m_tracking_enabled;
    
    // Override color state
    std::tuple<unsigned char, unsigned char, unsigned char> m_LED_override_color;
    bool m_LED_override_active;

    // Device state
    IControllerInterface *m_device;
    
    // Filter state
    ControllerPositionEstimation *m_tracker_position_estimation; // array of size TrackerManager::k_max_devices
    ControllerPositionEstimation *m_multicam_position_estimation;
    class OrientationFilter *m_orientation_filter;
    class PositionFilter *m_position_filter;
    int m_lastPollSeqNumProcessed;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_last_filter_update_timestamp;
    bool m_last_filter_update_timestamp_valid;
};

#endif // SERVER_CONTROLLER_VIEW_H
#ifndef SERVER_CONTROLLER_VIEW_H
#define SERVER_CONTROLLER_VIEW_H

//-- includes -----
#include "DeviceInterface.h"
#include "ServerDeviceView.h"
#include "PSMoveProtocolInterface.h"
#include "TrackerManager.h"
#include <chrono>
#include <vector>

// -- declarations -----
struct ControllerOpticalPoseEstimation
{
    std::chrono::time_point<std::chrono::high_resolution_clock> last_update_timestamp;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_visible_timestamp;
    bool bValidTimestamps;

    CommonDevicePosition position_cm; // centimeters
    CommonDeviceTrackingProjection projection;
    bool bCurrentlyTracking;

    CommonDeviceQuaternion orientation;
    bool bOrientationValid;

    inline void clear()
    {
        last_update_timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>();
        last_visible_timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>();
        bValidTimestamps= false;

        position_cm.clear();
        bCurrentlyTracking= false;

        orientation.clear();
        bOrientationValid= false;

        memset(&projection, 0, sizeof(CommonDeviceTrackingProjection));
        projection.shape_type= eCommonTrackingProjectionType::INVALID_PROJECTION;
    }
};

class ServerControllerView : public ServerDeviceView
{
public:
    ServerControllerView(const int device_id);
    virtual ~ServerControllerView();

    bool open(const class DeviceEnumerator *enumerator) override;
    void close() override;

	// Tell the pose filter that the controller is aligned with global forward 
	// with the given pose relative to it's identity pose.
	// Recenter the pose filter state accordingly.
	bool recenterOrientation(const CommonDeviceQuaternion& q_pose_relative_to_identity_pose);

	// Recreate and initialize the pose filter for the controller
	void resetPoseFilter();

    // Compute pose/prediction of tracking blob+IMU state
    void updateOpticalPoseEstimation(TrackerManager* tracker_manager);
    void updateStateAndPredict();

    // Registers the address of the bluetooth adapter on the host PC with the controller
    bool setHostBluetoothAddress(const std::string &address);
    
    IDeviceInterface* getDevice() const override {return m_device;}
    inline class IPoseFilter * getPoseFilterMutable() { return m_pose_filter; }
    inline const class IPoseFilter * getPoseFilter() const { return m_pose_filter; }

    // Estimate the given pose if the controller at some point into the future
    CommonDevicePose getFilteredPose(float time= 0.f) const;

    // Get the current physics from the filter position and orientation
    CommonDevicePhysics getFilteredPhysics() const;

    // Returns true if the device is connected via Bluetooth, false if by USB
    bool getIsBluetooth() const;

    // Returns the full usb device path for the controller
    std::string getUSBDevicePath() const;

	// Returns the vendor ID of the controller
	int getVendorID() const;

	// Returns the product ID of the controller
	int getProductID() const;

	// Returns the serial number for the controller
    std::string getSerial() const;

	// Returns the "controller_" + serial number for the controller
	std::string getConfigIdentifier() const;

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
	eCommonTrackingColorID getTrackingColorID() const;

    // Set the assigned tracking color ID for the controller
    void setTrackingColorID(eCommonTrackingColorID colorID);

    // Get the tracking is enabled on this controller
    inline bool getIsTrackingEnabled() const { return m_tracking_enabled && m_multicam_pose_estimation != nullptr; }

    // Increment the position tracking listener count
    // Starts position tracking this controller if the count was zero
    void startTracking();

    // Decrements the position tracking listener count
    // Stop tracking this controller if this count becomes zero
    void stopTracking();

    // Get the tracking shape for the controller
    bool getTrackingShape(CommonDeviceTrackingShape &outTrackingShape) const;

	// Get if the region-of-interest optimization is disabled for this controller
	inline bool getIsROIDisabled() const { return m_roi_disable_count > 0; }
	
	// Request the controller to not use the ROI optimization
	inline void pushDisableROI() { ++m_roi_disable_count; }

	// Undo the request to not use the ROI optimization
	inline void popDisableROI() { assert(m_roi_disable_count > 0); --m_roi_disable_count;  }

	// Get the prediction time used for ROI tracking
	float getROIPredictionTime() const;

    // Get the pose estimate relative to the given tracker id
    inline const ControllerOpticalPoseEstimation *getTrackerPoseEstimate(int trackerId) const {
        return (m_tracker_pose_estimations != nullptr) ? &m_tracker_pose_estimations[trackerId] : nullptr;
    }

    // Get the pose estimate derived from multicam pose tracking
    inline const ControllerOpticalPoseEstimation *getMulticamPoseEstimate() const { 
        return m_multicam_pose_estimation; 
    }

    // return true if one or more cameras saw this controller last update
    inline bool getIsCurrentlyTracking() const { 
        return getIsTrackingEnabled() ? m_multicam_pose_estimation->bCurrentlyTracking : false;
    }

    // Set the rumble value between 0.f-1.f on a channel
    bool setControllerRumble(float rumble_amount, CommonControllerState::RumbleChannel channel);

protected:
    void set_tracking_enabled_internal(bool bEnabled);
    void update_LED_color_internal();
    bool allocate_device_interface(const class DeviceEnumerator *enumerator) override;
    void free_device_interface() override;
    void publish_device_data_frame() override;
    static void generate_controller_data_frame_for_stream(
        const ServerControllerView *controller_view,
        const struct ControllerStreamInfo *stream_info,
        PSMoveProtocol::DeviceOutputDataFrame *data_frame);

private:
    // Tracking color state
    std::tuple<unsigned char, unsigned char, unsigned char> m_tracking_color;
    int m_tracking_listener_count;
    bool m_tracking_enabled;

	// Region-of-Interest state
	int m_roi_disable_count;
    
    // Override color state
    std::tuple<unsigned char, unsigned char, unsigned char> m_LED_override_color;
    bool m_LED_override_active;

    // Device state
    IControllerInterface *m_device;
    
    // Filter state
    ControllerOpticalPoseEstimation *m_tracker_pose_estimations; // array of size TrackerManager::k_max_devices
    ControllerOpticalPoseEstimation *m_multicam_pose_estimation;
    class IPoseFilter *m_pose_filter;
    class PoseFilterSpace *m_pose_filter_space;
    int m_lastPollSeqNumProcessed;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_last_filter_update_timestamp;
    bool m_last_filter_update_timestamp_valid;
};

#endif // SERVER_CONTROLLER_VIEW_H

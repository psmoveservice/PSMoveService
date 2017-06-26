#ifndef SERVER_HMD_VIEW_H
#define SERVER_HMD_VIEW_H

//-- includes -----
#include "ServerDeviceView.h"
#include "PSMoveProtocolInterface.h"
#include <cstring>

// -- pre-declarations -----
class TrackerManager;

// -- declarations -----
struct HMDOpticalPoseEstimation
{
	std::chrono::time_point<std::chrono::high_resolution_clock> last_update_timestamp;
	std::chrono::time_point<std::chrono::high_resolution_clock> last_visible_timestamp;
	bool bValidTimestamps;

	CommonDevicePosition position_cm;
	CommonDeviceTrackingProjection projection;
	bool bCurrentlyTracking;

	CommonDeviceQuaternion orientation;
	bool bOrientationValid;

	inline void clear()
	{
		last_update_timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>();
		last_visible_timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>();
		bValidTimestamps = false;

		position_cm.clear();
		bCurrentlyTracking = false;

		orientation.clear();
		bOrientationValid = false;

		memset(&projection, 0, sizeof(CommonDeviceTrackingProjection));
		projection.shape_type = eCommonTrackingProjectionType::INVALID_PROJECTION;
	}
};

class ServerHMDView : public ServerDeviceView
{
public:
    ServerHMDView(const int device_id);
    ~ServerHMDView();

    bool open(const class DeviceEnumerator *enumerator) override;
    void close() override;

	// Recreate and initialize the pose filter for the HMD
	void resetPoseFilter();

	// Compute pose/prediction of tracking blob+IMU state
	void updateOpticalPoseEstimation(TrackerManager* tracker_manager);
    void updateStateAndPredict();

    IDeviceInterface* getDevice() const override { return m_device; }
	inline class IPoseFilter * getPoseFilterMutable() { return m_pose_filter; }
	inline const class IPoseFilter * getPoseFilter() const { return m_pose_filter; }

	// Estimate the given pose if the controller at some point into the future
	CommonDevicePose getFilteredPose(float time = 0.f) const;

	// Get the current physics from the filter position and orientation
	CommonDevicePhysics getFilteredPhysics() const;

    // Returns the full usb device path for the controller
    std::string getUSBDevicePath() const;

	// Returns the "controller_" + serial number for the controller
	std::string getConfigIdentifier() const;

    // Returns what type of HMD this HMD view represents
    CommonDeviceState::eDeviceType getHMDDeviceType() const;

    // Fetch the controller state at the given sample index.
    // A lookBack of 0 corresponds to the most recent data.
    const struct CommonHMDState * getState(int lookBack = 0) const;

	// Get the tracking is enabled on this controller
	inline bool getIsTrackingEnabled() const { return m_tracking_enabled && m_multicam_pose_estimation != nullptr; }

	// Increment the position tracking listener count
	// Starts position tracking this HMD if the count was zero
	void startTracking();

	// Decrements the position tracking listener count
	// Stop tracking this HMD if this count becomes zero
	void stopTracking();

	// Get the tracking shape for the controller
	bool getTrackingShape(CommonDeviceTrackingShape &outTrackingShape) const;

	// Get the currently assigned tracking color ID for the controller
	eCommonTrackingColorID getTrackingColorID() const;

    // Set the assigned tracking color ID for the controller
    bool setTrackingColorID(eCommonTrackingColorID colorID);

	// Get if the region-of-interest optimization is disabled for this HMD
	inline bool getIsROIDisabled() const { return m_roi_disable_count > 0; }

	// Request the HMD to not use the ROI optimization
	inline void pushDisableROI() { ++m_roi_disable_count; }

	// Undo the request to not use the ROI optimization
	inline void popDisableROI() { assert(m_roi_disable_count > 0); --m_roi_disable_count; }

	// get the prediction time used for region of interest calculation
	float getROIPredictionTime() const;

	// Get the pose estimate relative to the given tracker id
	inline const HMDOpticalPoseEstimation *getTrackerPoseEstimate(int trackerId) const {
		return (m_tracker_pose_estimations != nullptr) ? &m_tracker_pose_estimations[trackerId] : nullptr;
	}

	// Get the pose estimate derived from multicam pose tracking
	inline const HMDOpticalPoseEstimation *getMulticamPoseEstimate() const {
		return m_multicam_pose_estimation;
	}

	// return true if one or more cameras saw this controller last update
	inline bool getIsCurrentlyTracking() const {
		return getIsTrackingEnabled() ? m_multicam_pose_estimation->bCurrentlyTracking : false;
	}

protected:
	void set_tracking_enabled_internal(bool bEnabled);
    bool allocate_device_interface(const class DeviceEnumerator *enumerator) override;
    void free_device_interface() override;
    void publish_device_data_frame() override;
    static void generate_hmd_data_frame_for_stream(
        const ServerHMDView *hmd_view,
        const struct HMDStreamInfo *stream_info,
        DeviceOutputDataFramePtr &data_frame);

private:
	// Tracking color state
	int m_tracking_listener_count;
	bool m_tracking_enabled;

	// ROI state
	int m_roi_disable_count;

	// Device State
    IHMDInterface *m_device;

	// Filter state
	HMDOpticalPoseEstimation *m_tracker_pose_estimations; // array of size TrackerManager::k_max_devices
	HMDOpticalPoseEstimation *m_multicam_pose_estimation;
	class IPoseFilter *m_pose_filter;
	class PoseFilterSpace *m_pose_filter_space;
    int m_lastPollSeqNumProcessed;
	std::chrono::time_point<std::chrono::high_resolution_clock> m_last_filter_update_timestamp;
	bool m_last_filter_update_timestamp_valid;
};

#endif // SERVER_HMD_VIEW_H
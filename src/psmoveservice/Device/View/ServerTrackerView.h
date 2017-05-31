#ifndef SERVER_TRACKER_VIEW_H
#define SERVER_TRACKER_VIEW_H

//-- includes -----
#include "ServerDeviceView.h"
#include "PSMoveProtocolInterface.h"
#include <vector>

// -- pre-declarations -----
namespace PSMoveProtocol
{
    class Response_ResultTrackerSettings;
    class TrackingColorPreset;
};

// -- declarations -----
class ServerTrackerView : public ServerDeviceView
{
public:
    ServerTrackerView(const int device_id);
    ~ServerTrackerView();

    bool open(const class DeviceEnumerator *enumerator) override;
    void close() override;

    // Starts or stops streaming of the video feed to the shared memory buffer.
    // Keep a ref count of how many clients are following the stream.
    void startSharedMemoryVideoStream();
    void stopSharedMemoryVideoStream();

    // Fetch the next video frame and copy to shared memory
    bool poll() override;

    IDeviceInterface* getDevice() const override {return m_device;}

    // Returns what type of tracker this tracker view represents
    CommonDeviceState::eDeviceType getTrackerDeviceType() const;

    // Returns what type of driver this tracker uses
    ITrackerInterface::eDriverType getTrackerDriverType() const;

    // Returns the full usb device path for the controller
    std::string getUSBDevicePath() const;

    // Returns the name of the shared memory block video frames are written to
    std::string getSharedMemoryStreamName() const;
    
    void loadSettings();
    void saveSettings();

	double getFrameWidth() const;
	void setFrameWidth(double value, bool bUpdateConfig);

	double getFrameHeight() const;
	void setFrameHeight(double value, bool bUpdateConfig);

	double getFrameRate() const;
	void setFrameRate(double value, bool bUpdateConfig);

    double getExposure() const;
    void setExposure(double value, bool bUpdateConfig);

    double getGain() const;
    void setGain(double value, bool bUpdateConfig);
    
    bool computeProjectionForController(
        const class ServerControllerView* tracked_controller, 
		const struct CommonDeviceTrackingShape *tracking_shape,
        struct ControllerOpticalPoseEstimation *out_pose_estimate);
    bool computeProjectionForHMD(
		const class ServerHMDView* tracked_hmd,
		const struct CommonDeviceTrackingShape *tracking_shape,
		struct HMDOpticalPoseEstimation *out_pose_estimate);
    bool computePoseForProjection(
		const struct CommonDeviceTrackingProjection *projection,
		const struct CommonDeviceTrackingShape *tracking_shape,
		const struct CommonDevicePose *pose_guess,
		struct ControllerOpticalPoseEstimation *out_pose_estimate);
    
    std::vector<CommonDeviceScreenLocation> projectTrackerRelativePositions(
                                const std::vector<CommonDevicePosition> &objectPositions) const;
    
    CommonDeviceScreenLocation projectTrackerRelativePosition(const CommonDevicePosition *trackerRelativePosition) const;
    
    CommonDevicePosition computeWorldPosition(const CommonDevicePosition *tracker_relative_position) const;
    CommonDeviceQuaternion computeWorldOrientation(const CommonDeviceQuaternion *tracker_relative_orientation) const;

    CommonDevicePosition computeTrackerPosition(const CommonDevicePosition *world_relative_position) const;
    CommonDeviceQuaternion computeTrackerOrientation(const CommonDeviceQuaternion *world_relative_orientation) const;

    /// Given a single screen location on two different trackers, compute the triangulated world space location
    static CommonDevicePosition triangulateWorldPosition(
        const ServerTrackerView *tracker, const CommonDeviceScreenLocation *screen_location,
        const ServerTrackerView *other_tracker, const CommonDeviceScreenLocation *other_screen_location);

	/// Given a set of screen locations on two different trackers, compute the triangulated world space locations
	static void triangulateWorldPositions(
		const ServerTrackerView *tracker, 
		const CommonDeviceScreenLocation *screen_locations,
		const ServerTrackerView *other_tracker,
		const CommonDeviceScreenLocation *other_screen_locations,
		const int screen_location_count,
		CommonDevicePosition *out_result);

    /// Given screen projections on two different trackers, compute the triangulated world space location
    static CommonDevicePose triangulateWorldPose(
        const ServerTrackerView *tracker, const CommonDeviceTrackingProjection *tracker_relative_projection,
        const ServerTrackerView *other_tracker, const CommonDeviceTrackingProjection *other_tracker_relative_projection);

    void getCameraIntrinsics(
        float &outFocalLengthX, float &outFocalLengthY,
        float &outPrincipalX, float &outPrincipalY,
        float &outDistortionK1, float &outDistortionK2, float &outDistortionK3,
        float &outDistortionP1, float &outDistortionP2) const;
    void setCameraIntrinsics(
        float focalLengthX, float focalLengthY,
        float principalX, float principalY,
        float distortionK1, float distortionK2, float distortionK3,
        float distortionP1, float distortionP2);

    CommonDevicePose getTrackerPose() const;
    void setTrackerPose(const struct CommonDevicePose *pose);

    void getPixelDimensions(float &outWidth, float &outHeight) const;
    void getFOV(float &outHFOV, float &outVFOV) const;
    void getZRange(float &outZNear, float &outZFar) const;

    void gatherTrackerOptions(PSMoveProtocol::Response_ResultTrackerSettings* settings) const;
    bool setOptionIndex(const std::string &option_name, int option_index);
    bool getOptionIndex(const std::string &option_name, int &out_option_index) const;

    void gatherTrackingColorPresets(const class ServerControllerView *controller, PSMoveProtocol::Response_ResultTrackerSettings* settings) const;
	void gatherTrackingColorPresets(const class ServerHMDView *hmd, PSMoveProtocol::Response_ResultTrackerSettings* settings) const;

    void setControllerTrackingColorPreset(const class ServerControllerView *controller, eCommonTrackingColorID color, const CommonHSVColorRange *preset);
    void getControllerTrackingColorPreset(const class ServerControllerView *controller, eCommonTrackingColorID color, CommonHSVColorRange *out_preset) const;

	void setHMDTrackingColorPreset(const class ServerHMDView *controller, eCommonTrackingColorID color, const CommonHSVColorRange *preset);
	void getHMDTrackingColorPreset(const class ServerHMDView *controller, eCommonTrackingColorID color, CommonHSVColorRange *out_preset) const;

protected:
    bool allocate_device_interface(const class DeviceEnumerator *enumerator) override;
    void free_device_interface() override;
    void publish_device_data_frame() override;
    static void generate_tracker_data_frame_for_stream(
        const ServerTrackerView *tracker_view, const struct TrackerStreamInfo *stream_info,
        DeviceOutputDataFramePtr &data_frame);

private:
    char m_shared_memory_name[256];
    class SharedVideoFrameReadWriteAccessor *m_shared_memory_accesor;
    int m_shared_memory_video_stream_count;
    class OpenCVBufferState *m_opencv_buffer_state;
    ITrackerInterface *m_device;
};

#endif // SERVER_TRACKER_VIEW_H

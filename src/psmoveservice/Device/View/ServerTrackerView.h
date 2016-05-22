#ifndef SERVER_TRACKER_VIEW_H
#define SERVER_TRACKER_VIEW_H

//-- includes -----
#include "ServerDeviceView.h"
#include "PSMoveProtocolInterface.h"

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

    // Get controller colors and update tracking blob positions/predictions
    void updateStateAndPredict() override;

    IDeviceInterface* getDevice() const override {return m_device;}

    // Returns what type of tracker this tracker view represents
    CommonDeviceState::eDeviceType getTrackerDeviceType() const;

    // Returns what type of driver this tracker uses
    ITrackerInterface::eDriverType getTrackerDriverType() const;

    // Returns the full usb device path for the controller
    std::string getUSBDevicePath() const;

    // Returns the name of the shared memory block video frames are written to
    std::string getSharedMemoryStreamName() const;
    
    double getExposure() const;
    void setExposure(double value);

    void getCameraIntrinsics(
        float &outFocalLengthX, float &outFocalLengthY,
        float &outPrincipalX, float &outPrincipalY) const;
    void setCameraIntrinsics(
        float focalLengthX, float focalLengthY,
        float principalX, float principalY);

    void getTrackerPose(struct CommonDevicePose *outPose, struct CommonDevicePose *outHmdRelativePose) const;
    void setTrackerPose(const struct CommonDevicePose *pose, const struct CommonDevicePose *hmdRelativePose);

    void getPixelDimensions(float &outWidth, float &outHeight) const;
    void getFOV(float &outHFOV, float &outVFOV) const;
    void getZRange(float &outZNear, float &outZFar) const;

protected:
    bool allocate_device_interface(const class DeviceEnumerator *enumerator) override;
    void free_device_interface() override;
    void publish_device_data_frame() override;
    static void generate_tracker_data_frame_for_stream(
        const ServerTrackerView *tracker_view, const struct TrackerStreamInfo *stream_info,
        DeviceDataFramePtr &data_frame);

private:
    char m_shared_memory_name[256];
    class SharedVideoFrameReadWriteAccessor *m_shared_memory_accesor;
    int m_shared_memory_video_stream_count;
    ITrackerInterface *m_device;
};

#endif // SERVER_TRACKER_VIEW_H

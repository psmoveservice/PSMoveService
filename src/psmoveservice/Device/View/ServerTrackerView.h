#ifndef SERVER_TRACKER_VIEW_H
#define SERVER_TRACKER_VIEW_H

//-- includes -----
#include "ServerDeviceView.h"


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

protected:
    bool allocate_device_interface(const class DeviceEnumerator *enumerator) override;
    void free_device_interface() override;
    void publish_device_data_frame() override;

private:
    char m_shared_memory_name[256];
    class SharedVideoFrameReadWriteAccessor *m_shared_memory_accesor;
    int m_shared_memory_video_stream_count;
    ITrackerInterface *m_device;
};

#endif // SERVER_TRACKER_VIEW_H

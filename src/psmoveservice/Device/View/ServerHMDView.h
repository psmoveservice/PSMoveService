#ifndef SERVER_HMD_VIEW_H
#define SERVER_HMD_VIEW_H

//-- includes -----
#include "ServerDeviceView.h"
#include "PSMoveProtocolInterface.h"

// -- declarations -----
class ServerHMDView : public ServerDeviceView
{
public:
    ServerHMDView(const int device_id);
    ~ServerHMDView();

    bool open(const class DeviceEnumerator *enumerator) override;

    // Get the pose + prediction for HMD
    void updateStateAndPredict() override;

    IDeviceInterface* getDevice() const override { return m_device; }

    // Estimate the given pose if the controller
    // Positive time values estimate into the future
    // Negative time values get pose values from the past
    CommonDevicePose getPose(int msec_time = 0) const;

    // Returns the full usb device path for the controller
    std::string getUSBDevicePath() const;

    // Returns what type of HMD this HMD view represents
    CommonDeviceState::eDeviceType getHMDDeviceType() const;

    // Fetch the controller state at the given sample index.
    // A lookBack of 0 corresponds to the most recent data.
    const struct CommonHMDState * getState(int lookBack = 0) const;

protected:
    bool allocate_device_interface(const class DeviceEnumerator *enumerator) override;
    void free_device_interface() override;
    void publish_device_data_frame() override;
    static void generate_hmd_data_frame_for_stream(
        const ServerHMDView *hmd_view,
        const struct HMDStreamInfo *stream_info,
        DeviceDataFramePtr &data_frame);

private:
    IHMDInterface *m_device;
    class OrientationFilter *m_orientation_filter;
    class PositionFilter *m_position_filter;
    int m_lastPollSeqNumProcessed;
};

#endif // SERVER_HMD_VIEW_H
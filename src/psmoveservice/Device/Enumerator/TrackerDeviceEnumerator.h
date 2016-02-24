#ifndef TRACKER_DEVICE_ENUMERATOR_H
#define TRACKER_DEVICE_ENUMERATOR_H

#include "DeviceEnumerator.h"

class TrackerDeviceEnumerator : public DeviceEnumerator
{
public:
    TrackerDeviceEnumerator();
    TrackerDeviceEnumerator(CommonDeviceState::eDeviceType deviceType);
    ~TrackerDeviceEnumerator();

    bool is_valid() const override;
    bool next() override;
    const char *get_path() const override;
    inline int get_camera_index() const { return camera_index; }

private:
    char cur_path[256];
    struct libusb_context* usb_context;
    struct libusb_device **devs, *cur_dev;
    int dev_index, dev_count;
    int camera_index;
};

#endif // TRACKER_DEVICE_ENUMERATOR_H
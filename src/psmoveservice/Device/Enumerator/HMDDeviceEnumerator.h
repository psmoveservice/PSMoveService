#ifndef HMD_DEVICE_ENUMERATOR_H
#define HMD_DEVICE_ENUMERATOR_H

// -- includes -----
#include "DeviceEnumerator.h"

// -- definitions -----
class HMDDeviceEnumerator : public DeviceEnumerator
{
public:
    HMDDeviceEnumerator();
    HMDDeviceEnumerator(CommonDeviceState::eDeviceType deviceType);
    ~HMDDeviceEnumerator();

    bool is_valid() const override;
    bool next() override;
    const char *get_path() const override;
    inline int get_hmd_index() const { return hmd_index; }

protected:
    bool recompute_current_device_validity();

private:
    char cur_path[256];
    struct libusb_context* usb_context;
    struct libusb_device **devs, *cur_dev;
    unsigned char dev_port_numbers[MAX_USB_DEVICE_PORT_PATH];
    int dev_index, dev_count;
    int hmd_index;
    bool dev_valid;
};

#endif // HMD_DEVICE_ENUMERATOR_H
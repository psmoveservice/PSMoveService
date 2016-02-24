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

private:
    struct hid_device_info *devs, *cur_dev;
};

#endif // HMD_DEVICE_ENUMERATOR_H
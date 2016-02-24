#ifndef CONTROLLER_DEVICE_ENUMERATOR_H
#define CONTROLLER_DEVICE_ENUMERATOR_H

#include "DeviceEnumerator.h"

class ControllerDeviceEnumerator : public DeviceEnumerator
{
public:
    ControllerDeviceEnumerator();
    ControllerDeviceEnumerator(CommonDeviceState::eDeviceType deviceType);
    ~ControllerDeviceEnumerator();

    bool is_valid() const override;
    bool next() override;
    const char *get_path() const override;

    bool get_serial_number(char *out_mb_serial, const size_t mb_buffer_size) const;

private:
    struct hid_device_info *devs, *cur_dev;
};

#endif // CONTROLLER_DEVICE_ENUMERATOR_H
#ifndef CONTROLLER_ENUMERATOR_H
#define CONTROLLER_ENUMERATOR_H

#include "ControllerInterface.h"

class ControllerDeviceEnumerator
{
public:
    ControllerDeviceEnumerator();
    ControllerDeviceEnumerator(CommonControllerState::eControllerDeviceType deviceType);
    ~ControllerDeviceEnumerator();

    bool is_valid() const;
    bool next();

    const char *get_path() const;
    bool get_serial_number(char *out_mb_serial, const size_t mb_buffer_size) const;    
    inline CommonControllerState::eControllerDeviceType get_device_type() const
    {
        return m_deviceType;
    }

private:
    struct hid_device_info *devs, *cur_dev;
    CommonControllerState::eControllerDeviceType m_deviceType;
};

#endif // CONTROLLER_ENUMERATOR_H
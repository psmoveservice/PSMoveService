#ifndef DEVICE_ENUMERATOR_H
#define DEVICE_ENUMERATOR_H

#include "DeviceInterface.h"
#include "stdlib.h" // size_t

class DeviceEnumerator
{
public:
    DeviceEnumerator();
    DeviceEnumerator(CommonDeviceState::eDeviceType deviceType);
    virtual ~DeviceEnumerator();
    virtual bool is_valid() const =0;
    virtual bool next()=0;
    virtual const char *get_path() const =0;
    
    inline CommonDeviceState::eDeviceType get_device_type() const
    {
        return m_deviceType;
    }
protected:
    CommonDeviceState::eDeviceType m_deviceType;
};

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

#endif // DEVICE_ENUMERATOR_H
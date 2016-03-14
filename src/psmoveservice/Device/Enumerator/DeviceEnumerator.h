#ifndef DEVICE_ENUMERATOR_H
#define DEVICE_ENUMERATOR_H

// -- includes -----
#include "DeviceInterface.h"
#include "stdlib.h" // size_t

// -- macros ----
#define GET_DEVICE_TYPE_INDEX(device_type)  (device_type & 0x0f)
#define GET_DEVICE_TYPE_CLASS(device_type)  (device_type & 0xf0)

// -- definitions -----
struct USBDeviceInfo
{
    unsigned short vendor_id;
    unsigned short product_id;
};

class DeviceEnumerator
{
public:
    DeviceEnumerator() : 
        m_deviceType(static_cast<CommonDeviceState::eDeviceType>(0))
    { }
    DeviceEnumerator(CommonDeviceState::eDeviceType deviceType) : 
        m_deviceType(deviceType)
    { }
    virtual ~DeviceEnumerator() {}

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

#endif // DEVICE_ENUMERATOR_H
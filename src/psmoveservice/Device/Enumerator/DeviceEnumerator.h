#ifndef DEVICE_ENUMERATOR_H
#define DEVICE_ENUMERATOR_H

// -- includes -----
#include "DeviceInterface.h"
#include "stdlib.h" // size_t

// -- definitions -----
class DeviceEnumerator
{
public:
    DeviceEnumerator() 
		: m_deviceType(CommonDeviceState::INVALID_DEVICE_TYPE)
		, m_deviceTypeFilter(CommonDeviceState::INVALID_DEVICE_TYPE)
    { }
    DeviceEnumerator(CommonDeviceState::eDeviceType deviceTypeFilter)
		: m_deviceType(CommonDeviceState::INVALID_DEVICE_TYPE)
		, m_deviceTypeFilter(deviceTypeFilter)
    { }
    virtual ~DeviceEnumerator() {}

    virtual bool is_valid() const =0;
    virtual bool next()=0;
	virtual int get_vendor_id() const =0;
	virtual int get_product_id() const =0;
    virtual const char *get_path() const =0;
    
    inline CommonDeviceState::eDeviceType get_device_type() const
    {
        return m_deviceType;
    }

    inline CommonDeviceState::eDeviceType get_device_type_filter() const
    {
        return m_deviceTypeFilter;
    }

protected:
    CommonDeviceState::eDeviceType m_deviceType;
	CommonDeviceState::eDeviceType m_deviceTypeFilter;
};

#endif // DEVICE_ENUMERATOR_H
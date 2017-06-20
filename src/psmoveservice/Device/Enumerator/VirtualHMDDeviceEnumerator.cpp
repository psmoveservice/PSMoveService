// -- includes -----
#include "VirtualHMDDeviceEnumerator.h"
#include "ServerUtility.h"
#include "assert.h"
#include "hidapi.h"
#include "string.h"

//-- Statics
int VirtualHMDDeviceEnumerator::virtual_hmd_count= 0;

// -- VirtualHMDDeviceEnumerator -----
VirtualHMDDeviceEnumerator::VirtualHMDDeviceEnumerator()
    : DeviceEnumerator(CommonDeviceState::VirtualHMD)
{
	m_deviceType= CommonDeviceState::VirtualHMD;
    m_device_index= 0;

    m_current_device_identifier= "VirtualHMD__0";
    m_device_count= virtual_hmd_count;
}

const char *VirtualHMDDeviceEnumerator::get_path() const
{
	return m_current_device_identifier.c_str();
}

int VirtualHMDDeviceEnumerator::get_vendor_id() const
{
	return is_valid() ? 0x0000 : -1;
}

int VirtualHMDDeviceEnumerator::get_product_id() const
{
	return is_valid() ? 0x0000 : -1;
}

bool VirtualHMDDeviceEnumerator::is_valid() const
{
	return m_device_index < m_device_count;
}

bool VirtualHMDDeviceEnumerator::next()
{
	bool foundValid = false;

	++m_device_index;
    if (m_device_index < m_device_count)
    {
        char device_path[32];
        ServerUtility::format_string(device_path, sizeof(device_path), "VirtualHMD__%s", m_device_index);

        m_current_device_identifier= device_path;
    }

	return foundValid;
}
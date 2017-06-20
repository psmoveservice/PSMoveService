// -- includes -----
#include "VirtualControllerEnumerator.h"
#include "ServerUtility.h"
#include "assert.h"
#include "hidapi.h"
#include "string.h"

//-- Statics
int VirtualControllerEnumerator::virtual_controller_count= 0;

// -- VirtualControllerDeviceEnumerator -----
VirtualControllerEnumerator::VirtualControllerEnumerator()
    : DeviceEnumerator(CommonDeviceState::VirtualController)
{
	m_deviceType= CommonDeviceState::VirtualController;
    m_device_index= 0;

    m_current_device_identifier= "VirtualController_0";
    m_device_count= virtual_controller_count;
}

const char *VirtualControllerEnumerator::get_path() const
{
	return m_current_device_identifier.c_str();
}

int VirtualControllerEnumerator::get_vendor_id() const
{
	return is_valid() ? 0x0000 : -1;
}

int VirtualControllerEnumerator::get_product_id() const
{
	return is_valid() ? 0x0000 : -1;
}

bool VirtualControllerEnumerator::is_valid() const
{
	return m_device_index < m_device_count;
}

bool VirtualControllerEnumerator::next()
{
	bool foundValid = false;

	++m_device_index;
    if (m_device_index < m_device_count)
    {
        char device_path[32];
        ServerUtility::format_string(device_path, sizeof(device_path), "VirtualController_%d", m_device_index);

        m_current_device_identifier= device_path;
    }

	return foundValid;
}
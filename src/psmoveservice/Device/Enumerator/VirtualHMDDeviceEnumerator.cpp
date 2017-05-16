// -- includes -----
#include "VirtualHMDDeviceEnumerator.h"
#include "DeviceManager.h"
#include "HMDManager.h"
#include "ServerUtility.h"
#include "assert.h"
#include "hidapi.h"
#include "string.h"

// -- VirtualHMDDeviceEnumerator -----
VirtualHMDDeviceEnumerator::VirtualHMDDeviceEnumerator()
    : DeviceEnumerator(CommonDeviceState::VirtualHMD)
{
	m_deviceType= CommonDeviceState::VirtualHMD;
    m_device_index= 0;

	const HMDManagerConfig &cfg= DeviceManager::getInstance()->m_hmd_manager->getConfig();
    m_device_count= static_cast<int>(cfg.virtual_hmds.size());

	if (!is_valid())
	{
		next();
	}
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
        const HMDManagerConfig &cfg= DeviceManager::getInstance()->m_hmd_manager->getConfig();

        m_current_device_identifier= cfg.virtual_hmds[m_device_index];
    }

	return foundValid;
}
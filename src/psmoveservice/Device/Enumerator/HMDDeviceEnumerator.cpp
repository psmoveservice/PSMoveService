// -- includes -----
#include "HMDDeviceEnumerator.h"
#include "ServerUtility.h"
#include "assert.h"
#include "hidapi.h"
#include "string.h"

// -- private definitions -----
#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

// -- macros ----
#define MAX_HMD_TYPE_INDEX                  GET_DEVICE_TYPE_INDEX(CommonDeviceState::SUPPORTED_HMD_TYPE_COUNT)

// -- globals -----
USBDeviceInfo g_supported_hmd_infos[MAX_HMD_TYPE_INDEX] = {
    { 0x054c, 0x09af }, // Sony Morpheus
};

// -- HMDDeviceEnumerator -----
HMDDeviceEnumerator::HMDDeviceEnumerator()
    : DeviceEnumerator(CommonDeviceState::Morpheus)
	, devs(nullptr)
	, cur_dev(nullptr)
{
    assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_HMD_TYPE_INDEX);

	USBDeviceInfo &dev_info = g_supported_hmd_infos[GET_DEVICE_TYPE_INDEX(m_deviceType)];
	devs = hid_enumerate(dev_info.vendor_id, dev_info.product_id);
	cur_dev = devs;

	if (!is_valid())
	{
		next();
	}
}

HMDDeviceEnumerator::HMDDeviceEnumerator(CommonDeviceState::eDeviceType deviceType)
    : DeviceEnumerator(deviceType)
	, devs(nullptr)
	, cur_dev(nullptr)
{
    assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_HMD_TYPE_INDEX);

	USBDeviceInfo &dev_info = g_supported_hmd_infos[GET_DEVICE_TYPE_INDEX(m_deviceType)];
	devs = hid_enumerate(dev_info.vendor_id, dev_info.product_id);
	cur_dev = devs;

	if (!is_valid())
	{
		next();
	}
}

HMDDeviceEnumerator::~HMDDeviceEnumerator()
{
	if (devs != nullptr)
	{
		hid_free_enumeration(devs);
	}
}

const char *HMDDeviceEnumerator::get_path() const
{
	return (cur_dev != nullptr) ? cur_dev->path : nullptr;
}

bool HMDDeviceEnumerator::get_serial_number(char *out_mb_serial, const size_t mb_buffer_size) const
{
	bool success = false;

	if (cur_dev != nullptr && cur_dev->serial_number != nullptr)
	{
		success = ServerUtility::convert_wcs_to_mbs(cur_dev->serial_number, out_mb_serial, mb_buffer_size);
	}

	return success;
}

bool HMDDeviceEnumerator::is_valid() const
{
	return cur_dev != nullptr;
}

bool HMDDeviceEnumerator::next()
{
	bool foundValid = false;

	while (!foundValid && m_deviceType < CommonDeviceState::SUPPORTED_HMD_TYPE_COUNT)
	{
		if (cur_dev != nullptr)
		{
			cur_dev = cur_dev->next;
			foundValid = is_valid();
		}

		// If there are more device types to scan
		// move on to the next vid/pid device enumeration
		if (!foundValid && cur_dev == nullptr)
		{
			m_deviceType = static_cast<CommonDeviceState::eDeviceType>(m_deviceType + 1);

			// Free any previous enumeration
			if (devs != nullptr)
			{
				hid_free_enumeration(devs);
				cur_dev = nullptr;
				devs = nullptr;
			}

			if (GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_HMD_TYPE_INDEX)
			{
				USBDeviceInfo &dev_info = g_supported_hmd_infos[GET_DEVICE_TYPE_INDEX(m_deviceType)];

				// Create a new HID enumeration
				devs = hid_enumerate(dev_info.vendor_id, dev_info.product_id);
				cur_dev = devs;
				foundValid = is_valid();
			}
		}
	}

	return foundValid;
}
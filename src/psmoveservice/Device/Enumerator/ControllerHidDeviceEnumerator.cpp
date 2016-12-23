// -- includes -----
#include "ControllerHidDeviceEnumerator.h"
#include "ServerUtility.h"
#include "USBDeviceInfo.h"
#include "assert.h"
#include "hidapi.h"
#include "string.h"

// -- private definitions -----
#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

// -- macros ----
#define MAX_CONTROLLER_TYPE_INDEX           GET_DEVICE_TYPE_INDEX(CommonDeviceState::SUPPORTED_CONTROLLER_TYPE_COUNT)

// -- globals -----
struct HIDApiDeviceFilter
{
	USBDeviceFilter filter;
	bool bHIDApiSupported;
};


HIDApiDeviceFilter g_supported_controller_infos[MAX_CONTROLLER_TYPE_INDEX] = {
	{ 0x054c, 0x03d5, true}, // PSMove
	{ 0x054c, 0x0268, false}, // PSNavi/DualShock3 (sadly these controllers don't properly support HID
	{ 0x054c, 0x05C4, true}, // PSDualShock4
};

// -- ControllerHidDeviceEnumerator -----
ControllerHidDeviceEnumerator::ControllerHidDeviceEnumerator()
	: DeviceEnumerator(CommonDeviceState::PSMove)
	, devs(nullptr)
	, cur_dev(nullptr)
{
	assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_CONTROLLER_TYPE_INDEX);

	HIDApiDeviceFilter &dev_info = g_supported_controller_infos[GET_DEVICE_TYPE_INDEX(m_deviceType)];
	devs = hid_enumerate(dev_info.filter.vendor_id, dev_info.filter.product_id);
	cur_dev = devs;

	if (!is_valid())
	{
		next();
	}
}

ControllerHidDeviceEnumerator::ControllerHidDeviceEnumerator(
	CommonDeviceState::eDeviceType deviceType)
	: DeviceEnumerator(deviceType)
	, devs(nullptr)
	, cur_dev(nullptr)
{
	assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_CONTROLLER_TYPE_INDEX);

	HIDApiDeviceFilter &dev_info = g_supported_controller_infos[GET_DEVICE_TYPE_INDEX(m_deviceType)];
	devs = hid_enumerate(dev_info.filter.vendor_id, dev_info.filter.product_id);
	cur_dev = devs;

	if (!is_valid())
	{
		next();
	}
}

ControllerHidDeviceEnumerator::~ControllerHidDeviceEnumerator()
{
	if (devs != nullptr)
	{
		hid_free_enumeration(devs);
	}
}

int ControllerHidDeviceEnumerator::get_vendor_id() const
{
	return (cur_dev != nullptr) ? cur_dev->vendor_id : -1;
}

int ControllerHidDeviceEnumerator::get_product_id() const
{
	return (cur_dev != nullptr) ? cur_dev->product_id : -1;
}

const char *ControllerHidDeviceEnumerator::get_path() const
{
	return (cur_dev != nullptr) ? cur_dev->path : nullptr;
}

bool ControllerHidDeviceEnumerator::get_serial_number(char *out_mb_serial, const size_t mb_buffer_size) const
{
	bool success = false;

	if (cur_dev != nullptr && cur_dev->serial_number != nullptr)
	{
		success = ServerUtility::convert_wcs_to_mbs(cur_dev->serial_number, out_mb_serial, mb_buffer_size);
	}

	return success;
}

bool ControllerHidDeviceEnumerator::is_valid() const
{
	bool bIsValid = cur_dev != nullptr;

#ifdef _WIN32
	/**
	* Windows Quirk: Each psmove dev is enumerated 3 times.
	* The one with "&col01#" in the path is the one we will get most of our data from. Only count this one.
	* The one with "&col02#" in the path is the one we will get the bluetooth address from.
	**/
	if (bIsValid && m_deviceType == CommonDeviceState::PSMove && strstr(cur_dev->path, "&col01#") == nullptr)
	{
		bIsValid = false;
	}
#endif

	return bIsValid;
}

bool ControllerHidDeviceEnumerator::next()
{
	bool foundValid = false;

	while (!foundValid && m_deviceType < CommonDeviceState::SUPPORTED_CONTROLLER_TYPE_COUNT)
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

			if (GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_CONTROLLER_TYPE_INDEX)
			{
				HIDApiDeviceFilter &dev_info = g_supported_controller_infos[GET_DEVICE_TYPE_INDEX(m_deviceType)];

				if (dev_info.bHIDApiSupported)
				{
					// Create a new HID enumeration
					devs = hid_enumerate(dev_info.filter.vendor_id, dev_info.filter.product_id);
					cur_dev = devs;
					foundValid = is_valid();
				}
			}
		}
	}

	return foundValid;
}
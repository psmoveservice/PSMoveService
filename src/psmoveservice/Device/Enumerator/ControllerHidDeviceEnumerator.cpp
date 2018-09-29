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

// -- globals -----
struct HIDApiDeviceFilter
{
	USBDeviceFilter filter;
	CommonDeviceState::eDeviceType deviceType;
	bool bHIDApiSupported;
};

const int MAX_HID_CONTROLLER_TYPE_COUNT= 4;
HIDApiDeviceFilter g_supported_hid_controller_infos[MAX_HID_CONTROLLER_TYPE_COUNT] = {
	{ {0x054c, 0x03d5}, CommonDeviceState::PSMove, true}, // PSMove
	{ {0x054c, 0x0C5E}, CommonDeviceState::PSMove, true}, // PSMove (newer model "CECH-ZCM2U")
	{ {0x054c, 0x0268}, CommonDeviceState::PSNavi, false}, // PSNavi/DualShock3 (sadly these controllers don't properly support HID
	{ {0x054c, 0x05C4}, CommonDeviceState::PSDualShock4, true} // PSDualShock4
};

// -- ControllerHidDeviceEnumerator -----
ControllerHidDeviceEnumerator::ControllerHidDeviceEnumerator()
	: DeviceEnumerator()
	, m_HIDdevices(nullptr)
	, m_currentHIDDevice(nullptr)
	, m_enumeratorIndex(-1)
{
	next();
}

ControllerHidDeviceEnumerator::ControllerHidDeviceEnumerator(
	CommonDeviceState::eDeviceType deviceTypeFilter)
	: DeviceEnumerator(deviceTypeFilter)
	, m_HIDdevices(nullptr)
	, m_currentHIDDevice(nullptr)
	, m_enumeratorIndex(-1)
{
	next();
}

ControllerHidDeviceEnumerator::~ControllerHidDeviceEnumerator()
{
	if (m_HIDdevices != nullptr)
	{
		hid_free_enumeration(m_HIDdevices);
	}
}

int ControllerHidDeviceEnumerator::get_vendor_id() const
{
	return (m_currentHIDDevice != nullptr) ? m_currentHIDDevice->vendor_id : -1;
}

int ControllerHidDeviceEnumerator::get_product_id() const
{
	return (m_currentHIDDevice != nullptr) ? m_currentHIDDevice->product_id : -1;
}

const char *ControllerHidDeviceEnumerator::get_path() const
{
	return (m_currentHIDDevice != nullptr) ? m_currentHIDDevice->path : nullptr;
}

bool ControllerHidDeviceEnumerator::get_serial_number(char *out_mb_serial, const size_t mb_buffer_size) const
{
	bool success = false;

	if (m_currentHIDDevice != nullptr && m_currentHIDDevice->serial_number != nullptr)
	{
		success = ServerUtility::convert_wcs_to_mbs(m_currentHIDDevice->serial_number, out_mb_serial, mb_buffer_size);
	}

	return success;
}

bool ControllerHidDeviceEnumerator::is_valid() const
{
	bool bIsValid = m_currentHIDDevice != nullptr;

#ifdef _WIN32
	/**
	* Windows Quirk: Each psmove dev is enumerated 3 times.
	* The one with "&col01#" in the path is the one we will get most of our data from. Only count this one.
	* The one with "&col02#" in the path is the one we will get the bluetooth address from.
	**/
	if (bIsValid && m_deviceType == CommonDeviceState::PSMove && strstr(m_currentHIDDevice->path, "&col01#") == nullptr)
	{
		bIsValid = false;
	}
#endif

	return bIsValid;
}

bool ControllerHidDeviceEnumerator::next()
{
	bool foundValid = false;

	while (!foundValid && m_enumeratorIndex < MAX_HID_CONTROLLER_TYPE_COUNT)
	{
		if (m_currentHIDDevice != nullptr)
		{
			m_currentHIDDevice = m_currentHIDDevice->next;
			foundValid = is_valid();
		}

		// If there are more device types to scan
		// move on to the next vid/pid device enumeration
		if (!foundValid && m_currentHIDDevice == nullptr)
		{
			++m_enumeratorIndex;

			// Free any previous enumeration
			if (m_HIDdevices != nullptr)
			{
				hid_free_enumeration(m_HIDdevices);
				m_currentHIDDevice = nullptr;
				m_HIDdevices = nullptr;
			}

			if (m_enumeratorIndex < MAX_HID_CONTROLLER_TYPE_COUNT)
			{
				HIDApiDeviceFilter &dev_info = g_supported_hid_controller_infos[m_enumeratorIndex];
				m_deviceType = dev_info.deviceType;

				if (dev_info.bHIDApiSupported &&
					(m_deviceType == m_deviceTypeFilter || m_deviceTypeFilter == CommonDeviceState::INVALID_DEVICE_TYPE))
				{
					// Create a new HID enumeration
					m_HIDdevices = hid_enumerate(dev_info.filter.vendor_id, dev_info.filter.product_id);
					m_currentHIDDevice = m_HIDdevices;
					foundValid = is_valid();
				}
			}
		}
	}

	return foundValid;
}

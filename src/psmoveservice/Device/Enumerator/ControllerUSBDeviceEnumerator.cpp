// -- includes -----
#include "ControllerUSBDeviceEnumerator.h"
#include "ServerUtility.h"
#include "USBDeviceManager.h"
#include "assert.h"
#include "string.h"

// -- private definitions -----
#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

// -- macros ----
#define MAX_CONTROLLER_TYPE_INDEX               GET_DEVICE_TYPE_INDEX(CommonDeviceState::SUPPORTED_CONTROLLER_TYPE_COUNT)

// -- globals -----
struct GamepadFilter
{
	USBDeviceFilter filter;
	bool bUSBApiSupported;
};

// NOTE: This list must match the controller order in CommonDeviceState::eDeviceType
GamepadFilter g_supported_libusb_controller_filters[MAX_CONTROLLER_TYPE_INDEX] = {
	{ 0x054c, 0x03d5, false }, // PSMove
	{ 0x054c, 0x042F, true }, // PSNavi
	{ 0x054c, 0x05C4, false }, // PSDualShock4
};

// -- private prototypes -----
static bool get_usb_controller_type(USBDeviceEnumerator* enumerator, CommonDeviceState::eDeviceType &out_device_type);

// -- methods -----
ControllerUSBDeviceEnumerator::ControllerUSBDeviceEnumerator()
	: DeviceEnumerator(CommonDeviceState::PSMove)
	, m_usb_enumerator(nullptr)
	, m_controllerIndex(0)
{
	USBDeviceManager *usbRequestMgr = USBDeviceManager::getInstance();

	assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_CONTROLLER_TYPE_INDEX);
	m_usb_enumerator = usb_device_enumerator_allocate();

	if (get_usb_controller_type(m_usb_enumerator, m_deviceType))
	{
		// Cache the current usb path
		usb_device_enumerator_get_path(m_usb_enumerator, m_currentUSBPath, sizeof(m_currentUSBPath));		
	}
	else
	{
		next();
	}
}

ControllerUSBDeviceEnumerator::ControllerUSBDeviceEnumerator(CommonDeviceState::eDeviceType deviceType)
	: DeviceEnumerator(deviceType)
	, m_usb_enumerator(nullptr)
	, m_controllerIndex(0)
{
	USBDeviceManager *usbRequestMgr = USBDeviceManager::getInstance();

	assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_CONTROLLER_TYPE_INDEX);
	m_usb_enumerator = usb_device_enumerator_allocate();

	// If the first USB device handle isn't a tracker, move on to the next device
	if (get_usb_controller_type(m_usb_enumerator, m_deviceType))
	{
		// Cache the current USB path
		usb_device_enumerator_get_path(m_usb_enumerator, m_currentUSBPath, sizeof(m_currentUSBPath));
	}
	else
	{
		next();
	}
}

ControllerUSBDeviceEnumerator::~ControllerUSBDeviceEnumerator()
{
	if (m_usb_enumerator != nullptr)
	{
		usb_device_enumerator_free(m_usb_enumerator);
	}
}


const char *ControllerUSBDeviceEnumerator::get_path() const
{
	const char *result = nullptr;

	if (is_valid())
	{
		// Return a pointer to our member variable that has the path cached
		result = m_currentUSBPath;
	}

	return result;
}

int ControllerUSBDeviceEnumerator::get_vendor_id() const
{
	USBDeviceFilter devInfo;
	int vendor_id = -1;

	if (is_valid() && usb_device_enumerator_get_filter(m_usb_enumerator, devInfo))
	{
		vendor_id = devInfo.vendor_id;
	}

	return vendor_id;
}

int ControllerUSBDeviceEnumerator::get_product_id() const
{
	USBDeviceFilter devInfo;
	int product_id = -1;

	if (is_valid() && usb_device_enumerator_get_filter(m_usb_enumerator, devInfo))
	{
		product_id = devInfo.product_id;
	}

	return product_id;
}

bool ControllerUSBDeviceEnumerator::is_valid() const
{
	return m_usb_enumerator != nullptr && usb_device_enumerator_is_valid(m_usb_enumerator);
}

bool ControllerUSBDeviceEnumerator::next()
{
	USBDeviceManager *usbRequestMgr = USBDeviceManager::getInstance();
	bool foundValid = false;

	while (is_valid() && !foundValid)
	{
		usb_device_enumerator_next(m_usb_enumerator);

		if (is_valid() && get_usb_controller_type(m_usb_enumerator, m_deviceType))
		{
			// Cache the path to the device
			usb_device_enumerator_get_path(m_usb_enumerator, m_currentUSBPath, sizeof(m_currentUSBPath));
			foundValid = true;
			break;
		}
	}

	if (foundValid)
	{
		++m_controllerIndex;
	}

	return foundValid;
}

//-- private methods -----
static bool get_usb_controller_type(USBDeviceEnumerator *enumerator, CommonDeviceState::eDeviceType &out_device_type)
{
	USBDeviceFilter devInfo;
	bool bIsValidDevice = false;

	if (usb_device_enumerator_get_filter(enumerator, devInfo))
	{
		// See if the next filtered device is a controller type that we care about
		for (int tracker_type_index = 0; tracker_type_index < MAX_CONTROLLER_TYPE_INDEX; ++tracker_type_index)
		{
			const GamepadFilter &supported_type = g_supported_libusb_controller_filters[tracker_type_index];

			if (supported_type.bUSBApiSupported &&
				devInfo.product_id == supported_type.filter.product_id &&
				devInfo.vendor_id == supported_type.filter.vendor_id)
			{
				out_device_type = static_cast<CommonDeviceState::eDeviceType>(CommonDeviceState::Controller + tracker_type_index);
				bIsValidDevice = true;
				break;
			}
		}
	}

	return bIsValidDevice;
}
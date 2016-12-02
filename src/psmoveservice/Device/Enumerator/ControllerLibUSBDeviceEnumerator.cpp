// -- includes -----
#include "ControllerLibUSBDeviceEnumerator.h"
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
struct LibUSBDeviceFilter
{
	USBDeviceFilter filter;
	bool bLibUSBSupported;
};

// NOTE: This list must match the controller order in CommonDeviceState::eDeviceType
LibUSBDeviceFilter g_supported_libusb_controller_filters[MAX_CONTROLLER_TYPE_INDEX] = {
	{ 0x054c, 0x03d5, false }, // PSMove
	{ 0x054c, 0x042F, true }, // PSNavi
	{ 0x054c, 0x05C4, false }, // PSDualShock4
};

// -- private prototypes -----
static bool get_usb_controller_type(t_usb_device_handle usb_device_handle, CommonDeviceState::eDeviceType &out_device_type);

// -- methods -----
ControllerLibUSBDeviceEnumerator::ControllerLibUSBDeviceEnumerator()
	: DeviceEnumerator(CommonDeviceState::PSNavi)
	, m_USBDeviceHandle(k_invalid_usb_device_handle)
	, m_controllerIndex(0)
{
	USBDeviceManager *usbRequestMgr = USBDeviceManager::getInstance();

	assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_CONTROLLER_TYPE_INDEX);
	m_USBDeviceHandle = usbRequestMgr->getFirstUSBDeviceHandle();

	// If the first USB device handle isn't a tracker, move on to the next device
	if (get_usb_controller_type(m_USBDeviceHandle, m_deviceType))
	{
		// Cache the current usb path
		usbRequestMgr->getUSBDevicePath(m_USBDeviceHandle, m_currentUSBPath, sizeof(m_currentUSBPath));
	}
	else
	{
		next();
	}
}

ControllerLibUSBDeviceEnumerator::ControllerLibUSBDeviceEnumerator(CommonDeviceState::eDeviceType deviceType)
	: DeviceEnumerator(deviceType)
	, m_USBDeviceHandle(k_invalid_usb_device_handle)
	, m_controllerIndex(0)
{
	USBDeviceManager *usbRequestMgr = USBDeviceManager::getInstance();

	assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_CONTROLLER_TYPE_INDEX);
	m_USBDeviceHandle = usbRequestMgr->getFirstUSBDeviceHandle();

	// If the first USB device handle isn't a tracker, move on to the next device
	if (get_usb_controller_type(m_USBDeviceHandle, m_deviceType))
	{
		// Cache the current usb path
		usbRequestMgr->getUSBDevicePath(m_USBDeviceHandle, m_currentUSBPath, sizeof(m_currentUSBPath));
	}
	else
	{
		next();
	}
}

const char *ControllerLibUSBDeviceEnumerator::get_path() const
{
	const char *result = nullptr;

	if (is_valid())
	{
		// Return a pointer to our member variable that has the path cached
		result = m_currentUSBPath;
	}

	return result;
}

bool ControllerLibUSBDeviceEnumerator::is_valid() const
{
	return m_USBDeviceHandle != k_invalid_usb_device_handle;
}

bool ControllerLibUSBDeviceEnumerator::next()
{
	USBDeviceManager *usbRequestMgr = USBDeviceManager::getInstance();
	bool foundValid = false;

	while (is_valid() && !foundValid)
	{
		m_USBDeviceHandle = usbRequestMgr->getNextUSBDeviceHandle(m_USBDeviceHandle);

		if (is_valid() && get_usb_controller_type(m_USBDeviceHandle, m_deviceType))
		{
			// Cache the path to the device
			usbRequestMgr->getUSBDevicePath(m_USBDeviceHandle, m_currentUSBPath, sizeof(m_currentUSBPath));
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
static bool get_usb_controller_type(t_usb_device_handle usb_device_handle, CommonDeviceState::eDeviceType &out_device_type)
{
	USBDeviceFilter devInfo;
	bool bIsValidDevice = false;

	if (USBDeviceManager::getInstance()->getUSBDeviceInfo(usb_device_handle, devInfo))
	{
		// See if the next filtered device is a camera that we care about
		for (int tracker_type_index = 0; tracker_type_index < MAX_CONTROLLER_TYPE_INDEX; ++tracker_type_index)
		{
			const LibUSBDeviceFilter &supported_type = g_supported_libusb_controller_filters[tracker_type_index];

			if (supported_type.bLibUSBSupported &&
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
// -- includes -----
#include "TrackerDeviceEnumerator.h"
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
#define MAX_CAMERA_TYPE_INDEX               GET_DEVICE_TYPE_INDEX(CommonDeviceState::SUPPORTED_CAMERA_TYPE_COUNT)

// -- globals -----
// NOTE: This list must match the tracker order in CommonDeviceState::eDeviceType
USBDeviceFilter k_supported_tracker_infos[MAX_CAMERA_TYPE_INDEX] = {
    { 0x1415, 0x2000 }, // PS3Eye
    //{ 0x05a9, 0x058a }, // PS4 Camera - TODO
};

// -- private prototypes -----
static bool get_usb_tracker_type(USBDeviceEnumerator* enumerator, CommonDeviceState::eDeviceType &out_device_type);

// -- methods -----
TrackerDeviceEnumerator::TrackerDeviceEnumerator()
	: DeviceEnumerator(CommonDeviceState::PS3EYE)
	, m_usb_enumerator(nullptr)
    , m_cameraIndex(0)
{
	USBDeviceManager *usbRequestMgr = USBDeviceManager::getInstance();

	assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_CAMERA_TYPE_INDEX);
	m_usb_enumerator = usb_device_enumerator_allocate();

	// If the first USB device handle isn't a tracker, move on to the next device
	if (get_usb_tracker_type(m_usb_enumerator, m_deviceType))
	{
		// Cache the current usb path
		usb_device_enumerator_get_path(m_usb_enumerator, m_currentUSBPath, sizeof(m_currentUSBPath));
	}
	else
	{
		next();
	}
}

TrackerDeviceEnumerator::~TrackerDeviceEnumerator()
{
	if (m_usb_enumerator != nullptr)
	{
		usb_device_enumerator_free(m_usb_enumerator);
	}
}

int TrackerDeviceEnumerator::get_vendor_id() const
{
	USBDeviceFilter devInfo;
	int vendor_id = -1;

	if (is_valid() && usb_device_enumerator_get_filter(m_usb_enumerator, devInfo))
	{
		vendor_id = devInfo.vendor_id;
	}

	return vendor_id;
}

int TrackerDeviceEnumerator::get_product_id() const
{
	USBDeviceFilter devInfo;
	int product_id = -1;

	if (is_valid() && usb_device_enumerator_get_filter(m_usb_enumerator, devInfo))
	{
		product_id = devInfo.product_id;
	}

	return product_id;
}

const char *TrackerDeviceEnumerator::get_path() const
{
    const char *result = nullptr;

    if (is_valid())
    {
        // Return a pointer to our member variable that has the path cached
        result= m_currentUSBPath;
    }

    return result;
}

bool TrackerDeviceEnumerator::is_valid() const
{
	return m_usb_enumerator != nullptr && usb_device_enumerator_is_valid(m_usb_enumerator);
}

bool TrackerDeviceEnumerator::next()
{
	USBDeviceManager *usbRequestMgr = USBDeviceManager::getInstance();
	bool foundValid = false;

	while (is_valid() && !foundValid)
	{
		usb_device_enumerator_next(m_usb_enumerator);

		if (is_valid() && get_usb_tracker_type(m_usb_enumerator, m_deviceType))
		{
			// Cache the path to the device
			usb_device_enumerator_get_path(m_usb_enumerator, m_currentUSBPath, sizeof(m_currentUSBPath));
			foundValid = true;
			break;
		}
	}

	if (foundValid)
	{
		++m_cameraIndex;
	}

	return foundValid;
}

//-- private methods -----
static bool get_usb_tracker_type(USBDeviceEnumerator *enumerator, CommonDeviceState::eDeviceType &out_device_type)
{
	USBDeviceFilter devInfo;
	bool bIsValidDevice = false;

	if (usb_device_enumerator_get_filter(enumerator, devInfo))
	{
		// See if the next filtered device is a camera that we care about
		for (int tracker_type_index = 0; tracker_type_index < MAX_CAMERA_TYPE_INDEX; ++tracker_type_index)
		{
			const USBDeviceFilter &supported_type = k_supported_tracker_infos[tracker_type_index];

			if (devInfo.product_id == supported_type.product_id &&
				devInfo.vendor_id == supported_type.vendor_id)
			{
				out_device_type = static_cast<CommonDeviceState::eDeviceType>(CommonDeviceState::TrackingCamera + tracker_type_index);
				bIsValidDevice = true;
				break;
			}
		}
	}

	return bIsValidDevice;
}
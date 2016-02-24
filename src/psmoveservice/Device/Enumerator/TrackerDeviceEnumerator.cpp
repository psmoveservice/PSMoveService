// -- includes -----
#include "TrackerDeviceEnumerator.h"
#include "ServerUtility.h"
#include "assert.h"
#include "libusb.h"
#include "string.h"

// -- private definitions -----
#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

// -- macros ----
#define MAX_CAMERA_TYPE_INDEX               GET_DEVICE_TYPE_INDEX(CommonDeviceState::SUPPORTED_CAMERA_TYPE_COUNT)

// -- globals -----
USBDeviceInfo g_supported_tracker_infos[MAX_CAMERA_TYPE_INDEX] = {
    { 0x1415, 0x2000 }, // PS3Eye
    //{0x045e, 0x02ae}, // V1 Kinect
};

// -- methods -----
TrackerDeviceEnumerator::TrackerDeviceEnumerator()
    : DeviceEnumerator(CommonDeviceState::PS3EYE)
    , usb_context(nullptr)
    , devs(nullptr)
    , cur_dev(nullptr)
    , dev_index(0)
    , dev_count(0)
    , camera_index(-1)
{
    assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_CAMERA_TYPE_INDEX);

    libusb_init(&usb_context);
    dev_count = static_cast<int>(libusb_get_device_list(usb_context, &devs));
    cur_dev = (devs != nullptr) ? devs[0] : nullptr;
    camera_index = 0;

    if (!is_valid())
    {
        camera_index = -1;
        next();
    }
    else
    {
        camera_index = 0;
    }
}

TrackerDeviceEnumerator::TrackerDeviceEnumerator(CommonDeviceState::eDeviceType deviceType)
    : DeviceEnumerator(deviceType)
    , devs(nullptr)
    , cur_dev(nullptr)
    , dev_index(0)
    , dev_count(0)
    , camera_index(-1)
{
    assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_CAMERA_TYPE_INDEX);

    libusb_init(&usb_context);
    dev_count = static_cast<int>(libusb_get_device_list(usb_context, &devs));
    cur_dev = (devs != nullptr) ? devs[0] : nullptr;

    if (!is_valid())
    {
        camera_index = -1;
        next();
    }
    else
    {
        camera_index = 0;
    }
}

TrackerDeviceEnumerator::~TrackerDeviceEnumerator()
{
    if (devs != nullptr)
    {
        libusb_free_device_list(devs, 1);
    }

    libusb_exit(usb_context);
}

const char *TrackerDeviceEnumerator::get_path() const
{
    const char *result = nullptr;

    if (cur_dev != nullptr)
    {
        struct libusb_device_descriptor dev_desc;
        libusb_get_device_descriptor(cur_dev, &dev_desc);

        _snprintf(
            (char *)(cur_path), sizeof(cur_path),
            "USB\\VID_%04X&PID_%04X\\%d\\%d",
            dev_desc.idVendor, dev_desc.idProduct, dev_index, camera_index);

        result = cur_path;
    }

    return result;
}

bool TrackerDeviceEnumerator::is_valid() const
{
    bool bIsValid = false;

    if (cur_dev != nullptr)
    {
        USBDeviceInfo &dev_info = g_supported_tracker_infos[GET_DEVICE_TYPE_INDEX(m_deviceType)];

        struct libusb_device_descriptor dev_desc;
        libusb_get_device_descriptor(cur_dev, &dev_desc);

        if (dev_desc.idVendor == dev_info.vendor_id &&
            dev_desc.idProduct == dev_info.product_id)
        {
            libusb_device_handle *devhandle;
            int err = libusb_open(cur_dev, &devhandle);

            if (err == 0)
            {
                libusb_close(devhandle);
                bIsValid = true;
            }
        }
    }

    return bIsValid;
}

bool TrackerDeviceEnumerator::next()
{
    bool foundValid = false;

    while (cur_dev != nullptr && !foundValid)
    {
        ++dev_index;
        cur_dev = (dev_index < dev_count) ? devs[dev_index] : nullptr;
        foundValid = is_valid();

        // If there are more device types to scan
        // move on to the next vid/pid device enumeration
        if (cur_dev == nullptr &&
            GET_DEVICE_TYPE_CLASS(m_deviceType + 1) == CommonDeviceState::TrackingCamera &&
            (m_deviceType + 1) < CommonDeviceState::SUPPORTED_CAMERA_TYPE_COUNT)
        {
            m_deviceType = static_cast<CommonDeviceState::eDeviceType>(m_deviceType + 1);

            // Reset the device iterator
            dev_index = 0;
            cur_dev = (devs != nullptr) ? devs[0] : nullptr;
            foundValid = is_valid();
        }
    }

    if (foundValid)
    {
        ++camera_index;
    }

    return foundValid;
}
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
    //{0x2833, 0x0201 }, // RiftDK2 Sensor
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

    if (!recompute_current_device_validity())
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
    , dev_valid(false)
{
    assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_CAMERA_TYPE_INDEX);

    memset(dev_port_numbers, 255, sizeof(dev_port_numbers));

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

        snprintf(
            (char *)(cur_path), sizeof(cur_path),
            "USB\\VID_%04X&PID_%04X\\%d\\%d",
            dev_desc.idVendor, dev_desc.idProduct, dev_index, camera_index);

        result = cur_path;
    }

    return result;
}

bool TrackerDeviceEnumerator::is_valid() const
{
    return dev_valid;
}

bool TrackerDeviceEnumerator::recompute_current_device_validity()
{
    dev_valid = false;

    if (cur_dev != nullptr)
    {
        USBDeviceInfo &dev_info = g_supported_tracker_infos[GET_DEVICE_TYPE_INDEX(m_deviceType)];
        struct libusb_device_descriptor dev_desc;

        int libusb_result = libusb_get_device_descriptor(cur_dev, &dev_desc);

        if (libusb_result == 0 &&
            dev_desc.idVendor == dev_info.vendor_id &&
            dev_desc.idProduct == dev_info.product_id)
        {
            uint8_t port_numbers[MAX_USB_DEVICE_PORT_PATH];
            
            memset(port_numbers, 0, sizeof(port_numbers));
            int elements_filled= libusb_get_port_numbers(cur_dev, port_numbers, MAX_USB_DEVICE_PORT_PATH);

            if (elements_filled > 0)
            {
                // Make sure this device is actually different from the last device we looked at
                // (i.e. has a different device port path)
                if (memcmp(port_numbers, dev_port_numbers, sizeof(port_numbers)) != 0)
                {
                    libusb_device_handle *devhandle;

                    // Finally need to test that we can actually open the device
                    // (or see that device is already open)
                    libusb_result = libusb_open(cur_dev, &devhandle);
                    if (libusb_result == LIBUSB_SUCCESS || libusb_result == LIBUSB_ERROR_ACCESS)
                    {
                        if (libusb_result == LIBUSB_SUCCESS)
                        {
                            libusb_close(devhandle);
                        }

                        // Cache the port number for the last valid device found
                        memcpy(dev_port_numbers, port_numbers, sizeof(port_numbers));

                        dev_valid = true;
                    }
                }
            }
        }
    }

    return dev_valid;
}

bool TrackerDeviceEnumerator::next()
{
    bool foundValid = false;

    while (cur_dev != nullptr && !foundValid)
    {
        ++dev_index;
        cur_dev = (dev_index < dev_count) ? devs[dev_index] : nullptr;
        foundValid = recompute_current_device_validity();

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
            foundValid = recompute_current_device_validity();
        }
    }

    if (foundValid)
    {
        ++camera_index;
    }

    return foundValid;
}
// -- includes -----
#include "DeviceEnumerator.h"
#include "ServerUtility.h"
#include "assert.h"
#include "hidapi.h"
#include "libusb.h"
#include "string.h"

// -- private definitions -----
struct USBDeviceInfo
{
    unsigned short vendor_id;
    unsigned short product_id;
};


#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

// -- macros ----
#define GET_DEVICE_TYPE_INDEX(device_type)  (device_type & 0x0f)
#define GET_DEVICE_TYPE_CLASS(device_type)  (device_type & 0xf0)
#define MAX_CONTROLLER_TYPE_INDEX           GET_DEVICE_TYPE_INDEX(CommonDeviceState::SUPPORTED_CONTROLLER_TYPE_COUNT)
#define MAX_CAMERA_TYPE_INDEX               GET_DEVICE_TYPE_INDEX(CommonDeviceState::SUPPORTED_CAMERA_TYPE_COUNT)
#define MAX_HMD_TYPE_INDEX                  GET_DEVICE_TYPE_INDEX(CommonDeviceState::SUPPORTED_HMD_TYPE_COUNT)

// -- globals -----
USBDeviceInfo g_supported_controller_infos[MAX_CONTROLLER_TYPE_INDEX] = {
    {0x054c, 0x03d5}, // PSMove
    {0x054c, 0x042F}, // PSNavi
    //{0x054c, 0x0268}, // PSDualShock3
};

USBDeviceInfo g_supported_tracker_infos[MAX_CAMERA_TYPE_INDEX] = {
    { 0x1415, 0x2000 }, // PS3Eye
    //{0x045e, 0x02ae}, // V1 Kinect
};

USBDeviceInfo g_supported_hmd_infos[MAX_HMD_TYPE_INDEX] = {
    { 0x2833, 0x0021 }, // DK2
};

// -- DeviceEnumerator -----
DeviceEnumerator::DeviceEnumerator(CommonDeviceState::eDeviceType deviceType)
    : m_deviceType(deviceType)
{
}

DeviceEnumerator::~DeviceEnumerator()
{
    
}

// -- ControllerDeviceEnumerator -----
ControllerDeviceEnumerator::ControllerDeviceEnumerator() 
    : DeviceEnumerator(CommonDeviceState::PSMove)
    , devs(nullptr)
    , cur_dev(nullptr)
{
    assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_CONTROLLER_TYPE_INDEX);

    USBDeviceInfo &dev_info = g_supported_controller_infos[GET_DEVICE_TYPE_INDEX(m_deviceType)];
    devs = hid_enumerate(dev_info.vendor_id, dev_info.product_id);
    cur_dev = devs;

    if (!is_valid())
    {
        next();
    }
}

ControllerDeviceEnumerator::ControllerDeviceEnumerator(CommonDeviceState::eDeviceType deviceType)
    : DeviceEnumerator(deviceType)
    , devs(nullptr)
    , cur_dev(nullptr)
{
    assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_CONTROLLER_TYPE_INDEX);

    USBDeviceInfo &dev_info = g_supported_controller_infos[GET_DEVICE_TYPE_INDEX(m_deviceType)];
    devs = hid_enumerate(dev_info.vendor_id, dev_info.product_id);
    cur_dev = devs;

    if (!is_valid())
    {
        next();
    }
}

ControllerDeviceEnumerator::~ControllerDeviceEnumerator()
{
    if (devs != nullptr)
    {
        hid_free_enumeration(devs);
    }
}

const char *ControllerDeviceEnumerator::get_path() const
{
    return (cur_dev != nullptr) ? cur_dev->path : nullptr;
}

bool ControllerDeviceEnumerator::get_serial_number(char *out_mb_serial, const size_t mb_buffer_size) const
{
    bool success= false;

    if (cur_dev != nullptr && cur_dev->serial_number != nullptr)
    {
        success= ServerUtility::convert_wcs_to_mbs(cur_dev->serial_number, out_mb_serial, mb_buffer_size);
    }

    return success;
}

bool ControllerDeviceEnumerator::is_valid() const
{
    bool bIsValid= cur_dev != nullptr;

#ifdef _WIN32
    /**
    * Windows Quirk: Each psmove dev is enumerated 3 times.
    * The one with "&col01#" in the path is the one we will get most of our data from. Only count this one.
    * The one with "&col02#" in the path is the one we will get the bluetooth address from.
    **/
    if (bIsValid && m_deviceType == CommonDeviceState::PSMove && strstr(cur_dev->path, "&col01#") == nullptr)
    {
        bIsValid= false;
    }
#endif

    return bIsValid;
}

bool ControllerDeviceEnumerator::next()
{
    bool foundValid= false;

    while (cur_dev != nullptr && !foundValid)
    {
        cur_dev = cur_dev->next;
        foundValid= is_valid();

        // If there are more device types to scan
        // move on to the next vid/pid device enumeration
        if (cur_dev == nullptr &&
            GET_DEVICE_TYPE_CLASS(m_deviceType + 1) == CommonDeviceState::Controller &&
            (m_deviceType + 1) < CommonDeviceState::SUPPORTED_CONTROLLER_TYPE_COUNT)
        {
            m_deviceType= static_cast<CommonDeviceState::eDeviceType>(m_deviceType + 1);
            USBDeviceInfo &dev_info = g_supported_controller_infos[GET_DEVICE_TYPE_INDEX(m_deviceType)];

            // Free any previous enumeration
            if (devs != nullptr)
            {
                hid_free_enumeration(devs);
            }

            // Create a new HID enumeration
            devs = hid_enumerate(dev_info.vendor_id, dev_info.product_id);
            cur_dev = devs;
            foundValid= false;
        }
    }

    return foundValid;
}

// -- TrackerDeviceEnumerator -----
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

// -- HMDDeviceEnumerator -----
HMDDeviceEnumerator::HMDDeviceEnumerator()
    : DeviceEnumerator(CommonDeviceState::OVRDK2)
{
    assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_HMD_TYPE_INDEX);

    if (!is_valid())
    {
        next();
    }
}

HMDDeviceEnumerator::HMDDeviceEnumerator(CommonDeviceState::eDeviceType deviceType)
    : DeviceEnumerator(deviceType)
{
    assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_HMD_TYPE_INDEX);

    if (!is_valid())
    {
        next();
    }
}

HMDDeviceEnumerator::~HMDDeviceEnumerator()
{
}

const char *HMDDeviceEnumerator::get_path() const
{
    return nullptr;
}

bool HMDDeviceEnumerator::is_valid() const
{
    return false;
}

bool HMDDeviceEnumerator::next()
{
    bool foundValid = false;

    return foundValid;
}
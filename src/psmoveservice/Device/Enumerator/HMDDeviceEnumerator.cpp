// -- includes -----
#include "HMDDeviceEnumerator.h"
#include "DeviceManager.h"
#include "HMDManager.h"
#include "ServerUtility.h"
#include "assert.h"
#include "libusb.h"
#include "string.h"

// -- macros ----
#define MAX_HMD_TYPE_INDEX                  GET_DEVICE_TYPE_INDEX(CommonDeviceState::SUPPORTED_HMD_TYPE_COUNT)

// -- globals -----
USBDeviceInfo g_supported_hmd_infos[MAX_HMD_TYPE_INDEX] = {
    // Oculus DevKit1
    { 0x2833, 0x0021 }, // Oculus DevKit2
    // Oculus CrescentBay
    // Oculus EngineeringSample06
    // Oculus EngineeringSample09
    // Oculus ConsumerVersion1
};

// -- HMDDeviceEnumerator -----
HMDDeviceEnumerator::HMDDeviceEnumerator()
    : DeviceEnumerator(CommonDeviceState::OculusDK2)
    , usb_context(nullptr)
    , devs(nullptr)
    , cur_dev(nullptr)
    , dev_index(0)
    , dev_count(0)
    , hmd_index(-1)
    , dev_valid(false)
{
    assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_HMD_TYPE_INDEX);

    libusb_init(&usb_context);
    dev_count = static_cast<int>(libusb_get_device_list(usb_context, &devs));
    cur_dev = (devs != nullptr) ? devs[0] : nullptr;
    hmd_index = 0;

    if (!recompute_current_device_validity())
    {
        hmd_index = -1;
        next();
    }
    else
    {
        hmd_index = 0;
    }
}

HMDDeviceEnumerator::HMDDeviceEnumerator(CommonDeviceState::eDeviceType deviceType)
    : DeviceEnumerator(deviceType)
    , usb_context(nullptr)
    , devs(nullptr)
    , cur_dev(nullptr)
    , dev_index(0)
    , dev_count(0)
    , hmd_index(-1)
    , dev_valid(false)
{
    assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_HMD_TYPE_INDEX);

    memset(dev_port_numbers, 255, sizeof(dev_port_numbers));

    libusb_init(&usb_context);
    dev_count = static_cast<int>(libusb_get_device_list(usb_context, &devs));
    cur_dev = (devs != nullptr) ? devs[0] : nullptr;

    if (!is_valid())
    {
        hmd_index = -1;
        next();
    }
    else
    {
        hmd_index = 0;
    }
}

HMDDeviceEnumerator::~HMDDeviceEnumerator()
{
    if (devs != nullptr)
    {
        libusb_free_device_list(devs, 1);
    }

    libusb_exit(usb_context);
}

const char *HMDDeviceEnumerator::get_path() const
{
    const char *result = nullptr;

    if (cur_dev != nullptr)
    {
        struct libusb_device_descriptor dev_desc;
        libusb_get_device_descriptor(cur_dev, &dev_desc);

        ServerUtility::format_string(
            (char *)(cur_path), sizeof(cur_path),
            "USB\\VID_%04X&PID_%04X\\%d\\%d",
            dev_desc.idVendor, dev_desc.idProduct, dev_index, hmd_index);

        result = cur_path;
    }

    return result;
}

bool HMDDeviceEnumerator::is_valid() const
{
    return dev_valid;
}

bool HMDDeviceEnumerator::recompute_current_device_validity()
{
    dev_valid = false;

    if (cur_dev != nullptr)
    {
        bool bDeviceAPIAvailable = true;

        switch (m_deviceType)
        {
        case CommonDeviceState::OculusDK2:
            // Don't bother enumerating Oculus devices if the Oculus API isn't initialized
            bDeviceAPIAvailable = DeviceManager::getInstance()->m_hmd_manager->getIsOculusAPIInitialized();
            break;
        default:
            bDeviceAPIAvailable = true;
            break;
        }

        if (bDeviceAPIAvailable)
        {
            USBDeviceInfo &dev_info = g_supported_hmd_infos[GET_DEVICE_TYPE_INDEX(m_deviceType)];
            struct libusb_device_descriptor dev_desc;

            int libusb_result = libusb_get_device_descriptor(cur_dev, &dev_desc);

            if (libusb_result == 0 &&
                dev_desc.idVendor == dev_info.vendor_id &&
                dev_desc.idProduct == dev_info.product_id)
            {
                uint8_t port_numbers[MAX_USB_DEVICE_PORT_PATH];

                memset(port_numbers, 0, sizeof(port_numbers));
                int elements_filled = libusb_get_port_numbers(cur_dev, port_numbers, MAX_USB_DEVICE_PORT_PATH);

                if (elements_filled > 0)
                {
                    // Make sure this device is actually different from the last device we looked at
                    // (i.e. has a different device port path)
                    if (memcmp(port_numbers, dev_port_numbers, sizeof(port_numbers)) != 0)
                    {
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

bool HMDDeviceEnumerator::next()
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
            GET_DEVICE_TYPE_CLASS(m_deviceType + 1) == CommonDeviceState::HeadMountedDisplay &&
            (m_deviceType + 1) < CommonDeviceState::SUPPORTED_HMD_TYPE_COUNT)
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
        ++hmd_index;
    }

    return foundValid;
}
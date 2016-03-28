// -- includes -----
#include "HMDDeviceEnumerator.h"
#include "DeviceManager.h"
#include "HMDManager.h"
#include "ServerUtility.h"
#include "assert.h"
#include "hidapi.h"
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
    bool bIsValid = cur_dev != nullptr;

    if (bIsValid)
    {
        switch (m_deviceType)
        {
        case CommonDeviceState::OculusDK2:
            // Don't bother enumerating Oculus devices if the Oculus API isn't initialized
            bIsValid = DeviceManager::getInstance()->m_hmd_manager->getIsOculusAPIInitialized();
            break;
        default:
            bIsValid = true;
            break;
        }
    }

    return bIsValid;
}

bool HMDDeviceEnumerator::next()
{
    bool foundValid = false;

    while (cur_dev != nullptr && !foundValid)
    {
        cur_dev = cur_dev->next;
        foundValid = is_valid();

        // If there are more device types to scan
        // move on to the next vid/pid device enumeration
        if (cur_dev == nullptr &&
            GET_DEVICE_TYPE_CLASS(m_deviceType + 1) == CommonDeviceState::HeadMountedDisplay &&
            (m_deviceType + 1) < CommonDeviceState::SUPPORTED_HMD_TYPE_COUNT)
        {
            m_deviceType = static_cast<CommonDeviceState::eDeviceType>(m_deviceType + 1);
            USBDeviceInfo &dev_info = g_supported_hmd_infos[GET_DEVICE_TYPE_INDEX(m_deviceType)];

            // Free any previous enumeration
            if (devs != nullptr)
            {
                hid_free_enumeration(devs);
            }

            // Create a new HID enumeration
            devs = hid_enumerate(dev_info.vendor_id, dev_info.product_id);
            cur_dev = devs;
            foundValid = false;
        }
    }

    return foundValid;
}
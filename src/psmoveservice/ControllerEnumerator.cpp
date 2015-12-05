// -- includes -----
#include "ControllerEnumerator.h"
#include "ServerUtility.h"
#include "assert.h"
#include "hidapi.h"
#include "string.h"

//-- constants -----
#define PSMOVE_VID 0x054c
#define PSMOVE_PID 0x03d5

// -- private definitions -----
struct USBDeviceInfo
{
    unsigned short vendor_id;
    unsigned short product_id;
};

// -- globals -----
USBDeviceInfo g_supported_controller_infos[CommonControllerState::SUPPORTED_CONTROLLER_TYPE_COUNT] = {
    {0x054c, 0x03d5}, // PSMove
    //{0x054c, 0x042F}, // PSNavi
    //{0x054c, 0x0268}, // PSDualShock3
};

// -- public interface -----
ControllerDeviceEnumerator::ControllerDeviceEnumerator() 
    : devs(nullptr)
    , cur_dev(nullptr)
    , m_deviceType(CommonControllerState::PSMove)
{
    USBDeviceInfo &dev_info= g_supported_controller_infos[m_deviceType];

    devs = hid_enumerate(dev_info.vendor_id, dev_info.product_id);
    cur_dev = devs;
}

ControllerDeviceEnumerator::ControllerDeviceEnumerator(
    CommonControllerState::eControllerDeviceType deviceType)
    : devs(nullptr)
    , cur_dev(nullptr)
    , m_deviceType(deviceType)
{
    assert(m_deviceType >= 0 && m_deviceType < CommonControllerState::SUPPORTED_CONTROLLER_TYPE_COUNT);
    USBDeviceInfo &dev_info= g_supported_controller_infos[m_deviceType];

    devs = hid_enumerate(dev_info.vendor_id, dev_info.product_id);
    cur_dev = devs;
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
    return cur_dev != nullptr;
}

bool ControllerDeviceEnumerator::next()
{
    bool foundValid= false;

    while (cur_dev != nullptr && !foundValid)
    {
        cur_dev = cur_dev->next;
        foundValid= cur_dev != nullptr;

#ifdef _WIN32
        /**
        * Windows Quirk: Each dev is enumerated 3 times.
        * The one with "&col01#" in the path is the one we will get most of our data from. Only count this one.
        * The one with "&col02#" in the path is the one we will get the bluetooth address from.
        **/
        if (foundValid && strstr(cur_dev->path, "&col01#") == nullptr)
        {
            foundValid= false;
        }
#endif

        // If there are more device types to scan
        // move on to the next vid/pid device enumeration
        if (cur_dev == nullptr && 
            (m_deviceType + 1) < CommonControllerState::SUPPORTED_CONTROLLER_TYPE_COUNT)
        {
            m_deviceType= static_cast<CommonControllerState::eControllerDeviceType>(m_deviceType + 1);
            USBDeviceInfo &dev_info= g_supported_controller_infos[m_deviceType];

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
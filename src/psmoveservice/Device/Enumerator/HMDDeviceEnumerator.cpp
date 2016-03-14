// -- includes -----
#include "HMDDeviceEnumerator.h"
#include "ServerUtility.h"
#include "assert.h"
#include "hidapi.h"
#include "string.h"

// -- macros ----
#define MAX_HMD_TYPE_INDEX                  GET_DEVICE_TYPE_INDEX(CommonDeviceState::SUPPORTED_HMD_TYPE_COUNT)

// -- globals -----
USBDeviceInfo g_supported_hmd_infos[MAX_HMD_TYPE_INDEX] = {
    { 0x2833, 0x0021 }, // DK2
};

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
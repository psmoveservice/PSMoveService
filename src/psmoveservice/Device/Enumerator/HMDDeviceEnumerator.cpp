// -- includes -----
#include "HMDDeviceEnumerator.h"
#include "HidHMDDeviceEnumerator.h"
#include "VirtualHMDDeviceEnumerator.h"
#include "assert.h"
#include "string.h"

// -- globals -----

// -- HMDDeviceEnumerator -----
HMDDeviceEnumerator::HMDDeviceEnumerator(
	eAPIType _apiType)
	: DeviceEnumerator()
	, api_type(_apiType)
	, enumerators(nullptr)
	, enumerator_count(0)
	, enumerator_index(0)
{
	switch (_apiType)
	{
	case eAPIType::CommunicationType_HID:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = new HidHMDDeviceEnumerator;
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_VIRTUAL:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = new VirtualHMDDeviceEnumerator;
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_ALL:
		enumerators = new DeviceEnumerator *[2];
		enumerators[0] = new HidHMDDeviceEnumerator;
		enumerators[1] = new VirtualHMDDeviceEnumerator;
		enumerator_count = 2;
		break;
	}

	if (is_valid())
	{
		m_deviceType= enumerators[enumerator_index]->get_device_type();
	}
	else
    {
        next();
    }
}

HMDDeviceEnumerator::~HMDDeviceEnumerator()
{
	for (int index = 0; index < enumerator_count; ++index)
	{
		delete enumerators[index];
	}
	delete[] enumerators;
}

const char *HMDDeviceEnumerator::get_path() const
{
    return (enumerator_index < enumerator_count) ? enumerators[enumerator_index]->get_path() : nullptr;
}

int HMDDeviceEnumerator::get_vendor_id() const
{
	return (enumerator_index < enumerator_count) ? enumerators[enumerator_index]->get_vendor_id() : -1;
}

int HMDDeviceEnumerator::get_product_id() const
{
	return (enumerator_index < enumerator_count) ? enumerators[enumerator_index]->get_product_id() : -1;
}

HMDDeviceEnumerator::eAPIType HMDDeviceEnumerator::get_api_type() const
{
	HMDDeviceEnumerator::eAPIType result= HMDDeviceEnumerator::CommunicationType_INVALID;

	switch (api_type)
	{
	case eAPIType::CommunicationType_HID:
		result = (enumerator_index < enumerator_count) ? HMDDeviceEnumerator::CommunicationType_HID : HMDDeviceEnumerator::CommunicationType_INVALID;
		break;
	case eAPIType::CommunicationType_VIRTUAL:
		result = (enumerator_index < enumerator_count) ? HMDDeviceEnumerator::CommunicationType_VIRTUAL : HMDDeviceEnumerator::CommunicationType_INVALID;
		break;
	case eAPIType::CommunicationType_ALL:
		if (enumerator_index < enumerator_count)
		{
			switch (enumerator_index)
			{
			case 0:
				result = HMDDeviceEnumerator::CommunicationType_HID;
				break;
			case 1:
				result = HMDDeviceEnumerator::CommunicationType_VIRTUAL;
				break;
			default:
				result = HMDDeviceEnumerator::CommunicationType_INVALID;
				break;
			}
		}
		else
		{
			result = HMDDeviceEnumerator::CommunicationType_INVALID;
		}
		break;
	}

	return result;
}

const HidHMDDeviceEnumerator *HMDDeviceEnumerator::get_hid_hmd_enumerator() const
{
	HidHMDDeviceEnumerator *enumerator = nullptr;

	switch (api_type)
	{
	case eAPIType::CommunicationType_HID:
		enumerator = (enumerator_index < enumerator_count) ? static_cast<HidHMDDeviceEnumerator *>(enumerators[0]) : nullptr;
		break;
	case eAPIType::CommunicationType_VIRTUAL:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_ALL:
		if (enumerator_index < enumerator_count)
		{
			enumerator = (enumerator_index == 0) ? static_cast<HidHMDDeviceEnumerator *>(enumerators[0]) : nullptr;
		}
		else
		{
			enumerator = nullptr;
		}
		break;
	}

	return enumerator;
}

const VirtualHMDDeviceEnumerator *HMDDeviceEnumerator::get_virtual_hmd_enumerator() const
{
	VirtualHMDDeviceEnumerator *enumerator = nullptr;

	switch (api_type)
	{
	case eAPIType::CommunicationType_HID:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_VIRTUAL:
		enumerator = (enumerator_index < enumerator_count) ? static_cast<VirtualHMDDeviceEnumerator *>(enumerators[0]) : nullptr;
		break;
	case eAPIType::CommunicationType_ALL:
		if (enumerator_index < enumerator_count)
		{
			enumerator = (enumerator_index == 1) ? static_cast<VirtualHMDDeviceEnumerator *>(enumerators[1]) : nullptr;
		}
		else
		{
			enumerator = nullptr;
		}
		break;
	}

	return enumerator;
}

bool HMDDeviceEnumerator::is_valid() const
{
    bool bIsValid = false;

	if (enumerator_index < enumerator_count)
	{
		bIsValid = enumerators[enumerator_index]->is_valid();
	}

    return bIsValid;
}

bool HMDDeviceEnumerator::next()
{
    bool foundValid = false;

    while (!foundValid && enumerator_index < enumerator_count)
    {
		if (enumerators[enumerator_index]->is_valid())
		{
			enumerators[enumerator_index]->next();
			foundValid = enumerators[enumerator_index]->is_valid();
		}
		else
		{
			++enumerator_index;

			if (enumerator_index < enumerator_count)
			{
				foundValid = enumerators[enumerator_index]->is_valid();
			}
		}
    }

	if (foundValid)
	{
		m_deviceType = enumerators[enumerator_index]->get_device_type();
	}
	else
	{
		m_deviceType = CommonDeviceState::SUPPORTED_CONTROLLER_TYPE_COUNT; // invalid
	}

    return foundValid;
}
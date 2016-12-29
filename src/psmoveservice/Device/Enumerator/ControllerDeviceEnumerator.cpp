// -- includes -----
#include "ControllerDeviceEnumerator.h"
#include "ControllerHidDeviceEnumerator.h"
#include "ControllerUSBDeviceEnumerator.h"
#include "ControllerGamepadEnumerator.h"
#include "assert.h"
#include "string.h"

// -- globals -----

// -- ControllerDeviceEnumerator -----
ControllerDeviceEnumerator::ControllerDeviceEnumerator(
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
		enumerators[0] = new ControllerHidDeviceEnumerator;
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_USB:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = new ControllerUSBDeviceEnumerator;
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_GAMEPAD:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = new ControllerGamepadEnumerator;
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_ALL:
		enumerators = new DeviceEnumerator *[3];
		enumerators[0] = new ControllerHidDeviceEnumerator;
		enumerators[1] = new ControllerUSBDeviceEnumerator;
		enumerators[2] = new ControllerGamepadEnumerator;
		enumerator_count = 3;
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

ControllerDeviceEnumerator::ControllerDeviceEnumerator(
	eAPIType _apiType,
	CommonDeviceState::eDeviceType deviceType)
    : DeviceEnumerator(deviceType)
	, api_type(_apiType)
	, enumerators(nullptr)
	, enumerator_count(0)
	, enumerator_index(0)
{
	switch (_apiType)
	{
	case eAPIType::CommunicationType_HID:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = new ControllerHidDeviceEnumerator(deviceType);
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_USB:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = new ControllerUSBDeviceEnumerator(deviceType);
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_GAMEPAD:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = new ControllerGamepadEnumerator(deviceType);
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_ALL:
		enumerators = new DeviceEnumerator *[3];
		enumerators[0] = new ControllerHidDeviceEnumerator(deviceType);
		enumerators[1] = new ControllerUSBDeviceEnumerator(deviceType);
		enumerators[2] = new ControllerGamepadEnumerator(deviceType);
		enumerator_count = 3;
		break;
	}

	if (is_valid())
	{
		m_deviceType = enumerators[enumerator_index]->get_device_type();
	}
	else
	{
		next();
	}
}

ControllerDeviceEnumerator::~ControllerDeviceEnumerator()
{
	for (int index = 0; index < enumerator_count; ++index)
	{
		delete enumerators[index];
	}
	delete[] enumerators;
}

const char *ControllerDeviceEnumerator::get_path() const
{
    return (enumerator_index < enumerator_count) ? enumerators[enumerator_index]->get_path() : nullptr;
}

int ControllerDeviceEnumerator::get_vendor_id() const
{
	return (enumerator_index < enumerator_count) ? enumerators[enumerator_index]->get_vendor_id() : -1;
}

int ControllerDeviceEnumerator::get_product_id() const
{
	return (enumerator_index < enumerator_count) ? enumerators[enumerator_index]->get_product_id() : -1;
}

bool ControllerDeviceEnumerator::get_serial_number(char *out_mb_serial, const size_t mb_buffer_size) const
{
    bool success = false;

    if ((api_type == eAPIType::CommunicationType_HID) ||
		(api_type == eAPIType::CommunicationType_ALL && enumerator_index == 0))
    {
		ControllerHidDeviceEnumerator *hid_enumerator = static_cast<ControllerHidDeviceEnumerator *>(enumerators[enumerator_index]);

        success = hid_enumerator->get_serial_number(out_mb_serial, mb_buffer_size);
    }

    return success;
}

ControllerDeviceEnumerator::eAPIType ControllerDeviceEnumerator::get_api_type() const
{
	ControllerDeviceEnumerator::eAPIType result= ControllerDeviceEnumerator::CommunicationType_INVALID;

	switch (api_type)
	{
	case eAPIType::CommunicationType_HID:
		result = (enumerator_index < enumerator_count) ? ControllerDeviceEnumerator::CommunicationType_HID : ControllerDeviceEnumerator::CommunicationType_INVALID;
		break;
	case eAPIType::CommunicationType_USB:
		result = (enumerator_index < enumerator_count) ? ControllerDeviceEnumerator::CommunicationType_USB : ControllerDeviceEnumerator::CommunicationType_INVALID;
		break;
	case eAPIType::CommunicationType_GAMEPAD:
		result = (enumerator_index < enumerator_count) ? ControllerDeviceEnumerator::CommunicationType_GAMEPAD : ControllerDeviceEnumerator::CommunicationType_INVALID;
		break;
	case eAPIType::CommunicationType_ALL:
		if (enumerator_index < enumerator_count)
		{
			switch (enumerator_index)
			{
			case 0:
				result = ControllerDeviceEnumerator::CommunicationType_HID;
				break;
			case 1:
				result = ControllerDeviceEnumerator::CommunicationType_USB;
				break;
			case 2:
				result = ControllerDeviceEnumerator::CommunicationType_GAMEPAD;
				break;
			default:
				result = ControllerDeviceEnumerator::CommunicationType_INVALID;
				break;
			}
		}
		else
		{
			result = ControllerDeviceEnumerator::CommunicationType_INVALID;
		}
		break;
	}

	return result;
}

const ControllerHidDeviceEnumerator *ControllerDeviceEnumerator::get_hid_controller_enumerator() const
{
	ControllerHidDeviceEnumerator *enumerator = nullptr;

	switch (api_type)
	{
	case eAPIType::CommunicationType_HID:
		enumerator = (enumerator_index < enumerator_count) ? static_cast<ControllerHidDeviceEnumerator *>(enumerators[0]) : nullptr;
		break;
	case eAPIType::CommunicationType_USB:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_GAMEPAD:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_ALL:
		if (enumerator_index < enumerator_count)
		{
			enumerator = (enumerator_index == 0) ? static_cast<ControllerHidDeviceEnumerator *>(enumerators[0]) : nullptr;
		}
		else
		{
			enumerator = nullptr;
		}
		break;
	}

	return enumerator;
}

const ControllerUSBDeviceEnumerator *ControllerDeviceEnumerator::get_usb_controller_enumerator() const
{
	ControllerUSBDeviceEnumerator *enumerator = nullptr;

	switch (api_type)
	{
	case eAPIType::CommunicationType_HID:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_USB:
		enumerator = (enumerator_index < enumerator_count) ? static_cast<ControllerUSBDeviceEnumerator *>(enumerators[0]) : nullptr;
		break;
	case eAPIType::CommunicationType_GAMEPAD:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_ALL:
		if (enumerator_index < enumerator_count)
		{
			enumerator = (enumerator_index == 1) ? static_cast<ControllerUSBDeviceEnumerator *>(enumerators[1]) : nullptr;
		}
		else
		{
			enumerator = nullptr;
		}
		break;
	}

	return enumerator;
}

const ControllerGamepadEnumerator *ControllerDeviceEnumerator::get_gamepad_controller_enumerator() const
{
	ControllerGamepadEnumerator *enumerator = nullptr;

	switch (api_type)
	{
	case eAPIType::CommunicationType_HID:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_USB:
		enumerator = nullptr;
		break;
	case eAPIType::CommunicationType_GAMEPAD:
		enumerator = (enumerator_index < enumerator_count) ? static_cast<ControllerGamepadEnumerator *>(enumerators[0]) : nullptr;
		break;
	case eAPIType::CommunicationType_ALL:
		if (enumerator_index < enumerator_count)
		{
			enumerator = (enumerator_index == 2) ? static_cast<ControllerGamepadEnumerator *>(enumerators[2]) : nullptr;
		}
		else
		{
			enumerator = nullptr;
		}
		break;
	}

	return enumerator;
}

bool ControllerDeviceEnumerator::is_valid() const
{
    bool bIsValid = false;

	if (enumerator_index < enumerator_count)
	{
		bIsValid = enumerators[enumerator_index]->is_valid();
	}

    return bIsValid;
}

bool ControllerDeviceEnumerator::next()
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
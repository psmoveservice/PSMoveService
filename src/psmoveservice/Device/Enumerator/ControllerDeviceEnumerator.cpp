// -- includes -----
#include "ControllerDeviceEnumerator.h"
#include "ControllerHidDeviceEnumerator.h"
#include "ControllerLibUSBDeviceEnumerator.h"
#include "assert.h"
#include "string.h"

// -- globals -----

// -- ControllerDeviceEnumerator -----
ControllerDeviceEnumerator::ControllerDeviceEnumerator(
	eAPIType _apiType)
	: DeviceEnumerator(CommonDeviceState::PSMove)
	, api_type(_apiType)
	, enumerators(nullptr)
	, enumerator_count(0)
	, enumerator_index(0)
{
	switch (_apiType)
	{
	case eAPIType::CommunicationType_HID:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = new ControllerHidDeviceEnumerator(CommonDeviceState::PSMove);
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_LIBUSB:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = new ControllerLibUSBDeviceEnumerator(CommonDeviceState::PSMove);
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_ALL:
		enumerators = new DeviceEnumerator *[2];
		enumerators[0] = new ControllerHidDeviceEnumerator(CommonDeviceState::PSMove);
		enumerators[1] = new ControllerLibUSBDeviceEnumerator(CommonDeviceState::PSMove);
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
	case eAPIType::CommunicationType_LIBUSB:
		enumerators = new DeviceEnumerator *[1];
		enumerators[0] = new ControllerLibUSBDeviceEnumerator(deviceType);
		enumerator_count = 1;
		break;
	case eAPIType::CommunicationType_ALL:
		enumerators = new DeviceEnumerator *[2];
		enumerators[0] = new ControllerHidDeviceEnumerator(deviceType);
		enumerators[1] = new ControllerLibUSBDeviceEnumerator(deviceType);
		enumerator_count = 2;
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

t_usb_device_handle ControllerDeviceEnumerator::get_usb_device_handle() const
{
	t_usb_device_handle result = k_invalid_usb_device_handle;

	if ((api_type == eAPIType::CommunicationType_LIBUSB) ||
		(api_type == eAPIType::CommunicationType_ALL && enumerator_index == 1))
	{
		ControllerLibUSBDeviceEnumerator *libusb_enumerator = static_cast<ControllerLibUSBDeviceEnumerator *>(enumerators[enumerator_index]);

		result = libusb_enumerator->get_usb_device_handle();
	}

	return result;
}

ControllerDeviceEnumerator::eAPIType ControllerDeviceEnumerator::get_api_type() const
{
	ControllerDeviceEnumerator::eAPIType result= ControllerDeviceEnumerator::CommunicationType_INVALID;

	switch (api_type)
	{
	case eAPIType::CommunicationType_HID:
		result = (enumerator_index < enumerator_count) ? ControllerDeviceEnumerator::CommunicationType_HID : ControllerDeviceEnumerator::CommunicationType_INVALID;
		break;
	case eAPIType::CommunicationType_LIBUSB:
		result = (enumerator_index < enumerator_count) ? ControllerDeviceEnumerator::CommunicationType_LIBUSB : ControllerDeviceEnumerator::CommunicationType_INVALID;
		break;
	case eAPIType::CommunicationType_ALL:
		if (enumerator_index < enumerator_count)
		{
			result = (enumerator_index == 0) ? ControllerDeviceEnumerator::CommunicationType_HID : ControllerDeviceEnumerator::CommunicationType_LIBUSB;
		}
		else
		{
			result = ControllerDeviceEnumerator::CommunicationType_INVALID;
		}
		break;
	}

	return result;
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
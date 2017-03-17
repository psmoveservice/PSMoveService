// -- includes -----
#include "ControllerGamepadEnumerator.h"
#include "ServerUtility.h"
#include "USBDeviceInfo.h"

#include "assert.h"
#include "string.h"

#include "gamepad/Gamepad.h"

// -- private definitions -----
#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

// -- macros ----
#define MAX_CONTROLLER_TYPE_INDEX           GET_DEVICE_TYPE_INDEX(CommonDeviceState::SUPPORTED_CONTROLLER_TYPE_COUNT)

// -- globals -----
struct GamepadAPIDeviceFilter
{
	USBDeviceFilter filter;
	bool bGamepadApiSupported;
};

GamepadAPIDeviceFilter g_supported_gamepad_infos[MAX_CONTROLLER_TYPE_INDEX] = {
	{ 0x054c, 0x03d5, false}, // PSMove
	{ 0x045e, 0x028e, true}, // PSNavi pretending to be an XBox 360 controller via ScpService
	{ 0x054c, 0x05C4, false}, // PSDualShock4
};

static bool is_gamepad_supported(
	int gamepad_index, 
	CommonDeviceState::eDeviceType device_type_filter,
	CommonDeviceState::eDeviceType &out_device_type);

// -- ControllerGamepadEnumerator -----
ControllerGamepadEnumerator::ControllerGamepadEnumerator()
	: DeviceEnumerator()
	, m_controllerIndex(-1)
{
	m_deviceType= CommonDeviceState::PSMove;
	assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_CONTROLLER_TYPE_INDEX);

	Gamepad_detectDevices();
	next();
}

ControllerGamepadEnumerator::ControllerGamepadEnumerator(
	CommonDeviceState::eDeviceType deviceTypeFilter)
	: DeviceEnumerator(deviceTypeFilter)
	, m_controllerIndex(-1)
{
	m_deviceType= deviceTypeFilter;
	m_deviceTypeFilter= deviceTypeFilter;
	assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_CONTROLLER_TYPE_INDEX);

	Gamepad_detectDevices();
	next();
}

int ControllerGamepadEnumerator::get_vendor_id() const
{
	return is_valid() ? Gamepad_deviceAtIndex(static_cast<unsigned int>(m_controllerIndex))->vendorID : -1;
}

int ControllerGamepadEnumerator::get_product_id() const
{
	return is_valid() ? Gamepad_deviceAtIndex(static_cast<unsigned int>(m_controllerIndex))->productID : -1;
}

const char *ControllerGamepadEnumerator::get_path() const
{
	return is_valid() ? m_currentUSBPath : "";
}

bool ControllerGamepadEnumerator::is_valid() const
{
	return m_controllerIndex < static_cast<int>(Gamepad_numDevices());
}

bool ControllerGamepadEnumerator::next()
{
	bool foundValid = false;

	while (m_controllerIndex < static_cast<int>(Gamepad_numDevices()) && !foundValid)
	{
		++m_controllerIndex;

		if (m_controllerIndex < static_cast<int>(Gamepad_numDevices()) && 
			is_gamepad_supported(m_controllerIndex, m_deviceTypeFilter, m_deviceType))
		{
			ServerUtility::format_string(m_currentUSBPath, sizeof(m_currentUSBPath), "gamepad_%d", m_controllerIndex);		
			foundValid = true;
		}
	}

	return foundValid;
}

//-- private methods -----
static bool is_gamepad_supported(
	int gamepad_index, 
	CommonDeviceState::eDeviceType device_type_filter,
	CommonDeviceState::eDeviceType &out_device_type)
{
	bool bIsValidDevice = false;

	const Gamepad_device *devInfo= Gamepad_deviceAtIndex(static_cast<unsigned int>(gamepad_index));

	if (devInfo != nullptr)
	{
		// See if the next filtered device is a controller type that we care about
		for (int gamepad_type_index = 0; gamepad_type_index < MAX_CONTROLLER_TYPE_INDEX; ++gamepad_type_index)
		{
			const GamepadAPIDeviceFilter &supported_type = g_supported_gamepad_infos[gamepad_type_index];

			if (supported_type.bGamepadApiSupported &&
				devInfo->productID == supported_type.filter.product_id &&
				devInfo->vendorID == supported_type.filter.vendor_id)
			{				
				CommonDeviceState::eDeviceType device_type =
					static_cast<CommonDeviceState::eDeviceType>(CommonDeviceState::Controller + gamepad_type_index);

				if (device_type_filter == device_type || device_type_filter == CommonDeviceState::INVALID_DEVICE_TYPE)
				{
					out_device_type= device_type;
					bIsValidDevice = true;
					break;
				}				
			}
		}
	}

	return bIsValidDevice;
}
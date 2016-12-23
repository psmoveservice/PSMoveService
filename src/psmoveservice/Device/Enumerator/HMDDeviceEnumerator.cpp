// -- includes -----
#include "HMDDeviceEnumerator.h"
#include "ServerUtility.h"
#include "USBDeviceInfo.h" // for MAX_USB_DEVICE_PORT_PATH, t_usb_device_handle
#include "assert.h"
#include "hidapi.h"
#include "string.h"
#include <sstream>
#include <iomanip>

// -- private definitions -----
#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

// -- macros ----
#define MAX_HMD_TYPE_INDEX                  GET_DEVICE_TYPE_INDEX(CommonDeviceState::SUPPORTED_HMD_TYPE_COUNT)

// -- globals -----
USBDeviceFilter g_supported_hmd_infos[MAX_HMD_TYPE_INDEX] = {
    { 0x054c, 0x09af }, // Sony Morpheus
};

// -- HMDDeviceEnumerator -----
HMDDeviceEnumerator::HMDDeviceEnumerator()
    : DeviceEnumerator(CommonDeviceState::Morpheus)
{
    assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_HMD_TYPE_INDEX);

	build_interface_list();

	if (!is_valid())
	{
		next();
	}
}

HMDDeviceEnumerator::HMDDeviceEnumerator(CommonDeviceState::eDeviceType deviceType)
    : DeviceEnumerator(deviceType)
{
    assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_HMD_TYPE_INDEX);

	build_interface_list();

	if (!is_valid())
	{
		next();
	}
}

const char *HMDDeviceEnumerator::get_path() const
{
	return current_device_identifier.c_str();
}

int HMDDeviceEnumerator::get_vendor_id() const
{
	return is_valid() ? g_supported_hmd_infos[GET_DEVICE_TYPE_INDEX(m_deviceType)].vendor_id : -1;
}

int HMDDeviceEnumerator::get_product_id() const
{
	return is_valid() ? g_supported_hmd_infos[GET_DEVICE_TYPE_INDEX(m_deviceType)].product_id : -1;
}

bool HMDDeviceEnumerator::is_valid() const
{
	return current_device_interfaces.size() > 0;
}

bool HMDDeviceEnumerator::next()
{
	bool foundValid = false;

	while (!foundValid && m_deviceType < CommonDeviceState::SUPPORTED_HMD_TYPE_COUNT)
	{
		m_deviceType = static_cast<CommonDeviceState::eDeviceType>(m_deviceType + 1);

		if (GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_HMD_TYPE_INDEX)
		{
			build_interface_list();

			foundValid = is_valid();
		}
		else
		{
			current_device_identifier = "";
			current_device_interfaces.clear();
		}
	}

	return foundValid;
}

std::string HMDDeviceEnumerator::get_interface_path(int interface_number) const
{
	std::string path = "";

	for (int list_index = 0; list_index < current_device_interfaces.size(); ++list_index)
	{
		if (current_device_interfaces[list_index].interface_number == interface_number)
		{
			path = current_device_interfaces[list_index].device_path;
			break;
		}
	}

	return path;
}

void HMDDeviceEnumerator::build_interface_list()
{
	USBDeviceFilter &dev_info = g_supported_hmd_infos[GET_DEVICE_TYPE_INDEX(m_deviceType)];
	hid_device_info * devs = hid_enumerate(dev_info.vendor_id, dev_info.product_id);

	current_device_identifier = "";
	current_device_interfaces.clear();

	if (devs != nullptr)
	{
		std::stringstream device_id_builder;
		device_id_builder << 
			"USB\\VID_" << std::hex << std::setfill('0') << std::setw(4) << dev_info.vendor_id <<
			"&PID_" << std::hex << std::setfill('0') << std::setw(4) << dev_info.product_id;

		current_device_identifier = device_id_builder.str();

		for (hid_device_info *cur_dev = devs; cur_dev != nullptr; cur_dev = cur_dev->next)
		{
			HMDDeviceInterface hmd_interface;

			hmd_interface.device_path = cur_dev->path;
			hmd_interface.interface_number = cur_dev->interface_number;

			current_device_interfaces.push_back(hmd_interface);
		}

		hid_free_enumeration(devs);
	}
}
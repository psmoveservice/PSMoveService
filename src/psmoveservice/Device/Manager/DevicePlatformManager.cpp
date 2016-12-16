//-- includes -----
#include "DevicePlatformManager.h"
#ifdef WIN32
#include "DevicePlatformAPIWin32.h"
#endif // WIN32
#include <assert.h>

// -- statics -----
DevicePlatformManager *DevicePlatformManager::m_instance = nullptr;

// -- public interface -----
DevicePlatformManager::DevicePlatformManager(enum eDevicePlatformApiType api_type)
	: m_api_type(api_type)
	, m_api(nullptr)
{
	switch (api_type)
	{
	case _eDevicePlatformApiType_None:
		m_api = nullptr;
		break;
#ifdef WIN32
	case _eDevicePlatformApiType_Win32:
		m_api = new DeviceHotplugAPIWin32;
		break;
#endif
	default:
		assert(0 && "unreachable");
		break;
	}
}

DevicePlatformManager::~DevicePlatformManager()
{
	if (m_api != nullptr)
	{
		delete m_api;
	}
}

// -- System ----
bool DevicePlatformManager::startup()
{
	bool bSuccess = true;

	if (m_api != nullptr)
	{
		bSuccess= m_api->startup(this);
	}

	if (bSuccess)
	{
		m_instance = this;
	}

	return bSuccess;
}

void DevicePlatformManager::poll()
{
	if (m_api != nullptr)
	{
		m_api->poll();
	}
}

void DevicePlatformManager::shutdown()
{
	if (m_api != nullptr)
	{
		m_api->shutdown();
	}

	m_instance = nullptr;
}

// -- Queries ---
bool DevicePlatformManager::get_device_property(
	const DeviceClass deviceClass,
	const int vendor_id,
	const int product_id,
	const char *property_name,
	char *buffer,
	const int buffer_size)
{
	bool bSuccess = false;

	if (m_api != nullptr)
	{
		bSuccess = m_api->get_device_property(deviceClass, vendor_id, product_id, property_name, buffer, buffer_size);
	}

	return bSuccess;
}

// -- Notification --
void DevicePlatformManager::registerHotplugListener(const CommonDeviceState::eDeviceClass deviceClass, IDeviceHotplugListener *listener)
{
	DeviceHotplugListener entry;
	entry.listener = listener;

	switch (deviceClass)
	{
	case CommonDeviceState::Controller:
	case CommonDeviceState::HeadMountedDisplay:
		entry.device_class = DeviceClass::DeviceClass_HID;
		break;
	case CommonDeviceState::TrackingCamera:
		entry.device_class = DeviceClass::DeviceClass_Camera;
		break;
	default:
		break;
	}

	m_listeners.push_back(entry);
}

void DevicePlatformManager::handle_device_connected(enum DeviceClass device_class, const std::string &device_path)
{
	for (auto &it = m_listeners.begin(); it != m_listeners.end(); ++it)
	{
		if (it->device_class == device_class)
		{
			it->listener->handle_device_connected(device_class, device_path);
		}
	}
}

void DevicePlatformManager::handle_device_disconnected(enum DeviceClass device_class, const std::string &device_path)
{
	for (auto &it = m_listeners.begin(); it != m_listeners.end(); ++it)
	{
		if (it->device_class == device_class)
		{
			it->listener->handle_device_disconnected(device_class, device_path);
		}
	}
}
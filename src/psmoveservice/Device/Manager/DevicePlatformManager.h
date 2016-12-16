#ifndef DEVICE_HOTPLUG_NOTIFIER_H
#define DEVICE_HOTPLUG_NOTIFIER_H

// -- include ----
#include "DevicePlatformInterface.h"
#include "DeviceInterface.h"
#include <vector>

//-- constants -----
enum eDevicePlatformApiType
{
	_eDevicePlatformApiType_None,
#ifdef WIN32
	_eDevicePlatformApiType_Win32,
#endif // WIN32
};

// -- definitions ----

struct DeviceHotplugListener
{
	DeviceClass device_class;
	IDeviceHotplugListener *listener;
};

class DevicePlatformManager : public IDeviceHotplugListener
{
public:
	DevicePlatformManager(enum eDevicePlatformApiType api_type);
	virtual ~DevicePlatformManager();

	static inline DevicePlatformManager *getInstance()
	{
		return m_instance;
	}

	// -- System ----
	bool startup();
	void poll();
	void shutdown();

	// -- Queries ---
	inline eDevicePlatformApiType get_api_type() const { return m_api_type; }
	bool get_device_property(
		const DeviceClass deviceClass,
		const int vendor_id,
		const int product_id,
		const char *property_name,
		char *buffer,
		const int buffer_size);

	// -- Notification --
	void registerHotplugListener(const CommonDeviceState::eDeviceClass deviceClass, IDeviceHotplugListener *listener);
	void handle_device_connected(enum DeviceClass device_class, const std::string &device_path) override;
	void handle_device_disconnected(enum DeviceClass device_class, const std::string &device_path) override;

private:
	/// Singleton instance of the class
	/// Assigned in startup, cleared in teardown
	static DevicePlatformManager *m_instance;

	// OS specific implementation of hotplug notification
	eDevicePlatformApiType m_api_type;
	IDevicePlatformAPI *m_api;

	// List of registered hot-plug listeners
	std::vector<DeviceHotplugListener> m_listeners;
};

#endif // DEVICE_HOTPLUG_NOTIFIER_H
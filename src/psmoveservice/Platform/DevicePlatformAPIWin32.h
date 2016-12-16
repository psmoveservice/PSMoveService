#ifndef DEVICE_PLATFORM_API_WIN32_H
#define DEVICE_PLATFORM_API_WIN32_H

// -- include -----
#include "DevicePlatformInterface.h"

// -- definitions -----
class DevicePlatformAPIWin32 : public IDevicePlatformAPI
{
public:
	DevicePlatformAPIWin32();
	virtual ~DevicePlatformAPIWin32();

	// System
	bool startup(IDeviceHotplugListener *broadcaster) override;
	void poll() override;
	void shutdown() override;

	// Queries
	bool get_device_property(
		const DeviceClass deviceClass,
		const int vendor_id,
		const int product_id,
		const char *property_name,
		char *buffer,
		const int buffer_size) override;
};

#endif // DEVICE_PLATFORM_API_WIN32_H

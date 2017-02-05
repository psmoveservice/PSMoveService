#ifndef PLATFORM_DEVICE_API_WIN32_H
#define PLATFORM_DEVICE_API_WIN32_H

// -- include -----
#include "DevicePlatformInterface.h"

// -- definitions -----
class PlatformDeviceAPIWin32 : public IPlatformDeviceAPI
{
public:
	PlatformDeviceAPIWin32();
	virtual ~PlatformDeviceAPIWin32();

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

#endif // PLATFORM_DEVICE_API_WIN32_H

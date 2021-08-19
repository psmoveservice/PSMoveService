#ifndef PLATFORM_DEVICE_API_LINUX_H
#define PLATFORM_DEVICE_API_LINUX_H

// -- include -----
#include "DevicePlatformInterface.h"

// -- definitions -----
class PlatformDeviceAPILinux : public IPlatformDeviceAPI
{
public:
	PlatformDeviceAPILinux();
	virtual ~PlatformDeviceAPILinux();

	// System
	bool startup(IDeviceHotplugListener *broadcaster) override;
	void poll() override;
	void shutdown() override;

	// Events
	void handle_bluetooth_request_started() override;
	void handle_bluetooth_request_finished() override;

	// Queries
	bool get_device_property(
		const DeviceClass deviceClass,
		const int vendor_id,
		const int product_id,
		const char *property_name,
		char *buffer,
		const int buffer_size) override;
};

#endif // PLATFORM_DEVICE_API_LINUX_H

// -- include -----
#include "PlatformDeviceAPILinux.h"
#include "ServerLog.h"

#include <assert.h>

#include <string>
#include <vector>
#include <iostream>
#include <iomanip>

//-- constants -----
const char *k_reg_property_driver_desc = "DriverDesc";
const char *k_reg_property_driver_version = "DriverVersion";
const char *k_reg_property_matching_device_id = "MatchingDeviceId";
const char *k_reg_property_provider_name = "ProviderName";
const char *k_reg_property_vendor = "Vendor";


// -- globals ----
IDeviceHotplugListener *g_hotplug_broadcaster = nullptr;

//-- private definitions -----


// -- definitions -----
PlatformDeviceAPILinux::PlatformDeviceAPILinux()
{

}

PlatformDeviceAPILinux::~PlatformDeviceAPILinux()
{

}

// System
bool PlatformDeviceAPILinux::startup(IDeviceHotplugListener *broadcaster)
{
	bool bSuccess = true;

	return bSuccess;
}

void PlatformDeviceAPILinux::poll()
{

}

void PlatformDeviceAPILinux::shutdown()
{

}

// Events
void PlatformDeviceAPILinux::handle_bluetooth_request_started()
{

}

void PlatformDeviceAPILinux::handle_bluetooth_request_finished()
{

}

// Queries
bool PlatformDeviceAPILinux::get_device_property(
	const DeviceClass deviceClass,
	const int vendor_id,
	const int product_id,
	const char *property_name,
	char *buffer,
	const int buffer_size)
{
	bool success = false;


	return success;
}

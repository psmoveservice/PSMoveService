#ifndef HID_HMD_DEVICE_ENUMERATOR_H
#define HID_HMD_DEVICE_ENUMERATOR_H

// -- includes -----
#include "DeviceEnumerator.h"
#include <vector>
#include <string>

// -- definitions -----
struct HidHMDDeviceInterface
{
	std::string device_path;
	int interface_number;
};

class HidHMDDeviceEnumerator : public DeviceEnumerator
{
public:
    HidHMDDeviceEnumerator();

    bool is_valid() const override;
    bool next() override;
	int get_vendor_id() const override;
	int get_product_id() const override;
    const char *get_path() const override;

	std::string get_interface_path(int interface_number) const;

private:
	void build_interface_list();

	std::string current_device_identifier;
	std::vector<HidHMDDeviceInterface> current_device_interfaces;
};

#endif // HID_HMD_DEVICE_ENUMERATOR_H

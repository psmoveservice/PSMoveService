#ifndef HMD_DEVICE_ENUMERATOR_H
#define HMD_DEVICE_ENUMERATOR_H

// -- includes -----
#include "DeviceEnumerator.h"
#include <vector>
#include <string>

// -- definitions -----
struct HMDDeviceInterface
{
	std::string device_path;
	int interface_number;
};

class HMDDeviceEnumerator : public DeviceEnumerator
{
public:
    HMDDeviceEnumerator();
    HMDDeviceEnumerator(CommonDeviceState::eDeviceType deviceType);

    bool is_valid() const override;
    bool next() override;
	int get_vendor_id() const;
	int get_product_id() const;
    const char *get_path() const override;

	std::string get_interface_path(int interface_number) const;

private:
	void build_interface_list();

	std::string current_device_identifier;
	std::vector<HMDDeviceInterface> current_device_interfaces;
};

#endif // HMD_DEVICE_ENUMERATOR_H
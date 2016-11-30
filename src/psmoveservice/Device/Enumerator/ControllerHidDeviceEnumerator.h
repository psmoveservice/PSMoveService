#ifndef CONTROLLER_HID_DEVICE_ENUMERATOR_H
#define CONTROLLER_HID_DEVICE_ENUMERATOR_H

#include "DeviceEnumerator.h"

class ControllerHidDeviceEnumerator : public DeviceEnumerator
{
public:
	ControllerHidDeviceEnumerator();
	ControllerHidDeviceEnumerator(CommonDeviceState::eDeviceType deviceType);
	~ControllerHidDeviceEnumerator();

	bool is_valid() const override;
	bool next() override;
	const char *get_path() const override;

	bool get_serial_number(char *out_mb_serial, const size_t mb_buffer_size) const;

private:
	struct hid_device_info *devs, *cur_dev;
};

#endif // CONTROLLER_HID_DEVICE_ENUMERATOR_H
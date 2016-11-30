#ifndef CONTROLLER_LIBUSB_DEVICE_ENUMERATOR_H
#define CONTROLLER_LIBUSB_DEVICE_ENUMERATOR_H

#include "DeviceEnumerator.h"

class ControllerLibUSBDeviceEnumerator : public DeviceEnumerator
{
public:
	ControllerLibUSBDeviceEnumerator();
	ControllerLibUSBDeviceEnumerator(CommonDeviceState::eDeviceType deviceType);
	~ControllerLibUSBDeviceEnumerator();

	bool is_valid() const override;
	bool next() override;
	const char *get_path() const override;
	inline int get_contoller_index() const { return controller_index; }

protected:
	bool recompute_current_device_validity();

private:
	char cur_path[256];
	struct libusb_context* usb_context;
	struct libusb_device **devs, *cur_dev;
	unsigned char dev_port_numbers[MAX_USB_DEVICE_PORT_PATH];
	int dev_index, dev_count;
	int controller_index;
	bool dev_valid;
};

#endif // CONTROLLER_LIBUSB_DEVICE_ENUMERATOR_H

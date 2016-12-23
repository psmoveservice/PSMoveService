#ifndef CONTROLLER_LIBUSB_DEVICE_ENUMERATOR_H
#define CONTROLLER_LIBUSB_DEVICE_ENUMERATOR_H

//-- includes -----
#include "DeviceEnumerator.h"
#include "USBApiInterface.h"

//-- definitions -----
class ControllerUSBDeviceEnumerator : public DeviceEnumerator
{
public:
	ControllerUSBDeviceEnumerator();
	ControllerUSBDeviceEnumerator(CommonDeviceState::eDeviceType deviceType);
	~ControllerUSBDeviceEnumerator();

	bool is_valid() const override;
	bool next() override;
	const char *get_path() const override;
	int get_vendor_id() const override;
	int get_product_id() const override;
	inline int get_contoller_index() const { return m_controllerIndex; }
	inline struct USBDeviceEnumerator* get_usb_device_enumerator() const { return m_usb_enumerator; }

private:
	char m_currentUSBPath[256];
	struct USBDeviceEnumerator* m_usb_enumerator;
	int m_controllerIndex;
};

#endif // CONTROLLER_LIBUSB_DEVICE_ENUMERATOR_H

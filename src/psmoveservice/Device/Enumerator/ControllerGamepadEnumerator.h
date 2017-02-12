#ifndef CONTROLLER_GAMEPAD_DEVICE_ENUMERATOR_H
#define CONTROLLER_GAMEPAD_DEVICE_ENUMERATOR_H

//-- includes -----
#include "DeviceEnumerator.h"
#include "USBApiInterface.h"

//-- definitions -----
class ControllerGamepadEnumerator : public DeviceEnumerator
{
public:
	ControllerGamepadEnumerator();
	ControllerGamepadEnumerator(CommonDeviceState::eDeviceType deviceTypeFilter);

	bool is_valid() const override;
	bool next() override;
	const char *get_path() const override;
	int get_vendor_id() const override;
	int get_product_id() const override;
	inline int get_contoller_index() const { return m_controllerIndex; }

private:
	char m_currentUSBPath[256];
	int m_controllerIndex;
};

#endif // CONTROLLER_GAMEPAD_DEVICE_ENUMERATOR_H

#ifndef HMD_DEVICE_ENUMERATOR_H
#define HMD_DEVICE_ENUMERATOR_H

#include "DeviceEnumerator.h"

class HMDDeviceEnumerator : public DeviceEnumerator
{
public:
	enum eAPIType
	{
		CommunicationType_INVALID= -1,
		CommunicationType_HID,
		CommunicationType_VIRTUAL,
		CommunicationType_ALL
	};

    HMDDeviceEnumerator(eAPIType api_type);
    ~HMDDeviceEnumerator();

    bool is_valid() const override;
    bool next() override;
    const char *get_path() const override;

	int get_vendor_id() const override;
	int get_product_id() const override;
	eAPIType get_api_type() const;
	const class HidHMDDeviceEnumerator *get_hid_hmd_enumerator() const;
	const class VirtualHMDDeviceEnumerator *get_virtual_hmd_enumerator() const;

private:
	eAPIType api_type;
	DeviceEnumerator **enumerators;
	int enumerator_count;
	int enumerator_index;
};

#endif // HMD_DEVICE_ENUMERATOR_H

#ifndef TRACKER_DEVICE_ENUMERATOR_H
#define TRACKER_DEVICE_ENUMERATOR_H

//-- includes -----
#include "DeviceEnumerator.h"
#include "USBApiInterface.h"

//-- definitions -----
class TrackerDeviceEnumerator : public DeviceEnumerator
{
public:
    TrackerDeviceEnumerator();
	~TrackerDeviceEnumerator();

    bool is_valid() const override;
    bool next() override;
	int get_vendor_id() const;
	int get_product_id() const;
    const char *get_path() const override;
    inline int get_camera_index() const { return m_cameraIndex; }
	inline struct USBDeviceEnumerator* get_usb_device_enumerator() const { return m_usb_enumerator; }

private:
    char m_currentUSBPath[256];
	struct USBDeviceEnumerator* m_usb_enumerator;
    int m_cameraIndex;
};

#endif // TRACKER_DEVICE_ENUMERATOR_H
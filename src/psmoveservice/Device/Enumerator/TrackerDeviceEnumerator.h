#ifndef TRACKER_DEVICE_ENUMERATOR_H
#define TRACKER_DEVICE_ENUMERATOR_H

//-- includes -----
#include "DeviceEnumerator.h"
#include "USBDeviceInfo.h" // for MAX_USB_DEVICE_PORT_PATH, t_usb_device_handle

//-- definitions -----
class TrackerDeviceEnumerator : public DeviceEnumerator
{
public:
    TrackerDeviceEnumerator();

    bool is_valid() const override;
    bool next() override;
    const char *get_path() const override;
    inline int get_camera_index() const { return m_cameraIndex; }
    inline t_usb_device_handle get_usb_device_handle() const { return m_USBDeviceHandle; }

private:
    char m_currentUSBPath[256];
    t_usb_device_handle m_USBDeviceHandle;
    int m_cameraIndex;
};

#endif // TRACKER_DEVICE_ENUMERATOR_H
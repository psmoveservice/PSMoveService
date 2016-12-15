#ifndef USB_DEVICE_INFO_H
#define USB_DEVICE_INFO_H

//-- includes -----

// -- macros ----
#define MAX_USB_DEVICE_PORT_PATH 7
#define GET_DEVICE_TYPE_INDEX(device_type)  (device_type & 0x0f)
#define GET_DEVICE_TYPE_CLASS(device_type)  (device_type & 0xf0)

//-- definitions -----
struct USBDeviceFilter
{
    unsigned short vendor_id;
    unsigned short product_id;
};

#endif  // USB_DEVICE_INFO_H
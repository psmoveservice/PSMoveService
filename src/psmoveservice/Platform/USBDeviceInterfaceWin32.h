#ifndef USB_DEVICE_INTERFACE_WIN32_H
#define USB_DEVICE_INTERFACE_WIN32_H

#ifndef _WIN32
#error "Only include this file in windows builds!"
#endif // _WIN32

namespace USBDeviceInterface
{
    enum DeviceClass
    {
        Camera,

        k_max_supported_device_classes
    };

    extern const char *k_reg_property_driver_desc;
    extern const char *k_reg_property_driver_version;
    extern const char *k_reg_property_matching_device_id;
    extern const char *k_reg_property_provider_name;
    extern const char *k_reg_property_vendor;

    bool fetch_driver_reg_property_for_usb_device(
        const DeviceClass deviceClass,
        const int vendor_id, 
        const int product_id, 
        const char *property_name, 
        char *buffer, 
        const int buffer_size);
};

#endif // USB_DEVICE_INTERFACE_WIN32_H
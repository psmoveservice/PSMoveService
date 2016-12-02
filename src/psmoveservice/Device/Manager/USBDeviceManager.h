#ifndef USB_DEVICE_MANAGER_H
#define USB_DEVICE_MANAGER_H

//-- includes -----
#include "USBDeviceRequest.h"
#include <functional>

//-- definitions -----
/// Manages async control and bulk transfer requests to usb devices via libusb.
class USBDeviceManager
{
public:
    USBDeviceManager(struct USBDeviceFilter *device_whitelist, size_t device_whitelist_length);
    virtual ~USBDeviceManager();

    static inline USBDeviceManager *getInstance()
    {
        return m_instance;
    }

    inline class USBDeviceManagerImpl *getImplementation()
    {
        return m_implementation_ptr;
    }

    // -- System ----
    bool startup(); /**< Initialize the libusb thread. */
    void update();  /**< Process events from the libusb thread. */
    void shutdown();/**< Shutdown the libusb thread. */

    // -- Device Actions ----
    bool openUSBDevice(t_usb_device_handle handle);
    void closeUSBDevice(t_usb_device_handle handle);

    // -- Device Queries ----
	static const char *getErrorString(eUSBResultCode result_code);
    int getUSBDeviceCount() const;
    t_usb_device_handle getFirstUSBDeviceHandle() const;
    t_usb_device_handle getNextUSBDeviceHandle(t_usb_device_handle handle) const;
    bool getUSBDeviceInfo(t_usb_device_handle handle, USBDeviceFilter &outDeviceInfo) const;
    bool getUSBDevicePath(t_usb_device_handle handle, char *outBuffer, size_t bufferSize) const;
    bool getUSBDevicePortPath(t_usb_device_handle handle, char *outBuffer, size_t bufferSize) const;
    bool getIsUSBDeviceOpen(t_usb_device_handle handle) const;

    // -- Request Queue ----
    // Send the transfer request to the worker thread asynchronously
    bool submitTransferRequestAsync(
        const USBTransferRequest &request,
        std::function<void(USBTransferResult&)> callback = noop_transfer_result_callback);
	// Send the transfer request to the worker thread and block until it completes
	USBTransferResult submitTransferRequestBlocking(const USBTransferRequest &request);

    static void noop_transfer_result_callback(USBTransferResult &result) {};

private:
    // Always use the overloaded constructor
    USBDeviceManager();

    /// private implementation
    class USBDeviceManagerImpl *m_implementation_ptr;

    /// Singleton instance of the class
    /// Assigned in startup, cleared in teardown
    static USBDeviceManager *m_instance;
};

#endif  // USB_DEVICE_MANAGER_H
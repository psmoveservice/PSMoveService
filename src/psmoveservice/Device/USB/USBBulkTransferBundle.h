#ifndef USB_BULK_TRANSFER_BUNDLE_H
#define USB_BULK_TRANSFER_BUNDLE_H

//-- includes -----
#include "USBDeviceRequest.h"

//-- definitions -----
/// Internal class used to manage a set of libusb bulk transfer packets.
class USBBulkTransferBundle
{
public:
    USBBulkTransferBundle(
        const USBRequestPayload_BulkTransfer &request,
        struct libusb_device *dev,
        struct libusb_device_handle *dev_handle);
    virtual ~USBBulkTransferBundle();

    // Interface
    bool initialize();
    bool startTransfers();
    void cancelTransfers();

    // Events
    void notifyActiveTransfersDecremented();

    // Accessors
    inline const USBRequestPayload_BulkTransfer &getTransferRequest() const
    {
        return m_request;
    }

    inline t_usb_device_handle getUSBDeviceHandle() const
    {
        return m_request.usb_device_handle;
    }

    inline int getActiveTransferCount() const
    {
        return m_active_transfer_count;
    }

    // Helpers
    // Search for an input transfer endpoint in the endpoint descriptor
    // of the device interfaces alt_settings
    static bool find_bulk_transfer_endpoint(struct libusb_device *device, unsigned char &out_endpoint_addr);

protected:
    void dispose();

private:
    USBRequestPayload_BulkTransfer m_request;
    struct libusb_device *m_device;
    struct libusb_device_handle *m_device_handle;

    int m_active_transfer_count;
    bool m_is_canceled;
    struct libusb_transfer** bulk_transfer_requests;
    unsigned char* transfer_buffer;
};

#endif // USB_BULK_TRANSFER_BUNDLE_H
#ifndef LIB_USB_BULK_TRANSFER_BUNDLE_H
#define LIB_USB_BULK_TRANSFER_BUNDLE_H

//-- includes -----
#include "USBApiInterface.h"
#include "USBDeviceRequest.h"

//-- definitions -----
/// Internal class used to manage a set of libusb bulk transfer packets.
class LibUSBBulkTransferBundle : public IUSBBulkTransferBundle
{
public:
    LibUSBBulkTransferBundle(
        const USBDeviceState *device_state,
		const struct USBRequestPayload_BulkTransfer *request);
    virtual ~LibUSBBulkTransferBundle();

    // Interface
    bool initialize() override;
    bool startTransfers() override;
    void cancelTransfers() override;

    // Events
    void notifyActiveTransfersDecremented();

    // Accessors
	const USBRequestPayload_BulkTransfer &getTransferRequest() const override;
	t_usb_device_handle getUSBDeviceHandle() const override;
	int getActiveTransferCount() const override;

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
#ifndef USB_DEVICE_REQUEST_H
#define USB_DEVICE_REQUEST_H

//-- includes -----
#include "USBApiInterface.h"
#include "USBDeviceInfo.h"
#include <functional>

//-- constants -----
// HID Class-Specific Requests values. See section 7.2 of the HID specifications 
#define HID_GET_REPORT                0x01 
#define HID_GET_IDLE                  0x02 
#define HID_GET_PROTOCOL              0x03 
#define HID_SET_REPORT                0x09 
#define HID_SET_IDLE                  0x0A 
#define HID_SET_PROTOCOL              0x0B 
#define HID_REPORT_TYPE_INPUT         0x01 
#define HID_REPORT_TYPE_OUTPUT        0x02 
#define HID_REPORT_TYPE_FEATURE       0x03 

// In: device-to-host
#define USB_ENDPOINT_IN               0x80

// Out: host-to-device
#define USB_ENDPOINT_OUT              0x00

// Request type bits of the bmRequestType field in control transfers.
enum usb_request_type {
	/** Standard */
	USB_REQUEST_TYPE_STANDARD = (0x00 << 5),

	/** Class */
	USB_REQUEST_TYPE_CLASS = (0x01 << 5),

	/** Vendor */
	USB_REQUEST_TYPE_VENDOR = (0x02 << 5),

	/** Reserved */
	USB_REQUEST_TYPE_RESERVED = (0x03 << 5)
};

// Recipient bits of the bmRequestType" field in control transfers. Values 4 through 31 are reserved. 
enum usb_request_recipient
{
	/** Device */
	USB_RECIPIENT_DEVICE = 0x00,

	/** Interface */
	USB_RECIPIENT_INTERFACE = 0x01,

	/** Endpoint */
	USB_RECIPIENT_ENDPOINT = 0x02,

	/** Other */
	USB_RECIPIENT_OTHER = 0x03,
};

#define USB_CTRL_IN                   USB_ENDPOINT_IN|USB_REQUEST_TYPE_CLASS|USB_RECIPIENT_INTERFACE 
#define USB_CTRL_OUT                  USB_ENDPOINT_OUT|USB_REQUEST_TYPE_CLASS|USB_RECIPIENT_INTERFACE 

enum eUSBTransferRequestType
{
	_USBRequestType_InterruptTransfer,
    _USBRequestType_ControlTransfer,
    _USBRequestType_StartBulkTransfer,
    _USBRequestType_CancelBulkTransfer,
};

enum eUSBTransferResultType
{
	_USBResultType_InterrupTransfer,
    _USBResultType_ControlTransfer,
    _USBResultType_BulkTransfer
};

#define MAX_INTERRUPT_TRANSFER_PAYLOAD 64
#define MAX_CONTROL_TRANSFER_PAYLOAD 64

//-- typedefs -----
typedef void(*usb_bulk_transfer_cb_fn)(unsigned char *packet_data, int packet_length, void *userdata);

//-- definitions -----

//-- Request Structures --
struct USBRequestPayload_InterruptTransfer
{
	t_usb_device_handle usb_device_handle;
	unsigned int timeout;
	unsigned int length;
	unsigned char data[MAX_INTERRUPT_TRANSFER_PAYLOAD];
	unsigned char endpoint;
};

struct USBRequestPayload_ControlTransfer
{
    t_usb_device_handle usb_device_handle;
    unsigned int timeout;
    unsigned short wValue;
    unsigned short wIndex;
    unsigned short wLength;
    unsigned char data[MAX_CONTROL_TRANSFER_PAYLOAD];
    unsigned char bmRequestType;
    unsigned char bRequest;
};

struct USBRequestPayload_BulkTransfer
{
    t_usb_device_handle usb_device_handle;
    int transfer_packet_size;
    int in_flight_transfer_packet_count;
    usb_bulk_transfer_cb_fn on_data_callback;
    void *transfer_callback_userdata;
    bool bAutoResubmit;
};

struct USBRequestPayload_CancelBulkTransfer
{
    t_usb_device_handle usb_device_handle;
};

struct USBTransferRequest
{
    union
    {
		USBRequestPayload_InterruptTransfer interrupt_transfer;
        USBRequestPayload_ControlTransfer control_transfer;
        USBRequestPayload_BulkTransfer start_bulk_transfer;
        USBRequestPayload_CancelBulkTransfer cancel_bulk_transfer;
    } payload;
    eUSBTransferRequestType request_type;
};

//-- Result Structures --
struct USBResultPayload_BulkTransfer
{
    t_usb_device_handle usb_device_handle;
    eUSBResultCode result_code;
};

struct USBResultPayload_ControlTransfer
{
    t_usb_device_handle usb_device_handle;
    eUSBResultCode result_code;
    unsigned char data[MAX_CONTROL_TRANSFER_PAYLOAD];
    int dataLength;
};

struct USBResultPayload_InterruptTransfer
{
	t_usb_device_handle usb_device_handle;
	eUSBResultCode result_code;
	unsigned char data[MAX_INTERRUPT_TRANSFER_PAYLOAD];
	int dataLength;
};

struct USBTransferResult
{
    union 
    {
		USBResultPayload_InterruptTransfer interrupt_transfer;
        USBResultPayload_ControlTransfer control_transfer;
        USBResultPayload_BulkTransfer bulk_transfer;
    } payload;
    eUSBTransferResultType result_type;
};

struct USBTransferRequestState
{
	USBTransferRequest request;
	std::function<void(USBTransferResult&)> callback;
};

struct USBTransferResultState
{
	USBTransferResult result;
	std::function<void(USBTransferResult&)> callback;
};

#endif // USB_DEVICE_REQUEST_H
//-- includes -----
#include "LibUSBApi.h"
#include "LibUSBBulkTransferBundle.h"
#include "ServerLog.h"
#include "ServerUtility.h"
#include "USBDeviceInfo.h"
#include "USBDeviceRequest.h"
#include "USBDeviceManager.h"

#include "libusb.h"

#include <assert.h>

//-- definitions -----
struct APIContext
{
	libusb_context* lib_usb_context;
};

struct LibUSBDeviceEnumerator : USBDeviceEnumerator
{
	libusb_device **device_list;
};

//-- private methods -----
static void LIBUSB_CALL interrupt_transfer_cb(struct libusb_transfer *transfer);
static void LIBUSB_CALL control_transfer_cb(struct libusb_transfer *transfer);

static bool libusb_device_get_path(libusb_device *dev, char *outBuffer, size_t bufferSize);
static bool libusb_device_get_port_path(libusb_device *dev, char *outBuffer, size_t bufferSize);

//-- public interface -----
LibUSBApi::LibUSBApi() : IUSBApi()
{
	m_apiContext = new APIContext;
}

LibUSBApi::~LibUSBApi()
{
	delete m_apiContext;
}

bool LibUSBApi::startup()
{
	libusb_init(&m_apiContext->lib_usb_context);
	libusb_set_debug(m_apiContext->lib_usb_context, 1);

	return true;
}

void LibUSBApi::poll()
{
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 50 * 1000; // ms

	// Give libusb a chance to process transfer requests and post events
	libusb_handle_events_timeout_completed(m_apiContext->lib_usb_context, &tv, NULL);
}

void LibUSBApi::shutdown()
{
	if (m_apiContext->lib_usb_context != nullptr)
	{
		libusb_exit(m_apiContext->lib_usb_context);
		m_apiContext->lib_usb_context = nullptr;
	}
}

USBDeviceEnumerator* LibUSBApi::device_enumerator_create()
{
	LibUSBDeviceEnumerator *libusb_enumerator = new LibUSBDeviceEnumerator;
	memset(libusb_enumerator, 0, sizeof(LibUSBDeviceEnumerator));

	if (libusb_get_device_list(m_apiContext->lib_usb_context, &libusb_enumerator->device_list) < 0)
	{
		SERVER_LOG_INFO("usb_enumerate") << "Unable to fetch device list.";
	}

	return libusb_enumerator;
}

bool LibUSBApi::device_enumerator_is_valid(USBDeviceEnumerator* enumerator)
{
	LibUSBDeviceEnumerator *libusb_enumerator = static_cast<LibUSBDeviceEnumerator *>(enumerator);

	return libusb_enumerator->device_list[libusb_enumerator->device_index] != nullptr;
}

bool LibUSBApi::device_enumerator_get_filter(const USBDeviceEnumerator* enumerator, USBDeviceFilter *outDeviceInfo) const
{
	const LibUSBDeviceEnumerator *libusb_enumerator = static_cast<const LibUSBDeviceEnumerator *>(enumerator);
	const libusb_device *dev = libusb_enumerator->device_list[libusb_enumerator->device_index];
	bool bSuccess = false;

	if (dev != nullptr)
	{
		struct libusb_device_descriptor dev_desc;
		libusb_get_device_descriptor(const_cast<libusb_device *>(dev), &dev_desc);

		outDeviceInfo->product_id = dev_desc.idProduct;
		outDeviceInfo->vendor_id = dev_desc.idVendor;
		bSuccess = true;
	}

	return bSuccess;
}

bool LibUSBApi::device_enumerator_get_path(const USBDeviceEnumerator* enumerator, char *outBuffer, size_t bufferSize) const
{
	const LibUSBDeviceEnumerator *libusb_enumerator = static_cast<const LibUSBDeviceEnumerator *>(enumerator);
	const libusb_device *dev = libusb_enumerator->device_list[libusb_enumerator->device_index];
	bool bSuccess = false;

	if (dev != nullptr)
	{
		bSuccess = libusb_device_get_path(const_cast<libusb_device *>(dev), outBuffer, bufferSize);
	}

	return bSuccess;
}

void LibUSBApi::device_enumerator_next(USBDeviceEnumerator* enumerator)
{
	LibUSBDeviceEnumerator *libusb_enumerator = static_cast<LibUSBDeviceEnumerator *>(enumerator);

	if (device_enumerator_is_valid(libusb_enumerator))
	{
		++libusb_enumerator->device_index;
	}
}

void LibUSBApi::device_enumerator_dispose(USBDeviceEnumerator* enumerator)
{
	LibUSBDeviceEnumerator *libusb_enumerator = static_cast<LibUSBDeviceEnumerator *>(enumerator);
	assert(libusb_enumerator != nullptr);

	if (libusb_enumerator->device_list != nullptr)
	{
		libusb_free_device_list(libusb_enumerator->device_list, 1);
	}

	delete libusb_enumerator;
}

USBDeviceState *LibUSBApi::open_usb_device(USBDeviceEnumerator* enumerator)
{
	LibUSBDeviceEnumerator *libusb_enumerator = static_cast<LibUSBDeviceEnumerator *>(enumerator);
	LibUSBDeviceState *libusb_device_state = nullptr;

	if (device_enumerator_is_valid(enumerator))
	{
		bool bOpened = false;
		
		libusb_device_state = new LibUSBDeviceState;
		libusb_device_state->clear();

		libusb_device_state->device = libusb_enumerator->device_list[libusb_enumerator->device_index];
		libusb_ref_device(libusb_device_state->device);

		int res = libusb_open(libusb_device_state->device, &libusb_device_state->device_handle);
		if (res == LIBUSB_SUCCESS)
		{
			res = libusb_claim_interface(libusb_device_state->device_handle, 0);
			if (res == 0)
			{
				libusb_device_state->is_interface_claimed = true;
				bOpened = true;

				SERVER_LOG_INFO("USBAsyncRequestManager::openUSBDevice") << "Successfully opened device " << libusb_device_state->public_handle;
			}
			else
			{
				SERVER_LOG_ERROR("USBAsyncRequestManager::openUSBDevice") << "Failed to claim USB device: " << libusb_error_name(res);
			}
		}
		else
		{
			SERVER_LOG_ERROR("USBAsyncRequestManager::openUSBDevice") << "Failed to open USB device: " << libusb_error_name(res);
		}

		if (!bOpened)
		{
			close_usb_device(libusb_device_state);
			libusb_device_state= nullptr;
		}
	}

	return libusb_device_state;
}

void LibUSBApi::close_usb_device(USBDeviceState* device_state)
{
	if (device_state != nullptr)
	{
		LibUSBDeviceState *libusb_device_state = static_cast<LibUSBDeviceState *>(device_state);

		if (libusb_device_state->is_interface_claimed)
		{
			SERVER_LOG_INFO("USBAsyncRequestManager::closeUSBDevice") << "Released USB interface on handle " << libusb_device_state->public_handle;
			libusb_release_interface(libusb_device_state->device_handle, 0);
			libusb_device_state->is_interface_claimed = false;
		}

		if (libusb_device_state->device_handle != nullptr)
		{
			SERVER_LOG_INFO("USBAsyncRequestManager::closeUSBDevice") << "Close USB device on handle " << libusb_device_state->public_handle;
			libusb_close(libusb_device_state->device_handle);
			libusb_device_state->device_handle = nullptr;
		}

		if (libusb_device_state->device != nullptr)
		{
			libusb_unref_device(libusb_device_state->device);
			libusb_device_state->device = nullptr;
		}

		delete libusb_device_state;
	}
}

bool LibUSBApi::can_usb_device_be_opened(USBDeviceEnumerator* enumerator, char *outReason, size_t bufferSize)
{
	LibUSBDeviceEnumerator *libusb_enumerator = static_cast<LibUSBDeviceEnumerator *>(enumerator);
	bool bCanBeOpened= false;

	if (device_enumerator_is_valid(enumerator))
	{
		bool bOpened = false;
		
		struct libusb_device *device = libusb_enumerator->device_list[libusb_enumerator->device_index];
		struct libusb_device_handle *device_handle= nullptr;

		int libusb_result = libusb_open(device, &device_handle);

		// Can be opened if we can open the device now or it's already opened
		if (libusb_result == LIBUSB_SUCCESS || libusb_result == LIBUSB_ERROR_ACCESS)
		{
			if (libusb_result == LIBUSB_SUCCESS)
			{
				strncpy(outReason, "SUCCESS(can be opened)", bufferSize);
				libusb_close(device_handle);
			}
			else
			{
				strncpy(outReason, "SUCCESS(already opened)", bufferSize);
			}
			
			bCanBeOpened= true;
		}
		else
		{
			strncpy(outReason, libusb_strerror(static_cast<libusb_error>(libusb_result)), bufferSize);
		}
	}

	return bCanBeOpened;
}

eUSBResultCode LibUSBApi::submit_interrupt_transfer(
	const USBDeviceState* device_state,
	const USBTransferRequestState *requestState)
{
	USBTransferRequestState *requestStateOnHeap = nullptr;
	const LibUSBDeviceState *libusb_device_state = static_cast<const LibUSBDeviceState *>(device_state);

	eUSBResultCode result_code= _USBResultCode_Started;
	bool bSuccess = true;

	struct libusb_transfer *transfer = libusb_alloc_transfer(0);
	if (transfer == nullptr)
	{
		result_code = _USBResultCode_NoMemory;
		bSuccess = false;
	}

	if (bSuccess)
	{
		requestStateOnHeap = new USBTransferRequestState;
		if (requestStateOnHeap != nullptr)
		{
			// Make a copy of the request on the heap so that it's safe
			// to point to in the transfer userdata
			requestStateOnHeap->request = requestState->request;
			requestStateOnHeap->callback = requestState->callback;
		}
		else
		{
			result_code = _USBResultCode_NoMemory;
			bSuccess = false;
		}
	}

	if (bSuccess)
	{
		int libusb_result = LIBUSB_SUCCESS;

		libusb_fill_interrupt_transfer(
			transfer,
			libusb_device_state->device_handle,
			requestStateOnHeap->request.payload.interrupt_transfer.endpoint,
			requestStateOnHeap->request.payload.interrupt_transfer.data,
			requestStateOnHeap->request.payload.interrupt_transfer.length,
			interrupt_transfer_cb,
			requestStateOnHeap,
			requestStateOnHeap->request.payload.interrupt_transfer.timeout);

		libusb_result = libusb_submit_transfer(transfer);
		if (libusb_result == LIBUSB_SUCCESS)
		{
			result_code = _USBResultCode_Started;
		}
		else
		{
			result_code = _USBResultCode_SubmitFailed;
			bSuccess = false;
		}
	}

	if (!bSuccess)
	{
		if (requestStateOnHeap != nullptr)
		{
			delete requestStateOnHeap;
		}

		if (transfer != nullptr)
		{
			libusb_free_transfer(transfer);
		}
	}

	return result_code;
}

static void LIBUSB_CALL interrupt_transfer_cb(struct libusb_transfer *transfer)
{
	const USBTransferRequestState *requestStateOnHeap = reinterpret_cast<const USBTransferRequestState *>(transfer->user_data);
	const USBRequestPayload_InterruptTransfer *request = &requestStateOnHeap->request.payload.interrupt_transfer;

	USBTransferResult result;

	memset(&result, 0, sizeof(USBTransferResult));
	result.result_type = _USBResultType_InterrupTransfer;
	result.payload.control_transfer.usb_device_handle = request->usb_device_handle;

	if ((request->endpoint & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN &&
		transfer->actual_length > 0)
	{
		// Libusb will write the result on the request data buffer since that's the buffer pointer we gave it
		memcpy(&result.payload.control_transfer.data, request->data, transfer->actual_length);
	}
	result.payload.control_transfer.dataLength = transfer->actual_length;

	switch (transfer->status)
	{
	case LIBUSB_TRANSFER_COMPLETED:
		result.payload.control_transfer.result_code = _USBResultCode_Completed;
		break;
	case LIBUSB_TRANSFER_TIMED_OUT:
		result.payload.control_transfer.result_code = _USBResultCode_TimedOut;
		break;
	case LIBUSB_TRANSFER_STALL:
		result.payload.control_transfer.result_code = _USBResultCode_Pipe;
		break;
	case LIBUSB_TRANSFER_NO_DEVICE:
		result.payload.control_transfer.result_code = _USBResultCode_DeviceNotOpen;
		break;
	case LIBUSB_TRANSFER_OVERFLOW:
		result.payload.control_transfer.result_code = _USBResultCode_Overflow;
		break;
	case LIBUSB_TRANSFER_ERROR:
		result.payload.control_transfer.result_code = _USBResultCode_GeneralError;
		break;
	case LIBUSB_TRANSFER_CANCELLED:
		result.payload.control_transfer.result_code = _USBResultCode_Canceled;
		break;
	default:
		result.payload.control_transfer.result_code = _USBResultCode_GeneralError;
	}

#if defined(DEBUG_USB)
	if ((request->endpoint & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_OUT)
	{
		debug("USBMgr RESULT: interrupt transfer write - dev: %d, endpoint: 0x%X, length: %d -> %s\n",
			requestStateOnHeap->request.payload.control_transfer.usb_device_handle,
			request->endpoint,
			request->length,
			transfer->status == LIBUSB_TRANSFER_COMPLETED ? "SUCCESS" : "FAILED");
	}
	else
	{
		debug("USBMgr RESULT: control transfer read - dev: %d, endpoint: 0x%X, length: %d -> 0x%X (%s)\n",
			requestStateOnHeap->request.payload.control_transfer.usb_device_handle,
			request->endpoint,
			request->length,
			transfer->status == LIBUSB_TRANSFER_COMPLETED ? "SUCCESS" : "FAILED");
	}
#endif

	// Add the result to the outgoing result queue
	usb_device_post_transfer_result(result, requestStateOnHeap->callback);

	// Free request state stored in the heap now that the result is posted
	delete requestStateOnHeap;

	// Free the libusb allocated transfer
	libusb_free_transfer(transfer);
}

eUSBResultCode LibUSBApi::submit_control_transfer(
	const USBDeviceState* device_state,
	const USBTransferRequestState *requestState)
{
	const USBRequestPayload_ControlTransfer &request = requestState->request.payload.control_transfer;

	USBTransferRequestState *requestStateOnHeap = nullptr;
	const LibUSBDeviceState *libusb_device_state = static_cast<const LibUSBDeviceState *>(device_state);

	eUSBResultCode result_code = _USBResultCode_Started;
	bool bSuccess = true;

	struct libusb_transfer *transfer = libusb_alloc_transfer(0);
	if (transfer == nullptr)
	{
		result_code = _USBResultCode_NoMemory;
		bSuccess = false;
	}

	if (bSuccess)
	{
		requestStateOnHeap = new USBTransferRequestState;
		if (requestStateOnHeap != nullptr)
		{
			// Make a copy of the request on the heap so that it's safe
			// to point to in the transfer userdata
			requestStateOnHeap->request = requestState->request;
			requestStateOnHeap->callback = requestState->callback;
		}
		else
		{
			result_code = _USBResultCode_NoMemory;
			bSuccess = false;
		}
	}

	unsigned char *buffer= nullptr;
	if (bSuccess)
	{
		buffer = (unsigned char*)malloc(LIBUSB_CONTROL_SETUP_SIZE + requestState->request.payload.control_transfer.wLength);
		if (buffer == nullptr)
		{
			result_code = _USBResultCode_NoMemory;
			bSuccess = false;
		}
	}

	if (bSuccess)
	{
		int libusb_result = LIBUSB_SUCCESS;

		libusb_fill_control_setup(
			buffer,
			request.bmRequestType,
			request.bRequest,
			request.wValue,
			request.wIndex,
			request.wLength);

		if ((request.bmRequestType & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_OUT)
		{
			memcpy(buffer + LIBUSB_CONTROL_SETUP_SIZE, request.data, request.wLength);
		}

		libusb_fill_control_transfer(
			transfer,
			libusb_device_state->device_handle,
			buffer,
			control_transfer_cb,
			requestStateOnHeap,
			request.timeout);
		transfer->flags = LIBUSB_TRANSFER_FREE_BUFFER;

		libusb_result = libusb_submit_transfer(transfer);
		if (libusb_result == LIBUSB_SUCCESS)
		{
			result_code = _USBResultCode_Started;
		}
		else
		{
			result_code = _USBResultCode_SubmitFailed;
			bSuccess = false;
		}
	}

	if (!bSuccess)
	{
		if (transfer != nullptr)
		{
            // This will free the buffer since LIBUSB_TRANSFER_FREE_BUFFER is set on the transfer
			libusb_free_transfer(transfer);
		}
        else if (buffer != nullptr)
		{
			free(buffer);
		}

		if (requestStateOnHeap != nullptr)
		{
			delete requestStateOnHeap;
		}
	}

	return result_code;
}

static void LIBUSB_CALL control_transfer_cb(struct libusb_transfer *transfer)
{
	const USBTransferRequestState *requestStateOnHeap = reinterpret_cast<const USBTransferRequestState *>(transfer->user_data);
	const USBRequestPayload_ControlTransfer *request = &requestStateOnHeap->request.payload.control_transfer;

	USBTransferResult result;

	memset(&result, 0, sizeof(USBTransferResult));
	result.result_type = _USBResultType_ControlTransfer;
	result.payload.control_transfer.usb_device_handle = request->usb_device_handle;

	if ((request->bmRequestType & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN &&
		transfer->actual_length > 0)
	{
		memcpy(&result.payload.control_transfer.data, libusb_control_transfer_get_data(transfer), transfer->actual_length);
	}
	result.payload.control_transfer.dataLength = transfer->actual_length;

	switch (transfer->status)
	{
	case LIBUSB_TRANSFER_COMPLETED:
		result.payload.control_transfer.result_code = _USBResultCode_Completed;
		break;
	case LIBUSB_TRANSFER_TIMED_OUT:
		result.payload.control_transfer.result_code = _USBResultCode_TimedOut;
		break;
	case LIBUSB_TRANSFER_STALL:
		result.payload.control_transfer.result_code = _USBResultCode_Pipe;
		break;
	case LIBUSB_TRANSFER_NO_DEVICE:
		result.payload.control_transfer.result_code = _USBResultCode_DeviceNotOpen;
		break;
	case LIBUSB_TRANSFER_OVERFLOW:
		result.payload.control_transfer.result_code = _USBResultCode_Overflow;
		break;
	case LIBUSB_TRANSFER_ERROR:
		result.payload.control_transfer.result_code = _USBResultCode_GeneralError;
		break;
	case LIBUSB_TRANSFER_CANCELLED:
		result.payload.control_transfer.result_code = _USBResultCode_Canceled;
		break;
	default:
		result.payload.control_transfer.result_code = _USBResultCode_GeneralError;
	}

#if defined(DEBUG_USB)
	if ((request->bmRequestType & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_OUT)
	{
		debug("USBMgr RESULT: control transfer write - dev: %d, reg: 0x%X, value: 0x%x -> %s\n",
			requestStateOnHeap->request.payload.control_transfer.usb_device_handle,
			request->wIndex,
			request->data[0],
			transfer->status == LIBUSB_TRANSFER_COMPLETED ? "SUCCESS" : "FAILED");
	}
	else
	{
		debug("USBMgr RESULT: control transfer read - dev: %d, reg: 0x%X -> 0x%X (%s)\n",
			requestStateOnHeap->request.payload.control_transfer.usb_device_handle,
			request->wIndex,
			result.payload.control_transfer.data,
			transfer->status == LIBUSB_TRANSFER_COMPLETED ? "SUCCESS" : "FAILED");
	}
#endif

	// Add the result to the outgoing result queue
	usb_device_post_transfer_result(result, requestStateOnHeap->callback);

	// Free request state stored in the heap now that the result is posted
	delete requestStateOnHeap;

	// Free the libusb allocated transfer
	libusb_free_transfer(transfer);
}

IUSBBulkTransferBundle *LibUSBApi::allocate_bulk_transfer_bundle(const USBDeviceState *device_state, const USBRequestPayload_BulkTransfer *request)
{
	return new LibUSBBulkTransferBundle(device_state, request);
}

bool LibUSBApi::get_usb_device_filter(const USBDeviceState* device_state, struct USBDeviceFilter *outDeviceInfo) const
{
	bool bSuccess = false;

	if (device_state != nullptr)
	{
		const LibUSBDeviceState *libusb_device_state = static_cast<const LibUSBDeviceState *>(device_state);
		const libusb_device *dev = libusb_device_state->device;

		if (dev != nullptr)
		{
			struct libusb_device_descriptor dev_desc;
			libusb_get_device_descriptor(const_cast<libusb_device *>(dev), &dev_desc);

			outDeviceInfo->product_id = dev_desc.idProduct;
			outDeviceInfo->vendor_id = dev_desc.idVendor;
			bSuccess = true;
		}
	}

	return bSuccess;
}

bool LibUSBApi::get_usb_device_path(USBDeviceState* device_state, char *outBuffer, size_t bufferSize) const
{
	bool bSuccess = false;

	if (device_state != nullptr)
	{
		LibUSBDeviceState *libusb_device_state = static_cast<LibUSBDeviceState *>(device_state);
		libusb_device *dev = libusb_device_state->device;

		bSuccess = libusb_device_get_path(dev, outBuffer, bufferSize);
	}

	return bSuccess;
}

bool LibUSBApi::get_usb_device_port_path(USBDeviceState* device_state, char *outBuffer, size_t bufferSize) const
{
	bool bSuccess = false;

	if (device_state != nullptr)
	{
		LibUSBDeviceState *libusb_device_state = static_cast<LibUSBDeviceState *>(device_state);
		libusb_device *dev = libusb_device_state->device;

		bSuccess = libusb_device_get_port_path(dev, outBuffer, bufferSize);
	}

	return bSuccess;
}

static bool libusb_device_get_path(libusb_device *dev, char *outBuffer, size_t bufferSize)
{
	bool bSuccess = false;

	if (dev != nullptr)
	{
		struct libusb_device_descriptor dev_desc;
		libusb_get_device_descriptor(dev, &dev_desc);

		char port_path[32];
		if (libusb_device_get_port_path(dev, port_path, sizeof(port_path)))
		{
			int nCharsWritten =
				ServerUtility::format_string(
					outBuffer, bufferSize,
					"USB\\VID_%04X&PID_%04X\\%s",
					dev_desc.idVendor, dev_desc.idProduct, port_path);

			bSuccess = (nCharsWritten > 0);
		}
	}

	return bSuccess;
}

static bool libusb_device_get_port_path(libusb_device *dev, char *outBuffer, size_t bufferSize)
{
	bool bSuccess = false;

	if (dev != nullptr)
	{
		uint8_t port_numbers[MAX_USB_DEVICE_PORT_PATH];

		memset(outBuffer, 0, bufferSize);

		memset(port_numbers, 0, sizeof(port_numbers));
		int port_count = libusb_get_port_numbers(dev, port_numbers, MAX_USB_DEVICE_PORT_PATH);
		int bus_id = libusb_get_bus_number(dev);

		ServerUtility::format_string(outBuffer, bufferSize, "b%d", bus_id);
		if (port_count > 0)
		{
			bSuccess = true;

			for (int port_index = 0; port_index < port_count; ++port_index)
			{
				uint8_t port_number = port_numbers[port_index];

				if (ServerUtility::format_string(
					outBuffer, bufferSize,
					(port_index == 0) ? "%s_p%d" : "%s.%d",
					outBuffer, port_number) < 0)
				{
					bSuccess = false;
					break;
				}
			}
		}
	}

	return bSuccess;
}
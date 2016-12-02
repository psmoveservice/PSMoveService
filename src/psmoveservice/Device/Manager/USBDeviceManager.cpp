//-- includes -----
#include "USBDeviceManager.h"
#include "USBDeviceInfo.h"
#include "USBBulkTransferBundle.h"
#include "ServerLog.h"
#include "ServerUtility.h"

#include "libusb.h"

#include <atomic>
#include <thread>
#include <vector>

#include <boost/lockfree/spsc_queue.hpp>

//-- macros -----
//#define DEBUG_USBDEVICEMANAGER
#if defined(DEBUG_USBDEVICEMANAGER)
#define debug(...) fprintf(stdout, __VA_ARGS__)
#else
#define debug(...) 
#endif

//-- private implementation -----
// -USBAsyncRequestManagerImpl-
/// Internal implementation of the USB async request manager.
class USBDeviceManagerImpl
{
protected:
    struct RequestState
    {
        USBTransferRequest request;
        std::function<void(USBTransferResult&)> callback;
    };

    struct ResultState
    {
        USBTransferResult result;
        std::function<void(USBTransferResult&)> callback;
    };

    struct LibUSBDeviceState
    {
        t_usb_device_handle handle;
        libusb_device *device;
        libusb_device_handle *device_handle;
        bool is_interface_claimed;
    };

public:
    USBDeviceManagerImpl(struct USBDeviceFilter *device_whitelist, size_t device_whitelist_length)
        : m_usb_context(nullptr)
        , m_exit_signaled({ false })
        , m_active_control_transfers(0)
		, m_active_interrupt_transfers(0)
        , m_thread_started(false)
    {
        for (size_t list_index = 0; list_index < device_whitelist_length; ++list_index)
        {
            m_device_whitelist.push_back(device_whitelist[list_index]);
        }
    }

    virtual ~USBDeviceManagerImpl()
    {
    }

    // -- System ----
    bool startup()
    {
        bool bSuccess= true;

        SERVER_LOG_INFO("USBAsyncRequestManager::startup") << "Initializing libusb context";
        libusb_init(&m_usb_context);
        libusb_set_debug(m_usb_context, 1);

        // Get a list of all of the available USB devices that are on the white-list
        rebuildFilteredDeviceList();

        return bSuccess;
    }

    void update()
    {
        // If the thread terminated, reset the started and exited flags
        if (m_exit_signaled)
        {
            m_thread_started= false;
            m_exit_signaled= false;
        }

        if (!m_thread_started)
        {
            // If the thread isn't running, process the request as well as the results
            while(processRequests())
            {
                processResults();
            }

            // If there are bulk transfers now active, start up the worker thread to manage them
            if (m_active_bulk_transfer_bundles.size() > 0)
            {
                startWorkerThread();
            }
        }
        else
        {
            // If the thread is running, only process the results since the thread is handling the requests
            processResults();
        }
    }

    void shutdown()
    {
        // Shutdown any async transfers
        if (m_thread_started)
        {
            stopWorkerThread();
        }

        // Cleanup any requests
        requestProcessingTeardown();

        if (m_usb_context != nullptr)
        {
            // Unref any libusb devices
            freeDeviceStateList();

            // Free the libusb context
            libusb_exit(m_usb_context);
            m_usb_context= nullptr;
        }
    }

    // -- Device Actions ----
    bool openUSBDevice(t_usb_device_handle handle)
    {
        bool bOpened= false;

        if (!getIsUSBDeviceOpen(handle))
        {
            LibUSBDeviceState *state= get_libusb_state_from_handle(handle);

            if (state != nullptr)
            {
                int res = libusb_open(state->device, &state->device_handle);
                if (res == 0)
                {
                    res = libusb_claim_interface(state->device_handle, 0);
                    if (res == 0)
                    {
                        state->is_interface_claimed = true;
                        bOpened = true;

                        SERVER_LOG_INFO("USBAsyncRequestManager::openUSBDevice") << "Successfully opened device " << handle;
                    }
                    else
                    {
                        SERVER_LOG_ERROR("USBAsyncRequestManager::openUSBDevice") << "Failed to claim USB device: " << res;
                    }
                }
                else
                {
                    SERVER_LOG_ERROR("USBAsyncRequestManager::openUSBDevice") << "Failed to open USB device: " << res;
                }
            }
            else
            {
                SERVER_LOG_ERROR("USBAsyncRequestManager::openUSBDevice") << "Invalid device handle: " << handle;
            }

            if (!bOpened)
            {
                closeUSBDevice(handle);
            }
        }

        return bOpened;
    }

    void closeUSBDevice(t_usb_device_handle handle)
    {
        LibUSBDeviceState *state = get_libusb_state_from_handle(handle);

        if (state->is_interface_claimed)
        {
            SERVER_LOG_INFO("USBAsyncRequestManager::closeUSBDevice") << "Released USB interface on handle " << handle;
            libusb_release_interface(state->device_handle, 0);
            state->is_interface_claimed= false;
        }

        if (state->device_handle != nullptr)
        {
            SERVER_LOG_INFO("USBAsyncRequestManager::closeUSBDevice") << "Close USB device on handle " << handle;
            libusb_close(state->device_handle);
            state->device_handle= nullptr;
        }
    }

    // -- Device Queries ----
    int getUSBDeviceCount() const
    {
        return static_cast<int>(m_device_state_list.size());
    }

    t_usb_device_handle getFirstUSBDeviceHandle() const
    {
        return (m_device_state_list.size() > 0) ? static_cast<t_usb_device_handle>(0) : k_invalid_usb_device_handle;
    }

    t_usb_device_handle getNextUSBDeviceHandle(t_usb_device_handle handle) const
    {
        int device_index= static_cast<int>(handle);

        return (device_index + 1 < getUSBDeviceCount()) ? static_cast<t_usb_device_handle>(device_index + 1) : k_invalid_usb_device_handle;
    }

    bool getUSBDeviceInfo(t_usb_device_handle handle, USBDeviceFilter &outDeviceInfo) const
    {
        bool bSuccess= false;
        const libusb_device *dev = get_libusb_device_from_handle_const(handle);

        if (dev != nullptr)
        {
            struct libusb_device_descriptor dev_desc;
            libusb_get_device_descriptor(const_cast<libusb_device *>(dev), &dev_desc);

            outDeviceInfo.product_id= dev_desc.idProduct;
            outDeviceInfo.vendor_id= dev_desc.idVendor;
            bSuccess= true;
        }

        return bSuccess;
    }

    bool getUSBDevicePath(t_usb_device_handle handle, char *outBuffer, size_t bufferSize) const
    {
        bool bSuccess = false;
        int device_index = static_cast<int>(handle);        

        if (device_index >= 0 && device_index < getUSBDeviceCount())
        {
            libusb_device *dev = m_device_state_list[device_index].device;

            struct libusb_device_descriptor dev_desc;
            libusb_get_device_descriptor(dev, &dev_desc);

            //###HipsterSloth $TODO Put bus/port numbers here
            int nCharsWritten= 
                ServerUtility::format_string(
                    outBuffer, bufferSize,
                    "USB\\VID_%04X&PID_%04X\\%d",
                    dev_desc.idVendor, dev_desc.idProduct, device_index);

            bSuccess = (nCharsWritten > 0);
        }

        return bSuccess;
    }

    bool getUSBDevicePortPath(t_usb_device_handle handle, char *outBuffer, size_t bufferSize) const
    {
        bool bSuccess = false;
        int device_index = static_cast<int>(handle);        

        if (device_index >= 0 && device_index < getUSBDeviceCount())
        {
            libusb_device *device = m_device_state_list[device_index].device;
            uint8_t port_numbers[MAX_USB_DEVICE_PORT_PATH];

            memset(outBuffer, 0, bufferSize);

            memset(port_numbers, 0, sizeof(port_numbers));
            int port_count = libusb_get_port_numbers(device, port_numbers, MAX_USB_DEVICE_PORT_PATH);
            int bus_id = libusb_get_bus_number(device);

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

    bool getIsUSBDeviceOpen(t_usb_device_handle handle) const
    {
        bool bIsOpen= false;
        const LibUSBDeviceState *state= get_libusb_state_from_handle_const(handle);

        if (state != nullptr)
        {
            bIsOpen= (state->device_handle != nullptr);
        }

        return bIsOpen;
    }

    // -- Request Queue ----
    bool submitTransferRequest(const USBTransferRequest &request, std::function<void(USBTransferResult&)> callback)
    {
        RequestState requestState = {request, callback};
        bool bAddedRequest= false;

        if (request_queue.push(requestState))
        {
            // Give the other thread a chance to process the request
            ServerUtility::sleep_ms(10);
            bAddedRequest= true;
        }

        return bAddedRequest;
    }

protected:
    void startWorkerThread()
    {
        if (!m_thread_started)
        {
            SERVER_LOG_INFO("USBAsyncRequestManager::startup") << "Starting USB event thread";
            m_worker_thread = std::thread(&USBDeviceManagerImpl::workerThreadFunc, this);
            m_thread_started = true;
        }
    }

    bool processRequests()
    {
        bool bHadRequests= false;

        // Process incoming USB transfer requests
        RequestState requestState;
        while (request_queue.pop(requestState))
        {
            switch (requestState.request.request_type)
            {
			case eUSBTransferRequestType::_USBRequestType_InterruptTransfer:
				handleInterruptTransferRequest(requestState);
				break;
            case eUSBTransferRequestType::_USBRequestType_ControlTransfer:
                handleControlTransferRequest(requestState);
                break;
            case eUSBTransferRequestType::_USBRequestType_StartBulkTransfer:
                handleStartBulkTransferRequest(requestState);
                break;
            case eUSBTransferRequestType::_USBRequestType_CancelBulkTransfer:
                handleCancelBulkTransferRequest(requestState);
                break;
            }

            bHadRequests= true;
        }

        if (m_active_bulk_transfer_bundles.size() > 0 || 
            m_canceled_bulk_transfer_bundles.size() > 0 ||
            m_active_control_transfers > 0 ||
			m_active_interrupt_transfers > 0)
        {
            int poll_count = 0;

            // If we have a transfer pending, 
            // keep polling until we get the result back
            while (poll_count == 0 || m_active_control_transfers > 0 || m_active_interrupt_transfers > 0)
            {
                struct timeval tv;
                tv.tv_sec = 0;
                tv.tv_usec = 50 * 1000; // ms

                // Give libusb a change to process transfer requests and post events
                libusb_handle_events_timeout_completed(m_usb_context, &tv, NULL);

                ++poll_count;
            }

            // Cleanup any requests that no longer have any pending cancellations
            cleanupCanceledRequests();
        }

        return bHadRequests;
    }

    void processResults()
    {
        ResultState resultState;

        // Process all pending results
        while (result_queue.pop(resultState))
        {
            // Fire the callback on the result
            resultState.callback(resultState.result);
        }
    }

    void requestProcessingTeardown()
    {
        // Drain the request queue
        while (request_queue.pop());

        // Cancel all active transfers
        while (m_active_bulk_transfer_bundles.size() > 0)
        {
            USBBulkTransferBundle *bundle= m_active_bulk_transfer_bundles.back();
            m_active_bulk_transfer_bundles.pop_back();
            bundle->cancelTransfers();
            m_canceled_bulk_transfer_bundles.push_back(bundle);
        }

        // Wait for the canceled bulk transfers and control transfers to exit
        while (m_canceled_bulk_transfer_bundles.size() > 0 || m_active_control_transfers > 0 || m_active_interrupt_transfers > 0)
        {
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 50 * 1000; // ms

            // Give libusb a change to process the cancellation requests
            libusb_handle_events_timeout_completed(m_usb_context, &tv, NULL);

            // Cleanup any requests that no longer have any pending cancellations
            cleanupCanceledRequests();
        }
    }

    void workerThreadFunc()
    {
        ServerUtility::set_current_thread_name("USB Async Worker Thread");

        // Stay in the message loop until asked to exit by the main thread
        while (!m_exit_signaled)
        {
            processRequests();

            // Shut the thread down if we aren't managing any bulk transfers
            if (m_active_bulk_transfer_bundles.size() == 0 &&
                m_canceled_bulk_transfer_bundles.size() == 0)
            {
                m_exit_signaled= true;
            }
        }
    }

    void cleanupCanceledRequests()
    {
        for (auto it = m_canceled_bulk_transfer_bundles.begin(); it != m_canceled_bulk_transfer_bundles.end(); ++it)
        {
            USBBulkTransferBundle *bundle = *it;

            //###HipsterSloth $TODO Timeout the cancellation?
            if (bundle->getActiveTransferCount() == 0)
            {
                m_canceled_bulk_transfer_bundles.erase(it);
                delete bundle;
            }
        }
    }

	void handleInterruptTransferRequest(const RequestState &requestState)
	{
		const USBRequestPayload_InterruptTransfer &request = requestState.request.payload.interrupt_transfer;

		LibUSBDeviceState *state = get_libusb_state_from_handle(request.usb_device_handle);
		eUSBResultCode result_code;
		bool bSuccess = true;

#if defined(DEBUG_USBDEVICEMANAGER)
		if ((request.endpoint & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_OUT)
		{
			debug("USBMgr REQUEST: interrupt transfer write - dev: %d, endpoint: 0x%X, datalen: %d\n",
				requestState.request.payload.interrupt_transfer.usb_device_handle,
				requestState.request.payload.interrupt_transfer.endpoint,
				requestState.request.payload.interrupt_transfer.length);
		}
		else
		{
			debug("USBMgr REQUEST: control transfer read - dev: %d, endpoint: 0x%X, datalen: %d\n",
				requestState.request.payload.control_transfer.usb_device_handle,
				requestState.request.payload.interrupt_transfer.endpoint,
				requestState.request.payload.interrupt_transfer.length);
		}
#endif

		if (state != nullptr)
		{
			RequestState *requestStateOnHeap = nullptr;
			struct libusb_transfer *transfer;
			unsigned char *buffer;

			if (bSuccess)
			{
				transfer = libusb_alloc_transfer(0);
				if (transfer == nullptr)
				{
					result_code = _USBResultCode_NoMemory;
					bSuccess = false;
				}
			}

			if (bSuccess)
			{
				requestStateOnHeap = new RequestState;
				if (requestStateOnHeap == nullptr)
				{
					result_code = _USBResultCode_NoMemory;
					bSuccess = false;
				}
			}

			if (bSuccess)
			{
				int libusb_result = LIBUSB_SUCCESS;

				// Make a copy of the request on the heap so that it's safe
				// to point to in the transfer userdata
				requestStateOnHeap->request = requestState.request;
				requestStateOnHeap->callback = requestState.callback;

				libusb_fill_interrupt_transfer(
					transfer,
					state->device_handle,
					requestStateOnHeap->request.payload.interrupt_transfer.endpoint,
					requestStateOnHeap->request.payload.interrupt_transfer.data,
					requestStateOnHeap->request.payload.interrupt_transfer.length,
					interrupt_transfer_cb,
					requestStateOnHeap,
					request.timeout);

				libusb_result = libusb_submit_transfer(transfer);
				if (libusb_result == LIBUSB_SUCCESS)
				{
					// One more active interrupt transfer
					++m_active_interrupt_transfers;

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
					libusb_free_transfer(transfer);
				}

				if (buffer != nullptr)
				{
					free(buffer);
				}

				if (requestStateOnHeap != nullptr)
				{
					delete requestStateOnHeap;
				}
			}
		}
		else
		{
			result_code = _USBResultCode_BadHandle;
			bSuccess = false;
		}

		// If the control transfer didn't successfully start, post a failure result now
		if (!bSuccess)
		{
			USBTransferResult result;

			memset(&result, 0, sizeof(USBTransferResult));
			result.payload.interrupt_transfer.usb_device_handle = request.usb_device_handle;
			result.payload.interrupt_transfer.result_code = result_code;
			result.result_type = _USBResultType_InterrupTransfer;

			postUSBTransferResult(result, requestState.callback);
		}
	}

    void handleControlTransferRequest(const RequestState &requestState)
    {
        const USBRequestPayload_ControlTransfer &request = requestState.request.payload.control_transfer;

        LibUSBDeviceState *state = get_libusb_state_from_handle(request.usb_device_handle);
        eUSBResultCode result_code;
        bool bSuccess= true;

#if defined(DEBUG_USBDEVICEMANAGER)
        if ((request.bmRequestType & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_OUT)
        {
            debug("USBMgr REQUEST: control transfer write - dev: %d, reg: 0x%X, value: 0x%x\n", 
                requestState.request.payload.control_transfer.usb_device_handle,
                requestState.request.payload.control_transfer.wIndex,
                requestState.request.payload.control_transfer.data[0]);
        }
        else
        {
            debug("USBMgr REQUEST: control transfer read - dev: %d, reg: 0x%X\n", 
                requestState.request.payload.control_transfer.usb_device_handle,
                requestState.request.payload.control_transfer.wIndex);
        }
#endif

        if (state != nullptr)
        {
            RequestState *requestStateOnHeap= nullptr;
            struct libusb_transfer *transfer;
            unsigned char *buffer;

            if (bSuccess)
            {
                transfer = libusb_alloc_transfer(0);
                if (transfer == nullptr)
                {
                    result_code = _USBResultCode_NoMemory;
                    bSuccess= false;
                }
            }

            if (bSuccess)
            {
                buffer = (unsigned char*)malloc(LIBUSB_CONTROL_SETUP_SIZE + request.wLength);
                if (buffer == nullptr)
                {
                    result_code = _USBResultCode_NoMemory;
                    bSuccess= false;
                }
            }

            if (bSuccess)
            {
                requestStateOnHeap= new RequestState;
                if (requestStateOnHeap == nullptr)
                {
                    result_code = _USBResultCode_NoMemory;
                    bSuccess= false;
                }
            }

            if (bSuccess)
            {
                int libusb_result= LIBUSB_SUCCESS;

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

                // Make a copy of the request on the heap so that it's safe
                // to point to in the transfer userdata
                requestStateOnHeap->request= requestState.request;
                requestStateOnHeap->callback= requestState.callback;

                libusb_fill_control_transfer(
                    transfer,
                    state->device_handle,
                    buffer,
                    control_transfer_cb,
                    requestStateOnHeap,
                    request.timeout);
                transfer->flags = LIBUSB_TRANSFER_FREE_BUFFER;

                libusb_result = libusb_submit_transfer(transfer);
                if (libusb_result == LIBUSB_SUCCESS)
                {
                    // One more active control transfer
                    ++m_active_control_transfers;

                    result_code = _USBResultCode_Started;
                }
                else
                {
                    result_code = _USBResultCode_SubmitFailed;
                    bSuccess= false;
                }
            }

            if (!bSuccess)
            {
                if (transfer != nullptr)
                {
                    libusb_free_transfer(transfer);
                }

                if (buffer != nullptr)
                {
                    free(buffer);
                }

                if (requestStateOnHeap != nullptr)
                {
                    delete requestStateOnHeap;
                }
            }
        }
        else
        {
            result_code = _USBResultCode_BadHandle;
            bSuccess= false;
        }

        // If the control transfer didn't successfully start, post a failure result now
        if (!bSuccess)
        {
            USBTransferResult result;

            memset(&result, 0, sizeof(USBTransferResult));
            result.payload.control_transfer.usb_device_handle= request.usb_device_handle;
            result.payload.control_transfer.result_code= result_code;
            result.result_type = _USBResultType_ControlTransfer;

            postUSBTransferResult(result, requestState.callback);
        }
    }

	static void LIBUSB_CALL interrupt_transfer_cb(struct libusb_transfer *transfer)
	{
		const RequestState *requestStateOnHeap = reinterpret_cast<const RequestState *>(transfer->user_data);
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

#if defined(DEBUG_USBDEVICEMANAGER)
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
		USBDeviceManager::getInstance()->getImplementation()->postUSBTransferResult(result, requestStateOnHeap->callback);

		// Free request state stored in the heap now that the result is posted
		delete requestStateOnHeap;

		// Free the libusb allocated transfer
		libusb_free_transfer(transfer);
	}

    static void LIBUSB_CALL control_transfer_cb(struct libusb_transfer *transfer)
    {
        const RequestState *requestStateOnHeap = reinterpret_cast<const RequestState *>(transfer->user_data);
        const USBRequestPayload_ControlTransfer *request= &requestStateOnHeap->request.payload.control_transfer;

        USBTransferResult result;

        memset(&result, 0, sizeof(USBTransferResult));
        result.result_type= _USBResultType_ControlTransfer;
        result.payload.control_transfer.usb_device_handle= request->usb_device_handle;
                
        if ((request->bmRequestType & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN && 
            transfer->actual_length > 0)
        {
            memcpy(&result.payload.control_transfer.data, libusb_control_transfer_get_data(transfer), transfer->actual_length);
        }        
		result.payload.control_transfer.dataLength = transfer->actual_length;

        switch (transfer->status) 
        {
        case LIBUSB_TRANSFER_COMPLETED:
            result.payload.control_transfer.result_code= _USBResultCode_Completed;
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

#if defined(DEBUG_USBDEVICEMANAGER)
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
        USBDeviceManager::getInstance()->getImplementation()->postUSBTransferResult(result, requestStateOnHeap->callback);

        // Free request state stored in the heap now that the result is posted
        delete requestStateOnHeap;

        // Free the libusb allocated transfer
        libusb_free_transfer(transfer);
    }

    void postUSBTransferResult(const USBTransferResult &result, std::function<void(USBTransferResult&)> callback)
    {
        ResultState state = {result, callback};

        // If a control transfer just completed (successfully or unsuccessfully)
        // decrement the outstanding control transfer count
        if (result.result_type == _USBResultType_ControlTransfer)
        {
            assert(m_active_control_transfers > 0);
            --m_active_control_transfers;
        }
		// If a interrupt transfer just completed (successfully or unsuccessfully)
		// decrement the outstanding interrupt transfer count
		else if (result.result_type == _USBResultType_InterrupTransfer)
		{
			assert(m_active_interrupt_transfers > 0);
			--m_active_interrupt_transfers;
		}

        result_queue.push(state);
    }

    void handleStartBulkTransferRequest(const RequestState &requestState)
    {
        const USBRequestPayload_BulkTransfer &request= requestState.request.payload.start_bulk_transfer;

        LibUSBDeviceState *state = get_libusb_state_from_handle(request.usb_device_handle);
        eUSBResultCode result_code;

        if (state != nullptr)
        {
            if (state->device_handle != nullptr)
            {
                // Only start a bulk transfer if the device doesn't have one going already
                auto it = std::find_if(
                    m_active_bulk_transfer_bundles.begin(),
                    m_active_bulk_transfer_bundles.end(),
                    [&request](const USBBulkTransferBundle *bundle) {
                        return bundle->getUSBDeviceHandle() == request.usb_device_handle;
                });

                if (it == m_active_bulk_transfer_bundles.end())
                {
                    USBBulkTransferBundle *bundle = 
                        new USBBulkTransferBundle(request, state->device, state->device_handle);

                    // Allocate and initialize the bulk transfers
                    if (bundle->initialize())
                    {
                        // Attempt to start all the transfers
                        if (bundle->startTransfers())
                        {
                            // Success! Add the bundle to the list of active bundles
                            m_active_bulk_transfer_bundles.push_back(bundle);
                            result_code = _USBResultCode_Started;
                        }
                        else
                        {                            
                            // Unable to start all of the transfers in the bundle
                            if (bundle->getActiveTransferCount() > 0)
                            {
                                // If any transfers started we have to cancel the ones that started
                                // and wait for the cancellation request to complete.
                                bundle->cancelTransfers();
                                m_canceled_bulk_transfer_bundles.push_back(bundle);
                            }
                            else
                            {
                                // No transfer requests started.
                                // Delete the bundle right away.
                                delete bundle;
                            }

                            result_code = _USBResultCode_SubmitFailed;
                        }
                    }
                    else
                    {
                        result_code = _USBResultCode_NoMemory;
                        delete bundle;
                    }
                }
                else
                {
                    result_code = _USBResultCode_TransferAlreadyStarted;
                }
            }
            else
            {
                result_code = _USBResultCode_DeviceNotOpen;
            }
        }
        else
        {
            result_code = _USBResultCode_BadHandle;
        }

        // Post the transfer result to the outbound result queue
        {
            USBTransferResult result;

            result.result_type = _USBResultType_BulkTransfer;
            result.payload.bulk_transfer.usb_device_handle= request.usb_device_handle;
            result.payload.bulk_transfer.result_code = result_code;

            postUSBTransferResult(result, requestState.callback);
        }
    }

    void handleCancelBulkTransferRequest(const RequestState &requestState)
    {
        const USBRequestPayload_CancelBulkTransfer &request= requestState.request.payload.cancel_bulk_transfer;

        libusb_device *dev = get_libusb_device_from_handle(request.usb_device_handle);
        eUSBResultCode result_code;

        if (dev != nullptr)
        {
            auto it = std::find_if(
                m_active_bulk_transfer_bundles.begin(),
                m_active_bulk_transfer_bundles.end(),
                [&request](const USBBulkTransferBundle *bundle) {
                    return bundle->getUSBDeviceHandle() == request.usb_device_handle;
                });

            if (it != m_active_bulk_transfer_bundles.end())
            {
                USBBulkTransferBundle *bundle = *it;

                // Tell the bundle to cancel all active transfers.
                // This is an asynchronous operation.
                bundle->cancelTransfers();

                // Remove the bundle from the list of active transfers
                m_active_bulk_transfer_bundles.erase(it);

                // Put the bundle on the list of canceled transfers.
                // The bundle will get cleaned up once all active transfers are done.
                m_canceled_bulk_transfer_bundles.push_back(bundle);

                result_code = _USBResultCode_Canceled;
            }
            else
            {
                result_code = _USBResultCode_TransferNotActive;
            }
        }
        else
        {
            result_code= _USBResultCode_BadHandle;
        }

        // Post the transfer result to the outbound result queue
        {
            USBTransferResult result;

            result.result_type = _USBResultType_BulkTransfer;
            result.payload.bulk_transfer.usb_device_handle = request.usb_device_handle;
            result.payload.bulk_transfer.result_code = result_code;

            postUSBTransferResult(result, requestState.callback);
        }
    }

    void stopWorkerThread()
    {
        if (m_thread_started)
        {
            if (!m_exit_signaled)
            {
                SERVER_LOG_INFO("USBAsyncRequestManager::startup") << "Stopping USB event thread...";
                m_exit_signaled = true;
                m_worker_thread.join();
                SERVER_LOG_INFO("USBAsyncRequestManager::startup") << "USB event thread stopped";
            }
            else
            {
                SERVER_LOG_INFO("USBAsyncRequestManager::startup") << "USB event thread already stopped";
            }

            m_thread_started = false;
            m_exit_signaled = false;
        }
    }

    inline const LibUSBDeviceState *get_libusb_state_from_handle_const(t_usb_device_handle handle) const
    {
        const LibUSBDeviceState *state = nullptr;
        int device_index = static_cast<int>(handle);

        if (device_index >= 0 && device_index < getUSBDeviceCount())
        {
            state = &m_device_state_list[device_index];
        }

        return state;
    }

    inline LibUSBDeviceState *get_libusb_state_from_handle(t_usb_device_handle handle)
    {
        return const_cast<LibUSBDeviceState *>(get_libusb_state_from_handle_const(handle));
    }

    inline const libusb_device *get_libusb_device_from_handle_const(t_usb_device_handle handle) const
    {
        const LibUSBDeviceState *state= get_libusb_state_from_handle_const(handle);
        const libusb_device *device= nullptr;

        if (state != nullptr)
        {
            device = state->device;
        }

        return device;
    }

    inline libusb_device *get_libusb_device_from_handle(t_usb_device_handle handle)
    {
        return const_cast<libusb_device *>(get_libusb_device_from_handle_const(handle));
    }

    void rebuildFilteredDeviceList()
    {
        libusb_device **device_list;
        if (libusb_get_device_list(m_usb_context, &device_list) < 0)
        {
            SERVER_LOG_INFO("USBAsyncRequestManager::rebuildFilteredDeviceList") << "Unable to fetch device list.";
        }

        unsigned char dev_port_numbers[MAX_USB_DEVICE_PORT_PATH] = { 0 };
        for (int i= 0; device_list[i] != NULL; ++i)
        {
            libusb_device *dev= device_list[i];

            if (isDeviceInWhitelist(dev))
            {
                uint8_t port_numbers[MAX_USB_DEVICE_PORT_PATH];
                memset(port_numbers, 0, sizeof(port_numbers));
                int elements_filled = libusb_get_port_numbers(dev, port_numbers, MAX_USB_DEVICE_PORT_PATH);

                if (elements_filled > 0)
                {
                    // Make sure this device is actually different from the last device we looked at
                    // (i.e. has a different device port path)
                    if (memcmp(port_numbers, dev_port_numbers, sizeof(port_numbers)) != 0)
                    {
                        libusb_device_handle *devhandle;
                        int libusb_result = libusb_open(dev, &devhandle);

                        if (libusb_result == LIBUSB_SUCCESS || libusb_result == LIBUSB_ERROR_ACCESS)
                        {
                            if (libusb_result == LIBUSB_SUCCESS)
                            {
                                libusb_close(devhandle);

                                // Add a device state entry to the list
                                {
                                    LibUSBDeviceState device_state;

                                    // The "handle" is really just an index into the device state list
                                    device_state.handle= static_cast<t_usb_device_handle>(m_device_state_list.size());
                                    device_state.device = dev;
                                    device_state.device_handle = nullptr;
                                    device_state.is_interface_claimed = false;

                                    m_device_state_list.push_back(device_state);
                                }

                                libusb_ref_device(dev);
                            }

                            // Cache the port number for the last valid device found
                            memcpy(dev_port_numbers, port_numbers, sizeof(port_numbers));
                        }
                    }
                }
            }
        }

        libusb_free_device_list(device_list, 1);
    }

    void freeDeviceStateList()
    {
        for (auto it = m_device_state_list.begin(); it != m_device_state_list.end(); ++it)
        {
            const LibUSBDeviceState &device_state= *it;

            closeUSBDevice(device_state.handle);
            libusb_unref_device(device_state.device);
        }
        m_device_state_list.clear();
    }

    bool isDeviceInWhitelist(libusb_device *dev)
    {
        libusb_device_descriptor desc;
        libusb_get_device_descriptor(dev, &desc);

        auto iter= std::find_if(
            m_device_whitelist.begin(), 
            m_device_whitelist.end(), 
            [&desc](const USBDeviceFilter &entry)->bool 
            {
                return desc.idVendor == entry.vendor_id && desc.idProduct == entry.product_id;
            });
        
        return iter != m_device_whitelist.end();
    }

private:
    // Multithreaded state
    libusb_context* m_usb_context;
    bool m_bUseMultithreading;
    std::atomic_bool m_exit_signaled;
    boost::lockfree::spsc_queue<RequestState, boost::lockfree::capacity<128> > request_queue;
    boost::lockfree::spsc_queue<ResultState, boost::lockfree::capacity<128> > result_queue;

    // Worker thread state
    std::vector<USBBulkTransferBundle *> m_active_bulk_transfer_bundles;
    std::vector<USBBulkTransferBundle *> m_canceled_bulk_transfer_bundles;
    int m_active_control_transfers;
	int m_active_interrupt_transfers;

    // Main thread state
    bool m_thread_started;
    std::thread m_worker_thread;
    std::vector<USBDeviceFilter> m_device_whitelist;
    std::vector<LibUSBDeviceState> m_device_state_list;
};

//-- public interface -----
USBDeviceManager *USBDeviceManager::m_instance = NULL;

USBDeviceManager::USBDeviceManager(struct USBDeviceFilter *device_whitelist, size_t device_whitelist_length)
    : m_implementation_ptr(new USBDeviceManagerImpl(device_whitelist, device_whitelist_length))
{
}

USBDeviceManager::~USBDeviceManager()
{
    if (m_instance != NULL)
    {
        SERVER_LOG_ERROR("~USBAsyncRequestManager()") << "USB Async Request Manager deleted without shutdown() getting called first";
    }

    if (m_implementation_ptr != nullptr)
    {
        delete m_implementation_ptr;
        m_implementation_ptr = nullptr;
    }
}

bool USBDeviceManager::startup()
{
    m_instance = this;
    return m_implementation_ptr->startup();
}

void USBDeviceManager::update()
{
    m_implementation_ptr->update();
}

void USBDeviceManager::shutdown()
{
    m_implementation_ptr->shutdown();
    m_instance = NULL;
}

bool USBDeviceManager::openUSBDevice(t_usb_device_handle handle)
{
    return m_implementation_ptr->openUSBDevice(handle);
}

void USBDeviceManager::closeUSBDevice(t_usb_device_handle handle)
{
    m_implementation_ptr->closeUSBDevice(handle);
}

int USBDeviceManager::getUSBDeviceCount() const
{
    return m_implementation_ptr->getUSBDeviceCount();
}

const char *USBDeviceManager::getErrorString(eUSBResultCode result_code)
{
	const char *result = "UNKNOWN USB ERROR";

	switch (result_code)
	{
	case _USBResultCode_Started:
		result = "Transfer Started";
		break;
	case _USBResultCode_Canceled:
		result = "Transfer Cancelled";
		break;
	case _USBResultCode_Completed:
		result = "Transfer Completed";
		break;
	case _USBResultCode_GeneralError:
		result = "General USB Error";
		break;
	case _USBResultCode_BadHandle:
		result = "Bad USB handle";
		break;
	case _USBResultCode_NoMemory:
		result = "Out of Memory";
		break;
	case _USBResultCode_SubmitFailed:
		result = "Transfer Submit Failed";
		break;
	case _USBResultCode_DeviceNotOpen:
		result = "USB Device Not Open";
		break;
	case _USBResultCode_TransferNotActive:
		result = "Transfer Not Active";
		break;
	case _USBResultCode_TransferAlreadyStarted:
		result = "Transfer Already Active";
		break;
	case _USBResultCode_Overflow:
		result = "Overflow Error";
		break;
	case _USBResultCode_Pipe:
		result = "Pipe Error";
		break;
	case _USBResultCode_TimedOut:
		break;
	}

	return result;
}

t_usb_device_handle USBDeviceManager::getFirstUSBDeviceHandle() const
{
    return m_implementation_ptr->getFirstUSBDeviceHandle();
}

t_usb_device_handle USBDeviceManager::getNextUSBDeviceHandle(t_usb_device_handle handle) const
{
    return m_implementation_ptr->getNextUSBDeviceHandle(handle);
}

bool USBDeviceManager::getUSBDeviceInfo(t_usb_device_handle handle, USBDeviceFilter &outDeviceInfo) const
{
    return m_implementation_ptr->getUSBDeviceInfo(handle, outDeviceInfo);
}

bool USBDeviceManager::getUSBDevicePath(t_usb_device_handle handle, char *outBuffer, size_t bufferSize) const
{
    return m_implementation_ptr->getUSBDevicePath(handle, outBuffer, bufferSize);
}

bool USBDeviceManager::getUSBDevicePortPath(t_usb_device_handle handle, char *outBuffer, size_t bufferSize) const
{
    return m_implementation_ptr->getUSBDevicePortPath(handle, outBuffer, bufferSize);
}

bool USBDeviceManager::getIsUSBDeviceOpen(t_usb_device_handle handle) const
{
    return m_implementation_ptr->getIsUSBDeviceOpen(handle);
}

bool USBDeviceManager::submitTransferRequestAsync(
    const USBTransferRequest &request,
    std::function<void(USBTransferResult&)> callback)
{
    return m_implementation_ptr->submitTransferRequest(request, callback);
}

USBTransferResult USBDeviceManager::submitTransferRequestBlocking(
	const USBTransferRequest &request)
{
	USBTransferResult result;
	bool bIsPending = true;

	// Submit the async usb control transfer request to the worker thread
	m_implementation_ptr->submitTransferRequest(
		request,
		[&result, &bIsPending](USBTransferResult &r)
		{
			result = r;
			bIsPending = false;
		}
	);

	// Spin until the transfer completes
	while (bIsPending)
	{
		// Give the worker thread a chance to do work
		ServerUtility::sleep_ms(1);

		// Poll to see if the transfer completed
		// (will execute the callback on completion)
		m_implementation_ptr->update();
	}

	return result;
}
//-- includes -----
#include "USBBulkTransferBundle.h"
#include "ServerLog.h"
#include "ServerUtility.h"

#include "libusb.h"

#include <assert.h>
#include <memory>

//-- private methods -----
static void LIBUSB_CALL transfer_callback_function(struct libusb_transfer *bulk_transfer);

//-- implementation -----
USBBulkTransferBundle::USBBulkTransferBundle(
    const USBRequestPayload_BulkTransfer &request,
    libusb_device *dev,
    libusb_device_handle *dev_handle)
    : m_request(request)
    , m_device(dev)
    , m_device_handle(dev_handle)
    , m_active_transfer_count(0)
    , m_is_canceled(false)
    , bulk_transfer_requests(nullptr)
    , transfer_buffer(nullptr)
{
}

USBBulkTransferBundle::~USBBulkTransferBundle()
{
    dispose();

    if (m_active_transfer_count > 0)
    {
        SERVER_MT_LOG_INFO("USBBulkTransferBundle::destructor") << "active transfer count non-zero!";
    }
}

bool USBBulkTransferBundle::initialize()
{
    bool bSuccess = (m_active_transfer_count == 0);
    uint8_t bulk_endpoint = 0;

    // Find the bulk transfer endpoint          
    if (bSuccess)
    {
        if (find_bulk_transfer_endpoint(m_device, bulk_endpoint))
        {
            libusb_clear_halt(m_device_handle, bulk_endpoint);
        }
        else
        {
            bSuccess = false;
        }
    }

    // Allocate the libusb transfer request array
    if (bSuccess)
    {
        size_t xfer_array_byte_size = m_request.in_flight_transfer_packet_count * sizeof(libusb_transfer *);
        bulk_transfer_requests = (libusb_transfer **)malloc(xfer_array_byte_size);

        if (bulk_transfer_requests != nullptr)
        {
            memset(bulk_transfer_requests, 0, xfer_array_byte_size);
        }
        else
        {
            bSuccess = false;
        }
    }

    // Allocate the transfer buffer
    if (bSuccess)
    {
        // Allocate the transfer buffer that the requests write data into
        size_t xfer_buffer_size = m_request.in_flight_transfer_packet_count * m_request.transfer_packet_size;
        transfer_buffer = (uint8_t *)malloc(xfer_buffer_size);

        if (transfer_buffer != nullptr)
        {
            memset(transfer_buffer, 0, xfer_buffer_size);
        }
        else
        {
            bSuccess = false;
        }
    }

    // Allocate and initialize the transfers
    if (bSuccess)
    {
        for (int transfer_index = 0; transfer_index < m_request.in_flight_transfer_packet_count; ++transfer_index)
        {
            bulk_transfer_requests[transfer_index] = libusb_alloc_transfer(0);

            if (bulk_transfer_requests[transfer_index] != nullptr)
            {
                libusb_fill_bulk_transfer(
                    bulk_transfer_requests[transfer_index],
                    m_device_handle,
                    bulk_endpoint,
                    transfer_buffer + transfer_index*m_request.transfer_packet_size,
                    m_request.transfer_packet_size,
                    transfer_callback_function,
                    reinterpret_cast<void*>(this),
                    0);
            }
            else
            {
                bSuccess = false;
            }
        }
    }

    return bSuccess;
}

void USBBulkTransferBundle::dispose()
{
    assert(m_active_transfer_count == 0);

    for (int transfer_index = 0;
        transfer_index < m_request.in_flight_transfer_packet_count;
        ++transfer_index)
    {
        if (bulk_transfer_requests[transfer_index] != nullptr)
        {
            libusb_free_transfer(bulk_transfer_requests[transfer_index]);
        }
    }

    if (transfer_buffer != nullptr)
    {
        free(transfer_buffer);
        transfer_buffer = nullptr;
    }

    if (bulk_transfer_requests != nullptr)
    {
        free(bulk_transfer_requests);
        bulk_transfer_requests = nullptr;
    }
}

bool USBBulkTransferBundle::startTransfers()
{
    bool bSuccess = (m_active_transfer_count == 0 && !m_is_canceled);

    // Start the transfers
    if (bSuccess)
    {
        for (int transfer_index = 0; transfer_index < m_request.in_flight_transfer_packet_count; ++transfer_index)
        {
            libusb_transfer *bulk_transfer = bulk_transfer_requests[transfer_index];

            if (libusb_submit_transfer(bulk_transfer) == 0)
            {
                ++m_active_transfer_count;
            }
            else
            {
                bSuccess = false;
                break;
            }
        }
    }

    return bSuccess;
}

void USBBulkTransferBundle::notifyActiveTransfersDecremented()
{
    assert(m_active_transfer_count > 0);
    --m_active_transfer_count;
}

static void LIBUSB_CALL transfer_callback_function(struct libusb_transfer *bulk_transfer)
{
    USBBulkTransferBundle *bundle = reinterpret_cast<USBBulkTransferBundle*>(bulk_transfer->user_data);
    const auto &request = bundle->getTransferRequest();
    enum libusb_transfer_status status = bulk_transfer->status;

    if (status == LIBUSB_TRANSFER_COMPLETED)
    {

        // NOTE: This callback is getting executed on the worker thread!
        // It should not:
        // 1) Do any expensive work
        // 2) Call any blocking functions
        // 3) Access data on the main thread, unless it can do so in an atomic way
        request.on_data_callback(
            bulk_transfer->buffer,
            bulk_transfer->actual_length,
            request.transfer_callback_userdata);
    }

    // See if the request wants to resubmitted the moment it completes.
    // If the transfer was canceled, this overrides the auto-resubmit.
    bool bRestartedTransfer = false;
    if (status != LIBUSB_TRANSFER_CANCELLED && request.bAutoResubmit)
    {
        // Start the transfer over with the same properties
        if (libusb_submit_transfer(bulk_transfer) == 0)
        {
            bRestartedTransfer = true;
        }
    }

    // If the transfer didn't restart update the active transfer count
    if (!bRestartedTransfer)
    {
        bundle->notifyActiveTransfersDecremented();
    }
}

void USBBulkTransferBundle::cancelTransfers()
{
    assert(bulk_transfer_requests != nullptr);

    if (!m_is_canceled)
    {
        for (int transfer_index = 0;
            transfer_index < m_request.in_flight_transfer_packet_count;
            ++transfer_index)
        {
            libusb_transfer* bulk_transfer = bulk_transfer_requests[transfer_index];

            assert(bulk_transfer != nullptr);
            libusb_cancel_transfer(bulk_transfer);
        }

        m_is_canceled = true;
    }
}

// Search for an input transfer endpoint in the endpoint descriptor
// of the device interfaces alt_settings
bool USBBulkTransferBundle::find_bulk_transfer_endpoint(struct libusb_device *device, uint8_t &out_endpoint_addr)
{
    bool bSuccess = false;
    libusb_config_descriptor *config = nullptr;

    libusb_get_active_config_descriptor(device, &config);

    if (config != nullptr)
    {
        const libusb_interface_descriptor *altsetting = nullptr;

        for (int i = 0; i < config->bNumInterfaces; i++)
        {
            const libusb_interface_descriptor *test_altsetting = config->interface[i].altsetting;

            if (test_altsetting[0].bInterfaceNumber == 0)
            {
                altsetting = test_altsetting;
                break;
            }
        }

        if (altsetting != nullptr)
        {
            for (int i = 0; i < altsetting->bNumEndpoints; i++)
            {
                const libusb_endpoint_descriptor *endpoint_desc = &altsetting->endpoint[i];

                if ((endpoint_desc->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_BULK
                    && endpoint_desc->wMaxPacketSize != 0)
                {
                    out_endpoint_addr = endpoint_desc->bEndpointAddress;
                    bSuccess = true;
                    break;
                }
            }
        }

        libusb_free_config_descriptor(config);
    }

    return bSuccess;
}
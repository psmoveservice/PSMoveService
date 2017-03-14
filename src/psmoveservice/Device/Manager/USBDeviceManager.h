#ifndef USB_DEVICE_MANAGER_H
#define USB_DEVICE_MANAGER_H

//-- includes -----
#include "PSMoveConfig.h"
#include "USBDeviceRequest.h"
#include <functional>

//-- constants -----
enum eUSBApiType
{
	_USBApiType_INVALID= -1,

	_USBApiType_NullUSB,
	_USBApiType_LibUSB,
	_USBApiType_WinUSB,
};

//-- definitions -----
class USBManagerConfig : public PSMoveConfig
{
public:
    static const int CONFIG_VERSION;

    USBManagerConfig(const std::string &fnamebase = "USBManagerConfig");

    virtual const boost::property_tree::ptree config2ptree();
    virtual void ptree2config(const boost::property_tree::ptree &pt);

    long version;
	std::string usb_api_name;
	bool enable_usb_transfers;
};

/// Manages async control and bulk transfer requests to usb devices via selected usb api.
class USBDeviceManager
{
public:
    USBDeviceManager();
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

private:
	/// Configuration settings used by the USB manager
	USBManagerConfig m_cfg;

    /// private implementation
    class USBDeviceManagerImpl *m_implementation_ptr;

    /// Singleton instance of the class
    /// Assigned in startup, cleared in teardown
    static USBDeviceManager *m_instance;
};

// -- Device Enumeration ----
struct USBDeviceEnumerator* usb_device_enumerator_allocate();
bool usb_device_enumerator_is_valid(struct USBDeviceEnumerator* enumerator);
bool usb_device_enumerator_get_filter(struct USBDeviceEnumerator* enumerator, USBDeviceFilter &outDeviceInfo);
void usb_device_enumerator_next(struct USBDeviceEnumerator* enumerator);
void usb_device_enumerator_free(struct USBDeviceEnumerator* enumerator);
bool usb_device_enumerator_get_path(struct USBDeviceEnumerator* enumerator, char *outBuffer, size_t bufferSize);

// -- Device Actions ----
t_usb_device_handle usb_device_open(struct USBDeviceEnumerator* enumerator);
void usb_device_close(t_usb_device_handle usb_device_handle);

// Send the transfer request to the worker thread asynchronously
bool usb_device_submit_transfer_request_async(
	const USBTransferRequest &request,
	std::function<void(USBTransferResult&)> callback = [](USBTransferResult &result) {});

// Send the transfer request to the worker thread and block until it completes
USBTransferResult usb_device_submit_transfer_request_blocking(const USBTransferRequest &request);

// -- Device Queries ----
bool usb_device_can_be_opened(struct USBDeviceEnumerator* enumerator, char *outReason, size_t bufferSize);
bool usb_device_get_filter(t_usb_device_handle handle, USBDeviceFilter &outDeviceInfo);
bool usb_device_get_full_path(t_usb_device_handle handle, char *outBuffer, size_t bufferSize);
bool usb_device_get_port_path(t_usb_device_handle handle, char *outBuffer, size_t bufferSize);
bool usb_device_get_is_open(t_usb_device_handle handle);
const char *usb_device_get_error_string(eUSBResultCode result_code);

// -- Notifications ----
void usb_device_post_transfer_result(const USBTransferResult &result, std::function<void(USBTransferResult&)> callback);

#endif  // USB_DEVICE_MANAGER_H
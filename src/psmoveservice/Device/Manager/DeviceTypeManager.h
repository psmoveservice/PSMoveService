#ifndef DEVICE_TYPE_MANAGER_H
#define DEVICE_TYPE_MANAGER_H

//-- includes -----
#include <memory>
#include <chrono>
#include "PSMoveProtocol.pb.h"

//-- typedefs -----
class ServerDeviceView;
typedef std::shared_ptr<ServerDeviceView> ServerDeviceViewPtr;

//-- definitions -----
/// ABC for device managers for controllers, trackers, hmds.
class DeviceTypeManager
{
public:
    DeviceTypeManager(const int recon_int = 1000, const int poll_int = 2);
    virtual ~DeviceTypeManager();

    virtual bool startup();
    virtual void shutdown();

    void poll();
    void publish();

    virtual int getMaxDevices() const = 0;

    /**
    Returns an upcast device view ptr. Useful for generic functions that are
    simple wrappers around the device functions:
    open(), getIsOpen(), update(), close(), getIsReadyToPoll()
    For anything that requires knowledge of the device, use the class-specific
    Server<Type>ViewPtr get<Type>ViewPtr(int device_id) functions instead.
    */
    ServerDeviceViewPtr getDeviceViewPtr(int device_id);

    int reconnect_interval;
    int poll_interval;

protected:
    void poll_devices();

    /** This method tries make the list of open devices in m_devices match
    the list of connected devices in the device enumerator.
    No device objects are created or destroyed.
    Pointers are just shuffled around and devices opened and closed.
    */
    bool update_connected_devices();

    virtual bool can_poll_connected_devices();
    virtual bool can_update_connected_devices();
    virtual class DeviceEnumerator *allocate_device_enumerator() = 0;
    virtual void free_device_enumerator(class DeviceEnumerator *) = 0;
    virtual ServerDeviceView *allocate_device_view(int device_id) = 0;

    void send_device_list_changed_notification();

    virtual const PSMoveProtocol::Response_ResponseType getListUpdatedResponseType() = 0;

    int find_first_closed_device_device_id();
    int find_open_device_device_id(const class DeviceEnumerator *enumerator);

    std::chrono::time_point<std::chrono::high_resolution_clock> m_last_reconnect_time;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_last_poll_time;

    ServerDeviceViewPtr *m_deviceViews;
};

#endif // DEVICE_TYPE_MANAGER
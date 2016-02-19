#ifndef DEVICE_MANAGER_H
#define DEVICE_MANAGER_H

//-- includes -----
#include <memory>
#include <chrono>
#include "DeviceEnumerator.h"
#include "PSMoveProtocol.pb.h"

//-- typedefs -----
class DeviceManagerConfig;
typedef std::shared_ptr<DeviceManagerConfig> DeviceManagerConfigPtr;

class ServerDeviceView;
typedef std::shared_ptr<ServerDeviceView> ServerDeviceViewPtr;

class ServerControllerView;
typedef std::shared_ptr<ServerControllerView> ServerControllerViewPtr;

class ServerTrackerView;
typedef std::shared_ptr<ServerTrackerView> ServerTrackerViewPtr;

class ServerHMDView;
typedef std::shared_ptr<ServerHMDView> ServerHMDViewPtr;

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
    void updateStateAndPredict();
    void publish();
    
    virtual int getMaxDevices() const =0;
    
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

    virtual bool can_update_connected_devices() = 0;
    virtual DeviceEnumerator *allocate_device_enumerator() = 0;
    virtual void free_device_enumerator(DeviceEnumerator *) = 0;
    virtual ServerDeviceView *allocate_device_view(int device_id) = 0;
    
    void send_device_list_changed_notification();
    
    virtual const PSMoveProtocol::Response_ResponseType getListUpdatedResponseType()=0;
    
    int find_first_closed_device_device_id();
    int find_open_device_device_id(const DeviceEnumerator &enumerator);
       
    std::chrono::time_point<std::chrono::high_resolution_clock> m_last_reconnect_time;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_last_poll_time;

    ServerDeviceViewPtr *m_deviceViews;
};

class ControllerManager : public DeviceTypeManager
{
public:
    ControllerManager();
    
    /// Call hid_init()
    bool startup() override;
    
    /// Call hid_close()
    void shutdown() override;
    
    static const int k_max_devices = 5;
    int getMaxDevices() const override
    { return ControllerManager::k_max_devices; }
    
    ServerControllerViewPtr getControllerViewPtr(int device_id);

    bool setControllerRumble(int controller_id, int rumble_amount);
    bool resetPose(int controller_id);
    
protected:
    bool can_update_connected_devices() override;
    DeviceEnumerator *allocate_device_enumerator() override;
    void free_device_enumerator(DeviceEnumerator *) override;
    ServerDeviceView *allocate_device_view(int device_id) override;
    
    const PSMoveProtocol::Response_ResponseType getListUpdatedResponseType() override
    { return ControllerManager::k_list_udpated_response_type; }
    
private:
    static const PSMoveProtocol::Response_ResponseType k_list_udpated_response_type = PSMoveProtocol::Response_ResponseType_CONTROLLER_LIST_UPDATED;
};

class TrackerManager : public DeviceTypeManager
{
public:
    TrackerManager();
    
    static const int k_max_devices = 1;
    int getMaxDevices() const override
    { return TrackerManager::k_max_devices; }
    
    ServerTrackerViewPtr getTrackerViewPtr(int device_id);
    
protected:
    bool can_update_connected_devices() override;
    DeviceEnumerator *allocate_device_enumerator() override;
    void free_device_enumerator(DeviceEnumerator *) override;
    ServerDeviceView *allocate_device_view(int device_id) override;
    
    const PSMoveProtocol::Response_ResponseType getListUpdatedResponseType() override
    { return TrackerManager::k_list_udpated_response_type; }

private:
    static const PSMoveProtocol::Response_ResponseType k_list_udpated_response_type = PSMoveProtocol::Response_ResponseType_TRACKER_LIST_UPDATED;
};

class HMDManager : public DeviceTypeManager
{
public:
    HMDManager();

    static const int k_max_devices = 1;
    int getMaxDevices() const override
    { return HMDManager::k_max_devices; }
    
    ServerHMDViewPtr getHMDViewPtr(int device_id);
   
protected:
    bool can_update_connected_devices() override;
    DeviceEnumerator *allocate_device_enumerator() override;
    void free_device_enumerator(DeviceEnumerator *) override;
    ServerDeviceView *allocate_device_view(int device_id) override;
    
    const PSMoveProtocol::Response_ResponseType getListUpdatedResponseType() override
    { return HMDManager::k_list_udpated_response_type; }

private:
    static const PSMoveProtocol::Response_ResponseType k_list_udpated_response_type = PSMoveProtocol::Response_ResponseType_HMD_LIST_UPDATED;
};

/// This is the class that is actually used by the PSMoveService.
class DeviceManager
{
public:
    DeviceManager();
    virtual ~DeviceManager();

    bool startup(); /**< Initialize the interfaces for each specific manager. */
    void update();  /**< Poll all connected devices for each specific manager. */
    void shutdown();/**< Shutdown the interfaces for each specific manager. */

    static inline DeviceManager *getInstance()
    { return m_instance; }

    inline int getControllerViewMaxCount() const
    { return m_controller_manager.getMaxDevices(); }
    inline int getTrackerViewMaxCount() const
    { return m_tracker_manager.getMaxDevices(); }
    inline int geHMDViewMaxCount() const
    { return m_hmd_manager.getMaxDevices(); }
        
    ServerControllerViewPtr getControllerViewPtr(int controller_id);
    ServerTrackerViewPtr getTrackerViewPtr(int tracker_id);
    ServerHMDViewPtr getHMDViewPtr(int hmd_id);
    
private:
    DeviceManagerConfigPtr m_config;

    /// Singleton instance of the class
    /// Assigned in startup, cleared in teardown
    static DeviceManager *m_instance;

public:
    ControllerManager m_controller_manager;
    TrackerManager m_tracker_manager;
    HMDManager m_hmd_manager;
};

#endif  // DEVICE_MANAGER_H
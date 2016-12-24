#ifndef DEVICE_MANAGER_H
#define DEVICE_MANAGER_H

//-- includes -----
#include <memory>
#include <chrono>
//#include "PSMoveProtocol.pb.h"

//-- typedefs -----
class DeviceManagerConfig;
typedef std::shared_ptr<DeviceManagerConfig> DeviceManagerConfigPtr;

class ServerControllerView;
typedef std::shared_ptr<ServerControllerView> ServerControllerViewPtr;

class ServerTrackerView;
typedef std::shared_ptr<ServerTrackerView> ServerTrackerViewPtr;

class ServerHMDView;
typedef std::shared_ptr<ServerHMDView> ServerHMDViewPtr;

//-- definitions -----
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

    int getControllerViewMaxCount() const;
    int getTrackerViewMaxCount() const;
    int getHMDViewMaxCount() const;
        
    ServerControllerViewPtr getControllerViewPtr(int controller_id);
    ServerTrackerViewPtr getTrackerViewPtr(int tracker_id);
    ServerHMDViewPtr getHMDViewPtr(int hmd_id);
    
private:
    DeviceManagerConfigPtr m_config;

    /// Singleton instance of the class
    /// Assigned in startup, cleared in teardown
    static DeviceManager *m_instance;

public:
	class PlatformDeviceManager *m_platform_manager;
    class ControllerManager *m_controller_manager;
    class TrackerManager *m_tracker_manager;
    class HMDManager *m_hmd_manager;
};

#endif  // DEVICE_MANAGER_H
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
        
    ServerControllerViewPtr getControllerViewPtr(int controller_id);
    ServerTrackerViewPtr getTrackerViewPtr(int tracker_id);
    
private:
    DeviceManagerConfigPtr m_config;

    /// Singleton instance of the class
    /// Assigned in startup, cleared in teardown
    static DeviceManager *m_instance;

public:
    class ControllerManager *m_controller_manager;
    class TrackerManager *m_tracker_manager;
};

#endif  // DEVICE_MANAGER_H
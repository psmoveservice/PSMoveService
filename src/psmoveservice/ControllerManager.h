#ifndef CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_H

//-- includes -----
#include <memory>
#include <chrono>
#include "ControllerEnumerator.h"

//-- constants -----
static const int k_max_controllers= 5;

//-- typedefs -----
class ControllerManagerConfig;
typedef std::shared_ptr<ControllerManagerConfig> ControllerManagerConfigPtr;

class ServerControllerView;
typedef std::shared_ptr<ServerControllerView> ServerControllerViewPtr;

//-- definitions -----
class ControllerManager 
{
public:
    ControllerManager();
    virtual ~ControllerManager();

    static ControllerManager *getInstance() { return m_instance; }

    bool startup();
    void update();
    void shutdown();

    ServerControllerViewPtr getControllerView(int controller_id);
    int getControllerViewCount() const;

    bool setControllerRumble(int controller_id, int rumble_amount);
    bool resetPose(int controller_id);    

private:
    void update_controllers();
    void update_connected_controllers();
    void send_controller_list_changed_notification();
    int find_open_controller_controller_id(const ControllerDeviceEnumerator &enumerator);
    int find_first_closed_controller_controller_id();

    /// Singleton instance of the class
    /// Assigned in startup, cleared in teardown
    static ControllerManager *m_instance;
    
    ControllerManagerConfigPtr m_config;
    ServerControllerViewPtr m_controllers[k_max_controllers];
    std::chrono::time_point<std::chrono::high_resolution_clock> m_last_reconnect_time;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_last_poll_time;
};

#endif  // CONTROLLER_MANAGER_H

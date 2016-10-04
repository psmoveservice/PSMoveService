//-- includes -----
#include "DeviceManager.h"

#include "ControllerManager.h"
#include "DeviceEnumerator.h"
#include "OrientationFilter.h"
#include "ServerControllerView.h"
#include "ServerTrackerView.h"
#include "ServerRequestHandler.h"
#include "ServerLog.h"
#include "ServerDeviceView.h"
#include "ServerNetworkManager.h"
#include "ServerUtility.h"
#include "PSMoveProtocol.pb.h"
#include "PSMoveConfig.h"
#include "TrackerManager.h"
#include <chrono>

//-- constants -----
static const int k_default_controller_reconnect_interval= 1000; // ms
static const int k_default_controller_poll_interval= 2; // ms
static const int k_default_tracker_reconnect_interval= 10000; // ms
static const int k_default_tracker_poll_interval= 13; // 1000/75 ms

class DeviceManagerConfig : public PSMoveConfig
{
public:
    DeviceManagerConfig(const std::string &fnamebase = "ControllerManagerConfig")
        : PSMoveConfig(fnamebase)
        , controller_reconnect_interval(k_default_controller_reconnect_interval)
        , controller_poll_interval(k_default_controller_poll_interval)
        , tracker_reconnect_interval(k_default_tracker_reconnect_interval)
        , tracker_poll_interval(k_default_tracker_poll_interval)
    {};

    const boost::property_tree::ptree
    config2ptree()
    {
        boost::property_tree::ptree pt;
    
        pt.put("controller_reconnect_interval", controller_reconnect_interval);
        pt.put("controller_poll_interval", controller_poll_interval);
        pt.put("tracker_reconnect_interval", tracker_reconnect_interval);
        pt.put("tracker_poll_interval", tracker_poll_interval);

        return pt;
    }

    void
    ptree2config(const boost::property_tree::ptree &pt)
    {
        controller_reconnect_interval = pt.get<int>("controller_reconnect_interval", k_default_controller_reconnect_interval);
        controller_poll_interval = pt.get<int>("controller_poll_interval", k_default_controller_poll_interval);
        tracker_reconnect_interval = pt.get<int>("tracker_reconnect_interval", k_default_tracker_reconnect_interval);
        tracker_poll_interval = pt.get<int>("tracker_poll_interval", k_default_tracker_poll_interval);
    }

    int controller_reconnect_interval;
    int controller_poll_interval;
    int tracker_reconnect_interval;
    int tracker_poll_interval;
};

// DeviceManager - This is the interface used by PSMoveService
DeviceManager *DeviceManager::m_instance= nullptr;

DeviceManager::DeviceManager()
    : m_config() // NULL config until startup
    , m_controller_manager(new ControllerManager())
    , m_tracker_manager(new TrackerManager())
{
}

DeviceManager::~DeviceManager()
{
    delete m_controller_manager;
    delete m_tracker_manager;
}

bool
DeviceManager::startup()
{
    bool success= true;

    m_config = DeviceManagerConfigPtr(new DeviceManagerConfig);

	// Load the config from disk
    m_config->load();

	// Save the config back out again in case defaults changed
	m_config->save();
    
    m_controller_manager->reconnect_interval = m_config->controller_reconnect_interval;
    m_controller_manager->poll_interval = m_config->controller_poll_interval;
    success &= m_controller_manager->startup();
    
    m_tracker_manager->reconnect_interval = m_config->tracker_reconnect_interval;
    m_tracker_manager->poll_interval = m_config->tracker_poll_interval;
    success &= m_tracker_manager->startup();

    m_instance= this;
    
    return success;
}

void
DeviceManager::update()
{
    m_controller_manager->poll(); // Update controller counts and poll button/IMU state
    m_tracker_manager->poll(); // Update tracker count and poll video frames

    m_controller_manager->updateStateAndPredict(m_tracker_manager); // Compute pose/prediction of tracking blob+IMU state

    m_controller_manager->publish(); // publish controller state to any listening clients  (common case)
    m_tracker_manager->publish(); // publish tracker state to any listening clients (probably only used by ConfigTool)
}

void
DeviceManager::shutdown()
{
    m_config->save();

    m_controller_manager->shutdown();
    m_tracker_manager->shutdown();

    m_instance= nullptr;
}

int 
DeviceManager::getControllerViewMaxCount() const
{
    return m_controller_manager->getMaxDevices();
}

int 
DeviceManager::getTrackerViewMaxCount() const
{
    return m_tracker_manager->getMaxDevices();
}

ServerControllerViewPtr
DeviceManager::getControllerViewPtr(int device_id)
{
    ServerControllerViewPtr result;
    if (ServerUtility::is_index_valid(device_id, m_controller_manager->getMaxDevices()))
    {
        result = m_controller_manager->getControllerViewPtr(device_id);
    }

    return result;
}

ServerTrackerViewPtr
DeviceManager::getTrackerViewPtr(int tracker_id)
{
    ServerTrackerViewPtr result;
    if (ServerUtility::is_index_valid(tracker_id, m_tracker_manager->getMaxDevices()))
    {
        result = m_tracker_manager->getTrackerViewPtr(tracker_id);
    }

    return result;
}
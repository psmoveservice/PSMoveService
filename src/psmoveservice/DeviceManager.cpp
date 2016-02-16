//-- includes -----
#include "DeviceManager.h"
#include "DeviceEnumerator.h"
#include "OrientationFilter.h"
#include "ServerControllerView.h"
#include "ServerHMDView.h"
#include "ServerTrackerView.h"
#include "ServerRequestHandler.h"
#include "ServerLog.h"
#include "ServerDeviceView.h"
#include "ServerNetworkManager.h"
#include "ServerUtility.h"
#include "PSMoveProtocol.pb.h"
#include "PSMoveConfig.h"
#include "hidapi.h"
#include <chrono>

// constants
static const int k_default_controller_reconnect_interval= 1000; // ms
static const int k_default_controller_poll_interval= 2; // ms
static const int k_default_tracker_reconnect_interval= 10000; // ms
static const int k_default_tracker_poll_interval= 13; // 1000/75 ms
static const int k_default_hmd_reconnect_interval= 10000; // ms
static const int k_default_hmd_poll_interval= 2; // ms

class DeviceManagerConfig : public PSMoveConfig
{
public:
    DeviceManagerConfig(const std::string &fnamebase = "ControllerManagerConfig")
        : PSMoveConfig(fnamebase)
        , controller_reconnect_interval(k_default_controller_reconnect_interval)
        , controller_poll_interval(k_default_controller_poll_interval)
        , tracker_reconnect_interval(k_default_tracker_reconnect_interval)
        , tracker_poll_interval(k_default_tracker_poll_interval)
        , hmd_reconnect_interval(k_default_hmd_reconnect_interval)
        , hmd_poll_interval(k_default_hmd_poll_interval)

    {};

    const boost::property_tree::ptree
    config2ptree()
    {
        boost::property_tree::ptree pt;
    
        pt.put("controller_reconnect_interval", controller_reconnect_interval);
        pt.put("controller_poll_interval", controller_poll_interval);
        pt.put("tracker_reconnect_interval", tracker_reconnect_interval);
        pt.put("tracker_poll_interval", tracker_poll_interval);
        pt.put("hmd_reconnect_interval", hmd_reconnect_interval);
        pt.put("hmd_poll_interval", hmd_poll_interval);

        return pt;
    }

    void
    ptree2config(const boost::property_tree::ptree &pt)
    {
        controller_reconnect_interval = pt.get<int>("controller_reconnect_interval", k_default_controller_reconnect_interval);
        controller_poll_interval = pt.get<int>("controller_poll_interval", k_default_controller_poll_interval);
        tracker_reconnect_interval = pt.get<int>("tracker_reconnect_interval", k_default_tracker_reconnect_interval);
        tracker_poll_interval = pt.get<int>("tracker_poll_interval", k_default_tracker_poll_interval);
        hmd_reconnect_interval = pt.get<int>("hmd_reconnect_interval", k_default_hmd_reconnect_interval);
        hmd_poll_interval = pt.get<int>("hmd_poll_interval", k_default_hmd_poll_interval);
    }

    int controller_reconnect_interval;
    int controller_poll_interval;
    int tracker_reconnect_interval;
    int tracker_poll_interval;
    int hmd_reconnect_interval;
    int hmd_poll_interval;
};

// DeviceTypeManager

/// Constructor and set intervals (ms) for reconnect and polling
DeviceTypeManager::DeviceTypeManager(const int recon_int, const int poll_int)
    : reconnect_interval(recon_int)
    , poll_interval(poll_int)
{
}

DeviceTypeManager::~DeviceTypeManager()
{
    
}

/// Override if the device type needs to initialize any services (e.g., hid_init)
bool
DeviceTypeManager::startup()
{
    return true;
}

/// Calls poll_devices and update_connected_devices if poll_interval and reconnect_interval has elapsed, respectively.
void
DeviceTypeManager::poll()
{
    std::chrono::time_point<std::chrono::high_resolution_clock> now= std::chrono::high_resolution_clock::now();
    
    // See if it's time to poll controllers for data
    std::chrono::duration<double, std::milli> update_diff = now - m_last_poll_time;
    
    if (update_diff.count() >= poll_interval)
    {
        poll_devices();
        m_last_poll_time= now;
    }
    
    // See if it's time to try update the list of connected devices
    std::chrono::duration<double, std::milli> reconnect_diff = now - m_last_reconnect_time;
    if (reconnect_diff.count() >= reconnect_interval)
    {
        if (update_connected_devices())
        {
            m_last_reconnect_time= now;
        }
    }
}

void 
DeviceTypeManager::updateStateAndPredict()
{
    // Recompute the state-space data about the device and make predictions about the future
    for (int device_id = 0; device_id < getMaxDevices(); ++device_id)
    {
        ServerDeviceViewPtr device = getDeviceViewPtr(device_id);
        
        device->updateStateAndPredict();
    }
}

void 
DeviceTypeManager::publish()
{
    // Publish any new data to client connections
    for (int device_id = 0; device_id < getMaxDevices(); ++device_id)
    {
        ServerDeviceViewPtr device = getDeviceViewPtr(device_id);
        
        device->publish();
    }
}

void
DeviceTypeManager::shutdown()
{
    // Close any controllers that were opened
    for (int device_id = 0; device_id < getMaxDevices(); ++device_id)
    {
        ServerDeviceViewPtr device = getDeviceViewPtr(device_id);

        if (device && device->getIsOpen())
        {
            getDeviceViewPtr(device_id)->close();
        }
    }
}

void
DeviceTypeManager::send_device_list_changed_notification()
{
    ResponsePtr response(new PSMoveProtocol::Response);
    response->set_type(getListUpdatedResponseType());
    response->set_request_id(-1);
    response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);
    
    ServerNetworkManager::get_instance()->send_notification_to_all_clients(response);
}

void
DeviceTypeManager::poll_devices()
{
    bool bAllUpdatedOk= true;
    
    for (int device_id = 0; device_id < getMaxDevices(); ++device_id)
    {
        ServerDeviceViewPtr device = getDeviceViewPtr(device_id);
        bAllUpdatedOk &= device->poll();
    }
    
    if (!bAllUpdatedOk)
    {
        send_device_list_changed_notification();
    }
}

int
DeviceTypeManager::find_open_device_device_id(const DeviceEnumerator &enumerator)
{
    int result_device_id= -1;
    
    for (int device_id = 0; device_id < getMaxDevices(); ++device_id)
    {
        ServerDeviceViewPtr device= getDeviceViewPtr(device_id);
        
        if (device && device->matchesDeviceEnumerator(&enumerator))
        {
            result_device_id= device_id;
            break;
        }
    }
    
    return result_device_id;
}

int
DeviceTypeManager::find_first_closed_device_device_id()
{
    int result_device_id= -1;
    for (int device_id = 0; device_id < getMaxDevices(); ++device_id)
    {
        ServerDeviceViewPtr device= getDeviceViewPtr(device_id);
        
        if (device && !device->getIsOpen())
        {
            result_device_id= device_id;
            break;
        }
    }
    return result_device_id;
}

// ControllerManager

// TODO: Can initialization of m_devices done in type-agnostic way?
// If so, then it should happen in parent DeviceTypeManager::DeviceTypeManager()
ControllerManager::ControllerManager()
: DeviceTypeManager(1000, 2)
{
    // Allocate all of the devices
    for (int controller_id = 0; controller_id < k_max_devices; ++controller_id)
    {
        ServerControllerViewPtr controller = ServerControllerViewPtr(new ServerControllerView(controller_id));
        
        m_devices[controller_id]= controller;
    }
}

ControllerManager::~ControllerManager()
{
    // Deallocate the controllers
    for (int device_id = 0; device_id < k_max_devices; ++device_id)
    {
        m_devices[device_id]= ServerControllerViewPtr();
    }
}

bool
ControllerManager::startup()
{
    bool success = true;
    // Initialize HIDAPI
    if (hid_init() == -1)
    {
        SERVER_LOG_ERROR("ControllerManager::startup") << "Failed to initialize HIDAPI";
        success= false;
    }
    return success;
}

void
ControllerManager::shutdown()
{
    DeviceTypeManager::shutdown();
    
    // Shutdown HIDAPI
    hid_exit();
}

bool
ControllerManager::update_connected_devices()
{
    bool success = false;
    // Don't do any connection opening/closing until all pending bluetooth operations are finished
    if (!ServerRequestHandler::get_instance()->any_active_bluetooth_requests())
    {
        ServerControllerViewPtr *temp_controllers_list = new ServerControllerViewPtr[k_max_devices];
        bool bSendControllerUpdatedNotification= false;
        
        // Step 1
        // See if any controllers shuffled order OR if any new controllers were attached.
        // Migrate open controllers to a new temp list in the order
        // that they appear in the device enumerator.
        for (ControllerDeviceEnumerator enumerator; enumerator.is_valid(); enumerator.next())
        {
            // Find controller index for the controller with the matching device path
            int controller_id= find_open_device_device_id(enumerator);
            
            // Existing controller case (Most common)
            if (controller_id != -1)
            {
                // Fetch the controller from it's existing controller slot
                ServerControllerViewPtr existingController= m_devices[controller_id];
                
                // Move it to the same slot in the temp list
                temp_controllers_list[controller_id]= existingController;
                
                // Remove it from the previous list
                m_devices[controller_id]= ServerControllerViewPtr();
            }
            // New controller connected case
            else
            {
                int controller_id= find_first_closed_device_device_id();
                
                if (controller_id != -1)
                {
                    // Fetch the controller from it's existing controller slot
                    ServerControllerViewPtr existingController= m_devices[controller_id];
                    
                    // Move it to the available slot
                    existingController->setDeviceID(static_cast<int>(controller_id));
                    temp_controllers_list[controller_id]= existingController;
                    
                    // Remove it from the previous list
                    m_devices[controller_id]= ServerControllerViewPtr();
                    
                    // Attempt to open the controller
                    if (existingController->open(&enumerator))
                    {
                        SERVER_LOG_INFO("ControllerManager::reconnect_controllers") <<
                        "Controller controller_id " << controller_id << " connected";
                        bSendControllerUpdatedNotification= true;
                    }
                }
                else
                {
                    SERVER_LOG_ERROR("ControllerManager::reconnect_controllers") << "Can't connect any more new controllers. Too many open controllers";
                    break;
                }
            }
        }
        
        // Step 2
        // Close any remaining open controllers not listed in the device enumerator.
        // Copy over any closed controllers to the temp.
        for (int existing_controller_id = 0; existing_controller_id < k_max_devices; ++existing_controller_id)
        {
            ServerControllerViewPtr &existingController= m_devices[existing_controller_id];
            
            if (existingController)
            {
                // Any "open" controllers remaining in the old list need to be closed
                // since they no longer appear in the connected device list.
                // This probably shouldn't happen very often (at all?) as polling should catch
                // disconnected controllers first.
                if (existingController->getIsOpen())
                {
                    SERVER_LOG_WARNING("ControllerManager::reconnect_controllers") << "Closing controller "
                    << existing_controller_id << " since it's no longer in the device list.";
                    existingController->close();
                    bSendControllerUpdatedNotification= true;
                }
                
                // Move it to the temp slot
                temp_controllers_list[existing_controller_id]= existingController;
                
                // Remove it from the previous list
                m_devices[existing_controller_id]= ServerControllerViewPtr();
            }
        }
        
        // Step 3
        // Copy the temp controller list back over top the original list
        for (int controller_id = 0; controller_id < k_max_devices; ++controller_id)
        {
            m_devices[controller_id]= temp_controllers_list[controller_id];
        }
        
        if (bSendControllerUpdatedNotification)
        {
            send_device_list_changed_notification();
        }
        
        delete[] temp_controllers_list;
        
        success = true;
    }
    
    return success;
}

bool
ControllerManager::setControllerRumble(int controller_id, int rumble_amount)
{
    bool result= false;
    
    if (ServerUtility::is_index_valid(controller_id, k_max_devices))
    {
        result = m_devices[controller_id]->setControllerRumble(rumble_amount);
    }
    
    return result;
}

bool
ControllerManager::resetPose(int controller_id)
{
    bool bSuccess = false;
    ServerControllerViewPtr ControllerPtr = getControllerViewPtr(controller_id);

    if (ControllerPtr)
    {
        OrientationFilter *filter= ControllerPtr->getOrientationFilter();

        if (filter != nullptr)
        {
            filter->resetOrientation();
            bSuccess = true;
        }
    }

    return bSuccess;
}

ServerDeviceViewPtr
ControllerManager::getDeviceViewPtr(int device_id)
{
    return m_devices[device_id];
}

ServerControllerViewPtr
ControllerManager::getControllerViewPtr(int device_id)
{
    return m_devices[device_id];
}

// Tracker

TrackerManager::TrackerManager()
    : DeviceTypeManager(10000, 13)
{
    // Allocate all of the devices
    for (int tracker_id = 0; tracker_id < k_max_devices; ++tracker_id)
    {
        ServerTrackerViewPtr tracker = ServerTrackerViewPtr(new ServerTrackerView(tracker_id));
        
        m_devices[tracker_id]= tracker;
    }
}

TrackerManager::~TrackerManager()
{
    // Deallocate the controllers
    for (int device_id = 0; device_id < k_max_devices; ++device_id)
    {
        m_devices[device_id]= ServerTrackerViewPtr();
    }
}

bool
TrackerManager::update_connected_devices()
{
    return false;
}

ServerDeviceViewPtr
TrackerManager::getDeviceViewPtr(int device_id)
{
    return m_devices[device_id];
}

ServerTrackerViewPtr
TrackerManager::getTrackerViewPtr(int device_id)
{
    return m_devices[device_id];
}

// HMD

HMDManager::HMDManager()
: DeviceTypeManager(1000, 2)
{
    // Allocate all of the devices
    for (int hmd_id = 0; hmd_id < k_max_devices; ++hmd_id)
    {
        ServerHMDViewPtr hmd = ServerHMDViewPtr(new ServerHMDView(hmd_id));
        
        m_devices[hmd_id]= hmd;
    }
}

HMDManager::~HMDManager()
{
    // Deallocate the controllers
    for (int hmd_id = 0; hmd_id < k_max_devices; ++hmd_id)
    {
        m_devices[hmd_id]= ServerHMDViewPtr();
    }
}

ServerDeviceViewPtr
HMDManager::getDeviceViewPtr(int device_id)
{
    return m_devices[device_id];
}

ServerHMDViewPtr
HMDManager::getHMDViewPtr(int device_id)
{
    return m_devices[device_id];
}

bool
HMDManager::update_connected_devices()
{
    return false;
}

// DeviceManager - This is the interface used by PSMoveService
DeviceManager *DeviceManager::m_instance= nullptr;

DeviceManager::DeviceManager()
    : m_config() // NULL config until startup
    , m_controller_manager()
    , m_tracker_manager()
    , m_hmd_manager()
{
}

DeviceManager::~DeviceManager()
{
}

bool
DeviceManager::startup()
{
    bool success= true;

    m_config = DeviceManagerConfigPtr(new DeviceManagerConfig);
    m_config->load();
    
    m_controller_manager.reconnect_interval = m_config->controller_reconnect_interval;
    m_controller_manager.poll_interval = m_config->controller_poll_interval;
    success &= m_controller_manager.startup();
    
    m_tracker_manager.reconnect_interval = m_config->tracker_reconnect_interval;
    m_tracker_manager.poll_interval = m_config->tracker_poll_interval;
    success &= m_tracker_manager.startup();
    
    m_hmd_manager.reconnect_interval = m_config->hmd_reconnect_interval;
    m_hmd_manager.poll_interval = m_config->hmd_poll_interval;
    success &= m_hmd_manager.startup();

    m_instance= this;
    
    return success;
}

void
DeviceManager::update()
{
    m_controller_manager.poll(); // Update controller counts and poll button/IMU state
    m_tracker_manager.poll(); // Update tracker count and poll video frames
    m_hmd_manager.poll(); // Update HMD count and poll position/orientation state

    m_tracker_manager.updateStateAndPredict(); // Get controller colors and update tracking blob positions/predictions
    m_controller_manager.updateStateAndPredict(); // Compute pose/prediction of tracking blob+IMU state
    m_hmd_manager.updateStateAndPredict(); // Get the pose + prediction for HMD

    m_controller_manager.publish(); // publish controller state to any listening clients  (common case)
    m_tracker_manager.publish(); // publish tracker state to any listening clients (probably only used by ConfigTool)
    m_hmd_manager.publish(); // publish hmd state to any listening clients (probably only used by ConfigTool)
}

void
DeviceManager::shutdown()
{
    m_config->save();

    m_controller_manager.shutdown();
    m_tracker_manager.shutdown();
    m_hmd_manager.shutdown();

    m_instance= nullptr;
}

ServerControllerViewPtr
DeviceManager::getControllerViewPtr(int device_id)
{
    ServerControllerViewPtr result;
    if (ServerUtility::is_index_valid(device_id, m_controller_manager.getMaxDevices()))
    {
        result= m_controller_manager.getControllerViewPtr(device_id);
    }
    return result;
}

/*
ServerTrackerView DeviceManager::getTrackerViewPtr(int tracker_id)
{
    
}

ServerHMDView DeviceManager::getHMDViewPtr(int hmd_id)
{
    
}
*/
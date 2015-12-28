//-- includes -----
#include "ControllerManager.h"
#include "ControllerEnumerator.h"
#include "ServerRequestHandler.h"
#include "ServerLog.h"
#include "ServerControllerView.h"
#include "ServerNetworkManager.h"
#include "ServerUtility.h"
#include "PSMoveProtocol.pb.h"
#include "PSMoveConfig.h"
#include "hidapi.h"
#include <boost/date_time/posix_time/posix_time_types.hpp>

//-- constants -----
static const int k_default_controller_reconnect_interval= 1000; // 1000ms
static const int k_default_controller_poll_interval= 2; // 2ms

//-- private implementation -----
class ControllerManagerConfig : public PSMoveConfig
{
public:
    ControllerManagerConfig(const std::string &fnamebase = "ControllerManagerConfig")
        : PSMoveConfig(fnamebase)
        , controller_update_interval(k_default_controller_poll_interval)
        , controller_reconnect_interval(k_default_controller_reconnect_interval)
    {};

    const boost::property_tree::ptree
    config2ptree()
    {
        boost::property_tree::ptree pt;
    
        pt.put("controller_poll_interval", controller_update_interval);
        pt.put("controller_reconnect_interval", controller_reconnect_interval);

        return pt;
    }

    void
    ptree2config(const boost::property_tree::ptree &pt)
    {
        controller_update_interval = pt.get<int>("controller_poll_interval", k_default_controller_poll_interval);
        controller_reconnect_interval = pt.get<int>("controller_reconnect_interval", k_default_controller_reconnect_interval);
    }

    int controller_update_interval;
    int controller_reconnect_interval;
};
typedef std::shared_ptr<ControllerManagerConfig> ControllerManagerConfigPtr;

class ControllerManagerImpl
{
public:
    ControllerManagerImpl()
        : m_config() // NULL config until startup
        , m_last_reconnect_time()
        , m_last_poll_time()
    {
        // Allocate all of the controllers
        for (int controller_id = 0; controller_id < k_max_controllers; ++controller_id)
        {
            ServerControllerViewPtr controller = ServerControllerViewPtr(new ServerControllerView(controller_id));

            m_controllers[controller_id]= controller;
        }
    }

    virtual ~ControllerManagerImpl()
    {
        // Deallocate the controllers
        for (int controller_id = 0; controller_id < k_max_controllers; ++controller_id)
        {
            m_controllers[controller_id]= ServerControllerViewPtr();
        }        
    }

    bool startup()
    {
        bool success= true;

        m_config = ControllerManagerConfigPtr(new ControllerManagerConfig);
        m_config->load();

        // Initialize HIDAPI
        if (hid_init() == -1)
        {
            SERVER_LOG_ERROR("ControllerManagerImpl::startup") << "Failed to initialize HIDAPI";
            success= false;
        }
        
        return success;
    }

    void update()
    {        
        boost::posix_time::ptime now= boost::posix_time::microsec_clock::local_time();

        // See if it's time to poll controllers for data
        boost::posix_time::time_duration update_diff = now - m_last_poll_time;
        if (update_diff.total_milliseconds() >= m_config->controller_update_interval)
        {
            update_controllers();
            m_last_poll_time= now;
        }

        // See if it's time to try update the list of connected controllers
        boost::posix_time::time_duration reconnect_diff = now - m_last_reconnect_time;
        if (reconnect_diff.total_milliseconds() >= m_config->controller_reconnect_interval)
        {
            // Don't do any connection opening/closing until all pending bluetooth operations are finished
            if (!ServerRequestHandler::get_instance()->any_active_bluetooth_requests())
            {
                // This method tries make the list of open controllers in m_controllers match 
                // the list of connected controller devices in the device enumerator.
                // Not controller objects are created or destroyed.
                // Pointers are just shuffled around and controllers opened and closed.
                update_connected_controllers();
            }

            m_last_reconnect_time= now;
        }
    }

    void shutdown()
    {
        m_config->save();

        // Close any controllers that were opened
        for (int controller_id = 0; controller_id < k_max_controllers; ++controller_id)
        {
            if (m_controllers[controller_id]->getIsOpen())
            {
                m_controllers[controller_id]->close();
            }
        }

        // Shutdown HIDAPI
        hid_exit();
    }

    ServerControllerViewPtr getController(int controller_id)
    {
        ServerControllerViewPtr result;

        if (ServerUtility::is_index_valid(controller_id, k_max_controllers))
        {
            result= m_controllers[controller_id];
        }

        return result;
    }

    int getControllerViewCount() const
    {
        return k_max_controllers;
    }

    bool setControllerRumble(int controller_id, int rumble_amount)
    {
        bool result= false;

        if (ServerUtility::is_index_valid(controller_id, k_max_controllers))
        {
            result = m_controllers[controller_id]->setControllerRumble(rumble_amount);
        }

        return result;
    }

    bool resetPose(int controller_id)
    {
        //###bwalker $TODO Once we are computing pose
        return false;
    }

protected:
    void update_controllers()
    {
        bool bAllUpdatedOk= true;

        for (int controller_id = 0; controller_id < k_max_controllers; ++controller_id)
        {
            ServerControllerViewPtr &controller= m_controllers[controller_id];

            bAllUpdatedOk&= controller->update();
        }

        if (!bAllUpdatedOk)
        {
            send_controller_list_changed_notification();
        }
    }

    void update_connected_controllers()
    {
        ServerControllerViewPtr temp_controllers_list[k_max_controllers];
        bool bSendControllerUpdatedNotification= false;

        // Step 1
        // See if any controllers shuffled order OR if any new controllers were attached.
        // Migrate open controllers to a new temp list in the order
        // that they appear in the device enumerator.
        for (ControllerDeviceEnumerator enumerator; enumerator.is_valid(); enumerator.next())
        {
            // Find controller index for the controller with the matching device path
            int controller_id= find_open_controller_controller_id(enumerator);

            // Existing controller case (Most common)
            if (controller_id != -1)
            {
                // Fetch the controller from it's existing controller slot
                ServerControllerViewPtr existingController= m_controllers[controller_id];

                // Move it to the same slot in the temp list
                temp_controllers_list[controller_id]= existingController;

                // Remove it from the previous list
                m_controllers[controller_id]= ServerControllerViewPtr();
            }
            // New controller connected case
            else
            {
                int controller_id= find_first_closed_controller_controller_id();

                if (controller_id != -1)
                {
                    // Fetch the controller from it's existing controller slot
                    ServerControllerViewPtr existingController= m_controllers[controller_id];

                    // Move it to the available slot
                    existingController->setControllerID(static_cast<int>(controller_id));
                    temp_controllers_list[controller_id]= existingController;

                    // Remove it from the previous list
                    m_controllers[controller_id]= ServerControllerViewPtr();

                    // Attempt to open the controller
                    if (existingController->open(&enumerator))
                    {
                        SERVER_LOG_INFO("ControllerManagerImpl::reconnect_controllers") << 
                            "Controller controller_id " << controller_id << " connected";
                        bSendControllerUpdatedNotification= true;
                    }
                }
                else
                {
                    SERVER_LOG_ERROR("ControllerManagerImpl::reconnect_controllers") << "Can't connect any more new controllers. Too many open controllers";
                    break;
                }
            }
        }

        // Step 2
        // Close any remaining open controllers not listed in the device enumerator.
        // Copy over any closed controllers to the temp.
        for (int existing_controller_id = 0; existing_controller_id < k_max_controllers; ++existing_controller_id)
        {
            ServerControllerViewPtr &existingController= m_controllers[existing_controller_id];

            if (existingController)
            {
                // Any "open" controllers remaining in the old list need to be closed
                // since they no longer appear in the connected device list.
                // This probably shouldn't happen very often (at all?) as polling should catch
                // disconnected controllers first.
                if (existingController->getIsOpen())
                {
                    SERVER_LOG_WARNING("ControllerManagerImpl::reconnect_controllers") << "Closing controller " 
                        << existing_controller_id << " since it's no longer in the device list.";
                    existingController->close();
                    bSendControllerUpdatedNotification= true;
                }

                // Move it to the temp slot
                temp_controllers_list[existing_controller_id]= existingController;

                // Remove it from the previous list
                m_controllers[existing_controller_id]= ServerControllerViewPtr();
            }
        }

        // Step 3
        // Copy the temp controller list back over top the original list
        for (int controller_id = 0; controller_id < k_max_controllers; ++controller_id)
        {
            m_controllers[controller_id]= temp_controllers_list[controller_id];
        }

        if (bSendControllerUpdatedNotification)
        {
            send_controller_list_changed_notification();
        }
    }

    void send_controller_list_changed_notification()
    {
        ResponsePtr response(new PSMoveProtocol::Response);
        response->set_type(PSMoveProtocol::Response_ResponseType_CONTROLLER_LIST_UPDATED);
        response->set_request_id(-1);
        response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);

        ServerNetworkManager::get_instance()->send_notification_to_all_clients(response);
    }
    
    int find_open_controller_controller_id(const ControllerDeviceEnumerator &enumerator)
    {
        int result_controller_id= -1;

        for (int controller_id = 0; controller_id < k_max_controllers; ++controller_id)
        {
            ServerControllerViewPtr &controller= m_controllers[controller_id];

            if (controller && controller->getIsOpen() && controller->matchesDeviceEnumerator(&enumerator))
            {
                result_controller_id= controller_id;
                break;
            }
        }
        
        return result_controller_id;
    }

    int find_first_closed_controller_controller_id()
    {
        int result_controller_id= -1;

        for (int controller_id = 0; controller_id < k_max_controllers; ++controller_id)
        {
            ServerControllerViewPtr &controller= m_controllers[controller_id];

            if (controller && !controller->getIsOpen())
            {
                result_controller_id= controller_id;
                break;
            }
        }
        
        return result_controller_id;
    }

private:
    ControllerManagerConfigPtr m_config;
    ServerControllerViewPtr m_controllers[k_max_controllers];
    boost::posix_time::ptime m_last_reconnect_time;
    boost::posix_time::ptime m_last_poll_time;
};

//-- public interface -----
ControllerManager *ControllerManager::m_instance = NULL;

ControllerManager::ControllerManager()
    : m_implementation_ptr(new ControllerManagerImpl)
{
}

ControllerManager::~ControllerManager()
{
    delete m_implementation_ptr;
}

bool ControllerManager::startup()
{
    ControllerManager::m_instance= this;

    return m_implementation_ptr->startup();
}

void ControllerManager::update()
{
    m_implementation_ptr->update();
}

void ControllerManager::shutdown()
{
    ControllerManager::m_instance= nullptr;

    m_implementation_ptr->shutdown();
}

ServerControllerViewPtr ControllerManager::getControllerView(int controller_id)
{
    return m_implementation_ptr->getController(controller_id);
}

int ControllerManager::getControllerViewCount() const
{
    return m_implementation_ptr->getControllerViewCount();
}

bool ControllerManager::setControllerRumble(int controller_id, int rumble_amount)
{
    return m_implementation_ptr->setControllerRumble(controller_id, rumble_amount);
}

bool ControllerManager::resetPose(int controller_id)
{
    return m_implementation_ptr->resetPose(controller_id);
}
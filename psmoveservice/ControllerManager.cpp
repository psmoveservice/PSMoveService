//-- includes -----
#include "ControllerManager.h"
#include "ServerRequestHandler.h"
#include "ServerLog.h"
#include "PSMoveProtocol.pb.h"
#include "PSMoveController.h"
#include "PSMoveConfig.h"
#include <boost/date_time/posix_time/posix_time_types.hpp>

//-- constants -----
static const int k_default_controller_reconnect_interval= 1000; // 1000ms
static const int k_default_controller_poll_interval= 2; // 2ms

//-- typedefs -----
typedef std::shared_ptr<PSMoveController> PSMoveControllerPtr;

//-- macros -----
#define SET_BUTTON_BIT(bitmask, bit_index, button_state) \
    bitmask|= (button_state == Button_DOWN || button_state == Button_PRESSED) ? (0x1 << (bit_index)) : 0x0;

//-- private implementation -----
class ControllerManagerConfig : public PSMoveConfig
{
public:
    ControllerManagerConfig(const std::string &fnamebase = "ControllerManagerConfig")
        : PSMoveConfig(fnamebase)
        , controller_poll_interval(k_default_controller_poll_interval)
        , controller_reconnect_interval(k_default_controller_reconnect_interval)
    {};

    const boost::property_tree::ptree
    ControllerManagerConfig::config2ptree()
    {
        boost::property_tree::ptree pt;
    
        pt.put("controller_poll_interval", controller_poll_interval);
        pt.put("controller_reconnect_interval", controller_reconnect_interval);

        return pt;
    }

    void
    ControllerManagerConfig::ptree2config(const boost::property_tree::ptree &pt)
    {
        controller_poll_interval = pt.get<int>("controller_poll_interval", k_default_controller_poll_interval);
        controller_reconnect_interval = pt.get<int>("controller_reconnect_interval", k_default_controller_reconnect_interval);
    }

    int controller_poll_interval;
    int controller_reconnect_interval;
};
typedef std::shared_ptr<ControllerManagerConfig> ControllerManagerConfigPtr;

class ControllerManagerImpl
{
public:
    ControllerManagerImpl()
        : m_config() // NULL config until startup
        , m_sequence_number(0)
        , m_last_reconnect_time()
        , m_last_poll_time()
    {
        // Allocate all of the controllers
        for (int psmove_id = 0; psmove_id < k_max_psmove_controllers; ++psmove_id)
        {
            PSMoveControllerPtr controller = PSMoveControllerPtr(new PSMoveController(psmove_id));

            m_controllers[psmove_id]= controller;
        }
    }

    virtual ~ControllerManagerImpl()
    {
        // Deallocate the controllers
        for (int psmove_id = 0; psmove_id < k_max_psmove_controllers; ++psmove_id)
        {
            m_controllers[psmove_id]= PSMoveControllerPtr();
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
        boost::posix_time::time_duration poll_diff = now - m_last_poll_time;
        if (poll_diff.total_milliseconds() >= m_config->controller_poll_interval)
        {
            poll_open_controllers();
            m_last_poll_time= now;
        }

        // See if it's time to try update the list of connected controllers
        boost::posix_time::time_duration reconnect_diff = now - m_last_reconnect_time;
        if (reconnect_diff.total_milliseconds() >= m_config->controller_reconnect_interval)
        {
            // This method tries make the list of open controllers in m_controllers match 
            // the list of connected controller devices in the device enumerator.
            // Not controller objects are created or destroyed.
            // Pointers are just shuffled around and controllers opened and closed.
            update_connected_controllers();
            m_last_reconnect_time= now;
        }
    }

    void shutdown()
    {
        m_config->save();

        // Close any controllers that were opened
        for (int psmove_id = 0; psmove_id < k_max_psmove_controllers; ++psmove_id)
        {
            if (m_controllers[psmove_id]->getIsOpen())
            {
                m_controllers[psmove_id]->close();
            }
        }

        // Shutdown HIDAPI
        hid_exit();
    }

    bool setControllerRumble(int psmove_id, int rumble_amount)
    {
        return false;
    }

    bool resetPose(int psmove_id)
    {
        return false;
    }

protected:
    void poll_open_controllers()
    {
        for (int psmove_id = 0; psmove_id < k_max_psmove_controllers; ++psmove_id)
        {
            PSMoveControllerPtr &controller= m_controllers[psmove_id];

            if (controller->getIsOpen())
            {
                switch (controller->readDataIn())
                {
                case PSMoveController::_ReadDataResultSuccessNoData:
                    {
                        //###bwalker $TODO - Close controllers we get not data from in a while
                    }
                    break;
                case PSMoveController::_ReadDataResultSuccessNewData:
                    {
                        publish_controller_data_frame(controller);
                    }
                    break;
                case PSMoveController::_ReadDataResultFailure:
                    {
                        SERVER_LOG_INFO("ControllerManagerImpl::poll_open_controllers") << 
                            "Controller psmove_id " << psmove_id << " closing due to failed read";
                        controller->close();
                        //###bwalker $TODO - Send notification to the client?
                    }
                    break;
                }                
            }
        }

    }

    void update_connected_controllers()
    {
        PSMoveControllerPtr temp_controllers_list[k_max_psmove_controllers];

        // Step 1
        // See if any controllers shuffled order OR if any new controllers were attached.
        // Migrate open controllers to a new temp list in the order
        // that they appear in the device enumerator.
        int new_psmove_id = 0;
        for (PSMoveDeviceEnumerator enumerator; enumerator.is_valid(); enumerator.next())
        {
            // Find controller index for the controller with the matching device path
            int psmove_id= find_open_controller_psmove_id(enumerator);

            // Existing controller case (Most common)
            if (psmove_id != -1)
            {
                // Fetch the controller from it's existing controller slot
                PSMoveControllerPtr existingController= m_controllers[psmove_id];

                // See if an open controller changed order
                if (new_psmove_id != psmove_id)
                {
                    // Update the psmove_id on the controller
                    existingController->setPSMoveID(static_cast<int>(new_psmove_id));
                    
                    SERVER_LOG_INFO("ControllerManagerImpl::reconnect_controllers") << 
                        "Controller psmove_id " << psmove_id << " moved to psmove_id " << new_psmove_id;
                    //###bwalker $TODO - Send notification to the client?
                }

                // Move it to the new slot
                temp_controllers_list[new_psmove_id]= existingController;

                // Remove it from the previous list
                m_controllers[psmove_id]= PSMoveControllerPtr();
            }
            // New controller connected case
            else
            {
                int psmove_id= find_first_closed_controller_psmove_id();

                if (psmove_id != -1)
                {
                    // Fetch the controller from it's existing controller slot
                    PSMoveControllerPtr existingController= m_controllers[psmove_id];

                    // Move it to the new slot
                    existingController->setPSMoveID(static_cast<int>(new_psmove_id));
                    temp_controllers_list[new_psmove_id]= existingController;

                    // Remove it from the previous list
                    m_controllers[psmove_id]= PSMoveControllerPtr();

                    // Attempt to open the controller
                    if (existingController->open(enumerator))
                    {
                        SERVER_LOG_INFO("ControllerManagerImpl::reconnect_controllers") << 
                            "Controller psmove_id " << psmove_id << " connected";
                        //###bwalker $TODO - Send notification to the client?
                    }
                }
                else
                {
                    SERVER_LOG_ERROR("ControllerManagerImpl::reconnect_controllers") << "Can't connect any more new controllers. Too many open controllers";
                    break;
                }
            }

            ++new_psmove_id;
        }

        // Step 2
        // Close any remaining open controllers not listed in the device enumerator.
        // Copy over any closed controllers to the temp.
        for (int existing_psmove_id = 0; existing_psmove_id < k_max_psmove_controllers; ++existing_psmove_id)
        {
            PSMoveControllerPtr &existingController= m_controllers[existing_psmove_id];

            if (existingController)
            {
                // Any "open" controllers remaining in the old list need to be closed
                // since they no longer appear in the connected device list.
                // This probably shouldn't happen very often (at all?) as polling should catch
                // disconnected controllers first.
                if (existingController->getIsOpen())
                {
                    SERVER_LOG_WARNING("ControllerManagerImpl::reconnect_controllers") << "Closing controller " 
                        << existing_psmove_id << " since it's no longer in the device list.";
                    existingController->close();
                    //###bwalker $TODO - Send notification to the client?
                }

                // Move it to the new slot
                existingController->setPSMoveID(static_cast<int>(new_psmove_id));
                temp_controllers_list[new_psmove_id]= existingController;
                ++new_psmove_id;

                // Remove it from the previous list
                m_controllers[existing_psmove_id]= PSMoveControllerPtr();
            }
        }

        // Step 3
        // Copy the temp controller list back over top the original list
        for (int psmove_id = 0; psmove_id < k_max_psmove_controllers; ++psmove_id)
        {
            m_controllers[psmove_id]= temp_controllers_list[psmove_id];
        }
    }
    
    int find_open_controller_psmove_id(const PSMoveDeviceEnumerator &enumerator)
    {
        int result_psmove_id= -1;

        for (int psmove_id = 0; psmove_id < k_max_psmove_controllers; ++psmove_id)
        {
            PSMoveControllerPtr &controller= m_controllers[psmove_id];

            if (controller && controller->getIsOpen() && controller->matchesDeviceEnumerator(enumerator))
            {
                result_psmove_id= psmove_id;
                break;
            }
        }
        
        return result_psmove_id;
    }

    int find_first_closed_controller_psmove_id()
    {
        int result_psmove_id= -1;

        for (int psmove_id = 0; psmove_id < k_max_psmove_controllers; ++psmove_id)
        {
            PSMoveControllerPtr &controller= m_controllers[psmove_id];

            if (controller && !controller->getIsOpen())
            {
                result_psmove_id= psmove_id;
                break;
            }
        }
        
        return result_psmove_id;
    }

    void publish_controller_data_frame(
        PSMoveControllerPtr controller)
    {
        psmovePosef controller_pose= controller->getPose();
        PSMoveState controller_state= controller->getState();

        //###bwalker $TODO This is a hacky way to simulate controller data frame updates
        ControllerDataFramePtr data_frame(new PSMoveProtocol::ControllerDataFrame);

        //data_frame->set_psmove_id(controller->getPSMoveID());
        data_frame->set_psmove_id(controller->getPSMoveID());
        data_frame->set_sequence_num(m_sequence_number);
        m_sequence_number++;
        
        data_frame->set_isconnected(true);
        data_frame->set_iscurrentlytracking(false);
        data_frame->set_istrackingenabled(true);

        data_frame->mutable_orientation()->set_w(controller_pose.qw);
        data_frame->mutable_orientation()->set_x(controller_pose.qx);
        data_frame->mutable_orientation()->set_y(controller_pose.qy);
        data_frame->mutable_orientation()->set_z(controller_pose.qz);

        data_frame->mutable_position()->set_x(controller_pose.px);
        data_frame->mutable_position()->set_y(controller_pose.py);
        data_frame->mutable_position()->set_z(controller_pose.pz);

        unsigned int button_bitmask= 0;
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::TRIANGLE, controller_state.Triangle);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::CIRCLE, controller_state.Circle);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::CROSS, controller_state.Cross);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::SQUARE, controller_state.Square);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::SELECT, controller_state.Select);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::START, controller_state.Start);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::PS, controller_state.PS);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::MOVE, controller_state.Move);
        data_frame->set_button_down_bitmask(0);

        data_frame->set_trigger_value(controller_state.Trigger);

        ServerRequestHandler::get_instance()->publish_controller_data_frame(data_frame);
    }

private:
    ControllerManagerConfigPtr m_config;
    PSMoveControllerPtr m_controllers[k_max_psmove_controllers];
    int m_sequence_number;
    boost::posix_time::ptime m_last_reconnect_time;
    boost::posix_time::ptime m_last_poll_time;
};

//-- public interface -----
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
    return m_implementation_ptr->startup();
}

void ControllerManager::update()
{
    m_implementation_ptr->update();
}

void ControllerManager::shutdown()
{
    m_implementation_ptr->shutdown();
}

bool ControllerManager::setControllerRumble(int psmove_id, int rumble_amount)
{
    return m_implementation_ptr->setControllerRumble(psmove_id, rumble_amount);
}

bool ControllerManager::resetPose(int psmove_id)
{
    return m_implementation_ptr->resetPose(psmove_id);
}
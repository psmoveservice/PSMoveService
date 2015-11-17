//-- includes -----
#include "ControllerManager.h"
#include "ServerRequestHandler.h"
#include "PSMoveProtocol.pb.h"
#include "PSMoveController.h"
#include <boost/date_time/posix_time/posix_time_types.hpp>

//-- typedefs -----
typedef std::shared_ptr<PSMoveController> PSMoveControllerPtr;

//-- private implementation -----
class ControllerManagerImpl
{
public:
    ControllerManagerImpl()
        : m_sequence_number(0)
    {
    }

    bool startup()
    {
        return true;
    }

    void update()
    {        
        boost::posix_time::ptime now= boost::posix_time::second_clock::local_time();
        boost::posix_time::time_duration diff = now - m_last_publish_time;

        //###bwalker $TODO This is a hacky way to simulate controller data frame updates
        if (diff.total_milliseconds() >= 1000)
        {
            PSMoveControllerPtr controller;

            publish_controller_data_frame(controller);
            m_last_publish_time= now;
        }
    }

    void shutdown()
    {
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
    void publish_controller_data_frame(
        PSMoveControllerPtr controller)
    {
        //###bwalker $TODO This is a hacky way to simulate controller data frame updates
        ControllerDataFramePtr data_frame(new PSMoveProtocol::ControllerDataFrame);

        //data_frame->set_psmove_id(controller->getPSMoveID());
        data_frame->set_psmove_id(0);
        data_frame->set_sequence_num(m_sequence_number);
        m_sequence_number++;
        
        data_frame->set_isconnected(true);
        data_frame->set_iscurrentlytracking(true);
        data_frame->set_istrackingenabled(true);

        data_frame->mutable_orientation()->set_w(1.f);
        data_frame->mutable_orientation()->set_x(0.f);
        data_frame->mutable_orientation()->set_y(0.f);
        data_frame->mutable_orientation()->set_z(0.f);

        data_frame->mutable_position()->set_x(0.f);
        data_frame->mutable_position()->set_y(0.f);
        data_frame->mutable_position()->set_z(0.f);

        data_frame->set_button_down_bitmask(0);
        data_frame->set_trigger_value(0);

        ServerRequestHandler::get_instance()->publish_controller_data_frame(data_frame);
    }

private:
    int m_sequence_number;

    //###bwalker $TODO - temp for pushing fake controller data
    boost::posix_time::ptime m_last_publish_time;
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
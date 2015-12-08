//-- includes -----
#include "ServerControllerView.h"
#include "ServerLog.h"
#include "ServerRequestHandler.h"
#include "ControllerInterface.h"
#include "PSMoveController/PSMoveController.h"
#include "PSNaviController/PSNaviController.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include "ServerUtility.h"

#include <chrono>

//-- macros -----
#define SET_BUTTON_BIT(bitmask, bit_index, button_state) \
    bitmask|= (button_state == CommonControllerState::Button_DOWN || button_state == CommonControllerState::Button_PRESSED) ? (0x1 << (bit_index)) : 0x0;

//-- public implementation -----
ServerControllerView::ServerControllerView(
    const int controller_id)
    : m_controllerID(controller_id)
    , m_sequence_number(0)
    , m_last_updated_tick(0)
    , m_controller(nullptr)
{
    m_controllerID= controller_id;
    m_controller = new PSMoveController();
}

ServerControllerView::~ServerControllerView()
{
    delete m_controller;
}

bool ServerControllerView::matchesDeviceEnumerator(
    const ControllerDeviceEnumerator *enumerator) const
{
    return m_controller->matchesDeviceEnumerator(enumerator);
}

bool ServerControllerView::open(
    const ControllerDeviceEnumerator *enumerator)
{
    bool bSuccess= m_controller->open(enumerator);

    if (bSuccess)
    {
        // Consider a successful opening as an update
        m_last_updated_tick= 
            std::chrono::duration_cast< std::chrono::milliseconds >(
                    std::chrono::system_clock::now().time_since_epoch()).count();
    }

    return bSuccess;
}

void ServerControllerView::close()
{
    m_controller->close();
}

bool ServerControllerView::update()
{
    bool bSuccessfullyUpdated= true;

    // Only poll data from open, bluetooth controllers
    if (m_controller->getIsOpen() && m_controller->getIsBluetooth())
    {
        switch (m_controller->poll())
        {
        case IControllerInterface::_PollResultSuccessNoData:
            {
                long long now = 
                    std::chrono::duration_cast< std::chrono::milliseconds >(
                        std::chrono::system_clock::now().time_since_epoch()).count();
                long diff= static_cast<long>(now - m_last_updated_tick);
                long max_timeout= m_controller->getDataTimeout();

                if (diff > max_timeout)
                {
                    SERVER_LOG_INFO("ServerControllerView::poll_open_controllers") << 
                        "Controller id " << m_controllerID << " closing due to no data timeout (" << max_timeout << "ms)";
                    m_controller->close();

                    bSuccessfullyUpdated= false;
                }
            }
            break;
        case IControllerInterface::_PollResultSuccessNewData:
            {
                m_last_updated_tick= 
                    std::chrono::duration_cast< std::chrono::milliseconds >(
                            std::chrono::system_clock::now().time_since_epoch()).count();

                publish_controller_data_frame();

                bSuccessfullyUpdated= true;
            }
            break;
        case IControllerInterface::_PollResultFailure:
            {
                SERVER_LOG_INFO("ServerControllerView::poll_open_controllers") << 
                    "Controller id " << m_controllerID << " closing due to failed read";
                m_controller->close();

                bSuccessfullyUpdated= false;
            }
            break;
        }                
    }

    return bSuccessfullyUpdated;
}

psmovePosef
ServerControllerView::getPose(int msec_time) const
{
    psmovePosef nullPose;

    nullPose.clear();
    return nullPose;
}

bool 
ServerControllerView::getIsBluetooth() const
{
    return m_controller->getIsBluetooth();
}

// Returns the full usb device path for the controller
std::string 
ServerControllerView::getUSBDevicePath() const
{
    return m_controller->getUSBDevicePath();
}

// Returns the serial number for the controller
std::string 
ServerControllerView::getSerial() const
{
    return m_controller->getSerial();
}

bool 
ServerControllerView::getIsOpen() const
{
    return m_controller->getIsOpen();
}

CommonControllerState::eControllerDeviceType 
ServerControllerView::getControllerDeviceType() const
{
    return m_controller->getControllerDeviceType();
}

// Fetch the controller state at the given sample index.
// A lookBack of 0 corresponds to the most recent data.
void ServerControllerView::getState(
    struct CommonControllerState *out_state, int lookBack) const
{
    m_controller->getState(out_state, lookBack);
}

// Set the rumble value between 0-255
bool ServerControllerView::setControllerRumble(int rumble_amount)
{
    bool result= false;

    if (getIsOpen())
    {
        switch(getControllerDeviceType())
        {
            case CommonControllerState::PSMove:
            {
                unsigned char rumble_byte= ServerUtility::int32_to_int8_verify(rumble_amount);
                static_cast<PSMoveController *>(m_controller)->setRumbleIntensity(rumble_byte);
            } break;

            case CommonControllerState::PSNavi:
            {
                result= false; // No rumble on the navi
            } break;

            default:
                assert(false && "Unhanded controller type!");
        }
    }

    return result;
}

// -- private methods -----
void ServerControllerView::publish_controller_data_frame()
{
    psmovePosef controller_pose= getPose();    

    ControllerDataFramePtr data_frame(new PSMoveProtocol::ControllerDataFrame);

    data_frame->set_controller_id(getControllerID());
    data_frame->set_sequence_num(m_sequence_number);
    m_sequence_number++;
    
    data_frame->set_isconnected(m_controller->getIsOpen());

    switch (getControllerDeviceType())
    {
    case CommonControllerState::PSMove:
        {
            PSMoveControllerState psmove_state;
            PSMoveProtocol::ControllerDataFrame_PSMoveState *psmove_data_frame= data_frame->mutable_psmove_state();

            m_controller->getState(&psmove_state);

            //###bwalker $TODO - Publish real tracking status
            psmove_data_frame->set_iscurrentlytracking(false);
            psmove_data_frame->set_istrackingenabled(true);

            psmove_data_frame->mutable_orientation()->set_w(controller_pose.qw);
            psmove_data_frame->mutable_orientation()->set_x(controller_pose.qx);
            psmove_data_frame->mutable_orientation()->set_y(controller_pose.qy);
            psmove_data_frame->mutable_orientation()->set_z(controller_pose.qz);

            psmove_data_frame->mutable_position()->set_x(controller_pose.px);
            psmove_data_frame->mutable_position()->set_y(controller_pose.py);
            psmove_data_frame->mutable_position()->set_z(controller_pose.pz);

            psmove_data_frame->set_trigger_value(psmove_state.Trigger);

            unsigned int button_bitmask= 0;
            SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::TRIANGLE, psmove_state.Triangle);
            SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::CIRCLE, psmove_state.Circle);
            SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::CROSS, psmove_state.Cross);
            SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::SQUARE, psmove_state.Square);
            SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::SELECT, psmove_state.Select);
            SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::START, psmove_state.Start);
            SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::PS, psmove_state.PS);
            SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::MOVE, psmove_state.Move);
            data_frame->set_button_down_bitmask(button_bitmask);

            data_frame->set_controller_type(PSMoveProtocol::PSMOVE);
        } break;
    case CommonControllerState::PSNavi:
        {
            PSNaviControllerState psnavi_state;
            PSMoveProtocol::ControllerDataFrame_PSNaviState *psnavi_data_frame= data_frame->mutable_psnavi_state();

            m_controller->getState(&psnavi_state);

            psnavi_data_frame->set_trigger_value(psnavi_state.Trigger);
            psnavi_data_frame->set_stick_xaxis(psnavi_state.Stick_XAxis);
            psnavi_data_frame->set_stick_yaxis(psnavi_state.Stick_YAxis);

            unsigned int button_bitmask= 0;
            SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::L1, psnavi_state.L1);
            SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::L2, psnavi_state.L2);
            SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::L3, psnavi_state.L3);
            SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::CIRCLE, psnavi_state.Circle);
            SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::CROSS, psnavi_state.Cross);
            SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::PS, psnavi_state.PS);
            SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::UP, psnavi_state.DPad_Up);
            SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::RIGHT, psnavi_state.DPad_Right);
            SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::DOWN, psnavi_state.DPad_Down);
            SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::LEFT, psnavi_state.DPad_Left);
            data_frame->set_button_down_bitmask(button_bitmask);

            data_frame->set_controller_type(PSMoveProtocol::PSNAVI);
        } break;
    default:
        assert(0 && "Unhandled controller type");
    }
    
    ServerRequestHandler::get_instance()->publish_controller_data_frame(data_frame);
}
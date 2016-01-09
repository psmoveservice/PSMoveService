//-- includes -----
#include "ServerDeviceView.h"

#include "BluetoothRequests.h"
#include "ServerLog.h"
#include "ServerRequestHandler.h"
#include "DeviceInterface.h"
#include "PSMoveController/PSMoveController.h"
#include "PSNaviController/PSNaviController.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include "ServerUtility.h"

#include <chrono>

//-- macros -----
#define SET_BUTTON_BIT(bitmask, bit_index, button_state) \
    bitmask|= (button_state == CommonControllerState::Button_DOWN || button_state == CommonControllerState::Button_PRESSED) ? (0x1 << (bit_index)) : 0x0;

//-- private methods -----
static void generate_psmove_data_frame_for_stream(
    const ServerControllerView *controller_view, const ControllerStreamInfo *stream_info, ControllerDataFramePtr &data_frame);
static void generate_psnavi_data_frame_for_stream(
    const ServerControllerView *controller_view, const ControllerStreamInfo *stream_info, ControllerDataFramePtr &data_frame);

//-- public implementation -----
ServerDeviceView::ServerDeviceView(
    const int device_id)
    : m_last_updated_tick(0)
    , m_sequence_number(0)
    , m_deviceID(device_id)
{
}

ServerDeviceView::~ServerDeviceView()
{
}

bool
ServerDeviceView::open(const DeviceEnumerator *enumerator)
{
    bool bSuccess= getDevice()->open(enumerator);
    
    if (bSuccess)
    {
        // Consider a successful opening as an update
        m_last_updated_tick=
        std::chrono::duration_cast< std::chrono::milliseconds >(
                                                                std::chrono::system_clock::now().time_since_epoch()).count();
    }
    return bSuccess;
}

bool
ServerDeviceView::getIsOpen() const
{
    return getDevice()->getIsOpen();
}

bool ServerDeviceView::update()
{
    bool bSuccessfullyUpdated= true;
    
    IDeviceInterface* device = getDevice();
    // Only poll data from open, bluetooth controllers
    if (device->getIsReadyToPoll())
    {
        switch (device->poll())
        {
            case IControllerInterface::_PollResultSuccessNoData:
            {
                long long now =
                std::chrono::duration_cast< std::chrono::milliseconds >(
                                                                        std::chrono::system_clock::now().time_since_epoch()).count();
                long diff= static_cast<long>(now - m_last_updated_tick);
                long max_timeout= device->getDataTimeout();
                
                if (diff > max_timeout)
                {
                    SERVER_LOG_INFO("ServerControllerView::poll_open_controllers") <<
                    "Controller id " << getDeviceID() << " closing due to no data timeout (" << max_timeout << "ms)";
                    device->close();
                    
                    bSuccessfullyUpdated= false;
                }
            }
                break;
                
            case IControllerInterface::_PollResultSuccessNewData:
            {
                m_last_updated_tick=
                std::chrono::duration_cast< std::chrono::milliseconds >(
                                                                        std::chrono::system_clock::now().time_since_epoch()).count();
                publish_device_data_frame();
                
                bSuccessfullyUpdated= true;
            }
                break;
                
            case IControllerInterface::_PollResultFailure:
            {
                SERVER_LOG_INFO("ServerControllerView::poll_open_controllers") <<
                "Controller id " << getDeviceID() << " closing due to failed read";
                device->close();
                
                bSuccessfullyUpdated= false;
            }
                break;
        }
    }
    
    return bSuccessfullyUpdated;
}

void
ServerDeviceView::close()
{
    getDevice()->close();
}

bool
ServerDeviceView::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    return getDevice()->matchesDeviceEnumerator(enumerator);
}


// -- Controller View -----

ServerControllerView::ServerControllerView(const int device_id)
    : ServerDeviceView(device_id)
    , m_device(nullptr)
{
    m_device = new PSMoveController();
}

ServerControllerView::~ServerControllerView()
{
    delete m_device;  // Deleting abstract object should be OK because
                      // this (ServerDeviceView) is abstract as well.
                      // All non-abstract children will have non-abstract types
                      // for m_device.
}

bool ServerControllerView::setHostBluetoothAddress(
    const std::string &address)
{
    return m_device->setHostBluetoothAddress(address);
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
    return m_device->getIsBluetooth();
}

// Returns the full usb device path for the controller
std::string 
ServerControllerView::getUSBDevicePath() const
{
    return m_device->getUSBDevicePath();
}

// Returns the serial number for the controller
std::string 
ServerControllerView::getSerial() const
{
    return m_device->getSerial();
}

std::string 
ServerControllerView::getHostBluetoothAddress() const
{
    return m_device->getHostBluetoothAddress();
}

CommonDeviceState::eDeviceType
ServerControllerView::getControllerDeviceType() const
{
    return m_device->getDeviceType();
}

// Fetch the controller state at the given sample index.
// A lookBack of 0 corresponds to the most recent data.
void ServerControllerView::getState(
    struct CommonControllerState *out_state, int lookBack) const
{
    m_device->getState(out_state, lookBack);
}

// Set the rumble value between 0-255
bool ServerControllerView::setControllerRumble(int rumble_amount)
{
    bool result= false;

    if (getIsOpen())
    {
        switch(getControllerDeviceType())
        {
            case CommonDeviceState::PSMove:
            {
                unsigned char rumble_byte= ServerUtility::int32_to_int8_verify(rumble_amount);
                static_cast<PSMoveController *>(m_device)->setRumbleIntensity(rumble_byte);
            } break;

            case CommonDeviceState::PSNavi:
            {
                result= false; // No rumble on the navi
            } break;

            default:
                assert(false && "Unhanded controller type!");
        }
    }

    return result;
}

void ServerControllerView::publish_device_data_frame()
{
    // Tell the server request handler we want to send out controller updates.
    // This will call generate_controller_data_frame_for_stream for each listening connection.
    ServerRequestHandler::get_instance()->publish_controller_data_frame(
        this, &ServerControllerView::generate_controller_data_frame_for_stream);

    m_sequence_number++;
}

void ServerControllerView::generate_controller_data_frame_for_stream(
    const ServerControllerView *controller_view,
    const ControllerStreamInfo *stream_info,
    ControllerDataFramePtr &data_frame)
{
    data_frame->set_controller_id(controller_view->getDeviceID());
    data_frame->set_sequence_num(controller_view->m_sequence_number);   
    data_frame->set_isconnected(controller_view->getDevice()->getIsOpen());

    switch (controller_view->getControllerDeviceType())
    {
    case CommonControllerState::PSMove:
        {
            generate_psmove_data_frame_for_stream(controller_view, stream_info, data_frame);
        } break;
    case CommonControllerState::PSNavi:
        {
            generate_psnavi_data_frame_for_stream(controller_view, stream_info, data_frame);
        } break;
    default:
        assert(0 && "Unhandled controller type");
    }
}

static void generate_psmove_data_frame_for_stream(
    const ServerControllerView *controller_view,
    const ControllerStreamInfo *stream_info,
    ControllerDataFramePtr &data_frame)
{
    const PSMoveController *psmove_controller= controller_view->castCheckedConst<PSMoveController>();
    const PSMoveControllerConfig &psmove_config= psmove_controller->getConfig();

    PSMoveProtocol::ControllerDataFrame_PSMoveState *psmove_data_frame= data_frame->mutable_psmove_state();

    psmovePosef controller_pose= controller_view->getPose();

    PSMoveControllerState psmove_state;
    controller_view->getState(&psmove_state);

    psmove_data_frame->set_validhardwarecalibration(psmove_config.is_valid);
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

    // If requested, get the raw sensor data for the controller
    if (stream_info->include_raw_sensor_data)
    {
        PSMoveProtocol::ControllerDataFrame_PSMoveState_RawSensorData *raw_sensor_data=
            psmove_data_frame->mutable_raw_sensor_data();

        // One frame: [mx, my, mz] 
        assert(psmove_state.Mag.size() == 3);
        raw_sensor_data->mutable_magnetometer()->set_i(psmove_state.Mag[0]);
        raw_sensor_data->mutable_magnetometer()->set_j(psmove_state.Mag[1]);
        raw_sensor_data->mutable_magnetometer()->set_k(psmove_state.Mag[2]);

        // Two frames: [[ax0, ay0, az0], [ax1, ay1, az1]] 
        // Take the most recent frame: [ax1, ay1, az1]
        assert(psmove_state.Accel.size() == 2);
        assert(psmove_state.Accel[0].size() == 3);
        assert(psmove_state.Accel[1].size() == 3);
        raw_sensor_data->mutable_accelerometer()->set_i(psmove_state.Accel[1][0]);
        raw_sensor_data->mutable_accelerometer()->set_j(psmove_state.Accel[1][1]);
        raw_sensor_data->mutable_accelerometer()->set_k(psmove_state.Accel[1][2]);

        // Two frames: [[wx0, wy0, wz0], [wx1, wy1, wz1]] 
        // Take the most recent frame: [wx1, wy1, wz1]
        assert(psmove_state.Gyro.size() == 2);
        assert(psmove_state.Gyro[0].size() == 3);
        assert(psmove_state.Gyro[1].size() == 3);
        raw_sensor_data->mutable_gyroscope()->set_i(psmove_state.Gyro[1][0]);
        raw_sensor_data->mutable_gyroscope()->set_j(psmove_state.Gyro[1][1]);
        raw_sensor_data->mutable_gyroscope()->set_k(psmove_state.Gyro[1][2]);
    }

    data_frame->set_controller_type(PSMoveProtocol::PSMOVE);
}

static void generate_psnavi_data_frame_for_stream(
    const ServerControllerView *controller_view,
    const ControllerStreamInfo *stream_info,
    ControllerDataFramePtr &data_frame)
{
    PSMoveProtocol::ControllerDataFrame_PSNaviState *psnavi_data_frame= data_frame->mutable_psnavi_state();

    PSNaviControllerState psnavi_state;
    controller_view->getState(&psnavi_state);

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
}

// -- Tracker View -----

ServerTrackerView::ServerTrackerView(const int device_id)
: ServerDeviceView(device_id)
, m_device(nullptr)
{
    //TODO: new PSMoveTracker();
    m_device = new PSMoveController();
}

ServerTrackerView::~ServerTrackerView()
{
    delete m_device;
}

// -- HMD View -----
ServerHMDView::ServerHMDView(const int device_id)
: ServerDeviceView(device_id)
, m_device(nullptr)
{
    //TODO: new HMD();
    m_device = new PSMoveController();
}

ServerHMDView::~ServerHMDView()
{
    delete m_device;
}
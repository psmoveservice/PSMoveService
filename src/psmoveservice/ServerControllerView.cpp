//-- includes -----
#include "ServerControllerView.h"

#include "BluetoothRequests.h"
#include "MathAlignment.h"
#include "ServerLog.h"
#include "ServerRequestHandler.h"
#include "Filter/OrientationFilter.h"
#include "PSMoveController/PSMoveController.h"
#include "PSNaviController/PSNaviController.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include "ServerUtility.h"

//-- constants -----

//-- macros -----
#define SET_BUTTON_BIT(bitmask, bit_index, button_state) \
    bitmask|= (button_state == CommonControllerState::Button_DOWN || button_state == CommonControllerState::Button_PRESSED) ? (0x1 << (bit_index)) : 0x0;

//-- private methods -----
static void init_orientation_filter_for_psmove(
    const PSMoveController *psmoveController, OrientationFilter *orientation_filter);
static void update_orientation_filter_for_psmove(
    const PSMoveController *psmoveController, const PSMoveControllerState *psmoveState, OrientationFilter *orientationFilter);

static void generate_psmove_data_frame_for_stream(
    const ServerControllerView *controller_view, const ControllerStreamInfo *stream_info, ControllerDataFramePtr &data_frame);
static void generate_psnavi_data_frame_for_stream(
    const ServerControllerView *controller_view, const ControllerStreamInfo *stream_info, ControllerDataFramePtr &data_frame);

//-- public implementation -----
ServerControllerView::ServerControllerView(const int device_id)
    : ServerDeviceView(device_id)
    , m_device(nullptr)
    , m_orientation_filter(nullptr)
    , m_lastPollSeqNumProcessed(-1)
{
}

ServerControllerView::~ServerControllerView()
{
}

bool ServerControllerView::allocate_device_interface(
    const class DeviceEnumerator *enumerator)
{
    switch (enumerator->get_device_type())
    {
    case CommonDeviceState::PSMove:
        {
            m_device = new PSMoveController();
            m_orientation_filter = new OrientationFilter();
        } break;
    case CommonDeviceState::PSNavi:
        {
            m_device= new PSNaviController();
            m_orientation_filter= nullptr;
        } break;
    default:
        break;
    }

    return m_device != nullptr;
}

void ServerControllerView::free_device_interface()
{
    if (m_orientation_filter != nullptr)
    {
        delete m_orientation_filter;
        m_orientation_filter= nullptr;
    }

    if (m_device != nullptr)
    {
        delete m_device;  // Deleting abstract object should be OK because
                          // this (ServerDeviceView) is abstract as well.
                          // All non-abstract children will have non-abstract types
                          // for m_device.
        m_device= nullptr;
    }
}

bool ServerControllerView::open(const class DeviceEnumerator *enumerator)
{
    // Attempt to open the controller
    bool bSuccess= ServerDeviceView::open(enumerator);

    // Setup the orientation filter based on the controller configuration
    if (bSuccess)
    {
        IDeviceInterface *device= getDevice();

        switch (device->getDeviceType())
        {
        case CommonDeviceState::PSMove:
            {
                const PSMoveController *psmoveController= this->castCheckedConst<PSMoveController>();

                init_orientation_filter_for_psmove(psmoveController, m_orientation_filter);
            } break;
        case CommonDeviceState::PSNavi:
            // No orientation filter for the navi
            assert(m_orientation_filter == nullptr);
            break;
        default:
            break;
        }

        // Reset the poll sequence number high water mark
        m_lastPollSeqNumProcessed= -1;
    }

    //###bwalker $TODO Setup the position filter based on the controller configuration

    return bSuccess;
}

void ServerControllerView::updateStateAndPredict()
{
    if (!getHasUnpublishedState())
    {
        return;
    }

    // Look backward in time to find the first controller update state with a poll sequence number 
    // newer than the last sequence number we've processed.
    int firstLookBack = -1;
    int testLookBack = 0;
    const CommonControllerState *state= getState(testLookBack);
    while (state != nullptr && state->PollSequenceNumber > m_lastPollSeqNumProcessed)
    {
        firstLookBack= testLookBack;
        testLookBack++;
        state= getState(testLookBack);
    }
    assert(firstLookBack >= 0);

    // Process the polled controller states forward in time
    // computing the new orientation along the way.
    for (int lookBack= firstLookBack; lookBack >= 0; --lookBack)
    {
        const CommonControllerState *controllerState= getState(lookBack);

        switch (controllerState->DeviceType)
        {
        case CommonControllerState::PSMove:
            {
                const PSMoveController *psmoveController= this->castCheckedConst<PSMoveController>();
                const PSMoveControllerState *psmoveState= static_cast<const PSMoveControllerState *>(controllerState);

                update_orientation_filter_for_psmove(psmoveController, psmoveState, m_orientation_filter);
            } break;
        case CommonControllerState::PSNavi:
            {
                // No orientation to update
                assert(m_orientation_filter == nullptr);
            } break;
        default:
            assert(0 && "Unhandled controller type");
        }

        // Consider this controller state sequence num processed
        m_lastPollSeqNumProcessed= controllerState->PollSequenceNumber;
    }

    // TODO: Process tracker updates and update the position filter
}

bool ServerControllerView::setHostBluetoothAddress(
    const std::string &address)
{
    return m_device->setHostBluetoothAddress(address);
}

psmovePosef
ServerControllerView::getPose(int msec_time) const
{
    psmovePosef pose;

    pose.clear();

    if (m_orientation_filter != nullptr)
    {
        Eigen::Quaternionf orientation= m_orientation_filter->getOrientation(msec_time);

        pose.qw= orientation.w();
        pose.qx= orientation.x();
        pose.qy= orientation.y();
        pose.qz= orientation.z();
    }

    //###bwalker $TODO - extract position
    //if (m_position_filter != nullptr)
    //{
    //    Eigen::Vector3f position= m_position_filter->getPosition(msec_time);

    //    pose.x= position.x();
    //    pose.y= position.y();
    //    pose.z= position.z();
    //}

    return pose;
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
const struct CommonControllerState * ServerControllerView::getState(
    int lookBack) const
{
    const struct CommonDeviceState *device_state= m_device->getState(lookBack);
    assert(device_state == nullptr ||
           (device_state->DeviceType >= CommonDeviceState::Controller && 
            device_state->DeviceType < CommonDeviceState::SUPPORTED_CONTROLLER_TYPE_COUNT));

    return static_cast<const CommonControllerState *>(device_state);
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
    const PSMoveControllerConfig *psmove_config= psmove_controller->getConfig();
    const CommonControllerState *controller_state= controller_view->getState();
    const psmovePosef controller_pose= controller_view->getPose();

    PSMoveProtocol::ControllerDataFrame_PSMoveState *psmove_data_frame= data_frame->mutable_psmove_state();
   
    if (controller_state != nullptr)
    {        
        assert(controller_state->DeviceType == CommonDeviceState::PSMove);
        const PSMoveControllerState * psmove_state= static_cast<const PSMoveControllerState *>(controller_state);

        psmove_data_frame->set_validhardwarecalibration(psmove_config->is_valid);
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

        psmove_data_frame->set_trigger_value(psmove_state->Trigger);

        unsigned int button_bitmask= 0;
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::TRIANGLE, psmove_state->Triangle);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::CIRCLE, psmove_state->Circle);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::CROSS, psmove_state->Cross);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::SQUARE, psmove_state->Square);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::SELECT, psmove_state->Select);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::START, psmove_state->Start);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::PS, psmove_state->PS);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::MOVE, psmove_state->Move);
        data_frame->set_button_down_bitmask(button_bitmask);

        // If requested, get the raw sensor data for the controller
        if (stream_info->include_raw_sensor_data)
        {
            PSMoveProtocol::ControllerDataFrame_PSMoveState_RawSensorData *raw_sensor_data=
                psmove_data_frame->mutable_raw_sensor_data();

            // One frame: [mx, my, mz] 
            assert(psmove_state->Mag.size() == 3);
            raw_sensor_data->mutable_magnetometer()->set_i(psmove_state->Mag[0]);
            raw_sensor_data->mutable_magnetometer()->set_j(psmove_state->Mag[1]);
            raw_sensor_data->mutable_magnetometer()->set_k(psmove_state->Mag[2]);

            // Two frames: [[ax0, ay0, az0], [ax1, ay1, az1]] 
            // Take the most recent frame: [ax1, ay1, az1]
            assert(psmove_state->Accel.size() == 2);
            assert(psmove_state->Accel[0].size() == 3);
            assert(psmove_state->Accel[1].size() == 3);
            raw_sensor_data->mutable_accelerometer()->set_i(psmove_state->Accel[1][0]);
            raw_sensor_data->mutable_accelerometer()->set_j(psmove_state->Accel[1][1]);
            raw_sensor_data->mutable_accelerometer()->set_k(psmove_state->Accel[1][2]);

            // Two frames: [[wx0, wy0, wz0], [wx1, wy1, wz1]] 
            // Take the most recent frame: [wx1, wy1, wz1]
            assert(psmove_state->Gyro.size() == 2);
            assert(psmove_state->Gyro[0].size() == 3);
            assert(psmove_state->Gyro[1].size() == 3);
            raw_sensor_data->mutable_gyroscope()->set_i(psmove_state->Gyro[1][0]);
            raw_sensor_data->mutable_gyroscope()->set_j(psmove_state->Gyro[1][1]);
            raw_sensor_data->mutable_gyroscope()->set_k(psmove_state->Gyro[1][2]);
        }
    }   

    data_frame->set_controller_type(PSMoveProtocol::PSMOVE);
}

static void generate_psnavi_data_frame_for_stream(
    const ServerControllerView *controller_view,
    const ControllerStreamInfo *stream_info,
    ControllerDataFramePtr &data_frame)
{
    PSMoveProtocol::ControllerDataFrame_PSNaviState *psnavi_data_frame= data_frame->mutable_psnavi_state();
    const CommonControllerState *controller_state= controller_view->getState();

    if (controller_state != nullptr)
    {
        assert(controller_state->DeviceType == CommonDeviceState::PSNavi);
        const PSNaviControllerState *psnavi_state= static_cast<const PSNaviControllerState *>(controller_state);

        psnavi_data_frame->set_trigger_value(psnavi_state->Trigger);
        psnavi_data_frame->set_stick_xaxis(psnavi_state->Stick_XAxis);
        psnavi_data_frame->set_stick_yaxis(psnavi_state->Stick_YAxis);

        unsigned int button_bitmask= 0;
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::L1, psnavi_state->L1);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::L2, psnavi_state->L2);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::L3, psnavi_state->L3);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::CIRCLE, psnavi_state->Circle);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::CROSS, psnavi_state->Cross);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::PS, psnavi_state->PS);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::UP, psnavi_state->DPad_Up);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::RIGHT, psnavi_state->DPad_Right);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::DOWN, psnavi_state->DPad_Down);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::ControllerDataFrame::LEFT, psnavi_state->DPad_Left);
        data_frame->set_button_down_bitmask(button_bitmask);
    }

    data_frame->set_controller_type(PSMoveProtocol::PSNAVI);
}

static void
init_orientation_filter_for_psmove(
    const PSMoveController *psmoveController, 
    OrientationFilter *orientation_filter)
{
    const PSMoveControllerConfig *psmove_config= psmoveController->getConfig();

    // Setup the space the orientation filter operates in
    Eigen::Vector3f identityGravity= Eigen::Vector3f(0.f, 1.f, 0.f);
    Eigen::Vector3f identityMagnetometer = psmove_config->magnetometer_identity;
    Eigen::Matrix3f calibrationTransform= *k_eigen_identity_pose_laying_flat;
    Eigen::Matrix3f sensorTransform= *k_eigen_sensor_transform_opengl;
    OrientationFilterSpace filterSpace(identityGravity, identityMagnetometer, calibrationTransform, sensorTransform);
                
    orientation_filter->setFilterSpace(filterSpace);

    // Use the complementary MARG fusion filter by default
    orientation_filter->setFusionType(OrientationFilter::FusionTypeComplementaryMARG);
}

static void                
update_orientation_filter_for_psmove(
    const PSMoveController *psmoveController, 
    const PSMoveControllerState *psmoveState,
    OrientationFilter *orientationFilter)
{
    const PSMoveControllerConfig *config= psmoveController->getConfig();

    //###bwalker $TODO Determine time deltas from the timestamps on the controller frames
    const float delta_time= 1.f / 120.f;

    OrientationSensorPacket sensorPacket;

    // Re-scale the magnetometer int-vector into a float vector in the range <-1,-1,-1> to <1,1,1>
    // using the min and max magnetometer extents stored in the controller config.
    {
        const Eigen::Vector3f sample= 
            Eigen::Vector3f(
                static_cast<float>(psmoveState->Mag[0]), 
                static_cast<float>(psmoveState->Mag[1]), 
                static_cast<float>(psmoveState->Mag[2]));

        // Project the averaged magnetometer sample into the space of the ellipse
        // And then normalize it (any deviation from unit length is error)
        sensorPacket.magnetometer =
            eigen_alignment_project_point_on_ellipsoid_basis(sample, config->magnetometer_ellipsoid);
        eigen_vector3f_normalize_with_default(sensorPacket.magnetometer, Eigen::Vector3f(0.f, 1.f, 0.f));
    }

    // Each state update contains two readings (one earlier and one later) of accelerometer and gyro data
    for (int frame=0; frame < 2; ++frame)
    {
        sensorPacket.accelerometer= 
            Eigen::Vector3f(psmoveState->Accel[frame][0], psmoveState->Accel[frame][1], psmoveState->Accel[frame][2]);
        sensorPacket.gyroscope= 
            Eigen::Vector3f(psmoveState->Gyro[frame][0], psmoveState->Gyro[frame][1], psmoveState->Gyro[frame][2]);

        // Update the orientation filter using the sensor packet.
        // NOTE: The magnetometer reading is the same for both sensor readings.
        orientationFilter->update(delta_time, sensorPacket);
    }
}
//-- includes -----
#include "ServerControllerView.h"

#include "BluetoothRequests.h"
#include "ControllerManager.h"
#include "DeviceManager.h"
#include "MathAlignment.h"
#include "ServerLog.h"
#include "ServerRequestHandler.h"
#include "OrientationFilter.h"
#include "PositionFilter.h"
#include "PSMoveController.h"
#include "PSNaviController.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include "ServerUtility.h"
#include "ServerTrackerView.h"

#include <glm/glm.hpp>

//-- constants -----

//-- macros -----
#define SET_BUTTON_BIT(bitmask, bit_index, button_state) \
    bitmask|= (button_state == CommonControllerState::Button_DOWN || button_state == CommonControllerState::Button_PRESSED) ? (0x1 << (bit_index)) : 0x0;

//-- private methods -----
static void init_filters_for_psmove(
    const PSMoveController *psmoveController, 
    OrientationFilter *orientation_filter, PositionFilter *position_filter);
static void update_filters_for_psmove(
    const PSMoveController *psmoveController, const PSMoveControllerState *psmoveState, 
    OrientationFilter *orientationFilter, PositionFilter *position_filter);

static void generate_psmove_data_frame_for_stream(
    const ServerControllerView *controller_view, const ControllerStreamInfo *stream_info, DeviceDataFramePtr &data_frame);
static void generate_psnavi_data_frame_for_stream(
    const ServerControllerView *controller_view, const ControllerStreamInfo *stream_info, DeviceDataFramePtr &data_frame);

//-- public implementation -----
ServerControllerView::ServerControllerView(const int device_id)
    : ServerDeviceView(device_id)
    , m_tracking_color_id(eCommonTrackingColorID::INVALID)
    , m_device(nullptr)
    , m_orientation_filter(nullptr)
    , m_position_filter(nullptr)
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
            m_position_filter = new PositionFilter();
        } break;
    case CommonDeviceState::PSNavi:
        {
            m_device= new PSNaviController();
            m_orientation_filter= nullptr;
            m_position_filter = nullptr;
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

    if (m_position_filter != nullptr)
    {
        delete m_position_filter;
        m_position_filter = nullptr;
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
    bool bAllocateTrackingColor = false;

    // Setup the orientation filter based on the controller configuration
    if (bSuccess)
    {
        IDeviceInterface *device= getDevice();

        switch (device->getDeviceType())
        {
        case CommonDeviceState::PSMove:
            {
                const PSMoveController *psmoveController= this->castCheckedConst<PSMoveController>();

                init_filters_for_psmove(psmoveController, m_orientation_filter, m_position_filter);
                bAllocateTrackingColor = true;
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

    // If needed for this kind of controller, assign a tracking color id
    if (bAllocateTrackingColor)
    {
        assert(m_tracking_color_id == eCommonTrackingColorID::INVALID);
        m_tracking_color_id= DeviceManager::getInstance()->m_controller_manager->allocateTrackingColorID();
    }

    return bSuccess;
}

void ServerControllerView::close()
{
    ServerDeviceView::close();

    if (m_tracking_color_id != eCommonTrackingColorID::INVALID)
    {
        DeviceManager::getInstance()->m_controller_manager->freeTrackingColorID(m_tracking_color_id);

        m_tracking_color_id = eCommonTrackingColorID::INVALID;
    }
}

void ServerControllerView::updatePositionEstimation(TrackerManager* tracker_manager)
{
    // TODO: Probably need to first update IMU state to get velocity.
    // If velocity is too high, don't bother getting a new position.
    // Though it may be enough to just use the camera ROI as the limit.
    
    int positions_found = 0;
    glm::vec3 position3d_list[TrackerManager::k_max_devices];
    int valid_tracker_ids[TrackerManager::k_max_devices];
    
    for (int tracker_id = 0; tracker_id < tracker_manager->getMaxDevices(); ++tracker_id)
    {
        ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);
        if (tracker->computePositionForController(this, &position3d_list[positions_found]))
        {
            valid_tracker_ids[positions_found] = tracker_id;
            ++positions_found;
        }
    }
    
    if (false)//positions_found > 1)
    {
        glm::vec3 position2d_list[TrackerManager::k_max_devices];
        for (int list_index= 0; list_index < positions_found; ++list_index)
        {
            int tracker_id= valid_tracker_ids[list_index];
            ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);
            
            // Convert the camera intrinsic matric to an opencv matrix
            //const glm::mat4 glmCameraMatrix = tracker->getCameraIntrinsicMatrix();
            //cv::Matx33f cvCameraMatrix = glmMat4ToCvMat33f(glmCameraMatrix);
            
            // Get the camera pose
            //const glm::mat4 glmCameraPose = tracker->getCameraPoseMatrix();
            
            // Convert to openCV rvec/tvec
            //cv::Mat rvec(3, 1, cv::DataType<double>::type);
            //cv::Mat tvec(3, 1, cv::DataType<double>::type);
            
            // Convert 3dpoint to an open cv point
            //cv::vector<cv::Point3f> cvObjectPoints;
            
            // Project the 3d point onto the cameras image plane
            //cv::projectPoints(cvObjectPoints, rvec, tvec, cvCameraMatrix, cvDistCoeffs, projectedPoints);
            // Add the project point back to the 2d point list
        }
        // Select the best pair of 2d points to use and feed them into
        // cv::triangulatePoints to get a 3d position
    }
    else
    {
        // Use the 3d position for the only valid tracker
    }
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

                update_filters_for_psmove(psmoveController, psmoveState, m_orientation_filter, m_position_filter);
            } break;
        case CommonControllerState::PSNavi:
            {
                // No orientation or position to update
                assert(m_orientation_filter == nullptr);
                assert(m_position_filter == nullptr);
            } break;
        default:
            assert(0 && "Unhandled controller type");
        }

        // Consider this controller state sequence num processed
        m_lastPollSeqNumProcessed= controllerState->PollSequenceNumber;
    }
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

    if (m_position_filter != nullptr)
    {
        Eigen::Vector3f position= m_position_filter->getPosition(msec_time);

        pose.px= position.x();
        pose.py= position.y();
        pose.pz= position.z();
    }

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
    DeviceDataFramePtr &data_frame)
{
    PSMoveProtocol::DeviceDataFrame_ControllerDataPacket *controller_data_frame= 
        data_frame->mutable_controller_data_packet();

    controller_data_frame->set_controller_id(controller_view->getDeviceID());
    controller_data_frame->set_sequence_num(controller_view->m_sequence_number);
    controller_data_frame->set_isconnected(controller_view->getDevice()->getIsOpen());

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

    data_frame->set_device_category(PSMoveProtocol::DeviceDataFrame::CONTROLLER);
}

static void generate_psmove_data_frame_for_stream(
    const ServerControllerView *controller_view,
    const ControllerStreamInfo *stream_info,
    DeviceDataFramePtr &data_frame)
{
    const PSMoveController *psmove_controller= controller_view->castCheckedConst<PSMoveController>();
    const PSMoveControllerConfig *psmove_config= psmove_controller->getConfig();
    const CommonControllerState *controller_state= controller_view->getState();
    const psmovePosef controller_pose= controller_view->getPose();

    PSMoveProtocol::DeviceDataFrame_ControllerDataPacket *controller_data_frame= data_frame->mutable_controller_data_packet();
    PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_PSMoveState *psmove_data_frame = controller_data_frame->mutable_psmove_state();
   
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
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket::TRIANGLE, psmove_state->Triangle);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket::CIRCLE, psmove_state->Circle);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket::CROSS, psmove_state->Cross);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket::SQUARE, psmove_state->Square);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket::SELECT, psmove_state->Select);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket::START, psmove_state->Start);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket::PS, psmove_state->PS);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket::MOVE, psmove_state->Move);
        controller_data_frame->set_button_down_bitmask(button_bitmask);

        // If requested, get the raw sensor data for the controller
        if (stream_info->include_raw_sensor_data)
        {
            PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_PSMoveState_RawSensorData *raw_sensor_data=
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

        // If requested, get the raw tracker data for the controller
        if (stream_info->include_raw_tracker_data)
        {
            PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_PSMoveState_RawTrackerData *raw_tracker_data =
                psmove_data_frame->mutable_raw_tracker_data();

            //###HipsterSloth $TODO Publish real raw tracking
            //PSMoveProtocol::Pixel *pixel= raw_tracker_data->add_screen_locations();
            //PSMoveProtocol::Position *position = raw_tracker_data->add_relative_positions();
            //raw_tracker_data->add_tracker_ids(tracker_id);

            raw_tracker_data->set_valid_tracker_count(0);
        }
    }   

    controller_data_frame->set_controller_type(PSMoveProtocol::PSMOVE);
}

static void generate_psnavi_data_frame_for_stream(
    const ServerControllerView *controller_view,
    const ControllerStreamInfo *stream_info,
    DeviceDataFramePtr &data_frame)
{
    PSMoveProtocol::DeviceDataFrame_ControllerDataPacket *controller_data_frame = data_frame->mutable_controller_data_packet();
    PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_PSNaviState *psnavi_data_frame = controller_data_frame->mutable_psnavi_state();

    const CommonControllerState *controller_state= controller_view->getState();

    if (controller_state != nullptr)
    {
        assert(controller_state->DeviceType == CommonDeviceState::PSNavi);
        const PSNaviControllerState *psnavi_state= static_cast<const PSNaviControllerState *>(controller_state);

        psnavi_data_frame->set_trigger_value(psnavi_state->Trigger);
        psnavi_data_frame->set_stick_xaxis(psnavi_state->Stick_XAxis);
        psnavi_data_frame->set_stick_yaxis(psnavi_state->Stick_YAxis);

        unsigned int button_bitmask= 0;
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket::L1, psnavi_state->L1);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket::L2, psnavi_state->L2);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket::L3, psnavi_state->L3);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket::CIRCLE, psnavi_state->Circle);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket::CROSS, psnavi_state->Cross);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket::PS, psnavi_state->PS);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket::UP, psnavi_state->DPad_Up);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket::RIGHT, psnavi_state->DPad_Right);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket::DOWN, psnavi_state->DPad_Down);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket::LEFT, psnavi_state->DPad_Left);
        controller_data_frame->set_button_down_bitmask(button_bitmask);
    }

    controller_data_frame->set_controller_type(PSMoveProtocol::PSNAVI);
}

static void
init_filters_for_psmove(
    const PSMoveController *psmoveController, 
    OrientationFilter *orientation_filter,
    PositionFilter *position_filter)
{
    const PSMoveControllerConfig *psmove_config = psmoveController->getConfig();

    {
        // Setup the space the orientation filter operates in
        Eigen::Vector3f identityGravity = Eigen::Vector3f(0.f, 1.f, 0.f);
        Eigen::Vector3f identityMagnetometer = psmove_config->magnetometer_identity;
        Eigen::Matrix3f calibrationTransform = *k_eigen_identity_pose_laying_flat;
        Eigen::Matrix3f sensorTransform = *k_eigen_sensor_transform_opengl;
        OrientationFilterSpace filterSpace(identityGravity, identityMagnetometer, calibrationTransform, sensorTransform);

        orientation_filter->setFilterSpace(filterSpace);

        // Use the complementary MARG fusion filter by default
        orientation_filter->setFusionType(OrientationFilter::FusionTypeComplementaryMARG);
    }

    {
        //###bwalker $TODO Setup the space the position filter operates in
        Eigen::Matrix3f sensorTransform = Eigen::Matrix3f::Identity();
        PositionFilterSpace filterSpace(sensorTransform);

        position_filter->setFilterSpace(filterSpace);

        //###bwalker $TODO Use the LowPass filter by default
        position_filter->setFusionType(PositionFilter::FusionTypePassThru);
    }
}

static void                
update_filters_for_psmove(
    const PSMoveController *psmoveController, 
    const PSMoveControllerState *psmoveState,
    OrientationFilter *orientationFilter,
    PositionFilter *position_filter)
{
    const PSMoveControllerConfig *config = psmoveController->getConfig();

    // Update the orientation filter
    {
        //###bwalker $TODO Determine time deltas from the timestamps on the controller frames
        const float delta_time = 1.f / 120.f;

        OrientationSensorPacket sensorPacket;

        // Re-scale the magnetometer int-vector into a float vector in the range <-1,-1,-1> to <1,1,1>
        // using the min and max magnetometer extents stored in the controller config.
        {
            const Eigen::Vector3f sample =
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
        for (int frame = 0; frame < 2; ++frame)
        {
            sensorPacket.orientation = orientationFilter->getOrientation();

            sensorPacket.accelerometer =
                Eigen::Vector3f(psmoveState->Accel[frame][0], psmoveState->Accel[frame][1], psmoveState->Accel[frame][2]);
            sensorPacket.gyroscope =
                Eigen::Vector3f(psmoveState->Gyro[frame][0], psmoveState->Gyro[frame][1], psmoveState->Gyro[frame][2]);

            // Update the orientation filter using the sensor packet.
            // NOTE: The magnetometer reading is the same for both sensor readings.
            orientationFilter->update(delta_time, sensorPacket);
        }
    }


    // Update the position filter
    {
        //###bwalker $TODO Determine time deltas from trackers
        const float delta_time = 1.f / 60.f;

        //###bwalker $TODO - Extract the position from the attached trackers
        PositionSensorPacket sensorPacket;

        sensorPacket.position = Eigen::Vector3f::Zero();
        sensorPacket.velocity = Eigen::Vector3f::Zero();
        sensorPacket.acceleration = Eigen::Vector3f::Zero();

        position_filter->update(delta_time, sensorPacket);
    }
}
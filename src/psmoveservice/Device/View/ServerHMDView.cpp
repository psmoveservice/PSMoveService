//-- includes -----
#include "ServerHMDView.h"
#include "OculusHMD.h"
#include "OrientationFilter.h"
#include "PositionFilter.h"
#include "PSMoveProtocol.pb.h"
#include "ServerRequestHandler.h"

//-- private methods -----
static void init_filters_for_oculus_hmd(
    const OculusHMD *oculusHMD,
    OrientationFilter *orientation_filter, PositionFilter *position_filter);
static void update_filters_for_oculus_hmd(
    const OculusHMD *oculusHMD, const OculusHMDState *oculusHMDState,
    OrientationFilter *orientationFilter, PositionFilter *position_filter);
static void generate_oculus_hmd_data_frame_for_stream(
    const ServerHMDView *hmd_view, const HMDStreamInfo *stream_info,
    DeviceDataFramePtr &data_frame);

static Eigen::Vector3f CommonDevicePosition_to_EigenVector3f(const CommonDevicePosition &p);
static Eigen::Vector3f CommonDeviceVector_to_EigenVector3f(const CommonDeviceVector &v);
static Eigen::Quaternionf CommonDeviceQuaternion_to_EigenQuaternionf(const CommonDeviceQuaternion &q);

static CommonDevicePosition EigenVector3f_to_CommonDevicePosition(const Eigen::Vector3f &p);
static CommonDeviceQuaternion EigenQuaternionf_to_CommonDeviceQuaternion(const Eigen::Quaternionf &q);

//-- public implementation -----
ServerHMDView::ServerHMDView(const int device_id)
    : ServerDeviceView(device_id)
    , m_device(nullptr)
    , m_orientation_filter(nullptr)
    , m_position_filter(nullptr)
    , m_lastPollSeqNumProcessed(-1)
{
    //###bwalker $TODO The device should be allocated in open() based on the enumerator type
    m_device = nullptr;
}

ServerHMDView::~ServerHMDView()
{
    if (m_orientation_filter != nullptr)
    {
        delete m_orientation_filter;
        m_orientation_filter = nullptr;
    }

    if (m_position_filter != nullptr)
    {
        delete m_position_filter;
        m_position_filter = nullptr;
    }

    if (m_device != nullptr)
    {
        delete m_device;
    }
}

bool ServerHMDView::allocate_device_interface(const class DeviceEnumerator *enumerator)
{
    switch (enumerator->get_device_type())
    {
    case CommonDeviceState::OculusDK2:
        {
            m_device = new OculusDevKit2();
        } break;
    default:
        break;
    }

    return m_device != nullptr;
}

void ServerHMDView::free_device_interface()
{
    if (m_device != nullptr)
    {
        delete m_device;
        m_device = nullptr;
    }
}

bool ServerHMDView::open(const class DeviceEnumerator *enumerator)
{
    // Attempt to open the controller
    bool bSuccess = ServerDeviceView::open(enumerator);

    // Setup the orientation filter based on the controller configuration
    if (bSuccess)
    {
        IDeviceInterface *device = getDevice();

        switch (device->getDeviceType())
        {
        case CommonDeviceState::OculusDK2:
            {
                const OculusHMD *oculusHMD = this->castCheckedConst<OculusHMD>();

                init_filters_for_oculus_hmd(oculusHMD, m_orientation_filter, m_position_filter);
            } break;
        default:
            break;
        }

        // Reset the poll sequence number high water mark
        m_lastPollSeqNumProcessed = -1;
    }

    return bSuccess;
}

void ServerHMDView::updateStateAndPredict()
{
    if (!getHasUnpublishedState())
    {
        return;
    }

    // Look backward in time to find the first hmd update state with a poll sequence number 
    // newer than the last sequence number we've processed.
    int firstLookBack = -1;
    int testLookBack = 0;
    const CommonHMDState *state = getState(testLookBack);
    while (state != nullptr && state->PollSequenceNumber > m_lastPollSeqNumProcessed)
    {
        firstLookBack = testLookBack;
        testLookBack++;
        state = getState(testLookBack);
    }
    assert(firstLookBack >= 0);

    // Process the polled controller states forward in time
    // computing the new orientation along the way.
    for (int lookBack = firstLookBack; lookBack >= 0; --lookBack)
    {
        const CommonHMDState *hmdState = getState(lookBack);

        switch (hmdState->DeviceType)
        {
        case CommonHMDState::OculusDK2:
        {
            const OculusHMD *oculusHMD = this->castCheckedConst<OculusHMD>();
            const OculusHMDState *oculusHMDState = static_cast<const OculusHMDState *>(hmdState);

            update_filters_for_oculus_hmd(oculusHMD, oculusHMDState, m_orientation_filter, m_position_filter);
        } break;
        default:
            assert(0 && "Unhandled HMD type");
        }

        // Consider this controller state sequence num processed
        m_lastPollSeqNumProcessed = hmdState->PollSequenceNumber;
    }
}

CommonDevicePose
ServerHMDView::getPose(int msec_time) const
{
    CommonDevicePose pose;

    pose.clear();

    if (m_orientation_filter != nullptr)
    {
        Eigen::Quaternionf orientation = m_orientation_filter->getOrientation(msec_time);

        pose.Orientation = EigenQuaternionf_to_CommonDeviceQuaternion(orientation);
    }

    if (m_position_filter != nullptr)
    {
        Eigen::Vector3f position = m_position_filter->getPosition(msec_time);

        pose.Position = EigenVector3f_to_CommonDevicePosition(position);
    }

    return pose;
}

// Returns the full usb device path for the controller
std::string
ServerHMDView::getUSBDevicePath() const
{
    return m_device->getUSBDevicePath();
}

// Returns the serial number for the controller
std::string
ServerHMDView::getSerial() const
{
    return m_device->getSerial();
}

CommonDeviceState::eDeviceType
ServerHMDView::getHMDDeviceType() const
{
    return m_device->getDeviceType();
}

// Fetch the controller state at the given sample index.
// A lookBack of 0 corresponds to the most recent data.
const struct CommonHMDState * ServerHMDView::getState(
    int lookBack) const
{
    const struct CommonDeviceState *device_state = m_device->getState(lookBack);
    assert(device_state == nullptr ||
        (device_state->DeviceType >= CommonDeviceState::HeadMountedDisplay &&
        device_state->DeviceType < CommonDeviceState::SUPPORTED_HMD_TYPE_COUNT));

    return static_cast<const CommonHMDState *>(device_state);
}

void ServerHMDView::publish_device_data_frame()
{
    // Tell the server request handler we want to send out HMD updates.
    // This will call generate_hmd_data_frame_for_stream for each listening connection.
    ServerRequestHandler::get_instance()->publish_hmd_data_frame(
        this, &ServerHMDView::generate_hmd_data_frame_for_stream);
}

void ServerHMDView::generate_hmd_data_frame_for_stream(
    const ServerHMDView *hmd_view,
    const struct HMDStreamInfo *stream_info,
    DeviceDataFramePtr &data_frame)
{
    PSMoveProtocol::DeviceDataFrame_HMDDataPacket *hmd_data_frame =
        data_frame->mutable_hmd_data_packet();

    hmd_data_frame->set_hmd_id(hmd_view->getDeviceID());
    hmd_data_frame->set_sequence_num(hmd_view->m_sequence_number);
    hmd_data_frame->set_isconnected(hmd_view->getDevice()->getIsOpen());

    // Write the common HMD pose
    {
        const CommonHMDState *hmdState= hmd_view->getState(0);
        const CommonDeviceQuaternion &Orientation = hmdState->Pose.Orientation;
        const CommonDevicePosition &Position = hmdState->Pose.Position;

        hmd_data_frame->mutable_orientation()->set_w(Orientation.w);
        hmd_data_frame->mutable_orientation()->set_x(Orientation.x);
        hmd_data_frame->mutable_orientation()->set_y(Orientation.y);
        hmd_data_frame->mutable_orientation()->set_z(Orientation.z);

        hmd_data_frame->mutable_position()->set_x(Position.x);
        hmd_data_frame->mutable_position()->set_y(Position.y);
        hmd_data_frame->mutable_position()->set_z(Position.z);
    }

    switch (hmd_view->getHMDDeviceType())
    {
    case CommonHMDState::OculusDK2:
        {
            generate_oculus_hmd_data_frame_for_stream(hmd_view, stream_info, data_frame);
        } break;
    default:
        assert(0 && "Unhandled HMD type");
    }

    data_frame->set_device_category(PSMoveProtocol::DeviceDataFrame::HMD);
}

static void
init_filters_for_oculus_hmd(
    const OculusHMD *oculusHMD,
    OrientationFilter *orientation_filter,
    PositionFilter *position_filter)
{
    const OculusHMDConfig *psmove_config = oculusHMD->getConfig();

    {
        // Setup the space the orientation filter operates in
        Eigen::Vector3f identityGravity = Eigen::Vector3f::Zero();
        Eigen::Vector3f identityMagnetometer = Eigen::Vector3f::Zero();
        Eigen::Matrix3f calibrationTransform = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f sensorTransform = Eigen::Matrix3f::Identity();
        OrientationFilterSpace filterSpace(identityGravity, identityMagnetometer, calibrationTransform, sensorTransform);
        orientation_filter->setFilterSpace(filterSpace);

        // Don't need any orientation filtering with oculus HMDs since the API provides it's own
        orientation_filter->setFusionType(OrientationFilter::FusionTypePassThru);
    }

    {
        // Setup the space the position filter operates in
        Eigen::Matrix3f sensorTransform = Eigen::Matrix3f::Identity();
        PositionFilterSpace filterSpace(sensorTransform);
        position_filter->setFilterSpace(filterSpace);

        // Don't need any position filtering with oculus HMDs since the API provides it's own
        position_filter->setFusionType(PositionFilter::FusionTypePassThru);
    }
}

static void
update_filters_for_oculus_hmd(
    const OculusHMD *oculusHMD,
    const OculusHMDState *oculusHMDState,
    OrientationFilter *orientationFilter,
    PositionFilter *position_filter)
{
    const OculusHMDConfig *config = oculusHMD->getConfig();

    // Update the orientation filter
    {
        //###bwalker $TODO Determine time deltas from the timestamps on the controller frames
        const float delta_time = 1.f / 120.f;

        OrientationSensorPacket sensorPacket;

        sensorPacket.orientation = 
            CommonDeviceQuaternion_to_EigenQuaternionf(oculusHMDState->Pose.Orientation);
        sensorPacket.accelerometer =
            CommonDeviceVector_to_EigenVector3f(oculusHMDState->Accelerometer);
        sensorPacket.gyroscope =
            CommonDeviceVector_to_EigenVector3f(oculusHMDState->Gyro);
        sensorPacket.magnetometer =
            CommonDeviceVector_to_EigenVector3f(oculusHMDState->Magnetometer);

        // Update the orientation filter using the sensor packet.
        orientationFilter->update(delta_time, sensorPacket);
    }


    // Update the position filter
    {
        //###bwalker $TODO Determine time deltas from trackers
        const float delta_time = 1.f / 60.f;

        //###bwalker $TODO - Extract the position from the attached trackers
        PositionSensorPacket sensorPacket;

        sensorPacket.position = CommonDevicePosition_to_EigenVector3f(oculusHMDState->Pose.Position);
        sensorPacket.velocity = CommonDeviceVector_to_EigenVector3f(oculusHMDState->LinearVelocity);
        sensorPacket.acceleration = CommonDeviceVector_to_EigenVector3f(oculusHMDState->LinearAcceleration);

        position_filter->update(delta_time, sensorPacket);
    }
}

static void generate_oculus_hmd_data_frame_for_stream(
    const ServerHMDView *hmd_view,
    const HMDStreamInfo *stream_info,
    DeviceDataFramePtr &data_frame)
{
    const OculusHMD *oculus_hmd = hmd_view->castCheckedConst<OculusHMD>();
    const OculusHMDConfig *psmove_config = oculus_hmd->getConfig();
    const CommonHMDState *hmd_state = hmd_view->getState();
    const CommonDevicePose hmd_pose = hmd_view->getPose();

    PSMoveProtocol::DeviceDataFrame_HMDDataPacket *hmd_data_frame = data_frame->mutable_hmd_data_packet();

    if (hmd_state != nullptr)
    {
        assert(hmd_state->DeviceType == CommonDeviceState::OculusDK2);
        const OculusHMDState * oculus_hmd_state = static_cast<const OculusHMDState *>(hmd_state);

        PSMoveProtocol::DeviceDataFrame_HMDDataPacket_OculusDK2State* oculus_data_frame = 
            hmd_data_frame->mutable_oculus_dk2_state();

        oculus_data_frame->mutable_linear_velocity()->set_i(oculus_hmd_state->LinearVelocity.i);
        oculus_data_frame->mutable_linear_velocity()->set_j(oculus_hmd_state->LinearVelocity.j);
        oculus_data_frame->mutable_linear_velocity()->set_k(oculus_hmd_state->LinearVelocity.k);

        oculus_data_frame->mutable_linear_acceleration()->set_i(oculus_hmd_state->LinearAcceleration.i);
        oculus_data_frame->mutable_linear_acceleration()->set_j(oculus_hmd_state->LinearAcceleration.j);
        oculus_data_frame->mutable_linear_acceleration()->set_k(oculus_hmd_state->LinearAcceleration.k);

        oculus_data_frame->mutable_angular_velocity()->set_i(oculus_hmd_state->AngularVelocity.i);
        oculus_data_frame->mutable_angular_velocity()->set_j(oculus_hmd_state->AngularVelocity.j);
        oculus_data_frame->mutable_angular_velocity()->set_k(oculus_hmd_state->AngularVelocity.k);

        oculus_data_frame->mutable_angular_acceleration()->set_i(oculus_hmd_state->AngularAcceleration.i);
        oculus_data_frame->mutable_angular_acceleration()->set_j(oculus_hmd_state->AngularAcceleration.j);
        oculus_data_frame->mutable_angular_acceleration()->set_k(oculus_hmd_state->AngularAcceleration.k);

        oculus_data_frame->set_state_time(static_cast<float>(oculus_hmd_state->StateTime));

        // If requested, get the raw sensor data for the hmd
        if (stream_info->include_raw_sensor_data)
        {
            PSMoveProtocol::DeviceDataFrame_HMDDataPacket_OculusDK2State_RawSensorData *raw_sensor_data =
                oculus_data_frame->mutable_raw_sensor_data();

            raw_sensor_data->mutable_magnetometer()->set_i(oculus_hmd_state->Magnetometer.i);
            raw_sensor_data->mutable_magnetometer()->set_j(oculus_hmd_state->Magnetometer.j);
            raw_sensor_data->mutable_magnetometer()->set_k(oculus_hmd_state->Magnetometer.k);

            raw_sensor_data->mutable_accelerometer()->set_i(oculus_hmd_state->Accelerometer.i);
            raw_sensor_data->mutable_accelerometer()->set_j(oculus_hmd_state->Accelerometer.j);
            raw_sensor_data->mutable_accelerometer()->set_k(oculus_hmd_state->Accelerometer.k);

            raw_sensor_data->mutable_gyroscope()->set_i(oculus_hmd_state->Gyro.i);
            raw_sensor_data->mutable_gyroscope()->set_j(oculus_hmd_state->Gyro.j);
            raw_sensor_data->mutable_gyroscope()->set_k(oculus_hmd_state->Gyro.k);

            raw_sensor_data->set_temparature(oculus_hmd_state->Temperature);
            raw_sensor_data->set_imu_sample_time(oculus_hmd_state->IMUSampleTime);
        }
    }

    hmd_data_frame->set_hmd_type(PSMoveProtocol::OculusDK2);
}

static Eigen::Vector3f CommonDevicePosition_to_EigenVector3f(const CommonDevicePosition &p)
{
    return Eigen::Vector3f(p.x, p.y, p.z);
}

static Eigen::Vector3f CommonDeviceVector_to_EigenVector3f(const CommonDeviceVector &v)
{
    return Eigen::Vector3f(v.i, v.j, v.k);
}

static Eigen::Quaternionf CommonDeviceQuaternion_to_EigenQuaternionf(const CommonDeviceQuaternion &q)
{
    return Eigen::Quaternionf(q.w, q.x, q.y, q.z);
}

static CommonDevicePosition EigenVector3f_to_CommonDevicePosition(const Eigen::Vector3f &p)
{
    CommonDevicePosition result;

    result.x = p.x();
    result.y = p.y();
    result.z = p.z();

    return result;
}

static CommonDeviceQuaternion EigenQuaternionf_to_CommonDeviceQuaternion(const Eigen::Quaternionf &q)
{
    CommonDeviceQuaternion result;

    result.w = q.w();
    result.x = q.x();
    result.y = q.y();
    result.z = q.z();

    return result;
}


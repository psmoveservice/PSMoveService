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
static const float k_min_time_delta_seconds = 1 / 120.f;
static const float k_max_time_delta_seconds = 1 / 30.f;

//-- macros -----
#define SET_BUTTON_BIT(bitmask, bit_index, button_state) \
    bitmask|= (button_state == CommonControllerState::Button_DOWN || button_state == CommonControllerState::Button_PRESSED) ? (0x1 << (bit_index)) : 0x0;

//-- private methods -----
static void init_filters_for_psmove(
    const PSMoveController *psmoveController, 
    OrientationFilter *orientation_filter, PositionFilter *position_filter);
static void update_filters_for_psmove(
    const PSMoveController *psmoveController, const PSMoveControllerState *psmoveState, const float delta_time,
    const ControllerPositionEstimation *positionEstimation,
    OrientationFilter *orientationFilter, PositionFilter *position_filter);

static void generate_psmove_data_frame_for_stream(
    const ServerControllerView *controller_view, const ControllerStreamInfo *stream_info, PSMoveProtocol::DeviceOutputDataFrame* data_frame);
static void generate_psnavi_data_frame_for_stream(
    const ServerControllerView *controller_view, const ControllerStreamInfo *stream_info, PSMoveProtocol::DeviceOutputDataFrame* data_frame);

//-- public implementation -----
ServerControllerView::ServerControllerView(const int device_id)
    : ServerDeviceView(device_id)
    , m_tracking_color_id(eCommonTrackingColorID::INVALID_COLOR)
    , m_tracking_listener_count(0)
    , m_tracking_enabled(false)
    , m_LED_override_active(false)
    , m_device(nullptr)
    , m_tracker_position_estimation(nullptr)
    , m_multicam_position_estimation(nullptr)
    , m_orientation_filter(nullptr)
    , m_position_filter(nullptr)
    , m_lastPollSeqNumProcessed(-1)
    , m_last_filter_update_timestamp()
    , m_last_filter_update_timestamp_valid(false)
{
    m_tracking_color = std::make_tuple(0x00, 0x00, 0x00);
    m_LED_override_color = std::make_tuple(0x00, 0x00, 0x00);
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

            m_tracker_position_estimation = new ControllerPositionEstimation[TrackerManager::k_max_devices];
            for (int tracker_index = 0; tracker_index < TrackerManager::k_max_devices; ++tracker_index)
            {
                m_tracker_position_estimation[tracker_index].clear();
            }

            m_multicam_position_estimation = new ControllerPositionEstimation();
            m_multicam_position_estimation->clear();
        } break;
    case CommonDeviceState::PSNavi:
        {
            m_device= new PSNaviController();
            m_orientation_filter= nullptr;
            m_position_filter = nullptr;
            m_multicam_position_estimation = nullptr;
        } break;
    default:
        break;
    }

    return m_device != nullptr;
}

void ServerControllerView::free_device_interface()
{
    if (m_multicam_position_estimation != nullptr)
    {
        delete m_multicam_position_estimation;
        m_multicam_position_estimation= nullptr;
    }

    if (m_tracker_position_estimation != nullptr)
    {
        delete[] m_tracker_position_estimation;
        m_tracker_position_estimation = nullptr;
    }

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

                // Don't bother initializing any filters or allocating a tracking color
                // for usb connected controllers
                if (psmoveController->getIsBluetooth())
                {
                    init_filters_for_psmove(psmoveController, m_orientation_filter, m_position_filter);
                    m_multicam_position_estimation->clear();

                    bAllocateTrackingColor = true;
                }
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
        assert(m_tracking_color_id == eCommonTrackingColorID::INVALID_COLOR);
        m_tracking_color_id= DeviceManager::getInstance()->m_controller_manager->allocateTrackingColorID();
    }

    // Clear the filter update timestamp
    m_last_filter_update_timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>();
    m_last_filter_update_timestamp_valid= false;

    return bSuccess;
}

void ServerControllerView::close()
{
    set_tracking_enabled_internal(false);

    if (m_tracking_color_id != eCommonTrackingColorID::INVALID_COLOR)
    {
        DeviceManager::getInstance()->m_controller_manager->freeTrackingColorID(m_tracking_color_id);

        m_tracking_color_id = eCommonTrackingColorID::INVALID_COLOR;
    }

    ServerDeviceView::close();
}

void ServerControllerView::updatePositionEstimation(TrackerManager* tracker_manager)
{
    const std::chrono::time_point<std::chrono::high_resolution_clock> now= std::chrono::high_resolution_clock::now();

    // TODO: Probably need to first update IMU state to get velocity.
    // If velocity is too high, don't bother getting a new position.
    // Though it may be enough to just use the camera ROI as the limit.
    
    if (getIsTrackingEnabled())
    {
        int valid_tracker_ids[TrackerManager::k_max_devices];
        int positions_found = 0;

        // Compute an estimated 3d tracked position of the controller 
        // from the perspective of each tracker
        for (int tracker_id = 0; tracker_id < tracker_manager->getMaxDevices(); ++tracker_id)
        {
            ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);
            ControllerPositionEstimation &positionEstimate = m_tracker_position_estimation[tracker_id];

            positionEstimate.bCurrentlyTracking = false;

            if (tracker->getIsOpen())
            {
                if (tracker->computePositionForController(
                    this,
                    &positionEstimate.position,
                    &positionEstimate.projection))
                {
                    positionEstimate.bCurrentlyTracking = true;
                    positionEstimate.last_visible_timestamp = now;

                    valid_tracker_ids[positions_found] = tracker_id;
                    ++positions_found;
                }
            }

            // Keep track of the last time the position estimate was updated
            positionEstimate.last_update_timestamp = now;
            positionEstimate.bValidTimestamps = true;
        }

        // If multiple trackers can see the controller, 
        // triangulate all pairs of trackers and average the results
        if (positions_found > 1)
        {
            // Project the tracker relative 3d tracking position back on to the tracker camera plane
            CommonDeviceScreenLocation position2d_list[TrackerManager::k_max_devices];
            for (int list_index = 0; list_index < positions_found; ++list_index)
            {
                const int tracker_id = valid_tracker_ids[list_index];
                const ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);
                const ControllerPositionEstimation &positionEstimate = m_tracker_position_estimation[tracker_id];
                
                position2d_list[list_index] = tracker->projectTrackerRelativePosition(&positionEstimate.position);
            }

            int pair_count = 0;
            CommonDevicePosition average_world_position = { 0.f, 0.f, 0.f };
            for (int list_index = 0; list_index < positions_found; ++list_index)
            {
                const int tracker_id = valid_tracker_ids[list_index];
                const CommonDeviceScreenLocation &screen_location = position2d_list[list_index];
                const ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);

                for (int other_list_index = list_index + 1; other_list_index < positions_found; ++other_list_index)
                {
                    const int other_tracker_id = valid_tracker_ids[other_list_index];
                    const CommonDeviceScreenLocation &other_screen_location = position2d_list[other_list_index];
                    const ServerTrackerViewPtr other_tracker = tracker_manager->getTrackerViewPtr(other_tracker_id);

                    // Using the screen locations on two different trackers we can triangulate a world position
                    CommonDevicePosition world_position =
                        ServerTrackerView::triangulateWorldPosition(
                            tracker.get(), &screen_location,
                            other_tracker.get(), &other_screen_location);

                    average_world_position.x += world_position.x;
                    average_world_position.y += world_position.y;
                    average_world_position.z += world_position.z;

                    ++pair_count;
                }
            }

            if (pair_count > 1)
            {
                const float N = static_cast<float>(pair_count);

                average_world_position.x /= N;
                average_world_position.y /= N;
                average_world_position.z /= N;
            }

            m_multicam_position_estimation->position = average_world_position;
            m_multicam_position_estimation->bCurrentlyTracking = true;
        }
        // If only one tracker can see the controller, then just use the position estimate from that
        else if (positions_found == 1)
        {
            // Put the tracker relative position into world space
            const int tracker_id = valid_tracker_ids[0];
            const ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);
            const CommonDevicePosition &tracker_relative_position = m_tracker_position_estimation[tracker_id].position;

            m_multicam_position_estimation->position = tracker->computeWorldPosition(&tracker_relative_position);
            m_multicam_position_estimation->bCurrentlyTracking = true;
        }
        // If no trackers can see the controller, maintain the last known position and time it was seen
        else
        {
            m_multicam_position_estimation->bCurrentlyTracking= false;
        }

        // Update the position estimation timestamps
        if (positions_found > 0)
        {
            m_multicam_position_estimation->last_visible_timestamp = now;
        }
        m_multicam_position_estimation->last_update_timestamp = now;
        m_multicam_position_estimation->bValidTimestamps = true;
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
    int firstLookBackIndex = -1;
    int testLookBack = 0;
    const CommonControllerState *state= getState(testLookBack);
    while (state != nullptr && state->PollSequenceNumber > m_lastPollSeqNumProcessed)
    {
        firstLookBackIndex= testLookBack;
        testLookBack++;
        state= getState(testLookBack);
    }
    assert(firstLookBackIndex >= 0);

    // Compute the time in seconds since the last update
    const std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
    float time_delta_seconds;
    if (m_last_filter_update_timestamp_valid)
    {
        const std::chrono::duration<float, std::milli> time_delta = now - m_last_filter_update_timestamp;
        const float time_delta_milli = time_delta.count();

        // convert delta to seconds clamp time delta between 120hz and 30hz
        time_delta_seconds = clampf(time_delta_milli / 1000.f, k_min_time_delta_seconds, k_max_time_delta_seconds);
    }
    else
    {
        time_delta_seconds = k_max_time_delta_seconds; 
    }
    m_last_filter_update_timestamp = now;
    m_last_filter_update_timestamp_valid = true;

    // Evenly apply the list of controller state updates over the time since last filter update
    float per_state_time_delta_seconds = time_delta_seconds / static_cast<float>(firstLookBackIndex + 1);

    // Process the polled controller states forward in time
    // computing the new orientation along the way.
    for (int lookBackIndex= firstLookBackIndex; lookBackIndex >= 0; --lookBackIndex)
    {
        const CommonControllerState *controllerState= getState(lookBackIndex);

        switch (controllerState->DeviceType)
        {
        case CommonControllerState::PSMove:
            {
                const PSMoveController *psmoveController= this->castCheckedConst<PSMoveController>();
                const PSMoveControllerState *psmoveState= static_cast<const PSMoveControllerState *>(controllerState);

                // Only update the position filter when tracking is enabled
                update_filters_for_psmove(
                    psmoveController, psmoveState, 
                    per_state_time_delta_seconds,
                    m_multicam_position_estimation, 
                    m_orientation_filter, 
                    getIsTrackingEnabled() ? m_position_filter : nullptr);
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

CommonDevicePose
ServerControllerView::getFilteredPose(float time) const
{
    CommonDevicePose pose;

    pose.clear();

    if (m_orientation_filter != nullptr)
    {
        Eigen::Quaternionf orientation= m_orientation_filter->getOrientation(time);

        pose.Orientation.w= orientation.w();
        pose.Orientation.x= orientation.x();
        pose.Orientation.y= orientation.y();
        pose.Orientation.z= orientation.z();
    }

    if (m_position_filter != nullptr)
    {
        Eigen::Vector3f position= m_position_filter->getPosition(time);

        pose.Position.x= position.x();
        pose.Position.y= position.y();
        pose.Position.z= position.z();
    }

    return pose;
}

CommonDevicePhysics 
ServerControllerView::getFilteredPhysics() const
{
    CommonDevicePhysics physics;

    if (m_orientation_filter != nullptr)
    {
        const Eigen::Vector3f first_derivative= m_orientation_filter->getAngularVelocity();
        const Eigen::Vector3f second_derivative= m_orientation_filter->getAngularAcceleration();

        physics.AngularVelocity.i = first_derivative.x();
        physics.AngularVelocity.j = first_derivative.y();
        physics.AngularVelocity.k = first_derivative.z();

        physics.AngularAcceleration.i = second_derivative.x();
        physics.AngularAcceleration.j = second_derivative.y();
        physics.AngularAcceleration.k = second_derivative.z();
    }

    if (m_position_filter != nullptr)
    {
        Eigen::Vector3f velocity(m_position_filter->getVelocity());
        Eigen::Vector3f acceleration(m_position_filter->getAcceleration());

        physics.Velocity.i = velocity.x();
        physics.Velocity.j = velocity.y();
        physics.Velocity.k = velocity.z();

        physics.Acceleration.i = acceleration.x();
        physics.Acceleration.j = acceleration.y();
        physics.Acceleration.k = acceleration.z();
    }

    return physics;
}

bool 
ServerControllerView::getIsBluetooth() const
{
    return (m_device != nullptr) ? m_device->getIsBluetooth() : false;
}

// Returns the full usb device path for the controller
std::string 
ServerControllerView::getUSBDevicePath() const
{
    return (m_device != nullptr) ? m_device->getUSBDevicePath() : "";
}

// Returns the serial number for the controller
std::string 
ServerControllerView::getSerial() const
{
    return(m_device != nullptr) ? m_device->getSerial() : "";
}

std::string 
ServerControllerView::getAssignedHostBluetoothAddress() const
{
    return (m_device != nullptr) ? m_device->getAssignedHostBluetoothAddress() : "";
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
    const struct CommonDeviceState *device_state = m_device->getState(lookBack);
    assert(device_state == nullptr ||
        ((int)device_state->DeviceType >= (int)CommonDeviceState::Controller &&
        device_state->DeviceType < CommonDeviceState::SUPPORTED_CONTROLLER_TYPE_COUNT));

    return static_cast<const CommonControllerState *>(device_state);
}

void ServerControllerView::setLEDOverride(unsigned char r, unsigned char g, unsigned char b)
{
    m_LED_override_color = std::make_tuple(r, g, b);
    m_LED_override_active = true;
    update_LED_color_internal();
}

void ServerControllerView::clearLEDOverride()
{
    m_LED_override_color = std::make_tuple(0x00, 0x00, 0x00);
    m_LED_override_active = false;
    update_LED_color_internal();
}

void ServerControllerView::setTrackingColorID(eCommonTrackingColorID colorID)
{
    if (colorID != m_tracking_color_id)
    {
        bool bWasTracking = getIsTrackingEnabled();

        if (bWasTracking)
        {
            set_tracking_enabled_internal(false);
        }

        m_tracking_color_id = colorID;

        if (bWasTracking)
        {
            set_tracking_enabled_internal(true);
        }
    }
}

void ServerControllerView::startTracking()
{
    if (!m_tracking_enabled)
    {
        set_tracking_enabled_internal(true);
    }

    ++m_tracking_listener_count;
}

void ServerControllerView::stopTracking()
{
    assert(m_tracking_listener_count > 0);
    --m_tracking_listener_count;

    if (m_tracking_listener_count <= 0 && m_tracking_enabled)
    {
        set_tracking_enabled_internal(false);
    }
}

void ServerControllerView::set_tracking_enabled_internal(bool bEnabled)
{
    if (m_tracking_enabled != bEnabled)
    {
        if (bEnabled)
        {
            switch (m_tracking_color_id)
            {
            case PSMoveProtocol::Magenta:
                m_tracking_color= std::make_tuple(0xFF, 0x00, 0xFF);
                break;
            case PSMoveProtocol::Cyan:
                m_tracking_color = std::make_tuple(0x00, 0xFF, 0xFF);
                break;
            case PSMoveProtocol::Yellow:
                m_tracking_color = std::make_tuple(0xFF, 0xFF, 0x00);
                break;
            case PSMoveProtocol::Red:
                m_tracking_color = std::make_tuple(0xFF, 0x00, 0x00);
                break;
            case PSMoveProtocol::Green:
                m_tracking_color = std::make_tuple(0x00, 0xFF, 0x00);
                break;
            case PSMoveProtocol::Blue:
                m_tracking_color = std::make_tuple(0x00, 0x00, 0xFF);
                break;
            default:
                assert(0 && "unreachable");
            }
        }
        else
        {
            m_tracking_color = std::make_tuple(0x00, 0x00, 0x00);
        }

        m_tracking_enabled = bEnabled;

        update_LED_color_internal();
    }
}

void ServerControllerView::update_LED_color_internal()
{
    unsigned char r, g, b;
    if (m_LED_override_active)
    {
        r = std::get<0>(m_LED_override_color);
        g = std::get<1>(m_LED_override_color);
        b = std::get<2>(m_LED_override_color);
    }
    else if (m_tracking_enabled)
    {
        assert(m_tracking_color_id != eCommonTrackingColorID::INVALID_COLOR);
        r = std::get<0>(m_tracking_color);
        g = std::get<1>(m_tracking_color);
        b = std::get<2>(m_tracking_color);
    }
    else
    {
        r = g = b = 0;
    }

    switch (getControllerDeviceType())
    {
    case CommonDeviceState::PSMove:
        {
            this->castChecked<PSMoveController>()->setLED(r, g, b);
        } break;
    case CommonDeviceState::PSNavi:
        {
            // Do nothing...
        } break;
    default:
        assert(false && "Unhanded controller type!");
    }
}

// Get the tracking shape for the controller
bool ServerControllerView::getTrackingShape(CommonDeviceTrackingShape &trackingShape)
{
    m_device->getTrackingShape(trackingShape);

    return trackingShape.shape_type != eCommonTrackingShapeType::INVALID_SHAPE;
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
    PSMoveProtocol::DeviceOutputDataFrame* data_frame)
{
    PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket *controller_data_frame= 
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

    data_frame->set_device_category(PSMoveProtocol::DeviceOutputDataFrame::CONTROLLER);
}

static void generate_psmove_data_frame_for_stream(
    const ServerControllerView *controller_view,
    const ControllerStreamInfo *stream_info,
    PSMoveProtocol::DeviceOutputDataFrame* data_frame)
{
    const PSMoveController *psmove_controller= controller_view->castCheckedConst<PSMoveController>();
    const PSMoveControllerConfig *psmove_config= psmove_controller->getConfig();
    const CommonControllerState *controller_state= controller_view->getState();
    const CommonDevicePose controller_pose = controller_view->getFilteredPose(psmove_config->prediction_time);

    auto *controller_data_frame= data_frame->mutable_controller_data_packet();
    auto *psmove_data_frame = controller_data_frame->mutable_psmove_state();
   
    if (controller_state != nullptr)
    {        
        assert(controller_state->DeviceType == CommonDeviceState::PSMove);
        const PSMoveControllerState * psmove_state= static_cast<const PSMoveControllerState *>(controller_state);

        psmove_data_frame->set_validhardwarecalibration(psmove_config->is_valid);
        psmove_data_frame->set_iscurrentlytracking(controller_view->getIsCurrentlyTracking());
        psmove_data_frame->set_istrackingenabled(controller_view->getIsTrackingEnabled());

        psmove_data_frame->mutable_orientation()->set_w(controller_pose.Orientation.w);
        psmove_data_frame->mutable_orientation()->set_x(controller_pose.Orientation.x);
        psmove_data_frame->mutable_orientation()->set_y(controller_pose.Orientation.y);
        psmove_data_frame->mutable_orientation()->set_z(controller_pose.Orientation.z);

        if (stream_info->include_position_data)
        {
            psmove_data_frame->mutable_position()->set_x(controller_pose.Position.x);
            psmove_data_frame->mutable_position()->set_y(controller_pose.Position.y);
            psmove_data_frame->mutable_position()->set_z(controller_pose.Position.z);
        }
        else
        {
            psmove_data_frame->mutable_position()->set_x(0);
            psmove_data_frame->mutable_position()->set_y(0);
            psmove_data_frame->mutable_position()->set_z(0);
        }

        psmove_data_frame->set_trigger_value(psmove_state->TriggerValue);

        unsigned int button_bitmask= 0;
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::TRIANGLE, psmove_state->Triangle);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::CIRCLE, psmove_state->Circle);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::CROSS, psmove_state->Cross);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::SQUARE, psmove_state->Square);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::SELECT, psmove_state->Select);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::START, psmove_state->Start);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::PS, psmove_state->PS);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::MOVE, psmove_state->Move);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::TRIGGER, psmove_state->Trigger);
        controller_data_frame->set_button_down_bitmask(button_bitmask);

        // If requested, get the raw sensor data for the controller
        if (stream_info->include_raw_sensor_data)
        {
            auto *raw_sensor_data= psmove_data_frame->mutable_raw_sensor_data();

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
            auto *raw_tracker_data = psmove_data_frame->mutable_raw_tracker_data();
            int valid_tracker_count= 0;

            for (int trackerId = 0; trackerId < TrackerManager::k_max_devices; ++trackerId)
            {
                const ControllerPositionEstimation *positionEstimate= 
                    controller_view->getTrackerPositionEstimate(trackerId);

                if (positionEstimate != nullptr && positionEstimate->bCurrentlyTracking)
                {
                    const CommonDevicePosition &trackerRelativePosition = positionEstimate->position;
                    const ServerTrackerViewPtr tracker_view = DeviceManager::getInstance()->getTrackerViewPtr(trackerId);

                    // Project the 3d camera position back onto the tracker screen
                    {
                        const CommonDeviceScreenLocation trackerScreenLocation =
                            tracker_view->projectTrackerRelativePosition(&trackerRelativePosition);
                        PSMoveProtocol::Pixel *pixel = raw_tracker_data->add_screen_locations();

                        pixel->set_x(trackerScreenLocation.x);
                        pixel->set_y(trackerScreenLocation.y);
                    }

                    // Add the tracker relative 3d position
                    {
                        PSMoveProtocol::Position *position= raw_tracker_data->add_relative_positions();
                        
                        position->set_x(trackerRelativePosition.x);
                        position->set_y(trackerRelativePosition.y);
                        position->set_z(trackerRelativePosition.z);
                    }

                    // Add the tracker relative projection shapes
                    {
                        const CommonDeviceTrackingProjection &trackerRelativeProjection = 
                            positionEstimate->projection;

                        switch (trackerRelativeProjection.shape_type)
                        {
                        case eCommonTrackingProjectionType::ProjectionType_Ellipse:
                            {
                                PSMoveProtocol::Ellipse *ellipse= raw_tracker_data->add_projected_spheres();
                                
                                ellipse->mutable_center()->set_x(trackerRelativeProjection.shape.ellipse.center.x);
                                ellipse->mutable_center()->set_y(trackerRelativeProjection.shape.ellipse.center.y);
                                ellipse->set_half_x_extent(trackerRelativeProjection.shape.ellipse.half_x_extent);
                                ellipse->set_half_y_extent(trackerRelativeProjection.shape.ellipse.half_y_extent);
                                ellipse->set_angle(trackerRelativeProjection.shape.ellipse.angle);
                            } break;
                        case eCommonTrackingProjectionType::ProjectionType_Quad:
                            {
                                PSMoveProtocol::Polygon *polygon = raw_tracker_data->add_projected_blobs();

                                for (int vert_index = 0; vert_index < 4; ++vert_index)
                                {
                                    PSMoveProtocol::Pixel *pixel= polygon->add_vertices();

                                    pixel->set_x(trackerRelativeProjection.shape.quad.corners[vert_index].x);
                                    pixel->set_x(trackerRelativeProjection.shape.quad.corners[vert_index].y);
                                }
                            } break;
                        }
                    }

                    raw_tracker_data->add_tracker_ids(trackerId);
                    ++valid_tracker_count;
                }
            }

            raw_tracker_data->set_valid_tracker_count(valid_tracker_count);
        }

        // if requested, get the physics data for the controller
        {
            const CommonDevicePhysics controller_physics = controller_view->getFilteredPhysics();
            auto *physics_data = psmove_data_frame->mutable_physics_data();

            physics_data->mutable_velocity()->set_i(controller_physics.Velocity.i);
            physics_data->mutable_velocity()->set_j(controller_physics.Velocity.j);
            physics_data->mutable_velocity()->set_k(controller_physics.Velocity.k);

            physics_data->mutable_acceleration()->set_i(controller_physics.Acceleration.i);
            physics_data->mutable_acceleration()->set_j(controller_physics.Acceleration.j);
            physics_data->mutable_acceleration()->set_k(controller_physics.Acceleration.k);

            physics_data->mutable_angular_velocity()->set_i(controller_physics.AngularVelocity.i);
            physics_data->mutable_angular_velocity()->set_j(controller_physics.AngularVelocity.j);
            physics_data->mutable_angular_velocity()->set_k(controller_physics.AngularVelocity.k);

            physics_data->mutable_angular_acceleration()->set_i(controller_physics.AngularAcceleration.i);
            physics_data->mutable_angular_acceleration()->set_j(controller_physics.AngularAcceleration.j);
            physics_data->mutable_angular_acceleration()->set_k(controller_physics.AngularAcceleration.k);
        }
    }   

    controller_data_frame->set_controller_type(PSMoveProtocol::PSMOVE);
}

static void generate_psnavi_data_frame_for_stream(
    const ServerControllerView *controller_view,
    const ControllerStreamInfo *stream_info,
    PSMoveProtocol::DeviceOutputDataFrame* data_frame)
{
    PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket *controller_data_frame = data_frame->mutable_controller_data_packet();
    PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_PSNaviState *psnavi_data_frame = controller_data_frame->mutable_psnavi_state();

    const CommonControllerState *controller_state= controller_view->getState();

    if (controller_state != nullptr)
    {
        assert(controller_state->DeviceType == CommonDeviceState::PSNavi);
        const PSNaviControllerState *psnavi_state= static_cast<const PSNaviControllerState *>(controller_state);

        psnavi_data_frame->set_trigger_value(psnavi_state->Trigger);
        psnavi_data_frame->set_stick_xaxis(psnavi_state->Stick_XAxis);
        psnavi_data_frame->set_stick_yaxis(psnavi_state->Stick_YAxis);

        unsigned int button_bitmask= 0;
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::L1, psnavi_state->L1);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::L2, psnavi_state->L2);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::L3, psnavi_state->L3);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::CIRCLE, psnavi_state->Circle);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::CROSS, psnavi_state->Cross);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::PS, psnavi_state->PS);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::UP, psnavi_state->DPad_Up);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::RIGHT, psnavi_state->DPad_Right);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::DOWN, psnavi_state->DPad_Down);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::LEFT, psnavi_state->DPad_Left);
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
        position_filter->setFusionType(PositionFilter::FusionTypeLowPass);
    }
}

static void                
update_filters_for_psmove(
    const PSMoveController *psmoveController, 
    const PSMoveControllerState *psmoveState,
    const float delta_time,
    const ControllerPositionEstimation *positionEstimation,
    OrientationFilter *orientationFilter,
    PositionFilter *position_filter)
{
    const PSMoveControllerConfig *config = psmoveController->getConfig();

    // Update the orientation filter
    if (orientationFilter != nullptr)
    {
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
            orientationFilter->update(delta_time / 2.f, sensorPacket);
        }
    }


    // Update the position filter
    if (position_filter != nullptr)
    {
        PositionSensorPacket sensorPacket;
        sensorPacket.position = 
            Eigen::Vector3f(
                positionEstimation->position.x,
                positionEstimation->position.y,
                positionEstimation->position.z);
        sensorPacket.bPositionValid = positionEstimation->bCurrentlyTracking;
        sensorPacket.acceleration = Eigen::Vector3f(0.f, 0.f, 0.f);

        position_filter->update(delta_time, sensorPacket);

        //###HipsterSloth $TODO Feed the world space controller acceleration into the filter
        //for (int frame = 0; frame < 2; ++frame)
        //{
        //    Eigen::Quaternionf orientation= orientationFilter->getOrientation();
        //    Eigen::Vector3f accelerometer =
        //        Eigen::Vector3f(psmoveState->Accel[frame][0], psmoveState->Accel[frame][1], psmoveState->Accel[frame][2]);
        //    Eigen::Vector3f acceleration= eigen_vector3f_clockwise_rotate(orientation, accelerometer);

        //    sensorPacket.acceleration= acceleration;

        //    // Update the orientation filter using the sensor packet.
        //    // NOTE: The magnetometer reading is the same for both sensor readings.
        //    position_filter->update(delta_time / 2.f, sensorPacket);
        //}
    }
}
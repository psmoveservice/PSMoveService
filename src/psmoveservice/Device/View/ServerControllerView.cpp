//-- includes -----
#include "ServerControllerView.h"

#include "BluetoothRequests.h"
#include "ControllerManager.h"
#include "DeviceManager.h"
#include "MathAlignment.h"
#include "ServerLog.h"
#include "ServerRequestHandler.h"
#include "CompoundPoseFilter.h"
#include "KalmanPoseFilter.h"
#include "PoseFilterInterface.h"
#include "PSDualShock4Controller.h"
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
	PoseFilterSpace **out_pose_filter_space,
    IPoseFilter **out_pose_filter);
static void update_filters_for_psmove(
    const PSMoveController *psmoveController, const PSMoveControllerState *psmoveState, const float delta_time,
    const ControllerOpticalPoseEstimation *positionEstimation, 
	const PoseFilterSpace *poseFilterSpace,
    IPoseFilter *pose_filter);

static void init_filters_for_psdualshock4(
    const PSDualShock4Controller *psdualshock4Controller, 
	PoseFilterSpace **out_pose_filter_space,
	IPoseFilter **out_pose_filter);
static void update_filters_for_psdualshock4(
    const PSDualShock4Controller *psdualshock4Controller, const PSDualShock4ControllerState *psmoveState, const float delta_time,
    const ControllerOpticalPoseEstimation *positionEstimation,
	const PoseFilterSpace *poseFilterSpace,
    IPoseFilter *pose_filter);

static void generate_psmove_data_frame_for_stream(
    const ServerControllerView *controller_view, const ControllerStreamInfo *stream_info, DeviceOutputDataFramePtr &data_frame);
static void generate_psnavi_data_frame_for_stream(
    const ServerControllerView *controller_view, const ControllerStreamInfo *stream_info, DeviceOutputDataFramePtr &data_frame);
static void generate_psdualshock4_data_frame_for_stream(
    const ServerControllerView *controller_view, const ControllerStreamInfo *stream_info, DeviceOutputDataFramePtr &data_frame);

//-- public implementation -----
ServerControllerView::ServerControllerView(const int device_id)
    : ServerDeviceView(device_id)
    , m_tracking_color_id(eCommonTrackingColorID::INVALID_COLOR)
    , m_tracking_listener_count(0)
    , m_tracking_enabled(false)
    , m_LED_override_active(false)
    , m_device(nullptr)
    , m_tracker_pose_estimation(nullptr)
    , m_multicam_pose_estimation(nullptr)
    , m_pose_filter(nullptr)
    , m_pose_filter_space(nullptr)
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

			init_filters_for_psmove(
				static_cast<PSMoveController *>(m_device), 
				&m_pose_filter_space, &m_pose_filter);

            m_tracker_pose_estimation = new ControllerOpticalPoseEstimation[TrackerManager::k_max_devices];
            for (int tracker_index = 0; tracker_index < TrackerManager::k_max_devices; ++tracker_index)
            {
                m_tracker_pose_estimation[tracker_index].clear();
            }

            m_multicam_pose_estimation = new ControllerOpticalPoseEstimation();
            m_multicam_pose_estimation->clear();
        } break;
    case CommonDeviceState::PSNavi:
        {
            m_device= new PSNaviController();
            m_pose_filter= nullptr;
            m_multicam_pose_estimation = nullptr;
        } break;
    case CommonDeviceState::PSDualShock4:
        {
            m_device = new PSDualShock4Controller();

			init_filters_for_psdualshock4(
				static_cast<PSDualShock4Controller *>(m_device),
				&m_pose_filter_space, &m_pose_filter);

            m_tracker_pose_estimation = new ControllerOpticalPoseEstimation[TrackerManager::k_max_devices];
            for (int tracker_index = 0; tracker_index < TrackerManager::k_max_devices; ++tracker_index)
            {
                m_tracker_pose_estimation[tracker_index].clear();
            }

            m_multicam_pose_estimation = new ControllerOpticalPoseEstimation();
            m_multicam_pose_estimation->clear();
        } break;
    default:
        break;
    }

    return m_device != nullptr;
}

void ServerControllerView::free_device_interface()
{
    if (m_multicam_pose_estimation != nullptr)
    {
        delete m_multicam_pose_estimation;
        m_multicam_pose_estimation= nullptr;
    }

    if (m_tracker_pose_estimation != nullptr)
    {
        delete[] m_tracker_pose_estimation;
        m_tracker_pose_estimation = nullptr;
    }

    if (m_pose_filter != nullptr)
    {
        delete m_pose_filter;
        m_pose_filter= nullptr;
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
                    m_pose_filter->resetState();
                    m_multicam_pose_estimation->clear();

                    bAllocateTrackingColor = true;
                }
            } break;
        case CommonDeviceState::PSNavi:
            // No orientation filter for the navi
            assert(m_pose_filter == nullptr);
            break;
        case CommonDeviceState::PSDualShock4:
            {
                const PSDualShock4Controller *psdualshock4Controller = this->castCheckedConst<PSDualShock4Controller>();

                // Don't bother initializing any filters or allocating a tracking color
                // for usb connected controllers
                if (psdualshock4Controller->getIsBluetooth())
                {
					m_pose_filter->resetState();
                    m_multicam_pose_estimation->clear();

                    bAllocateTrackingColor = true;
                }
            } break;
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

void ServerControllerView::updateOpticalPoseEstimation(TrackerManager* tracker_manager)
{
    const std::chrono::time_point<std::chrono::high_resolution_clock> now= std::chrono::high_resolution_clock::now();

    // TODO: Probably need to first update IMU state to get velocity.
    // If velocity is too high, don't bother getting a new position.
    // Though it may be enough to just use the camera ROI as the limit.
    
    if (getIsTrackingEnabled())
    {
        Eigen::Quaternionf controller_world_orientations[TrackerManager::k_max_devices];
        float controller_orientation_weights[TrackerManager::k_max_devices];
        int orientations_found = 0;

        int valid_position_tracker_ids[TrackerManager::k_max_devices];
        int positions_found = 0;

        float screen_area_sum= 0;

        // Compute an estimated 3d tracked position of the controller 
        // from the perspective of each tracker
        for (int tracker_id = 0; tracker_id < tracker_manager->getMaxDevices(); ++tracker_id)
        {
            ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);
            ControllerOpticalPoseEstimation &trackerPoseEstimateRef = m_tracker_pose_estimation[tracker_id];

            const bool bWasTracking= trackerPoseEstimateRef.bCurrentlyTracking;

            // Assume we're going to lose tracking this frame
            trackerPoseEstimateRef.bCurrentlyTracking = false;

            if (tracker->getIsOpen())
            {
                // See how long it's been since we got a new video frame
                const std::chrono::time_point<std::chrono::high_resolution_clock> now= 
                    std::chrono::high_resolution_clock::now();
                const std::chrono::duration<float, std::milli> timeSinceNewDataMillis= 
                    now - tracker->getLastNewDataTimestamp();
                const float timeoutMilli= 
                    static_cast<float>(DeviceManager::getInstance()->m_tracker_manager->getConfig().optical_tracking_timeout);

                // Can't compute tracking on video data that's too old
                if (timeSinceNewDataMillis.count() < timeoutMilli)
                {
                    // Initially the newTrackerPoseEstimate is a copy of the existing pose
                    bool bIsVisibleThisUpdate= false;

                    // If a new video frame is available this tick, 
                    // attempt to update the tracking location
                    if (tracker->getHasUnpublishedState())
                    {
                        ControllerOpticalPoseEstimation newTrackerPoseEstimate= trackerPoseEstimateRef;
                        CommonDevicePose poseGuess= {trackerPoseEstimateRef.position, trackerPoseEstimateRef.orientation};

                        if (tracker->computePoseForController(
                                this, 
                                trackerPoseEstimateRef.bOrientationValid ? &poseGuess : nullptr,
                                &newTrackerPoseEstimate))
                        {
                            bIsVisibleThisUpdate= true;

                            trackerPoseEstimateRef= newTrackerPoseEstimate;
                            trackerPoseEstimateRef.last_visible_timestamp = now;
                        }
                    }

                    // If the position estimate isn't too old (or updated this tick), 
                    // say we have a valid tracked location
                    if (bWasTracking || bIsVisibleThisUpdate)
                    {
                        const std::chrono::duration<float, std::milli> timeSinceLastVisibleMillis= 
                            now - trackerPoseEstimateRef.last_visible_timestamp;

                        if (timeSinceLastVisibleMillis.count() < timeoutMilli)
                        {
                            const float tracker_screen_area= trackerPoseEstimateRef.projection.screen_area;

                            // Sum up the tracking screen area over all of the trackers that can see the controller
                            screen_area_sum+= tracker_screen_area;

                            // If this tracker has a valid position for the controller
                            // add it to the tracker id list
                            valid_position_tracker_ids[positions_found] = tracker_id;
                            ++positions_found;

                            // If the pose has a valid tracker relative orientation,
                            // convert the orientation to world space and add it
                            // to a weighted list of orientations
                            if (trackerPoseEstimateRef.bOrientationValid)
                            {
                                const CommonDeviceQuaternion &tracker_relative_quaternion = trackerPoseEstimateRef.orientation;
                                const CommonDeviceQuaternion &world_quaternion =
                                    tracker->computeWorldOrientation(&tracker_relative_quaternion);
                                const Eigen::Quaternionf eigen_quaternion(
                                    world_quaternion.w, world_quaternion.x, world_quaternion.y, world_quaternion.z);

                                controller_world_orientations[orientations_found]= eigen_quaternion;
                                controller_orientation_weights[orientations_found]= tracker_screen_area;
                                ++orientations_found;
                            }

                            // Flag this pose estimate as invalid
                            trackerPoseEstimateRef.bCurrentlyTracking = true;
                        }
                    }
                }
            }

            // Keep track of the last time the position estimate was updated
            trackerPoseEstimateRef.last_update_timestamp = now;
            trackerPoseEstimateRef.bValidTimestamps = true;
        }

        // If multiple trackers can see the controller, 
        // triangulate all pairs of trackers and average the results
        if (positions_found > 1)
        {
            // Project the tracker relative 3d tracking position back on to the tracker camera plane
            CommonDeviceScreenLocation position2d_list[TrackerManager::k_max_devices];
            for (int list_index = 0; list_index < positions_found; ++list_index)
            {
                const int tracker_id = valid_position_tracker_ids[list_index];
                const ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);
                const ControllerOpticalPoseEstimation &positionEstimate = m_tracker_pose_estimation[tracker_id];
                
                position2d_list[list_index] = tracker->projectTrackerRelativePosition(&positionEstimate.position);
            }

            int pair_count = 0;
            CommonDevicePosition average_world_position = { 0.f, 0.f, 0.f };
            for (int list_index = 0; list_index < positions_found; ++list_index)
            {
                const int tracker_id = valid_position_tracker_ids[list_index];
                const CommonDeviceScreenLocation &screen_location = position2d_list[list_index];
                const ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);

                for (int other_list_index = list_index + 1; other_list_index < positions_found; ++other_list_index)
                {
                    const int other_tracker_id = valid_position_tracker_ids[other_list_index];
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

            // Store the averaged tracking position
            m_multicam_pose_estimation->position = average_world_position;
            m_multicam_pose_estimation->bCurrentlyTracking = true;

            // Compute the average projection area.
            // This is proportional to our position tracking quality.
            m_multicam_pose_estimation->projection.screen_area= 
                screen_area_sum / static_cast<float>(positions_found);
        }
        // If only one tracker can see the controller, then just use the position estimate from that
        else if (positions_found == 1)
        {
            // Put the tracker relative position into world space
            const int tracker_id = valid_position_tracker_ids[0];
            const ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);
            const CommonDevicePosition &tracker_relative_position = m_tracker_pose_estimation[tracker_id].position;

            // Only one tracker can see the controller
            m_multicam_pose_estimation->position = tracker->computeWorldPosition(&tracker_relative_position);
            m_multicam_pose_estimation->bCurrentlyTracking = true;

            // The average screen area is just the sum
            m_multicam_pose_estimation->projection.screen_area= screen_area_sum;
        }
        // If no trackers can see the controller, maintain the last known position and time it was seen
        else
        {
            m_multicam_pose_estimation->bCurrentlyTracking= false;
        }

        // Compute a weighted average of all of the orientations we found.
        // Weighted by the projection area (higher projection area proportional to better quality orientation)        
        if (orientations_found > 0)
        {
            Eigen::Quaternionf avg_world_orientation;

            if (eigen_quaternion_compute_weighted_average(
                controller_world_orientations,
                controller_orientation_weights,
                orientations_found,
                &avg_world_orientation))
            {
                m_multicam_pose_estimation->orientation.w= avg_world_orientation.w();
                m_multicam_pose_estimation->orientation.x= avg_world_orientation.x();
                m_multicam_pose_estimation->orientation.y= avg_world_orientation.y();
                m_multicam_pose_estimation->orientation.z= avg_world_orientation.z();
                m_multicam_pose_estimation->bOrientationValid= true;
            }
            else
            {
                m_multicam_pose_estimation->bOrientationValid= false;
            }
        }
        else
        {
            m_multicam_pose_estimation->bOrientationValid= false;
        }

        // Update the position estimation timestamps
        if (positions_found > 0)
        {
            m_multicam_pose_estimation->last_visible_timestamp = now;
        }
        m_multicam_pose_estimation->last_update_timestamp = now;
        m_multicam_pose_estimation->bValidTimestamps = true;
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
                    m_multicam_pose_estimation,
					m_pose_filter_space,
                    m_pose_filter);
            } break;
        case CommonControllerState::PSNavi:
            {
                // No orientation or position to update
                assert(m_pose_filter == nullptr);
            } break;
        case CommonControllerState::PSDualShock4:
            {
                const PSDualShock4Controller *psdualshock4Controller = this->castCheckedConst<PSDualShock4Controller>();
                const PSDualShock4ControllerState *psdualshock4State = 
                    static_cast<const PSDualShock4ControllerState *>(controllerState);

                // Only update the position filter when tracking is enabled
                update_filters_for_psdualshock4(
                    psdualshock4Controller, psdualshock4State,
                    per_state_time_delta_seconds,
                    m_multicam_pose_estimation,
					m_pose_filter_space,
                    m_pose_filter);
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

    if (m_pose_filter != nullptr)
    {
        const Eigen::Quaternionf orientation= m_pose_filter->getOrientation(time);
        const Eigen::Vector3f position= m_pose_filter->getPosition(time);

        pose.Orientation.w= orientation.w();
        pose.Orientation.x= orientation.x();
        pose.Orientation.y= orientation.y();
        pose.Orientation.z= orientation.z();

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

    if (m_pose_filter != nullptr)
    {
        const Eigen::Vector3f first_derivative= m_pose_filter->getAngularVelocity();
        const Eigen::Vector3f second_derivative= m_pose_filter->getAngularAcceleration();
        const Eigen::Vector3f velocity(m_pose_filter->getVelocity());
        const Eigen::Vector3f acceleration(m_pose_filter->getAcceleration());

        physics.AngularVelocity.i = first_derivative.x();
        physics.AngularVelocity.j = first_derivative.y();
        physics.AngularVelocity.k = first_derivative.z();

        physics.AngularAcceleration.i = second_derivative.x();
        physics.AngularAcceleration.j = second_derivative.y();
        physics.AngularAcceleration.k = second_derivative.z();

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

// Returns the "controller_" + serial number for the controller
std::string
ServerControllerView::getConfigIdentifier() const
{
	std::string	identifier= "";

	if (m_device != nullptr)
	{
		std::string	prefix= "controller_";
		
		identifier= prefix+m_device->getSerial();
	}

	return identifier;
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
    case CommonDeviceState::PSDualShock4:
        {
            this->castChecked<PSDualShock4Controller>()->setLED(r, g, b);
        } break;
    default:
        assert(false && "Unhanded controller type!");
    }
}

// Get the tracking shape for the controller
bool ServerControllerView::getTrackingShape(CommonDeviceTrackingShape &trackingShape) const
{
    m_device->getTrackingShape(trackingShape);

    return trackingShape.shape_type != eCommonTrackingShapeType::INVALID_SHAPE;
}

// Set the rumble value between 0.f - 1.f on a given channel
bool ServerControllerView::setControllerRumble(
    float rumble_amount,
    CommonControllerState::RumbleChannel channel)
{
    bool result= false;

    if (getIsOpen())
    {
        switch(getControllerDeviceType())
        {
        case CommonDeviceState::PSMove:
            {
                unsigned char rumble_byte= static_cast<unsigned char>(clampf01(rumble_amount)*255.f);

                static_cast<PSMoveController *>(m_device)->setRumbleIntensity(rumble_byte);
                result = true;
            } break;

        case CommonDeviceState::PSNavi:
            {
                result= false; // No rumble on the navi
            } break;

        case CommonDeviceState::PSDualShock4:
            {
                unsigned char rumble_byte = static_cast<unsigned char>(clampf01(rumble_amount)*255.f);
                PSDualShock4Controller *controller= static_cast<PSDualShock4Controller *>(m_device);

                if (channel == CommonControllerState::RumbleChannel::ChannelLeft ||
                    channel == CommonControllerState::RumbleChannel::ChannelAll)
                {
                    controller->setLeftRumbleIntensity(rumble_byte);
                }

                if (channel == CommonControllerState::RumbleChannel::ChannelRight ||
                    channel == CommonControllerState::RumbleChannel::ChannelAll)
                {
                    controller->setRightRumbleIntensity(rumble_byte);
                }

                result = true;
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
    DeviceOutputDataFramePtr &data_frame)
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
    case CommonControllerState::PSDualShock4:
        {
            generate_psdualshock4_data_frame_for_stream(controller_view, stream_info, data_frame);
        } break;
    default:
        assert(0 && "Unhandled controller type");
    }

    data_frame->set_device_category(PSMoveProtocol::DeviceOutputDataFrame::CONTROLLER);
}

static void generate_psmove_data_frame_for_stream(
    const ServerControllerView *controller_view,
    const ControllerStreamInfo *stream_info,
    DeviceOutputDataFramePtr &data_frame)
{
    const PSMoveController *psmove_controller= controller_view->castCheckedConst<PSMoveController>();
    const IPoseFilter *pose_filter= controller_view->getPoseFilter();
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
		//TODO: Collapse these two flags down into isPoseValid
        psmove_data_frame->set_isorientationvalid(pose_filter->getIsStateValid());
        psmove_data_frame->set_ispositionvalid(pose_filter->getIsStateValid());

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
            raw_sensor_data->mutable_magnetometer()->set_i(psmove_state->RawMag[0]);
            raw_sensor_data->mutable_magnetometer()->set_j(psmove_state->RawMag[1]);
            raw_sensor_data->mutable_magnetometer()->set_k(psmove_state->RawMag[2]);

            // Two frames: [[ax0, ay0, az0], [ax1, ay1, az1]] 
            // Take the most recent frame: [ax1, ay1, az1]
            raw_sensor_data->mutable_accelerometer()->set_i(psmove_state->RawAccel[1][0]);
            raw_sensor_data->mutable_accelerometer()->set_j(psmove_state->RawAccel[1][1]);
            raw_sensor_data->mutable_accelerometer()->set_k(psmove_state->RawAccel[1][2]);

            // Two frames: [[wx0, wy0, wz0], [wx1, wy1, wz1]] 
            // Take the most recent frame: [wx1, wy1, wz1]
            raw_sensor_data->mutable_gyroscope()->set_i(psmove_state->RawGyro[1][0]);
            raw_sensor_data->mutable_gyroscope()->set_j(psmove_state->RawGyro[1][1]);
            raw_sensor_data->mutable_gyroscope()->set_k(psmove_state->RawGyro[1][2]);
        }

        // If requested, get the calibrated sensor data for the controller
        if (stream_info->include_calibrated_sensor_data)
        {
            auto *calibrated_sensor_data = psmove_data_frame->mutable_calibrated_sensor_data();

            // One frame: [mx, my, mz] 
            calibrated_sensor_data->mutable_magnetometer()->set_i(psmove_state->CalibratedMag[0]);
            calibrated_sensor_data->mutable_magnetometer()->set_j(psmove_state->CalibratedMag[1]);
            calibrated_sensor_data->mutable_magnetometer()->set_k(psmove_state->CalibratedMag[2]);

            // Two frames: [[ax0, ay0, az0], [ax1, ay1, az1]] 
            // Take the most recent frame: [ax1, ay1, az1]
            calibrated_sensor_data->mutable_accelerometer()->set_i(psmove_state->CalibratedAccel[1][0]);
            calibrated_sensor_data->mutable_accelerometer()->set_j(psmove_state->CalibratedAccel[1][1]);
            calibrated_sensor_data->mutable_accelerometer()->set_k(psmove_state->CalibratedAccel[1][2]);

            // Two frames: [[wx0, wy0, wz0], [wx1, wy1, wz1]] 
            // Take the most recent frame: [wx1, wy1, wz1]
            calibrated_sensor_data->mutable_gyroscope()->set_i(psmove_state->CalibratedGyro[1][0]);
            calibrated_sensor_data->mutable_gyroscope()->set_j(psmove_state->CalibratedGyro[1][1]);
            calibrated_sensor_data->mutable_gyroscope()->set_k(psmove_state->CalibratedGyro[1][2]);
        }

        // If requested, get the raw tracker data for the controller
        if (stream_info->include_raw_tracker_data)
        {
            auto *raw_tracker_data = psmove_data_frame->mutable_raw_tracker_data();
            int valid_tracker_count= 0;

            for (int trackerId = 0; trackerId < TrackerManager::k_max_devices; ++trackerId)
            {
                const ControllerOpticalPoseEstimation *positionEstimate= 
                    controller_view->getTrackerPoseEstimate(trackerId);

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

                        assert(trackerRelativeProjection.shape_type == eCommonTrackingProjectionType::ProjectionType_Ellipse);
                        PSMoveProtocol::Ellipse *ellipse= raw_tracker_data->add_projected_spheres();
                                
                        ellipse->mutable_center()->set_x(trackerRelativeProjection.shape.ellipse.center.x);
                        ellipse->mutable_center()->set_y(trackerRelativeProjection.shape.ellipse.center.y);
                        ellipse->set_half_x_extent(trackerRelativeProjection.shape.ellipse.half_x_extent);
                        ellipse->set_half_y_extent(trackerRelativeProjection.shape.ellipse.half_y_extent);
                        ellipse->set_angle(trackerRelativeProjection.shape.ellipse.angle);
                    }

                    raw_tracker_data->add_tracker_ids(trackerId);
                    ++valid_tracker_count;
                }
            }

            raw_tracker_data->set_valid_tracker_count(valid_tracker_count);
        }

        // if requested, get the physics data for the controller
        if (stream_info->include_physics_data)
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
    DeviceOutputDataFramePtr &data_frame)
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

static void generate_psdualshock4_data_frame_for_stream(
    const ServerControllerView *controller_view,
    const ControllerStreamInfo *stream_info,
    DeviceOutputDataFramePtr &data_frame)
{
    const PSDualShock4Controller *ds4_controller = controller_view->castCheckedConst<PSDualShock4Controller>();
    const IPoseFilter *pose_filter= controller_view->getPoseFilter();
    const PSDualShock4ControllerConfig *psmove_config = ds4_controller->getConfig();
    const CommonControllerState *controller_state = controller_view->getState();
    const CommonDevicePose controller_pose = controller_view->getFilteredPose(psmove_config->prediction_time);

    auto *controller_data_frame = data_frame->mutable_controller_data_packet();
    auto *psds4_data_frame = controller_data_frame->mutable_psdualshock4_state();

    if (controller_state != nullptr)
    {
        assert(controller_state->DeviceType == CommonDeviceState::PSDualShock4);
        const PSDualShock4ControllerState * psds4_state = static_cast<const PSDualShock4ControllerState *>(controller_state);

        psds4_data_frame->set_validhardwarecalibration(psmove_config->is_valid);
        psds4_data_frame->set_iscurrentlytracking(controller_view->getIsCurrentlyTracking());
        psds4_data_frame->set_istrackingenabled(controller_view->getIsTrackingEnabled());
		//TODO: Collapse these two flags down into isPoseValid
        psds4_data_frame->set_isorientationvalid(pose_filter->getIsStateValid());
        psds4_data_frame->set_ispositionvalid(pose_filter->getIsStateValid());

        psds4_data_frame->mutable_orientation()->set_w(controller_pose.Orientation.w);
        psds4_data_frame->mutable_orientation()->set_x(controller_pose.Orientation.x);
        psds4_data_frame->mutable_orientation()->set_y(controller_pose.Orientation.y);
        psds4_data_frame->mutable_orientation()->set_z(controller_pose.Orientation.z);

        if (stream_info->include_position_data)
        {
            psds4_data_frame->mutable_position()->set_x(controller_pose.Position.x);
            psds4_data_frame->mutable_position()->set_y(controller_pose.Position.y);
            psds4_data_frame->mutable_position()->set_z(controller_pose.Position.z);
        }
        else
        {
            psds4_data_frame->mutable_position()->set_x(0);
            psds4_data_frame->mutable_position()->set_y(0);
            psds4_data_frame->mutable_position()->set_z(0);
        }

        psds4_data_frame->set_left_thumbstick_x(psds4_state->LeftAnalogX);
        psds4_data_frame->set_left_thumbstick_y(psds4_state->LeftAnalogY);

        psds4_data_frame->set_right_thumbstick_x(psds4_state->RightAnalogX);
        psds4_data_frame->set_right_thumbstick_y(psds4_state->RightAnalogY);

        psds4_data_frame->set_left_trigger_value(psds4_state->LeftTrigger);
        psds4_data_frame->set_right_trigger_value(psds4_state->RightTrigger);

        unsigned int button_bitmask = 0;
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::UP, psds4_state->DPad_Up);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::DOWN, psds4_state->DPad_Down);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::LEFT, psds4_state->DPad_Left);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::RIGHT, psds4_state->DPad_Right);

        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::L1, psds4_state->L1);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::R1, psds4_state->R1);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::L2, psds4_state->L2);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::R2, psds4_state->R2);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::L3, psds4_state->L3);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::R3, psds4_state->R3);

        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::TRIANGLE, psds4_state->Triangle);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::CIRCLE, psds4_state->Circle);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::CROSS, psds4_state->Cross);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::SQUARE, psds4_state->Square);

        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::SHARE, psds4_state->Share);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::OPTIONS, psds4_state->Options);

        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::PS, psds4_state->PS);
        SET_BUTTON_BIT(button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket::TRACKPAD, psds4_state->TrackPadButton);
        controller_data_frame->set_button_down_bitmask(button_bitmask);

        // If requested, get the raw sensor data for the controller
        if (stream_info->include_raw_sensor_data)
        {
            auto *raw_sensor_data = psds4_data_frame->mutable_raw_sensor_data();

            raw_sensor_data->mutable_accelerometer()->set_i(psds4_state->RawAccelerometer[0]);
            raw_sensor_data->mutable_accelerometer()->set_j(psds4_state->RawAccelerometer[1]);
            raw_sensor_data->mutable_accelerometer()->set_k(psds4_state->RawAccelerometer[2]);

            raw_sensor_data->mutable_gyroscope()->set_i(psds4_state->RawGyro[0]);
            raw_sensor_data->mutable_gyroscope()->set_j(psds4_state->RawGyro[1]);
            raw_sensor_data->mutable_gyroscope()->set_k(psds4_state->RawGyro[2]);
        }

        // If requested, get the calibrated sensor data for the controller
        if (stream_info->include_calibrated_sensor_data)
        {
            auto *calibrated_sensor_data = psds4_data_frame->mutable_calibrated_sensor_data();

            calibrated_sensor_data->mutable_accelerometer()->set_i(psds4_state->CalibratedAccelerometer.i);
            calibrated_sensor_data->mutable_accelerometer()->set_j(psds4_state->CalibratedAccelerometer.j);
            calibrated_sensor_data->mutable_accelerometer()->set_k(psds4_state->CalibratedAccelerometer.k);

            calibrated_sensor_data->mutable_gyroscope()->set_i(psds4_state->CalibratedGyro.i);
            calibrated_sensor_data->mutable_gyroscope()->set_j(psds4_state->CalibratedGyro.j);
            calibrated_sensor_data->mutable_gyroscope()->set_k(psds4_state->CalibratedGyro.k);
        }

        // If requested, get the raw tracker data for the controller
        if (stream_info->include_raw_tracker_data)
        {
            auto *raw_tracker_data = psds4_data_frame->mutable_raw_tracker_data();
            int valid_tracker_count = 0;

            for (int trackerId = 0; trackerId < TrackerManager::k_max_devices; ++trackerId)
            {
                const ControllerOpticalPoseEstimation *poseEstimate =
                    controller_view->getTrackerPoseEstimate(trackerId);

                if (poseEstimate != nullptr && poseEstimate->bCurrentlyTracking)
                {
                    const CommonDevicePosition &trackerRelativePosition = poseEstimate->position;
                    const CommonDeviceQuaternion &trackerRelativeOrientation = poseEstimate->orientation;
                    const ServerTrackerViewPtr tracker_view = DeviceManager::getInstance()->getTrackerViewPtr(trackerId);

                    // Project the 3d camera position back onto the tracker screen
                    {
                        const CommonDeviceScreenLocation trackerScreenLocation =
                            tracker_view->projectTrackerRelativePosition(&trackerRelativePosition);
                        PSMoveProtocol::Pixel *pixel = raw_tracker_data->add_screen_locations();

                        pixel->set_x(trackerScreenLocation.x);
                        pixel->set_y(trackerScreenLocation.y);
                    }

                    // Add the tracker relative 3d pose
                    {
                        PSMoveProtocol::Position *position = raw_tracker_data->add_relative_positions();
                        PSMoveProtocol::Orientation *orientation = raw_tracker_data->add_relative_orientations();

                        position->set_x(trackerRelativePosition.x);
                        position->set_y(trackerRelativePosition.y);
                        position->set_z(trackerRelativePosition.z);

                        orientation->set_w(trackerRelativeOrientation.w);
                        orientation->set_x(trackerRelativeOrientation.x);
                        orientation->set_y(trackerRelativeOrientation.y);
                        orientation->set_z(trackerRelativeOrientation.z);
                    }

                    // Add the tracker relative projection shapes
                    {
                        const CommonDeviceTrackingProjection &trackerRelativeProjection =
                            poseEstimate->projection;

                        assert(trackerRelativeProjection.shape_type == eCommonTrackingProjectionType::ProjectionType_LightBar);
                        PSMoveProtocol::Polygon *polygon = raw_tracker_data->add_projected_blobs();

                        for (int vert_index = 0; vert_index < 3; ++vert_index)
                        {
                            PSMoveProtocol::Pixel *pixel = polygon->add_vertices();

                            pixel->set_x(trackerRelativeProjection.shape.lightbar.triangle[vert_index].x);
                            pixel->set_y(trackerRelativeProjection.shape.lightbar.triangle[vert_index].y);
                        }

                        for (int vert_index = 0; vert_index < 4; ++vert_index)
                        {
                            PSMoveProtocol::Pixel *pixel = polygon->add_vertices();

                            pixel->set_x(trackerRelativeProjection.shape.lightbar.quad[vert_index].x);
                            pixel->set_y(trackerRelativeProjection.shape.lightbar.quad[vert_index].y);
                        }
                    }

                    raw_tracker_data->add_tracker_ids(trackerId);
                    ++valid_tracker_count;
                }
            }

            raw_tracker_data->set_valid_tracker_count(valid_tracker_count);
        }

        // if requested, get the physics data for the controller
        if (stream_info->include_physics_data)
        {
            const CommonDevicePhysics controller_physics = controller_view->getFilteredPhysics();
            auto *physics_data = psds4_data_frame->mutable_physics_data();

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

    controller_data_frame->set_controller_type(PSMoveProtocol::PSDUALSHOCK4);
}

static void
init_filters_for_psmove(
    const PSMoveController *psmoveController,
	PoseFilterSpace **out_pose_filter_space,
    IPoseFilter **out_pose_filter)
{
    const PSMoveControllerConfig *psmove_config = psmoveController->getConfig();

    // Setup the space the orientation filter operates in
    PoseFilterSpace *pose_filter_space = new PoseFilterSpace();
    pose_filter_space->setIdentityGravity(Eigen::Vector3f(0.f, 1.f, 0.f));
	pose_filter_space->setIdentityMagnetometer(
        Eigen::Vector3f(psmove_config->magnetometer_identity.i,
						psmove_config->magnetometer_identity.j,
						psmove_config->magnetometer_identity.k));
	pose_filter_space->setCalibrationTransform(*k_eigen_identity_pose_laying_flat);
	pose_filter_space->setSensorTransform(*k_eigen_sensor_transform_opengl);

	// Copy the pose filter constants from the controller config
	PoseFilterConstants constants;
	constants.orientation_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
	constants.orientation_constants.magnetometer_calibration_direction = pose_filter_space->getMagnetometerCalibrationDirection();
	constants.orientation_constants.gyro_drift= psmove_config->gyro_drift;
	constants.orientation_constants.gyro_error= psmove_config->gyro_variance;
	constants.position_constants.accelerometer_noise_radius= psmove_config->accelerometer_noise_radius;
	constants.position_constants.max_velocity= psmove_config->max_velocity;

	// TODO: Allow the config to select the filter type
	// For now hard code the usage of a compound pose filter
    CompoundPoseFilter *pose_filter = new CompoundPoseFilter();
	pose_filter->init(OrientationFilterTypeComplementaryMARG, PositionFilterTypeLowPassOptical, constants);

	*out_pose_filter_space= pose_filter_space;
	*out_pose_filter= pose_filter;
}

static void                
update_filters_for_psmove(
    const PSMoveController *psmoveController, 
    const PSMoveControllerState *psmoveState,
    const float delta_time,
    const ControllerOpticalPoseEstimation *poseEstimation,
	const PoseFilterSpace *poseFilterSpace,
    IPoseFilter *poseFilter)
{
    const PSMoveControllerConfig *config = psmoveController->getConfig();
    Eigen::Quaternionf orientationFrames[2] = {Eigen::Quaternionf::Identity(), Eigen::Quaternionf::Identity()};

    // Update the orientation filter
    if (poseFilter != nullptr)
    {
        PoseSensorPacket sensorPacket;

		// PSMove cant do optical orientation
        sensorPacket.optical_orientation = Eigen::Quaternionf::Identity();
        sensorPacket.optical_orientation_quality= 0.f;

		// PSMove does have an optical position
		if (poseEstimation->bCurrentlyTracking)
		{
			sensorPacket.optical_position =
				Eigen::Vector3f(
					poseEstimation->position.x,
					poseEstimation->position.y,
					poseEstimation->position.z);
			sensorPacket.optical_position_quality=
				clampf01(
					safe_divide_with_default(
						poseEstimation->projection.screen_area - config->min_position_quality_screen_area,
						config->max_position_quality_screen_area - config->min_position_quality_screen_area,
						1.f));
		}
		else
		{
			sensorPacket.optical_position = Eigen::Vector3f::Zero();
			sensorPacket.optical_position_quality= 0.f;
		}

		// One magnetometer update for every two accel/gryo readings
        sensorPacket.imu_magnetometer =
            Eigen::Vector3f(
                psmoveState->CalibratedMag[0],
                psmoveState->CalibratedMag[1],
                psmoveState->CalibratedMag[2]);

        // Each state update contains two readings (one earlier and one later) of accelerometer and gyro data
        for (int frame = 0; frame < 2; ++frame)
        {
			PoseFilterPacket filterPacket;
			
            sensorPacket.imu_accelerometer =
                Eigen::Vector3f(
                    psmoveState->CalibratedAccel[frame][0], 
                    psmoveState->CalibratedAccel[frame][1], 
                    psmoveState->CalibratedAccel[frame][2]);
            sensorPacket.imu_gyroscope =
                Eigen::Vector3f(
                    psmoveState->CalibratedGyro[frame][0], 
                    psmoveState->CalibratedGyro[frame][1], 
                    psmoveState->CalibratedGyro[frame][2]);

			// Create a filter input packet from the sensor data 
			// and the filter's previous orientation and position
			poseFilterSpace->createFilterPacket(
				sensorPacket, 
				poseFilter->getOrientation(), poseFilter->getPosition(),
				filterPacket);

            poseFilter->update(delta_time / 2.f, filterPacket);
        }
    }
}

static void
init_filters_for_psdualshock4(
    const PSDualShock4Controller *psmoveController,
	PoseFilterSpace **out_pose_filter_space,
    IPoseFilter **out_pose_filter)
{
    const PSDualShock4ControllerConfig *ds4_config = psmoveController->getConfig();

    // Setup the space the orientation filter operates in
    PoseFilterSpace *pose_filter_space = new PoseFilterSpace();
    pose_filter_space->setIdentityGravity(
		Eigen::Vector3f(
            ds4_config->identity_gravity_direction.i,
            ds4_config->identity_gravity_direction.j,
            ds4_config->identity_gravity_direction.k));
	pose_filter_space->setIdentityMagnetometer(Eigen::Vector3f::Zero());  // No magnetometer on DS4 :(
	pose_filter_space->setCalibrationTransform(*k_eigen_identity_pose_upright);
	pose_filter_space->setSensorTransform(*k_eigen_sensor_transform_identity);

	// Copy the pose filter constants from the controller config
	PoseFilterConstants constants;
	constants.orientation_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
	constants.orientation_constants.magnetometer_calibration_direction = pose_filter_space->getMagnetometerCalibrationDirection();
	constants.orientation_constants.gyro_drift= ds4_config->gyro_drift;
	constants.orientation_constants.gyro_error= ds4_config->gyro_variance;
	constants.position_constants.accelerometer_noise_radius= ds4_config->accelerometer_noise_radius;
	constants.position_constants.max_velocity= ds4_config->max_velocity;

	// TODO: Allow the config to select the filter type
	// For now hard code the usage of a compound pose filter
    CompoundPoseFilter *pose_filter = new CompoundPoseFilter();
	pose_filter->init(
		OrientationFilterTypeComplementaryOpticalARG, 
		PositionFilterTypeComplimentaryOpticalIMU, 
		constants);

	*out_pose_filter_space= pose_filter_space;
	*out_pose_filter= pose_filter;
}

static void
update_filters_for_psdualshock4(
    const PSDualShock4Controller *psmoveController,
    const PSDualShock4ControllerState *psdualShock4State,
    const float delta_time,
    const ControllerOpticalPoseEstimation *poseEstimation,
	const PoseFilterSpace *poseFilterSpace,
    IPoseFilter *poseFilter)
{
    const PSDualShock4ControllerConfig *config = psmoveController->getConfig();

    if (poseFilter != nullptr)
    {
        PoseSensorPacket sensorPacket;

        if (poseEstimation->bOrientationValid)
        {
            sensorPacket.optical_orientation = 
                Eigen::Quaternionf(
                    poseEstimation->orientation.w, 
                    poseEstimation->orientation.x,
                    poseEstimation->orientation.y,
                    poseEstimation->orientation.z);
            sensorPacket.optical_orientation_quality=
                clampf01(
                    safe_divide_with_default(
                        poseEstimation->projection.screen_area - config->min_orientation_quality_screen_area,
                        config->max_orientation_quality_screen_area - config->min_orientation_quality_screen_area,
                        1.f));
        }
        else
        {
            sensorPacket.optical_orientation = Eigen::Quaternionf::Identity();
            sensorPacket.optical_orientation_quality= 0.f;
        }

        if (poseEstimation->bCurrentlyTracking)
        {
            sensorPacket.optical_position =
                Eigen::Vector3f(
                    poseEstimation->position.x,
                    poseEstimation->position.y,
                    poseEstimation->position.z);
            sensorPacket.optical_position_quality= 
                clampf01(
                    safe_divide_with_default(
                        poseEstimation->projection.screen_area - config->min_position_quality_screen_area,
                        config->max_position_quality_screen_area - config->min_position_quality_screen_area,
                        1.f));
        }
        else
        {
            sensorPacket.optical_position = Eigen::Vector3f::Zero();
            sensorPacket.optical_position_quality= 0.f;
        }

        sensorPacket.imu_accelerometer =
            Eigen::Vector3f(
                psdualShock4State->CalibratedAccelerometer.i,
                psdualShock4State->CalibratedAccelerometer.j,
                psdualShock4State->CalibratedAccelerometer.k);
        sensorPacket.imu_gyroscope =
            Eigen::Vector3f(
                psdualShock4State->CalibratedGyro.i, 
                psdualShock4State->CalibratedGyro.j,
                psdualShock4State->CalibratedGyro.k);
        sensorPacket.imu_gyroscope= Eigen::Vector3f::Zero();

        {
            PoseFilterPacket filterPacket;

			// Create a filter input packet from the sensor data 
			// and the filter's previous orientation and position
			poseFilterSpace->createFilterPacket(
				sensorPacket, 
				poseFilter->getOrientation(), poseFilter->getPosition(),
				filterPacket);

            poseFilter->update(delta_time, filterPacket);
        }
    }
}
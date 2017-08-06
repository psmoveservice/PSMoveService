//-- includes -----
#include "DeviceManager.h"
#include "ServerHMDView.h"
#include "MathAlignment.h"
#include "MorpheusHMD.h"
#include "VirtualHMD.h"
#include "CompoundPoseFilter.h"
#include "PoseFilterInterface.h"
#include "PSMoveProtocol.pb.h"
#include "ServerLog.h"
#include "ServerRequestHandler.h"
#include "ServerTrackerView.h"
#include "TrackerManager.h"

//-- constants -----
static const float k_min_time_delta_seconds = 1 / 120.f;
static const float k_max_time_delta_seconds = 1 / 30.f;

//-- private methods -----
static void init_filters_for_morpheus_hmd(
	const MorpheusHMD *morpheusHMD, PoseFilterSpace **out_pose_filter_space, IPoseFilter **out_pose_filter);
static void init_filters_for_virtual_hmd(
	const VirtualHMD *virtualHMD, PoseFilterSpace **out_pose_filter_space, IPoseFilter **out_pose_filter);
static IPoseFilter *pose_filter_factory(
	const CommonDeviceState::eDeviceType deviceType,
	const std::string &position_filter_type, const std::string &orientation_filter_type,
	const PoseFilterConstants &constants);
static void update_filters_for_morpheus_hmd(
	const MorpheusHMD *morpheusHMD, const MorpheusHMDState *morpheusHMDState,
	const float delta_time,
	const HMDOpticalPoseEstimation *poseEstimation, const PoseFilterSpace *poseFilterSpace, IPoseFilter *poseFilter);
static void update_filters_for_virtual_hmd(
	const VirtualHMD *virtualHMD, const VirtualHMDState *virtualHMDState,
	const float delta_time,
	const HMDOpticalPoseEstimation *poseEstimation, const PoseFilterSpace *poseFilterSpace, IPoseFilter *poseFilter);
static void generate_morpheus_hmd_data_frame_for_stream(
    const ServerHMDView *hmd_view, const HMDStreamInfo *stream_info,
    DeviceOutputDataFramePtr &data_frame);
static void generate_virtual_hmd_data_frame_for_stream(
    const ServerHMDView *hmd_view, const HMDStreamInfo *stream_info,
    DeviceOutputDataFramePtr &data_frame);

static Eigen::Vector3f CommonDevicePosition_to_EigenVector3f(const CommonDevicePosition &p);
static Eigen::Vector3f CommonDeviceVector_to_EigenVector3f(const CommonDeviceVector &v);
static Eigen::Quaternionf CommonDeviceQuaternion_to_EigenQuaternionf(const CommonDeviceQuaternion &q);

static CommonDevicePosition EigenVector3f_to_CommonDevicePosition(const Eigen::Vector3f &p);
static CommonDeviceQuaternion EigenQuaternionf_to_CommonDeviceQuaternion(const Eigen::Quaternionf &q);

static void computeSpherePoseForHmdFromSingleTracker(
    const ServerHMDView *hmdView,
    const ServerTrackerViewPtr tracker,
    HMDOpticalPoseEstimation *tracker_pose_estimation,
    HMDOpticalPoseEstimation *multicam_pose_estimation);
static void computePointCloudPoseForHmdFromSingleTracker(
    const ServerHMDView *hmdView,
    const ServerTrackerViewPtr tracker,
    HMDOpticalPoseEstimation *tracker_pose_estimation,
    HMDOpticalPoseEstimation *multicam_pose_estimation);
static void computeSpherePoseForHmdFromMultipleTrackers(
    const ServerHMDView *controllerView,
    const TrackerManager* tracker_manager,
    const int *valid_projection_tracker_ids,
    const int projections_found,
    HMDOpticalPoseEstimation *tracker_pose_estimations,
    HMDOpticalPoseEstimation *multicam_pose_estimation);
static void computePointCloudPoseForHmdFromMultipleTrackers(
    const ServerHMDView *controllerView,
    const TrackerManager* tracker_manager,
    const int *valid_projection_tracker_ids,
    const int projections_found,
    HMDOpticalPoseEstimation *tracker_pose_estimations,
    HMDOpticalPoseEstimation *multicam_pose_estimation);

//-- public implementation -----
ServerHMDView::ServerHMDView(const int device_id)
	: ServerDeviceView(device_id)
	, m_tracking_listener_count(0)
	, m_tracking_enabled(false)
	, m_roi_disable_count(0)
	, m_device(nullptr)
	, m_tracker_pose_estimations(nullptr)
	, m_multicam_pose_estimation(nullptr)
	, m_pose_filter(nullptr)
	, m_pose_filter_space(nullptr)
	, m_lastPollSeqNumProcessed(-1)
	, m_last_filter_update_timestamp()
	, m_last_filter_update_timestamp_valid(false)
{
}

ServerHMDView::~ServerHMDView()
{
}

bool ServerHMDView::allocate_device_interface(const class DeviceEnumerator *enumerator)
{
    switch (enumerator->get_device_type())
    {
    case CommonDeviceState::Morpheus:
        {
            m_device = new MorpheusHMD();
			m_pose_filter = nullptr; // no pose filter until the device is opened

			m_tracker_pose_estimations = new HMDOpticalPoseEstimation[TrackerManager::k_max_devices];
			for (int tracker_index = 0; tracker_index < TrackerManager::k_max_devices; ++tracker_index)
			{
				m_tracker_pose_estimations[tracker_index].clear();
			}

			m_multicam_pose_estimation = new HMDOpticalPoseEstimation();
			m_multicam_pose_estimation->clear();
        } break;
    case CommonDeviceState::VirtualHMD:
        {
            m_device = new VirtualHMD();
			m_pose_filter = nullptr; // no pose filter until the device is opened

			m_tracker_pose_estimations = new HMDOpticalPoseEstimation[TrackerManager::k_max_devices];
			for (int tracker_index = 0; tracker_index < TrackerManager::k_max_devices; ++tracker_index)
			{
				m_tracker_pose_estimations[tracker_index].clear();
			}

			m_multicam_pose_estimation = new HMDOpticalPoseEstimation();
			m_multicam_pose_estimation->clear();
        } break;
    default:
        break;
    }

    return m_device != nullptr;
}

void ServerHMDView::free_device_interface()
{
	if (m_multicam_pose_estimation != nullptr)
	{
		delete m_multicam_pose_estimation;
		m_multicam_pose_estimation = nullptr;
	}

	if (m_tracker_pose_estimations != nullptr)
	{
		delete[] m_tracker_pose_estimations;
		m_tracker_pose_estimations = nullptr;
	}

	if (m_pose_filter_space != nullptr)
	{
		delete m_pose_filter_space;
		m_pose_filter_space = nullptr;
	}

	if (m_pose_filter != nullptr)
	{
		delete m_pose_filter;
		m_pose_filter = nullptr;
	}

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
        bool bAllocateTrackingColor = false;

		switch (device->getDeviceType())
		{
		case CommonDeviceState::Morpheus:
        case CommonDeviceState::VirtualHMD:
			{
				// Create a pose filter based on the HMD type
				resetPoseFilter();
				m_multicam_pose_estimation->clear();
                bAllocateTrackingColor= true;
			} break;
		default:
			break;
		}

        // If needed for this kind of HMD, assign a tracking color id
        if (bAllocateTrackingColor)
        {
            eCommonTrackingColorID tracking_color_id;
            assert(m_device != nullptr);

            // If this device already has a valid tracked color assigned, 
            // claim it from the pool (or another controller/hmd that had it previously)
            if (m_device->getTrackingColorID(tracking_color_id) && tracking_color_id != eCommonTrackingColorID::INVALID_COLOR)
            {
                DeviceManager::getInstance()->m_tracker_manager->claimTrackingColorID(this, tracking_color_id);
            }
            else
            {
                // Allocate a color from the list of remaining available color ids
                eCommonTrackingColorID allocatedColorID= DeviceManager::getInstance()->m_tracker_manager->allocateTrackingColorID();

                // Attempt to assign the tracking color id to the controller
                if (!m_device->setTrackingColorID(allocatedColorID))
                {
                    // If the device can't be assigned a tracking color, release the color back to the pool
                    DeviceManager::getInstance()->m_tracker_manager->freeTrackingColorID(allocatedColorID);
                }
            }
        }

        // Reset the poll sequence number high water mark
        m_lastPollSeqNumProcessed = -1;
    }

    return bSuccess;
}

void ServerHMDView::close()
{
    set_tracking_enabled_internal(false);

    eCommonTrackingColorID tracking_color_id= eCommonTrackingColorID::INVALID_COLOR;
    if (m_device != nullptr && m_device->getTrackingColorID(tracking_color_id))
    {
        if (tracking_color_id != eCommonTrackingColorID::INVALID_COLOR)
        {
            DeviceManager::getInstance()->m_tracker_manager->freeTrackingColorID(tracking_color_id);
        }
    }

    ServerDeviceView::close();
}

void ServerHMDView::resetPoseFilter()
{
	assert(m_device != nullptr);

	if (m_pose_filter != nullptr)
	{
		delete m_pose_filter;
		m_pose_filter = nullptr;
	}

	if (m_pose_filter_space != nullptr)
	{
		delete m_pose_filter_space;
		m_pose_filter_space = nullptr;
	}

	switch (m_device->getDeviceType())
	{
	case CommonDeviceState::Morpheus:
		{
			const MorpheusHMD *morpheusHMD = this->castCheckedConst<MorpheusHMD>();

			init_filters_for_morpheus_hmd(morpheusHMD, &m_pose_filter_space, &m_pose_filter);
		} break;
	case CommonDeviceState::VirtualHMD:
		{
			const VirtualHMD *virtualHMD = this->castCheckedConst<VirtualHMD>();

			init_filters_for_virtual_hmd(virtualHMD, &m_pose_filter_space, &m_pose_filter);
		} break;
	default:
		break;
	}
}

void ServerHMDView::updateOpticalPoseEstimation(TrackerManager* tracker_manager)
{
    const std::chrono::time_point<std::chrono::high_resolution_clock> now= std::chrono::high_resolution_clock::now();

    // TODO: Probably need to first update IMU state to get velocity.
    // If velocity is too high, don't bother getting a new position.
    // Though it may be enough to just use the camera ROI as the limit.
    
    if (getIsTrackingEnabled())
    {
        int valid_projection_tracker_ids[TrackerManager::k_max_devices];
        int projections_found = 0;

        CommonDeviceTrackingShape trackingShape;
        m_device->getTrackingShape(trackingShape);
        assert(trackingShape.shape_type != eCommonTrackingShapeType::INVALID_SHAPE);

        // Find the projection of the controller from the perspective of each tracker.
        // In the case of sphere projections, go ahead and compute the tracker relative position as well.
        for (int tracker_id = 0; tracker_id < tracker_manager->getMaxDevices(); ++tracker_id)
        {
            ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);
            HMDOpticalPoseEstimation &trackerPoseEstimateRef = m_tracker_pose_estimations[tracker_id];

            const bool bWasTracking= trackerPoseEstimateRef.bCurrentlyTracking;

            // Assume we're going to lose tracking this frame
            bool bCurrentlyTracking = false;

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
                        // Create a copy of the pose estimate state so that in event of a 
                        // failure part way through computing the projection we don't
                        // set partially valid state
                        HMDOpticalPoseEstimation newTrackerPoseEstimate= trackerPoseEstimateRef;

                        if (tracker->computeProjectionForHMD(
                                this, 
                                &trackingShape,
                                &newTrackerPoseEstimate))
                        {
                            bIsVisibleThisUpdate= true;

                            // Actually apply the pose estimate state
                            trackerPoseEstimateRef= newTrackerPoseEstimate;
                            trackerPoseEstimateRef.last_visible_timestamp = now;
                        }
                    }

                    // If the projection isn't too old (or updated this tick), 
                    // say we have a valid tracked location
                    if (bWasTracking || bIsVisibleThisUpdate)
                    {
                        const std::chrono::duration<float, std::milli> timeSinceLastVisibleMillis= 
                            now - trackerPoseEstimateRef.last_visible_timestamp;

                        if (timeSinceLastVisibleMillis.count() < timeoutMilli)
                        {
                            // If this tracker has a valid projection for the controller
                            // add it to the tracker id list
                            valid_projection_tracker_ids[projections_found] = tracker_id;
                            ++projections_found;

                            // Flag this pose estimate as invalid
                            bCurrentlyTracking = true;
                        }
                    }
                }
            }

            // Keep track of the last time the position estimate was updated
            trackerPoseEstimateRef.last_update_timestamp = now;
            trackerPoseEstimateRef.bValidTimestamps = true;
            trackerPoseEstimateRef.bCurrentlyTracking = bCurrentlyTracking;
        }

        // How we compute the final world pose estimate varies based on
        // * Number of trackers that currently have a valid projections of the controller
        // * The kind of projection shape (psmove sphere or ds4 lightbar)
        if (projections_found > 1)
        {
            // If multiple trackers can see the controller, 
            // triangulate all pairs of projections and average the results
            switch (trackingShape.shape_type)
            {
            case eCommonTrackingShapeType::Sphere:
                computeSpherePoseForHmdFromMultipleTrackers(
                    this,
                    tracker_manager,
                    valid_projection_tracker_ids,
                    projections_found,
                    m_tracker_pose_estimations,
                    m_multicam_pose_estimation);
                break;
            case eCommonTrackingShapeType::PointCloud:
                computePointCloudPoseForHmdFromMultipleTrackers(
                    this,
                    tracker_manager,
                    valid_projection_tracker_ids,
                    projections_found,
                    m_tracker_pose_estimations,
                    m_multicam_pose_estimation);
                break;
            default:
                assert(false && "unreachable");
            }
        }
        else if (projections_found == 1 && !DeviceManager::getInstance()->m_tracker_manager->getConfig().ignore_pose_from_one_tracker)
        {
            const int tracker_id = valid_projection_tracker_ids[0];
            const ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);

            // If only one tracker can see the controller, 
            // then use the tracker to derive a world space location
            switch (trackingShape.shape_type)
            {
            case eCommonTrackingShapeType::Sphere:
                computeSpherePoseForHmdFromSingleTracker(
                    this,
                    tracker,
                    &m_tracker_pose_estimations[tracker_id],
                    m_multicam_pose_estimation);
                break;
            case eCommonTrackingShapeType::PointCloud:
                computePointCloudPoseForHmdFromSingleTracker(
                    this,
                    tracker,
                    &m_tracker_pose_estimations[tracker_id],
                    m_multicam_pose_estimation);
                break;
            default:
                assert(false && "unreachable");
            }
        }
        // If no trackers can see the controller, maintain the last known position and time it was seen
        else
        {
            m_multicam_pose_estimation->bCurrentlyTracking= false;
        }

        // Update the position estimation timestamps
        if (m_multicam_pose_estimation->bCurrentlyTracking)
        {
            m_multicam_pose_estimation->last_visible_timestamp = now;
        }
        m_multicam_pose_estimation->last_update_timestamp = now;
        m_multicam_pose_estimation->bValidTimestamps = true;
    }
}

void ServerHMDView::updateStateAndPredict()
{
	if (!getHasUnpublishedState())
	{
		return;
	}

	// Look backward in time to find the first HMD update state with a poll sequence number 
	// newer than the last sequence number we've processed.
	int firstLookBackIndex = -1;
	int testLookBack = 0;
	const CommonHMDState *state = getState(testLookBack);
	while (state != nullptr && state->PollSequenceNumber > m_lastPollSeqNumProcessed)
	{
		firstLookBackIndex = testLookBack;
		testLookBack++;
		state = getState(testLookBack);
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

	// Evenly apply the list of hmd state updates over the time since last filter update
	float per_state_time_delta_seconds = time_delta_seconds / static_cast<float>(firstLookBackIndex + 1);

	// Process the polled hmd states forward in time
	// computing the new orientation along the way.
	for (int lookBackIndex = firstLookBackIndex; lookBackIndex >= 0; --lookBackIndex)
	{
		const CommonHMDState *hmdState = getState(lookBackIndex);

		switch (hmdState->DeviceType)
		{
		case CommonHMDState::Morpheus:
		    {
			    const MorpheusHMD *morpheusHMD = this->castCheckedConst<MorpheusHMD>();
			    const MorpheusHMDState *morpheusHMDState = static_cast<const MorpheusHMDState *>(hmdState);

			    // Only update the position filter when tracking is enabled
			    update_filters_for_morpheus_hmd(
				    morpheusHMD, morpheusHMDState,
				    per_state_time_delta_seconds,
				    m_multicam_pose_estimation,
				    m_pose_filter_space,
				    m_pose_filter);
		    } break;
		case CommonHMDState::VirtualHMD:
		    {
			    const VirtualHMD *virtualHMD = this->castCheckedConst<VirtualHMD>();
			    const VirtualHMDState *virtualHMDState = static_cast<const VirtualHMDState *>(hmdState);

			    // Only update the position filter when tracking is enabled
			    update_filters_for_virtual_hmd(
				    virtualHMD, virtualHMDState,
				    per_state_time_delta_seconds,
				    m_multicam_pose_estimation,
				    m_pose_filter_space,
				    m_pose_filter);
		    } break;
		default:
			assert(0 && "Unhandled HMD type");
		}

		// Consider this hmd state sequence num processed
		m_lastPollSeqNumProcessed = hmdState->PollSequenceNumber;
	}
}

CommonDevicePose
ServerHMDView::getFilteredPose(float time) const
{
	CommonDevicePose pose;

	pose.clear();

	if (m_pose_filter != nullptr)
	{
		const Eigen::Quaternionf orientation = m_pose_filter->getOrientation(time);
		const Eigen::Vector3f position_cm = m_pose_filter->getPositionCm(time);

		pose.Orientation.w = orientation.w();
		pose.Orientation.x = orientation.x();
		pose.Orientation.y = orientation.y();
		pose.Orientation.z = orientation.z();

		pose.PositionCm.x = position_cm.x();
		pose.PositionCm.y = position_cm.y();
		pose.PositionCm.z = position_cm.z();
	}

	return pose;
}

CommonDevicePhysics
ServerHMDView::getFilteredPhysics() const
{
	CommonDevicePhysics physics;

	if (m_pose_filter != nullptr)
	{
		const Eigen::Vector3f first_derivative = m_pose_filter->getAngularVelocityRadPerSec();
		const Eigen::Vector3f second_derivative = m_pose_filter->getAngularAccelerationRadPerSecSqr();
		const Eigen::Vector3f velocity(m_pose_filter->getVelocityCmPerSec());
		const Eigen::Vector3f acceleration(m_pose_filter->getAccelerationCmPerSecSqr());

		physics.AngularVelocityRadPerSec.i = first_derivative.x();
		physics.AngularVelocityRadPerSec.j = first_derivative.y();
		physics.AngularVelocityRadPerSec.k = first_derivative.z();

		physics.AngularAccelerationRadPerSecSqr.i = second_derivative.x();
		physics.AngularAccelerationRadPerSecSqr.j = second_derivative.y();
		physics.AngularAccelerationRadPerSecSqr.k = second_derivative.z();

		physics.VelocityCmPerSec.i = velocity.x();
		physics.VelocityCmPerSec.j = velocity.y();
		physics.VelocityCmPerSec.k = velocity.z();

		physics.AccelerationCmPerSecSqr.i = acceleration.x();
		physics.AccelerationCmPerSecSqr.j = acceleration.y();
		physics.AccelerationCmPerSecSqr.k = acceleration.z();
	}

	return physics;
}

// Returns the full usb device path for the controller
std::string
ServerHMDView::getUSBDevicePath() const
{
    return m_device->getUSBDevicePath();
}

// Returns the "controller_" + serial number for the controller
std::string
ServerHMDView::getConfigIdentifier() const
{
	std::string	identifier = "";

	if (m_device != nullptr)
	{
		if (m_device->getDeviceType() == CommonDeviceState::Morpheus)
		{
			identifier = "hmd_morpheus";
		}
		else if (m_device->getDeviceType() == CommonDeviceState::VirtualHMD)
		{
            // THe "USB device path" for a Virtual HMD is actually just the Virtual HMD identifier, i.e. "VirtualHMD_0"
			identifier = "hmd_"+m_device->getUSBDevicePath();
		}
		else
		{
			identifier = "hmd_unknown";
		}
	}

	return identifier;
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
        ((int)device_state->DeviceType >= (int)CommonDeviceState::HeadMountedDisplay &&
        device_state->DeviceType < CommonDeviceState::SUPPORTED_HMD_TYPE_COUNT));

    return static_cast<const CommonHMDState *>(device_state);
}

void ServerHMDView::startTracking()
{
	if (!m_tracking_enabled)
	{
		set_tracking_enabled_internal(true);
	}

	++m_tracking_listener_count;
}

void ServerHMDView::stopTracking()
{
	assert(m_tracking_listener_count > 0);
	--m_tracking_listener_count;

	if (m_tracking_listener_count <= 0 && m_tracking_enabled)
	{
		set_tracking_enabled_internal(false);
	}
}

void ServerHMDView::set_tracking_enabled_internal(bool bEnabled)
{
	if (m_tracking_enabled != bEnabled)
	{
		switch (getHMDDeviceType())
		{
		case CommonHMDState::Morpheus:
			castChecked<MorpheusHMD>()->setTrackingEnabled(bEnabled);
			break;
		case CommonHMDState::VirtualHMD:
			castChecked<VirtualHMD>()->setTrackingEnabled(bEnabled);
			break;
		default:
			assert(0 && "unreachable");
		}

		m_tracking_enabled = bEnabled;
	}
}

// Get the tracking shape for the controller
bool ServerHMDView::getTrackingShape(CommonDeviceTrackingShape &trackingShape) const
{
	m_device->getTrackingShape(trackingShape);

	return trackingShape.shape_type != eCommonTrackingShapeType::INVALID_SHAPE;
}

eCommonTrackingColorID ServerHMDView::getTrackingColorID() const
{
	eCommonTrackingColorID tracking_color_id = eCommonTrackingColorID::INVALID_COLOR;

	if (m_device != nullptr)
	{
		m_device->getTrackingColorID(tracking_color_id);
	}

	return tracking_color_id;
}

bool ServerHMDView::setTrackingColorID(eCommonTrackingColorID colorID)
{
    bool bSuccess= true;

    if (colorID != getTrackingColorID())
    {
        if (m_device != nullptr)
        {
            bSuccess= m_device->setTrackingColorID(colorID);

            if (bSuccess && getIsTrackingEnabled())
            {
                set_tracking_enabled_internal(false);
                set_tracking_enabled_internal(true);
            }
        }
        else
        {
            bSuccess= false;
        }
    }

    return bSuccess;
}

float ServerHMDView::getROIPredictionTime() const
{
	return m_device->getPredictionTime();
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
    DeviceOutputDataFramePtr &data_frame)
{
    PSMoveProtocol::DeviceOutputDataFrame_HMDDataPacket *hmd_data_frame =
        data_frame->mutable_hmd_data_packet();

    hmd_data_frame->set_hmd_id(hmd_view->getDeviceID());
    hmd_data_frame->set_sequence_num(hmd_view->m_sequence_number);
    hmd_data_frame->set_isconnected(hmd_view->getDevice()->getIsOpen());

    switch (hmd_view->getHMDDeviceType())
    {
    case CommonHMDState::Morpheus:
        {
            generate_morpheus_hmd_data_frame_for_stream(hmd_view, stream_info, data_frame);
        } break;
    case CommonHMDState::VirtualHMD:
        {
            generate_virtual_hmd_data_frame_for_stream(hmd_view, stream_info, data_frame);
        } break;
    default:
        assert(0 && "Unhandled HMD type");
    }

    data_frame->set_device_category(PSMoveProtocol::DeviceOutputDataFrame::HMD);
}

static void
init_filters_for_morpheus_hmd(
    const MorpheusHMD *morpheusHMD,
	PoseFilterSpace **out_pose_filter_space,
	IPoseFilter **out_pose_filter)
{
    const MorpheusHMDConfig *hmd_config = morpheusHMD->getConfig();

	// Setup the space the pose filter operates in
	PoseFilterSpace *pose_filter_space = new PoseFilterSpace();
	pose_filter_space->setIdentityGravity(Eigen::Vector3f(0.f, 1.f, 0.f));
	pose_filter_space->setIdentityMagnetometer(Eigen::Vector3f::Zero());
	pose_filter_space->setCalibrationTransform(*k_eigen_identity_pose_upright);
	pose_filter_space->setSensorTransform(*k_eigen_sensor_transform_identity);

	CommonDeviceVector accel_var = hmd_config->get_calibrated_accelerometer_variance();
	CommonDeviceVector gyro_var = hmd_config->get_calibrated_gyro_variance();
	CommonDeviceVector gyro_drift = hmd_config->get_calibrated_gyro_drift();

	// Copy the pose filter constants from the controller config
	PoseFilterConstants constants;
	constants.clear();

	constants.orientation_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
	constants.orientation_constants.accelerometer_variance = Eigen::Vector3f(accel_var.i, accel_var.j, accel_var.k);
	constants.position_constants.accelerometer_drift = Eigen::Vector3f::Zero();
	constants.orientation_constants.magnetometer_calibration_direction = pose_filter_space->getMagnetometerCalibrationDirection();
	constants.orientation_constants.gyro_drift = Eigen::Vector3f(gyro_drift.i, gyro_drift.j, gyro_drift.k);
	constants.orientation_constants.gyro_variance = Eigen::Vector3f(gyro_var.i, gyro_var.j, gyro_var.k);
	constants.orientation_constants.mean_update_time_delta = hmd_config->mean_update_time_delta;
	constants.orientation_constants.orientation_variance_curve.A = hmd_config->orientation_variance;
	constants.orientation_constants.orientation_variance_curve.B = 0.f;
	constants.orientation_constants.orientation_variance_curve.MaxValue = 1.f;
	constants.orientation_constants.magnetometer_variance = Eigen::Vector3f::Zero();
	constants.orientation_constants.magnetometer_drift = Eigen::Vector3f::Zero();

	constants.position_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
	constants.position_constants.accelerometer_variance = Eigen::Vector3f(accel_var.i, accel_var.j, accel_var.k);
	constants.position_constants.accelerometer_drift = Eigen::Vector3f::Zero();
	constants.position_constants.accelerometer_noise_radius = 0.f; // TODO
	constants.position_constants.max_velocity = hmd_config->max_velocity;
	constants.position_constants.mean_update_time_delta = hmd_config->mean_update_time_delta;
	constants.position_constants.position_variance_curve.A = hmd_config->position_variance_exp_fit_a;
	constants.position_constants.position_variance_curve.B = hmd_config->position_variance_exp_fit_b;
	constants.position_constants.position_variance_curve.MaxValue = 1.f;

	*out_pose_filter_space = pose_filter_space;
	*out_pose_filter = pose_filter_factory(
		CommonDeviceState::eDeviceType::PSMove,
		hmd_config->position_filter_type,
		hmd_config->orientation_filter_type,
		constants);
}

static void init_filters_for_virtual_hmd(
    const VirtualHMD *virtualHMD, PoseFilterSpace **out_pose_filter_space, IPoseFilter **out_pose_filter)
{
    const VirtualHMDConfig *hmd_config = virtualHMD->getConfig();

	// Setup the space the pose filter operates in
	PoseFilterSpace *pose_filter_space = new PoseFilterSpace();
	pose_filter_space->setIdentityGravity(Eigen::Vector3f(0.f, 1.f, 0.f));
	pose_filter_space->setIdentityMagnetometer(Eigen::Vector3f::Zero());
	pose_filter_space->setCalibrationTransform(*k_eigen_identity_pose_upright);
	pose_filter_space->setSensorTransform(*k_eigen_sensor_transform_identity);

	// Copy the pose filter constants from the controller config
	PoseFilterConstants constants;
	constants.clear();

	constants.orientation_constants.gravity_calibration_direction = Eigen::Vector3f::Zero();
	constants.orientation_constants.accelerometer_variance = Eigen::Vector3f::Zero();
	constants.position_constants.accelerometer_drift = Eigen::Vector3f::Zero();
	constants.orientation_constants.magnetometer_calibration_direction = Eigen::Vector3f::Zero();
	constants.orientation_constants.gyro_drift = Eigen::Vector3f::Zero();
	constants.orientation_constants.gyro_variance = Eigen::Vector3f::Zero();
	constants.orientation_constants.mean_update_time_delta = 0.f;
	constants.orientation_constants.orientation_variance_curve.A = 0.f;
	constants.orientation_constants.orientation_variance_curve.B = 0.f;
	constants.orientation_constants.orientation_variance_curve.MaxValue = 0.f;
	constants.orientation_constants.magnetometer_variance = Eigen::Vector3f::Zero();
	constants.orientation_constants.magnetometer_drift = Eigen::Vector3f::Zero();

	constants.position_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
	constants.position_constants.accelerometer_variance = Eigen::Vector3f::Zero();
	constants.position_constants.accelerometer_drift = Eigen::Vector3f::Zero();
	constants.position_constants.accelerometer_noise_radius = 0.f; // TODO
	constants.position_constants.max_velocity = hmd_config->max_velocity;
	constants.position_constants.mean_update_time_delta = hmd_config->mean_update_time_delta;
	constants.position_constants.position_variance_curve.A = hmd_config->position_variance_exp_fit_a;
	constants.position_constants.position_variance_curve.B = hmd_config->position_variance_exp_fit_b;
	constants.position_constants.position_variance_curve.MaxValue = 1.f;

	*out_pose_filter_space = pose_filter_space;
	*out_pose_filter = pose_filter_factory(
		CommonDeviceState::eDeviceType::VirtualHMD,
		hmd_config->position_filter_type,
		"",
		constants);
}

static IPoseFilter *
pose_filter_factory(
	const CommonDeviceState::eDeviceType deviceType,
	const std::string &position_filter_type,
	const std::string &orientation_filter_type,
	const PoseFilterConstants &constants)
{
	static IPoseFilter *filter = nullptr;

	// Convert the position filter type string into an enum
	PositionFilterType position_filter_enum = PositionFilterTypeNone;
	if (position_filter_type == "PassThru")
	{
		position_filter_enum = PositionFilterTypePassThru;
	}
	else if (position_filter_type == "LowPassOptical")
	{
		position_filter_enum = PositionFilterTypeLowPassOptical;
	}
	else if (position_filter_type == "LowPassIMU")
	{
		position_filter_enum = PositionFilterTypeLowPassIMU;
	}
	else if (position_filter_type == "LowPassExponential")
	{
		position_filter_enum = PositionFilterTypeLowPassExponential;
	}
	else if (position_filter_type == "ComplimentaryOpticalIMU")
	{
		position_filter_enum = PositionFilterTypeComplimentaryOpticalIMU;
	}
	else if (position_filter_type == "PositionKalman")
	{
		position_filter_enum = PositionFilterTypeKalman;
	}
	else
	{
		SERVER_LOG_INFO("pose_filter_factory()") <<
			"Unknown position filter type: " << position_filter_type << ". Using default.";

		// fallback to a default based on hmd type
		switch (deviceType)
		{
		case CommonDeviceState::Morpheus:
        case CommonDeviceState::VirtualHMD:
			position_filter_enum = PositionFilterTypeLowPassIMU;
			break;
		default:
			assert(0 && "unreachable");
		}
	}

	// Convert the orientation filter type string into an enum
	OrientationFilterType orientation_filter_enum = OrientationFilterTypeNone;
	if (orientation_filter_type == "")
	{
		orientation_filter_enum = OrientationFilterTypeNone;
	}
	else if (orientation_filter_type == "PassThru")
	{
		orientation_filter_enum = OrientationFilterTypePassThru;
	}
	else if (orientation_filter_type == "MadgwickARG")
	{
		orientation_filter_enum = OrientationFilterTypeMadgwickARG;
	}
	else if (orientation_filter_type == "ComplementaryOpticalARG")
	{
		orientation_filter_enum = OrientationFilterTypeComplementaryOpticalARG;
	}
	else if (orientation_filter_type == "OrientationKalman")
	{
		orientation_filter_enum = OrientationFilterTypeKalman;
	}
	else
	{
		SERVER_LOG_INFO("pose_filter_factory()") <<
			"Unknown orientation filter type: " << orientation_filter_type << ". Using default.";

		// fallback to a default based on controller type
		switch (deviceType)
		{
		case CommonDeviceState::Morpheus:
			orientation_filter_enum = OrientationFilterTypeComplementaryOpticalARG;
			break;
        case CommonDeviceState::VirtualHMD:
            orientation_filter_enum = OrientationFilterTypeNone;
            break;
		default:
			assert(0 && "unreachable");
		}
	}

	CompoundPoseFilter *compound_pose_filter = new CompoundPoseFilter();
	compound_pose_filter->init(deviceType, orientation_filter_enum, position_filter_enum, constants);
	filter = compound_pose_filter;

	assert(filter != nullptr);

	return filter;
}

static void
update_filters_for_morpheus_hmd(
    const MorpheusHMD *morpheusHMD,
    const MorpheusHMDState *morpheusHMDState,
	const float delta_time,
	const HMDOpticalPoseEstimation *poseEstimation,
	const PoseFilterSpace *poseFilterSpace,
	IPoseFilter *poseFilter)
{
    const MorpheusHMDConfig *config = morpheusHMD->getConfig();

	// Update the orientation filter
	if (poseFilter != nullptr)
	{
		PoseSensorPacket sensorPacket;

		sensorPacket.imu_magnetometer_unit = Eigen::Vector3f::Zero();

		if (poseEstimation->bOrientationValid)
		{
			sensorPacket.optical_orientation =
				Eigen::Quaternionf(
					poseEstimation->orientation.w,
					poseEstimation->orientation.x,
					poseEstimation->orientation.y,
					poseEstimation->orientation.z);
		}
		else
		{
			sensorPacket.optical_orientation = Eigen::Quaternionf::Identity();
		}

		if (poseEstimation->bCurrentlyTracking)
		{
			sensorPacket.optical_position_cm =
				Eigen::Vector3f(
					poseEstimation->position_cm.x,
					poseEstimation->position_cm.y,
					poseEstimation->position_cm.z);
			sensorPacket.tracking_projection_area_px_sqr = poseEstimation->projection.screen_area;
		}
		else
		{
			sensorPacket.optical_position_cm = Eigen::Vector3f::Zero();
			sensorPacket.tracking_projection_area_px_sqr = 0.f;
		}

		// Each state update contains two readings (one earlier and one later) of accelerometer and gyro data
		for (int frame = 0; frame < 2; ++frame)
		{
			const MorpheusHMDSensorFrame &sensorFrame= morpheusHMDState->SensorFrames[frame];

			sensorPacket.imu_accelerometer_g_units =
				Eigen::Vector3f(
					sensorFrame.CalibratedAccel.i,
					sensorFrame.CalibratedAccel.j,
					sensorFrame.CalibratedAccel.k);
			sensorPacket.imu_gyroscope_rad_per_sec =
				Eigen::Vector3f(
					sensorFrame.CalibratedGyro.i,
					sensorFrame.CalibratedGyro.j,
					sensorFrame.CalibratedGyro.k);
			sensorPacket.imu_magnetometer_unit = Eigen::Vector3f::Zero();

			{
				PoseFilterPacket filterPacket;

				// Create a filter input packet from the sensor data 
				// and the filter's previous orientation and position
				poseFilterSpace->createFilterPacket(
					sensorPacket,
					poseFilter,
					filterPacket);

				poseFilter->update(delta_time / 2.f, filterPacket);
			}
		}
	}
}

static void
update_filters_for_virtual_hmd(
    const VirtualHMD *virtualHMD,
    const VirtualHMDState *virtualHMDState,
	const float delta_time,
	const HMDOpticalPoseEstimation *poseEstimation,
	const PoseFilterSpace *poseFilterSpace,
	IPoseFilter *poseFilter)
{
    const VirtualHMDConfig *config = virtualHMD->getConfig();

	// Update the orientation filter
	if (poseFilter != nullptr)
	{
		PoseSensorPacket sensorPacket;

		sensorPacket.imu_magnetometer_unit = Eigen::Vector3f::Zero();
		sensorPacket.optical_orientation = Eigen::Quaternionf::Identity();

		if (poseEstimation->bCurrentlyTracking)
		{
			sensorPacket.optical_position_cm =
				Eigen::Vector3f(
					poseEstimation->position_cm.x,
					poseEstimation->position_cm.y,
					poseEstimation->position_cm.z);
			sensorPacket.tracking_projection_area_px_sqr = poseEstimation->projection.screen_area;
		}
		else
		{
			sensorPacket.optical_position_cm = Eigen::Vector3f::Zero();
			sensorPacket.tracking_projection_area_px_sqr = 0.f;
		}

		sensorPacket.imu_accelerometer_g_units = Eigen::Vector3f::Zero();
		sensorPacket.imu_gyroscope_rad_per_sec = Eigen::Vector3f::Zero();
		sensorPacket.imu_magnetometer_unit = Eigen::Vector3f::Zero();

		{
			PoseFilterPacket filterPacket;

			// Create a filter input packet from the sensor data 
			// and the filter's previous orientation and position
			poseFilterSpace->createFilterPacket(sensorPacket, poseFilter, filterPacket);

			poseFilter->update(delta_time, filterPacket);
		}
	}
}

static void generate_morpheus_hmd_data_frame_for_stream(
    const ServerHMDView *hmd_view,
    const HMDStreamInfo *stream_info,
    DeviceOutputDataFramePtr &data_frame)
{
    const MorpheusHMD *morpheus_hmd = hmd_view->castCheckedConst<MorpheusHMD>();
    const MorpheusHMDConfig *morpheus_config = morpheus_hmd->getConfig();
	const IPoseFilter *pose_filter = hmd_view->getPoseFilter();
    const CommonHMDState *hmd_state = hmd_view->getState();
    const CommonDevicePose hmd_pose = hmd_view->getFilteredPose();

    PSMoveProtocol::DeviceOutputDataFrame_HMDDataPacket *hmd_data_frame = data_frame->mutable_hmd_data_packet();

    if (hmd_state != nullptr)
    {
        assert(hmd_state->DeviceType == CommonDeviceState::Morpheus);
        const MorpheusHMDState * morpheus_hmd_state = static_cast<const MorpheusHMDState *>(hmd_state);

        PSMoveProtocol::DeviceOutputDataFrame_HMDDataPacket_MorpheusState* morpheus_data_frame = 
            hmd_data_frame->mutable_morpheus_state();

		morpheus_data_frame->set_iscurrentlytracking(hmd_view->getIsCurrentlyTracking());
		morpheus_data_frame->set_istrackingenabled(hmd_view->getIsTrackingEnabled());
		morpheus_data_frame->set_isorientationvalid(pose_filter->getIsStateValid());
		morpheus_data_frame->set_ispositionvalid(pose_filter->getIsStateValid());

		morpheus_data_frame->mutable_orientation()->set_w(hmd_pose.Orientation.w);
		morpheus_data_frame->mutable_orientation()->set_x(hmd_pose.Orientation.x);
		morpheus_data_frame->mutable_orientation()->set_y(hmd_pose.Orientation.y);
		morpheus_data_frame->mutable_orientation()->set_z(hmd_pose.Orientation.z);

		if (stream_info->include_position_data)
		{
			morpheus_data_frame->mutable_position_cm()->set_x(hmd_pose.PositionCm.x);
			morpheus_data_frame->mutable_position_cm()->set_y(hmd_pose.PositionCm.y);
			morpheus_data_frame->mutable_position_cm()->set_z(hmd_pose.PositionCm.z);
		}
		else
		{
			morpheus_data_frame->mutable_position_cm()->set_x(0);
			morpheus_data_frame->mutable_position_cm()->set_y(0);
			morpheus_data_frame->mutable_position_cm()->set_z(0);
		}

		// If requested, get the raw sensor data for the hmd
		if (stream_info->include_physics_data)
		{
			const CommonDevicePhysics hmd_physics = hmd_view->getFilteredPhysics();
			auto *physics_data = morpheus_data_frame->mutable_physics_data();

			physics_data->mutable_velocity_cm_per_sec()->set_i(hmd_physics.VelocityCmPerSec.i);
			physics_data->mutable_velocity_cm_per_sec()->set_j(hmd_physics.VelocityCmPerSec.j);
			physics_data->mutable_velocity_cm_per_sec()->set_k(hmd_physics.VelocityCmPerSec.k);

			physics_data->mutable_acceleration_cm_per_sec_sqr()->set_i(hmd_physics.AccelerationCmPerSecSqr.i);
			physics_data->mutable_acceleration_cm_per_sec_sqr()->set_j(hmd_physics.AccelerationCmPerSecSqr.j);
			physics_data->mutable_acceleration_cm_per_sec_sqr()->set_k(hmd_physics.AccelerationCmPerSecSqr.k);

			physics_data->mutable_angular_velocity_rad_per_sec()->set_i(hmd_physics.AngularVelocityRadPerSec.i);
			physics_data->mutable_angular_velocity_rad_per_sec()->set_j(hmd_physics.AngularVelocityRadPerSec.j);
			physics_data->mutable_angular_velocity_rad_per_sec()->set_k(hmd_physics.AngularVelocityRadPerSec.k);

			physics_data->mutable_angular_acceleration_rad_per_sec_sqr()->set_i(hmd_physics.AngularAccelerationRadPerSecSqr.i);
			physics_data->mutable_angular_acceleration_rad_per_sec_sqr()->set_j(hmd_physics.AngularAccelerationRadPerSecSqr.j);
			physics_data->mutable_angular_acceleration_rad_per_sec_sqr()->set_k(hmd_physics.AngularAccelerationRadPerSecSqr.k);
		}

        // If requested, get the raw sensor data for the hmd
        if (stream_info->include_raw_sensor_data)
        {
            PSMoveProtocol::DeviceOutputDataFrame_HMDDataPacket_MorpheusState_RawSensorData *raw_sensor_data =
                morpheus_data_frame->mutable_raw_sensor_data();

			// Two frames: [[ax0, ay0, az0], [ax1, ay1, az1]] 
			// Take the most recent frame: [ax1, ay1, az1]
			raw_sensor_data->mutable_accelerometer()->set_i(morpheus_hmd_state->SensorFrames[1].RawAccel.i);
			raw_sensor_data->mutable_accelerometer()->set_j(morpheus_hmd_state->SensorFrames[1].RawAccel.j);
			raw_sensor_data->mutable_accelerometer()->set_k(morpheus_hmd_state->SensorFrames[1].RawAccel.k);

			// Two frames: [[wx0, wy0, wz0], [wx1, wy1, wz1]] 
			// Take the most recent frame: [wx1, wy1, wz1]
			raw_sensor_data->mutable_gyroscope()->set_i(morpheus_hmd_state->SensorFrames[1].RawGyro.i);
			raw_sensor_data->mutable_gyroscope()->set_j(morpheus_hmd_state->SensorFrames[1].RawGyro.j);
			raw_sensor_data->mutable_gyroscope()->set_k(morpheus_hmd_state->SensorFrames[1].RawGyro.k);
        }

		// If requested, get the raw sensor data for the hmd
		if (stream_info->include_calibrated_sensor_data)
		{
			PSMoveProtocol::DeviceOutputDataFrame_HMDDataPacket_MorpheusState_CalibratedSensorData *calibrated_sensor_data =
				morpheus_data_frame->mutable_calibrated_sensor_data();

			// Two frames: [[ax0, ay0, az0], [ax1, ay1, az1]] 
			// Take the most recent frame: [ax1, ay1, az1]
			calibrated_sensor_data->mutable_accelerometer()->set_i(morpheus_hmd_state->SensorFrames[1].CalibratedAccel.i);
			calibrated_sensor_data->mutable_accelerometer()->set_j(morpheus_hmd_state->SensorFrames[1].CalibratedAccel.j);
			calibrated_sensor_data->mutable_accelerometer()->set_k(morpheus_hmd_state->SensorFrames[1].CalibratedAccel.k);

			// Two frames: [[wx0, wy0, wz0], [wx1, wy1, wz1]] 
			// Take the most recent frame: [wx1, wy1, wz1]
			calibrated_sensor_data->mutable_gyroscope()->set_i(morpheus_hmd_state->SensorFrames[1].CalibratedGyro.i);
			calibrated_sensor_data->mutable_gyroscope()->set_j(morpheus_hmd_state->SensorFrames[1].CalibratedGyro.j);
			calibrated_sensor_data->mutable_gyroscope()->set_k(morpheus_hmd_state->SensorFrames[1].CalibratedGyro.k);
		}

		// If requested, get the raw tracker data for the controller
		if (stream_info->include_raw_tracker_data)
		{
			auto *raw_tracker_data = morpheus_data_frame->mutable_raw_tracker_data();
            int selectedTrackerId= stream_info->selected_tracker_index;
            unsigned int validTrackerBitmask= 0;

            for (int trackerId = 0; trackerId < TrackerManager::k_max_devices; ++trackerId)
            {
			    const HMDOpticalPoseEstimation *positionEstimate = hmd_view->getTrackerPoseEstimate(trackerId);

                if (positionEstimate != nullptr && positionEstimate->bCurrentlyTracking)
                {
                    validTrackerBitmask&= (1 << trackerId);

                    if (trackerId == selectedTrackerId)
                    {
				        const CommonDevicePosition &trackerRelativePosition = positionEstimate->position_cm;
				        const ServerTrackerViewPtr tracker_view = DeviceManager::getInstance()->getTrackerViewPtr(trackerId);

				        // Project the 3d camera position back onto the tracker screen
				        {
					        const CommonDeviceScreenLocation trackerScreenLocation =
						        tracker_view->projectTrackerRelativePosition(&trackerRelativePosition);
					        PSMoveProtocol::Pixel *pixel = raw_tracker_data->mutable_screen_location();

					        pixel->set_x(trackerScreenLocation.x);
					        pixel->set_y(trackerScreenLocation.y);
				        }

				        // Add the tracker relative 3d position
				        {
					        PSMoveProtocol::Position *position = raw_tracker_data->mutable_relative_position_cm();

					        position->set_x(trackerRelativePosition.x);
					        position->set_y(trackerRelativePosition.y);
					        position->set_z(trackerRelativePosition.z);
				        }

				        // Add the tracker relative projection shapes
				        {
					        const CommonDeviceTrackingProjection &trackerRelativeProjection =
						        positionEstimate->projection;

					        assert(trackerRelativeProjection.shape_type == eCommonTrackingProjectionType::ProjectionType_Points);
					        PSMoveProtocol::Polygon *polygon = raw_tracker_data->mutable_projected_point_cloud();

					        for (int vert_index = 0; vert_index < trackerRelativeProjection.shape.points.point_count; ++vert_index)
					        {
						        PSMoveProtocol::Pixel *pixel = polygon->add_vertices();

						        pixel->set_x(trackerRelativeProjection.shape.points.point[vert_index].x);
						        pixel->set_y(trackerRelativeProjection.shape.points.point[vert_index].y);
					        }
				        }

				        raw_tracker_data->set_tracker_id(trackerId);
                    }
                }
            }
            raw_tracker_data->set_valid_tracker_bitmask(validTrackerBitmask);
		}
    }

    hmd_data_frame->set_hmd_type(PSMoveProtocol::Morpheus);
}

static void generate_virtual_hmd_data_frame_for_stream(
    const ServerHMDView *hmd_view,
    const HMDStreamInfo *stream_info,
    DeviceOutputDataFramePtr &data_frame)
{
    const VirtualHMD *virtual_hmd = hmd_view->castCheckedConst<VirtualHMD>();
    const VirtualHMDConfig *virtual_hmd_config = virtual_hmd->getConfig();
	const IPoseFilter *pose_filter = hmd_view->getPoseFilter();
    const CommonHMDState *hmd_state = hmd_view->getState();
    const CommonDevicePose hmd_pose = hmd_view->getFilteredPose();

    PSMoveProtocol::DeviceOutputDataFrame_HMDDataPacket *hmd_data_frame = data_frame->mutable_hmd_data_packet();

    if (hmd_state != nullptr)
    {
        assert(hmd_state->DeviceType == CommonDeviceState::VirtualHMD);
        const VirtualHMDState * virtual_hmd_state = static_cast<const VirtualHMDState *>(hmd_state);

        auto * virtual_hmd_data_frame = hmd_data_frame->mutable_virtual_hmd_state();

		virtual_hmd_data_frame->set_iscurrentlytracking(hmd_view->getIsCurrentlyTracking());
		virtual_hmd_data_frame->set_istrackingenabled(hmd_view->getIsTrackingEnabled());
		virtual_hmd_data_frame->set_ispositionvalid(pose_filter->getIsStateValid());

		if (stream_info->include_position_data)
		{
			virtual_hmd_data_frame->mutable_position_cm()->set_x(hmd_pose.PositionCm.x);
			virtual_hmd_data_frame->mutable_position_cm()->set_y(hmd_pose.PositionCm.y);
			virtual_hmd_data_frame->mutable_position_cm()->set_z(hmd_pose.PositionCm.z);
		}
		else
		{
			virtual_hmd_data_frame->mutable_position_cm()->set_x(0);
			virtual_hmd_data_frame->mutable_position_cm()->set_y(0);
			virtual_hmd_data_frame->mutable_position_cm()->set_z(0);
		}

		// If requested, get the raw sensor data for the hmd
		if (stream_info->include_physics_data)
		{
			const CommonDevicePhysics hmd_physics = hmd_view->getFilteredPhysics();
			auto *physics_data = virtual_hmd_data_frame->mutable_physics_data();

			physics_data->mutable_velocity_cm_per_sec()->set_i(hmd_physics.VelocityCmPerSec.i);
			physics_data->mutable_velocity_cm_per_sec()->set_j(hmd_physics.VelocityCmPerSec.j);
			physics_data->mutable_velocity_cm_per_sec()->set_k(hmd_physics.VelocityCmPerSec.k);

			physics_data->mutable_acceleration_cm_per_sec_sqr()->set_i(hmd_physics.AccelerationCmPerSecSqr.i);
			physics_data->mutable_acceleration_cm_per_sec_sqr()->set_j(hmd_physics.AccelerationCmPerSecSqr.j);
			physics_data->mutable_acceleration_cm_per_sec_sqr()->set_k(hmd_physics.AccelerationCmPerSecSqr.k);
		}

		// If requested, get the raw tracker data for the controller
		if (stream_info->include_raw_tracker_data)
		{
			auto *raw_tracker_data = virtual_hmd_data_frame->mutable_raw_tracker_data();
			int selectedTrackerId= stream_info->selected_tracker_index;
            unsigned int validTrackerBitmask= 0;

            for (int trackerId = 0; trackerId < TrackerManager::k_max_devices; ++trackerId)
            {
			    const HMDOpticalPoseEstimation *positionEstimate = hmd_view->getTrackerPoseEstimate(trackerId);

                if (positionEstimate != nullptr && positionEstimate->bCurrentlyTracking)
                {
                    validTrackerBitmask&= (1 << trackerId);

                    if (trackerId == selectedTrackerId)
                    {
				        const CommonDevicePosition &trackerRelativePosition = positionEstimate->position_cm;
				        const ServerTrackerViewPtr tracker_view = DeviceManager::getInstance()->getTrackerViewPtr(selectedTrackerId);

				        // Project the 3d camera position back onto the tracker screen
				        {
					        const CommonDeviceScreenLocation trackerScreenLocation =
						        tracker_view->projectTrackerRelativePosition(&trackerRelativePosition);
					        PSMoveProtocol::Pixel *pixel = raw_tracker_data->mutable_screen_location();

					        pixel->set_x(trackerScreenLocation.x);
					        pixel->set_y(trackerScreenLocation.y);
				        }

				        // Add the tracker relative 3d position
				        {
					        PSMoveProtocol::Position *position = raw_tracker_data->mutable_relative_position_cm();

					        position->set_x(trackerRelativePosition.x);
					        position->set_y(trackerRelativePosition.y);
					        position->set_z(trackerRelativePosition.z);
				        }

				        // Add the tracker relative projection shapes
				        {
					        const CommonDeviceTrackingProjection &trackerRelativeProjection =
						        positionEstimate->projection;

					        assert(trackerRelativeProjection.shape_type == eCommonTrackingProjectionType::ProjectionType_Ellipse);
					        PSMoveProtocol::Ellipse *ellipse = raw_tracker_data->mutable_projected_sphere();

                            ellipse->mutable_center()->set_x(trackerRelativeProjection.shape.ellipse.center.x);
                            ellipse->mutable_center()->set_y(trackerRelativeProjection.shape.ellipse.center.y);
                            ellipse->set_half_x_extent(trackerRelativeProjection.shape.ellipse.half_x_extent);
                            ellipse->set_half_y_extent(trackerRelativeProjection.shape.ellipse.half_y_extent);
                            ellipse->set_angle(trackerRelativeProjection.shape.ellipse.angle);
				        }

				        raw_tracker_data->set_tracker_id(selectedTrackerId);
                    }
                }
            }
            raw_tracker_data->set_valid_tracker_bitmask(validTrackerBitmask);
		}
    }

    hmd_data_frame->set_hmd_type(PSMoveProtocol::VirtualHMD);
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

static void computeSpherePoseForHmdFromSingleTracker(
    const ServerHMDView *hmdView,
    const ServerTrackerViewPtr tracker,
    HMDOpticalPoseEstimation *tracker_pose_estimation,
    HMDOpticalPoseEstimation *multicam_pose_estimation)
{
    // No orientation for the sphere projection
    multicam_pose_estimation->orientation.clear();
    multicam_pose_estimation->bOrientationValid = false;

    // For the sphere projection, the tracker relative position has already been computed
    // Put the tracker relative position into world space
    multicam_pose_estimation->position_cm = tracker->computeWorldPosition(&tracker_pose_estimation->position_cm);
    multicam_pose_estimation->bCurrentlyTracking = true;

    // Copy over the screen projection area
    multicam_pose_estimation->projection.screen_area = tracker_pose_estimation->projection.screen_area;
}

static void computePointCloudPoseForHmdFromSingleTracker(
    const ServerHMDView *hmdView,
    const ServerTrackerViewPtr tracker,
    HMDOpticalPoseEstimation *tracker_pose_estimation,
    HMDOpticalPoseEstimation *multicam_pose_estimation)
{
    // No orientation for the sphere projection
    multicam_pose_estimation->orientation= tracker->computeWorldOrientation(&tracker_pose_estimation->orientation);
    multicam_pose_estimation->bOrientationValid = tracker_pose_estimation->bOrientationValid;

    // For the sphere projection, the tracker relative position has already been computed
    // Put the tracker relative position into world space
    multicam_pose_estimation->position_cm = tracker->computeWorldPosition(&tracker_pose_estimation->position_cm);
    multicam_pose_estimation->bCurrentlyTracking = true;

    // Copy over the screen projection area
    multicam_pose_estimation->projection.screen_area = tracker_pose_estimation->projection.screen_area;
}

static void computeSpherePoseForHmdFromMultipleTrackers(
    const ServerHMDView *hmdView,
    const TrackerManager* tracker_manager,
    const int *valid_projection_tracker_ids,
    const int projections_found,
    HMDOpticalPoseEstimation *tracker_pose_estimations,
    HMDOpticalPoseEstimation *multicam_pose_estimation)
{
    const TrackerManagerConfig &cfg = tracker_manager->getConfig();
    float screen_area_sum = 0;

    // Project the tracker relative 3d tracking position back on to the tracker camera plane
    // and sum up the total controller projection area across all trackers
    CommonDeviceScreenLocation position2d_list[TrackerManager::k_max_devices];
    for (int list_index = 0; list_index < projections_found; ++list_index)
    {
        const int tracker_id = valid_projection_tracker_ids[list_index];
        const ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);
        const HMDOpticalPoseEstimation &poseEstimate = tracker_pose_estimations[tracker_id];

        position2d_list[list_index] = tracker->projectTrackerRelativePosition(&poseEstimate.position_cm);
        screen_area_sum += poseEstimate.projection.screen_area;
    }

    // Compute triangulations amongst all pairs of projections
    int pair_count = 0;
    int biggest_prjection_id = -1;
    CommonDevicePosition average_world_position = { 0.f, 0.f, 0.f };
    for (int list_index = 0; list_index < projections_found; ++list_index)
    {
        const int tracker_id = valid_projection_tracker_ids[list_index];
        const CommonDeviceScreenLocation &screen_location = position2d_list[list_index];
        const ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);

        for (int other_list_index = list_index + 1; other_list_index < projections_found; ++other_list_index)
        {
            const int other_tracker_id = valid_projection_tracker_ids[other_list_index];
            const CommonDeviceScreenLocation &other_screen_location = position2d_list[other_list_index];
            const ServerTrackerViewPtr other_tracker = tracker_manager->getTrackerViewPtr(other_tracker_id);
            // if trackers are on opposite sides
            if (cfg.exclude_opposed_cameras)
            {
                if ((tracker->getTrackerPose().PositionCm.x > 0) == (other_tracker->getTrackerPose().PositionCm.x < 0) &&
                    (tracker->getTrackerPose().PositionCm.z > 0) == (other_tracker->getTrackerPose().PositionCm.z < 0))
                {
                    float screen_area = tracker_pose_estimations[tracker_id].projection.screen_area;
                    float other_screen_area = tracker_pose_estimations[other_tracker_id].projection.screen_area;

                    biggest_prjection_id = screen_area > other_screen_area ? tracker_id : other_tracker_id;

                    continue;
                }
            }

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

    if (pair_count == 0 && biggest_prjection_id >= 0 && !DeviceManager::getInstance()->m_tracker_manager->getConfig().ignore_pose_from_one_tracker)
    {
        // Position not triangulated from opposed camera, estimate from one tracker only.
        computeSpherePoseForHmdFromSingleTracker(
            hmdView,
            tracker_manager->getTrackerViewPtr(biggest_prjection_id),
            &tracker_pose_estimations[biggest_prjection_id],
            multicam_pose_estimation);		
    }
    else if(pair_count > 0)
    {
        // Compute the average position
        const float N = static_cast<float>(pair_count);

        average_world_position.x /= N;
        average_world_position.y /= N;
        average_world_position.z /= N;

        // Store the averaged tracking position
        const float q = tracker_manager->getConfig().controller_position_smoothing;
        if (q <= 0.01f)
        {
            multicam_pose_estimation->position_cm = average_world_position;
        }
        else
        {
            multicam_pose_estimation->position_cm.x = q * multicam_pose_estimation->position_cm.x + (1 - q) * average_world_position.x;
            multicam_pose_estimation->position_cm.y = q * multicam_pose_estimation->position_cm.y + (1 - q) * average_world_position.y;
            multicam_pose_estimation->position_cm.z = q * multicam_pose_estimation->position_cm.z + (1 - q) * average_world_position.z;
        }

        multicam_pose_estimation->bCurrentlyTracking = true;
    }

    // No orientation for the sphere projection
    multicam_pose_estimation->orientation.clear();
    multicam_pose_estimation->bOrientationValid = false;

    // Compute the average projection area.
    // This is proportional to our position tracking quality.
    multicam_pose_estimation->projection.screen_area =
        screen_area_sum / static_cast<float>(projections_found);
}

static void computePointCloudPoseForHmdFromMultipleTrackers(
    const ServerHMDView *hmdView,
    const TrackerManager* tracker_manager,
    const int *valid_projection_tracker_ids,
    const int projections_found,
    HMDOpticalPoseEstimation *tracker_pose_estimations,
    HMDOpticalPoseEstimation *multicam_pose_estimation)
{
    const TrackerManagerConfig &cfg = tracker_manager->getConfig();
    float screen_area_sum = 0;

    // Project the tracker relative 3d tracking position back on to the tracker camera plane
    // and sum up the total controller projection area across all trackers
    CommonDeviceScreenLocation position2d_list[TrackerManager::k_max_devices];
    for (int list_index = 0; list_index < projections_found; ++list_index)
    {
        const int tracker_id = valid_projection_tracker_ids[list_index];
        const ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);
        const HMDOpticalPoseEstimation &poseEstimate = tracker_pose_estimations[tracker_id];

        position2d_list[list_index] = tracker->projectTrackerRelativePosition(&poseEstimate.position_cm);
        screen_area_sum += poseEstimate.projection.screen_area;
    }

    // Compute triangulations amongst all pairs of projections
    int pair_count = 0;
    int biggest_prjection_id = -1;
    CommonDevicePosition average_world_position = { 0.f, 0.f, 0.f };
    for (int list_index = 0; list_index < projections_found; ++list_index)
    {
        const int tracker_id = valid_projection_tracker_ids[list_index];
        const CommonDeviceScreenLocation &screen_location = position2d_list[list_index];
        const ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);

        for (int other_list_index = list_index + 1; other_list_index < projections_found; ++other_list_index)
        {
            const int other_tracker_id = valid_projection_tracker_ids[other_list_index];
            const CommonDeviceScreenLocation &other_screen_location = position2d_list[other_list_index];
            const ServerTrackerViewPtr other_tracker = tracker_manager->getTrackerViewPtr(other_tracker_id);
            // if trackers are on opposite sides
            if (cfg.exclude_opposed_cameras)
            {
                if ((tracker->getTrackerPose().PositionCm.x > 0) == (other_tracker->getTrackerPose().PositionCm.x < 0) &&
                    (tracker->getTrackerPose().PositionCm.z > 0) == (other_tracker->getTrackerPose().PositionCm.z < 0))
                {
                    float screen_area = tracker_pose_estimations[tracker_id].projection.screen_area;
                    float other_screen_area = tracker_pose_estimations[other_tracker_id].projection.screen_area;

                    biggest_prjection_id = screen_area > other_screen_area ? tracker_id : other_tracker_id;

                    continue;
                }
            }

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

    if (pair_count == 0 && biggest_prjection_id >= 0 && !DeviceManager::getInstance()->m_tracker_manager->getConfig().ignore_pose_from_one_tracker)
    {
        // Position not triangulated from opposed camera, estimate from one tracker only.
        computePointCloudPoseForHmdFromSingleTracker(
            hmdView,
            tracker_manager->getTrackerViewPtr(biggest_prjection_id),
            &tracker_pose_estimations[biggest_prjection_id],
            multicam_pose_estimation);		
    }
    else if(pair_count > 0)
    {
        // Compute the average position
        const float N = static_cast<float>(pair_count);

        average_world_position.x /= N;
        average_world_position.y /= N;
        average_world_position.z /= N;

        // Store the averaged tracking position
        const float q = tracker_manager->getConfig().controller_position_smoothing;
        if (q <= 0.01f)
        {
            multicam_pose_estimation->position_cm = average_world_position;
        }
        else
        {
            multicam_pose_estimation->position_cm.x = q * multicam_pose_estimation->position_cm.x + (1 - q) * average_world_position.x;
            multicam_pose_estimation->position_cm.y = q * multicam_pose_estimation->position_cm.y + (1 - q) * average_world_position.y;
            multicam_pose_estimation->position_cm.z = q * multicam_pose_estimation->position_cm.z + (1 - q) * average_world_position.z;
        }

        multicam_pose_estimation->bCurrentlyTracking = true;
    }

    // No orientation for the sphere projection
    multicam_pose_estimation->orientation.clear();
    multicam_pose_estimation->bOrientationValid = false;

    // Compute the average projection area.
    // This is proportional to our position tracking quality.
    multicam_pose_estimation->projection.screen_area =
        screen_area_sum / static_cast<float>(projections_found);
}
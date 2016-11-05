//-- includes -----
#include "ClientHMDView.h"
#include "MathUtility.h"
#include "PSMoveProtocol.pb.h"
#include <chrono>
#include <assert.h>

//-- pre-declarations -----

//-- constants -----
const MorpheusPhysicsData k_empty_physics_data = { { 0.f, 0.f, 0.f },{ 0.f, 0.f, 0.f },{ 0.f, 0.f, 0.f },{ 0.f, 0.f, 0.f } };
const MorpheusRawSensorData k_empty_psmove_raw_sensor_data = { { 0, 0, 0 },{ 0, 0, 0 } };
const MorpheusCalibratedSensorData k_empty_psmove_calibrated_sensor_data = { { 0.f, 0.f, 0.f },{ 0.f, 0.f, 0.f } };
const PSMoveFloatVector3 k_identity_gravity_calibration_direction = { 0.f, 1.f, 0.f };
const MorpheusRawTrackerData k_empty_raw_tracker_data = { 0 };

//-- prototypes ----

//-- implementation -----

//-- ClientMorpheusView -----
void ClientMorpheusView::Clear()
{
	bValid = false;
	bIsTrackingEnabled = false;
	bIsCurrentlyTracking = false;

	Pose.Clear();
	PhysicsData.Clear();
	RawSensorData.Clear();
	RawTrackerData.Clear();
}

const MorpheusPhysicsData &ClientMorpheusView::GetPhysicsData() const
{
	return IsValid() ? PhysicsData : k_empty_physics_data;
}

const MorpheusRawSensorData &ClientMorpheusView::GetRawSensorData() const
{
	return IsValid() ? RawSensorData : k_empty_psmove_raw_sensor_data;
}

const MorpheusCalibratedSensorData &ClientMorpheusView::GetCalibratedSensorData() const
{
	return IsValid() ? CalibratedSensorData : k_empty_psmove_calibrated_sensor_data;
}

bool ClientMorpheusView::GetIsStable() const
{
	return GetIsStableAndAlignedWithGravity();
}

bool ClientMorpheusView::GetIsStableAndAlignedWithGravity() const
{
	const float k_cosine_20_degrees = 0.9396926f;

	// Get the direction the gravity vector should be pointing 
	// while the controller is in cradle pose.
	PSMoveFloatVector3 acceleration_direction = CalibratedSensorData.Accelerometer;
	const float acceleration_magnitude = acceleration_direction.normalize_with_default(*k_psmove_float_vector3_zero);

	const bool isOk =
		is_nearly_equal(1.f, acceleration_magnitude, 0.1f) &&
		PSMoveFloatVector3::dot(k_identity_gravity_calibration_direction, acceleration_direction) >= k_cosine_20_degrees;

	return isOk;
}

const MorpheusRawTrackerData &ClientMorpheusView::GetRawTrackerData() const
{
	return RawTrackerData;
}

void ClientMorpheusView::ApplyHMDDataFrame(
    const PSMoveProtocol::DeviceOutputDataFrame_HMDDataPacket *data_frame)
{
    if (data_frame->isconnected())
    {
        const auto &morpheus_data_frame = data_frame->morpheus_state();

		this->bIsTrackingEnabled = morpheus_data_frame.istrackingenabled();
		this->bIsCurrentlyTracking = morpheus_data_frame.iscurrentlytracking();
		this->bIsOrientationValid = morpheus_data_frame.isorientationvalid();
		this->bIsPositionValid = morpheus_data_frame.ispositionvalid();

		this->Pose.Orientation.w = morpheus_data_frame.orientation().w();
		this->Pose.Orientation.x = morpheus_data_frame.orientation().x();
		this->Pose.Orientation.y = morpheus_data_frame.orientation().y();
		this->Pose.Orientation.z = morpheus_data_frame.orientation().z();

		this->Pose.Position.x = morpheus_data_frame.position().x();
		this->Pose.Position.y = morpheus_data_frame.position().y();
		this->Pose.Position.z = morpheus_data_frame.position().z();

		if (morpheus_data_frame.has_raw_sensor_data())
		{
			const auto &raw_sensor_data = morpheus_data_frame.raw_sensor_data();

			this->RawSensorData.Accelerometer.i = raw_sensor_data.accelerometer().i();
			this->RawSensorData.Accelerometer.j = raw_sensor_data.accelerometer().j();
			this->RawSensorData.Accelerometer.k = raw_sensor_data.accelerometer().k();

			this->RawSensorData.Gyroscope.i = raw_sensor_data.gyroscope().i();
			this->RawSensorData.Gyroscope.j = raw_sensor_data.gyroscope().j();
			this->RawSensorData.Gyroscope.k = raw_sensor_data.gyroscope().k();
		}
		else
		{
			this->RawSensorData.Clear();
		}

		if (morpheus_data_frame.has_calibrated_sensor_data())
		{
			const auto &calibrated_sensor_data = morpheus_data_frame.calibrated_sensor_data();

			this->CalibratedSensorData.Accelerometer.i = calibrated_sensor_data.accelerometer().i();
			this->CalibratedSensorData.Accelerometer.j = calibrated_sensor_data.accelerometer().j();
			this->CalibratedSensorData.Accelerometer.k = calibrated_sensor_data.accelerometer().k();

			this->CalibratedSensorData.Gyroscope.i = calibrated_sensor_data.gyroscope().i();
			this->CalibratedSensorData.Gyroscope.j = calibrated_sensor_data.gyroscope().j();
			this->CalibratedSensorData.Gyroscope.k = calibrated_sensor_data.gyroscope().k();
		}
		else
		{
			this->CalibratedSensorData.Clear();
		}

		if (morpheus_data_frame.has_raw_tracker_data())
		{
			const auto &raw_tracker_data = morpheus_data_frame.raw_tracker_data();

			this->RawTrackerData.ValidTrackerLocations =
				std::min(raw_tracker_data.valid_tracker_count(), PSMOVESERVICE_MAX_TRACKER_COUNT);

			for (int listIndex = 0; listIndex < this->RawTrackerData.ValidTrackerLocations; ++listIndex)
			{
				const PSMoveProtocol::Pixel &locationOnTracker = raw_tracker_data.screen_locations(listIndex);
				const PSMoveProtocol::Position &positionOnTracker = raw_tracker_data.relative_positions(listIndex);

				this->RawTrackerData.TrackerIDs[listIndex] = raw_tracker_data.tracker_ids(listIndex);
				this->RawTrackerData.ScreenLocations[listIndex] =
					PSMoveScreenLocation::create(locationOnTracker.x(), locationOnTracker.y());
				this->RawTrackerData.RelativePositions[listIndex] =
					PSMovePosition::create(
						positionOnTracker.x(), positionOnTracker.y(), positionOnTracker.z());

				if (raw_tracker_data.projected_point_cloud_size() > 0)
				{
					const PSMoveProtocol::Polygon &protocolPointCloud = raw_tracker_data.projected_point_cloud(listIndex);
					PSMoveTrackingProjection &projection = this->RawTrackerData.TrackingProjections[listIndex];

					projection.shape.pointcloud.point_count = std::min(protocolPointCloud.vertices_size(), 7);
					for (int point_index = 0; point_index < projection.shape.pointcloud.point_count; ++point_index)
					{
						const PSMoveProtocol::Pixel &point= protocolPointCloud.vertices(point_index);

						projection.shape.pointcloud.points[point_index].x = point.x();
						projection.shape.pointcloud.points[point_index].y = point.y();
					}					
					projection.shape_type = PSMoveTrackingProjection::eShapeType::PointCloud;
				}
				else
				{
					PSMoveTrackingProjection &projection = this->RawTrackerData.TrackingProjections[listIndex];

					projection.shape_type = PSMoveTrackingProjection::eShapeType::INVALID_PROJECTION;
				}
			}
		}
		else
		{
			this->RawTrackerData.Clear();
		}

		if (morpheus_data_frame.has_physics_data())
		{
			const auto &raw_physics_data = morpheus_data_frame.physics_data();

			this->PhysicsData.Velocity.i = raw_physics_data.velocity().i();
			this->PhysicsData.Velocity.j = raw_physics_data.velocity().j();
			this->PhysicsData.Velocity.k = raw_physics_data.velocity().k();

			this->PhysicsData.Acceleration.i = raw_physics_data.acceleration().i();
			this->PhysicsData.Acceleration.j = raw_physics_data.acceleration().j();
			this->PhysicsData.Acceleration.k = raw_physics_data.acceleration().k();

			this->PhysicsData.AngularVelocity.i = raw_physics_data.angular_velocity().i();
			this->PhysicsData.AngularVelocity.j = raw_physics_data.angular_velocity().j();
			this->PhysicsData.AngularVelocity.k = raw_physics_data.angular_velocity().k();

			this->PhysicsData.AngularAcceleration.i = raw_physics_data.angular_acceleration().i();
			this->PhysicsData.AngularAcceleration.j = raw_physics_data.angular_acceleration().j();
			this->PhysicsData.AngularAcceleration.k = raw_physics_data.angular_acceleration().k();
		}
		else
		{
			this->PhysicsData.Clear();
		}

        this->bValid = true;
    }
    else
    {
        Clear();
    }
}

//-- ClientHMDView -----
ClientHMDView::ClientHMDView(int HmdID)
{
    Clear();
    this->HmdID = HmdID;
}

void ClientHMDView::Clear()
{
    HmdID = -1;
    SequenceNum = -1;
    ListenerCount = 0;

    IsConnected = false;

    HMDViewType = None;
    memset(&ViewState, 0, sizeof(ViewState));

    data_frame_last_received_time =
        std::chrono::duration_cast< std::chrono::milliseconds >(
        std::chrono::system_clock::now().time_since_epoch()).count();
    data_frame_average_fps = 0.f;
}

void ClientHMDView::ApplyHMDDataFrame(
    const PSMoveProtocol::DeviceOutputDataFrame_HMDDataPacket *data_frame)
{
    assert(data_frame->hmd_id() == HmdID);

    // Compute the data frame receive window statistics if we have received enough samples
    {
        long long now =
            std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()).count();
        long long diff = now - data_frame_last_received_time;

        if (diff > 0)
        {
            float seconds = static_cast<float>(diff) / 1000.f;
            float fps = 1.f / seconds;

            data_frame_average_fps = (0.9f)*data_frame_average_fps + (0.1f)*fps;
        }

        data_frame_last_received_time = now;
    }

    if (data_frame->sequence_num() > this->SequenceNum)
    {
        this->SequenceNum = data_frame->sequence_num();
        this->IsConnected = data_frame->isconnected();

        switch (data_frame->hmd_type())
        {
        case PSMoveProtocol::Morpheus:
        {
            this->HMDViewType = Morpheus;
            this->ViewState.MorpheusView.ApplyHMDDataFrame(data_frame);
        } break;

        default:
            assert(0 && "Unhandled HMD type");
        }
    }
}

const PSMovePose &ClientHMDView::GetPose() const
{
	switch (HMDViewType)
	{
	case eHMDViewType::Morpheus:
		return GetMorpheusView().GetPose();
	default:
		assert(0 && "invalid HMD type");
		return *k_psmove_pose_identity;
	}
}

const PSMovePosition &ClientHMDView::GetPosition() const
{
	switch (HMDViewType)
	{
	case eHMDViewType::Morpheus:
		return GetMorpheusView().GetPosition();
	default:
		assert(0 && "invalid HMD type");
		return *k_psmove_position_origin;
	}
}

const PSMoveQuaternion &ClientHMDView::GetOrientation() const
{
	switch (HMDViewType)
	{
	case eHMDViewType::Morpheus:
		return GetMorpheusView().GetOrientation();
	default:
		assert(0 && "invalid HMD type");
		return *k_psmove_quaternion_identity;
	}
}

const MorpheusPhysicsData &ClientHMDView::GetPhysicsData() const
{
	switch (HMDViewType)
	{
	case eHMDViewType::Morpheus:
		return GetMorpheusView().GetPhysicsData();
	default:
		assert(0 && "invalid HMD type");
		return k_empty_physics_data;
	}
}

const MorpheusRawTrackerData &ClientHMDView::GetRawTrackerData() const
{
	switch (HMDViewType)
	{
	case eHMDViewType::Morpheus:
		return GetMorpheusView().GetRawTrackerData();
	default:
		assert(0 && "invalid HMD type");
		return k_empty_raw_tracker_data;
	}
}

bool ClientHMDView::GetIsCurrentlyTracking() const
{
	switch (HMDViewType)
	{
	case eHMDViewType::Morpheus:
		return GetMorpheusView().GetIsCurrentlyTracking();
	default:
		assert(0 && "invalid HMD type");
		return false;
	}
}

bool ClientHMDView::GetIsPoseValid() const
{
	switch (HMDViewType)
	{
	case eHMDViewType::Morpheus:
		return GetMorpheusView().GetIsOrientationValid() && GetMorpheusView().GetIsPositionValid();
	default:
		assert(0 && "invalid HMD type");
		return false;
	}
}

bool ClientHMDView::GetIsStable() const
{
	switch (HMDViewType)
	{
	case eHMDViewType::Morpheus:
		return GetMorpheusView().GetIsStable();
	default:
		assert(0 && "invalid HMD type");
		return true;
	}
}

//-- includes -----
#include "ClientControllerView.h"
#include "ClientNetworkManager.h"
#include "ClientPSMoveAPI.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include "MathUtility.h"
#include <chrono>
#include <algorithm>
#include <assert.h>

//-- pre-declarations -----

//-- constants -----
const PSMovePhysicsData k_empty_physics_data = { { 0.f, 0.f, 0.f }, { 0.f, 0.f, 0.f }, { 0.f, 0.f, 0.f }, { 0.f, 0.f, 0.f } };
const PSMoveRawSensorData k_empty_psmove_raw_sensor_data = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
const PSMoveCalibratedSensorData k_empty_psmove_calibrated_sensor_data = { { 0.f, 0.f, 0.f }, { 0.f, 0.f, 0.f }, { 0.f, 0.f, 0.f } };
const PSDualShock4RawSensorData k_empty_ds4_raw_sensor_data = { { 0, 0, 0 }, { 0, 0, 0 } };
const PSDualShock4CalibratedSensorData k_empty_ds4_calibrated_sensor_data = { { 0.f, 0.f, 0.f }, { 0.f, 0.f, 0.f } };
const PSMoveFloatVector3 k_identity_gravity_calibration_direction= {0.f, 1.f, 0.f};
const PSMoveRawTrackerData k_empty_raw_tracker_data = { 0 };

//-- prototypes ----
static void update_button_state(PSMoveButtonState &button, unsigned int button_bitmask, unsigned int button_bit);

//-- implementation -----

//-- ClientPSMoveView -----
void ClientPSMoveView::Clear()
{
    bValid= false;
    bHasValidHardwareCalibration= false;
    bIsTrackingEnabled= false;
    bIsCurrentlyTracking= false;
    bHasUnpublishedState = false;

    Pose.Clear();
    PhysicsData.Clear();
    RawSensorData.Clear();
    RawTrackerData.Clear();

    TriangleButton= PSMoveButton_UP;
    CircleButton= PSMoveButton_UP;
    CrossButton= PSMoveButton_UP;
    SquareButton= PSMoveButton_UP;
    SelectButton= PSMoveButton_UP;
    StartButton= PSMoveButton_UP;
    PSButton= PSMoveButton_UP;
    MoveButton= PSMoveButton_UP;
    TriggerButton= PSMoveButton_UP;

    TriggerValue= 0;

	ResetPoseButtonPressTime = 0;
	bResetPoseRequestSent = false;
	bPoseResetButtonEnabled = false;
}

const PSMovePhysicsData &ClientPSMoveView::GetPhysicsData() const
{
    return IsValid() ? PhysicsData : k_empty_physics_data;
}

const PSMoveRawSensorData &ClientPSMoveView::GetRawSensorData() const
{
    return IsValid() ? RawSensorData : k_empty_psmove_raw_sensor_data;
}

const PSMoveCalibratedSensorData &ClientPSMoveView::GetCalibratedSensorData() const
{
    return IsValid() ? CalibratedSensorData : k_empty_psmove_calibrated_sensor_data;
}

bool ClientPSMoveView::GetIsStable() const
{
    return GetIsStableAndAlignedWithGravity();
}

bool ClientPSMoveView::GetIsGyroStable() const
{
	const float k_gyro_noise = 10.f*k_degrees_to_radians; // noise threshold in rad/sec
	const float worst_rotation_rate = fabsf(CalibratedSensorData.Gyroscope.maxValue());
	const bool isOk = worst_rotation_rate < k_gyro_noise;

	return isOk;
}

bool ClientPSMoveView::GetIsStableAndAlignedWithGravity() const
{
    const float k_cosine_10_degrees = 0.984808f;

    // Get the direction the gravity vector should be pointing 
    // while the controller is in cradle pose.
    PSMoveFloatVector3 acceleration_direction = CalibratedSensorData.Accelerometer;
    const float acceleration_magnitude = acceleration_direction.normalize_with_default(*k_psmove_float_vector3_zero);

    const bool isOk =
        is_nearly_equal(1.f, acceleration_magnitude, 0.1f) &&
        PSMoveFloatVector3::dot(k_identity_gravity_calibration_direction, acceleration_direction) >= k_cosine_10_degrees;

    return isOk;
}

const PSMoveRawTrackerData &ClientPSMoveView::GetRawTrackerData() const
{    
    return RawTrackerData;
}

void ClientPSMoveView::ApplyControllerDataFrame(
	class ClientControllerView *parentView,
    const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket *data_frame)
{
    if (data_frame->isconnected())
    {
        const auto &psmove_data_frame = data_frame->psmove_state();

        this->bHasValidHardwareCalibration= psmove_data_frame.validhardwarecalibration();
        this->bIsTrackingEnabled= psmove_data_frame.istrackingenabled();
        this->bIsCurrentlyTracking= psmove_data_frame.iscurrentlytracking();
        this->bIsOrientationValid = psmove_data_frame.isorientationvalid();
        this->bIsPositionValid = psmove_data_frame.ispositionvalid();

        this->Pose.Orientation.w= psmove_data_frame.orientation().w();
        this->Pose.Orientation.x= psmove_data_frame.orientation().x();
        this->Pose.Orientation.y= psmove_data_frame.orientation().y();
        this->Pose.Orientation.z= psmove_data_frame.orientation().z();

        this->Pose.Position.x= psmove_data_frame.position_cm().x();
        this->Pose.Position.y= psmove_data_frame.position_cm().y();
        this->Pose.Position.z= psmove_data_frame.position_cm().z();

        if (psmove_data_frame.has_raw_sensor_data())
        {
            const auto &raw_sensor_data = psmove_data_frame.raw_sensor_data();

            this->RawSensorData.Magnetometer.i= raw_sensor_data.magnetometer().i();
            this->RawSensorData.Magnetometer.j= raw_sensor_data.magnetometer().j();
            this->RawSensorData.Magnetometer.k= raw_sensor_data.magnetometer().k();

            this->RawSensorData.Accelerometer.i= raw_sensor_data.accelerometer().i();
            this->RawSensorData.Accelerometer.j= raw_sensor_data.accelerometer().j();
            this->RawSensorData.Accelerometer.k= raw_sensor_data.accelerometer().k();

            this->RawSensorData.Gyroscope.i= raw_sensor_data.gyroscope().i();
            this->RawSensorData.Gyroscope.j= raw_sensor_data.gyroscope().j();
            this->RawSensorData.Gyroscope.k= raw_sensor_data.gyroscope().k();
        }
        else
        {
            this->RawSensorData.Clear();
        }

        if (psmove_data_frame.has_calibrated_sensor_data())
        {
            const auto &calibrated_sensor_data = psmove_data_frame.calibrated_sensor_data();

            this->CalibratedSensorData.Magnetometer.i = calibrated_sensor_data.magnetometer().i();
            this->CalibratedSensorData.Magnetometer.j = calibrated_sensor_data.magnetometer().j();
            this->CalibratedSensorData.Magnetometer.k = calibrated_sensor_data.magnetometer().k();

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

        if (psmove_data_frame.has_raw_tracker_data())
        {
            const auto &raw_tracker_data = psmove_data_frame.raw_tracker_data();

            this->RawTrackerData.ValidTrackerLocations = 
                std::min(raw_tracker_data.valid_tracker_count(), PSMOVESERVICE_MAX_TRACKER_COUNT);

            for (int listIndex = 0; listIndex < this->RawTrackerData.ValidTrackerLocations; ++listIndex)
            {
                const PSMoveProtocol::Pixel &locationOnTracker = raw_tracker_data.screen_locations(listIndex);
                const PSMoveProtocol::Position &positionOnTracker = raw_tracker_data.relative_positions_cm(listIndex);

                this->RawTrackerData.TrackerIDs[listIndex]= raw_tracker_data.tracker_ids(listIndex);
                this->RawTrackerData.ScreenLocations[listIndex] =
                    PSMoveScreenLocation::create(locationOnTracker.x(), locationOnTracker.y());
                this->RawTrackerData.RelativePositionsCm[listIndex] =
                    PSMovePosition::create(
                        positionOnTracker.x(), positionOnTracker.y(), positionOnTracker.z());
				this->RawTrackerData.RelativeOrientations[listIndex] = PSMoveQuaternion::create(1.f, 0.f, 0.f, 0.f);

                if (raw_tracker_data.projected_spheres_size() > 0)
                {
                    const PSMoveProtocol::Ellipse &protocolEllipse = raw_tracker_data.projected_spheres(listIndex);
                    PSMoveTrackingProjection &projection= this->RawTrackerData.TrackingProjections[listIndex];

                    projection.shape.ellipse.center.x = protocolEllipse.center().x();
                    projection.shape.ellipse.center.y = protocolEllipse.center().y();
                    projection.shape.ellipse.half_x_extent = protocolEllipse.half_x_extent();
                    projection.shape.ellipse.half_y_extent = protocolEllipse.half_y_extent();
                    projection.shape.ellipse.angle = protocolEllipse.angle();
                    projection.shape_type = PSMoveTrackingProjection::eShapeType::Ellipse;
                }
                else
                {
                    PSMoveTrackingProjection &projection = this->RawTrackerData.TrackingProjections[listIndex];

                    projection.shape_type = PSMoveTrackingProjection::eShapeType::INVALID_PROJECTION;
                }
            }            

			if (raw_tracker_data.has_multicam_position_cm())
			{
				const PSMoveProtocol::Position &multicam_position = raw_tracker_data.multicam_position_cm();

				this->RawTrackerData.MulticamPositionCm.x = multicam_position.x();
				this->RawTrackerData.MulticamPositionCm.y = multicam_position.y();
				this->RawTrackerData.MulticamPositionCm.z = multicam_position.z();
				this->RawTrackerData.bMulticamPositionValid = true;
			}
        }
        else
        {
            this->RawTrackerData.Clear();
        }

        if (psmove_data_frame.has_physics_data())
        {
            const auto &raw_physics_data = psmove_data_frame.physics_data();

            this->PhysicsData.VelocityCmPerSec.i = raw_physics_data.velocity_cm_per_sec().i();
            this->PhysicsData.VelocityCmPerSec.j = raw_physics_data.velocity_cm_per_sec().j();
            this->PhysicsData.VelocityCmPerSec.k = raw_physics_data.velocity_cm_per_sec().k();

            this->PhysicsData.AccelerationCmPerSecSqr.i = raw_physics_data.acceleration_cm_per_sec_sqr().i();
            this->PhysicsData.AccelerationCmPerSecSqr.j = raw_physics_data.acceleration_cm_per_sec_sqr().j();
            this->PhysicsData.AccelerationCmPerSecSqr.k = raw_physics_data.acceleration_cm_per_sec_sqr().k();

            this->PhysicsData.AngularVelocityRadPerSec.i = raw_physics_data.angular_velocity_rad_per_sec().i();
            this->PhysicsData.AngularVelocityRadPerSec.j = raw_physics_data.angular_velocity_rad_per_sec().j();
            this->PhysicsData.AngularVelocityRadPerSec.k = raw_physics_data.angular_velocity_rad_per_sec().k();

            this->PhysicsData.AngularAccelerationRadPerSecSqr.i = raw_physics_data.angular_acceleration_rad_per_sec_sqr().i();
            this->PhysicsData.AngularAccelerationRadPerSecSqr.j = raw_physics_data.angular_acceleration_rad_per_sec_sqr().j();
            this->PhysicsData.AngularAccelerationRadPerSecSqr.k = raw_physics_data.angular_acceleration_rad_per_sec_sqr().k();
        }
        else
        {
            this->PhysicsData.Clear();
        }

        unsigned int button_bitmask= data_frame->button_down_bitmask();
        update_button_state(TriangleButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_TRIANGLE);
        update_button_state(CircleButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_CIRCLE);
        update_button_state(CrossButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_CROSS);
        update_button_state(SquareButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_SQUARE);
        update_button_state(SelectButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_SELECT);
        update_button_state(StartButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_START);
        update_button_state(PSButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_PS);
        update_button_state(MoveButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_MOVE);
        update_button_state(TriggerButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_TRIGGER);

        //###bwalker $TODO make sure this is in the range [0, 255]
        this->TriggerValue= static_cast<unsigned char>(psmove_data_frame.trigger_value());

		// Battery level range [0, 5] - EE charging & EF full
		this->BatteryValue = static_cast<PSMoveBatteryState>(psmove_data_frame.battery_value());

        this->bValid= true;

		// Fire off a recenter action if the recenter button has been held for long enough
		ProcessRecenterAction(parentView);
    }
    else
    {
        Clear();
    }
}

void ClientPSMoveView::ProcessRecenterAction(class ClientControllerView *parentView)
{
	if (bPoseResetButtonEnabled)
	{
		long long now =
			std::chrono::duration_cast< std::chrono::milliseconds >(
				std::chrono::system_clock::now().time_since_epoch()).count();

		PSMoveButtonState resetPoseButtonState = GetButtonSelect();

		switch (resetPoseButtonState)
		{
		case PSMoveButtonState::PSMoveButton_PRESSED:
			{
				ResetPoseButtonPressTime = now;
			} break;
		case PSMoveButtonState::PSMoveButton_DOWN:
			{
				if (!bResetPoseRequestSent)
				{
					const long long k_hold_duration_milli = 250;
					long long pressDurationMilli = now - ResetPoseButtonPressTime;

					if (pressDurationMilli >= k_hold_duration_milli)
					{
						ClientPSMoveAPI::eat_response(
							ClientPSMoveAPI::reset_orientation(parentView, PSMoveQuaternion::identity()));
						bResetPoseRequestSent = true;
					}
				}
			} break;
		case PSMoveButtonState::PSMoveButton_RELEASED:
			{
				bResetPoseRequestSent = false;
			} break;
		}
	}
}

void ClientPSMoveView::Publish(
    PSMoveProtocol::DeviceInputDataFrame_ControllerDataPacket *data_frame)
{
    auto *psmove_state = data_frame->mutable_psmove_state();

    psmove_state->set_led_r(this->LED_r);
    psmove_state->set_led_g(this->LED_g);
    psmove_state->set_led_b(this->LED_b);
    psmove_state->set_rumble_value(this->Rumble);

    bHasUnpublishedState = false;
}

void ClientPSMoveView::SetRumble(float rumbleFraction)
{
    unsigned char newRumble = static_cast<unsigned char>(clampf01(rumbleFraction)*255.f);

    if (newRumble != Rumble)
    {
        Rumble = newRumble;

        bHasUnpublishedState = true;
    }
}

void ClientPSMoveView::SetLEDOverride(unsigned char r, unsigned char g, unsigned char b)
{
    if (r != LED_r || g != LED_g || b != LED_b)
    {
        LED_r = r;
        LED_g = g;
        LED_b = b;

        bHasUnpublishedState = true;
    }
}

//-- ClientPSNaviView -----
void ClientPSNaviView::Clear()
{
    bValid= false;

    L1Button= PSMoveButton_UP;
    L2Button= PSMoveButton_UP;
    L3Button= PSMoveButton_UP;
    CircleButton= PSMoveButton_UP;
    CrossButton= PSMoveButton_UP;
    PSButton= PSMoveButton_UP;
    TriggerButton= PSMoveButton_UP;
    DPadUpButton= PSMoveButton_UP;
    DPadRightButton= PSMoveButton_UP;
    DPadDownButton= PSMoveButton_UP;
    DPadLeftButton= PSMoveButton_UP;

    TriggerValue= 0;
    Stick_XAxis= 0x80;
    Stick_YAxis= 0x80;
}

void ClientPSNaviView::ApplyControllerDataFrame(const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket *data_frame)
{
    if (data_frame->isconnected())
    {
        const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_PSNaviState &psnavi_data_frame= data_frame->psnavi_state();

        unsigned int button_bitmask= data_frame->button_down_bitmask();
        update_button_state(L1Button, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_L1);
        update_button_state(L2Button, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_L2);
        update_button_state(L3Button, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_L3);
        update_button_state(CircleButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_CIRCLE);
        update_button_state(CrossButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_CROSS);
        update_button_state(PSButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_PS);
        update_button_state(TriggerButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_TRIGGER);
        update_button_state(DPadUpButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_UP);
        update_button_state(DPadRightButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_RIGHT);
        update_button_state(DPadDownButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_DOWN);
        update_button_state(DPadLeftButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_LEFT);

        //###bwalker $TODO make sure this is in the range [0, 255]
        this->TriggerValue= static_cast<unsigned char>(psnavi_data_frame.trigger_value());
        this->Stick_XAxis= static_cast<unsigned char>(psnavi_data_frame.stick_xaxis());
        this->Stick_YAxis= static_cast<unsigned char>(psnavi_data_frame.stick_yaxis());

        this->bValid= true;
    }
    else
    {
        Clear();
    }
}

void ClientPSNaviView::Publish(PSMoveProtocol::DeviceInputDataFrame_ControllerDataPacket *data_frame)
{
    // Nothing to publish
}

//-- ClientPSDualShock4View -----
void ClientPSDualShock4View::Clear()
{
    bValid = false;
    bHasValidHardwareCalibration = false;
    bIsTrackingEnabled = false;
    bIsCurrentlyTracking = false;
    bHasUnpublishedState = false;

    Pose.Clear();
    PhysicsData.Clear();
    RawSensorData.Clear();
    RawTrackerData.Clear();

    DPadUpButton = PSMoveButton_UP;
    DPadDownButton = PSMoveButton_UP;
    DPadLeftButton = PSMoveButton_UP;
    DPadRightButton = PSMoveButton_UP;

    TriangleButton = PSMoveButton_UP;
    CircleButton = PSMoveButton_UP;
    CrossButton = PSMoveButton_UP;
    SquareButton = PSMoveButton_UP;

    L1Button = PSMoveButton_UP;
    R1Button = PSMoveButton_UP;
    L2Button = PSMoveButton_UP;
    R2Button = PSMoveButton_UP;
    L3Button = PSMoveButton_UP;
    R3Button = PSMoveButton_UP;

    ShareButton = PSMoveButton_UP;
    OptionsButton = PSMoveButton_UP;

    PSButton = PSMoveButton_UP;
    TrackPadButton = PSMoveButton_UP;

    LeftAnalogX= 0.f;
    LeftAnalogY = 0.f;
    RightAnalogX = 0.f;
    RightAnalogY = 0.f;
    LeftTriggerValue = 0.f;
    RightTriggerValue = 0.f;

	ResetPoseButtonPressTime = 0;
	bResetPoseRequestSent= false;
	bPoseResetButtonEnabled = false;
}

void ClientPSDualShock4View::ApplyControllerDataFrame(
	class ClientControllerView *parentView,
	const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket *data_frame)
{
    if (data_frame->isconnected())
    {
        const auto &psds4_data_frame = data_frame->psdualshock4_state();

        this->bHasValidHardwareCalibration = psds4_data_frame.validhardwarecalibration();
        this->bIsTrackingEnabled = psds4_data_frame.istrackingenabled();
        this->bIsCurrentlyTracking = psds4_data_frame.iscurrentlytracking();
        this->bIsOrientationValid = psds4_data_frame.isorientationvalid();
        this->bIsPositionValid = psds4_data_frame.ispositionvalid();

        this->Pose.Orientation.w = psds4_data_frame.orientation().w();
        this->Pose.Orientation.x = psds4_data_frame.orientation().x();
        this->Pose.Orientation.y = psds4_data_frame.orientation().y();
        this->Pose.Orientation.z = psds4_data_frame.orientation().z();

        this->Pose.Position.x = psds4_data_frame.position_cm().x();
        this->Pose.Position.y = psds4_data_frame.position_cm().y();
        this->Pose.Position.z = psds4_data_frame.position_cm().z();

        if (psds4_data_frame.has_raw_sensor_data())
        {
            const auto &raw_sensor_data = psds4_data_frame.raw_sensor_data();

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

        if (psds4_data_frame.has_calibrated_sensor_data())
        {
            const auto &calibrated_sensor_data = psds4_data_frame.calibrated_sensor_data();

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

        if (psds4_data_frame.has_raw_tracker_data())
        {
            const auto &raw_tracker_data = psds4_data_frame.raw_tracker_data();

            this->RawTrackerData.ValidTrackerLocations =
                std::min(raw_tracker_data.valid_tracker_count(), PSMOVESERVICE_MAX_TRACKER_COUNT);

            for (int listIndex = 0; listIndex < this->RawTrackerData.ValidTrackerLocations; ++listIndex)
            {
                const PSMoveProtocol::Pixel &locationOnTracker = raw_tracker_data.screen_locations(listIndex);
                const PSMoveProtocol::Position &positionOnTrackerCm = raw_tracker_data.relative_positions_cm(listIndex);
                const PSMoveProtocol::Orientation &orientationOnTracker = raw_tracker_data.relative_orientations(listIndex);

                this->RawTrackerData.TrackerIDs[listIndex] = raw_tracker_data.tracker_ids(listIndex);
                this->RawTrackerData.ScreenLocations[listIndex] =
                    PSMoveScreenLocation::create(locationOnTracker.x(), locationOnTracker.y());
                this->RawTrackerData.RelativePositionsCm[listIndex] =
                    PSMovePosition::create(
                        positionOnTrackerCm.x(), positionOnTrackerCm.y(), positionOnTrackerCm.z());
                this->RawTrackerData.RelativeOrientations[listIndex] =
                    PSMoveQuaternion::create(
                        orientationOnTracker.w(), orientationOnTracker.x(), orientationOnTracker.y(), orientationOnTracker.z());

                if (raw_tracker_data.projected_blobs_size() > 0)
                {
                    const PSMoveProtocol::Polygon &protocolPolygon = raw_tracker_data.projected_blobs(listIndex);
                    PSMoveTrackingProjection &projection = this->RawTrackerData.TrackingProjections[listIndex];

                    assert (protocolPolygon.vertices_size() == 7);
                    projection.shape_type = PSMoveTrackingProjection::LightBar;

                    for (int vert_index = 0; vert_index < 3; ++vert_index)
                    {
                        const PSMoveProtocol::Pixel &pixel = protocolPolygon.vertices(vert_index);

                        projection.shape.lightbar.triangle[vert_index].x = pixel.x();
                        projection.shape.lightbar.triangle[vert_index].y = pixel.y();
                    }
                    for (int vert_index = 0; vert_index < 4; ++vert_index)
                    {
                        const PSMoveProtocol::Pixel &pixel = protocolPolygon.vertices(vert_index+3);

                        projection.shape.lightbar.quad[vert_index].x = pixel.x();
                        projection.shape.lightbar.quad[vert_index].y = pixel.y();
                    }
                }
                else
                {
                    PSMoveTrackingProjection &projection = this->RawTrackerData.TrackingProjections[listIndex];

                    projection.shape_type = PSMoveTrackingProjection::eShapeType::INVALID_PROJECTION;
                }
            }

			if (raw_tracker_data.has_multicam_position_cm())
			{
				const PSMoveProtocol::Position &multicam_position_cm = raw_tracker_data.multicam_position_cm();

				this->RawTrackerData.MulticamPositionCm.x = multicam_position_cm.x();
				this->RawTrackerData.MulticamPositionCm.y = multicam_position_cm.y();
				this->RawTrackerData.MulticamPositionCm.z = multicam_position_cm.z();
				this->RawTrackerData.bMulticamPositionValid = true;
			}

			if (raw_tracker_data.has_multicam_orientation())
			{
				const PSMoveProtocol::Orientation &multicam_orientation = raw_tracker_data.multicam_orientation();

				this->RawTrackerData.MulticamOrientation.w = multicam_orientation.w();
				this->RawTrackerData.MulticamOrientation.x = multicam_orientation.x();
				this->RawTrackerData.MulticamOrientation.y = multicam_orientation.y();
				this->RawTrackerData.MulticamOrientation.z = multicam_orientation.z();
				this->RawTrackerData.bMulticamOrientationValid = true;
			}
        }
        else
        {
            this->RawTrackerData.Clear();
        }

        if (psds4_data_frame.has_physics_data())
        {
            const auto &raw_physics_data = psds4_data_frame.physics_data();

            this->PhysicsData.VelocityCmPerSec.i = raw_physics_data.velocity_cm_per_sec().i();
            this->PhysicsData.VelocityCmPerSec.j = raw_physics_data.velocity_cm_per_sec().j();
            this->PhysicsData.VelocityCmPerSec.k = raw_physics_data.velocity_cm_per_sec().k();

            this->PhysicsData.AccelerationCmPerSecSqr.i = raw_physics_data.acceleration_cm_per_sec_sqr().i();
            this->PhysicsData.AccelerationCmPerSecSqr.j = raw_physics_data.acceleration_cm_per_sec_sqr().j();
            this->PhysicsData.AccelerationCmPerSecSqr.k = raw_physics_data.acceleration_cm_per_sec_sqr().k();

            this->PhysicsData.AngularVelocityRadPerSec.i = raw_physics_data.angular_velocity_rad_per_sec().i();
            this->PhysicsData.AngularVelocityRadPerSec.j = raw_physics_data.angular_velocity_rad_per_sec().j();
            this->PhysicsData.AngularVelocityRadPerSec.k = raw_physics_data.angular_velocity_rad_per_sec().k();

            this->PhysicsData.AngularAccelerationRadPerSecSqr.i = raw_physics_data.angular_acceleration_rad_per_sec_sqr().i();
            this->PhysicsData.AngularAccelerationRadPerSecSqr.j = raw_physics_data.angular_acceleration_rad_per_sec_sqr().j();
            this->PhysicsData.AngularAccelerationRadPerSecSqr.k = raw_physics_data.angular_acceleration_rad_per_sec_sqr().k();
        }
        else
        {
            this->PhysicsData.Clear();
        }

        unsigned int button_bitmask = data_frame->button_down_bitmask();

        update_button_state(DPadUpButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_UP);
        update_button_state(DPadDownButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_DOWN);
        update_button_state(DPadLeftButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_LEFT);
        update_button_state(DPadRightButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_RIGHT);

        update_button_state(TriangleButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_TRIANGLE);
        update_button_state(CircleButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_CIRCLE);
        update_button_state(CrossButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_CROSS);
        update_button_state(SquareButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_SQUARE);

        update_button_state(L1Button, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_L1);
        update_button_state(R1Button, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_R1);
        update_button_state(L2Button, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_L2);
        update_button_state(R2Button, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_R2);
        update_button_state(L3Button, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_L3);
        update_button_state(R3Button, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_R3);

        update_button_state(ShareButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_SHARE);
        update_button_state(OptionsButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_OPTIONS);

        update_button_state(PSButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_PS);
        update_button_state(TrackPadButton, button_bitmask, PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket_ButtonType_TRACKPAD);

        this->LeftAnalogX = psds4_data_frame.left_thumbstick_x();
        this->LeftAnalogY = psds4_data_frame.left_thumbstick_y();
        this->RightAnalogX = psds4_data_frame.right_thumbstick_x();
        this->RightAnalogY = psds4_data_frame.right_thumbstick_y();
        this->LeftTriggerValue = psds4_data_frame.left_trigger_value();
        this->RightTriggerValue = psds4_data_frame.right_trigger_value();

        this->bValid = true;

		// Fire off the recenter action if the recenter button has been held long enough
		ProcessRecenterAction(parentView);
    }
    else
    {
        Clear();
    }
}

void ClientPSDualShock4View::ProcessRecenterAction(class ClientControllerView *parentView)
{
	if (bPoseResetButtonEnabled)
	{
		long long now =
			std::chrono::duration_cast< std::chrono::milliseconds >(
				std::chrono::system_clock::now().time_since_epoch()).count();

		PSMoveButtonState resetPoseButtonState = GetButtonOptions();

		switch (resetPoseButtonState)
		{
		case PSMoveButtonState::PSMoveButton_PRESSED:
			{
				ResetPoseButtonPressTime = now;
			} break;
		case PSMoveButtonState::PSMoveButton_DOWN:
			{
				if (!bResetPoseRequestSent)
				{
					const long long k_hold_duration_milli = 250;
					long long pressDurationMilli = now - ResetPoseButtonPressTime;

					if (pressDurationMilli >= k_hold_duration_milli)
					{
						ClientPSMoveAPI::eat_response(
							ClientPSMoveAPI::reset_orientation(parentView, PSMoveQuaternion::identity()));
						bResetPoseRequestSent = true;
					}
				}
			} break;
		case PSMoveButtonState::PSMoveButton_RELEASED:
			{
				bResetPoseRequestSent = false;
			} break;
		}
	}
}

void ClientPSDualShock4View::Publish(
    PSMoveProtocol::DeviceInputDataFrame_ControllerDataPacket *data_frame)
{
    auto *psmove_state = data_frame->mutable_psdualshock4_state();

    psmove_state->set_led_r(this->LED_r);
    psmove_state->set_led_g(this->LED_g);
    psmove_state->set_led_b(this->LED_b);
    psmove_state->set_big_rumble_value(this->BigRumble);
    psmove_state->set_small_rumble_value(this->SmallRumble);

    bHasUnpublishedState = false;
}

void ClientPSDualShock4View::SetBigRumble(float rumbleFraction)
{
    unsigned char newRumble = static_cast<unsigned char>(clampf01(rumbleFraction)*255.f);

    if (newRumble != BigRumble)
    {
        BigRumble = newRumble;

        bHasUnpublishedState = true;
    }
}

void ClientPSDualShock4View::SetSmallRumble(float rumbleFraction)
{
    unsigned char newRumble = static_cast<unsigned char>(clampf01(rumbleFraction)*255.f);

    if (newRumble != SmallRumble)
    {
        SmallRumble = newRumble;

        bHasUnpublishedState = true;
    }
}

void ClientPSDualShock4View::SetLEDOverride(unsigned char r, unsigned char g, unsigned char b)
{
    if (r != LED_r || g != LED_g || b != LED_b)
    {
        LED_r = r;
        LED_g = g;
        LED_b = b;

        bHasUnpublishedState = true;
    }
}

const PSMovePhysicsData &ClientPSDualShock4View::GetPhysicsData() const
{
    return IsValid() ? PhysicsData : k_empty_physics_data;
}

const PSDualShock4RawSensorData &ClientPSDualShock4View::GetRawSensorData() const
{
    return IsValid() ? RawSensorData : k_empty_ds4_raw_sensor_data;
}

const PSDualShock4CalibratedSensorData &ClientPSDualShock4View::GetCalibratedSensorData() const
{
    return IsValid() ? CalibratedSensorData : k_empty_ds4_calibrated_sensor_data;
}

bool ClientPSDualShock4View::GetIsGyroStable() const
{
    const float k_gyro_noise= 10.f*k_degrees_to_radians; // noise threshold in rad/sec
    const float worst_rotation_rate = fabsf(CalibratedSensorData.Gyroscope.maxValue());
    const bool isOk = worst_rotation_rate < k_gyro_noise;

    return isOk;
}

bool ClientPSDualShock4View::GetIsStable() const
{
	return GetIsGyroStable();
}

const PSMoveRawTrackerData &ClientPSDualShock4View::GetRawTrackerData() const
{
    return RawTrackerData;
}

//-- ClientControllerView -----
ClientControllerView::ClientControllerView(int PSMoveID)
{
    Clear();
    this->ControllerID= PSMoveID;
}

void ClientControllerView::Clear()
{
    ControllerID = -1;
    OutputSequenceNum= -1;
    InputSequenceNum = -1;
    ListenerCount= 0;

    IsConnected= false;

    ControllerViewType= None;
    memset(&ViewState, 0, sizeof(ViewState));

    data_frame_last_received_time= 
        std::chrono::duration_cast< std::chrono::milliseconds >(
                std::chrono::system_clock::now().time_since_epoch()).count();
    data_frame_average_fps= 0.f;
}

void ClientControllerView::ApplyControllerDataFrame(
    const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket *data_frame)
{
    assert(data_frame->controller_id() == ControllerID);

    // Compute the data frame receive window statistics if we have received enough samples
    {
        long long now = 
            std::chrono::duration_cast< std::chrono::milliseconds >(
                std::chrono::system_clock::now().time_since_epoch()).count();
        long long diff= now - data_frame_last_received_time;

        if (diff > 0)
        {
            float seconds= static_cast<float>(diff) / 1000.f;
            float fps= 1.f / seconds;

            data_frame_average_fps= (0.9f)*data_frame_average_fps + (0.1f)*fps;
        }

        data_frame_last_received_time= now;
    }

    if (data_frame->sequence_num() > this->OutputSequenceNum)
    {
        this->OutputSequenceNum= data_frame->sequence_num();
        this->IsConnected= data_frame->isconnected();

        switch(data_frame->controller_type())
        {
        case PSMoveProtocol::PSMOVE:
            {
                this->ControllerViewType= PSMove;
                this->ViewState.PSMoveView.ApplyControllerDataFrame(this, data_frame);
            } break;

        case PSMoveProtocol::PSNAVI:
            {
                this->ControllerViewType= PSNavi;
                this->ViewState.PSNaviView.ApplyControllerDataFrame(data_frame);
            } break;

            case PSMoveProtocol::PSDUALSHOCK4:
            {
                this->ControllerViewType = PSDualShock4;
                this->ViewState.PSDualShock4View.ApplyControllerDataFrame(this, data_frame);
            } break;

            default:
                assert(0 && "Unhandled controller type");
        }
    }
}

bool ClientControllerView::GetHasUnpublishedState() const
{
    bool bHasUnpublishedState = false;

    switch (ControllerViewType)
    {
    case eControllerType::PSMove:
        bHasUnpublishedState = ViewState.PSMoveView.GetHasUnpublishedState();
        break;
    case eControllerType::PSNavi:
        bHasUnpublishedState = ViewState.PSNaviView.GetHasUnpublishedState();
        break;
    case eControllerType::PSDualShock4:
        bHasUnpublishedState = ViewState.PSDualShock4View.GetHasUnpublishedState();
        break;
    }

    return bHasUnpublishedState;
}

void ClientControllerView::Publish()
{
    if (GetHasUnpublishedState())
    {
        DeviceInputDataFramePtr data_frame(new PSMoveProtocol::DeviceInputDataFrame);
        data_frame->set_device_category(PSMoveProtocol::DeviceInputDataFrame_DeviceCategory_CONTROLLER);

        auto *controller_data_packet= data_frame->mutable_controller_data_packet();
        controller_data_packet->set_controller_id(ControllerID);
        controller_data_packet->set_sequence_num(++InputSequenceNum);

        switch (ControllerViewType)
        {
        case eControllerType::PSMove:
            controller_data_packet->set_controller_type(PSMoveProtocol::PSMOVE);
            ViewState.PSMoveView.Publish(controller_data_packet);
            break;
        case eControllerType::PSNavi:
            controller_data_packet->set_controller_type(PSMoveProtocol::PSNAVI);
            ViewState.PSNaviView.Publish(controller_data_packet);
            break;
        case eControllerType::PSDualShock4:
            controller_data_packet->set_controller_type(PSMoveProtocol::PSDUALSHOCK4);
            ViewState.PSDualShock4View.Publish(controller_data_packet);
            break;
        default:
            assert(0 && "Unhandled controller type");
        }

        // Send the controller data frame over the network
        ClientNetworkManager::get_instance()->send_device_data_frame(data_frame);
    }
}

const PSMovePose &ClientControllerView::GetPose() const
{
    switch (ControllerViewType)
    {
    case eControllerType::PSMove:
        return GetPSMoveView().GetPose();
    case eControllerType::PSNavi:
        // No Pose!
        return *k_psmove_pose_identity;
    case eControllerType::PSDualShock4:
        return GetPSDualShock4View().GetPose();
    default:
        assert(0 && "invalid controller type");
        return *k_psmove_pose_identity;
    }
}

const PSMovePosition &ClientControllerView::GetPosition() const
{
    switch (ControllerViewType)
    {
    case eControllerType::PSMove:
        return GetPSMoveView().GetPosition();
    case eControllerType::PSNavi:
        // No Position!
        return *k_psmove_position_origin;
    case eControllerType::PSDualShock4:
        return GetPSDualShock4View().GetPosition();
    default:
        assert(0 && "invalid controller type");
        return *k_psmove_position_origin;
    }
}

const PSMoveQuaternion &ClientControllerView::GetOrientation() const
{
    switch (ControllerViewType)
    {
    case eControllerType::PSMove:
        return GetPSMoveView().GetOrientation();
    case eControllerType::PSNavi:
        // No orientation!
        return *k_psmove_quaternion_identity;
    case eControllerType::PSDualShock4:
        return GetPSDualShock4View().GetOrientation();
    default:
        assert(0 && "invalid controller type");
        return *k_psmove_quaternion_identity;
    }
}

const PSMovePhysicsData &ClientControllerView::GetPhysicsData() const
{
    switch (ControllerViewType)
    {
    case eControllerType::PSMove:
        return GetPSMoveView().GetPhysicsData();
    case eControllerType::PSNavi:
        // No Physics!
        return k_empty_physics_data;
    case eControllerType::PSDualShock4:
        return GetPSDualShock4View().GetPhysicsData();
    default:
        assert(0 && "invalid controller type");
        return k_empty_physics_data;
    }
}

const PSMoveRawTrackerData &ClientControllerView::GetRawTrackerData() const
{
    switch (ControllerViewType)
    {
    case eControllerType::PSMove:
        return GetPSMoveView().GetRawTrackerData();
    case eControllerType::PSNavi:
        // No Physics!
        return k_empty_raw_tracker_data;
    case eControllerType::PSDualShock4:
        return GetPSDualShock4View().GetRawTrackerData();
    default:
        assert(0 && "invalid controller type");
        return k_empty_raw_tracker_data;
    }
}

bool ClientControllerView::GetIsCurrentlyTracking() const
{
    switch (ControllerViewType)
    {
    case eControllerType::PSMove:
        return GetPSMoveView().GetIsCurrentlyTracking();
    case eControllerType::PSNavi:
        // Never Tracking!
        return false;
    case eControllerType::PSDualShock4:
        return GetPSDualShock4View().GetIsCurrentlyTracking();
    default:
        assert(0 && "invalid controller type");
        return false;
    }
}

bool ClientControllerView::GetIsPoseValid() const
{
    switch (ControllerViewType)
    {
    case eControllerType::PSMove:
        return GetPSMoveView().GetIsOrientationValid() && GetPSMoveView().GetIsPositionValid() && GetPSMoveView().GetIsCurrentlyTracking();
    case eControllerType::PSNavi:
        // Never Tracking!
        return false;
    case eControllerType::PSDualShock4:
        return GetPSDualShock4View().GetIsOrientationValid() && GetPSDualShock4View().GetIsPositionValid();
    default:
        assert(0 && "invalid controller type");
        return false;
    }
}

bool ClientControllerView::GetIsGyroStable() const
{
	switch (ControllerViewType)
	{
	case eControllerType::PSMove:
		return GetPSMoveView().GetIsGyroStable();
	case eControllerType::PSNavi:
		// Always stable! (no physics)
		return true;
	case eControllerType::PSDualShock4:
		return GetPSDualShock4View().GetIsGyroStable();
	default:
		assert(0 && "invalid controller type");
		return true;
	}
}

bool ClientControllerView::GetIsStable() const
{
    switch (ControllerViewType)
    {
    case eControllerType::PSMove:
        return GetPSMoveView().GetIsStable();
    case eControllerType::PSNavi:
        // Always stable! (no physics)
        return true;
    case eControllerType::PSDualShock4:
        return GetPSDualShock4View().GetIsStable();
    default:
        assert(0 && "invalid controller type");
        return true;
    }
}

void ClientControllerView::SetLEDOverride(unsigned char r, unsigned char g, unsigned char b)
{
    switch (ControllerViewType)
    {
    case eControllerType::PSMove:
        GetPSMoveViewMutable().SetLEDOverride(r, g, b);
        break;
    case eControllerType::PSNavi:
        break;
    case eControllerType::PSDualShock4:
        GetPSDualShock4ViewMutable().SetLEDOverride(r, g, b);
        break;
    }
}

//-- helper functions -----
static void update_button_state(
    PSMoveButtonState &button,
    unsigned int button_bitmask,
    unsigned int button_bit)
{
    const bool is_down= (button_bitmask & (1 << button_bit)) > 0;

    switch (button)
    {
    case PSMoveButton_UP:
        button= is_down ? PSMoveButton_PRESSED : PSMoveButton_UP;
        break;
    case PSMoveButton_PRESSED:
        button= is_down ? PSMoveButton_DOWN : PSMoveButton_RELEASED;
        break;
    case PSMoveButton_DOWN:
        button= is_down ? PSMoveButton_DOWN : PSMoveButton_RELEASED;
        break;
    case PSMoveButton_RELEASED:
        button= is_down ? PSMoveButton_PRESSED : PSMoveButton_UP;
        break;
    };
}
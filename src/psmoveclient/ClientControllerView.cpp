//-- includes -----
#include "ClientControllerView.h"
#include "PSMoveProtocol.pb.h"
#include "MathUtility.h"
#include <chrono>
#include <algorithm>
#include <assert.h>

//-- pre-declarations -----

//-- constants -----
const PSMovePhysicsData k_empty_physics_data = { { 0.f, 0.f, 0.f }, { 0.f, 0.f, 0.f }, { 0.f, 0.f, 0.f }, { 0.f, 0.f, 0.f } };
const PSMoveRawSensorData k_empty_sensor_data = { { 0, 0, 0 }, { 0.f, 0.f, 0.f }, { 0.f, 0.f, 0.f } };
const PSMoveFloatVector3 k_identity_gravity_calibration_direction= {0.f, 1.f, 0.f};

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
}

const PSMovePhysicsData &ClientPSMoveView::GetPhysicsData() const
{
    return IsValid() ? PhysicsData : k_empty_physics_data;
}

const PSMoveRawSensorData &ClientPSMoveView::GetRawSensorData() const
{
    return IsValid() ? RawSensorData : k_empty_sensor_data;
}

const PSMoveFloatVector3 &ClientPSMoveView::GetIdentityGravityCalibrationDirection() const
{
    return k_identity_gravity_calibration_direction;
}

bool ClientPSMoveView::GetIsStableAndAlignedWithGravity() const
{
    const float k_cosine_10_degrees = 0.984808f;

    // Get the direction the gravity vector should be pointing 
    // while the controller is in cradle pose.
    PSMoveFloatVector3 acceleration_direction = RawSensorData.Accelerometer;
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
    const PSMoveProtocol::DeviceDataFrame_ControllerDataPacket *data_frame)
{
    if (data_frame->isconnected())
    {
        const auto &psmove_data_frame = data_frame->psmove_state();

        this->bHasValidHardwareCalibration= psmove_data_frame.validhardwarecalibration();
        this->bIsTrackingEnabled= psmove_data_frame.istrackingenabled();
        this->bIsCurrentlyTracking= psmove_data_frame.iscurrentlytracking();

        this->Pose.Orientation.w= psmove_data_frame.orientation().w();
        this->Pose.Orientation.x= psmove_data_frame.orientation().x();
        this->Pose.Orientation.y= psmove_data_frame.orientation().y();
        this->Pose.Orientation.z= psmove_data_frame.orientation().z();

        this->Pose.Position.x= psmove_data_frame.position().x();
        this->Pose.Position.y= psmove_data_frame.position().y();
        this->Pose.Position.z= psmove_data_frame.position().z();

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

        if (psmove_data_frame.has_raw_tracker_data())
        {
            const auto &raw_tracker_data = psmove_data_frame.raw_tracker_data();

            this->RawTrackerData.ValidTrackerLocations = 
                std::min(raw_tracker_data.valid_tracker_count(), PSMOVESERVICE_MAX_TRACKER_COUNT);

            for (int listIndex = 0; listIndex < this->RawTrackerData.ValidTrackerLocations; ++listIndex)
            {
                const PSMoveProtocol::Pixel &locationOnTracker = raw_tracker_data.screen_locations(listIndex);
                const PSMoveProtocol::Position &positionOnTracker = raw_tracker_data.relative_positions(listIndex);

                this->RawTrackerData.TrackerIDs[listIndex]= raw_tracker_data.tracker_ids(listIndex);
                this->RawTrackerData.ScreenLocations[listIndex] =
                    PSMoveScreenLocation::create(locationOnTracker.x(), locationOnTracker.y());
                this->RawTrackerData.RelativePositions[listIndex] =
                    PSMovePosition::create(
                        positionOnTracker.x(), positionOnTracker.y(), positionOnTracker.z());

                if (raw_tracker_data.projected_spheres_size() > 0)
                {
                    const PSMoveProtocol::Ellipse &protocolEllipse = raw_tracker_data.projected_spheres(0);
                    PSMoveTrackingProjection &projection= this->RawTrackerData.TrackingProjections[listIndex];

                    projection.shape.ellipse.center.x = protocolEllipse.center().x();
                    projection.shape.ellipse.center.y = protocolEllipse.center().y();
                    projection.shape.ellipse.half_x_extent = protocolEllipse.half_x_extent();
                    projection.shape.ellipse.half_y_extent = protocolEllipse.half_y_extent();
                    projection.shape.ellipse.angle = protocolEllipse.angle();
                    projection.shape_type = PSMoveTrackingProjection::eShapeType::Ellipse;
                }
                else if (raw_tracker_data.projected_blobs_size() > 0)
                {
                    const PSMoveProtocol::Polygon &protocolPolygon = raw_tracker_data.projected_blobs(0);
                    PSMoveTrackingProjection &projection = this->RawTrackerData.TrackingProjections[listIndex];

                    assert(protocolPolygon.vertices_size() == 4);
                    for (int vert_index = 0; vert_index < 4; ++vert_index)
                    {
                        const PSMoveProtocol::Pixel &pixel= protocolPolygon.vertices(vert_index);

                        projection.shape.quad.corners[vert_index].x = pixel.x();
                        projection.shape.quad.corners[vert_index].y = pixel.y();
                    }
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

        if (psmove_data_frame.has_physics_data())
        {
            const auto &raw_physics_data = psmove_data_frame.physics_data();

            this->PhysicsData.Velocity.i = raw_physics_data.velocity().i();
            this->PhysicsData.Velocity.j = raw_physics_data.velocity().j();
            this->PhysicsData.Velocity.k = raw_physics_data.velocity().k();

            this->PhysicsData.Acceleration.i = raw_physics_data.acceleration().i();
            this->PhysicsData.Acceleration.j = raw_physics_data.acceleration().j();
            this->PhysicsData.Acceleration.k = raw_physics_data.acceleration().k();

            this->PhysicsData.AngularVelocity.i = raw_physics_data.velocity().i();
            this->PhysicsData.AngularVelocity.j = raw_physics_data.velocity().j();
            this->PhysicsData.AngularVelocity.k = raw_physics_data.velocity().k();

            this->PhysicsData.AngularAcceleration.i = raw_physics_data.angular_acceleration().i();
            this->PhysicsData.AngularAcceleration.j = raw_physics_data.angular_acceleration().j();
            this->PhysicsData.AngularAcceleration.k = raw_physics_data.angular_acceleration().k();
        }
        else
        {
            this->PhysicsData.Clear();
        }

        unsigned int button_bitmask= data_frame->button_down_bitmask();
        update_button_state(TriangleButton, button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_ButtonType_TRIANGLE);
        update_button_state(CircleButton, button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_ButtonType_CIRCLE);
        update_button_state(CrossButton, button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_ButtonType_CROSS);
        update_button_state(SquareButton, button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_ButtonType_SQUARE);
        update_button_state(SelectButton, button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_ButtonType_SELECT);
        update_button_state(StartButton, button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_ButtonType_START);
        update_button_state(PSButton, button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_ButtonType_PS);
        update_button_state(MoveButton, button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_ButtonType_MOVE);
        update_button_state(TriggerButton, button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_ButtonType_TRIGGER);

        //###bwalker $TODO make sure this is in the range [0, 255]
        this->TriggerValue= static_cast<unsigned char>(psmove_data_frame.trigger_value());

        this->bValid= true;
    }
    else
    {
        Clear();
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

void ClientPSNaviView::ApplyControllerDataFrame(const PSMoveProtocol::DeviceDataFrame_ControllerDataPacket *data_frame)
{
    if (data_frame->isconnected())
    {
        const PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_PSNaviState &psnavi_data_frame= data_frame->psnavi_state();

        unsigned int button_bitmask= data_frame->button_down_bitmask();
        update_button_state(L1Button, button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_ButtonType_L1);
        update_button_state(L2Button, button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_ButtonType_L2);
        update_button_state(L3Button, button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_ButtonType_L3);
        update_button_state(CircleButton, button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_ButtonType_CIRCLE);
        update_button_state(CrossButton, button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_ButtonType_CROSS);
        update_button_state(PSButton, button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_ButtonType_PS);
        update_button_state(TriggerButton, button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_ButtonType_TRIGGER);
        update_button_state(DPadUpButton, button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_ButtonType_UP);
        update_button_state(DPadRightButton, button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_ButtonType_RIGHT);
        update_button_state(DPadDownButton, button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_ButtonType_DOWN);
        update_button_state(DPadLeftButton, button_bitmask, PSMoveProtocol::DeviceDataFrame_ControllerDataPacket_ButtonType_LEFT);

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

//-- ClientControllerView -----
ClientControllerView::ClientControllerView(int PSMoveID)
{
    Clear();
    this->ControllerID= PSMoveID;
}

void ClientControllerView::Clear()
{
    ControllerID = -1;
    SequenceNum= -1;
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
    const PSMoveProtocol::DeviceDataFrame_ControllerDataPacket *data_frame)
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

    if (data_frame->sequence_num() > this->SequenceNum)
    {
        this->SequenceNum= data_frame->sequence_num();
        this->IsConnected= data_frame->isconnected();

        switch(data_frame->controller_type())
        {
            case PSMoveProtocol::PSMOVE:
            {
                this->ControllerViewType= PSMove;
                this->ViewState.PSMoveView.ApplyControllerDataFrame(data_frame);
            } break;

            case PSMoveProtocol::PSNAVI:
            {
                this->ControllerViewType= PSNavi;
                this->ViewState.PSNaviView.ApplyControllerDataFrame(data_frame);
            } break;

            default:
                assert(0 && "Unhandled controller type");
        }
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
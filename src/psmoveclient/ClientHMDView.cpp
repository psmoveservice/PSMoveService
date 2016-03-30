//-- includes -----
#include "ClientHMDView.h"
#include "PSMoveProtocol.pb.h"
#include <chrono>
#include <assert.h>

//-- pre-declarations -----

//-- constants -----
const OculusDK2RawSensorData k_empty_sensor_data = { { 0.f, 0.f, 0.f }, { 0.f, 0.f, 0.f }, { 0.f, 0.f, 0.f }, 0.f, 0.f };

//-- prototypes ----

//-- implementation -----

//-- ClientOculusDK2View -----
void ClientOculusDK2View::Clear()
{
    bValid = false;

    RawSensorData.Clear();

    AngularVelocity= *k_psmove_float_vector3_zero;
    LinearVelocity = *k_psmove_float_vector3_zero;
    AngularAcceleration = *k_psmove_float_vector3_zero;
    LinearAcceleration = *k_psmove_float_vector3_zero;
    StateTime= 0.f;
}

const OculusDK2RawSensorData &ClientOculusDK2View::GetRawSensorData() const
{
    return IsValid() ? RawSensorData : k_empty_sensor_data;
}

void ClientOculusDK2View::ApplyHMDDataFrame(
    const PSMoveProtocol::DeviceDataFrame_HMDDataPacket *data_frame)
{
    if (data_frame->isconnected())
    {
        const PSMoveProtocol::DeviceDataFrame_HMDDataPacket_OculusDK2State &oculusdk2_data_frame = data_frame->oculus_dk2_state();

        this->AngularVelocity.i = oculusdk2_data_frame.angular_velocity().i();
        this->AngularVelocity.j = oculusdk2_data_frame.angular_velocity().j();
        this->AngularVelocity.k = oculusdk2_data_frame.angular_velocity().k();

        this->AngularAcceleration.i = oculusdk2_data_frame.angular_acceleration().i();
        this->AngularAcceleration.j = oculusdk2_data_frame.angular_acceleration().j();
        this->AngularAcceleration.k = oculusdk2_data_frame.angular_acceleration().k();

        this->LinearVelocity.i = oculusdk2_data_frame.linear_velocity().i();
        this->LinearVelocity.j = oculusdk2_data_frame.linear_velocity().j();
        this->LinearVelocity.k = oculusdk2_data_frame.linear_velocity().k();

        this->LinearAcceleration.i = oculusdk2_data_frame.linear_acceleration().i();
        this->LinearAcceleration.j = oculusdk2_data_frame.linear_acceleration().j();
        this->LinearAcceleration.k = oculusdk2_data_frame.linear_acceleration().k();

        this->StateTime = oculusdk2_data_frame.state_time();

        if (oculusdk2_data_frame.has_raw_sensor_data())
        {
            const PSMoveProtocol::DeviceDataFrame_HMDDataPacket_OculusDK2State_RawSensorData &raw_sensor_data =
                oculusdk2_data_frame.raw_sensor_data();

            this->RawSensorData.Magnetometer.i = raw_sensor_data.magnetometer().i();
            this->RawSensorData.Magnetometer.j = raw_sensor_data.magnetometer().j();
            this->RawSensorData.Magnetometer.k = raw_sensor_data.magnetometer().k();

            this->RawSensorData.Accelerometer.i = raw_sensor_data.accelerometer().i();
            this->RawSensorData.Accelerometer.j = raw_sensor_data.accelerometer().j();
            this->RawSensorData.Accelerometer.k = raw_sensor_data.accelerometer().k();

            this->RawSensorData.Gyroscope.i = raw_sensor_data.gyroscope().i();
            this->RawSensorData.Gyroscope.j = raw_sensor_data.gyroscope().j();
            this->RawSensorData.Gyroscope.k = raw_sensor_data.gyroscope().k();

            this->RawSensorData.Temparature = raw_sensor_data.temparature();
            this->RawSensorData.IMUSampleTime = raw_sensor_data.imu_sample_time();
        }
        else
        {
            this->RawSensorData.Clear();
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
    const PSMoveProtocol::DeviceDataFrame_HMDDataPacket *data_frame)
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

        this->Pose.Orientation.w = data_frame->orientation().w();
        this->Pose.Orientation.x = data_frame->orientation().x();
        this->Pose.Orientation.y = data_frame->orientation().y();
        this->Pose.Orientation.z = data_frame->orientation().z();

        this->Pose.Position.x = data_frame->position().x();
        this->Pose.Position.y = data_frame->position().y();
        this->Pose.Position.z = data_frame->position().z();

        switch (data_frame->hmd_type())
        {
        case PSMoveProtocol::OculusDK2:
        {
            this->HMDViewType = OculusDK2;
            this->ViewState.OculusDK2View.ApplyHMDDataFrame(data_frame);
        } break;

        default:
            assert(0 && "Unhandled HMD type");
        }
    }
}
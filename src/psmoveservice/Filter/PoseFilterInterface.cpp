// -- includes --
#include "PoseFilterInterface.h"

// -- constants --
// Calibration Pose transform
const Eigen::Matrix3f g_eigen_identity_pose_upright = Eigen::Matrix3f::Identity();
const Eigen::Matrix3f *k_eigen_identity_pose_upright = &g_eigen_identity_pose_upright;

const Eigen::Matrix3f g_eigen_identity_pose_laying_flat((Eigen::Matrix3f() << 1,0,0, 0,0,-1, 0,1,0).finished());
const Eigen::Matrix3f *k_eigen_identity_pose_laying_flat = &g_eigen_identity_pose_laying_flat;

//Sensor Transforms
const Eigen::Matrix3f g_eigen_sensor_transform_identity = Eigen::Matrix3f::Identity();
const Eigen::Matrix3f *k_eigen_sensor_transform_identity = &g_eigen_sensor_transform_identity;

const Eigen::Matrix3f g_eigen_sensor_transform_opengl((Eigen::Matrix3f() << 1,0,0, 0,0,1, 0,-1,0).finished());
const Eigen::Matrix3f *k_eigen_sensor_transform_opengl= &g_eigen_sensor_transform_opengl;

// -- public interface -----
//-- Orientation Filter Space -----
PoseFilterSpace::PoseFilterSpace()
    : m_IdentityGravity(Eigen::Vector3f(0.f, 1.f, 0.f))
    , m_IdentityMagnetometer(Eigen::Vector3f(0.f, -1.f, 0.f))
    , m_CalibrationTransform(Eigen::Matrix3f::Identity())
    , m_SensorTransform(Eigen::Matrix3f::Identity())
{
}

Eigen::Vector3f PoseFilterSpace::getGravityCalibrationDirection() const
{
	// First apply the calibration data transform.
	// This allows us to pretend the "identity pose" was some other orientation the vertical during calibration
    const Eigen::Vector3f calibrationSpaceVector= m_CalibrationTransform * m_IdentityGravity;

	// Next apply the sensor data transform.
	// This allows us to pretend the sensors are in some other coordinate system (like OpenGL where +Y is up)
    const Eigen::Vector3f filterSpaceVector= m_SensorTransform * calibrationSpaceVector;

    return filterSpaceVector;
}

Eigen::Vector3f PoseFilterSpace::getMagnetometerCalibrationDirection() const
{
	// First apply the calibration data transform.
	// This allows us to pretend the "identity pose" was some other orientation the vertical during calibration
    const Eigen::Vector3f calibrationSpaceVector= m_CalibrationTransform * m_IdentityMagnetometer;

	// Next apply the sensor data transform.
	// This allows us to pretend the sensors are in some other coordinate system (like OpenGL where +Y is up)
    const Eigen::Vector3f filterSpaceVector= m_SensorTransform * calibrationSpaceVector;

    return filterSpaceVector;
}

void PoseFilterSpace::createFilterPacket(
    const PoseSensorPacket &sensorPacket,
	const IPoseFilter *poseFilter,
    PoseFilterPacket &outFilterPacket) const
{
	poseFilter->getOrientation(), poseFilter->getPositionCm(),

	outFilterPacket.current_orientation= poseFilter->getOrientation();
	outFilterPacket.current_position_cm= poseFilter->getPositionCm();
	outFilterPacket.current_linear_velocity_cm_s = poseFilter->getVelocityCmPerSec();
	outFilterPacket.current_linear_acceleration_cm_s2 = poseFilter->getAccelerationCmPerSecSqr();

    outFilterPacket.optical_orientation = sensorPacket.optical_orientation;

	// Positional filtering is done is meters to improve numerical stability
    outFilterPacket.optical_position_cm = sensorPacket.optical_position_cm;
    outFilterPacket.tracking_projection_area_px_sqr= sensorPacket.tracking_projection_area_px_sqr;

    outFilterPacket.imu_gyroscope_rad_per_sec= m_SensorTransform * sensorPacket.imu_gyroscope_rad_per_sec;
    outFilterPacket.imu_accelerometer_g_units= m_SensorTransform * sensorPacket.imu_accelerometer_g_units;
    outFilterPacket.imu_magnetometer_unit= m_SensorTransform * sensorPacket.imu_magnetometer_unit;
        
	outFilterPacket.world_accelerometer=
		eigen_vector3f_clockwise_rotate(outFilterPacket.current_orientation, outFilterPacket.imu_accelerometer_g_units);
}
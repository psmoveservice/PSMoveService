#ifndef KALMAN_POSE_FILTER_H
#define KALMAN_POSE_FILTER_H

#include "PoseFilterInterface.h"

#include <kalman/UnscentedKalmanFilter.hpp>

/// Abstract Kalman Pose filter for controllers
class KalmanPoseFilter : public IPoseFilter
{
public:
	KalmanPoseFilter();

	bool init(const PoseFilterConstants &constant);

	// -- IStateFilter --
	bool getIsStateValid() const override;
	void resetState() override;
	void recenterState() override;

	// -- IPoseFilter ---
	Eigen::Quaternionf getOrientation(float time = 0.f) const override;
	Eigen::Vector3f getAngularVelocity() const override;
	Eigen::Vector3f getAngularAcceleration() const override;
	Eigen::Vector3f getPosition(float time = 0.f) const override;
	Eigen::Vector3f getVelocity() const override;
	Eigen::Vector3f getAcceleration() const override;

protected:
	PoseFilterConstants m_constants;
	class KalmanFilterImpl *m_filter;
};

/// Kalman Pose filter for Optical Pose + Angular Rate(Gyroscope) + Gravity(Accelerometer)
class KalmanPoseFilterDS4 : public KalmanPoseFilter
{
public:
	void update(const float delta_time, const PoseFilterPacket &packet) override;
};

/// Kalman Pose filter for Optical Position + Magnetometer + Angular Rate(Gyroscope) + Gravity(Accelerometer)
class PSMovePoseKalmanFilter : public KalmanPoseFilter
{
public:
	void update(const float delta_time, const PoseFilterPacket &packet) override;
};

#endif // DEVICE_INTERFACE_H
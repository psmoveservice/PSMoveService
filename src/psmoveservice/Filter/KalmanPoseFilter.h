#ifndef KALMAN_POSE_FILTER_H
#define KALMAN_POSE_FILTER_H

#include "PoseFilterInterface.h"

/// Abstract Kalman Pose filter for controllers
class KalmanPoseFilter : public IPoseFilter
{
public:
	KalmanPoseFilter();

	virtual bool init(const PoseFilterConstants &constant);
	virtual bool init(const PoseFilterConstants &constant, const Eigen::Vector3f &position, const Eigen::Quaternionf &orientation);

	// -- IStateFilter --
	bool getIsStateValid() const override;
	void resetState() override;
	void recenterOrientation(const Eigen::Quaternionf& q_pose) override;

	// -- IPoseFilter ---
    bool getIsPositionStateValid() const override;
    bool getIsOrientationStateValid() const override;
	Eigen::Quaternionf getOrientation(float time = 0.f) const override;
	Eigen::Vector3f getAngularVelocityRadPerSec() const override;
	Eigen::Vector3f getAngularAccelerationRadPerSecSqr() const override;
	Eigen::Vector3f getPositionCm(float time = 0.f) const override;
	Eigen::Vector3f getVelocityCmPerSec() const override;
	Eigen::Vector3f getAccelerationCmPerSecSqr() const override;

protected:
	PoseFilterConstants m_constants;
	class KalmanPoseFilterImpl *m_filter;
};

/// Kalman Pose filter for Optical Pose + Angular Rate(Gyroscope) + Gravity(Accelerometer)
class KalmanPoseFilterDS4 : public KalmanPoseFilter
{
public:
	bool init(const PoseFilterConstants &constant) override;
	bool init(const PoseFilterConstants &constant, const Eigen::Vector3f &position, const Eigen::Quaternionf &orientation) override;
	void update(const float delta_time, const PoseFilterPacket &packet) override;
};

/// Kalman Pose filter for Optical Position + Magnetometer + Angular Rate(Gyroscope) + Gravity(Accelerometer)
class KalmanPoseFilterPSMove : public KalmanPoseFilter
{
public:
	bool init(const PoseFilterConstants &constant) override;
	bool init(const PoseFilterConstants &constant, const Eigen::Vector3f &position, const Eigen::Quaternionf &orientation) override;
	void update(const float delta_time, const PoseFilterPacket &packet) override;
};

#endif // DEVICE_INTERFACE_H
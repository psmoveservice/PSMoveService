#ifndef KALMAN_POSE_FILTER_H
#define KALMAN_POSE_FILTER_H

#include "PoseFilterInterface.h"

/// Base Kalman Pose filter
class KalmanPoseFilter : public IPoseFilter
{
public:
	KalmanPoseFilter();
	virtual ~KalmanPoseFilter();

    virtual bool init(const PoseFilterConstants &constant);
	virtual bool init(const PoseFilterConstants &constant, 
              const Eigen::Vector3f &initial_position,
              const Eigen::Quaternionf &initial_orientation);

	// -- IStateFilter --
	bool getIsStateValid() const override;
    double getTimeInSeconds() const override;
	void resetState() override;
	void recenterOrientation(const Eigen::Quaternionf& q_pose) override;

	// -- IPoseFilter ---
    /// Not true until the filter has updated at least once
    bool getIsPositionStateValid() const override;

    /// Not true until the filter has updated at least once
    bool getIsOrientationStateValid() const override;

    /// Estimate the current orientation of the filter given a time offset into the future
    Eigen::Quaternionf getOrientation(float time = 0.f) const override;

    /// Get the current world space angular velocity of the filter state (rad/s)
    Eigen::Vector3f getAngularVelocityRadPerSec() const override;

    /// Get the current world space angular acceleration of the filter state (rad/s^2)
    Eigen::Vector3f getAngularAccelerationRadPerSecSqr() const override;

    /// Estimate the current position of the filter state given a time offset into the future (centimeters)
    Eigen::Vector3f getPositionCm(float time = 0.f) const override;

    /// Get the current velocity of the filter state (cm/s)
    Eigen::Vector3f getVelocityCmPerSec() const override;

    /// Get the current velocity of the filter state (cm/s^2)
    Eigen::Vector3f getAccelerationCmPerSecSqr() const override;

protected:
	PoseFilterConstants m_constants;
	class KalmanPoseFilterImpl *m_filter;
};

/// Kalman Pose filter for Optical Point Cloud
class KalmanPoseFilterPointCloud : public KalmanPoseFilter
{
public:
	bool init(const PoseFilterConstants &constant) override;
	bool init(const PoseFilterConstants &constant, 
              const Eigen::Vector3f &initial_position,
              const Eigen::Quaternionf &initial_orientation) override;
	void update(const float delta_time, const PoseFilterPacket &packet) override;
};

/// Kalman Pose filter for Optical Point Cloud + Angular Rate(Gyroscope) + Gravity(Accelerometer)
class KalmanPoseFilterMorpheus : public KalmanPoseFilter
{
public:
	bool init(const PoseFilterConstants &constant) override;
	bool init(const PoseFilterConstants &constant, 
              const Eigen::Vector3f &initial_position,
              const Eigen::Quaternionf &initial_orientation) override;
	void update(const float delta_time, const PoseFilterPacket &packet) override;
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

#endif // KALMAN_POSE_FILTER_H
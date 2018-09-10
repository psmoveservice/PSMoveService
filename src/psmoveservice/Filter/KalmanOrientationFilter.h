#ifndef KALMAN_ORIENTATION_FILTER_H
#define KALMAN_ORIENTATION_FILTER_H

#include "PoseFilterInterface.h"

/// Base Kalman Orientation filter
class KalmanOrientationFilter : public IOrientationFilter
{
public:
	KalmanOrientationFilter();
	virtual ~KalmanOrientationFilter();

	bool init(const OrientationFilterConstants &constant) override;
	bool init(const OrientationFilterConstants &constant, const Eigen::Quaternionf &initial_orientation) override;

	// -- IStateFilter --
	bool getIsStateValid() const override;
    double getTimeInSeconds() const override;
	void resetState() override;
	void recenterOrientation(const Eigen::Quaternionf& q_pose) override;

	// -- IOrientationFilter ---
	Eigen::Quaternionf getOrientation(float time = 0.f) const override;
	Eigen::Vector3f getAngularVelocityRadPerSec() const override;
	Eigen::Vector3f getAngularAccelerationRadPerSecSqr() const override;

protected:
	OrientationFilterConstants m_constants;
	class KalmanOrientationFilterImpl *m_filter;
};

/// Kalman Orientation filter for Optical + Angular Rate(Gyroscope) + Gravity(Accelerometer)
class KalmanOrientationFilterPSVR : public KalmanOrientationFilter
{
public:
	bool init(const OrientationFilterConstants &constant) override;
	bool init(const OrientationFilterConstants &constant, const Eigen::Quaternionf &orientation) override;
	void update(const float delta_time, const PoseFilterPacket &packet) override;
};

/// Kalman Orientation filter for Optical Yaw + Angular Rate(Gyroscope) + Gravity(Accelerometer)
class KalmanOrientationFilterDS4 : public KalmanOrientationFilter
{
public:
	bool init(const OrientationFilterConstants &constant) override;
	bool init(const OrientationFilterConstants &constant, const Eigen::Quaternionf &orientation) override;
	void update(const float delta_time, const PoseFilterPacket &packet) override;
};

/// Kalman Orientation filter for Magnetometer + Angular Rate(Gyroscope) + Gravity(Accelerometer)
class KalmanOrientationFilterPSMove : public KalmanOrientationFilter
{
public:
	bool init(const OrientationFilterConstants &constant) override;
	bool init(const OrientationFilterConstants &constant, const Eigen::Quaternionf &orientation) override;
	void update(const float delta_time, const PoseFilterPacket &packet) override;
};

#endif // KALMAN_ORIENTATION_FILTER_H
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
	virtual bool init(
		const OrientationFilterConstants &constants,
		const Eigen::Quaternionf &orientation);

	// -- IStateFilter --
	bool getIsStateValid() const override;
	void resetState() override;
	void recenterState() override;

	// -- IOrientationFilter ---
	Eigen::Quaternionf getOrientation(float time = 0.f) const override;
	Eigen::Vector3f getAngularVelocity() const override;
	Eigen::Vector3f getAngularAcceleration() const override;

protected:
	OrientationFilterConstants m_constants;
	class KalmanOrientationFilterImpl *m_filter;
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
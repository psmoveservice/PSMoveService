#ifndef KALMAN_POSITION_FILTER_H
#define KALMAN_POSITION_FILTER_H

#include "PoseFilterInterface.h"

/// Kalman Position filter for Optical Pose + Gravity(Accelerometer)
class KalmanPositionFilter : public IPositionFilter
{
public:
	KalmanPositionFilter();
	virtual ~KalmanPositionFilter();

	bool init(const PositionFilterConstants &constant) override;
	void update(const float delta_time, const PoseFilterPacket &packet) override;

	// -- IStateFilter --
	bool getIsStateValid() const override;
	void resetState() override;
	void recenterState() override;

	// -- IPoseFilter ---
	Eigen::Vector3f getPosition(float time = 0.f) const override;
	Eigen::Vector3f getVelocity() const override;
	Eigen::Vector3f getAcceleration() const override;

protected:
	PositionFilterConstants m_constants;
	class KalmanPositionFilterImpl *m_filter;
};

#endif // DEVICE_INTERFACE_H
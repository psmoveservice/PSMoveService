#ifndef POSITION_FILTER_H
#define POSITION_FILTER_H

//-- includes -----
#include "MathEigen.h"

//-- declarations -----
/// A snapshot of raw IMU data emitted from a device
struct PositionSensorPacket
{
    Eigen::Vector3f position;
    Eigen::Vector3f acceleration;
    bool bPositionValid;
};

/// A snapshot of raw IMU data transformed by a filter space so that it can be used to update an position filter
struct PositionFilterPacket
{
    Eigen::Vector3f position;
    Eigen::Vector3f acceleration;
    bool bPositionValid;
};

/// Used to transform sensor data from a device into an arbitrary space
class PositionFilterSpace
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PositionFilterSpace();
    PositionFilterSpace(const Eigen::Matrix3f &sensorTransform);

    inline void setSensorTransform(const Eigen::Matrix3f &sensorTransform)
    {
        m_SensorTransform = sensorTransform;
    }

    void convertSensorPacketToFilterPacket(
        const PositionSensorPacket &sensorPacket,
        PositionFilterPacket &outFilterPacket) const;

private:
    Eigen::Matrix3f m_SensorTransform;
};

/// A stateful filter used to fuse IMU sensor data into a position in a desired filter space
class PositionFilter
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum FusionType {
        FusionTypeNone,
        FusionTypePassThru,
        FusionTypeLowPass
        // TODO: Kalman
    };

    PositionFilter();
    virtual ~PositionFilter();

    inline PositionFilterSpace &getFilterSpace()
    {
        return m_FilterSpace;
    }

    // Estimate the current position of the filter given a time offset into the future
    Eigen::Vector3f getPosition(float time = 0.f);
    Eigen::Vector3f getVelocity();
    Eigen::Vector3f getAcceleration();

    void setFilterSpace(const PositionFilterSpace &filterSpace);
    void setFusionType(FusionType fusionType);

    void resetPosition();
    void resetFilterState();
    void update(const float delta_time, const PositionSensorPacket &packet);

private:
    PositionFilterSpace m_FilterSpace;
    struct PositionSensorFusionState *m_FusionState;
};

#endif // POSITION_FILTER_H
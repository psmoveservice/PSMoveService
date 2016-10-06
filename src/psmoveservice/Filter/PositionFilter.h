#ifndef POSITION_FILTER_H
#define POSITION_FILTER_H

//-- includes -----
#include "MathEigen.h"

//-- constants -----
enum PositionSource
{
    PositionSource_PreviousFrame,
    PositionSource_Optical
};

//-- declarations -----
/// A snapshot of raw IMU data emitted from a device
struct PositionSensorPacket
{
    Eigen::Vector3f world_position;
    PositionSource position_source;
    float position_quality; // [0, 1]

    Eigen::Quaternionf world_orientation; // output from orientation filter
    Eigen::Vector3f accelerometer;
};

/// A snapshot of raw IMU data transformed by a filter space so that it can be used to update an position filter
struct PositionFilterPacket
{
    Eigen::Vector3f position;
    PositionSource position_source;
    float position_quality; // [0, 1]

    Eigen::Vector3f world_accelerometer;
};

/// Filter parameters that remain constant during the lifetime of the the filter
struct PositionFilterConstants 
{
    float accelerometerNoiseRadius;
    float maxVelocity;
};

/// Used to transform sensor data from a device into an arbitrary space
class PositionFilterSpace
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PositionFilterSpace();
    PositionFilterSpace(
        const Eigen::Vector3f &identityGravity,
        const Eigen::Matrix3f &calibrationTransform,
        const Eigen::Matrix3f &sensorTransform);

    Eigen::Vector3f getGravityCalibrationDirection() const;

    inline void setCalibrationTransform(const Eigen::Matrix3f &calibrationTransform)
    { m_CalibrationTransform= calibrationTransform; }
    inline void setSensorTransform(const Eigen::Matrix3f &sensorTransform)
    { m_SensorTransform= sensorTransform; }


    void convertSensorPacketToFilterPacket(
        const PositionSensorPacket &sensorPacket,
        PositionFilterPacket &outFilterPacket) const;

private:
    Eigen::Vector3f m_IdentityGravity;

    Eigen::Matrix3f m_CalibrationTransform;
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
        FusionTypeLowPassOptical,
        FusionTypeLowPassIMU,
        FusionTypeComplimentaryOpticalIMU,
		FusionTypeLowPassExponential,
        // TODO: Kalman
    };

    PositionFilter();
    virtual ~PositionFilter();

    inline PositionFilterSpace &getFilterSpace()
    {
        return m_FilterSpace;
    }

    FusionType getFusionType() const;
    bool getIsFusionStateValid() const;
    /// Estimate the current position of the filter given a time offset into the future
    Eigen::Vector3f getPosition(float time = 0.f) const;
    Eigen::Vector3f getVelocity() const;
    Eigen::Vector3f getAcceleration() const;

    void setFilterSpace(const PositionFilterSpace &filterSpace);
    void setFusionType(FusionType fusionType);

    inline void setAccelerometerNoiseRadius(float noiseRadius)
    {
        m_FilterConstants.accelerometerNoiseRadius= noiseRadius;
    }
    inline void setMaxVelocity(float maxVelocity)
    {
        m_FilterConstants.maxVelocity= maxVelocity;
    }

    void resetPosition();
    void resetFilterState();
    void update(const float delta_time, const PositionSensorPacket &packet);

private:
    PositionFilterConstants m_FilterConstants;
    PositionFilterSpace m_FilterSpace;
    struct PositionSensorFusionState *m_FusionState;
};

#endif // POSITION_FILTER_H
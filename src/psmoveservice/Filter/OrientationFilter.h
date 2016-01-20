#ifndef ORIENTATION_FILTER_H
#define ORIENTATION_FILTER_H

//-- includes -----
#include "MathEigen.h"

//-- constants -----
// Calibration Pose transform
extern const Eigen::Matrix3f *k_eigen_identity_pose_upright;
extern const Eigen::Matrix3f *k_eigen_identity_pose_laying_flat;

//Sensor Transforms
extern const Eigen::Matrix3f *k_eigen_sensor_transform_identity;
extern const Eigen::Matrix3f *k_eigen_sensor_transform_opengl;

//-- declarations -----
/// A snapshot of IMU data emitted from a controller
struct OrientationSensorPacket
{
    Eigen::Vector3f accelerometer;
    Eigen::Vector3f magnetometer;
    Eigen::Vector3f gyroscope;
};

/// A snapshot of IMU data transformed by a filter space so that it can be used to update an orientation filter
struct OrientationFilterPacket
{
    Eigen::Vector3f normalized_accelerometer;
    Eigen::Vector3f normalized_magnetometer;
    Eigen::Vector3f gyroscope;
};

/// Used to transform sensor data from a controller into an arbitrary space
class OrientationFilterSpace
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    OrientationFilterSpace();
    OrientationFilterSpace(
        const Eigen::Vector3f &identityGravity,
        const Eigen::Vector3f &identityMagnetometer,
        const Eigen::Matrix3f &calibrationTransform,
        const Eigen::Matrix3f &sensorTransform);

    Eigen::Vector3f getGravityCalibrationDirection() const;
    Eigen::Vector3f getMagnetometerCalibrationDirection() const;

    inline void setCalibrationTransform(const Eigen::Matrix3f &calibrationTransform)
    { m_CalibrationTransform= calibrationTransform; }
    inline void setSensorTransform(const Eigen::Matrix3f &sensorTransform)
    { m_SensorTransform= sensorTransform; }

    void convertSensorPacketToFilterPacket(
        const OrientationSensorPacket &sensorPacket,
        OrientationFilterPacket &outFilterPacket) const;

private:
    Eigen::Vector3f m_IdentityGravity;
    Eigen::Vector3f m_IdentityMagnetometer;

    Eigen::Matrix3f m_CalibrationTransform;
    Eigen::Matrix3f m_SensorTransform;
};

/// A stateful filter used to fuse IMU sensor data into an orientation in a desired filter space
class OrientationFilter
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum FusionType {
        FusionTypeNone,
        FusionTypeMadgwickIMU,
        FusionTypeMadgwickMARG,
        FusionTypeComplementaryMARG,
    };

    OrientationFilter();
    virtual ~OrientationFilter();

    inline OrientationFilterSpace &getFilterSpace()
    { return m_FilterSpace; }
    Eigen::Quaternionf getOrientation();

    void setFilterSpace(const OrientationFilterSpace &filterSpace);
    void setFusionType(FusionType fusionType);

    void resetOrientation();
    void update(const float delta_time, const OrientationSensorPacket &packet);

private:
    OrientationFilterSpace m_FilterSpace;
    struct OrientationSensorFusionState *m_FusionState;
};

#endif // ORIENTATION_FILTER_H
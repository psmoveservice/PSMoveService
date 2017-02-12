#ifndef ORIENTATION_FILTER_H
#define ORIENTATION_FILTER_H

//-- includes -----
#include "PoseFilterInterface.h"

//-- definitions --
/// Abstract base class for all orientation only filters
class OrientationFilter : public IOrientationFilter
{
public:
    OrientationFilter();
    virtual ~OrientationFilter();

    //-- IStateFilter --
    bool getIsStateValid() const override;
    void resetState() override;
    void recenterOrientation(const Eigen::Quaternionf& q_pose) override;

    // -- IOrientationFilter --
    bool init(const OrientationFilterConstants &constant) override;
	bool init(const OrientationFilterConstants &constant, const Eigen::Quaternionf &initial_orientation) override;
    Eigen::Quaternionf getOrientation(float time = 0.f) const override;
    Eigen::Vector3f getAngularVelocityRadPerSec() const override;
    Eigen::Vector3f getAngularAccelerationRadPerSecSqr() const override;

protected:
    OrientationFilterConstants m_constants;
    struct OrientationFilterState *m_state;
};

/// Just use the optical orientation passed in unfiltered
class OrientationFilterPassThru : public OrientationFilter
{
public:
    void update(const float delta_time, const PoseFilterPacket &packet) override;
};

/// Angular Rate and Gravity fusion algorithm from Madgwick
class OrientationFilterMadgwickARG : public OrientationFilter
{
public:
    void update(const float delta_time, const PoseFilterPacket &packet) override;
};

/// Magnetic, Angular Rate, and Gravity fusion algorithm from Madgwick
class OrientationFilterMadgwickMARG : public OrientationFilterMadgwickARG
{
public:
    OrientationFilterMadgwickMARG()
        : OrientationFilterMadgwickARG()
        , m_omega_bias_x(0.f)
        , m_omega_bias_y(0.f)
        , m_omega_bias_z(0.f)
    {}

    void resetState() override;
    void update(const float delta_time, const PoseFilterPacket &packet) override;

protected:
    float m_omega_bias_x;
    float m_omega_bias_y;
    float m_omega_bias_z;
};

/// Angular Rate, Gravity, and Optical fusion algorithm
/// Blends between AngularRate-Grav Madgwick IMU update and optical orientation
class OrientationFilterComplementaryOpticalARG : public OrientationFilterMadgwickARG
{
public:
    void update(const float delta_time, const PoseFilterPacket &packet) override;
};

/// Magnetic, Angular Rate, Gravity and fusion algorithm (hybrid Madgwick)
/// Blends between best fit Mag-Grav alignment and Angular Rate integration
class OrientationFilterComplementaryMARG : public OrientationFilter
{
public:
    OrientationFilterComplementaryMARG()
        : OrientationFilter()
        , mg_weight(1.f)
    {}

    void resetState() override;
    void update(const float delta_time, const PoseFilterPacket &packet) override;

protected:
    float mg_weight;
};

#endif // ORIENTATION_FILTER_H
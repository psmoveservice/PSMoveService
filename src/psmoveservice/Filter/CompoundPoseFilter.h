#ifndef COMPOUND_POSE_FILTER_H
#define COMPOUND_POSE_FILTER_H

//-- includes -----
#include "PoseFilterInterface.h"

// -- constants --
enum OrientationFilterType {
    OrientationFilterTypeNone,
    OrientationFilterTypePassThru,
    OrientationFilterTypeMadgwickARG,
    OrientationFilterTypeMadgwickMARG,
    OrientationFilterTypeComplementaryOpticalARG,
    OrientationFilterTypeComplementaryMARG,
};

enum PositionFilterType {
    PositionFilterTypeNone,
    PositionFilterTypePassThru,
    PositionFilterTypeLowPassOptical,
    PositionFilterTypeLowPassIMU,
    PositionFilterTypeComplimentaryOpticalIMU,
	PositionFilterTypeKalman,
};

// -- definitions --
class CompoundPoseFilter : public IPoseFilter
{
public:
    CompoundPoseFilter() 
        : m_position_filter(nullptr)
        , m_orientation_filter(nullptr)
    {}
    virtual ~CompoundPoseFilter()
    { dispose(); }

    bool init(
        OrientationFilterType orientationFilterType, 
        PositionFilterType positionFilterType,
        const PoseFilterConstants &constant);

    // -- IStateFilter --
    bool getIsStateValid() const override;
    void update(const float delta_time, const PoseFilterPacket &packet) override;
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
    void dispose();

    IPositionFilter *m_position_filter;
    IOrientationFilter *m_orientation_filter;
};

#endif // COMPOUND_POSE_FILTER_H
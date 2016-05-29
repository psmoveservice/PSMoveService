// -- includes -----
#include "PositionFilter.h"
#include "MathEigen.h"
#include "ServerLog.h"
#include <deque>

//-- constants -----
// Max length of the position history we keep
#define k_position_history_max 16
#define k_max_lowpass_smoothing_distance 10.f // cm

// -- private definitions -----
struct StateSample
{
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    float delta_time;
};

struct PositionSensorFusionState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Is the current fustion state valid
    bool bIsValid;

    /// Current State of the filter
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;

    /// Position that's considered the origin position 
    Eigen::Vector3f origin_position;

    /// The filter fusion algorithm to use
    PositionFilter::FusionType fusion_type;

    void initialize()
    {
        bIsValid = false;
        position = Eigen::Vector3f::Zero();
        origin_position = Eigen::Vector3f::Zero();
        fusion_type = PositionFilter::FusionTypeLowPass;
    }
};

// -- globals -----

// -- private methods -----
static void position_fusion_lowpass_update(
    const float delta_time, const PositionFilterSpace *filter_space, const PositionFilterPacket *filter_packet,
    PositionSensorFusionState *fusion_state);

// -- public interface -----

//-- Orientation Filter Space -----
PositionFilterSpace::PositionFilterSpace()
    : m_SensorTransform(Eigen::Matrix3f::Identity())
{
}

PositionFilterSpace::PositionFilterSpace(
    const Eigen::Matrix3f &sensorTransform)
    : m_SensorTransform(sensorTransform)
{
}

void PositionFilterSpace::convertSensorPacketToFilterPacket(
    const PositionSensorPacket &sensorPacket,
    PositionFilterPacket &outFilterPacket) const
{
    if (sensorPacket.bPositionValid)
    {
        outFilterPacket.position = sensorPacket.position;
        outFilterPacket.bPositionValid = true;
    }
    else
    {
        outFilterPacket.position = Eigen::Vector3f::Zero();
        outFilterPacket.bPositionValid = false;
    }

    outFilterPacket.acceleration = m_SensorTransform * sensorPacket.acceleration;
}

//-- Orientation Filter -----
PositionFilter::PositionFilter()
    : m_FilterSpace()
    , m_FusionState(new PositionSensorFusionState)
{
    m_FusionState->initialize();
}

PositionFilter::~PositionFilter()
{
    delete m_FusionState;
}

Eigen::Vector3f PositionFilter::getPosition(float time)
{
    Eigen::Vector3f result = Eigen::Vector3f::Zero();

    if (m_FusionState->bIsValid)
    {
        Eigen::Vector3f predicted_position = 
            is_nearly_zero(time)
            ? m_FusionState->position
            : m_FusionState->position + m_FusionState->velocity * time;

        result= predicted_position - m_FusionState->origin_position;
    }

    return result;
}

Eigen::Vector3f PositionFilter::getVelocity()
{
    return (m_FusionState->bIsValid) ? m_FusionState->velocity : Eigen::Vector3f::Zero();
}

void PositionFilter::setFilterSpace(const PositionFilterSpace &filterSpace)
{
    m_FilterSpace = filterSpace;
    m_FusionState->initialize();
}

void PositionFilter::setFusionType(PositionFilter::FusionType fusionType)
{
    m_FusionState->fusion_type = fusionType;

    switch (m_FusionState->fusion_type)
    {
    case FusionTypeNone:
    case FusionTypePassThru:
        // No initialization
        break;
    case FusionTypeLowPass:
        break;
    // TODO: Kalman
    default:
        break;
    }
}

void PositionFilter::resetPosition()
{
    m_FusionState->origin_position = m_FusionState->position;
}

void PositionFilter::resetFilterState()
{
    m_FusionState->initialize();
}

void PositionFilter::update(
    const float delta_time,
    const PositionSensorPacket &sensorPacket)
{
    PositionFilterPacket filterPacket;
    m_FilterSpace.convertSensorPacketToFilterPacket(sensorPacket, filterPacket);

    Eigen::Vector3f position_backup = m_FusionState->position;
    Eigen::Vector3f velocity_backup = m_FusionState->velocity;

    switch (m_FusionState->fusion_type)
    {
    case FusionTypeNone:
        break;
    case FusionTypePassThru:
        m_FusionState->position = filterPacket.position;
        break;
    case FusionTypeLowPass:
        position_fusion_lowpass_update(delta_time, &m_FilterSpace, &filterPacket, m_FusionState);
        break;
    // TODO: Kalman
    }

    if (!eigen_vector3f_is_valid(m_FusionState->position))
    {
        SERVER_LOG_WARNING("PositionFilter") << "Position is NaN!";
        m_FusionState->position = position_backup;
    }

    if (!eigen_vector3f_is_valid(m_FusionState->velocity))
    {
        SERVER_LOG_WARNING("PositionFilter") << "Velocity is NaN!";
        m_FusionState->velocity = velocity_backup;
    }
}

// -- Position Filters ----
static void
position_fusion_lowpass_update(
    const float delta_time,
    const PositionFilterSpace *filter_space,
    const PositionFilterPacket *filter_packet,
    PositionSensorFusionState *fusion_state)
{
    if (filter_packet->bPositionValid && eigen_vector3f_is_valid(filter_packet->position))
    {
        if (fusion_state->bIsValid)
        {
            // Traveling k_max_lowpass_smoothing_distance in one frame should have 0 smoothing
            // Traveling 0+noise cm in one frame should have 60% smoothing
            Eigen::Vector3f diff = filter_packet->position - fusion_state->position;
            float distance = diff.norm();
            float new_position_weight = clampf01(lerpf(0.40f, 1.00f, distance / k_max_lowpass_smoothing_distance));

            // New position is blended against the old position
            const Eigen::Vector3f old_position = fusion_state->position;
            const Eigen::Vector3f new_position = filter_packet->position;
            fusion_state->position = old_position*(1.f - new_position_weight) + new_position*new_position_weight;

            // Compute the velocity of the blended position
            if (!is_nearly_zero(delta_time))
            {
                fusion_state->velocity = (new_position - old_position) / delta_time;
            }
            else
            {
                fusion_state->velocity = Eigen::Vector3f::Zero();
            }
        }
        else
        {
            // If this is the first filter packet, just accept the position as gospel
            fusion_state->position = filter_packet->position;
            fusion_state->velocity = Eigen::Vector3f::Zero();

            // Fusion state is valid now that we have one sample
            fusion_state->bIsValid = true;
        }
    }
}
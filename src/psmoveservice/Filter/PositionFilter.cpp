// -- includes -----
#include "PositionFilter.h"
#include "MathEigen.h"
#include "ServerLog.h"
#include <deque>

//-- constants -----
// Max length of the position history we keep
#define k_position_history_max 16

// -- private definitions -----
struct PositionSample
{
    Eigen::Vector3f position;
};

struct PositionSensorFusionState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Recent history of orientation readings
    std::deque<PositionSample> positionHistory;

    /* Output value as vector */
    Eigen::Vector3f position;

    /* Position that's considered the origin position */
    Eigen::Vector3f reset_position;

    PositionFilter::FusionType fusion_type;

    void initialize()
    {
        position = Eigen::Vector3f::Zero();
        reset_position = Eigen::Vector3f::Zero();
        fusion_type = PositionFilter::FusionTypePassThru;
        positionHistory.clear();
    }
};

// -- globals -----

// -- private methods -----

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
    outFilterPacket.position = sensorPacket.position;
    outFilterPacket.velocity =  m_SensorTransform * sensorPacket.velocity;
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

// Estimate the current orientation of the filter given a time offset
// Positive time values estimate into the future
// Negative time values get pose values from the past
Eigen::Vector3f PositionFilter::getPosition(int msec_time)
{
    //###bwalker $TODO Use the position history to compute an orientation

    Eigen::Vector3f result = m_FusionState->position - m_FusionState->reset_position;

    return result;
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
    // TODO: LowPass
    // TODO: Kalman
    default:
        break;
    }
}

void PositionFilter::resetPosition()
{
    m_FusionState->reset_position = m_FusionState->position;
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

    switch (m_FusionState->fusion_type)
    {
    case FusionTypeNone:
        break;
    case FusionTypePassThru:
        m_FusionState->position = filterPacket.position;
        break;
    // TODO: LowPass
    // TODO: Kalman
    }

    if (!eigen_vector3f_is_valid(m_FusionState->position))
    {
        SERVER_LOG_WARNING("PositionFilter") << "Position is NaN!";
        m_FusionState->position = position_backup;
    }

    // Add the new position sample to the position history
    {
        PositionSample sample;

        sample.position = m_FusionState->position;
        //###bwalker $TODO Timestamp?

        // Make room for new entry if at the max queue size
        if (m_FusionState->positionHistory.size() >= k_position_history_max)
        {
            m_FusionState->positionHistory.erase(
                m_FusionState->positionHistory.begin(),
                m_FusionState->positionHistory.begin() + m_FusionState->positionHistory.size() - k_position_history_max);
        }

        m_FusionState->positionHistory.push_back(sample);
    }
}
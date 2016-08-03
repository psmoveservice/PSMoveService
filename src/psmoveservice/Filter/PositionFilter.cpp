// -- includes -----
#include "PositionFilter.h"
#include "MathEigen.h"
#include "ServerLog.h"
#include <deque>

//-- constants -----
// Max length of the position history we keep
#define k_position_history_max 16

// THe max distance between samples that we apply low pass filter on the optical position filter
#define k_max_lowpass_smoothing_distance 10.f * k_centimeters_to_meters // meters

// Used for lowpass filter of accelerometer
#define k_accelerometer_frequency_cutoff 240.f // Hz

// Past this time the complimentary filter will no longer keep doing
// IMU extrapolation of an unseen controller
#define k_max_unseen_position_timeout 3000.f // ms

// 1 g-unit is equal 980.66499997877 gal (cm/s²)
#define k_g_units_to_gal  980.665000f // gal (cm/s²)
#define k_g_units_to_ms2  9.80665000f // m/s²

#define k_meters_to_centimeters  100.f
#define k_centimeters_to_meters  0.01f

// -- private definitions -----
struct PositionSensorFusionState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Is the current fustion state valid
    bool bIsValid;

    /// Current Physics State of the filter
    Eigen::Vector3f position; // meters
    Eigen::Vector3f velocity; // meters/s
    Eigen::Vector3f acceleration; // (meters/s²)

    /// Position that's considered the origin position 
    Eigen::Vector3f origin_position; // meters

    /// The filter fusion algorithm to use
    PositionFilter::FusionType fusion_type;

    std::chrono::time_point<std::chrono::high_resolution_clock> last_visible_position_timestamp;
    bool bLast_visible_position_timestamp_valid;

    void initialize()
    {
        bIsValid = false;
        position = Eigen::Vector3f::Zero();
        velocity = Eigen::Vector3f::Zero();
        acceleration = Eigen::Vector3f::Zero();
        origin_position = Eigen::Vector3f::Zero();
        fusion_type = PositionFilter::FusionTypeLowPassOptical;
        bLast_visible_position_timestamp_valid= false;
    }
};

// -- globals -----

// -- private methods -----
static void position_fusion_lowpass_optical_update(
    const float delta_time, const PositionFilterSpace *filter_space, const PositionFilterPacket *filter_packet,
    PositionSensorFusionState *fusion_state);
static void position_fusion_lowpass_imu_update(
    const float delta_time, const PositionFilterSpace *filter_space, const PositionFilterPacket *filter_packet,
    PositionSensorFusionState *fusion_state);
static void position_fusion_complimentary_optical_imu_update(
    const float delta_time, const PositionFilterSpace *filter_space, const PositionFilterPacket *filter_packet,
    PositionSensorFusionState *fusion_state);

// -- public interface -----

//-- Orientation Filter Space -----
PositionFilterSpace::PositionFilterSpace()
    :  m_IdentityGravity(Eigen::Vector3f(0.f, 1.f, 0.f))
    , m_CalibrationTransform(Eigen::Matrix3f::Identity())
    , m_SensorTransform(Eigen::Matrix3f::Identity())
{
}

PositionFilterSpace::PositionFilterSpace(
    const Eigen::Vector3f &identityGravity,
    const Eigen::Matrix3f &calibrationTransform,
    const Eigen::Matrix3f &sensorTransform)
    : m_IdentityGravity(identityGravity)
    , m_CalibrationTransform(calibrationTransform)
    , m_SensorTransform(sensorTransform)
{
}

Eigen::Vector3f PositionFilterSpace::getGravityCalibrationDirection() const
{
    // First apply the calibration data transform.
    // This allows us to pretend the "identity pose" was some other orientation the vertical during calibration
    const Eigen::Vector3f calibrationSpaceVector= m_CalibrationTransform * m_IdentityGravity;

    // Next apply the sensor data transform.
    // This allows us to pretend the sensors are in some other coordinate system (like OpenGL where +Y is up)
    const Eigen::Vector3f filterSpaceVector= m_SensorTransform * calibrationSpaceVector;

    return filterSpaceVector;
}

void PositionFilterSpace::convertSensorPacketToFilterPacket(
    const PositionSensorPacket &sensorPacket,
    PositionFilterPacket &outFilterPacket) const
{
    static float g_scale= k_g_units_to_ms2;

    Eigen::Vector3f local_accelerometer= m_SensorTransform * sensorPacket.accelerometer;
    Eigen::Vector3f local_acceleration= (local_accelerometer - getGravityCalibrationDirection()) * g_scale;

    outFilterPacket.world_acceleration= 
        eigen_vector3f_clockwise_rotate(sensorPacket.world_orientation, local_acceleration);

    outFilterPacket.position= sensorPacket.world_position * k_centimeters_to_meters;
    outFilterPacket.position_source= sensorPacket.position_source;
    outFilterPacket.position_quality= sensorPacket.position_quality;
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

PositionFilter::FusionType PositionFilter::getFusionType() const
{
    return m_FusionState->fusion_type;
}

Eigen::Vector3f PositionFilter::getPosition(float time) const
{
    Eigen::Vector3f result = Eigen::Vector3f::Zero();

    if (m_FusionState->bIsValid)
    {
        Eigen::Vector3f predicted_position = 
            is_nearly_zero(time)
            ? m_FusionState->position
            : m_FusionState->position + m_FusionState->velocity * time;

        result= predicted_position - m_FusionState->origin_position;
        result= result * k_meters_to_centimeters;
    }

    return result;
}

Eigen::Vector3f PositionFilter::getVelocity() const
{
    Eigen::Vector3f result= m_FusionState->velocity * k_meters_to_centimeters;

    return (m_FusionState->bIsValid) ? result : Eigen::Vector3f::Zero();
}

Eigen::Vector3f PositionFilter::getAcceleration() const
{
    Eigen::Vector3f result= m_FusionState->acceleration * k_meters_to_centimeters;

    return (m_FusionState->bIsValid) ? result : Eigen::Vector3f::Zero();
}

void PositionFilter::setFilterSpace(const PositionFilterSpace &filterSpace)
{
    m_FilterSpace = filterSpace;
    m_FusionState->initialize();
}

void PositionFilter::setFusionType(PositionFilter::FusionType fusionType)
{
    m_FusionState->fusion_type = fusionType;
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
    Eigen::Vector3f acceleration_backup = m_FusionState->acceleration;

    if (sensorPacket.position_quality > 0)
    {
        m_FusionState->last_visible_position_timestamp= std::chrono::high_resolution_clock::now();
        m_FusionState->bLast_visible_position_timestamp_valid= true;
    }

    switch (m_FusionState->fusion_type)
    {
    case FusionTypeNone:
        break;
    case FusionTypePassThru:
        m_FusionState->position = filterPacket.position;
        break;
    case FusionTypeLowPassOptical:
        position_fusion_lowpass_optical_update(delta_time, &m_FilterSpace, &filterPacket, m_FusionState);
        break;
    case FusionTypeLowPassIMU:
        position_fusion_lowpass_optical_update(delta_time, &m_FilterSpace, &filterPacket, m_FusionState);
        break;
    case FusionTypeComplimentaryOpticalIMU:
        position_fusion_complimentary_optical_imu_update(delta_time, &m_FilterSpace, &filterPacket, m_FusionState);
        break;
    // TODO: Kalman
    default:
        assert(0 && "unreachable");
    }

    if (!eigen_vector3f_is_valid(m_FusionState->position))
    {
        SERVER_LOG_WARNING("PositionFilter") << "Position is NaN!" << std::endl;
        m_FusionState->position = position_backup;
    }

    if (!eigen_vector3f_is_valid(m_FusionState->velocity))
    {
        SERVER_LOG_WARNING("PositionFilter") << "Velocity is NaN!" << std::endl;
        m_FusionState->velocity = velocity_backup;
    }

    if (!eigen_vector3f_is_valid(m_FusionState->acceleration))
    {
        SERVER_LOG_WARNING("PositionFilter") << "Acceleration is NaN!" << std::endl;
        m_FusionState->acceleration = acceleration_backup;
    }
}

// -- Position Filters ----
static Eigen::Vector3f lowpass_filter_position(
    const PositionFilterPacket *filter_packet,
    const PositionSensorFusionState *fusion_state)
{
    assert(fusion_state->bIsValid);

    // Traveling k_max_lowpass_smoothing_distance in one frame should have 0 smoothing
    // Traveling 0+noise cm in one frame should have 60% smoothing
    Eigen::Vector3f diff = filter_packet->position - fusion_state->position;
    float distance = diff.norm();
    float new_position_weight = clampf01(lerpf(0.40f, 1.00f, distance / k_max_lowpass_smoothing_distance));

    // New position is blended against the old position
    const Eigen::Vector3f &old_position = fusion_state->position;
    const Eigen::Vector3f &new_position = filter_packet->position;
    const Eigen::Vector3f filtered_new_position = 
        old_position*(1.f - new_position_weight) + new_position*new_position_weight;

    return filtered_new_position;
}

static void
position_fusion_lowpass_optical_update(
    const float delta_time,
    const PositionFilterSpace *filter_space,
    const PositionFilterPacket *filter_packet,
    PositionSensorFusionState *fusion_state)
{
    if (filter_packet->position_quality > 0.f && eigen_vector3f_is_valid(filter_packet->position))
    {
        if (fusion_state->bIsValid)
        {
            // New position is blended against the old position
            fusion_state->position = lowpass_filter_position(filter_packet, fusion_state);
            //###HipsterSloth $TODO derivatives of source signal, even a low pass filtered one, is too noisy
            fusion_state->velocity = Eigen::Vector3f::Zero(); // new_velocity;
            fusion_state->acceleration = Eigen::Vector3f::Zero(); //new_acceleration;
        }
        else
        {
            // If this is the first filter packet, just accept the position as gospel
            fusion_state->position = filter_packet->position;
            fusion_state->velocity = Eigen::Vector3f::Zero();
            fusion_state->acceleration = Eigen::Vector3f::Zero();

            // Fusion state is valid now that we have one sample
            fusion_state->bIsValid = true;
        }
    }
}

static void lowpass_filter_imu_step(
    const float delta_time,
    const PositionFilterPacket *filter_packet,
    const PositionSensorFusionState *fusion_state,
    Eigen::Vector3f *out_acceleration,
    Eigen::Vector3f *out_velocity,
    Eigen::Vector3f *out_position)
{
    assert(fusion_state->bIsValid);

    static float g_frquency= k_accelerometer_frequency_cutoff;

    // https://en.wikipedia.org/wiki/Low-pass_filter
    const float RC= 1.f/(k_real_two_pi*g_frquency);
    const float alpha= clampf01(delta_time / (RC + delta_time));

    const Eigen::Vector3f &old_gravity = fusion_state->acceleration;
    const Eigen::Vector3f &old_acceleration = fusion_state->acceleration;
    const Eigen::Vector3f &new_acceleration = filter_packet->world_acceleration;

    const Eigen::Vector3f filtered_new_acceleration = 
        alpha*new_acceleration + (1.f - alpha)*old_acceleration;

    *out_acceleration = filtered_new_acceleration;

    // x1 = x0 + v0*dt + 0.5*a1*dt^t
    *out_position = 
        0.5f*filtered_new_acceleration*delta_time*delta_time
        + fusion_state->velocity*delta_time
        + fusion_state->position;

    // v1 = v0*dt
    *out_velocity = 
        filtered_new_acceleration*delta_time
        + fusion_state->velocity;
}

static void
position_fusion_lowpass_imu_update(
    const float delta_time,
    const PositionFilterSpace *filter_space,
    const PositionFilterPacket *filter_packet,
    PositionSensorFusionState *fusion_state)
{
    if (eigen_vector3f_is_valid(filter_packet->world_acceleration))
    {
        if (fusion_state->bIsValid)
        {
            lowpass_filter_imu_step(
                delta_time,
                filter_packet,
                fusion_state,
                &fusion_state->acceleration,
                &fusion_state->velocity,
                &fusion_state->position);
        }
        else
        {
            // If this is the first filter packet, just accept the position as gospel
            fusion_state->position = filter_packet->position;
            fusion_state->velocity = Eigen::Vector3f::Zero();
            fusion_state->acceleration = Eigen::Vector3f::Zero();

            // Fusion state is valid now that we have one sample
            fusion_state->bIsValid = true;
        }
    }
}

static void
position_fusion_complimentary_optical_imu_update(
    const float delta_time,
    const PositionFilterSpace *filter_space,
    const PositionFilterPacket *filter_packet,
    PositionSensorFusionState *fusion_state)
{
    if (fusion_state->bIsValid)
    {
        bool bValidRecentPosition= false;

        // Make sure it hasn't been too long since we've last seen the controller
        // IMU integration will get bad pretty quickly
        if (fusion_state->bLast_visible_position_timestamp_valid)
        {
            const std::chrono::duration<float, std::milli> time_delta =
                std::chrono::high_resolution_clock::now() - fusion_state->last_visible_position_timestamp;
            const float time_delta_milli = time_delta.count();

            bValidRecentPosition= time_delta_milli < k_max_unseen_position_timeout;
        }

        if (bValidRecentPosition)
        {
            // Compute the new filter state based on the previous filter state and new sensor data
            Eigen::Vector3f imu_acceleration;
            Eigen::Vector3f imu_velocity;
            Eigen::Vector3f imu_position;
            lowpass_filter_imu_step(
                delta_time,
                filter_packet,
                fusion_state,
                &imu_acceleration,
                &imu_velocity,
                &imu_position);

            if (filter_packet->position_quality > 0)
            {
                // Compute a low-pass filter on the optical position update
                Eigen::Vector3f optical_position = lowpass_filter_position(filter_packet, fusion_state);

                // Blend the optical and IMU derived positions based on optical tracking quality
                const float optical_weight= filter_packet->position_quality;
                fusion_state->position= optical_weight*optical_position + (1.f - optical_weight)*imu_position;
            }
            else
            {
                fusion_state->position= imu_position;
            }

            // Always trust the IMU for acceleration and velocity
            // since the optically derived velocity and acceleration are too noisy
            fusion_state->velocity= imu_velocity;
            fusion_state->acceleration= imu_acceleration;
        }
        else
        {
            // Zero out the velocity and acceleration, but leave the position alone
            fusion_state->velocity = Eigen::Vector3f::Zero();
            fusion_state->acceleration = Eigen::Vector3f::Zero();

            // Fusion state is no longer valid
            fusion_state->bIsValid = false;
        }
    }
    else if (filter_packet->position_quality > 0.f && eigen_vector3f_is_valid(filter_packet->position))
    {
        // If this is the first filter packet, just accept the position as gospel
        fusion_state->position = filter_packet->position;
        fusion_state->velocity = Eigen::Vector3f::Zero();
        fusion_state->acceleration = Eigen::Vector3f::Zero();

        // Fusion state is valid now that we have one sample
        fusion_state->bIsValid = true;
    }
}

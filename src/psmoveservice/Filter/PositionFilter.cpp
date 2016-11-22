// -- includes -----
#include "PositionFilter.h"
#include "MathEigen.h"
#include "ServerLog.h"

#include <chrono>

//-- constants -----
// The max distance between samples that we apply low pass filter on the optical position filter
#define k_max_lowpass_smoothing_distance 10.f * k_centimeters_to_meters // meters

// Used for lowpass filter of accelerometer
#define k_accelerometer_frequency_cutoff 1000.f // Hz

// Decay rate to apply to the jerk
#define k_jerk_decay 0.9f

// Decay rate to apply to the acceleration
#define k_acceleration_decay 0.99f

// Decay rate to apply to the velocity
#define k_velocity_decay 0.99f

// Past this time the complimentary filter will no longer keep doing
// IMU extrapolation of an unseen controller
#define k_max_unseen_position_timeout 10000.f // ms

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

    /// Current Sensor State of the filter
    Eigen::Vector3f accelerometer; // (g-unit)
    Eigen::Vector3f accelerometer_derivative; // rate of change of the accelerometer in g-unit/s

    /// Current Physics State of the filter
    Eigen::Vector3f position; // meters
    Eigen::Vector3f velocity; // meters/s
    Eigen::Vector3f acceleration; // accelerometer minus gravity in meters/s^2

    /// Position that's considered the origin position 
    Eigen::Vector3f origin_position; // meters

	// required for smoothing position to get velocity
	float exp_delta_time;
	Eigen::Vector3f exp_position;

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
        accelerometer = Eigen::Vector3f::Zero();
        accelerometer_derivative = Eigen::Vector3f::Zero();
        origin_position = Eigen::Vector3f::Zero();
        bLast_visible_position_timestamp_valid= false;
		exp_delta_time = 0.f;
		exp_position = Eigen::Vector3f::Zero();
    }
};

// -- globals -----

// -- private methods -----
static void position_fusion_lowpass_exponential_update(
	const float delta_time,
	const PositionFilterConstants *constants, const PositionFilterSpace *space, const PositionFilterPacket *packet,
	PositionSensorFusionState *fusion_state);
static void position_fusion_lowpass_optical_update(
    const float delta_time, 
    const PositionFilterConstants *constants, const PositionFilterSpace *space, const PositionFilterPacket *packet,
    PositionSensorFusionState *fusion_state);
static void position_fusion_lowpass_imu_update(
    const float delta_time, 
    const PositionFilterConstants *constants, const PositionFilterSpace *space, const PositionFilterPacket *packet,
    PositionSensorFusionState *fusion_state);
static void position_fusion_complimentary_optical_imu_update(
    const float delta_time,
    const PositionFilterConstants *constants, const PositionFilterSpace *space, const PositionFilterPacket *packet,
    PositionSensorFusionState *fusion_state);

// -- public interface -----

//-- Orientation Filter Space -----
PositionFilterSpace::PositionFilterSpace()
    : m_IdentityGravity(Eigen::Vector3f(0.f, 1.f, 0.f))
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
    // Put the accelerometer reading in world space
    // Accelerometer readings in g-units
    Eigen::Vector3f local_accelerometer= m_SensorTransform * sensorPacket.accelerometer;
    outFilterPacket.world_accelerometer= 
        eigen_vector3f_clockwise_rotate(sensorPacket.world_orientation, local_accelerometer);
    
    // Internally we store the position in meters
    outFilterPacket.position= sensorPacket.world_position * k_centimeters_to_meters;
    outFilterPacket.position_source= sensorPacket.position_source;
    outFilterPacket.position_quality= sensorPacket.position_quality;
}

//-- Orientation Filter -----
PositionFilter::PositionFilter()
    : m_FilterSpace()
    , m_FusionState(new PositionSensorFusionState)
{
    memset(&m_FilterConstants, 0, sizeof(PositionFilterConstants));
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

bool PositionFilter::getIsFusionStateValid() const
{
    return m_FusionState->bIsValid;
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
    Eigen::Vector3f jerk_backup = m_FusionState->accelerometer_derivative;

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
        position_fusion_lowpass_optical_update(delta_time, &m_FilterConstants, &m_FilterSpace, &filterPacket, m_FusionState);
        break;
    case FusionTypeLowPassIMU:
        position_fusion_lowpass_optical_update(delta_time, &m_FilterConstants, &m_FilterSpace, &filterPacket, m_FusionState);
        break;
    case FusionTypeComplimentaryOpticalIMU:
        position_fusion_complimentary_optical_imu_update(delta_time, &m_FilterConstants, &m_FilterSpace, &filterPacket, m_FusionState);
        break;
	case FusionTypeLowPassExponential:
		position_fusion_lowpass_exponential_update(delta_time, &m_FilterConstants, &m_FilterSpace, &filterPacket, m_FusionState);
		break;
    // TODO: Kalman
    default:
        assert(0 && "unreachable");
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

    if (!eigen_vector3f_is_valid(m_FusionState->acceleration))
    {
        SERVER_LOG_WARNING("PositionFilter") << "Acceleration is NaN!";
        m_FusionState->acceleration = acceleration_backup;
    }

    if (!eigen_vector3f_is_valid(m_FusionState->accelerometer_derivative))
    {
        SERVER_LOG_WARNING("PositionFilter") << "Jerk is NaN!";
        m_FusionState->accelerometer_derivative = jerk_backup;
    }
}

// -- Position Filters ----
static Eigen::Vector3f threshold_vector3f(const Eigen::Vector3f &vector, const float min_length)
{
    const float length= vector.norm();
    const Eigen::Vector3f result= (length > min_length) ? vector : Eigen::Vector3f::Zero();

    return result;
}

static Eigen::Vector3f clamp_vector3f(const Eigen::Vector3f &vector, const float max_length)
{
    const float length= vector.norm();
    const Eigen::Vector3f result= 
        (length > max_length && length > k_real_epsilon) ? vector * (max_length / length) : vector;

    return result;
}

static Eigen::Vector3f lowpass_filter_vector3f(
    const float alpha,
    const Eigen::Vector3f &old_filtered_vector,
    const Eigen::Vector3f &new_vector)
{
    const Eigen::Vector3f filtered_vector= alpha*new_vector + (1.f - alpha)*old_filtered_vector;

    return filtered_vector;
}

static Eigen::Vector3f lowpass_filter_vector3f(
    const float delta_time,
    const float cutoff_frequency,
    const Eigen::Vector3f &old_filtered_vector,
    const Eigen::Vector3f &new_vector)
{
    // https://en.wikipedia.org/wiki/Low-pass_filter
    const float RC= 1.f/(k_real_two_pi*cutoff_frequency);
    const float alpha= clampf01(delta_time / (RC + delta_time));
    const Eigen::Vector3f filtered_vector= lowpass_filter_vector3f(alpha, old_filtered_vector, new_vector);

    return filtered_vector;
}

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
    const Eigen::Vector3f filtered_new_position = lowpass_filter_vector3f(new_position_weight, old_position, new_position);

    return filtered_new_position;
}

static void
position_fusion_lowpass_exponential_update(
	const float delta_time,
	const PositionFilterConstants *filter_constants,
	const PositionFilterSpace *filter_space,
	const PositionFilterPacket *filter_packet,
	PositionSensorFusionState *fusion_state)
{
	if (filter_packet->position_quality > 0.f && eigen_vector3f_is_valid(filter_packet->position))
	{
		fusion_state->accelerometer = filter_packet->world_accelerometer;
		fusion_state->accelerometer_derivative = Eigen::Vector3f::Zero();

		int queueLen = 3;
		float smooth = 0.6f;

		if (fusion_state->bIsValid)
		{
			// New position is blended against the old position
			fusion_state->position = lowpass_filter_position(filter_packet, fusion_state);

			float q = 0.4f;
			Eigen::Vector3f prev_position = fusion_state->exp_position;
			float prev_delta_time = fusion_state->exp_delta_time;

			fusion_state->exp_position = (fusion_state->position * q) + (prev_position * (1.0f - q));
			fusion_state->exp_delta_time = (delta_time * q) + (prev_delta_time * (1.0f - q));

			fusion_state->velocity = (fusion_state->exp_position - prev_position) / fusion_state->exp_delta_time;

			//fusion_state->velocity = Eigen::Vector3f::Zero();
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

static void
position_fusion_lowpass_optical_update(
    const float delta_time,
    const PositionFilterConstants *filter_constants,
    const PositionFilterSpace *filter_space,
    const PositionFilterPacket *filter_packet,
    PositionSensorFusionState *fusion_state)
{
    if (filter_packet->position_quality > 0.f && eigen_vector3f_is_valid(filter_packet->position))
    {        
        fusion_state->accelerometer= filter_packet->world_accelerometer;
        fusion_state->accelerometer_derivative= Eigen::Vector3f::Zero();

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
    const PositionFilterConstants *filter_constants,
    const PositionFilterPacket *filter_packet,
    PositionSensorFusionState *fusion_state,
    Eigen::Vector3f *out_position)
{
    assert(fusion_state->bIsValid);

    if (delta_time > k_real_epsilon)
    {
        // Gather sensor state from the previous frame
        const Eigen::Vector3f old_accelerometer= fusion_state->accelerometer;
        const Eigen::Vector3f old_accelerometer_derivative= fusion_state->accelerometer_derivative;

        // Gather physics state from the previous frame
        const Eigen::Vector3f old_position = fusion_state->position;
        const Eigen::Vector3f old_velocity = fusion_state->velocity;
        const Eigen::Vector3f old_acceleration = fusion_state->acceleration;

        // Gather new sensor readings
        // Need to negate the accelerometer reading since it points the opposite direction of gravity)
        const Eigen::Vector3f &new_accelerometer= -filter_packet->world_accelerometer;
   
        // Compute the filtered derivative of the accelerometer (a.k.a. the "jerk")
        Eigen::Vector3f new_accelerometer_derivative= Eigen::Vector3f::Zero();
        {
            const Eigen::Vector3f accelerometer_derivative=
                (new_accelerometer - old_accelerometer) / delta_time;

            // Throw out any jerk below the noise threshold
            const Eigen::Vector3f thresholded_derivative= 
                threshold_vector3f(accelerometer_derivative, filter_constants->accelerometerNoiseRadius);

            // Apply a decay filter to the jerk
            static float g_jerk_decay= k_jerk_decay;
            new_accelerometer_derivative= thresholded_derivative*g_jerk_decay;
        }

        // The accelerometer is in "g-units", where 1 g-unit = 9.8m/s^2
        const Eigen::Vector3f old_jerk= old_accelerometer_derivative * k_g_units_to_ms2;
        const Eigen::Vector3f new_jerk= new_accelerometer_derivative * k_g_units_to_ms2;

        // Convert the new acceleration by integrating the jerk
        Eigen::Vector3f new_acceleration= 
            0.5f*(old_jerk + new_jerk)*delta_time 
            + old_acceleration;

        // Apply a lowpass filter to the acceleration
        static float g_cutoff_frequency= k_accelerometer_frequency_cutoff;
        new_acceleration= 
            lowpass_filter_vector3f(
                g_cutoff_frequency, delta_time, 
                old_acceleration, new_acceleration);

        // Apply a decay filter to the acceleration
        static float g_acceleration_decay= k_acceleration_decay;
        new_acceleration*= g_acceleration_decay;

        // new velocity v1 = v0 + 0.5*(a0+a1)*dt, using trapezoid rule integration
        const Eigen::Vector3f new_unclamped_velocity = 
            0.5f*(old_acceleration + new_acceleration)*delta_time
            + old_velocity;

        // Make sure the velocity doesn't exceed the speed limit
        Eigen::Vector3f new_velocity =
            clamp_vector3f(new_unclamped_velocity, filter_constants->maxVelocity);
        
        // Apply a decay filter to the acceleration
        static float g_velocity_decay= k_velocity_decay;
        new_velocity*= g_velocity_decay;

        // new position x1 = x0 + 0.5*(v0+v1)*dt, using trapezoid rule integration
        // Pass the updated position out so that the caller can decide
        // how it should be applied to the fusion state 
        // (i.e. blend it with optical position)
        const Eigen::Vector3f new_position = 
            0.5f*(old_velocity + new_velocity)*delta_time
            + old_position;

        // Save out the updated sensor state
        fusion_state->accelerometer= new_accelerometer;
        fusion_state->accelerometer_derivative= new_accelerometer_derivative;

        // Save out the updated fusion state
        fusion_state->acceleration = new_acceleration;
        fusion_state->velocity = new_velocity;
        *out_position= new_position;
    }
    else
    {
        *out_position= fusion_state->position;
    }
}

static void
position_fusion_lowpass_imu_update(
    const float delta_time,
    const PositionFilterConstants *filter_constants,
    const PositionFilterSpace *filter_space,
    const PositionFilterPacket *filter_packet,
    PositionSensorFusionState *fusion_state)
{
    if (eigen_vector3f_is_valid(filter_packet->world_accelerometer))
    {
        if (fusion_state->bIsValid)
        {
            lowpass_filter_imu_step(
                delta_time,
                filter_constants,
                filter_packet,
                fusion_state,
                &fusion_state->position);
        }
        else
        {
            // If this is the first filter packet, just accept the position as gospel
            fusion_state->position = filter_packet->position;
            fusion_state->velocity = Eigen::Vector3f::Zero();
            fusion_state->acceleration = Eigen::Vector3f::Zero();
            fusion_state->accelerometer = filter_packet->world_accelerometer;
            fusion_state->accelerometer_derivative = Eigen::Vector3f::Zero();

            // Fusion state is valid now that we have one sample
            fusion_state->bIsValid = true;
        }
    }
}

static void
position_fusion_complimentary_optical_imu_update(
    const float delta_time,
    const PositionFilterConstants *filter_constants,
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

            static float g_max_unseen_position_timeout= k_max_unseen_position_timeout;
            bValidRecentPosition= time_delta_milli < g_max_unseen_position_timeout;
        }

        if (bValidRecentPosition)
        {
            // Compute the new filter state based on the previous filter state and new sensor data
            Eigen::Vector3f imu_position;
            lowpass_filter_imu_step(
                delta_time,
                filter_constants,
                filter_packet,
                fusion_state,
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
        }
        else
        {
            // Zero out the derived state, but leave the sensor state and position state alone
            fusion_state->velocity = Eigen::Vector3f::Zero();
            fusion_state->acceleration = Eigen::Vector3f::Zero();
            fusion_state->accelerometer_derivative = Eigen::Vector3f::Zero();

            // Fusion state is no longer valid
            fusion_state->bIsValid = false;
        }
    }
    else if (filter_packet->position_quality > 0.f && eigen_vector3f_is_valid(filter_packet->position))
    {
        // If this is the first filter packet, just accept the position and accelerometer as gospel
        fusion_state->position = filter_packet->position;
        fusion_state->velocity = Eigen::Vector3f::Zero();
        fusion_state->acceleration = Eigen::Vector3f::Zero();
        fusion_state->accelerometer = filter_packet->world_accelerometer;
        fusion_state->accelerometer_derivative = Eigen::Vector3f::Zero();

        // Fusion state is valid now that we have one sample
        fusion_state->bIsValid = true;
    }
}

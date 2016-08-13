// -- includes -----
#include "PositionFilter.h"
#include "MathEigen.h"
#include "ServerLog.h"
#include <deque>

//-- constants -----
// Max length of the position history we keep
#define k_position_history_max 16

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

// -- private definitions -----
struct PositionFilterState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Is the current state valid
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

    void reset()
    {
        bIsValid = false;
        position = Eigen::Vector3f::Zero();
        velocity = Eigen::Vector3f::Zero();
        acceleration = Eigen::Vector3f::Zero();
        accelerometer = Eigen::Vector3f::Zero();
        accelerometer_derivative = Eigen::Vector3f::Zero();
        origin_position = Eigen::Vector3f::Zero();
    }

	void apply_state(
		const Eigen::Vector3f &new_position,
		const Eigen::Vector3f &new_velocity,
		const Eigen::Vector3f &new_acceleration,
		const Eigen::Vector3f &new_accelerometer,
		const Eigen::Vector3f &new_accelerometer_derivative)
	{
		if (eigen_vector3f_is_valid(new_position))
		{
			position = new_position;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "Position is NaN!" << std::endl;
		}

		if (eigen_vector3f_is_valid(new_velocity))
		{
			velocity = new_velocity;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "Velocity is NaN!" << std::endl;
		}

		if (eigen_vector3f_is_valid(new_acceleration))
		{
			acceleration = new_acceleration;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "Acceleration is NaN!" << std::endl;
		}

		if (eigen_vector3f_is_valid(new_accelerometer))
		{
			accelerometer = new_accelerometer;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "Accelerometer is NaN!" << std::endl;
		}

		if (eigen_vector3f_is_valid(new_accelerometer_derivative))
		{
			accelerometer_derivative = new_accelerometer_derivative;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "AccelerometerDerivative is NaN!" << std::endl;
		}

        // state is valid now that we have had an update
        bIsValid= true;
	}
};

// -- private methods -----
static Eigen::Vector3f threshold_vector3f(const Eigen::Vector3f &vector, const float min_length);
static Eigen::Vector3f clamp_vector3f(const Eigen::Vector3f &vector, const float max_length);
static Eigen::Vector3f lowpass_filter_vector3f(
    const float alpha,
    const Eigen::Vector3f &old_filtered_vector,
    const Eigen::Vector3f &new_vector);
static Eigen::Vector3f lowpass_filter_vector3f(
    const float delta_time,
    const float cutoff_frequency,
    const Eigen::Vector3f &old_filtered_vector,
    const Eigen::Vector3f &new_vector);
static Eigen::Vector3f lowpass_filter_optical_position(
    const PoseFilterPacket *filter_packet,
    const PositionFilterState *fusion_state);
static void lowpass_filter_imu_step(
    const float delta_time,
    const PositionFilterConstants *filter_constants,
    const PoseFilterPacket *filter_packet,
    const PositionFilterState *old_state,
    PositionFilterState *new_state);

// -- public interface -----
//-- Orientation Filter -----
PositionFilter::PositionFilter()
    : m_state(new PositionFilterState)
{
    memset(&m_constants, 0, sizeof(PositionFilterConstants));
    resetState();
}

PositionFilter::~PositionFilter()
{
    delete m_state;
}

bool PositionFilter::getIsStateValid() const
{
    return m_state->bIsValid;
}

void PositionFilter::resetState()
{
    m_state->reset();
}

void PositionFilter::recenterState()
{
    m_state->origin_position= m_state->position;
}

bool PositionFilter::init(const PositionFilterConstants &constants)
{
    resetState();
    m_constants= constants;

    return true;
}

Eigen::Vector3f PositionFilter::getPosition(float time) const
{
    Eigen::Vector3f result = Eigen::Vector3f::Zero();

    if (m_state->bIsValid)
    {
        Eigen::Vector3f predicted_position = 
            is_nearly_zero(time)
            ? m_state->position
            : m_state->position + m_state->velocity * time;

        result= predicted_position - m_state->origin_position;
        result= result * k_meters_to_centimeters;
    }

    return result;
}

Eigen::Vector3f PositionFilter::getVelocity() const
{
    Eigen::Vector3f result= m_state->velocity * k_meters_to_centimeters;

    return (m_state->bIsValid) ? result : Eigen::Vector3f::Zero();
}

Eigen::Vector3f PositionFilter::getAcceleration() const
{
    Eigen::Vector3f result= m_state->acceleration * k_meters_to_centimeters;

    return (m_state->bIsValid) ? result : Eigen::Vector3f::Zero();
}

// -- Position Filters ----
// -- PositionFilterPassThru --
void PositionFilterPassThru::update(
	const float delta_time, 
	const PoseFilterPacket &packet)
{
	// Use the current position if the optical orientation is unavailable
    const Eigen::Vector3f &new_position= 
		(packet.optical_position_quality > 0.f) 
		? packet.optical_position
		: packet.current_position;

    // If this is the first filter packet, just accept the current position as gospel
	m_state->apply_state(
		new_position,
        //###HipsterSloth $TODO derivatives of source signal, even a low pass filtered one, is too noisy
		Eigen::Vector3f::Zero(), 
		Eigen::Vector3f::Zero(),
		packet.world_accelerometer, 
		Eigen::Vector3f::Zero());
}

// -- PositionFilterLowPassOptical --
void PositionFilterLowPassOptical::update(
	const float delta_time, 
	const PoseFilterPacket &packet)
{
    if (packet.optical_position_quality > 0.f && eigen_vector3f_is_valid(packet.optical_position))
    {        
		PositionFilterState new_state;

        new_state.accelerometer= packet.world_accelerometer;
        new_state.accelerometer_derivative= Eigen::Vector3f::Zero();

        if (m_state->bIsValid)
        {
            // New position is blended against the old position
            new_state.position = lowpass_filter_optical_position(&packet, m_state);
            //###HipsterSloth $TODO derivatives of source signal, even a low pass filtered one, is too noisy
            new_state.velocity = Eigen::Vector3f::Zero(); // new_velocity;
            new_state.acceleration = Eigen::Vector3f::Zero(); //new_acceleration;
        }
        else
        {
            // If this is the first filter packet, just accept the position as gospel
            new_state.position = packet.optical_position;
            new_state.velocity = Eigen::Vector3f::Zero();
            new_state.acceleration = Eigen::Vector3f::Zero();
        }

		m_state->apply_state(
			new_state.position,
			new_state.velocity,
			new_state.acceleration,
			new_state.accelerometer, 
			new_state.accelerometer_derivative);
    }
	else
	{
		m_state->reset();
	}
}

// -- PositionFilterLowPassIMU --
void PositionFilterLowPassIMU::update(
	const float delta_time,
	const PoseFilterPacket &packet)
{
    if (eigen_vector3f_is_valid(packet.imu_accelerometer))
    {
        if (m_state->bIsValid)
        {
			PositionFilterState new_state;

            lowpass_filter_imu_step(
                delta_time,
                &m_constants,
                &packet,
                m_state,
                &new_state);

			m_state->apply_state(
				new_state.position,
				new_state.velocity,
				new_state.acceleration,
				new_state.accelerometer, 
				new_state.accelerometer_derivative);
        }
        else
        {
            // If this is the first filter packet, just accept the current position as gospel
			m_state->apply_state(
				packet.current_position,
				Eigen::Vector3f::Zero(),
				Eigen::Vector3f::Zero(),
				packet.world_accelerometer, 
				Eigen::Vector3f::Zero());
        }
    }
}

// -- PositionFilterComplimentaryOpticalIMU --
void PositionFilterComplimentaryOpticalIMU::resetState()
{
    bLast_visible_position_timestamp_valid= false;
}

void PositionFilterComplimentaryOpticalIMU::update(const float delta_time, const PoseFilterPacket &packet)
{
    if (packet.optical_position_quality > 0)
    {
        last_visible_position_timestamp= std::chrono::high_resolution_clock::now();
        bLast_visible_position_timestamp_valid= true;
    }

    if (m_state->bIsValid)
    {
        bool bValidRecentPosition= false;

        // Make sure it hasn't been too long since we've last seen the controller
        // IMU integration will get bad pretty quickly
        if (bLast_visible_position_timestamp_valid)
        {
            const std::chrono::duration<float, std::milli> time_delta =
                std::chrono::high_resolution_clock::now() - last_visible_position_timestamp;
            const float time_delta_milli = time_delta.count();

            static float g_max_unseen_position_timeout= k_max_unseen_position_timeout;
            bValidRecentPosition= time_delta_milli < g_max_unseen_position_timeout;
        }

        if (bValidRecentPosition)
        {
            PositionFilterState new_imu_state;
			Eigen::Vector3f new_position;

            // Compute the new filter state based on the previous filter state and new sensor data
            lowpass_filter_imu_step(
                delta_time,
                &m_constants,
                &packet,
                m_state,
                &new_imu_state);

            if (packet.optical_position_quality > 0)
            {
				const Eigen::Vector3f &imu_position= new_imu_state.position;

                // Compute a low-pass filter on the optical position update
                Eigen::Vector3f optical_position = lowpass_filter_optical_position(&packet, m_state);

                // Blend the optical and IMU derived positions based on optical tracking quality
                const float optical_weight= packet.optical_position_quality;
                new_position= optical_weight*optical_position + (1.f - optical_weight)*imu_position;
            }
            else
            {
                new_position= new_imu_state.position;
            }

			m_state->apply_state(
				new_position,
				new_imu_state.velocity,
				new_imu_state.acceleration,
				new_imu_state.accelerometer, 
				new_imu_state.accelerometer_derivative);
        }
        else
        {
            // Zero out the derived state, but leave the sensor state and position state alone
            m_state->velocity = Eigen::Vector3f::Zero();
            m_state->acceleration = Eigen::Vector3f::Zero();
            m_state->accelerometer_derivative = Eigen::Vector3f::Zero();

            // Fusion state is no longer valid
            m_state->bIsValid = false;
        }
    }
    else if (packet.optical_position_quality > 0.f && eigen_vector3f_is_valid(packet.optical_position))
    {
        // If this is the first filter packet, just accept the position and accelerometer as gospel
		m_state->apply_state(
			packet.optical_position,
			Eigen::Vector3f::Zero(),
			Eigen::Vector3f::Zero(),
			packet.world_accelerometer,
			Eigen::Vector3f::Zero());
    }
}

//-- helper functions ---
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

static Eigen::Vector3f lowpass_filter_optical_position(
    const PoseFilterPacket *packet,
    const PositionFilterState *state)
{
    assert(state->bIsValid);
	assert(packet->optical_position_quality > 0.f);

    // Traveling k_max_lowpass_smoothing_distance in one frame should have 0 smoothing
    // Traveling 0+noise cm in one frame should have 60% smoothing
    Eigen::Vector3f diff = packet->optical_position - state->position;
    float distance = diff.norm();
    float new_position_weight = clampf01(lerpf(0.40f, 1.00f, distance / k_max_lowpass_smoothing_distance));

    // New position is blended against the old position
    const Eigen::Vector3f &old_position = state->position;
    const Eigen::Vector3f &new_position = packet->optical_position;
    const Eigen::Vector3f filtered_new_position = lowpass_filter_vector3f(new_position_weight, old_position, new_position);

    return filtered_new_position;
}

static void lowpass_filter_imu_step(
    const float delta_time,
    const PositionFilterConstants *filter_constants,
    const PoseFilterPacket *filter_packet,
    const PositionFilterState *old_state,
    PositionFilterState *new_state)
{
    assert(old_state->bIsValid);

    if (delta_time > k_real_epsilon)
    {
        // Gather sensor state from the previous frame
        const Eigen::Vector3f &old_accelerometer= old_state->accelerometer;
        const Eigen::Vector3f &old_accelerometer_derivative= old_state->accelerometer_derivative;

        // Gather physics state from the previous frame
        const Eigen::Vector3f &old_position = old_state->position;
        const Eigen::Vector3f &old_velocity = old_state->velocity;
        const Eigen::Vector3f &old_acceleration = old_state->acceleration;

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
                threshold_vector3f(accelerometer_derivative, filter_constants->accelerometer_noise_radius);

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
            clamp_vector3f(new_unclamped_velocity, filter_constants->max_velocity);
        
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
        new_state->accelerometer= new_accelerometer;
        new_state->accelerometer_derivative= new_accelerometer_derivative;

        // Save out the updated fusion state
        new_state->acceleration = new_acceleration;
        new_state->velocity = new_velocity;
        new_state->position= new_position;
    }
    else
    {
        *new_state= *old_state;
    }
}
//-- includes --
#include "KalmanPositionFilter.h"
#include "MathAlignment.h"

#include <kalman/MeasurementModel.hpp>
#include <kalman/SystemModel.hpp>
#include <kalman/SquareRootBase.hpp>
#include <kalman/SquareRootUnscentedKalmanFilter.hpp>

// The kalman filter runs way to slow in a fully unoptimized build.
// But if we just unoptimize this file in a release build it seems to run ok.
#if defined(_MSC_VER) && defined(UNOPTIMIZE_KALMAN_FILTERS)
#pragma optimize( "", off )
#endif

//-- constants --
enum PositionFilterStateEnum
{
    POSITION_X, // meters
    LINEAR_VELOCITY_X, // meters / s
    LINEAR_ACCELERATION_X, // meters /s^2
    POSITION_Y,
    LINEAR_VELOCITY_Y,
    LINEAR_ACCELERATION_Y,
    POSITION_Z,
    LINEAR_VELOCITY_Z,
    LINEAR_ACCELERATION_Z,

    STATE_PARAMETER_COUNT
};

enum PositionFilterMeasurementEnum {
    ACCELEROMETER_X, // gravity units
    ACCELEROMETER_Y,
    ACCELEROMETER_Z,
	OPTICAL_POSITION_X,
	OPTICAL_POSITION_Y,
	OPTICAL_POSITION_Z,

    MEASUREMENT_PARAMETER_COUNT
};

// From: http://nbviewer.jupyter.org/github/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/10-Unscented-Kalman-Filter.ipynb#Reasonable-Choices-for-the-Parameters
// beta=2 is a good choice for Gaussian problems, 
// kappa=3-n where n is the size of x is a good choice for kappa, 
// 0<=alpha<=1 is an appropriate choice for alpha, 
// where a larger value for alpha spreads the sigma points further from the mean.
#define k_ukf_alpha 0.6
#define k_ukf_beta 2.0
#define k_ukf_kappa -6.0 // 3 - STATE_PARAMETER_COUNT

// Arbitrary tuning scale applied to the measurement noise
#define R_ACCELEROMETER_SCALE 1.0

// Arbitrary tuning scale applied to the measurement noise
#define R_POSITION_SCALE 1.0

// Arbitrary tuning scale applied to the process noise
#define Q_SCALE 1.0

//-- private methods ---
template <class StateType>
void Q_discrete_3rd_order_white_noise(const double dT, const double var, const int state_index, Kalman::Covariance<StateType> &Q);

//-- private definitions --
template<typename T>
class PositionStateVector : public Kalman::Vector<T, STATE_PARAMETER_COUNT>
{
public:
    KALMAN_VECTOR(PositionStateVector, T, STATE_PARAMETER_COUNT)

    // Accessors
    Eigen::Vector3d get_position_meters() const { 
        return Eigen::Vector3d((*this)[POSITION_X], (*this)[POSITION_Y], (*this)[POSITION_Z]); 
    }
    Eigen::Vector3d get_linear_velocity_m_per_sec() const {
        return Eigen::Vector3d((*this)[LINEAR_VELOCITY_X], (*this)[LINEAR_VELOCITY_Y], (*this)[LINEAR_VELOCITY_Z]);
    }
    Eigen::Vector3d get_linear_acceleration_m_per_sec_sqr() const {
        return Eigen::Vector3d((*this)[LINEAR_ACCELERATION_X], (*this)[LINEAR_ACCELERATION_Y], (*this)[LINEAR_ACCELERATION_Z]);
    }

    // Mutators
    void set_position_meters(const Eigen::Vector3d &p) {
        (*this)[POSITION_X] = p.x(); (*this)[POSITION_Y] = p.y(); (*this)[POSITION_Z] = p.z();
    }
    void set_linear_velocity_m_per_sec(const Eigen::Vector3d &v) {
        (*this)[LINEAR_VELOCITY_X] = v.x(); (*this)[LINEAR_VELOCITY_Y] = v.y(); (*this)[LINEAR_VELOCITY_Z] = v.z();
    }
    void set_linear_acceleration_m_per_sec_sqr(const Eigen::Vector3d &a) {
        (*this)[LINEAR_ACCELERATION_X] = a.x(); (*this)[LINEAR_ACCELERATION_Y] = a.y(); (*this)[LINEAR_ACCELERATION_Z] = a.z();
    }
};
typedef PositionStateVector<double> PositionStateVectord;

template<typename T>
class PositionMeasurementVector : public Kalman::Vector<T, MEASUREMENT_PARAMETER_COUNT>
{
public:
    KALMAN_VECTOR(PositionMeasurementVector, T, MEASUREMENT_PARAMETER_COUNT)

    // Accessors
    Eigen::Vector3d get_accelerometer_g_units() const {
        return Eigen::Vector3d((*this)[ACCELEROMETER_X], (*this)[ACCELEROMETER_Y], (*this)[ACCELEROMETER_Z]);
    }
    Eigen::Vector3d get_optical_position_meters() const {
        return Eigen::Vector3d((*this)[OPTICAL_POSITION_X], (*this)[OPTICAL_POSITION_Y], (*this)[OPTICAL_POSITION_Z]);
    }

    // Mutators
    void set_accelerometer_g_units(const Eigen::Vector3d &a) {
        (*this)[ACCELEROMETER_X] = a.x(); (*this)[ACCELEROMETER_Y] = a.y(); (*this)[ACCELEROMETER_Z] = a.z();
    }
    void set_optical_position_meters(const Eigen::Vector3d &p) {
        (*this)[OPTICAL_POSITION_X] = p.x(); (*this)[OPTICAL_POSITION_Y] = p.y(); (*this)[OPTICAL_POSITION_Z] = p.z();
    }
};
typedef PositionMeasurementVector<double> PositionMeasurementVectord;

/**
* @brief System model for a controller
*
* This is the system model defining how a controller advances from one
* time-step to the next, i.e. how the system state evolves over time.
*/
class PositionSystemModel : public Kalman::SystemModel<PositionStateVectord, Kalman::Vector<double, 0>, Kalman::SquareRootBase>
{
public:
    inline void set_time_step(const double dt) { m_time_step = dt; }

    void init(const PositionFilterConstants &constants)
    {
		use_linear_acceleration = constants.use_linear_acceleration;
		m_last_tracking_projection_area_px_sqr = -1.f;
		update_process_noise(constants, 0.f);
    }

	void update_process_noise(const PositionFilterConstants &constants, const float tracking_projection_area_px_sqr)
	{
		// Only update the covariance when there is more than a 10px change in position quality
		if (m_last_tracking_projection_area_px_sqr < 0.f ||
			!is_nearly_equal(tracking_projection_area_px_sqr, m_last_tracking_projection_area_px_sqr, 10.f))
		{
			const double mean_position_dT = constants.mean_update_time_delta;
			const double mean_orientation_dT = constants.mean_update_time_delta;

			// Start off using the maximum variance values
			static double q_scale = Q_SCALE;
			const double position_variance_cm_sqr = static_cast<double>(constants.position_variance_curve.evaluate(tracking_projection_area_px_sqr));
			// variance_meters = variance_cm * (0.01)^2 because ...
			// var(k*x) = sum(k*x_i - k*mu)^2/(N-1) = k^2*sum(x_i - mu)^2/(N-1)
			// where k = k_centimeters_to_meters = 0.01
			const double position_variance_m_sqr = k_centimeters_to_meters*k_centimeters_to_meters*position_variance_cm_sqr;

			// Initialize the process covariance matrix Q
			Kalman::Covariance<PositionStateVectord> Q = Kalman::Covariance<PositionStateVectord>::Zero();
			Q_discrete_3rd_order_white_noise<PositionStateVectord>(mean_position_dT, q_scale*position_variance_m_sqr, POSITION_X, Q);
			Q_discrete_3rd_order_white_noise<PositionStateVectord>(mean_position_dT, q_scale*position_variance_m_sqr, POSITION_Y, Q);
			Q_discrete_3rd_order_white_noise<PositionStateVectord>(mean_position_dT, q_scale*position_variance_m_sqr, POSITION_Z, Q);
			setCovariance(Q);

			// Keep track last tracking projection area we built the covariance matrix for
			m_last_tracking_projection_area_px_sqr = tracking_projection_area_px_sqr;
		}
	}

    /**
    * @brief Definition of (non-linear) state transition function
    *
    * This function defines how the system state is propagated through time,
    * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to
    * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
    * the system control input \f$u\f$.
    *
    * @param [in] x The system state in current time-step
    * @param [in] u The control vector input
    * @returns The (predicted) system state in the next time-step
    */
    PositionStateVectord f(const PositionStateVectord& old_state, const Kalman::Vector<double, 0>& control) const
    {
        //! Predicted state vector after transition
        PositionStateVectord new_state;

        // Extract parameters from the old state
        const Eigen::Vector3d old_position_meters = old_state.get_position_meters();
        const Eigen::Vector3d old_linear_velocity_m_per_sec = old_state.get_linear_velocity_m_per_sec();
        const Eigen::Vector3d old_linear_acceleration_m_per_sec_sqr = old_state.get_linear_acceleration_m_per_sec_sqr();

        // Compute the position state update
		Eigen::Vector3d new_position_meters;
		Eigen::Vector3d new_linear_velocity_m_per_sec;
		if (use_linear_acceleration)
		{
			new_position_meters =
				old_position_meters
				+ old_linear_velocity_m_per_sec*m_time_step
				+ old_linear_acceleration_m_per_sec_sqr*m_time_step*m_time_step*0.5f;			
			new_linear_velocity_m_per_sec = 
				old_linear_velocity_m_per_sec 
				+ old_linear_acceleration_m_per_sec_sqr*m_time_step;
		}
		else
		{
			new_position_meters =
				old_position_meters
				+ old_linear_velocity_m_per_sec*m_time_step;			
			new_linear_velocity_m_per_sec =
				(m_time_step > k_real64_normal_epsilon)
				? (new_position_meters - old_position_meters) / m_time_step
				: old_linear_velocity_m_per_sec;
		}

		const Eigen::Vector3d &new_linear_acceleration_m_per_sec_sqr = old_linear_acceleration_m_per_sec_sqr;

        // Save results to the new state
        new_state.set_position_meters(new_position_meters);
        new_state.set_linear_velocity_m_per_sec(new_linear_velocity_m_per_sec);
        new_state.set_linear_acceleration_m_per_sec_sqr(new_linear_acceleration_m_per_sec_sqr);

        return new_state;
    }

protected:
	bool use_linear_acceleration;
	float m_last_tracking_projection_area_px_sqr;
    double m_time_step;
};

class PositionSRUKF : public Kalman::SquareRootUnscentedKalmanFilter<PositionStateVectord>
{
public:
	PositionSRUKF(double alpha = 1.0, double beta = 2.0, double kappa = 0.0)
		: Kalman::SquareRootUnscentedKalmanFilter<PositionStateVectord>(alpha, beta, kappa)
	{
	}

	State& getStateMutable()
	{
		return x;
	}
};

/**
* @brief Measurement model for measuring PSMove controller
*
* This is the measurement model for measuring the position and magnetometer of the PSMove controller.
* The measurement is given by the optical trackers.
*/
class PositionMeasurementModel : public Kalman::MeasurementModel<PositionStateVectord, PositionMeasurementVectord, Kalman::SquareRootBase>
{
public:
    void init(const PositionFilterConstants &constants)
    {
		m_last_tracking_projection_area_px_sqr = -1.f;
		update_measurement_covariance(constants, 0.f);
        
		m_identity_gravity_direction= constants.gravity_calibration_direction.cast<double>();
    }

	inline void set_current_orientation(const Eigen::Quaterniond &orientation)
	{
		m_current_orientation= orientation;
	}

	void update_measurement_covariance(
		const PositionFilterConstants &constants,
		const float tracking_projection_area_px_sqr)
	{
		// Only update the covariance when there is more than a 10px change in position quality
		if (m_last_tracking_projection_area_px_sqr < 0.f ||
			!is_nearly_equal(tracking_projection_area_px_sqr, m_last_tracking_projection_area_px_sqr, 10.f))
		{
			const Eigen::Vector3f &accelerometer_variance = constants.accelerometer_variance;
			const double position_variance_cm_sqr = static_cast<double>(constants.position_variance_curve.evaluate(tracking_projection_area_px_sqr));
			// variance_meters = variance_cm * (0.01)^2 because ...
			// var(k*x) = sum(k*x_i - k*mu)^2/(N-1) = k^2*sum(x_i - mu)^2/(N-1)
			// where k = k_centimeters_to_meters = 0.01
			const double position_variance_m_sqr = k_centimeters_to_meters*k_centimeters_to_meters*position_variance_cm_sqr;

			// Update the measurement covariance R
			static double r_accelometer_scale = R_ACCELEROMETER_SCALE;
			static double r_position_scale = R_POSITION_SCALE;
			Kalman::Covariance<PositionMeasurementVectord> R =
				Kalman::Covariance<PositionMeasurementVectord>::Zero();
			R(ACCELEROMETER_X, ACCELEROMETER_X) = r_accelometer_scale*constants.accelerometer_variance.x();
			R(ACCELEROMETER_Y, ACCELEROMETER_Y) = r_accelometer_scale*constants.accelerometer_variance.y();
			R(ACCELEROMETER_Z, ACCELEROMETER_Z) = r_accelometer_scale*constants.accelerometer_variance.z();
			R(OPTICAL_POSITION_X, OPTICAL_POSITION_X) = r_position_scale*position_variance_m_sqr;
			R(OPTICAL_POSITION_Y, OPTICAL_POSITION_Y) = r_position_scale*position_variance_m_sqr;
			R(OPTICAL_POSITION_Z, OPTICAL_POSITION_Z) = r_position_scale*position_variance_m_sqr;
			setCovariance(R);

			// Keep track last position quality we built the covariance matrix for
			m_last_tracking_projection_area_px_sqr = tracking_projection_area_px_sqr;
		}
	}

    /**
    * @brief Definition of (possibly non-linear) measurement function
    *
    * This function maps the system state to the measurement that is expected
    * to be received from the sensor assuming the system is currently in the
    * estimated state.
    *
    * @param [in] x The system state in current time-step
    * @returns The (predicted) sensor measurement for the system state
    */
    PositionMeasurementVectord h(const PositionStateVectord& x) const
    {
        PositionMeasurementVectord predicted_measurement;

        // Use the position and orientation from the state for predictions
        const Eigen::Vector3d position_meters= x.get_position_meters();

        // Use the current linear acceleration from the state to predict
        // what the accelerometer reading will be (in world space)
        const Eigen::Vector3d gravity_accel_g_units= m_identity_gravity_direction;
        const Eigen::Vector3d linear_accel_g_units= x.get_linear_acceleration_m_per_sec_sqr() * k_ms2_to_g_units;
        const Eigen::Vector3d accel_world_g_units= linear_accel_g_units + gravity_accel_g_units;

        // Put the accelerometer prediction into the local space of the controller
        const Eigen::Vector3d accel_local_g_units= eigen_vector3d_clockwise_rotate(m_current_orientation, accel_world_g_units);

        // Save the predictions into the measurement vector
        predicted_measurement.set_accelerometer_g_units(accel_local_g_units);
        predicted_measurement.set_optical_position_meters(position_meters);

        return predicted_measurement;
    }

protected:
    Eigen::Vector3d m_identity_gravity_direction;
	Eigen::Quaterniond m_current_orientation;
	float m_last_tracking_projection_area_px_sqr;
};

class KalmanPositionFilterImpl
{
public:
    /// Is the current fusion state valid
    bool bIsValid;

    /// True if we have seen a valid position measurement (>0 position quality)
    bool bSeenPositionMeasurement;

    /// Position that's considered the origin position 
    Eigen::Vector3f origin_position_meters; // meters

    /// Used to model how the physics of the controller evolves
    PositionSystemModel system_model;

	/// Used to project model onto predicted sensor measurements
	PositionMeasurementModel measurement_model;

    /// Unscented Kalman Filter instance
	PositionSRUKF ukf;

    KalmanPositionFilterImpl() 
		: ukf(k_ukf_alpha, k_ukf_beta, k_ukf_kappa)
    {
    }

    virtual void init(const PositionFilterConstants &constants)
    {
        bIsValid = false;
		bSeenPositionMeasurement= false;

        origin_position_meters = Eigen::Vector3f::Zero();

		measurement_model.init(constants);
        system_model.init(constants);
        ukf.init(PositionStateVectord::Zero());
    }

	virtual void init(const PositionFilterConstants &constants, const Eigen::Vector3f &initial_position_meters)
	{
		bIsValid = true;
		bSeenPositionMeasurement = true;

		origin_position_meters = Eigen::Vector3f::Zero();

		PositionStateVectord state_vector = PositionStateVectord::Zero();
		state_vector.set_position_meters(initial_position_meters.cast<double>());

		measurement_model.init(constants);
		system_model.init(constants);
		ukf.init(state_vector);
	}
};

//-- public interface --
//-- KalmanFilterOpticalPoseARG --
KalmanPositionFilter::KalmanPositionFilter() 
    : m_filter(nullptr)
{
    memset(&m_constants, 0, sizeof(PositionFilterConstants));
}

KalmanPositionFilter::~KalmanPositionFilter()
{
    if (m_filter != nullptr)
    {
        delete m_filter;
        m_filter;
    }
}

bool KalmanPositionFilter::init(const PositionFilterConstants &constants)
{
    m_constants = constants;

    // cleanup any existing filter
    if (m_filter != nullptr)
    {
        delete m_filter;
        m_filter;
    }

	// Create and initialize the private filter implementation
    KalmanPositionFilterImpl *filter = new KalmanPositionFilterImpl();
    filter->init(constants);
    m_filter = filter;

    return true;
}

bool KalmanPositionFilter::init(const PositionFilterConstants &constants, const Eigen::Vector3f &initial_position_meters)
{
	m_constants = constants;

	// cleanup any existing filter
	if (m_filter != nullptr)
	{
		delete m_filter;
		m_filter;
	}

	// Create and initialize the private filter implementation
	KalmanPositionFilterImpl *filter = new KalmanPositionFilterImpl();
	filter->init(constants, initial_position_meters);
	m_filter = filter;

	return true;
}

void KalmanPositionFilter::update(const float delta_time, const PoseFilterPacket &packet)
{
    if (m_filter->bIsValid)
    {
		// Adjust the amount we trust the optical state based on the tracking projection area
		m_filter->system_model.update_process_noise(m_constants, packet.tracking_projection_area_px_sqr);

        // Predict state for current time-step using the filters
        m_filter->system_model.set_time_step(delta_time);
        m_filter->ukf.predict(m_filter->system_model);

        // Get the measurement model for the DS4 from the derived filter impl
        PositionMeasurementModel &measurement_model = m_filter->measurement_model;

		// Update the measurement model with the latest controller orientation.
		// This comes from the orientation filter, which ran before this filter.
		Eigen::Quaterniond current_orientation= packet.current_orientation.cast<double>();
		measurement_model.set_current_orientation(current_orientation);

        // Project the current state onto a predicted measurement as a default
        // in case no observation is available
        PositionMeasurementVectord measurement = measurement_model.h(m_filter->ukf.getState());

		Eigen::Vector3d accelerometer = packet.imu_accelerometer_g_units.cast<double>();

		// Hacks to deal with accelerometer measurement issues
		if (m_constants.apply_gravity_mask)
		{			
			const double acc_magnitude = accelerometer.norm();
			const double bias_tolerance = 0.1f;
			const double alignment_tolerance = 0.984807753; // cos(10 degrees)

			const Eigen::Vector3d acc_local_g =
				eigen_vector3d_clockwise_rotate(current_orientation, m_constants.gravity_calibration_direction.cast<double>());

			//###HipsterSloth $TODO If there is any error in our orientation we don't correctly subtract
			// out the effect of gravity. This manifests as a phantom lateral linear acceleration 
			// that drives the velocity in the direction of the bias.
			// As a work around, we snap the accelerometer direction to the predicted gravity direction
			// if it's within an angular tolerance of the predicted gravity direction
			Eigen::Vector3d acc_normalized = accelerometer / acc_magnitude;
			if (acc_normalized.dot(acc_local_g) >= alignment_tolerance)
			{
				accelerometer = acc_local_g * acc_magnitude;
				acc_normalized = acc_local_g;
			}

			//###HipsterSloth $TODO If there is any bias in our accelerometer magnitude it manifests
			// as a phantom linear acceleration that drives the velocity in the direction of the bias.
			// As a work around, we normalize the accelerometer reading for any accelerometer magnitudes
			// within a tolerance of 1.0
			if (fabs(acc_magnitude - 1.0) < bias_tolerance)
			{
				accelerometer = acc_normalized;
			}
		}

		// Accelerometer and gyroscope measurements are always available
		measurement.set_accelerometer_g_units(accelerometer);

		// Adjust the amount we trust the optical measurements based on the tracking projection area
		measurement_model.update_measurement_covariance(
			m_constants,
			packet.tracking_projection_area_px_sqr);

        if (packet.tracking_projection_area_px_sqr > 0.f)
        {
			Eigen::Vector3f optical_position_meters= packet.get_optical_position_in_meters();

            // State internally stores position in meters
            measurement.set_optical_position_meters(optical_position_meters.cast<double>());

			// If this is the first time we have seen the position, snap the position state
			if (!m_filter->bSeenPositionMeasurement)
			{
				m_filter->ukf.getStateMutable().set_position_meters(optical_position_meters.cast<double>());
				m_filter->bSeenPositionMeasurement= true;
			}
        }

        // Update UKF
        m_filter->ukf.update(measurement_model, measurement);
    }
    else
    {
		PositionStateVectord state_vector = PositionStateVectord::Zero();

		if (packet.tracking_projection_area_px_sqr > 0.f)
		{
			Eigen::Vector3f optical_position_meters= packet.get_optical_position_in_meters();

			state_vector.set_position_meters(optical_position_meters.cast<double>());
			m_filter->bSeenPositionMeasurement= true;
		}

		m_filter->ukf.init(state_vector);
        m_filter->bIsValid= true;
    }
}

bool KalmanPositionFilter::getIsStateValid() const
{
    return m_filter->bIsValid;
}

void KalmanPositionFilter::resetState()
{
    m_filter->init(m_constants);
}

void KalmanPositionFilter::recenterOrientation(const Eigen::Quaternionf& q_pose)
{
}

Eigen::Vector3f KalmanPositionFilter::getPositionCm(float time) const
{
    Eigen::Vector3f result = Eigen::Vector3f::Zero();

    if (m_filter->bIsValid)
    {
        Eigen::Vector3f state_position_meters= m_filter->ukf.getState().get_position_meters().cast<float>();
		Eigen::Vector3f state_velocity_m_per_sec = m_filter->ukf.getState().get_linear_velocity_m_per_sec().cast<float>();
        Eigen::Vector3f predicted_position =
            is_nearly_zero(time)
            ? state_position_meters
            : state_position_meters + state_velocity_m_per_sec * time;

        result = (predicted_position - m_filter->origin_position_meters) * k_meters_to_centimeters;
    }

    return result;
}

Eigen::Vector3f KalmanPositionFilter::getVelocityCmPerSec() const
{
	Eigen::Vector3d vel= m_filter->ukf.getState().get_linear_velocity_m_per_sec() * k_meters_to_centimeters;

    return vel.cast<float>();
}

Eigen::Vector3f KalmanPositionFilter::getAccelerationCmPerSecSqr() const
{
    Eigen::Vector3d accel= m_filter->ukf.getState().get_linear_acceleration_m_per_sec_sqr() * k_meters_to_centimeters;

	return accel.cast<float>();
}

//-- Private functions --
// Adapted from: https://github.com/rlabbe/filterpy/blob/master/filterpy/common/discretization.py#L55-L57

// Returns the Q matrix for the Discrete Constant White Noise
// - dT is the time step
// - var is the variance in the process noise
// - state_index denotes where in Q the 3x3 covariance matrix should be written

// Q is computed as the G * G^T * variance, where G is the process noise per time step.
// In other words, G = [[.5dt^2][dt][1]] ^ T for the constant linear acceleration model.
template <class StateType>
void Q_discrete_3rd_order_white_noise(
    const double dT,
    const double var,
    const int state_index,
    Kalman::Covariance<StateType> &Q)
{
    const double dT_2 = dT*dT;
	const double dT_3 = dT_2*dT;
	const double dT_4 = dT_2*dT_2;

    const double q4 = var * dT_4;
    const double q3 = var * dT_3;
	const double q2 = var * dT_2;
	const double q1 = var * dT;

    const int &i= state_index;
    Q(i+0,i+0) = 0.25*q4; Q(i+0,i+1) = 0.5*q3; Q(i+0,i+2) = 0.5*q2;
    Q(i+1,i+0) =  0.5*q3; Q(i+1,i+1) =     q2; Q(i+1,i+2) =     q1;
    Q(i+2,i+0) =  0.5*q2; Q(i+2,i+1) =     q1; Q(i+2,i+2) =    1.0;
}
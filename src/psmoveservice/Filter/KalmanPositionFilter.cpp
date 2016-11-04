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

//-- private methods ---
template <class StateType>
void process_3rd_order_noise(const double dT, const double var, const int state_index, Kalman::Covariance<StateType> &Q);

template <class StateType>
void process_2nd_order_noise(const double dT, const double var, const int state_index, Kalman::Covariance<StateType> &Q);

//-- private definitions --
template<typename T>
class PositionStateVector : public Kalman::Vector<T, STATE_PARAMETER_COUNT>
{
public:
    KALMAN_VECTOR(PositionStateVector, T, STATE_PARAMETER_COUNT)

    // Accessors
    Eigen::Vector3d get_position() const { 
        return Eigen::Vector3d((*this)[POSITION_X], (*this)[POSITION_Y], (*this)[POSITION_Z]); 
    }
    Eigen::Vector3d get_linear_velocity() const {
        return Eigen::Vector3d((*this)[LINEAR_VELOCITY_X], (*this)[LINEAR_VELOCITY_Y], (*this)[LINEAR_VELOCITY_Z]);
    }
    Eigen::Vector3d get_linear_acceleration() const {
        return Eigen::Vector3d((*this)[LINEAR_ACCELERATION_X], (*this)[LINEAR_ACCELERATION_Y], (*this)[LINEAR_ACCELERATION_Z]);
    }

    // Mutators
    void set_position(const Eigen::Vector3d &p) {
        (*this)[POSITION_X] = p.x(); (*this)[POSITION_Y] = p.y(); (*this)[POSITION_Z] = p.z();
    }
    void set_linear_velocity(const Eigen::Vector3d &v) {
        (*this)[LINEAR_VELOCITY_X] = v.x(); (*this)[LINEAR_VELOCITY_Y] = v.y(); (*this)[LINEAR_VELOCITY_Z] = v.z();
    }
    void set_linear_acceleration(const Eigen::Vector3d &a) {
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
    Eigen::Vector3d get_accelerometer() const {
        return Eigen::Vector3d((*this)[ACCELEROMETER_X], (*this)[ACCELEROMETER_Y], (*this)[ACCELEROMETER_Z]);
    }
    Eigen::Vector3d get_optical_position() const {
        return Eigen::Vector3d((*this)[OPTICAL_POSITION_X], (*this)[OPTICAL_POSITION_Y], (*this)[OPTICAL_POSITION_Z]);
    }

    // Mutators
    void set_accelerometer(const Eigen::Vector3d &a) {
        (*this)[ACCELEROMETER_X] = a.x(); (*this)[ACCELEROMETER_Y] = a.y(); (*this)[ACCELEROMETER_Z] = a.z();
    }
    void set_optical_position(const Eigen::Vector3d &p) {
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
        const double mean_position_dT= constants.mean_update_time_delta;
        const double mean_orientation_dT= constants.mean_update_time_delta;

        // Start off using the maximum variance values
        const Eigen::Vector3f position_variance= 
			(constants.min_position_variance +
			constants.max_position_variance) * 0.5f;

        // Initialize the process covariance matrix Q
        Kalman::Covariance<PositionStateVectord> Q = Kalman::Covariance<PositionStateVectord>::Zero();
        process_3rd_order_noise<PositionStateVectord>(mean_position_dT, position_variance.x(), POSITION_X, Q);
		process_3rd_order_noise<PositionStateVectord>(mean_position_dT, position_variance.y(), POSITION_Y, Q);
		process_3rd_order_noise<PositionStateVectord>(mean_position_dT, position_variance.z(), POSITION_Z, Q);
        setCovariance(Q);
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
        const Eigen::Vector3d old_position = old_state.get_position();
        const Eigen::Vector3d old_linear_velocity = old_state.get_linear_velocity();
        const Eigen::Vector3d old_linear_acceleration = old_state.get_linear_acceleration();

        // Compute the position state update
        const Eigen::Vector3d new_position= 
            old_position 
            + old_linear_velocity*m_time_step 
            + old_linear_acceleration*m_time_step*m_time_step*0.5f;
        const Eigen::Vector3d new_linear_velocity= old_linear_velocity + old_linear_acceleration*m_time_step;
        const Eigen::Vector3d &new_linear_acceleration= old_linear_acceleration;

        // Save results to the new state
        new_state.set_position(new_position);
        new_state.set_linear_velocity(new_linear_velocity);
        new_state.set_linear_acceleration(new_linear_acceleration);

        return new_state;
    }

protected:
    double m_time_step;
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
		update_measurement_covariance(constants, 0.f);
        
		m_identity_gravity_direction= constants.gravity_calibration_direction.cast<double>();
    }

	inline void set_current_orientation(const Eigen::Quaterniond &orientation)
	{
		m_current_orientation= orientation;
	}

	void update_measurement_covariance(
		const PositionFilterConstants &constants,
		const float position_quality)
	{
        // Start off using the maximum variance values
		const Eigen::Vector3f &accelerometer_variance= constants.accelerometer_variance;
		const Eigen::Vector3f position_variance =
			position_quality*constants.min_position_variance +
			(1.f - position_quality)*constants.max_position_variance;

        // Update the measurement covariance R
        Kalman::Covariance<PositionMeasurementVectord> R = 
			Kalman::Covariance<PositionMeasurementVectord>::Zero();
		R(ACCELEROMETER_X, ACCELEROMETER_X) = constants.accelerometer_variance.x();
		R(ACCELEROMETER_Y, ACCELEROMETER_Y) = constants.accelerometer_variance.y();
		R(ACCELEROMETER_Z, ACCELEROMETER_Z) = constants.accelerometer_variance.z();
        R(OPTICAL_POSITION_X, OPTICAL_POSITION_X) = position_variance.x();
        R(OPTICAL_POSITION_Y, OPTICAL_POSITION_Y) = position_variance.y();
        R(OPTICAL_POSITION_Z, OPTICAL_POSITION_Z) = position_variance.z();
        setCovariance(R);
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
        const Eigen::Vector3d position= x.get_position();

        // Use the current linear acceleration from the state to predict
        // what the accelerometer reading will be (in world space)
        const Eigen::Vector3d gravity_accel_g_units= -m_identity_gravity_direction;
        const Eigen::Vector3d linear_accel_g_units= x.get_linear_acceleration() * k_ms2_to_g_units;
        const Eigen::Vector3d accel_world= linear_accel_g_units + gravity_accel_g_units;
        const Eigen::Quaterniond accel_world_quat(0.f, accel_world.x(), accel_world.y(), accel_world.z());

        // Put the accelerometer prediction into the local space of the controller
        const Eigen::Vector3d accel_local= m_current_orientation*(accel_world_quat*m_current_orientation.conjugate()).vec();

        // Save the predictions into the measurement vector
        predicted_measurement.set_accelerometer(accel_local);
        predicted_measurement.set_optical_position(position);

        return predicted_measurement;
    }

protected:
    Eigen::Vector3d m_identity_gravity_direction;
	Eigen::Quaterniond m_current_orientation;
};

class KalmanPositionFilterImpl
{
public:
    /// Is the current fusion state valid
    bool bIsValid;

    /// True if we have seen a valid position measurement (>0 position quality)
    bool bSeenPositionMeasurement;

    /// Position that's considered the origin position 
    Eigen::Vector3f origin_position; // meters

    /// All state parameters of the controller
    PositionStateVectord state_vector;

    /// Used to model how the physics of the controller evolves
    PositionSystemModel system_model;

	/// Used to project model onto predicted sensor measurements
	PositionMeasurementModel measurement_model;

    /// Unscented Kalman Filter instance
	Kalman::SquareRootUnscentedKalmanFilter<PositionStateVectord> ukf;

    KalmanPositionFilterImpl() 
		: ukf(k_ukf_alpha, k_ukf_beta, k_ukf_kappa)
    {
    }

    virtual void init(const PositionFilterConstants &constants)
    {
        bIsValid = false;
		bSeenPositionMeasurement= false;

        origin_position = Eigen::Vector3f::Zero();

        state_vector.setZero();

		measurement_model.init(constants);
        system_model.init(constants);
        ukf.init(state_vector);
    }

	virtual void init(const PositionFilterConstants &constants, const Eigen::Vector3f &initial_position)
	{
		bIsValid = true;
		bSeenPositionMeasurement = true;

		origin_position = Eigen::Vector3f::Zero();

		state_vector.setZero();
		state_vector.set_position(initial_position.cast<double>());

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

bool KalmanPositionFilter::init(const PositionFilterConstants &constants, const Eigen::Vector3f &initial_position)
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
	filter->init(constants, initial_position);
	m_filter = filter;

	return true;
}

void KalmanPositionFilter::update(const float delta_time, const PoseFilterPacket &packet)
{
    if (m_filter->bIsValid)
    {
        // Predict state for current time-step using the filters
        m_filter->system_model.set_time_step(delta_time);
        m_filter->state_vector = m_filter->ukf.predict(m_filter->system_model);

        // Get the measurement model for the DS4 from the derived filter impl
        PositionMeasurementModel &measurement_model = m_filter->measurement_model;

		// Update the measurement model with the latest controller orientation.
		// This comes from the orientation filter, which ran before this filter.
		measurement_model.set_current_orientation(packet.current_orientation.cast<double>());

        // Project the current state onto a predicted measurement as a default
        // in case no observation is available
        PositionMeasurementVectord measurement = measurement_model.h(m_filter->state_vector);

        // Accelerometer and gyroscope measurements are always available
        measurement.set_accelerometer(packet.imu_accelerometer.cast<double>());

        if (packet.optical_position_quality > 0.f)
        {
			Eigen::Vector3f optical_position= packet.get_optical_position_in_meters();

			// Adjust the amount we trust the optical measurements based on the quality parameters
            measurement_model.update_measurement_covariance(
				m_constants, 
				packet.optical_position_quality);

            // State internally stores position in meters
            measurement.set_optical_position(optical_position.cast<double>());

			// If this is the first time we have seen the position, snap the position state
			if (!m_filter->bSeenPositionMeasurement)
			{
				m_filter->state_vector.set_position(optical_position.cast<double>());
				m_filter->bSeenPositionMeasurement= true;
			}
        }

        // Update UKF
        m_filter->state_vector = m_filter->ukf.update(measurement_model, measurement);
    }
    else
    {
        m_filter->state_vector.setZero();

		if (packet.optical_position_quality > 0.f)
		{
			Eigen::Vector3f optical_position= packet.get_optical_position_in_meters();

			m_filter->state_vector.set_position(optical_position.cast<double>());
			m_filter->bSeenPositionMeasurement= true;
		}
		else
		{
			m_filter->state_vector.set_position(Eigen::Vector3d::Zero());
		}

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

void KalmanPositionFilter::recenterState(const Eigen::Vector3f& p_pose, const Eigen::Quaternionf& q_pose)
{
    m_filter->origin_position = p_pose + getPosition();
}

Eigen::Vector3f KalmanPositionFilter::getPosition(float time) const
{
    Eigen::Vector3f result = Eigen::Vector3f::Zero();

    if (m_filter->bIsValid)
    {
        Eigen::Vector3f state_position= m_filter->state_vector.get_position().cast<float>();
        Eigen::Vector3f predicted_position =
            is_nearly_zero(time)
            ? state_position
            : state_position + getVelocity() * time;

        result = (predicted_position - m_filter->origin_position) * k_meters_to_centimeters;
    }

    return result;
}

Eigen::Vector3f KalmanPositionFilter::getVelocity() const
{
	Eigen::Vector3d vel= m_filter->state_vector.get_linear_velocity() * k_meters_to_centimeters;

    return vel.cast<float>();
}

Eigen::Vector3f KalmanPositionFilter::getAcceleration() const
{
    Eigen::Vector3d accel= m_filter->state_vector.get_linear_acceleration() * k_meters_to_centimeters;

	return accel.cast<float>();
}

//-- Private functions --
template <class StateType>
void process_3rd_order_noise(
    const double dT,
    const double var,
    const int state_index,
    Kalman::Covariance<StateType> &Q)
{
    const double dT_2 = dT*dT;
	const double dT_3 = dT_2*dT;
	const double dT_4 = dT_2*dT_2;
	const double dT_5 = dT_3*dT_2;
	const double dT_6 = dT_3*dT_3;
	const double dT_7 = dT_4*dT_3;

    const double q7 = var * dT_7;
    const double q6 = var * dT_6;
    const double q5 = var * dT_5;
    const double q4 = var * dT_4;
    const double q3 = var * dT_3;

    const int &i= state_index;
    Q(i+0,i+0) = q7/252.0; Q(i+0,i+1) = q6/72.0; Q(i+0,i+2) = q5/30.0;
    Q(i+1,i+0) = q6/72.0;  Q(i+1,i+1) = q5/20.0; Q(i+1,i+2) = q4/8.0;
    Q(i+2,i+0) = q5/30.0;  Q(i+2,i+1) = q4/8.0;  Q(i+2,i+2) = q3/3.0;
}

template <class StateType>
void process_2nd_order_noise(
	const double dT, 
	const double var, 
	const int state_index, 
	Kalman::Covariance<StateType> &Q)
{
    const double dT_2 = dT*dT;
	const double dT_3 = dT_2*dT;
	const double dT_4 = dT_2*dT_2;
	const double dT_5 = dT_3*dT_2;

    const double q5 = var * dT_5;
    const double q4 = var * dT_4;
    const double q3 = var * dT_3;

    // Q = [.5dt^2, dt]*[.5dt^2, dt]^T * variance
    const int &i= state_index;
    Q(i+0,i+0) = q5/20.0; Q(i+0,i+1) = q4/8.0;
    Q(i+1,i+0) = q4/8.0;  Q(i+1,i+1) = q3/3.0;
}
//-- includes --
#include "KalmanOrientationFilter.h"
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
enum OrientationFilterStateEnum
{
	ERROR_QUATERNION_W,
	ERROR_Q_DOT_W,
    ERROR_QUATERNION_X,
	ERROR_Q_DOT_X,
	ERROR_QUATERNION_Y,
	ERROR_Q_DOT_Y,
	ERROR_QUATERNION_Z,
	ERROR_Q_DOT_Z,

    STATE_PARAMETER_COUNT
};

enum PSMoveMeasurementEnum {
	PSMOVE_ACCELEROMETER_X, // gravity units
	PSMOVE_ACCELEROMETER_Y,
	PSMOVE_ACCELEROMETER_Z,
	PSMOVE_GYROSCOPE_PITCH, // radians/s
	PSMOVE_GYROSCOPE_YAW,
	PSMOVE_GYROSCOPE_ROLL,
	PSMOVE_MAGNETOMETER_X, // unit vector
	PSMOVE_MAGNETOMETER_Y,
	PSMOVE_MAGNETOMETER_Z,

	PSMOVE_MEASUREMENT_PARAMETER_COUNT
};

enum DS4MeasurementEnum {
	DS4_ACCELEROMETER_X, // gravity units
	DS4_ACCELEROMETER_Y,
	DS4_ACCELEROMETER_Z,
	DS4_GYROSCOPE_PITCH, // radians/s
	DS4_GYROSCOPE_YAW,
	DS4_GYROSCOPE_ROLL,
	DS4_OPTICAL_EULER_ANGLE_HEADING, // radians

	DS4_MEASUREMENT_PARAMETER_COUNT
};

// Arbitrary tuning scale applied to the measurement noise
#define R_SCALE 1.0

// Arbitrary tuning scale applied to the process noise
#define Q_SCALE 1.0

// From: http://nbviewer.jupyter.org/github/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/10-Unscented-Kalman-Filter.ipynb#Reasonable-Choices-for-the-Parameters
// beta=2 is a good choice for Gaussian problems, 
// kappa=3-n where n is the size of x is a good choice for kappa, 
// 0<=alpha<=1 is an appropriate choice for alpha, 
// where a larger value for alpha spreads the sigma points further from the mean.
#define k_ukf_alpha 0.6
#define k_ukf_beta 2.0
#define k_ukf_kappa -3.0 // 3 - STATE_PARAMETER_COUNT

//-- private methods ---
template <class StateType>
void Q_discrete_2rd_order_white_noise(const double dT, const double var, const int state_index, Kalman::Covariance<StateType> &Q);

//-- private definitions --
template<typename T>
class OrientationStateVector : public Kalman::Vector<T, STATE_PARAMETER_COUNT>
{
public:
	KALMAN_VECTOR(OrientationStateVector, T, STATE_PARAMETER_COUNT)

	static OrientationStateVector<T> Identity()
	{
		OrientationStateVector<T> result= OrientationStateVector<T>::Zero();

		result[ERROR_QUATERNION_W] = 1.0;

		return result;
	}

    // Accessors
	Eigen::Quaterniond get_error_quaterniond() const {
		return Eigen::Quaterniond((*this)[ERROR_QUATERNION_W], (*this)[ERROR_QUATERNION_X], (*this)[ERROR_QUATERNION_Y], (*this)[ERROR_QUATERNION_Z]);
	}
	Eigen::Quaterniond get_error_quaterniond_dot() const {
		return Eigen::Quaterniond((*this)[ERROR_Q_DOT_W], (*this)[ERROR_Q_DOT_X], (*this)[ERROR_Q_DOT_Y], (*this)[ERROR_Q_DOT_Z]);
	}
    Eigen::Vector3d get_angular_velocity_rad_per_sec() const {
		Eigen::Quaterniond q = get_error_quaterniond();
		Eigen::Quaterniond q_dot= get_error_quaterniond_dot();
		return eigen_quaterniond_derivative_to_angular_velocity(q, q_dot);
    }

    // Mutators
	void set_error_quaterniond(const Eigen::Quaterniond &q) {
		(*this)[ERROR_QUATERNION_W] = q.w();
		(*this)[ERROR_QUATERNION_X] = q.x();
		(*this)[ERROR_QUATERNION_Y] = q.y();
		(*this)[ERROR_QUATERNION_Z] = q.z();
	}
	void set_error_quaterniond_dot(const Eigen::Quaterniond &q_dot) {
		(*this)[ERROR_Q_DOT_W] = q_dot.w();
		(*this)[ERROR_Q_DOT_X] = q_dot.x();
		(*this)[ERROR_Q_DOT_Y] = q_dot.y();
		(*this)[ERROR_Q_DOT_Z] = q_dot.z();
	}
    void set_angular_velocity_rad_per_sec(const Eigen::Vector3d &v) {
		Eigen::Quaterniond q = get_error_quaterniond();
		Eigen::Quaterniond q_dot = eigen_angular_velocity_to_quaterniond_derivative(q, v);
        (*this)[ERROR_Q_DOT_W] = q_dot.w();
		(*this)[ERROR_Q_DOT_X] = q_dot.x(); 
		(*this)[ERROR_Q_DOT_Y] = q_dot.y();
		(*this)[ERROR_Q_DOT_Z] = q_dot.z();
    }
};
typedef OrientationStateVector<double> OrientationStateVectord;

/**
* @brief System model for a controller
*
* This is the system model defining how a controller advances from one
* time-step to the next, i.e. how the system state evolves over time.
*/
class OrientationSystemModel : public Kalman::SystemModel<OrientationStateVectord, Kalman::Vector<double, 0>, Kalman::SquareRootBase>
{
public:
	inline void set_time_step(const double dt) { m_time_step = dt; }

	void init(const OrientationFilterConstants &constants)
	{
		m_last_tracking_projection_area = -1.f;
		update_process_noise(constants, 0.f);
	}

	void update_process_noise(const OrientationFilterConstants &constants, float tracking_projection_area)
	{
		// Only update the covariance when there is more than a 10px change in position quality
		if (m_last_tracking_projection_area < 0.f || 
			!is_nearly_equal(tracking_projection_area, m_last_tracking_projection_area, 10.f))
		{
			const double mean_orientation_dT = constants.mean_update_time_delta;

			// Start off using the maximum variance values
			static float q_scale = Q_SCALE;
			float orientation_variance = q_scale * constants.orientation_variance_curve.evaluate(tracking_projection_area);

			// Initialize the process covariance matrix Q
			Kalman::Covariance<OrientationStateVectord> Q = Kalman::Covariance<OrientationStateVectord>::Zero();
			Q_discrete_2rd_order_white_noise<OrientationStateVectord>(mean_orientation_dT, orientation_variance, ERROR_QUATERNION_W, Q);
			Q_discrete_2rd_order_white_noise<OrientationStateVectord>(mean_orientation_dT, orientation_variance, ERROR_QUATERNION_X, Q);
			Q_discrete_2rd_order_white_noise<OrientationStateVectord>(mean_orientation_dT, orientation_variance, ERROR_QUATERNION_Y, Q);
			Q_discrete_2rd_order_white_noise<OrientationStateVectord>(mean_orientation_dT, orientation_variance, ERROR_QUATERNION_Z, Q);
			setCovariance(Q);

			// Keep track last tracking projection area we built the covariance matrix for
			m_last_tracking_projection_area = tracking_projection_area;
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
	OrientationStateVectord f(const OrientationStateVectord& old_state, const Kalman::Vector<double, 0>& control) const
	{
		// Predicted state vector after transition
		OrientationStateVectord new_state;

		// Extract parameters from the old state
		const Eigen::Quaterniond error_q_old = old_state.get_error_quaterniond();
		const Eigen::Quaterniond error_q_dot = old_state.get_error_quaterniond_dot();

		// Compute the quaternion derivative of the current state
		// q_new= q + q_dot*dT
		const Eigen::Quaterniond error_q_step = Eigen::Quaterniond(error_q_dot.coeffs() * m_time_step);
		const Eigen::Quaterniond error_q_new = Eigen::Quaterniond(error_q_old.coeffs() + error_q_step.coeffs());

		// Save results to the new state
		new_state.set_error_quaterniond(error_q_new.normalized());
		new_state.set_error_quaterniond_dot(error_q_dot);

		return new_state;
	}

protected:
	double m_time_step;
	float m_last_tracking_projection_area;
};

class OrientationSRUKF : public Kalman::SquareRootUnscentedKalmanFilter<OrientationStateVectord>
{
public:
	OrientationSRUKF(double alpha = 1.0, double beta = 2.0, double kappa = 0.0)
		: Kalman::SquareRootUnscentedKalmanFilter<OrientationStateVectord>(alpha, beta, kappa)
	{
	}

	State& getStateMutable()
	{
		return x;
	}
};

template<typename T>
class PSMove_OrientationMeasurementVector : public Kalman::Vector<T, PSMOVE_MEASUREMENT_PARAMETER_COUNT>
{
public:
	KALMAN_VECTOR(PSMove_OrientationMeasurementVector, T, PSMOVE_MEASUREMENT_PARAMETER_COUNT)

	// Accessors
	Eigen::Vector3d get_accelerometer() const {
		return Eigen::Vector3d((*this)[PSMOVE_ACCELEROMETER_X], (*this)[PSMOVE_ACCELEROMETER_Y], (*this)[PSMOVE_ACCELEROMETER_Z]);
	}
	Eigen::Vector3d get_gyroscope() const {
		return Eigen::Vector3d((*this)[PSMOVE_GYROSCOPE_PITCH], (*this)[PSMOVE_GYROSCOPE_YAW], (*this)[PSMOVE_GYROSCOPE_ROLL]);
	}
	Eigen::Vector3d get_magnetometer() const {
		return Eigen::Vector3d((*this)[PSMOVE_MAGNETOMETER_X], (*this)[PSMOVE_MAGNETOMETER_Y], (*this)[PSMOVE_MAGNETOMETER_Z]);
	}

	// Mutators
	void set_accelerometer(const Eigen::Vector3d &a) {
		(*this)[PSMOVE_ACCELEROMETER_X] = a.x(); (*this)[PSMOVE_ACCELEROMETER_Y] = a.y(); (*this)[PSMOVE_ACCELEROMETER_Z] = a.z();
	}
	void set_gyroscope(const Eigen::Vector3d &g) {
		(*this)[PSMOVE_GYROSCOPE_PITCH] = g.x(); (*this)[PSMOVE_GYROSCOPE_YAW] = g.y(); (*this)[PSMOVE_GYROSCOPE_ROLL] = g.z();
	}
	void set_magnetometer(const Eigen::Vector3d &m) {
		(*this)[PSMOVE_MAGNETOMETER_X] = m.x(); (*this)[PSMOVE_MAGNETOMETER_Y] = m.y(); (*this)[PSMOVE_MAGNETOMETER_Z] = m.z();
	}
};
typedef PSMove_OrientationMeasurementVector<double> PSMove_OrientationMeasurementVectord;

template<typename T>
class DS4_OrientationMeasurementVector : public Kalman::Vector<T, DS4_MEASUREMENT_PARAMETER_COUNT>
{
public:
	KALMAN_VECTOR(DS4_OrientationMeasurementVector, T, DS4_MEASUREMENT_PARAMETER_COUNT)

	// Accessors
	Eigen::Vector3d get_accelerometer() const {
		return Eigen::Vector3d((*this)[DS4_ACCELEROMETER_X], (*this)[DS4_ACCELEROMETER_Y], (*this)[DS4_ACCELEROMETER_Z]);
	}
	Eigen::Vector3d get_gyroscope() const {
		return Eigen::Vector3d((*this)[DS4_GYROSCOPE_PITCH], (*this)[DS4_GYROSCOPE_YAW], (*this)[DS4_GYROSCOPE_ROLL]);
	}
	Eigen::Vector3d get_optical_euler_heading_angle() const {
		return (*this)[DS4_OPTICAL_EULER_ANGLE_HEADING];
	}

	// Mutators
	void set_accelerometer(const Eigen::Vector3d &a) {
		(*this)[DS4_ACCELEROMETER_X] = a.x(); (*this)[DS4_ACCELEROMETER_Y] = a.y(); (*this)[DS4_ACCELEROMETER_Z] = a.z();
	}
	void set_gyroscope(const Eigen::Vector3d &g) {
		(*this)[DS4_GYROSCOPE_PITCH] = g.x(); (*this)[DS4_GYROSCOPE_YAW] = g.y(); (*this)[DS4_GYROSCOPE_ROLL] = g.z();
	}
	void set_optical_euler_heading_angle(const double heading_radians) {
		(*this)[DS4_OPTICAL_EULER_ANGLE_HEADING] = heading_radians;
	}
};
typedef DS4_OrientationMeasurementVector<double> DS4_OrientationMeasurementVectord;

/**
* @brief Orientation Measurement model for measuring PSMove controller
*
* This is the measurement model for measuring the position and magnetometer of the PSMove controller.
* The measurement is given by the optical trackers.
*/
class PSMove_OrientationMeasurementModel : 
	public Kalman::MeasurementModel<OrientationStateVectord, PSMove_OrientationMeasurementVectord, Kalman::SquareRootBase>
{
public:
	void init(const OrientationFilterConstants &constants)
	{
		// Update the measurement covariance R
		Kalman::Covariance<PSMove_OrientationMeasurementVectord> R =
			Kalman::Covariance<PSMove_OrientationMeasurementVectord>::Zero();

		// Only diagonals used so no need to compute Cholesky
		static float r_accelerometer_scale = R_SCALE;
		static float r_gyro_scale = R_SCALE;
		static float r_magnetometer_scale = R_SCALE;
		R(PSMOVE_ACCELEROMETER_X, PSMOVE_ACCELEROMETER_X) = r_accelerometer_scale*constants.accelerometer_variance.x();
		R(PSMOVE_ACCELEROMETER_Y, PSMOVE_ACCELEROMETER_Y) = r_accelerometer_scale*constants.accelerometer_variance.y();
		R(PSMOVE_ACCELEROMETER_Z, PSMOVE_ACCELEROMETER_Z) = r_accelerometer_scale*constants.accelerometer_variance.z();
		R(PSMOVE_GYROSCOPE_PITCH, PSMOVE_GYROSCOPE_PITCH) = r_gyro_scale*constants.gyro_variance.x();
		R(PSMOVE_GYROSCOPE_YAW, PSMOVE_GYROSCOPE_YAW) = r_gyro_scale*constants.gyro_variance.y();
		R(PSMOVE_GYROSCOPE_ROLL, PSMOVE_GYROSCOPE_ROLL) = r_gyro_scale*constants.gyro_variance.z();
		R(PSMOVE_MAGNETOMETER_X, PSMOVE_MAGNETOMETER_X) = r_magnetometer_scale*constants.magnetometer_variance.x();
		R(PSMOVE_MAGNETOMETER_Y, PSMOVE_MAGNETOMETER_Y) = r_magnetometer_scale*constants.magnetometer_variance.y();
		R(PSMOVE_MAGNETOMETER_Z, PSMOVE_MAGNETOMETER_Z) = r_magnetometer_scale*constants.magnetometer_variance.z();
		setCovariance(R);

		identity_gravity_direction = constants.gravity_calibration_direction.cast<double>();
		identity_magnetometer_direction = constants.magnetometer_calibration_direction.cast<double>();
		m_last_world_orientation = Eigen::Quaterniond::Identity();
		m_last_world_linear_acceleration_m_per_sec_sqr = Eigen::Vector3d::Zero();
	}

	void update_world_orientation(const Eigen::Quaterniond &orientation)
	{
		m_last_world_orientation = orientation;
	}

	void update_world_linear_acceleration_m_per_sec_sqr(const Eigen::Vector3d &linear_acceleration)
	{
		m_last_world_linear_acceleration_m_per_sec_sqr = linear_acceleration;
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
	PSMove_OrientationMeasurementVectord h(const OrientationStateVectord& x) const
	{
		PSMove_OrientationMeasurementVectord predicted_measurement;

		// Use the orientation from the state for prediction
		const Eigen::Quaterniond error_orientation = x.get_error_quaterniond();
		const Eigen::Quaterniond world_to_local_orientation = eigen_quaternion_concatenate(m_last_world_orientation, error_orientation).normalized();

		// Use the current linear acceleration from last frame's state (###HipsterSloth $TODO Skews controller too much)
		// and the current orientation from the state to predict
		// what the accelerometer reading will be (in the space of the controller)
		const Eigen::Vector3d &gravity_accel_g_units = identity_gravity_direction;
		const Eigen::Vector3d linear_accel_g_units = Eigen::Vector3d::Zero(); //m_last_world_linear_acceleration * k_ms2_to_g_units;
		const Eigen::Vector3d accel_world = linear_accel_g_units + gravity_accel_g_units;
		const Eigen::Vector3d accel_local = eigen_vector3d_clockwise_rotate(world_to_local_orientation, accel_world);

		// Use the angular velocity from the state to predict what the gyro reading will be
		const Eigen::Vector3d gyro_local = x.get_angular_velocity_rad_per_sec();

		// Use the orientation from the state to predict
		// what the magnetometer reading should be (in the space of the controller)
		const Eigen::Vector3d &mag_world = identity_magnetometer_direction;
		const Eigen::Vector3d mag_local = eigen_vector3d_clockwise_rotate(world_to_local_orientation, mag_world);

		// Save the predictions into the measurement vector
		predicted_measurement.set_accelerometer(accel_local);
		predicted_measurement.set_magnetometer(mag_local);
		predicted_measurement.set_gyroscope(gyro_local);

		return predicted_measurement;
	}

public:
	Eigen::Vector3d identity_gravity_direction;
	Eigen::Vector3d identity_magnetometer_direction;
	Eigen::Quaterniond m_last_world_orientation;
	Eigen::Vector3d m_last_world_linear_acceleration_m_per_sec_sqr;
};

/**
* @brief Measurement model for measuring DS4 controller
*
* This is the measurement model for measuring the position and orientation of the DS4 controller.
* The measurement is given by the optical trackers.
*/
class DS4_OrientationMeasurementModel
	: public Kalman::MeasurementModel<OrientationStateVectord, DS4_OrientationMeasurementVectord, Kalman::SquareRootBase>
{
public:
	void init(const OrientationFilterConstants &constants)
	{
		m_last_tracking_projection_area = -1.f;
		update_measurement_statistics(constants, 0.f);

		identity_gravity_direction = constants.gravity_calibration_direction.cast<double>();
		m_last_world_orientation = Eigen::Quaterniond::Identity();
		m_last_world_linear_acceleration = Eigen::Vector3d::Zero();
	}

	void update_measurement_statistics(
		const OrientationFilterConstants &constants,
		const float tracking_projection_area)
	{
		// Only update the covariance when there is more than a 10px change in position quality
		if (m_last_tracking_projection_area < 0.f ||
			!is_nearly_equal(tracking_projection_area, m_last_tracking_projection_area, 10.f))
		{
			// Update the measurement covariance R
			Kalman::Covariance<DS4_OrientationMeasurementVectord> R =
				Kalman::Covariance<DS4_OrientationMeasurementVectord>::Zero();
			const float orientation_variance = constants.orientation_variance_curve.evaluate(tracking_projection_area);

			static float r_scale = R_SCALE;
			R(DS4_ACCELEROMETER_X, DS4_ACCELEROMETER_X) = r_scale*constants.accelerometer_variance.x();
			R(DS4_ACCELEROMETER_Y, DS4_ACCELEROMETER_Y) = r_scale*constants.accelerometer_variance.y();
			R(DS4_ACCELEROMETER_Z, DS4_ACCELEROMETER_Z) = r_scale*constants.accelerometer_variance.z();
			R(DS4_GYROSCOPE_PITCH, DS4_GYROSCOPE_PITCH) = r_scale*constants.gyro_variance.x();
			R(DS4_GYROSCOPE_YAW, DS4_GYROSCOPE_YAW) = r_scale*constants.gyro_variance.y();
			R(DS4_GYROSCOPE_ROLL, DS4_GYROSCOPE_ROLL) = r_scale*constants.gyro_variance.z();
			R(DS4_OPTICAL_EULER_ANGLE_HEADING, DS4_OPTICAL_EULER_ANGLE_HEADING) = r_scale*orientation_variance;
			setCovariance(R);

			// Keep track last tracking projection area we built the covariance matrix for
			m_last_tracking_projection_area = tracking_projection_area;
		}
	}

	void update_world_orientation(const Eigen::Quaterniond &orientation)
	{
		m_last_world_orientation = orientation;
	}

	void update_world_linear_acceleration(const Eigen::Vector3d &linear_acceleration)
	{
		m_last_world_linear_acceleration = linear_acceleration;
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
	DS4_OrientationMeasurementVectord h(const OrientationStateVectord& x) const
	{
		DS4_OrientationMeasurementVectord predicted_measurement;

		// Use the orientation from the state for prediction
		const Eigen::Quaterniond error_orientation = x.get_error_quaterniond();
		const Eigen::Quaterniond world_to_local_orientation = eigen_quaternion_concatenate(m_last_world_orientation, error_orientation).normalized();
		const Eigen::EulerAnglesd euler_angles = eigen_quaterniond_to_euler_angles(world_to_local_orientation);
		const double heading_angle = euler_angles.get_heading_radians();

		// Use the current linear acceleration from last frame's state (###HipsterSloth $TODO Skews controller too much)
		// and the current orientation from the state to predict
		// what the accelerometer reading will be (in the space of the controller)
		const Eigen::Vector3d &gravity_accel_g_units = identity_gravity_direction;
		const Eigen::Vector3d linear_accel_g_units = Eigen::Vector3d::Zero(); //m_last_world_linear_acceleration * k_ms2_to_g_units;
		const Eigen::Vector3d accel_world = linear_accel_g_units + gravity_accel_g_units;
		const Eigen::Vector3d accel_local = eigen_vector3d_clockwise_rotate(world_to_local_orientation, accel_world);

		// Use the angular velocity from the state to predict what the gyro reading will be
		const Eigen::Vector3d gyro_local = x.get_angular_velocity_rad_per_sec();

		// Save the predictions into the measurement vector
		predicted_measurement.set_accelerometer(accel_local);
		predicted_measurement.set_gyroscope(gyro_local);
		predicted_measurement.set_optical_euler_heading_angle(heading_angle);

		return predicted_measurement;
	}

public:
	float m_last_tracking_projection_area;
	Eigen::Vector3d identity_gravity_direction;
	Eigen::Quaterniond m_last_world_orientation;
	Eigen::Vector3d m_last_world_linear_acceleration;
};

class KalmanOrientationFilterImpl
{
public:
    /// Is the current fusion state valid
    bool bIsValid;

	/// True if we have seen a valid orientation measurement (>0 orientation quality)
	bool bSeenOrientationMeasurement;

    /// Used to model how the physics of the controller evolves
    OrientationSystemModel system_model;

    /// Unscented Kalman Filter instance
	OrientationSRUKF ukf;

	/// The final output of this filter.
	/// This isn't part of the UKF state vector because it's non-linear.
	/// Instead we store "error euler angles" in the UKF state vector and then apply it 
	/// to this quaternion after a time step and then zero out the error.
	Eigen::Quaterniond world_orientation;

	KalmanOrientationFilterImpl()
		: bIsValid(false)
		, bSeenOrientationMeasurement(false)
		, system_model()
		, ukf(k_ukf_alpha, k_ukf_beta, k_ukf_kappa)
        , world_orientation(Eigen::Quaternionf::Identity())
    {
    }

    virtual void init(const OrientationFilterConstants &constants)
    {
		bIsValid = false;
		bSeenOrientationMeasurement = false;

		world_orientation = Eigen::Quaterniond::Identity();

        system_model.init(constants);
        ukf.init(OrientationStateVectord::Identity());
    }

	virtual void init(
		const OrientationFilterConstants &constants,
		const Eigen::Quaternionf &orientation)
	{
		bIsValid = true;
		bSeenOrientationMeasurement = true;

		world_orientation = orientation.cast<double>();

		system_model.init(constants);
		ukf.init(OrientationStateVectord::Identity());
		apply_error_to_world_quaternion();
	}

	// -- World Quaternion Accessors --
	inline Eigen::Quaterniond compute_net_world_quaternion() const
	{
		const Eigen::Quaterniond error_quaternion= ukf.getState().get_error_quaterniond();
		const Eigen::Quaterniond output_quaternion = eigen_quaternion_concatenate(world_orientation, error_quaternion).normalized();
		return output_quaternion;
	}

	// -- World Quaternion Mutators --
	inline void set_world_quaternion(const Eigen::Quaterniond &orientation)
	{
		world_orientation = orientation;
		ukf.getStateMutable().set_error_quaterniond(Eigen::Quaterniond::Identity());
	}

	void apply_error_to_world_quaternion()
	{
		set_world_quaternion(compute_net_world_quaternion());
	}
};

class DS4KalmanOrientationFilterImpl : public KalmanOrientationFilterImpl
{
public:
	DS4_OrientationMeasurementModel measurement_model;

	void init(const OrientationFilterConstants &constants) override
	{
		KalmanOrientationFilterImpl::init(constants);
		measurement_model.init(constants);
	}

	void init(
		const OrientationFilterConstants &constants,
		const Eigen::Quaternionf &orientation) override
	{
		KalmanOrientationFilterImpl::init(constants, orientation);
		measurement_model.init(constants);
	}
};

class PSMoveKalmanPoseFilterImpl : public KalmanOrientationFilterImpl
{
public:
	PSMove_OrientationMeasurementModel measurement_model;

	void init(const OrientationFilterConstants &constants) override
	{
		KalmanOrientationFilterImpl::init(constants);
		measurement_model.init(constants);
	}

	void init(
		const OrientationFilterConstants &constants,
		const Eigen::Quaternionf &orientation) override
	{
		KalmanOrientationFilterImpl::init(constants, orientation);
		measurement_model.init(constants);
	}
};

//-- public interface --
//-- KalmanOrientationFilter --
KalmanOrientationFilter::KalmanOrientationFilter()
    : m_filter(nullptr)
{
    memset(&m_constants, 0, sizeof(OrientationFilterConstants));
}

KalmanOrientationFilter::~KalmanOrientationFilter()
{
    if (m_filter != nullptr)
    {
        delete m_filter;
        m_filter;
    }
}

bool KalmanOrientationFilter::init(const OrientationFilterConstants &constants)
{
    m_constants = constants;

    // cleanup any existing filter
    if (m_filter != nullptr)
    {
        delete m_filter;
        m_filter;
    }

	// Create and initialize the private filter implementation
    KalmanOrientationFilterImpl *filter = new KalmanOrientationFilterImpl();
    filter->init(constants);
    m_filter = filter;

    return true;
}

bool KalmanOrientationFilter::init(
	const OrientationFilterConstants &constants,
	const Eigen::Quaternionf &orientation)
{
	m_constants = constants;

	// cleanup any existing filter
	if (m_filter != nullptr)
	{
		delete m_filter;
		m_filter;
	}

	// Create and initialize the private filter implementation
	KalmanOrientationFilterImpl *filter = new KalmanOrientationFilterImpl();
	filter->init(constants, orientation);
	m_filter = filter;

	return true;
}

bool KalmanOrientationFilter::getIsStateValid() const
{
	return m_filter->bIsValid;
}

void KalmanOrientationFilter::resetState()
{
	m_filter->init(m_constants);
}

void KalmanOrientationFilter::recenterOrientation(const Eigen::Quaternionf& q_pose)
{
	m_filter->world_orientation = q_pose.cast<double>();
	m_filter->ukf.init(OrientationStateVectord::Identity());
}

Eigen::Quaternionf KalmanOrientationFilter::getOrientation(float time) const
{
	Eigen::Quaternionf result = Eigen::Quaternionf::Identity();

	if (m_filter->bIsValid)
	{
		const Eigen::Quaternionf state_orientation = m_filter->compute_net_world_quaternion().cast<float>();
		Eigen::Quaternionf predicted_orientation = state_orientation;

		if (fabsf(time) > k_real_epsilon)
		{
			const Eigen::Quaternionf &quaternion_derivative =
				eigen_angular_velocity_to_quaternion_derivative(result, getAngularVelocityRadPerSec());

			predicted_orientation = Eigen::Quaternionf(
				state_orientation.coeffs()
				+ quaternion_derivative.coeffs()*time).normalized();
		}

		result = predicted_orientation;
	}

	return result;
}

Eigen::Vector3f KalmanOrientationFilter::getAngularVelocityRadPerSec() const
{
	Eigen::Vector3d ang_vel = m_filter->ukf.getState().get_angular_velocity_rad_per_sec();

	return ang_vel.cast<float>();
}

Eigen::Vector3f KalmanOrientationFilter::getAngularAccelerationRadPerSecSqr() const
{
	return Eigen::Vector3f::Zero();
}

//-- KalmanOrientationFilterDS4 --
bool KalmanOrientationFilterDS4::init(const OrientationFilterConstants &constants)
{
	KalmanOrientationFilter::init(constants);

	DS4KalmanOrientationFilterImpl *filter = new DS4KalmanOrientationFilterImpl();
	filter->init(constants);
	m_filter = filter;

	return true;
}

bool KalmanOrientationFilterDS4::init(
	const OrientationFilterConstants &constants,
	const Eigen::Quaternionf &orientation)
{
	KalmanOrientationFilter::init(constants);

	DS4KalmanOrientationFilterImpl *filter = new DS4KalmanOrientationFilterImpl();
	filter->init(constants, orientation);
	m_filter = filter;

	return true;
}

void KalmanOrientationFilterDS4::update(const float delta_time, const PoseFilterPacket &packet)
{
	if (m_filter->bIsValid)
	{
		DS4KalmanOrientationFilterImpl *filter = static_cast<DS4KalmanOrientationFilterImpl *>(m_filter);

		// Adjust the amount we trust the process model based on the tracking projection area
		filter->system_model.update_process_noise(m_constants, packet.tracking_projection_area_px_sqr);

		// Predict state for current time-step using the filters
		filter->system_model.set_time_step(delta_time);
		filter->ukf.predict(filter->system_model);

		// Get the measurement model for the DS4 from the derived filter impl
		DS4_OrientationMeasurementModel &measurement_model = filter->measurement_model;

		// Update the linear acceleration on the measurement model with last frames acceleration state
		measurement_model.update_world_linear_acceleration(
			packet.get_current_acceleration_in_meters_per_second_squared().cast<double>());

		// If this is the first time we have seen an orientation measurement, snap the orientation state
		if (!m_filter->bSeenOrientationMeasurement && packet.tracking_projection_area_px_sqr > 0.f)
		{
			const Eigen::Quaterniond world_quaternion = packet.optical_orientation.cast<double>();

			measurement_model.update_world_orientation(world_quaternion);

			filter->set_world_quaternion(world_quaternion);
			filter->bSeenOrientationMeasurement = true;
		}

		// Project the current state onto a predicted measurement as a default
		// in case no observation is available
		DS4_OrientationMeasurementVectord local_measurement = measurement_model.h(filter->ukf.getState());

		// Adjust the amount we trust the optical measurements based on the tracking projection area
		measurement_model.update_measurement_statistics(
			m_constants,
			packet.tracking_projection_area_px_sqr);

		// If available, use the optical orientation measurement
		if (packet.tracking_projection_area_px_sqr > 0.f)
		{
			const Eigen::EulerAnglesd world_optical_euler_angles =
				eigen_quaterniond_to_euler_angles(packet.optical_orientation.cast<double>());
			const double world_optical_heading = world_optical_euler_angles.get_heading_radians();

			local_measurement.set_optical_euler_heading_angle(world_optical_heading);
		}

		// Accelerometer and gyroscope measurements are always available
		{
			const Eigen::Vector3d world_accelerometer = packet.imu_accelerometer_g_units.cast<double>();

			local_measurement.set_accelerometer(world_accelerometer);
			local_measurement.set_gyroscope(packet.imu_gyroscope_rad_per_sec.cast<double>());
		}

		// Update UKF
		filter->ukf.update(measurement_model, local_measurement);

		// Apply the orientation error in the UKF state to the output quaternion.
		// Zero out the error in the UKF state vector.
		filter->apply_error_to_world_quaternion();

		// Update the measurement model with the latest estimate of the orientation (without error)
		// so that we can predict what the controller relative sensor measurements will be
		measurement_model.update_world_orientation(filter->world_orientation);
	}
	else
	{
		m_filter->ukf.init(OrientationStateVectord::Identity());
		m_filter->bIsValid = true;
	}
}

//-- KalmanOrientationFilterPSMove --
bool KalmanOrientationFilterPSMove::init(const OrientationFilterConstants &constants)
{
	KalmanOrientationFilter::init(constants);

	PSMoveKalmanPoseFilterImpl *filter = new PSMoveKalmanPoseFilterImpl();
	filter->init(constants);
	m_filter = filter;

	return true;
}

bool KalmanOrientationFilterPSMove::init(
	const OrientationFilterConstants &constants,
	const Eigen::Quaternionf &orientation)
{
	KalmanOrientationFilter::init(constants);

	PSMoveKalmanPoseFilterImpl *filter = new PSMoveKalmanPoseFilterImpl();
	filter->init(constants, orientation);
	m_filter = filter;

	return true;
}

void KalmanOrientationFilterPSMove::update(const float delta_time, const PoseFilterPacket &packet)
{
    if (m_filter->bIsValid)
    {
		PSMoveKalmanPoseFilterImpl *filter = static_cast<PSMoveKalmanPoseFilterImpl *>(m_filter);

        // Predict state for current time-step using the filters
		filter->system_model.set_time_step(delta_time);
		filter->ukf.predict(filter->system_model);

        // Get the measurement model for the PSMove from the derived filter impl
		PSMove_OrientationMeasurementModel &measurement_model = filter->measurement_model;

		// Update the linear acceleration on the measurement model with last frames acceleration state
		measurement_model.update_world_linear_acceleration_m_per_sec_sqr(
			packet.get_current_acceleration_in_meters_per_second_squared().cast<double>());

		// If this is the first time we have seen an orientation measurement, 
		// snap the orientation state to a best fit alignment of the sensor measurements.
		if (!m_filter->bSeenOrientationMeasurement)
		{
			Eigen::Vector3f current_g = packet.imu_accelerometer_g_units;
			eigen_vector3f_normalize_with_default(current_g, Eigen::Vector3f::Zero());

			Eigen::Vector3f current_m = packet.imu_magnetometer_unit;
			eigen_vector3f_normalize_with_default(current_m, Eigen::Vector3f::Zero());

			const Eigen::Vector3f* mg_from[2] = { &m_constants.gravity_calibration_direction, &m_constants.magnetometer_calibration_direction };
			const Eigen::Vector3f* mg_to[2] = { &current_g, &current_m };

			// Always attempt to align even if we don't get within the alignment tolerance.
			// More often then not the attempted alignment is closer to the right answer 
			// then the identity quaternion.
			Eigen::Quaternionf initial_orientation;
			eigen_alignment_quaternion_between_vector_frames(
				mg_from, mg_to, 0.1f, Eigen::Quaternionf::Identity(), initial_orientation);

			measurement_model.update_world_orientation(initial_orientation.cast<double>());

			filter->set_world_quaternion(initial_orientation.cast<double>());
			filter->bSeenOrientationMeasurement = true;
		}

		// Accelerometer, gyroscope, magnetometer measurements are always available
		PSMove_OrientationMeasurementVectord measurement = PSMove_OrientationMeasurementVectord::Zero();
		measurement.set_accelerometer(packet.imu_accelerometer_g_units.cast<double>());
		measurement.set_gyroscope(packet.imu_gyroscope_rad_per_sec.cast<double>());
		measurement.set_magnetometer(packet.imu_magnetometer_unit.cast<double>());
		m_filter->bSeenOrientationMeasurement = true;

        // Update UKF
        filter->ukf.update(measurement_model, measurement);

		// Apply the orientation error in the UKF state to the output quaternion.
		// Zero out the error in the UKF state vector.
		filter->apply_error_to_world_quaternion();

		// Update the measurement model with the latest estimate of the orientation (without error)
		// so that we can predict what the controller relative sensor measurements will be
		measurement_model.update_world_orientation(filter->world_orientation);
    }
    else
    {
		m_filter->ukf.init(OrientationStateVectord::Identity());
        m_filter->bIsValid= true;
    }
}

//-- Private functions --
// Adapted from: https://github.com/rlabbe/filterpy/blob/master/filterpy/common/discretization.py#L52-L53

// Returns the Q matrix for the Discrete Constant White Noise
// - dT is the time step
// - var is the variance in the process noise
// - state_index denotes where in Q the 2x2 covariance matrix should be written

// Q is computed as the G * G^T * variance, where G is the process noise per time step.
// In other words, G = [[.5dt ^ 2][dt]] ^ T for the constant angular velocity model.
template <class StateType>
void Q_discrete_2rd_order_white_noise(
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

    // Q = [.5dt^2, dt]*[.5dt^2, dt]^T * variance
    const int &i= state_index;
    Q(i+0,i+0) = 0.25*q4; Q(i+0,i+1) = 0.5*q3;
    Q(i+1,i+0) =  0.5*q3; Q(i+1,i+1) = q2;
}

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
	EULER_ANGLE_ATTITUDE,
	ANGULAR_VELOCITY_PITCH,
    EULER_ANGLE_HEADING,
	ANGULAR_VELOCITY_YAW,
	EULER_ANGLE_BANK, // radians
	ANGULAR_VELOCITY_ROLL, // meters / s

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

    // Accessors
    Eigen::EulerAnglesd get_euler_angles() const { 
        return Eigen::EulerAnglesd((*this)[EULER_ANGLE_BANK], (*this)[EULER_ANGLE_HEADING], (*this)[EULER_ANGLE_ATTITUDE]);
    }
	Eigen::Quaterniond get_quaterniond() const {
		return eigen_euler_angles_to_quaterniond(get_euler_angles());
	}
    Eigen::Vector3d get_angular_velocity() const {
        return Eigen::Vector3d((*this)[ANGULAR_VELOCITY_PITCH], (*this)[ANGULAR_VELOCITY_YAW], (*this)[ANGULAR_VELOCITY_ROLL]);
    }

    // Mutators
    void set_euler_angles(const Eigen::EulerAnglesd &e) {
        (*this)[EULER_ANGLE_BANK] = e.get_bank_radians(); (*this)[EULER_ANGLE_HEADING] = e.get_heading_radians(); (*this)[EULER_ANGLE_ATTITUDE] = e.get_attitude_radians();
    }
	void set_quaterniond(const Eigen::Quaterniond &d) {
		set_euler_angles(eigen_quaterniond_to_euler_angles(d));
	}
    void set_angular_velocity(const Eigen::Vector3d &v) {
        (*this)[ANGULAR_VELOCITY_PITCH] = v.x(); (*this)[ANGULAR_VELOCITY_YAW] = v.y(); (*this)[ANGULAR_VELOCITY_ROLL] = v.z();
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
			Q_discrete_2rd_order_white_noise<OrientationStateVectord>(mean_orientation_dT, orientation_variance, EULER_ANGLE_BANK, Q);
			Q_discrete_2rd_order_white_noise<OrientationStateVectord>(mean_orientation_dT, orientation_variance, EULER_ANGLE_HEADING, Q);
			Q_discrete_2rd_order_white_noise<OrientationStateVectord>(mean_orientation_dT, orientation_variance, EULER_ANGLE_ATTITUDE, Q);
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
		//! Predicted state vector after transition
		OrientationStateVectord new_state;

		// Extract parameters from the old state
		const Eigen::Quaterniond q_old = old_state.get_quaterniond();
		const Eigen::Vector3d angular_velocity = old_state.get_angular_velocity();

		// Compute the quaternion derivative of the current state
		// q_new= q + q_dot*dT
		const Eigen::Quaterniond q_dot = eigen_angular_velocity_to_quaterniond_derivative(q_old, angular_velocity);
		const Eigen::Quaterniond q_step = Eigen::Quaterniond(q_dot.coeffs() * m_time_step);
		const Eigen::Quaterniond q_new = Eigen::Quaterniond(q_old.coeffs() + q_step.coeffs());

		// Save results to the new state
		new_state.set_quaterniond(q_new.normalized());
		new_state.set_angular_velocity(angular_velocity);

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
		static float r_scale = R_SCALE;
		R(PSMOVE_ACCELEROMETER_X, PSMOVE_ACCELEROMETER_X) = sqrt(r_scale*constants.accelerometer_variance.x());
		R(PSMOVE_ACCELEROMETER_Y, PSMOVE_ACCELEROMETER_Y) = sqrt(r_scale*constants.accelerometer_variance.y());
		R(PSMOVE_ACCELEROMETER_Z, PSMOVE_ACCELEROMETER_Z) = sqrt(r_scale*constants.accelerometer_variance.z());
		R(PSMOVE_GYROSCOPE_PITCH, PSMOVE_GYROSCOPE_PITCH) = sqrt(r_scale*constants.gyro_variance.x());
		R(PSMOVE_GYROSCOPE_YAW, PSMOVE_GYROSCOPE_YAW) = sqrt(r_scale*constants.gyro_variance.y());
		R(PSMOVE_GYROSCOPE_ROLL, PSMOVE_GYROSCOPE_ROLL) = sqrt(r_scale*constants.gyro_variance.z());
		R(PSMOVE_MAGNETOMETER_X, PSMOVE_MAGNETOMETER_X) = sqrt(r_scale*constants.magnetometer_variance.x());
		R(PSMOVE_MAGNETOMETER_Y, PSMOVE_MAGNETOMETER_Y) = sqrt(r_scale*constants.magnetometer_variance.y());
		R(PSMOVE_MAGNETOMETER_Z, PSMOVE_MAGNETOMETER_Z) = sqrt(r_scale*constants.magnetometer_variance.z());

		identity_gravity_direction = constants.gravity_calibration_direction.cast<double>();
		identity_magnetometer_direction = constants.magnetometer_calibration_direction.cast<double>();
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
		const Eigen::Quaterniond orientation = x.get_quaterniond();

		// Use the current orientation from the state to predict
		// what the accelerometer reading will be (in the space of the controller)
		const Eigen::Vector3d &accel_world = identity_gravity_direction;
		const Eigen::Quaterniond accel_world_quat(0.f, accel_world.x(), accel_world.y(), accel_world.z());
		const Eigen::Vector3d accel_local = orientation*(accel_world_quat*orientation.conjugate()).vec();

		// Use the angular velocity from the state to predict what the gyro reading will be
		const Eigen::Vector3d gyro_local = x.get_angular_velocity();

		// Use the orientation from the state to predict
		// what the magnetometer reading should be (in the space of the controller)
		const Eigen::Vector3d &mag_world = identity_magnetometer_direction;
		const Eigen::Quaterniond mag_world_quat(0.f, mag_world.x(), mag_world.y(), mag_world.z());
		const Eigen::Vector3d mag_local = orientation*(mag_world_quat*orientation.conjugate()).vec();

		// Save the predictions into the measurement vector
		predicted_measurement.set_accelerometer(accel_local);
		predicted_measurement.set_magnetometer(mag_local);
		predicted_measurement.set_gyroscope(gyro_local);

		return predicted_measurement;
	}

public:
	Eigen::Vector3d identity_gravity_direction;
	Eigen::Vector3d identity_magnetometer_direction;
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
			R(DS4_ACCELEROMETER_X, DS4_ACCELEROMETER_X) = sqrt(r_scale*constants.accelerometer_variance.x());
			R(DS4_ACCELEROMETER_Y, DS4_ACCELEROMETER_Y) = sqrt(r_scale*constants.accelerometer_variance.y());
			R(DS4_ACCELEROMETER_Z, DS4_ACCELEROMETER_Z) = sqrt(r_scale*constants.accelerometer_variance.z());
			R(DS4_GYROSCOPE_PITCH, DS4_GYROSCOPE_PITCH) = sqrt(r_scale*constants.gyro_variance.x());
			R(DS4_GYROSCOPE_YAW, DS4_GYROSCOPE_YAW) = sqrt(r_scale*constants.gyro_variance.y());
			R(DS4_GYROSCOPE_ROLL, DS4_GYROSCOPE_ROLL) = sqrt(r_scale*constants.gyro_variance.z());
			R(DS4_OPTICAL_EULER_ANGLE_HEADING, DS4_OPTICAL_EULER_ANGLE_HEADING) = sqrt(r_scale*orientation_variance);

			// Keep track last tracking projection area we built the covariance matrix for
			m_last_tracking_projection_area = tracking_projection_area;
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
	DS4_OrientationMeasurementVectord h(const OrientationStateVectord& x) const
	{
		DS4_OrientationMeasurementVectord predicted_measurement;

		// Use the orientation from the state for prediction
		const Eigen::EulerAnglesd &euler_angles = x.get_euler_angles();
		const double heading_angle = euler_angles.get_heading_radians();
		const Eigen::Quaterniond orientation = eigen_euler_angles_to_quaterniond(euler_angles);

		// Use the current orientation from the state to predict
		// what the accelerometer reading will be (in the space of the controller)
		const Eigen::Vector3d &accel_world = identity_gravity_direction;
		const Eigen::Quaterniond accel_world_quat(0.f, accel_world.x(), accel_world.y(), accel_world.z());
		const Eigen::Vector3d accel_local = orientation*(accel_world_quat*orientation.conjugate()).vec();

		// Use the angular velocity from the state to predict what the gyro reading will be
		const Eigen::Vector3d gyro_local = x.get_angular_velocity();

		// Save the predictions into the measurement vector
		predicted_measurement.set_accelerometer(accel_local);
		predicted_measurement.set_gyroscope(gyro_local);
		predicted_measurement.set_optical_euler_heading_angle(heading_angle);

		return predicted_measurement;
	}

public:
	float m_last_tracking_projection_area;
	Eigen::Vector3d identity_gravity_direction;
};

class KalmanOrientationFilterImpl
{
public:
    /// Is the current fusion state valid
    bool bIsValid;

	/// True if we have seen a valid orientation measurement (>0 orientation quality)
	bool bSeenOrientationMeasurement;

	/// Quaternion measured when controller points towards camera 
	Eigen::Quaternionf reset_orientation;

	/// The offset of the heading angle to the world space heading angle.
	/// This is used to keep state heading angle between +/- 90 degrees,
	/// which allows us to avoid filtering across the +/- 180 degree discontinuity
	double local_heading_to_world_heading;

    /// Used to model how the physics of the controller evolves
    OrientationSystemModel system_model;

    /// Unscented Kalman Filter instance
	OrientationSRUKF ukf;

	KalmanOrientationFilterImpl()
		: bIsValid(false)
		, bSeenOrientationMeasurement(false)
		, reset_orientation(Eigen::Quaternionf::Identity())
		, local_heading_to_world_heading(0.0)
		, system_model()
		, ukf(k_ukf_alpha, k_ukf_beta, k_ukf_kappa)
    {
    }

    virtual void init(const OrientationFilterConstants &constants)
    {
		bIsValid = false;
		bSeenOrientationMeasurement = false;

		reset_orientation = Eigen::Quaternionf::Identity();
		local_heading_to_world_heading = 0.0;

        system_model.init(constants);
        ukf.init(OrientationStateVectord::Zero());
    }

	virtual void init(
		const OrientationFilterConstants &constants,
		const Eigen::Quaternionf &orientation)
	{
		bIsValid = true;
		bSeenOrientationMeasurement = true;

		reset_orientation = Eigen::Quaternionf::Identity();
		local_heading_to_world_heading = 0.0;

		OrientationStateVectord state_vector = OrientationStateVectord::Zero();
		state_vector.set_quaterniond(orientation.cast<double>());

		system_model.init(constants);
		ukf.init(state_vector);
		clamp_state_heading();
	}

	// -- State Orientation Accessors --
	Eigen::EulerAnglesd get_state_world_euler_angles() const
	{
		const Eigen::EulerAnglesd local_euler_angles = ukf.getState().get_euler_angles();
		const Eigen::EulerAnglesd world_euler_angles(
			local_euler_angles.get_bank_radians(),
			wrap_ranged(local_euler_angles.get_heading_radians() + local_heading_to_world_heading, -k_real64_pi, k_real64_pi),
			local_euler_angles.get_attitude_radians());

		return world_euler_angles;
	}

	inline Eigen::Quaterniond get_state_world_quaternion() const
	{
		return eigen_euler_angles_to_quaterniond(get_state_world_euler_angles());
	}

	inline Eigen::EulerAnglesd get_state_local_euler_angles() const
	{
		return ukf.getState().get_euler_angles();
	}

	// -- State Orientation Mutators --
	inline void set_state_world_quaternion(const Eigen::Quaterniond &world_quaternion)
	{
		set_state_world_euler_angles(eigen_quaterniond_to_euler_angles(world_quaternion));
	}

	void set_state_world_euler_angles(const Eigen::EulerAnglesd &world_euler_angles)
	{
		local_heading_to_world_heading = 0;
		set_state_unclamped_local_euler_angles(world_euler_angles);
	}

	void set_state_unclamped_local_euler_angles(const Eigen::EulerAnglesd &unclamped_local_euler_angles)
	{
		const double clamped_local_bank = unclamped_local_euler_angles.get_bank_radians();
		double clamped_local_heading = unclamped_local_euler_angles.get_heading_radians();
		const double clamped_local_attitude = unclamped_local_euler_angles.get_attitude_radians();

		while (clamped_local_heading < -k_real64_half_pi || clamped_local_heading > k_real64_half_pi)
		{
			if (clamped_local_heading > k_real64_half_pi)
			{
				clamped_local_heading -= k_real64_half_pi;
				local_heading_to_world_heading += k_real64_half_pi;				
				local_heading_to_world_heading= wrap_ranged(local_heading_to_world_heading, -k_real64_two_pi, k_real64_two_pi);
			}
			else // clamped_local_heading < -k_real64_half_pi
			{
				clamped_local_heading+= k_real64_half_pi;
				local_heading_to_world_heading -= k_real64_half_pi;
				local_heading_to_world_heading= wrap_ranged(local_heading_to_world_heading, -k_real64_two_pi, k_real64_two_pi);
			}
		}

		ukf.getStateMutable().set_euler_angles(Eigen::EulerAnglesd(clamped_local_bank, clamped_local_heading, clamped_local_attitude));
	}

	void clamp_state_heading()
	{
		// Assume local heading angle is un-clamped and needs to be pulled in range
		set_state_unclamped_local_euler_angles(get_state_local_euler_angles());
	}

	// -- Orientation Space Converters --
	const Eigen::Vector3d world_vector_to_unclamped_local_vector(const Eigen::Vector3d &world_vector)
	{
		const Eigen::EulerAnglesd world_to_local_euler(0.f, -local_heading_to_world_heading, 0.f);
		const Eigen::Quaterniond world_to_local_quat = eigen_euler_angles_to_quaterniond(world_to_local_euler);
		const Eigen::Vector3d local_vector= eigen_vector3d_clockwise_rotate(world_to_local_quat, world_vector);

		return local_vector;
	}

	const double world_heading_to_unclamped_local_heading(const double world_heading)
	{
		double local_heading = wrap_ranged(world_heading - local_heading_to_world_heading, -k_real64_pi, k_real64_pi);

		return local_heading;
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

void KalmanOrientationFilter::recenterState(const Eigen::Vector3f& p_pose, const Eigen::Quaternionf& q_pose)
{
	Eigen::Quaternionf q_inverse = getOrientation().conjugate();

	eigen_quaternion_normalize_with_default(q_inverse, Eigen::Quaternionf::Identity());
	m_filter->reset_orientation = q_pose*q_inverse;
}

Eigen::Quaternionf KalmanOrientationFilter::getOrientation(float time) const
{
	Eigen::Quaternionf result = Eigen::Quaternionf::Identity();

	if (m_filter->bIsValid)
	{
		const Eigen::Quaternionf state_orientation = m_filter->get_state_world_quaternion().cast<float>();
		Eigen::Quaternionf predicted_orientation = state_orientation;

		if (fabsf(time) > k_real_epsilon)
		{
			const Eigen::Quaternionf &quaternion_derivative =
				eigen_angular_velocity_to_quaternion_derivative(result, getAngularVelocity());

			predicted_orientation = Eigen::Quaternionf(
				state_orientation.coeffs()
				+ quaternion_derivative.coeffs()*time).normalized();
		}

		result = m_filter->reset_orientation * predicted_orientation;
	}

	return result;
}

Eigen::Vector3f KalmanOrientationFilter::getAngularVelocity() const
{
	Eigen::Vector3d ang_vel = m_filter->ukf.getState().get_angular_velocity();

	return ang_vel.cast<float>();
}

Eigen::Vector3f KalmanOrientationFilter::getAngularAcceleration() const
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
		filter->system_model.update_process_noise(m_constants, packet.tracking_projection_area);

		// Predict state for current time-step using the filters
		filter->system_model.set_time_step(delta_time);
		filter->ukf.predict(filter->system_model);
		filter->clamp_state_heading();

		// Get the measurement model for the DS4 from the derived filter impl
		DS4_OrientationMeasurementModel &measurement_model = filter->measurement_model;

		// If this is the first time we have seen the orientation, snap the orientation state
		if (!m_filter->bSeenOrientationMeasurement && packet.tracking_projection_area > 0.f)
		{
			const Eigen::Quaterniond world_quaternion = packet.optical_orientation.cast<double>();

			filter->set_state_world_quaternion(world_quaternion);
			filter->bSeenOrientationMeasurement = true;
		}

		// Project the current state onto a predicted measurement as a default
		// in case no observation is available
		DS4_OrientationMeasurementVectord local_measurement = measurement_model.h(filter->ukf.getState());

		// Adjust the amount we trust the optical measurements based on the tracking projection area
		measurement_model.update_measurement_statistics(
			m_constants,
			packet.tracking_projection_area);

		// If available, use the optical orientation measurement
		if (packet.tracking_projection_area > 0.f)
		{
			const Eigen::EulerAnglesd world_optical_euler_angles =
				eigen_quaterniond_to_euler_angles(packet.optical_orientation.cast<double>());
			const double world_optical_heading = world_optical_euler_angles.get_heading_radians();
			const double local_optical_heading = filter->world_heading_to_unclamped_local_heading(world_optical_heading);

			local_measurement.set_optical_euler_heading_angle(local_optical_heading);
		}

		// Accelerometer and gyroscope measurements are always available
		{
			const Eigen::Vector3d world_accelerometer = packet.imu_accelerometer.cast<double>();
			const Eigen::Vector3d local_accelerometer = filter->world_vector_to_unclamped_local_vector(world_accelerometer);

			local_measurement.set_accelerometer(local_accelerometer);
			local_measurement.set_gyroscope(packet.imu_gyroscope.cast<double>());
		}

		// Update UKF
		filter->ukf.update(measurement_model, local_measurement);
		filter->clamp_state_heading();
	}
	else
	{
		m_filter->ukf.init(OrientationStateVectord::Zero());
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
		filter->clamp_state_heading();

        // Get the measurement model for the PSMove from the derived filter impl
		PSMove_OrientationMeasurementModel &measurement_model = filter->measurement_model;

		// Apply the world-to-local heading transform on the sensor measurements
		const Eigen::Vector3d world_accelerometer = packet.imu_accelerometer.cast<double>();
		const Eigen::Vector3d local_accelerometer = filter->world_vector_to_unclamped_local_vector(world_accelerometer);
		const Eigen::Vector3d world_magnetometer = packet.imu_magnetometer.cast<double>();
		const Eigen::Vector3d local_magnetometer = filter->world_vector_to_unclamped_local_vector(world_magnetometer);

		// Accelerometer, gyroscope, magnetometer measurements are always available
		PSMove_OrientationMeasurementVectord measurement = PSMove_OrientationMeasurementVectord::Zero();
		measurement.set_accelerometer(local_accelerometer);
		measurement.set_gyroscope(packet.imu_gyroscope.cast<double>());
		measurement.set_magnetometer(local_magnetometer);
		m_filter->bSeenOrientationMeasurement = true;

        // Update UKF
        filter->ukf.update(measurement_model, measurement);
		filter->clamp_state_heading();
    }
    else
    {
		m_filter->ukf.init(OrientationStateVectord::Zero());
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
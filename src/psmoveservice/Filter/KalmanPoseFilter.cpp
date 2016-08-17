//-- includes --
#include "KalmanPoseFilter.h"

#include <kalman/MeasurementModel.hpp>
#include <kalman/SystemModel.hpp>
#include <kalman/SquareRootUnscentedKalmanFilter.hpp>

//-- private definitions --
enum StateEnum
{
	POSITION_X, // meters
	POSITION_Y,
	POSITION_Z,
	LINEAR_VELOCITY_X, // meters / s
	LINEAR_VELOCITY_Y,
	LINEAR_VELOCITY_Z,
	LINEAR_ACCELERATION_X, // meters /s^2
	LINEAR_ACCELERATION_Y,
	LINEAR_ACCELERATION_Z,
	ORIENTATION_W, 
	ORIENTATION_X,
	ORIENTATION_Y,
	ORIENTATION_Z,
	ANGULAR_VELOCITY_X, // rad/s
	ANGULAR_VELOCITY_Y,
	ANGULAR_VELOCITY_Z,

	STATE_PARAMETER_COUNT
};

template<typename T>
class StateVector : public Kalman::Vector<T, STATE_PARAMETER_COUNT>
{
public:
	KALMAN_VECTOR(StateVector, T, STATE_PARAMETER_COUNT)

	// Accessors
	Eigen::Vector3f get_position() const { 
		return Eigen::Vector3f((*this)[POSITION_X], (*this)[POSITION_Y], (*this)[POSITION_Z]); 
	}
	Eigen::Vector3f get_linear_velocity() const {
		return Eigen::Vector3f((*this)[LINEAR_VELOCITY_X], (*this)[LINEAR_VELOCITY_Y], (*this)[LINEAR_VELOCITY_Z]);
	}
	Eigen::Vector3f get_linear_acceleration() const {
		return Eigen::Vector3f((*this)[LINEAR_ACCELERATION_X], (*this)[LINEAR_ACCELERATION_Y], (*this)[LINEAR_ACCELERATION_Z]);
	}
	Eigen::Quaternionf get_orientation() const {
		return Eigen::Quaternionf((*this)[ORIENTATION_W], (*this)[ORIENTATION_X], (*this)[ORIENTATION_Y], (*this)[ORIENTATION_Z]);
	}
	Eigen::Vector3f get_angular_velocity() const {
		return Eigen::Vector3f((*this)[ANGULAR_VELOCITY_X], (*this)[ANGULAR_VELOCITY_Y], (*this)[ANGULAR_VELOCITY_Z]);
	}

	// Mutators
	void set_position(const Eigen::Vector3f &p) {
		(*this)[POSITION_X] = p.x(); (*this)[POSITION_Y] = p.y(); (*this)[POSITION_Z] = p.z();
	}
	void set_linear_velocity(const Eigen::Vector3f &v) {
		(*this)[LINEAR_VELOCITY_X] = v.x(); (*this)[LINEAR_VELOCITY_Y] = v.y(); (*this)[LINEAR_VELOCITY_Z] = v.z();
	}
	void set_linear_acceleration(const Eigen::Vector3f &a) {
		(*this)[LINEAR_ACCELERATION_X] = a.x(); (*this)[LINEAR_ACCELERATION_Y] = a.y(); (*this)[LINEAR_ACCELERATION_Z] = a.z();
	}
	void set_orientation(const Eigen::Quaternionf &q) {
		(*this)[ORIENTATION_W] = q.w(); (*this)[ORIENTATION_X] = q.x(); (*this)[ORIENTATION_Y] = q.y(); (*this)[ORIENTATION_Z] = q.z();
	}
	void set_angular_velocity(const Eigen::Vector3f &v) {
		(*this)[ANGULAR_VELOCITY_X] = v.x(); (*this)[ANGULAR_VELOCITY_Y] = v.y(); (*this)[ANGULAR_VELOCITY_Z] = v.z();
	}
};

enum PSMoveMeasurementEnum {
	PSMOVE_ACCELEROMETER_X, // gravity units
	PSMOVE_ACCELEROMETER_Y,
	PSMOVE_ACCELEROMETER_Z,
	PSMOVE_GYROSCOPE_X, // rad / s
	PSMOVE_GYROSCOPE_Y,
	PSMOVE_GYROSCOPE_Z,
	PSMOVE_MAGNETOMETER_X,
	PSMOVE_MAGNETOMETER_Y,
	PSMOVE_MAGNETOMETER_Z,
	PSMOVE_OPTICAL_POSITION_X, // meters
	PSMOVE_OPTICAL_POSITION_Y,
	PSMOVE_OPTICAL_POSITION_Z,

	PSMOVE_MEASUREMENT_PARAMETER_COUNT
};

template<typename T>
class PSMove_MeasurementVector : public Kalman::Vector<T, PSMOVE_MEASUREMENT_PARAMETER_COUNT>
{
public:
	KALMAN_VECTOR(PSMove_MeasurementVector, T, PSMOVE_MEASUREMENT_PARAMETER_COUNT)

	// Accessors
	Eigen::Vector3f get_accelerometer() const {
		return Eigen::Vector3f((*this)[PSMOVE_ACCELEROMETER_X], (*this)[PSMOVE_ACCELEROMETER_Y], (*this)[PSMOVE_ACCELEROMETER_Z]);
	}
	Eigen::Vector3f get_gyroscope() const {
		return Eigen::Vector3f((*this)[PSMOVE_GYROSCOPE_X], (*this)[PSMOVE_GYROSCOPE_Y], (*this)[PSMOVE_GYROSCOPE_Z]);
	}
	Eigen::Vector3f get_magnetometer() const {
		return Eigen::Vector3f((*this)[PSMOVE_MAGNETOMETER_X], (*this)[PSMOVE_MAGNETOMETER_Y], (*this)[PSMOVE_MAGNETOMETER_Z]);
	}
	Eigen::Vector3f get_optical_position() const {
		return Eigen::Vector3f((*this)[PSMOVE_OPTICAL_POSITION_X], (*this)[PSMOVE_OPTICAL_POSITION_Y], (*this)[PSMOVE_OPTICAL_POSITION_Z]);
	}

	// Mutators
	void set_accelerometer(const Eigen::Vector3f &a) {
		(*this)[PSMOVE_ACCELEROMETER_X] = a.x(); (*this)[PSMOVE_ACCELEROMETER_Y] = a.y(); (*this)[PSMOVE_ACCELEROMETER_Z] = a.z();
	}
	void set_gyroscope(const Eigen::Vector3f &g) {
		(*this)[PSMOVE_GYROSCOPE_X] = g.x(); (*this)[PSMOVE_GYROSCOPE_Y] = g.y(); (*this)[PSMOVE_GYROSCOPE_Z] = g.z();
	}
	void set_optical_position(const Eigen::Vector3f &p) {
		(*this)[PSMOVE_OPTICAL_POSITION_X] = p.x(); (*this)[PSMOVE_OPTICAL_POSITION_Y] = p.y(); (*this)[PSMOVE_OPTICAL_POSITION_Z] = p.z();
	}
	void set_magnetometer(const Eigen::Vector3f &m) {
		(*this)[PSMOVE_MAGNETOMETER_X] = m.x(); (*this)[PSMOVE_MAGNETOMETER_Y] = m.y(); (*this)[PSMOVE_MAGNETOMETER_Z] = m.z();
	}
};

enum DS4MeasurementEnum {
	DS4_ACCELEROMETER_X,
	DS4_ACCELEROMETER_Y,
	DS4_ACCELEROMETER_Z,
	DS4_GYROSCOPE_X,
	DS4_GYROSCOPE_Y,
	DS4_GYROSCOPE_Z,
	DS4_OPTICAL_POSITION_X,
	DS4_OPTICAL_POSITION_Y,
	DS4_OPTICAL_POSITION_Z,
	DS4_OPTICAL_ORIENTATION_W,
	DS4_OPTICAL_ORIENTATION_X,
	DS4_OPTICAL_ORIENTATION_Y,
	DS4_OPTICAL_ORIENTATION_Z,

	DS4_MEASUREMENT_PARAMETER_COUNT
};

template<typename T>
class DS4_MeasurementVector : public Kalman::Vector<T, DS4_MEASUREMENT_PARAMETER_COUNT>
{
public:
	KALMAN_VECTOR(DS4_MeasurementVector, T, DS4_MEASUREMENT_PARAMETER_COUNT)

	// Accessors
	Eigen::Vector3f get_accelerometer() const {
		return Eigen::Vector3f((*this)[DS4_ACCELEROMETER_X], (*this)[DS4_ACCELEROMETER_Y], (*this)[DS4_ACCELEROMETER_Z]);
	}
	Eigen::Vector3f get_gyroscope() const {
		return Eigen::Vector3f((*this)[DS4_GYROSCOPE_X], (*this)[DS4_GYROSCOPE_Y], (*this)[DS4_GYROSCOPE_Z]);
	}
	Eigen::Vector3f get_optical_position() const {
		return Eigen::Vector3f((*this)[DS4_OPTICAL_POSITION_X], (*this)[DS4_OPTICAL_POSITION_Y], (*this)[DS4_OPTICAL_POSITION_Z]);
	}
	Eigen::Quaternionf get_optical_orientation() const {
		return Eigen::Quaternionf((*this)[DS4_OPTICAL_ORIENTATION_W], (*this)[DS4_OPTICAL_ORIENTATION_X], (*this)[DS4_OPTICAL_ORIENTATION_Y], (*this)[DS4_OPTICAL_ORIENTATION_Z]);
	}

	// Mutators
	void set_accelerometer(const Eigen::Vector3f &a) {
		(*this)[DS4_ACCELEROMETER_X] = a.x(); (*this)[DS4_ACCELEROMETER_Y] = a.y(); (*this)[DS4_ACCELEROMETER_Z] = a.z();
	}
	void set_gyroscope(const Eigen::Vector3f &g) {
		(*this)[DS4_GYROSCOPE_X] = g.x(); (*this)[DS4_GYROSCOPE_Y] = g.y(); (*this)[DS4_GYROSCOPE_Z] = g.z();
	}
	void set_optical_position(const Eigen::Vector3f &p) {
		(*this)[DS4_OPTICAL_POSITION_X] = p.x(); (*this)[DS4_OPTICAL_POSITION_Y] = p.y(); (*this)[DS4_OPTICAL_POSITION_Z] = p.z();
	}
	void set_optical_orientation(const Eigen::Quaternionf &q) {
		(*this)[DS4_OPTICAL_ORIENTATION_W] = q.w(); (*this)[DS4_OPTICAL_ORIENTATION_X] = q.x(); (*this)[DS4_OPTICAL_ORIENTATION_Y] = q.y(); (*this)[DS4_OPTICAL_ORIENTATION_Z] = q.z();
	}
};

/**
* @brief System model for a controller
*
* This is the system model defining how a controller advances from one
* time-step to the next, i.e. how the system state evolves over time.
*/
class SystemModel : public Kalman::SystemModel<StateVector<float>, Kalman::Vector<float, 0>, Kalman::SquareRootBase>
{
public:
	inline void set_time_step(const float dt) { m_time_step = dt; }

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
	StateVector<float> f(const StateVector<float>& old_state, const Kalman::Vector<float, 0>& control) const
	{
		//! Predicted state vector after transition
		StateVector<float> new_state;

		// Extract parameters from the old state
		const Eigen::Vector3f old_position = old_state.get_position();
		const Eigen::Vector3f old_linear_velocity = old_state.get_linear_velocity();
		const Eigen::Vector3f old_linear_acceleration = old_state.get_linear_acceleration();
		const Eigen::Quaternionf old_orientation = old_state.get_orientation();
		const Eigen::Vector3f old_angular_velocity = old_state.get_angular_velocity();

		// Compute the position state update
		const Eigen::Vector3f new_position= 
			old_position 
			+ old_linear_velocity*m_time_step 
			+ old_linear_acceleration*m_time_step*m_time_step*0.5f;
		const Eigen::Vector3f new_linear_velocity= old_linear_velocity + old_linear_acceleration*m_time_step;
		const Eigen::Vector3f &new_linear_acceleration= old_linear_acceleration;

		// Compute the orientation update
		const Eigen::Quaternionf quaternion_derivative =
			eigen_angular_velocity_to_quaternion_derivative(old_orientation, old_angular_velocity);
		const Eigen::Quaternionf new_orientation = Eigen::Quaternionf(
			old_orientation.coeffs()
			+ quaternion_derivative.coeffs()*m_time_step).normalized();
		const Eigen::Vector3f &new_angular_velocity= old_angular_velocity;

		// Save results to the new state
		new_state.set_position(new_position);
		new_state.set_linear_velocity(new_linear_velocity);
		new_state.set_linear_acceleration(new_linear_acceleration);
		new_state.set_orientation(new_orientation);
		new_state.set_angular_velocity(new_angular_velocity);

		return new_state;
	}

protected:
	float m_time_step;
};

/**
* @brief Measurement model for measuring PSMove controller
*
* This is the measurement model for measuring the position and magnetometer of the PSMove controller.
* The measurement is given by the optical trackers.
*/
class PSMove_MeasurementModel : public Kalman::MeasurementModel<StateVector<float>, PSMove_MeasurementVector<float>, Kalman::SquareRootBase>
{
public:
	inline void set_gravity_identity_direction(const Eigen::Vector3f &gravity) { m_identity_gravity_direction = gravity; }
	inline void set_magnetometer_identity_direction(const Eigen::Vector3f &magnetometer) { m_identity_magnetometer_direction = magnetometer; }

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
	PSMove_MeasurementVector<float> h(const StateVector<float>& x) const
	{
		PSMove_MeasurementVector<float> predicted_measurement;

		// Use the position and orientation from the state for predictions
		const Eigen::Vector3f position= x.get_position();
		const Eigen::Quaternionf orientation= x.get_orientation();

		// Use the current linear acceleration from the state to predict
		// what the accelerometer reading will be (in world space)
		const Eigen::Vector3f gravity_accel_g_units= -m_identity_gravity_direction;
		const Eigen::Vector3f linear_accel_g_units= x.get_linear_acceleration() * k_ms2_to_g_units;
		const Eigen::Vector3f accel_world= linear_accel_g_units + gravity_accel_g_units;
		const Eigen::Quaternionf accel_world_quat(0.f, accel_world.x(), accel_world.y(), accel_world.z());

		// Put the accelerometer prediction into the local space of the controller
		const Eigen::Vector3f accel_local= orientation*(accel_world_quat*orientation.conjugate()).vec();

		// Use the angular velocity from the state to predict what the gyro reading will be
		const Eigen::Vector3f gyro_local= x.get_angular_velocity(); 

		// Use the orientation from the state to predict
		// what the magnetometer reading should be
		const Eigen::Vector3f &mag_world= m_identity_magnetometer_direction;
		const Eigen::Quaternionf mag_world_quat(0.f, mag_world.x(), mag_world.y(), mag_world.z());
		const Eigen::Vector3f mag_local= orientation*(mag_world_quat*orientation.conjugate()).vec();

		// Save the predictions into the measurement vector
		predicted_measurement.set_accelerometer(accel_local);
		predicted_measurement.set_magnetometer(mag_local);
		predicted_measurement.set_gyroscope(gyro_local);
		predicted_measurement.set_optical_position(position);

		return predicted_measurement;
	}

protected:
	Eigen::Vector3f m_identity_gravity_direction;
	Eigen::Vector3f m_identity_magnetometer_direction;
};

/**
* @brief Measurement model for measuring DS4 controller
*
* This is the measurement model for measuring the position and orientation of the DS4 controller.
* The measurement is given by the optical trackers.
*/
class DS4_MeasurementModel : public Kalman::MeasurementModel<StateVector<float>, DS4_MeasurementVector<float>, Kalman::SquareRootBase>
{
public:
	inline void set_gravity_identity_direction(const Eigen::Vector3f &gravity) { m_identity_gravity_direction = gravity; }

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
	DS4_MeasurementVector<float> h(const StateVector<float>& x) const
	{
		DS4_MeasurementVector<float> predicted_measurement;

		// Use the position and orientation from the state for predictions
		const Eigen::Vector3f position= x.get_position();
		const Eigen::Quaternionf orientation= x.get_orientation();

		// Use the current linear acceleration from the state to predict
		// what the accelerometer reading will be (in world space)
		const Eigen::Vector3f gravity_accel_g_units= -m_identity_gravity_direction;
		const Eigen::Vector3f linear_accel_g_units= x.get_linear_acceleration() * k_ms2_to_g_units;
		const Eigen::Vector3f accel_world= linear_accel_g_units + gravity_accel_g_units;
		const Eigen::Quaternionf accel_world_quat(0.f, accel_world.x(), accel_world.y(), accel_world.z());

		// Put the accelerometer prediction into the local space of the controller
		const Eigen::Vector3f accel_local= orientation*(accel_world_quat*orientation.conjugate()).vec();

		// Use the angular velocity from the state to predict what the gyro reading will be
		const Eigen::Vector3f gyro_local= x.get_angular_velocity(); 

		// Save the predictions into the measurement vector
		predicted_measurement.set_accelerometer(accel_local);
		predicted_measurement.set_gyroscope(gyro_local);
		predicted_measurement.set_optical_position(position);
		predicted_measurement.set_optical_orientation(orientation);

		return predicted_measurement;
	}

protected:
	Eigen::Vector3f m_identity_gravity_direction;
};

class KalmanFilterImpl
{
public:
	/// Is the current fusion state valid
	bool bIsValid;

	/// Quaternion measured when controller points towards camera 
	Eigen::Quaternionf reset_orientation;

	/// Position that's considered the origin position 
	Eigen::Vector3f origin_position; // meters

	/// All state parameters of the controller
	StateVector<float> state_vector;

	/// Used to model how the physics of the controller evolves
	SystemModel system_model;

	/// Unscented Kalman Filter instance
	Kalman::SquareRootUnscentedKalmanFilter<StateVector<float>> *ukf;

	KalmanFilterImpl()
	{
		const float alpha = 1.f;
		const float beta = 2.f;
		const float kappa = -1.f;

		ukf = new Kalman::SquareRootUnscentedKalmanFilter<StateVector<float>>(alpha, beta, kappa);
	}

	virtual ~KalmanFilterImpl()
	{
		delete ukf;
	}

	void init(const PoseFilterConstants &constants)
	{
		bIsValid = false;
		reset_orientation = Eigen::Quaternionf::Identity();
		origin_position = Eigen::Vector3f::Zero();

		state_vector.setZero();

		init_system_model(constants);
		ukf->init(state_vector);
	}

	void init_system_model(const PoseFilterConstants &constants)
	{
		// TODO: What should these process variances actually be?
		const float k_initial_variance= 0.1f;
		Kalman::Covariance<StateVector<float>> process_covariance = 
			Kalman::Covariance<StateVector<float>>::Identity() * k_initial_variance;

		system_model.setCovariance(process_covariance);
	}
};

class DS4KalmanFilterImpl : public KalmanFilterImpl
{
public:
	DS4_MeasurementModel measurement_model;

	void init_measurement_model(const PoseFilterConstants &constants)
	{
		Kalman::Covariance<DS4_MeasurementVector<float>> measurement_covariance = Kalman::Covariance<DS4_MeasurementVector<float>>::Zero();

		// TODO: What should these noise variances actually be?
		measurement_covariance(DS4_OPTICAL_POSITION_X, DS4_OPTICAL_POSITION_X) = 1e-2f; // m_constants.position_constants.optical_position_variance
		measurement_covariance(DS4_OPTICAL_POSITION_Y, DS4_OPTICAL_POSITION_Y) = 1e-2f; // m_constants.position_constants.optical_position_variance
		measurement_covariance(DS4_OPTICAL_POSITION_Z, DS4_OPTICAL_POSITION_Z) = 1e-2f; // m_constants.position_constants.optical_position_variance
		measurement_covariance(DS4_OPTICAL_ORIENTATION_W, DS4_OPTICAL_ORIENTATION_W) = 1e-2f; // m_constants.position_constants.optical_orientation_variance
		measurement_covariance(DS4_OPTICAL_ORIENTATION_X, DS4_OPTICAL_ORIENTATION_X) = 1e-2f; // m_constants.position_constants.optical_orientation_variance 
		measurement_covariance(DS4_OPTICAL_ORIENTATION_Y, DS4_OPTICAL_ORIENTATION_Y) = 1e-2f; // m_constants.position_constants.optical_orientation_variance
		measurement_covariance(DS4_OPTICAL_ORIENTATION_Z, DS4_OPTICAL_ORIENTATION_Z) = 1e-2f; // m_constants.position_constants.optical_orientation_variance

		measurement_model.setCovariance(measurement_covariance);
	}
};

class PSMoveKalmanFilterImpl : public KalmanFilterImpl
{
public:
	PSMove_MeasurementModel measurement_model;

	void init_measurement_model(const PoseFilterConstants &constants)
	{
		Kalman::Covariance<PSMove_MeasurementVector<float>> measurement_covariance = Kalman::Covariance<PSMove_MeasurementVector<float>>::Zero();

		// TODO: What should these noise variances actually be?
		measurement_covariance(PSMOVE_OPTICAL_POSITION_X, PSMOVE_OPTICAL_POSITION_X) = 1e-2f; // m_constants.position_constants.optical_position_variance
		measurement_covariance(PSMOVE_OPTICAL_POSITION_Y, PSMOVE_OPTICAL_POSITION_Y) = 1e-2f; // m_constants.position_constants.optical_position_variance
		measurement_covariance(PSMOVE_OPTICAL_POSITION_Z, PSMOVE_OPTICAL_POSITION_Z) = 1e-2f; // m_constants.position_constants.optical_position_variance
		measurement_covariance(PSMOVE_MAGNETOMETER_X, PSMOVE_MAGNETOMETER_X) = 1e-2f; // m_constants.orientation_constants.magnetometer_variance
		measurement_covariance(PSMOVE_MAGNETOMETER_Y, PSMOVE_MAGNETOMETER_Y) = 1e-2f; // m_constants.orientation_constants.magnetometer_variance
		measurement_covariance(PSMOVE_MAGNETOMETER_Z, PSMOVE_MAGNETOMETER_Z) = 1e-2f; // m_constants.orientation_constants.magnetometer_variance

		measurement_model.setCovariance(measurement_covariance);
		measurement_model.set_magnetometer_identity_direction(constants.orientation_constants.magnetometer_calibration_direction);
	}
};

//-- public interface --
//-- KalmanFilterOpticalPoseARG --
KalmanPoseFilter::KalmanPoseFilter() 
	: m_filter(nullptr)
{
	memset(&m_constants, 0, sizeof(PoseFilterConstants));
}

bool KalmanPoseFilter::init(const PoseFilterConstants &constants)
{
	m_constants = constants;

	// cleanup any existing filter
	if (m_filter != nullptr)
	{
		delete m_filter;
		m_filter;
	}

	return true;
}

bool KalmanPoseFilter::getIsStateValid() const
{
	return m_filter->bIsValid;
}

void KalmanPoseFilter::resetState()
{
	m_filter->init(m_constants);
}

void KalmanPoseFilter::recenterState()
{
	Eigen::Quaternionf q_inverse = getOrientation().conjugate();

	eigen_quaternion_normalize_with_default(q_inverse, Eigen::Quaternionf::Identity());
	m_filter->reset_orientation = q_inverse;
	m_filter->origin_position = getPosition();
}

Eigen::Quaternionf KalmanPoseFilter::getOrientation(float time) const
{
	Eigen::Quaternionf result = Eigen::Quaternionf::Identity();

	if (m_filter->bIsValid)
	{
		const Eigen::Quaternionf state_orientation = m_filter->state_vector.get_orientation();
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

Eigen::Vector3f KalmanPoseFilter::getAngularVelocity() const
{
	return m_filter->state_vector.get_angular_velocity();
}

Eigen::Vector3f KalmanPoseFilter::getAngularAcceleration() const
{
	return Eigen::Vector3f::Zero();
}

Eigen::Vector3f KalmanPoseFilter::getPosition(float time) const
{
	Eigen::Vector3f result = Eigen::Vector3f::Zero();

	if (m_filter->bIsValid)
	{
		Eigen::Vector3f state_position= m_filter->state_vector.get_position();
		Eigen::Vector3f predicted_position =
			is_nearly_zero(time)
			? state_position
			: state_position + getVelocity() * time;

		result = predicted_position - m_filter->origin_position;
		result = result * k_meters_to_centimeters;
	}

	return result;
}

Eigen::Vector3f KalmanPoseFilter::getVelocity() const
{
	return m_filter->state_vector.get_linear_velocity() * k_meters_to_centimeters;
}

Eigen::Vector3f KalmanPoseFilter::getAcceleration() const
{
	return m_filter->state_vector.get_linear_acceleration() * k_meters_to_centimeters;
}

//-- KalmanFilterOpticalPoseARG --
bool KalmanPoseFilterDS4::init(const PoseFilterConstants &constants)
{
	KalmanPoseFilter::init(constants);

	DS4KalmanFilterImpl *filter = new DS4KalmanFilterImpl();
	filter->init(constants);
	filter->init_measurement_model(constants);
	m_filter = filter;

	return true;
}

void KalmanPoseFilterDS4::update(const float delta_time, const PoseFilterPacket &packet)
{
	if (m_filter->bIsValid)
	{
		// Predict state for current time-step using the filters
		m_filter->system_model.set_time_step(delta_time);
		m_filter->state_vector = m_filter->ukf->predict(m_filter->system_model);

		// Get the measurement model for the DS4 from the derived filter impl
		DS4_MeasurementModel &measurement_model = static_cast<DS4KalmanFilterImpl *>(m_filter)->measurement_model;

		// Project the current state onto a predicted measurement as a default
		// in case no observation is available
		DS4_MeasurementVector<float> measurement = measurement_model.h(m_filter->state_vector);

		// Accelerometer and gyroscope measurements are always available
		measurement.set_accelerometer(packet.imu_accelerometer);
		measurement.set_gyroscope(packet.imu_gyroscope);

		if (packet.optical_orientation_quality > 0.f || packet.optical_position_quality > 0.f)
		{
			//Kalman::Covariance<DS4_MeasurementVector<float>> covariance = measurement_model.getCovariance();

			// If available, use the optical orientation measurement
			if (packet.optical_orientation_quality > 0.f)
			{
				//###HipsterSloth $TODO - Update the the orientation quality in the covariance matrix
				//covariance(ORIENTATION_W, ORIENTATION_W) = m_constants.orientation_constants.optical_orientation_variance;
				//covariance(ORIENTATION_X, ORIENTATION_X) = m_constants.orientation_constants.optical_orientation_variance;
				//covariance(ORIENTATION_Y, ORIENTATION_Y) = m_constants.orientation_constants.optical_orientation_variance;
				//covariance(ORIENTATION_Z, ORIENTATION_Z) = m_constants.orientation_constants.optical_orientation_variance;

				measurement.set_optical_orientation(packet.optical_orientation);
			}

			// If available, use the optical position
			if (packet.optical_position_quality > 0.f)
			{
				//###HipsterSloth $TODO - Update the orientation quality in the covariance matrix
				//covariance(POSITION_X, POSITION_X) = m_constants.position_constants.optical_position_variance;
				//covariance(POSITION_Y, POSITION_Y) = m_constants.position_constants.optical_position_variance;
				//covariance(POSITION_Z, POSITION_Z) = m_constants.position_constants.optical_position_variance;

				// State internally stores position in meters
				measurement.set_optical_position(packet.optical_position * k_centimeters_to_meters);
			}

			//measurement_model.setCovariance(covariance);
		}

		// Update UKF
		m_filter->state_vector = m_filter->ukf->update(measurement_model, measurement);
	}
	else if (packet.optical_position_quality > 0.f)
	{
		m_filter->state_vector.setZero();
		m_filter->state_vector.set_position(packet.optical_position * k_centimeters_to_meters);

		if (packet.optical_position_quality > 0.f)
		{
			m_filter->state_vector.set_orientation(packet.optical_orientation);
		}
		else
		{
			m_filter->state_vector.set_orientation(Eigen::Quaternionf::Identity());
		}

		m_filter->bIsValid= true;
	}
}

//-- PSMovePoseKalmanFilter --
bool PSMovePoseKalmanFilter::init(const PoseFilterConstants &constants)
{
	KalmanPoseFilter::init(constants);

	PSMoveKalmanFilterImpl *filter = new PSMoveKalmanFilterImpl();
	filter->init(constants);
	filter->init_measurement_model(constants);
	m_filter = filter;

	return true;
}

void PSMovePoseKalmanFilter::update(const float delta_time, const PoseFilterPacket &packet)
{
	if (m_filter->bIsValid)
	{
		// Predict state for current time-step using the filters
		m_filter->state_vector = m_filter->ukf->predict(m_filter->system_model);

		// Get the measurement model for the PSMove from the derived filter impl
		PSMove_MeasurementModel &measurement_model = static_cast<PSMoveKalmanFilterImpl *>(m_filter)->measurement_model;

		// Project the current state onto a predicted measurement as a default
		// in case no observation is available
		PSMove_MeasurementVector<float> measurement = measurement_model.h(m_filter->state_vector);

		// Accelerometer, magnetometer and gyroscope measurements are always available
		measurement.set_accelerometer(packet.imu_accelerometer);
		measurement.set_gyroscope(packet.imu_gyroscope);
		measurement.set_magnetometer(packet.imu_magnetometer);

		// If available, use the optical position
		if (packet.optical_position_quality > 0.f)
		{
			//###HipsterSloth $TODO - Feed the position quality into the covariance matrix
			//Kalman::Covariance<PSMove_MeasurementVector<float><float>> covariance = measurement_model.getCovariance();
			//covariance(POSITION_X, POSITION_X) = m_constants.position_constants.optical_position_variance;
			//covariance(POSITION_Y, POSITION_Y) = m_constants.position_constants.optical_position_variance;
			//covariance(POSITION_Z, POSITION_Z) = m_constants.position_constants.optical_position_variance;
			//measurement_model.setCovariance(covariance);
			measurement.set_optical_position(packet.optical_position);
		}

		// Update UKF
		m_filter->state_vector = m_filter->ukf->update(measurement_model, measurement);
	}
	else if (packet.optical_position_quality > 0.f)
	{
		m_filter->state_vector.setZero();
		m_filter->state_vector.set_position(packet.optical_position * k_centimeters_to_meters);
		m_filter->state_vector.set_orientation(Eigen::Quaternionf::Identity());

		m_filter->bIsValid= true;
	}
}
//-- includes --
#include "KalmanPoseFilter.h"

#include <kalman/LinearizedMeasurementModel.hpp>
#include <kalman/LinearizedSystemModel.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>

//-- private definitions --
enum StateEnum
{
	POSITION_X,
	POSITION_Y,
	POSITION_Z,
	LINEAR_VELOCITY_X,
	LINEAR_VELOCITY_Y,
	LINEAR_VELOCITY_Z,
	LINEAR_ACCELERATION_X,
	LINEAR_ACCELERATION_Y,
	LINEAR_ACCELERATION_Z,
	ORIENTATION_W,
	ORIENTATION_X,
	ORIENTATION_Y,
	ORIENTATION_Z,
	ANGULAR_VELOCITY_X,
	ANGULAR_VELOCITY_Y,
	ANGULAR_VELOCITY_Z,
	ANGULAR_ACCELERATION_X,
	ANGULAR_ACCELERATION_Y,
	ANGULAR_ACCELERATION_Z,
	GYRO_BIAS_X,
	GYRO_BIAS_Y,
	GYRO_BIAS_Z,

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
	Eigen::Vector3f get_angular_acceleration() const {
		return Eigen::Vector3f((*this)[ANGULAR_ACCELERATION_X], (*this)[ANGULAR_ACCELERATION_Y], (*this)[ANGULAR_ACCELERATION_Z]);
	}
	Eigen::Vector3f get_gyro_bias() const {
		return Eigen::Vector3f((*this)[GYRO_BIAS_X], (*this)[GYRO_BIAS_Y], (*this)[GYRO_BIAS_Z]);
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
	void set_angular_acceleration(const Eigen::Vector3f &a) {
		(*this)[ANGULAR_ACCELERATION_X] = a.x(); (*this)[ANGULAR_ACCELERATION_Y] = a.y(); (*this)[ANGULAR_ACCELERATION_Z] = a.z();
	}
	void set_gyro_bias(const Eigen::Vector3f &b) {
		(*this)[GYRO_BIAS_X] = b.x(); (*this)[GYRO_BIAS_Y] = b.y(); (*this)[GYRO_BIAS_Z] = b.z();
	}
};

enum ControlEnum {
	CONTROL_TIME_DELTA,
	CONTROL_ACCELEROMETER_X,
	CONTROL_ACCELEROMETER_Y,
	CONTROL_ACCELEROMETER_Z,
	CONTROL_GYROSCOPE_X,
	CONTROL_GYROSCOPE_Y,
	CONTROL_GYROSCOPE_Z,

	CONTROL_PARAMETER_COUNT
};

template<typename T>
class ControlVector : public Kalman::Vector<T, CONTROL_PARAMETER_COUNT>
{
public:
	KALMAN_VECTOR(ControlVector, T, CONTROL_PARAMETER_COUNT)

	// Accessors
	float get_time_delta() const {
		return (*this)[CONTROL_TIME_DELTA];
	}
	Eigen::Vector3f get_world_accelerometer() const {
		return Eigen::Vector3f((*this)[CONTROL_ACCELEROMETER_X], (*this)[CONTROL_ACCELEROMETER_Y], (*this)[CONTROL_ACCELEROMETER_Z]);
	}
	Eigen::Vector3f get_gyroscope() const {
		return Eigen::Vector3f((*this)[CONTROL_GYROSCOPE_X], (*this)[CONTROL_GYROSCOPE_Y], (*this)[CONTROL_GYROSCOPE_Z]);
	}

	// Mutators
	void set_time_delta(float time_delta) {
		(*this)[CONTROL_TIME_DELTA] = time_delta;
	}
	void set_accelerometer(const Eigen::Vector3f &a) {
		(*this)[CONTROL_ACCELEROMETER_X] = a.x(); (*this)[CONTROL_ACCELEROMETER_Y] = a.y(); (*this)[CONTROL_ACCELEROMETER_Z] = a.z();
	}
	void set_gyroscope(const Eigen::Vector3f &g) {
		(*this)[CONTROL_GYROSCOPE_X] = g.x(); (*this)[CONTROL_GYROSCOPE_Y] = g.y(); (*this)[CONTROL_GYROSCOPE_Z] = g.z();
	}
};

enum PSMoveMeasurementEnum {
	PSMOVE_OPTICAL_POSITION_X,
	PSMOVE_OPTICAL_POSITION_Y,
	PSMOVE_OPTICAL_POSITION_Z,
	PSMOVE_MAGNETOMETER_X,
	PSMOVE_MAGNETOMETER_Y,
	PSMOVE_MAGNETOMETER_Z,

	PSMOVE_MEASUREMENT_PARAMETER_COUNT
};

template<typename T>
class PSMove_MeasurementVector : public Kalman::Vector<T, PSMOVE_MEASUREMENT_PARAMETER_COUNT>
{
public:
	KALMAN_VECTOR(PSMove_MeasurementVector, T, PSMOVE_MEASUREMENT_PARAMETER_COUNT)

	// Accessors
	Eigen::Vector3f get_optical_position() const {
		return Eigen::Vector3f((*this)[PSMOVE_OPTICAL_POSITION_X], (*this)[PSMOVE_OPTICAL_POSITION_Y], (*this)[PSMOVE_OPTICAL_POSITION_Z]);
	}
	Eigen::Vector3f get_magnetometer() const {
		return Eigen::Vector3f((*this)[PSMOVE_MAGNETOMETER_X], (*this)[PSMOVE_MAGNETOMETER_Y], (*this)[PSMOVE_MAGNETOMETER_Z]);
	}

	// Mutators
	void set_optical_position(const Eigen::Vector3f &p) {
		(*this)[PSMOVE_OPTICAL_POSITION_X] = p.x(); (*this)[PSMOVE_OPTICAL_POSITION_Y] = p.y(); (*this)[PSMOVE_OPTICAL_POSITION_Z] = p.z();
	}
	void set_magnetometer(const Eigen::Vector3f &m) {
		(*this)[PSMOVE_MAGNETOMETER_X] = m.x(); (*this)[PSMOVE_MAGNETOMETER_Y] = m.y(); (*this)[PSMOVE_MAGNETOMETER_Z] = m.z();
	}
};

enum DS4MeasurementEnum {
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
	Eigen::Vector3f get_optical_position() const {
		return Eigen::Vector3f((*this)[DS4_OPTICAL_POSITION_X], (*this)[DS4_OPTICAL_POSITION_Y], (*this)[DS4_OPTICAL_POSITION_Z]);
	}
	Eigen::Quaternionf get_optical_orientation() const {
		return Eigen::Quaternionf((*this)[DS4_OPTICAL_ORIENTATION_W], (*this)[DS4_OPTICAL_ORIENTATION_X], (*this)[DS4_OPTICAL_ORIENTATION_Y], (*this)[DS4_OPTICAL_ORIENTATION_Z]);
	}

	// Mutators
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
class SystemModel : public Kalman::LinearizedSystemModel<StateVector<float>, ControlVector<float>, Kalman::StandardBase>
{
public:
	inline void set_gravity_identity_direction(const Eigen::Vector3f &gravity) { m_identity_gravity = gravity; }
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
	StateVector<float> f(const StateVector<float>& old_state, const ControlVector<float>& control) const
	{
		//! Predicted state vector after transition
		StateVector<float> new_state;

		// Get the control parameters
		const float delta_time = control.get_time_delta();
		const Eigen::Vector3f new_accelerometer = control.get_world_accelerometer();
		const Eigen::Vector3f new_angular_velocity = control.get_gyroscope();

		// Extract parameters from the old state
		const Eigen::Quaternionf old_orientation = old_state.get_orientation();
		const Eigen::Vector3f old_angular_velocity = old_state.get_angular_velocity();
		const Eigen::Vector3f old_position = old_state.get_position();
		const Eigen::Vector3f old_linear_velocity = old_state.get_linear_velocity();

		// Compute the result of applying the angular velocity control to the state
		const Eigen::Quaternionf &quaternion_derivative =
			eigen_angular_velocity_to_quaternion_derivative(old_orientation, new_angular_velocity);
		//###HipsterSloth $TODO - We can do better then Euler integration
		//###HipsterSloth $TODO - Gyro Bias?
		const Eigen::Quaternionf new_orientation = 
			Eigen::Quaternionf(old_orientation.coeffs() + quaternion_derivative.coeffs()*delta_time).normalized();
		const Eigen::Vector3f new_angular_acceleration = (new_angular_velocity - old_angular_velocity) / delta_time;

		// Compute the result of applying the accelerometer control to the state
		//###HipsterSloth $REVIEW is this the right way to subtract out gravity?
		const Eigen::Vector3f gravity = eigen_vector3f_clockwise_rotate(new_orientation, m_identity_gravity);
		const Eigen::Vector3f new_linear_acceleration = (new_accelerometer - gravity) * k_g_units_to_ms2;
		//###HipsterSloth $TODO - We can do better then Euler integration
		const Eigen::Vector3f new_linear_velocity = old_linear_velocity + new_linear_acceleration*delta_time;
		const Eigen::Vector3f new_position = old_position + new_linear_velocity*delta_time;

		// Save results to the new state
		new_state.set_orientation(new_orientation);
		new_state.set_angular_velocity(new_angular_velocity);
		new_state.set_angular_acceleration(new_angular_acceleration);
		new_state.set_position(new_position);
		new_state.set_linear_velocity(new_linear_velocity);
		new_state.set_linear_acceleration(new_linear_acceleration);
		//###HipsterSloth $TODO
		new_state.set_gyro_bias(Eigen::Vector3f::Zero());

		return new_state;
	}

protected:
	Eigen::Vector3f m_identity_gravity;
};

/**
* @brief Measurement model for measuring PSMove controller
*
* This is the measurement model for measuring the position and magnetometer of the PSMove controller.
* The measurement is given by the optical trackers.
*/
class PSMove_MeasurementModel : public Kalman::LinearizedMeasurementModel<StateVector<float>, PSMove_MeasurementVector<float>, Kalman::StandardBase>
{
public:
	inline void set_magnetometer_identity_direction(const Eigen::Vector3f &magnetometer) { m_identity_magnetometer = magnetometer; }

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

		predicted_measurement.set_optical_position(x.get_position());
		predicted_measurement.set_magnetometer(eigen_vector3f_clockwise_rotate(x.get_orientation(), m_identity_magnetometer));

		return predicted_measurement;
	}

protected:
	Eigen::Vector3f m_identity_magnetometer;
};

/**
* @brief Measurement model for measuring DS4 controller
*
* This is the measurement model for measuring the position and orientation of the DS4 controller.
* The measurement is given by the optical trackers.
*/
class DS4_MeasurementModel : public Kalman::LinearizedMeasurementModel<StateVector<float>, DS4_MeasurementVector<float>, Kalman::StandardBase>
{
public:
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

		predicted_measurement.set_optical_position(x.get_position());
		predicted_measurement.set_optical_orientation(x.get_orientation());

		return predicted_measurement;
	}
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
	StateVector<float> state_vector_ukf;

	/// Unscented Kalman Filter instance
	Kalman::UnscentedKalmanFilter<StateVector<float>> *ukf;

	KalmanFilterImpl()
	{
		ukf = new Kalman::UnscentedKalmanFilter<StateVector<float>>(1);
	}

	virtual ~KalmanFilterImpl()
	{
		delete ukf;
	}

	virtual void reset()
	{
		bIsValid = false;
		reset_orientation = Eigen::Quaternionf::Identity();
		origin_position = Eigen::Vector3f::Zero();
		state_vector.setZero();
		state_vector_ukf.setZero();
		ukf->init(state_vector);
	}
};

//-- public interface --
//-- KalmanFilterOpticalPoseARG --
KalmanPoseFilter::KalmanPoseFilter() 
	: m_filter(new KalmanFilterImpl)
{
	memset(&m_constants, 0, sizeof(PoseFilterConstants));
}

bool KalmanPoseFilter::init(const PoseFilterConstants &constants)
{
	resetState();
	m_constants = constants;
	m_filter->reset();

	return true;
}

bool KalmanPoseFilter::getIsStateValid() const
{
	return m_filter->bIsValid;
}

void KalmanPoseFilter::resetState()
{
	m_filter->reset();
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
		const Eigen::Quaternionf state_orientation = m_filter->state_vector_ukf.get_orientation();
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
	return m_filter->state_vector_ukf.get_angular_velocity();
}

Eigen::Vector3f KalmanPoseFilter::getAngularAcceleration() const
{
	return m_filter->state_vector_ukf.get_angular_acceleration();
}

Eigen::Vector3f KalmanPoseFilter::getPosition(float time) const
{
	Eigen::Vector3f result = Eigen::Vector3f::Zero();

	if (m_filter->bIsValid)
	{
		Eigen::Vector3f state_position= m_filter->state_vector_ukf.get_position();
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
	return m_filter->state_vector_ukf.get_linear_velocity() * k_meters_to_centimeters;
}

Eigen::Vector3f KalmanPoseFilter::getAcceleration() const
{
	return m_filter->state_vector_ukf.get_linear_acceleration() * k_meters_to_centimeters;
}

//-- KalmanFilterOpticalPoseARG --
void KalmanPoseFilterDS4::update(const float delta_time, const PoseFilterPacket &packet)
{
	// Treat the accelerometer and gyroscope as control imputs for the physics
	ControlVector<float> control_vector;
	control_vector.set_time_delta(delta_time);
	control_vector.set_accelerometer(packet.world_accelerometer);
	control_vector.set_gyroscope(packet.imu_gyroscope);

	// Simulate the physics of the system
	SystemModel system_model;
	system_model.set_gravity_identity_direction(packet.gravity_calibration_direction);
	m_filter->state_vector = system_model.f(m_filter->state_vector, control_vector);

	// Predict state for current time-step using the filters
	m_filter->state_vector_ukf = m_filter->ukf->predict(system_model, control_vector);

	// Project the current state onto a predicted measurement
	DS4_MeasurementModel measurement_model;
	DS4_MeasurementVector<float> measurement = measurement_model.h(m_filter->state_vector);

	// If we have an optical orientation, use that instead
	//###HipsterSloth $TODO - How do we feed the the orientation quality into the covariances
	if (packet.optical_orientation_quality > 0.f)
	{
		measurement.set_optical_orientation(packet.optical_orientation);
	}

	// If we have an optical position, use that instead
	//###HipsterSloth $TODO - How do we feed the the position quality into the covariances
	if (packet.optical_position_quality > 0.f)
	{
		measurement.set_optical_position(packet.optical_position);
	}

	// Update UKF
	m_filter->state_vector_ukf = m_filter->ukf->update(measurement_model, measurement);
}

//-- PSMovePoseKalmanFilter --
void PSMovePoseKalmanFilter::update(const float delta_time, const PoseFilterPacket &packet)
{
	// Control input
	ControlVector<float> control_vector;
	control_vector.set_time_delta(delta_time);
	control_vector.set_accelerometer(packet.world_accelerometer);
	control_vector.set_gyroscope(packet.imu_gyroscope);

	// System
	SystemModel system_model;
	system_model.set_gravity_identity_direction(packet.gravity_calibration_direction);

	// Project the current state onto a predicted measurement
	PSMove_MeasurementModel measurement_model;
	measurement_model.set_magnetometer_identity_direction(packet.magnetometer_calibration_direction);
	PSMove_MeasurementVector<float> measurement = measurement_model.h(m_filter->state_vector);

	measurement.set_magnetometer(packet.imu_magnetometer);

	// If we have an optical position, use that instead
	//###HipsterSloth $TODO - How do we feed the the position quality into the covariances
	if (packet.optical_position_quality > 0.f)
	{
		measurement.set_optical_position(packet.optical_position);
	}

	// Update UKF
	m_filter->state_vector_ukf = m_filter->ukf->update(measurement_model, measurement);
}
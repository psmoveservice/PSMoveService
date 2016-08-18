//-- includes --
#include "KalmanPoseFilter.h"
#include "MathAlignment.h"

#include <kalman/MeasurementModel.hpp>
#include <kalman/SystemModel.hpp>
#include <kalman/SquareRootUnscentedKalmanFilter.hpp>

//-- constants --
enum StateEnum
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
    ANGLE_AXIS_X,  // axis * radians
    ANGULAR_VELOCITY_X, // rad/s
    ANGLE_AXIS_Y,
    ANGULAR_VELOCITY_Y,
    ANGLE_AXIS_Z,
    ANGULAR_VELOCITY_Z,

    STATE_PARAMETER_COUNT
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
    DS4_OPTICAL_ANGLE_AXIS_X,
    DS4_OPTICAL_ANGLE_AXIS_Y,
    DS4_OPTICAL_ANGLE_AXIS_Z,

    DS4_MEASUREMENT_PARAMETER_COUNT
};

// From: http://nbviewer.jupyter.org/github/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/10-Unscented-Kalman-Filter.ipynb#Reasonable-Choices-for-the-Parameters
// beta=2 is a good choice for Gaussian problems, 
// kappa=3-n where n is the size of x is a good choice for kappa, 
// 0<=alpha<=1 is an appropriate choice for alpha, 
// where a larger value for alpha spreads the sigma points further from the mean.
#define k_ukf_alpha 1.f
#define k_ukf_beta 2.f
#define k_ukf_kappa -1.f

//-- private methods ---
template <class StateType>
void Q_discrete_3rd_order_white_noise(const float dT, const float var, const int state_index, Kalman::Covariance<StateType> &Q);

template <class StateType>
void Q_discrete_2nd_order_white_noise(const float dT, const float var, const int state_index, Kalman::Covariance<StateType> &Q);

//-- private definitions --
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
    Eigen::AngleAxisf get_angle_axis() const {
        Eigen::Vector3f axis= Eigen::Vector3f((*this)[ANGLE_AXIS_X], (*this)[ANGLE_AXIS_Y], (*this)[ANGLE_AXIS_Z]);
        const float angle= eigen_vector3f_normalize_with_default(axis, Eigen::Vector3f::Zero());
        return Eigen::AngleAxisf(angle, axis);
    }
    Eigen::Quaternionf get_quaternion() const {
        return Eigen::Quaternionf(get_angle_axis());
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
    void set_angle_axis(const Eigen::AngleAxisf &a) {        
        const float angle= a.angle();
        (*this)[ANGLE_AXIS_X] = a.axis().x() * angle; 
        (*this)[ANGLE_AXIS_Y] = a.axis().y() * angle;
        (*this)[ANGLE_AXIS_Z] = a.axis().z() * angle;
    }
    void set_quaternion(const Eigen::Quaternionf &q) {
        const Eigen::AngleAxisf angle_axis(q);
        set_angle_axis(angle_axis);
    }
    void set_angular_velocity(const Eigen::Vector3f &v) {
        (*this)[ANGULAR_VELOCITY_X] = v.x(); (*this)[ANGULAR_VELOCITY_Y] = v.y(); (*this)[ANGULAR_VELOCITY_Z] = v.z();
    }

	StateVector addStateDelta(const StateVector &stateDelta)
	{
		// Do the default additive delta first
		StateVector result = (*this) + stateDelta;

		// Extract the orientation quaternion from this state (which is stored as an angle axis vector)
		const Eigen::Quaternionf orientation = this->get_quaternion();

		// Extract the delta quaternion (which is also stored as an angle axis vector)
		const Eigen::Quaternionf delta = stateDelta.get_quaternion();

		// Apply the delta to the orientation
		const Eigen::Quaternionf new_rotation = delta*orientation;

		// Stomp over the simple addition of the angle axis result
		result.set_quaternion(new_rotation);

		return result;
	}
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
    Eigen::AngleAxisf get_optical_angle_axis() const {
        Eigen::Vector3f axis= Eigen::Vector3f((*this)[DS4_OPTICAL_ANGLE_AXIS_X], (*this)[DS4_OPTICAL_ANGLE_AXIS_Y], (*this)[DS4_OPTICAL_ANGLE_AXIS_Z]);
        const float angle= eigen_vector3f_normalize_with_default(axis, Eigen::Vector3f::Zero());
        return Eigen::AngleAxisf(angle, axis);
    }
    Eigen::Quaternionf get_optical_quaternion() const {
        return Eigen::Quaternionf(get_optical_angle_axis());
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
    void set_angle_axis(const Eigen::AngleAxisf &a) {        
        const float angle= a.angle();
        (*this)[DS4_OPTICAL_ANGLE_AXIS_X] = a.axis().x() * angle; 
        (*this)[DS4_OPTICAL_ANGLE_AXIS_Y] = a.axis().y() * angle;
        (*this)[DS4_OPTICAL_ANGLE_AXIS_Z] = a.axis().z() * angle;
    }
    void set_optical_quaternion(const Eigen::Quaternionf &q) {
        const Eigen::AngleAxisf angle_axis(q);
        set_angle_axis(angle_axis);
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

    void init(const PoseFilterConstants &constants)
    {
        const float mean_position_dT= constants.position_constants.mean_update_time_delta;
        const float mean_orientation_dT= constants.position_constants.mean_update_time_delta;

        // Start off using the maximum variance values
        const float position_variance= constants.position_constants.max_position_variance;
		const float angle_axis_variance= constants.orientation_constants.max_orientation_variance;

        // Initialize the process covariance matrix Q
        Kalman::Covariance<StateVector<float>> Q = Kalman::Covariance<StateVector<float>>::Zero();
        Q_discrete_3rd_order_white_noise<StateVector<float>>(mean_position_dT, position_variance, POSITION_X, Q);
		Q_discrete_3rd_order_white_noise<StateVector<float>>(mean_position_dT, position_variance, POSITION_Y, Q);
		Q_discrete_3rd_order_white_noise<StateVector<float>>(mean_position_dT, position_variance, POSITION_Z, Q);
		Q_discrete_2nd_order_white_noise<StateVector<float>>(mean_orientation_dT, angle_axis_variance, ANGLE_AXIS_X, Q);
		Q_discrete_2nd_order_white_noise<StateVector<float>>(mean_orientation_dT, angle_axis_variance, ANGLE_AXIS_Y, Q);
		Q_discrete_2nd_order_white_noise<StateVector<float>>(mean_orientation_dT, angle_axis_variance, ANGLE_AXIS_Z, Q);
        setCovariance(Q);
    }

	void update_process_covariance(
		const PoseFilterConstants &constants,
		const float position_quality,
		const float orientation_quality)
	{
        const float mean_position_dT= constants.position_constants.mean_update_time_delta;
        const float mean_orientation_dT= constants.position_constants.mean_update_time_delta;

        // Start off using the maximum variance values
        const float position_variance= 
			lerp_clampf(
				constants.position_constants.max_position_variance,
				constants.position_constants.min_position_variance,
				position_quality);
		const float angle_axis_variance=
			lerp_clampf(
				constants.orientation_constants.max_orientation_variance,
				constants.orientation_constants.min_orientation_variance,
				orientation_quality);

        // Initialize the process covariance matrix Q
        Kalman::Covariance<StateVector<float>> Q = getCovariance();
        Q_discrete_3rd_order_white_noise<StateVector<float>>(mean_position_dT, position_variance, POSITION_X, Q);
		Q_discrete_3rd_order_white_noise<StateVector<float>>(mean_position_dT, position_variance, POSITION_Y, Q);
		Q_discrete_3rd_order_white_noise<StateVector<float>>(mean_position_dT, position_variance, POSITION_Z, Q);
		Q_discrete_2nd_order_white_noise<StateVector<float>>(mean_orientation_dT, angle_axis_variance, ANGLE_AXIS_X, Q);
		Q_discrete_2nd_order_white_noise<StateVector<float>>(mean_orientation_dT, angle_axis_variance, ANGLE_AXIS_Y, Q);
		Q_discrete_2nd_order_white_noise<StateVector<float>>(mean_orientation_dT, angle_axis_variance, ANGLE_AXIS_Z, Q);
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
    StateVector<float> f(const StateVector<float>& old_state, const Kalman::Vector<float, 0>& control) const
    {
        //! Predicted state vector after transition
        StateVector<float> new_state;

        // Extract parameters from the old state
        const Eigen::Vector3f old_position = old_state.get_position();
        const Eigen::Vector3f old_linear_velocity = old_state.get_linear_velocity();
        const Eigen::Vector3f old_linear_acceleration = old_state.get_linear_acceleration();
        const Eigen::Quaternionf old_orientation = old_state.get_quaternion();
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
        new_state.set_quaternion(new_orientation);
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
    void init(const PoseFilterConstants &constants)
    {
        Kalman::Covariance<PSMove_MeasurementVector<float>> R = 
			Kalman::Covariance<PSMove_MeasurementVector<float>>::Zero();

        const float position_variance= constants.position_constants.max_position_variance;
		const float magnetometer_variance= constants.orientation_constants.magnetometer_variance;

        R(PSMOVE_OPTICAL_POSITION_X, PSMOVE_OPTICAL_POSITION_X) = position_variance;
        R(PSMOVE_OPTICAL_POSITION_Y, PSMOVE_OPTICAL_POSITION_Y) = position_variance;
        R(PSMOVE_OPTICAL_POSITION_Z, PSMOVE_OPTICAL_POSITION_Z) = position_variance;
        R(PSMOVE_MAGNETOMETER_X, PSMOVE_MAGNETOMETER_X) = magnetometer_variance;
        R(PSMOVE_MAGNETOMETER_Y, PSMOVE_MAGNETOMETER_Y) = magnetometer_variance;
        R(PSMOVE_MAGNETOMETER_Z, PSMOVE_MAGNETOMETER_Z) = magnetometer_variance;

        setCovariance(R);
        
		m_identity_gravity_direction= constants.orientation_constants.gravity_calibration_direction;
		m_identity_magnetometer_direction= constants.orientation_constants.magnetometer_calibration_direction;
    }

	void update_measurement_covariance(
		const PoseFilterConstants &constants,
		const float position_quality)
	{
        // Start off using the maximum variance values
        const float position_variance= 
			lerp_clampf(
				constants.position_constants.max_position_variance,
				constants.position_constants.min_position_variance,
				position_quality);

        // Update the measurement covariance R
        Kalman::Covariance<PSMove_MeasurementVector<float>> R = getCovariance();
        R(PSMOVE_OPTICAL_POSITION_X, PSMOVE_OPTICAL_POSITION_X) = position_variance;
        R(PSMOVE_OPTICAL_POSITION_Y, PSMOVE_OPTICAL_POSITION_Y) = position_variance;
        R(PSMOVE_OPTICAL_POSITION_Z, PSMOVE_OPTICAL_POSITION_Z) = position_variance;
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
    PSMove_MeasurementVector<float> h(const StateVector<float>& x) const
    {
        PSMove_MeasurementVector<float> predicted_measurement;

        // Use the position and orientation from the state for predictions
        const Eigen::Vector3f position= x.get_position();
        const Eigen::Quaternionf orientation= x.get_quaternion();

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
    void init(const PoseFilterConstants &constants)
    {
        Kalman::Covariance<DS4_MeasurementVector<float>> measurement_covariance = 
			Kalman::Covariance<DS4_MeasurementVector<float>>::Zero();

        update_measurement_covariance(constants, 0.f, 0.f);

		m_identity_gravity_direction= constants.orientation_constants.gravity_calibration_direction;
    }

	void update_measurement_covariance(
		const PoseFilterConstants &constants,
		const float position_quality,
		const float orientation_quality)
	{
        // Start off using the maximum variance values
        const float position_variance= 
			lerp_clampf(
				constants.position_constants.max_position_variance,
				constants.position_constants.min_position_variance,
				position_quality);
		const float angle_axis_variance=
			lerp_clampf(
				constants.orientation_constants.max_orientation_variance,
				constants.orientation_constants.min_orientation_variance,
				orientation_quality);

        // Update the measurement covariance R
        Kalman::Covariance<DS4_MeasurementVector<float>> R = 
			Kalman::Covariance<DS4_MeasurementVector<float>>::Zero();
        R(PSMOVE_OPTICAL_POSITION_X, PSMOVE_OPTICAL_POSITION_X) = position_variance;
        R(PSMOVE_OPTICAL_POSITION_Y, PSMOVE_OPTICAL_POSITION_Y) = position_variance;
        R(PSMOVE_OPTICAL_POSITION_Z, PSMOVE_OPTICAL_POSITION_Z) = position_variance;
        R(DS4_OPTICAL_ANGLE_AXIS_X, DS4_OPTICAL_ANGLE_AXIS_X) = angle_axis_variance; 
        R(DS4_OPTICAL_ANGLE_AXIS_Y, DS4_OPTICAL_ANGLE_AXIS_Y) = angle_axis_variance;
        R(DS4_OPTICAL_ANGLE_AXIS_Z, DS4_OPTICAL_ANGLE_AXIS_Z) = angle_axis_variance;
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
    DS4_MeasurementVector<float> h(const StateVector<float>& x) const
    {
        DS4_MeasurementVector<float> predicted_measurement;

        // Use the position and orientation from the state for predictions
        const Eigen::Vector3f position= x.get_position();
        const Eigen::Quaternionf orientation= x.get_quaternion();

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
        predicted_measurement.set_optical_quaternion(orientation);

        return predicted_measurement;
    }

protected:
    Eigen::Vector3f m_identity_gravity_direction;
};

class CustomSRUFK : public Kalman::SquareRootUnscentedKalmanFilter<StateVector<float>>
{
public:
	CustomSRUFK(float alpha, float beta, float kappa)
		: Kalman::SquareRootUnscentedKalmanFilter<StateVector<float>>(alpha, beta, kappa)
	{ }

protected:
	/**
	* @brief Compute sigma points from current state estimate and state covariance
	*
	* @note This covers equations (17) and (22) of Algorithm 3.1 in the Paper
	*/
	bool computeSigmaPoints()
	{
		// Get square root of covariance
		Kalman::Matrix<float, StateVector<float>::RowsAtCompileTime, StateVector<float>::RowsAtCompileTime> _S = S.matrixL().toDenseMatrix();

		// Set left "block" (first column)
		sigmaStatePoints.template leftCols<1>() = x;

		// Apply the state delta column by column
		for (int col_index = 1; col_index <= StateVector<float>::RowsAtCompileTime; ++col_index)
		{
			// Set center block with x + gamma * S
			sigmaStatePoints.col(col_index) = x.addStateDelta(this->gamma * _S.col(col_index));

			// Set right block with x - gamma * S
			sigmaStatePoints.col(col_index+StateVector<float>::RowsAtCompileTime) = x.addStateDelta(-this->gamma * _S.col(col_index));
		}

		return true;
	}

	/**
	* @brief Compute state prediction from sigma points using pre-computed sigma weights
	*
	* @note This covers equations (19) and (24) of Algorithm 3.1 in the Paper
	*
	* @param [in] sigmaPoints The computed sigma points of the desired type (state or measurement)
	* @return The prediction
	*/
	StateVector<float> computeStatePredictionFromSigmaPoints(const SigmaPoints<StateVector<float>>& sigmaPoints)
	{
		// Use efficient matrix x vector computation to compute a weighted average of the sigma point samples
		// (the orientation portion will be wrong)
		StateVector<float> result = sigmaPoints * sigmaWeights_m;

		// Extract the sample orientations from the sigma point orientations
		Eigen::Quaternionf samples[SigmaPoints<StateVector<float>>::ColsAtCompileTime];
		float weights[SigmaPoints<StateVector<float>>::ColsAtCompileTime];
		for (int col_index = 0; col_index <= SigmaPoints<StateVector<float>>::ColsAtCompileTime; ++col_index)
		{
			const StateVector<float> sigmaPoint= sigmaPoints.col(col_index);
			Eigen::Quaternionf sigmaOrientation= sigmaPoint.get_quaternion();
		}

		// Compute the average of the quaternions
		Eigen::Quaternionf average_quat;
		eigen_quaternion_compute_weighted_average(samples, weights, SigmaPoints<StateVector<float>>::ColsAtCompileTime, &average_quat);

		// Stomp the incorrect orientation average
		result.set_quaternion(average_quat);

		return result;
	}

	/**
	* @brief Compute measurement prediction from sigma points using pre-computed sigma weights
	*
	* @note This covers equations (19) and (24) of Algorithm 3.1 in the Paper
	*
	* @param [in] sigmaPoints The computed sigma points of the desired type (state or measurement)
	* @return The prediction
	*/
	DS4_MeasurementVector<float> computeMeasurementPredictionFromSigmaPoints(const SigmaPoints<DS4_MeasurementVector<float>>& sigmaPoints)
	{
		// Use efficient matrix x vector computation to compute a weighted average of the sigma point samples
		// (the orientation portion will be wrong)
		DS4_MeasurementVector<float> result= sigmaPoints * sigmaWeights_m;

		// Extract the sample orientations from the sigma point orientations
		Eigen::Quaternionf samples[SigmaPoints<DS4_MeasurementVector<float>>::ColsAtCompileTime];
		float weights[SigmaPoints<DS4_MeasurementVector<float>>::ColsAtCompileTime];
		for (int col_index = 0; col_index <= SigmaPoints<DS4_MeasurementVector<float>>::ColsAtCompileTime; ++col_index)
		{
			const DS4_MeasurementVector<float> sigmaPoint = sigmaPoints.col(col_index);
			Eigen::Quaternionf sigmaOrientation = sigmaPoint.get_optical_quaternion();
		}

		// Compute the average of the quaternions
		Eigen::Quaternionf average_quat;
		eigen_quaternion_compute_weighted_average(samples, weights, SigmaPoints<DS4_MeasurementVector<float>>::ColsAtCompileTime, &average_quat);

		// Stomp the incorrect orientation average
		result.set_optical_quaternion(average_quat);

		return result;
	}

	/**
	* @brief Compute predicted state using system model and control input
	*
	* @param [in] s The System Model
	* @param [in] u The Control input
	* @return The predicted state
	*/
	template<class Control, template<class> class CovarianceBase>
	State computeStatePrediction(const SystemModelType<Control, CovarianceBase>& s, const Control& u)
	{
		// Pass each sigma point through non-linear state transition function
		computeSigmaPointTransition(s, u);

		// Compute predicted state from predicted sigma points
		return computeStatePredictionFromSigmaPoints(sigmaStatePoints);
	}

	/**
	* @brief Compute predicted measurement using measurement model and predicted sigma measurements
	*
	* @param [in] m The Measurement Model
	* @param [in] sigmaMeasurementPoints The predicted sigma measurement points
	* @return The predicted measurement
	*/
	template<class Measurement, template<class> class CovarianceBase>
	Measurement computeMeasurementPrediction(const MeasurementModelType<Measurement, CovarianceBase>& m, SigmaPoints<Measurement>& sigmaMeasurementPoints)
	{
		// Predict measurements for each sigma point
		computeSigmaPointMeasurements<Measurement>(m, sigmaMeasurementPoints);

		// Predict measurement from sigma measurement points
		return computeMeasurementPredictionFromSigmaPoints(sigmaMeasurementPoints);
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

    /// Used to model how the physics of the controller evolves
    SystemModel system_model;

    /// Unscented Kalman Filter instance
	CustomSRUFK ukf;

    KalmanFilterImpl() 
		: ukf(k_ukf_alpha, k_ukf_beta, k_ukf_kappa)
    {
    }

    virtual void init(const PoseFilterConstants &constants)
    {
        bIsValid = false;
        reset_orientation = Eigen::Quaternionf::Identity();
        origin_position = Eigen::Vector3f::Zero();

        state_vector.setZero();

        system_model.init(constants);
        ukf.init(state_vector);
    }
};

class DS4KalmanFilterImpl : public KalmanFilterImpl
{
public:
    DS4_MeasurementModel measurement_model;

	void init(const PoseFilterConstants &constants) override
	{
		KalmanFilterImpl::init(constants);
		measurement_model.init(constants);
	}
};

class PSMoveKalmanFilterImpl : public KalmanFilterImpl
{
public:
    PSMove_MeasurementModel measurement_model;

	void init(const PoseFilterConstants &constants) override
	{
		KalmanFilterImpl::init(constants);
		measurement_model.init(constants);
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
        const Eigen::Quaternionf state_orientation = m_filter->state_vector.get_quaternion();
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
    m_filter = filter;

    return true;
}

void KalmanPoseFilterDS4::update(const float delta_time, const PoseFilterPacket &packet)
{
    if (m_filter->bIsValid)
    {
        // Predict state for current time-step using the filters
        m_filter->system_model.set_time_step(delta_time);
        m_filter->state_vector = m_filter->ukf.predict(m_filter->system_model);

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
			// Adjust the amount we trust the optical measurements based on the quality parameters
            measurement_model.update_measurement_covariance(
				m_constants, 
				packet.optical_position_quality, 
				packet.optical_orientation_quality);

            // If available, use the optical orientation measurement
            if (packet.optical_orientation_quality > 0.f)
            {
                measurement.set_optical_quaternion(packet.optical_orientation);
            }

            // If available, use the optical position
            if (packet.optical_position_quality > 0.f)
            {
                // State internally stores position in meters
                measurement.set_optical_position(packet.optical_position * k_centimeters_to_meters);
            }
        }

        // Update UKF
        m_filter->state_vector = m_filter->ukf.update(measurement_model, measurement);
    }
    else if (packet.optical_position_quality > 0.f)
    {
        m_filter->state_vector.setZero();
        m_filter->state_vector.set_position(packet.optical_position * k_centimeters_to_meters);

        if (packet.optical_position_quality > 0.f)
        {
            m_filter->state_vector.set_quaternion(packet.optical_orientation);
        }
        else
        {
            m_filter->state_vector.set_quaternion(Eigen::Quaternionf::Identity());
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
    m_filter = filter;

    return true;
}

void PSMovePoseKalmanFilter::update(const float delta_time, const PoseFilterPacket &packet)
{
    if (m_filter->bIsValid)
    {
        // Predict state for current time-step using the filters
        m_filter->state_vector = m_filter->ukf.predict(m_filter->system_model);

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
			// Adjust the amount we trust the optical measurements based on the quality parameters
			measurement_model.update_measurement_covariance(m_constants, packet.optical_position_quality);

			// Assign the latest optical measurement from the packet
            measurement.set_optical_position(packet.optical_position);
        }

        // Update UKF
        m_filter->state_vector = m_filter->ukf.update(measurement_model, measurement);
    }
    else if (packet.optical_position_quality > 0.f)
    {
        m_filter->state_vector.setZero();
        m_filter->state_vector.set_position(packet.optical_position * k_centimeters_to_meters);
        m_filter->state_vector.set_quaternion(Eigen::Quaternionf::Identity());

        m_filter->bIsValid= true;
    }
}

//-- Private functions --
// From: Q_discrete_white_noise in https://github.com/rlabbe/filterpy/blob/master/filterpy/common/discretization.py
template <class StateType>
void Q_discrete_3rd_order_white_noise(
    const float dT,
    const float var,
    const int state_index,
    Kalman::Covariance<StateType> &Q)
{
    const float dT_squared = dT*dT;
    const float q4 = var * dT_squared*dT_squared;
    const float q3 = var * dT_squared*dT;
    const float q2 = var * dT_squared;
    const float q1 = var * dT;
    const float q0 = var;

    // Q = [.5dt^2, dt, 1]*[.5dt^2, dt, 1]^T * variance
    const int &i= state_index;
    Q(i+0,i+0) = 0.25f*q4; Q(i+0,i+1) = 0.5f*q3; Q(i+0,i+2) = 0.5f*q2;
    Q(i+1,i+0) =  0.5f*q3; Q(i+1,i+1) =      q2; Q(i+1,i+2) =      q1;
    Q(i+2,i+0) =  0.5f*q2; Q(i+2,i+1) =      q1; Q(i+2,i+2) =      q0;
}

template <class StateType>
void Q_discrete_2nd_order_white_noise(const float dT, const float var, const int state_index, Kalman::Covariance<StateType> &Q)
{
    const float dT_squared = dT*dT;
    const float q4 = var * dT_squared*dT_squared;
    const float q3 = var * dT_squared*dT;
    const float q2 = var * dT_squared;

    // Q = [.5dt^2, dt]*[.5dt^2, dt]^T * variance
    const int &i= state_index;
    Q(i+0,i+0) = 0.25f*q4; Q(i+0,i+1) = 0.5f*q3;
    Q(i+1,i+0) =  0.5f*q3; Q(i+1,i+1) =      q2;
}
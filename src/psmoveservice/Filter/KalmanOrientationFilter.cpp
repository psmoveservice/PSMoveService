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
    EULER_ANGLE_BANK, // radians
    ANGULAR_VELOCITY_BANK, // meters / s
    EULER_ANGLE_HEADING,
	ANGULAR_VELOCITY_HEADING,
    EULER_ANGLE_ATTITUDE,
    ANGULAR_VELOCITY_ATTITUDE,

    STATE_PARAMETER_COUNT
};

enum PSMoveMeasurementEnum {
	PSMOVE_ACCELEROMETER_X, // gravity units
	PSMOVE_ACCELEROMETER_Y,
	PSMOVE_ACCELEROMETER_Z,
	PSMOVE_GYROSCOPE_X, // radians/s
	PSMOVE_GYROSCOPE_Y,
	PSMOVE_GYROSCOPE_Z,
	PSMOVE_MAGNETOMETER_X, // unit vector
	PSMOVE_MAGNETOMETER_Y,
	PSMOVE_MAGNETOMETER_Z,

	PSMOVE_MEASUREMENT_PARAMETER_COUNT
};

enum DS4MeasurementEnum {
	DS4_ACCELEROMETER_X, // gravity units
	DS4_ACCELEROMETER_Y,
	DS4_ACCELEROMETER_Z,
	DS4_GYROSCOPE_X, // radians/s
	DS4_GYROSCOPE_Y,
	DS4_GYROSCOPE_Z,
	DS4_OPTICAL_EULER_ANGLE_HEADING, // radians

	DS4_MEASUREMENT_PARAMETER_COUNT
};

// Arbitrary tuning scale applied to the measurement noise
#define R_SCALE 10.0

// Arbitrary tuning scale applied to the process noise
#define Q_SCALE 10.0

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
void process_3rd_order_noise(const double dT, const double var, const int state_index, Kalman::Covariance<StateType> &Q);

template <class StateType>
void process_2nd_order_noise(const double dT, const double var, const int state_index, Kalman::Covariance<StateType> &Q);

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
        return Eigen::Vector3d((*this)[ANGULAR_VELOCITY_BANK], (*this)[ANGULAR_VELOCITY_HEADING], (*this)[ANGULAR_VELOCITY_ATTITUDE]);
    }

    // Mutators
    void set_euler_angles(const Eigen::EulerAnglesd &e) {
        (*this)[EULER_ANGLE_BANK] = e.get_x_radians(); (*this)[EULER_ANGLE_HEADING] = e.get_y_radians(); (*this)[EULER_ANGLE_ATTITUDE] = e.get_z_radians();
    }
	void set_quaterniond(const Eigen::Quaterniond &d) {
		set_euler_angles(eigen_quaterniond_to_euler_angles(d));
	}
    void set_angular_velocity(const Eigen::Vector3d &v) {
        (*this)[ANGULAR_VELOCITY_BANK] = v.x(); (*this)[ANGULAR_VELOCITY_HEADING] = v.y(); (*this)[ANGULAR_VELOCITY_ATTITUDE] = v.z();
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
		const double mean_position_dT = constants.mean_update_time_delta;
		const double mean_orientation_dT = constants.mean_update_time_delta;

		// Start off using the maximum variance values
		const float orientation_variance = (constants.min_orientation_variance + constants.max_orientation_variance) * 0.5f;

		// Initialize the process covariance matrix Q
		Kalman::Covariance<OrientationStateVectord> Q = Kalman::Covariance<OrientationStateVectord>::Zero();
		process_2nd_order_noise<OrientationStateVectord>(mean_position_dT, orientation_variance, EULER_ANGLE_BANK, Q);
		process_2nd_order_noise<OrientationStateVectord>(mean_position_dT, orientation_variance, EULER_ANGLE_HEADING, Q);
		process_2nd_order_noise<OrientationStateVectord>(mean_position_dT, orientation_variance, EULER_ANGLE_ATTITUDE, Q);
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
	OrientationStateVectord f(const OrientationStateVectord& old_state, const Kalman::Vector<double, 0>& control) const
	{
		//! Predicted state vector after transition
		OrientationStateVectord new_state;

		// Extract parameters from the old state
		const Eigen::Vector3d old_euler_angles = Eigen::Vector3d(old_state.get_euler_angles());
		const Eigen::Vector3d old_angular_velocity = old_state.get_angular_velocity();

		// Compute the position state update
		const Eigen::Vector3d new_euler_angles =
			old_euler_angles
			+ old_angular_velocity*m_time_step;
		const Eigen::Vector3d new_angular_velocity = old_angular_velocity;

		// Save results to the new state
		new_state.set_euler_angles(Eigen::EulerAnglesd(new_euler_angles));
		new_state.set_angular_velocity(new_angular_velocity);

		return new_state;
	}

protected:
	double m_time_step;
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
		return Eigen::Vector3d((*this)[PSMOVE_GYROSCOPE_X], (*this)[PSMOVE_GYROSCOPE_Y], (*this)[PSMOVE_GYROSCOPE_Z]);
	}
	Eigen::Vector3d get_magnetometer() const {
		return Eigen::Vector3d((*this)[PSMOVE_MAGNETOMETER_X], (*this)[PSMOVE_MAGNETOMETER_Y], (*this)[PSMOVE_MAGNETOMETER_Z]);
	}

	// Mutators
	void set_accelerometer(const Eigen::Vector3d &a) {
		(*this)[PSMOVE_ACCELEROMETER_X] = a.x(); (*this)[PSMOVE_ACCELEROMETER_Y] = a.y(); (*this)[PSMOVE_ACCELEROMETER_Z] = a.z();
	}
	void set_gyroscope(const Eigen::Vector3d &g) {
		(*this)[PSMOVE_GYROSCOPE_X] = g.x(); (*this)[PSMOVE_GYROSCOPE_Y] = g.y(); (*this)[PSMOVE_GYROSCOPE_Z] = g.z();
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
		return Eigen::Vector3d((*this)[DS4_GYROSCOPE_X], (*this)[DS4_GYROSCOPE_Y], (*this)[DS4_GYROSCOPE_Z]);
	}
	Eigen::Vector3d get_optical_euler_heading_angle() const {
		return (*this)[DS4_OPTICAL_EULER_ANGLE_HEADING];
	}

	// Mutators
	void set_accelerometer(const Eigen::Vector3d &a) {
		(*this)[DS4_ACCELEROMETER_X] = a.x(); (*this)[DS4_ACCELEROMETER_Y] = a.y(); (*this)[DS4_ACCELEROMETER_Z] = a.z();
	}
	void set_gyroscope(const Eigen::Vector3d &g) {
		(*this)[DS4_GYROSCOPE_X] = g.x(); (*this)[DS4_GYROSCOPE_Y] = g.y(); (*this)[DS4_GYROSCOPE_Z] = g.z();
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
		R(PSMOVE_ACCELEROMETER_X, PSMOVE_ACCELEROMETER_X) = sqrt(R_SCALE*constants.accelerometer_variance.x());
		R(PSMOVE_ACCELEROMETER_Y, PSMOVE_ACCELEROMETER_Y) = sqrt(R_SCALE*constants.accelerometer_variance.y());
		R(PSMOVE_ACCELEROMETER_Z, PSMOVE_ACCELEROMETER_Z) = sqrt(R_SCALE*constants.accelerometer_variance.z());
		R(PSMOVE_GYROSCOPE_X, PSMOVE_GYROSCOPE_X) = sqrt(R_SCALE*constants.gyro_variance.x());
		R(PSMOVE_GYROSCOPE_Y, PSMOVE_GYROSCOPE_Y) = sqrt(R_SCALE*constants.gyro_variance.y());
		R(PSMOVE_GYROSCOPE_Z, PSMOVE_GYROSCOPE_Z) = sqrt(R_SCALE*constants.gyro_variance.z());
		R(PSMOVE_MAGNETOMETER_X, PSMOVE_MAGNETOMETER_X) = sqrt(R_SCALE*constants.magnetometer_variance.x());
		R(PSMOVE_MAGNETOMETER_Y, PSMOVE_MAGNETOMETER_Y) = sqrt(R_SCALE*constants.magnetometer_variance.y());
		R(PSMOVE_MAGNETOMETER_Z, PSMOVE_MAGNETOMETER_Z) = sqrt(R_SCALE*constants.magnetometer_variance.z());

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
		update_measurement_statistics(constants, 0.f);

		identity_gravity_direction = constants.gravity_calibration_direction.cast<double>();
	}

	void update_measurement_statistics(
		const OrientationFilterConstants &constants,
		const float orientation_quality)
	{
		// Update the measurement covariance R
		Kalman::Covariance<DS4_OrientationMeasurementVectord> R =
			Kalman::Covariance<DS4_OrientationMeasurementVectord>::Zero();

		R(DS4_ACCELEROMETER_X, DS4_ACCELEROMETER_X) = sqrt(R_SCALE*constants.accelerometer_variance.x());
		R(DS4_ACCELEROMETER_Y, DS4_ACCELEROMETER_Y) = sqrt(R_SCALE*constants.accelerometer_variance.y());
		R(DS4_ACCELEROMETER_Z, DS4_ACCELEROMETER_Z) = sqrt(R_SCALE*constants.accelerometer_variance.z());
		R(DS4_GYROSCOPE_X, DS4_GYROSCOPE_X) = sqrt(R_SCALE*constants.gyro_variance.x());
		R(DS4_GYROSCOPE_Y, DS4_GYROSCOPE_Y) = sqrt(R_SCALE*constants.gyro_variance.y());
		R(DS4_GYROSCOPE_Z, DS4_GYROSCOPE_Z) = sqrt(R_SCALE*constants.gyro_variance.z());
		R(DS4_OPTICAL_EULER_ANGLE_HEADING, DS4_OPTICAL_EULER_ANGLE_HEADING) = 
			sqrt(R_SCALE*lerp_clampf(
				constants.max_orientation_variance,
				constants.min_orientation_variance,
				orientation_quality));
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

    /// All state parameters of the controller
    OrientationStateVectord state_vector;

    /// Used to model how the physics of the controller evolves
    OrientationSystemModel system_model;

    /// Unscented Kalman Filter instance
	Kalman::SquareRootUnscentedKalmanFilter<OrientationStateVectord> ukf;

	KalmanOrientationFilterImpl()
		: ukf(k_ukf_alpha, k_ukf_beta, k_ukf_kappa)
    {
    }

    virtual void init(const OrientationFilterConstants &constants)
    {
		bIsValid = false;
		bSeenOrientationMeasurement = true;

		reset_orientation = Eigen::Quaternionf::Identity();
		state_vector = OrientationStateVectord::Zero();

        system_model.init(constants);
        ukf.init(state_vector);
    }

	virtual void init(
		const OrientationFilterConstants &constants,
		const Eigen::Quaternionf &orientation)
	{
		bIsValid = true;
		bSeenOrientationMeasurement = false;

		reset_orientation = Eigen::Quaternionf::Identity();
		state_vector = OrientationStateVectord::Zero();
		state_vector.set_quaterniond(orientation.cast<double>());

		system_model.init(constants);
		ukf.init(state_vector);
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
		const Eigen::Quaternionf state_orientation = m_filter->state_vector.get_quaterniond().cast<float>();
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
	Eigen::Vector3d ang_vel = m_filter->state_vector.get_angular_velocity();

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

		// Predict state for current time-step using the filters
		filter->system_model.set_time_step(delta_time);
		filter->state_vector = m_filter->ukf.predict(filter->system_model);

		// Get the measurement model for the DS4 from the derived filter impl
		DS4_OrientationMeasurementModel &measurement_model = filter->measurement_model;

		// Project the current state onto a predicted measurement as a default
		// in case no observation is available
		DS4_OrientationMeasurementVectord measurement = measurement_model.h(filter->state_vector);

		// Accelerometer and gyroscope measurements are always available
		measurement.set_accelerometer(packet.imu_accelerometer.cast<double>());
		measurement.set_gyroscope(packet.imu_gyroscope.cast<double>());

		if (packet.optical_orientation_quality > 0.f)
		{
			// Adjust the amount we trust the optical measurements based on the quality parameters
			measurement_model.update_measurement_statistics(
				m_constants,
				packet.optical_orientation_quality);

			// If available, use the optical orientation measurement
			if (packet.optical_orientation_quality > 0.f)
			{
				const Eigen::EulerAnglesd optical_euler_angles =
					eigen_quaterniond_to_euler_angles(packet.optical_orientation.cast<double>());
				const double optical_heading = optical_euler_angles.get_heading_radians();

				measurement.set_optical_euler_heading_angle(optical_heading);

				// If this is the first time we have seen the orientation, snap the orientation state
				if (!m_filter->bSeenOrientationMeasurement)
				{
					filter->state_vector.set_quaterniond(packet.optical_orientation.cast<double>());
					m_filter->bSeenOrientationMeasurement = true;
				}
			}
		}

		// Update UKF
		m_filter->state_vector = filter->ukf.update(measurement_model, measurement);
	}
	else
	{
		m_filter->state_vector.setZero();
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
		filter->state_vector = m_filter->ukf.predict(filter->system_model);

        // Get the measurement model for the PSMove from the derived filter impl
		PSMove_OrientationMeasurementModel &measurement_model = filter->measurement_model;

		// Accelerometer, gyroscope, magnetometer measurements are always available
		PSMove_OrientationMeasurementVectord measurement = PSMove_OrientationMeasurementVectord::Zero();
		measurement.set_accelerometer(packet.imu_accelerometer.cast<double>());
		measurement.set_gyroscope(packet.imu_gyroscope.cast<double>());
		measurement.set_magnetometer(packet.imu_magnetometer.cast<double>());
		m_filter->bSeenOrientationMeasurement = true;

        // Update UKF
        m_filter->state_vector = filter->ukf.update(measurement_model, measurement);
    }
    else
    {
        m_filter->state_vector.setZero();
        m_filter->bIsValid= true;
    }
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
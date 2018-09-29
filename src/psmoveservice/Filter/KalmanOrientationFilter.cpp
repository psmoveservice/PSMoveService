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
enum OrientationStateEnum
{
	ERROR_QUATERNION_W,
    ERROR_QUATERNION_X,
	ERROR_QUATERNION_Y,
	ERROR_QUATERNION_Z,

    STATE_PARAMETER_COUNT
};

enum OrientationControlEnum
{
	CONTROL_GYROSCOPE_PITCH, // radians/s
	CONTROL_GYROSCOPE_YAW,
	CONTROL_GYROSCOPE_ROLL,

    CONTROL_PARAMETER_COUNT
};

enum OrientationMeasurementEnum {
	ACCELEROMETER_X, // gravity units
	ACCELEROMETER_Y,
	ACCELEROMETER_Z,

	G_MEASUREMENT_PARAMETER_COUNT,

	MAGNETOMETER_X = G_MEASUREMENT_PARAMETER_COUNT, // unit vector
	MAGNETOMETER_Y,
	MAGNETOMETER_Z,

	MG_MEASUREMENT_PARAMETER_COUNT
};

enum OpticalMeasurementEnum
{
	OPTICAL_QUATERNION_W,
    OPTICAL_QUATERNION_X,
	OPTICAL_QUATERNION_Y,
	OPTICAL_QUATERNION_Z,

    OPTICAL_MEASUREMENT_PARAMETER_COUNT
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
#define k_ukf_beta 2.0
#define k_ukf_kappa -1.0 // 3 - STATE_PARAMETER_COUNT
#define k_ukf_alpha 0.01

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

    // Mutators
	void set_error_quaterniond(const Eigen::Quaterniond &q) {
		(*this)[ERROR_QUATERNION_W] = q.w();
		(*this)[ERROR_QUATERNION_X] = q.x();
		(*this)[ERROR_QUATERNION_Y] = q.y();
		(*this)[ERROR_QUATERNION_Z] = q.z();
	}
};
typedef OrientationStateVector<double> OrientationStateVectord;

template<typename T>
class OrientationControlVector : public Kalman::Vector<T, CONTROL_PARAMETER_COUNT>
{
public:
	KALMAN_VECTOR(OrientationControlVector, T, CONTROL_PARAMETER_COUNT)

    // Accessors
	Eigen::Vector3d get_angular_rates() const {
		return Eigen::Vector3d((*this)[CONTROL_GYROSCOPE_PITCH], (*this)[CONTROL_GYROSCOPE_YAW], (*this)[CONTROL_GYROSCOPE_ROLL]);
	}

    // Mutators
	void set_angular_rates(const Eigen::Vector3d &v) {
		(*this)[CONTROL_GYROSCOPE_PITCH] = v.x();
		(*this)[CONTROL_GYROSCOPE_YAW] = v.y();
		(*this)[CONTROL_GYROSCOPE_ROLL] = v.z();
	}
};
typedef OrientationControlVector<double> OrientationControlVectord;

/**
* @brief System model for a controller
*
* This is the system model defining how a controller advances from one
* time-step to the next, i.e. how the system state evolves over time.
*/
class OrientationSystemModel : public Kalman::SystemModel<OrientationStateVectord, OrientationControlVectord, Kalman::SquareRootBase>
{
public:
	inline void set_time_step(const double dt) { m_time_step = dt; }

	void init(const OrientationFilterConstants &constants)
	{
		m_last_tracking_projection_area = -1.f;
		m_gyro_bias= constants.gyro_drift.cast<double>();
		update_process_noise(constants, 0.f);
	}

	void update_process_noise(const OrientationFilterConstants &constants, float tracking_projection_area)
	{
		// Only update the covariance when there is more than a 10px change in position quality
		if (m_last_tracking_projection_area < 0.f || 
			!is_nearly_equal(tracking_projection_area, m_last_tracking_projection_area, 10.f))
		{
			// Start off using the maximum variance values
			static float q_scale = Q_SCALE;
			float orientation_variance = q_scale * constants.orientation_variance_curve.evaluate(tracking_projection_area);

			// Initialize the process covariance matrix Q
			const float mean_orientation_dT = constants.mean_update_time_delta;
			const float discrete_variance= orientation_variance*mean_orientation_dT*mean_orientation_dT;
			Kalman::Covariance<OrientationStateVectord> Q = Kalman::Covariance<OrientationStateVectord>::Zero();
			Q(0,0) = discrete_variance;
			Q(1,1) = discrete_variance;
			Q(2,2) = discrete_variance;
			Q(3,3) = discrete_variance;
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
	OrientationStateVectord f(const OrientationStateVectord& old_state, const OrientationControlVectord& control) const
	{
		// Extract quaternion from the old state
		const Eigen::Quaterniond error_q_old = old_state.get_error_quaterniond();

		// Compute the true angular rate from the control vector
		const Eigen::Vector3d omega= control - m_gyro_bias;

		// Compute the quaternion derivative of the current state
		// q_new= q + q_dot*dT
		const Eigen::Quaterniond q_dot = eigen_angular_velocity_to_quaterniond_derivative(error_q_old, omega);
		const Eigen::Quaterniond error_q_step = Eigen::Quaterniond(q_dot.coeffs() * m_time_step);
		const Eigen::Quaterniond error_q_new = Eigen::Quaterniond(error_q_old.coeffs() + error_q_step.coeffs());

		// Save results to the new state
		OrientationStateVectord new_state;
		new_state.set_error_quaterniond(error_q_new.normalized());

		return new_state;
	}

protected:
	double m_time_step;
	float m_last_tracking_projection_area;
	Eigen::Vector3d m_gyro_bias;
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
class GravMeasurementVector : public Kalman::Vector<T, G_MEASUREMENT_PARAMETER_COUNT>
{
public:
	KALMAN_VECTOR(GravMeasurementVector, T, G_MEASUREMENT_PARAMETER_COUNT)

	// Accessors
	Eigen::Vector3d get_accelerometer() const {
		return Eigen::Vector3d((*this)[ACCELEROMETER_X], (*this)[ACCELEROMETER_Y], (*this)[ACCELEROMETER_Z]);
	}

	// Mutators
	void set_accelerometer(const Eigen::Vector3d &a) {
		(*this)[ACCELEROMETER_X] = a.x(); (*this)[ACCELEROMETER_Y] = a.y(); (*this)[ACCELEROMETER_Z] = a.z();
	}
};
typedef GravMeasurementVector<double> GravMeasurementVectord;

class GravMeasurementModel : 
	public Kalman::MeasurementModel<OrientationStateVectord, GravMeasurementVectord, Kalman::SquareRootBase>
{
public:
	void init(const OrientationFilterConstants &constants)
	{
		// Update the measurement covariance R
		Kalman::Covariance<GravMeasurementVectord> R =
			Kalman::Covariance<GravMeasurementVectord>::Zero();

		// Only diagonals used so no need to compute Cholesky
		static float r_accelerometer_scale = R_SCALE;
		R(ACCELEROMETER_X, ACCELEROMETER_X) = r_accelerometer_scale*constants.accelerometer_variance.x();
		R(ACCELEROMETER_Y, ACCELEROMETER_Y) = r_accelerometer_scale*constants.accelerometer_variance.y();
		R(ACCELEROMETER_Z, ACCELEROMETER_Z) = r_accelerometer_scale*constants.accelerometer_variance.z();
		setCovariance(R);

		identity_gravity_direction = constants.gravity_calibration_direction.cast<double>();
		m_last_world_orientation = Eigen::Quaterniond::Identity();
	}

	void update_world_orientation(const Eigen::Quaterniond &orientation)
	{
		m_last_world_orientation = orientation;
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
	GravMeasurementVectord h(const OrientationStateVectord& x) const
	{
		GravMeasurementVectord predicted_measurement;

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

		// Save the predictions into the measurement vector
		predicted_measurement.set_accelerometer(accel_local);

		return predicted_measurement;
	}

public:
	Eigen::Vector3d identity_gravity_direction;
	Eigen::Quaterniond m_last_world_orientation;
};

template<typename T>
class MagGravMeasurementVector : public Kalman::Vector<T, MG_MEASUREMENT_PARAMETER_COUNT>
{
public:
	KALMAN_VECTOR(MagGravMeasurementVector, T, MG_MEASUREMENT_PARAMETER_COUNT)

	// Accessors
	Eigen::Vector3d get_accelerometer() const {
		return Eigen::Vector3d((*this)[ACCELEROMETER_X], (*this)[ACCELEROMETER_Y], (*this)[ACCELEROMETER_Z]);
	}
	Eigen::Vector3d get_magnetometer() const {
		return Eigen::Vector3d((*this)[MAGNETOMETER_X], (*this)[MAGNETOMETER_Y], (*this)[MAGNETOMETER_Z]);
	}

	// Mutators
	void set_accelerometer(const Eigen::Vector3d &a) {
		(*this)[ACCELEROMETER_X] = a.x(); (*this)[ACCELEROMETER_Y] = a.y(); (*this)[ACCELEROMETER_Z] = a.z();
	}
	void set_magnetometer(const Eigen::Vector3d &m) {
		(*this)[MAGNETOMETER_X] = m.x(); (*this)[MAGNETOMETER_Y] = m.y(); (*this)[MAGNETOMETER_Z] = m.z();
	}
};
typedef MagGravMeasurementVector<double> MagGravMeasurementVectord;

class MagGravMeasurementModel : 
	public Kalman::MeasurementModel<OrientationStateVectord, MagGravMeasurementVectord, Kalman::SquareRootBase>
{
public:
	void init(const OrientationFilterConstants &constants)
	{
		// Update the measurement covariance R
		Kalman::Covariance<MagGravMeasurementVectord> R =
			Kalman::Covariance<MagGravMeasurementVectord>::Zero();

		// Only diagonals used so no need to compute Cholesky
		static float r_accelerometer_scale = R_SCALE;
		static float r_magnetometer_scale = R_SCALE;
		R(ACCELEROMETER_X, ACCELEROMETER_X) = r_accelerometer_scale*constants.accelerometer_variance.x();
		R(ACCELEROMETER_Y, ACCELEROMETER_Y) = r_accelerometer_scale*constants.accelerometer_variance.y();
		R(ACCELEROMETER_Z, ACCELEROMETER_Z) = r_accelerometer_scale*constants.accelerometer_variance.z();
		R(MAGNETOMETER_X, MAGNETOMETER_X) = r_magnetometer_scale*constants.magnetometer_variance.x();
		R(MAGNETOMETER_Y, MAGNETOMETER_Y) = r_magnetometer_scale*constants.magnetometer_variance.y();
		R(MAGNETOMETER_Z, MAGNETOMETER_Z) = r_magnetometer_scale*constants.magnetometer_variance.z();
		setCovariance(R);

		identity_gravity_direction = constants.gravity_calibration_direction.cast<double>();
		identity_magnetometer_direction = constants.magnetometer_calibration_direction.cast<double>();
		m_last_world_orientation = Eigen::Quaterniond::Identity();
	}

	void update_world_orientation(const Eigen::Quaterniond &orientation)
	{
		m_last_world_orientation = orientation;
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
	MagGravMeasurementVectord h(const OrientationStateVectord& x) const
	{
		MagGravMeasurementVectord predicted_measurement;

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

		// Use the orientation from the state to predict
		// what the magnetometer reading should be (in the space of the controller)
		const Eigen::Vector3d &mag_world = identity_magnetometer_direction;
		const Eigen::Vector3d mag_local = eigen_vector3d_clockwise_rotate(world_to_local_orientation, mag_world);

		// Save the predictions into the measurement vector
		predicted_measurement.set_accelerometer(accel_local);
		predicted_measurement.set_magnetometer(mag_local);

		return predicted_measurement;
	}

public:
	Eigen::Vector3d identity_gravity_direction;
	Eigen::Vector3d identity_magnetometer_direction;
	Eigen::Quaterniond m_last_world_orientation;
	Eigen::Vector3d m_last_world_linear_acceleration_m_per_sec_sqr;
};


template<typename T>
class OrientationMeasurementVector : public Kalman::Vector<T, OPTICAL_MEASUREMENT_PARAMETER_COUNT>
{
public:
	KALMAN_VECTOR(OrientationMeasurementVector, T, OPTICAL_MEASUREMENT_PARAMETER_COUNT)

    // Accessors
	Eigen::Quaterniond get_optical_quaterniond() const {
		return Eigen::Quaterniond(
			(*this)[OPTICAL_QUATERNION_W], 
			(*this)[OPTICAL_QUATERNION_X],
			(*this)[OPTICAL_QUATERNION_Y], 
			(*this)[OPTICAL_QUATERNION_Z]);
	}

    // Mutators
	void set_optical_quaterniond(const Eigen::Quaterniond &q) {
		(*this)[OPTICAL_QUATERNION_W] = q.w();
		(*this)[OPTICAL_QUATERNION_X] = q.x();
		(*this)[OPTICAL_QUATERNION_Y] = q.y();
		(*this)[OPTICAL_QUATERNION_Z] = q.z();
	}
};
typedef OrientationMeasurementVector<double> OrientationMeasurementVectord;

class OpticalOrientationMeasurementModel
	: public Kalman::MeasurementModel<OrientationStateVectord, OrientationMeasurementVectord, Kalman::SquareRootBase>
{
public:
	void init(const OrientationFilterConstants &constants)
	{
		m_last_tracking_projection_area = -1.f;
		m_last_world_orientation = Eigen::Quaterniond::Identity();
		update_measurement_statistics(constants, 0.f);
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
			Kalman::Covariance<OrientationMeasurementVectord> R =
				Kalman::Covariance<OrientationMeasurementVectord>::Zero();
			const float orientation_variance = constants.orientation_variance_curve.evaluate(tracking_projection_area);

			static float r_scale = R_SCALE;
			R(OPTICAL_QUATERNION_W, OPTICAL_QUATERNION_W) = r_scale*orientation_variance;
			R(OPTICAL_QUATERNION_X, OPTICAL_QUATERNION_X) = r_scale*orientation_variance;
			R(OPTICAL_QUATERNION_Y, OPTICAL_QUATERNION_Y) = r_scale*orientation_variance;
			R(OPTICAL_QUATERNION_Z, OPTICAL_QUATERNION_Z) = r_scale*orientation_variance;
			setCovariance(R);

			// Keep track last tracking projection area we built the covariance matrix for
			m_last_tracking_projection_area = tracking_projection_area;
		}
	}

	void update_world_orientation(const Eigen::Quaterniond &orientation)
	{
		m_last_world_orientation = orientation;
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
	OrientationMeasurementVectord h(const OrientationStateVectord& x) const
	{
		OrientationMeasurementVectord predicted_measurement;

		// Use the orientation from the state for prediction
		const Eigen::Quaterniond error_orientation = x.get_error_quaterniond();
		const Eigen::Quaterniond world_to_local_orientation = eigen_quaternion_concatenate(m_last_world_orientation, error_orientation).normalized();

		// Save the predictions into the measurement vector
		predicted_measurement.set_optical_quaterniond(world_to_local_orientation);

		return predicted_measurement;
	}

public:
	float m_last_tracking_projection_area;
	Eigen::Quaterniond m_last_world_orientation;
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

    double time;

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
        , time(0.0)
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

class PSVRKalmanPoseFilterImpl : public KalmanOrientationFilterImpl
{
public:
	GravMeasurementModel imu_measurement_model;
	OpticalOrientationMeasurementModel optical_measurement_model;

	void init(const OrientationFilterConstants &constants) override
	{
		KalmanOrientationFilterImpl::init(constants);
		imu_measurement_model.init(constants);
		optical_measurement_model.init(constants);
	}

	void init(
		const OrientationFilterConstants &constants,
		const Eigen::Quaternionf &orientation) override
	{
		KalmanOrientationFilterImpl::init(constants, orientation);
		imu_measurement_model.init(constants);
		optical_measurement_model.init(constants);
	}
};

class DS4KalmanOrientationFilterImpl : public KalmanOrientationFilterImpl
{
public:
	GravMeasurementModel imu_measurement_model;
	OpticalOrientationMeasurementModel optical_measurement_model;

	void init(const OrientationFilterConstants &constants) override
	{
		KalmanOrientationFilterImpl::init(constants);
		imu_measurement_model.init(constants);
		optical_measurement_model.init(constants);
	}

	void init(
		const OrientationFilterConstants &constants,
		const Eigen::Quaternionf &orientation) override
	{
		KalmanOrientationFilterImpl::init(constants, orientation);
		imu_measurement_model.init(constants);
		optical_measurement_model.init(constants);
	}
};

class PSMoveKalmanPoseFilterImpl : public KalmanOrientationFilterImpl
{
public:
	MagGravMeasurementModel imu_measurement_model;
	OpticalOrientationMeasurementModel optical_measurement_model;

	void init(const OrientationFilterConstants &constants) override
	{
		KalmanOrientationFilterImpl::init(constants);
		imu_measurement_model.init(constants);
		optical_measurement_model.init(constants);
	}

	void init(
		const OrientationFilterConstants &constants,
		const Eigen::Quaternionf &orientation) override
	{
		KalmanOrientationFilterImpl::init(constants, orientation);
		imu_measurement_model.init(constants);
		optical_measurement_model.init(constants);
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

double KalmanOrientationFilter::getTimeInSeconds() const
{
    return m_filter->time;
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
	Eigen::Vector3d ang_vel = Eigen::Vector3d::Zero(); //m_filter->ukf.getState().get_angular_velocity_rad_per_sec();

	return ang_vel.cast<float>();
}

Eigen::Vector3f KalmanOrientationFilter::getAngularAccelerationRadPerSecSqr() const
{
	return Eigen::Vector3f::Zero();
}

//-- KalmanOrientationFilterPSVR --
bool KalmanOrientationFilterPSVR::init(const OrientationFilterConstants &constants)
{
	KalmanOrientationFilter::init(constants);

	PSVRKalmanPoseFilterImpl *filter = new PSVRKalmanPoseFilterImpl();
	filter->init(constants);
	m_filter = filter;

	return true;
}

bool KalmanOrientationFilterPSVR::init(
	const OrientationFilterConstants &constants,
	const Eigen::Quaternionf &orientation)
{
	KalmanOrientationFilter::init(constants);

	PSVRKalmanPoseFilterImpl *filter = new PSVRKalmanPoseFilterImpl();
	filter->init(constants, orientation);
	m_filter = filter;

	return true;
}

void KalmanOrientationFilterPSVR::update(const float delta_time, const PoseFilterPacket &packet)
{
    if (m_filter->bIsValid)
    {
		PSVRKalmanPoseFilterImpl *filter = static_cast<PSVRKalmanPoseFilterImpl *>(m_filter);
		OpticalOrientationMeasurementModel &optical_measurement_model = filter->optical_measurement_model;
		GravMeasurementModel &imu_measurement_model= filter->imu_measurement_model;

        // Predict state for current time-step using the filters
		filter->system_model.set_time_step(delta_time);

		if (packet.has_imu_measurements())
		{
			OrientationControlVectord control;
			control.set_angular_rates(packet.imu_gyroscope_rad_per_sec.cast<double>());

			filter->ukf.predict(filter->system_model, control);
		}
		else
		{
			filter->ukf.predict(filter->system_model);
		}

		// Apply any optical measurement to the filter
		if (packet.has_optical_measurement())
		{
			assert(packet.tracking_projection_area_px_sqr > 0.f);
			Eigen::Quaterniond optical_orientation= packet.optical_orientation.cast<double>();

			// If this is the first time we have seen an orientation measurement, 
			// snap the orientation state to a best fit alignment of the sensor measurements.
			if (!m_filter->bSeenOrientationMeasurement)
			{
				optical_measurement_model.update_world_orientation(optical_orientation);
				imu_measurement_model.update_world_orientation(optical_orientation);
				filter->set_world_quaternion(optical_orientation);
				filter->bSeenOrientationMeasurement = true;
			}

			OrientationMeasurementVectord measurement = OrientationMeasurementVectord::Zero();
			measurement.set_optical_quaterniond(optical_orientation);
			filter->ukf.update(optical_measurement_model, measurement);
		}

		// Apply any IMU measurement to the filter
		if (packet.has_imu_measurements())
		{
			assert(packet.has_accelerometer_measurement);

			GravMeasurementVectord measurement = GravMeasurementVectord::Zero();
			measurement.set_accelerometer(packet.imu_accelerometer_g_units.cast<double>());
			filter->ukf.update(imu_measurement_model, measurement);
		}

		// Apply the orientation error in the UKF state to the output quaternion.
		// Zero out the error in the UKF state vector.
		filter->apply_error_to_world_quaternion();

		// Update the measurement model with the latest estimate of the orientation (without error)
		// so that we can predict what the controller relative sensor measurements will be
		optical_measurement_model.update_world_orientation(filter->world_orientation);
		imu_measurement_model.update_world_orientation(filter->world_orientation);
    }
    else
    {
		m_filter->ukf.init(OrientationStateVectord::Identity());
        m_filter->bIsValid= true;
    }
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
		PSVRKalmanPoseFilterImpl *filter = static_cast<PSVRKalmanPoseFilterImpl *>(m_filter);
		OpticalOrientationMeasurementModel &optical_measurement_model = filter->optical_measurement_model;
		GravMeasurementModel &imu_measurement_model= filter->imu_measurement_model;

        // Predict state for current time-step using the filters
		filter->system_model.set_time_step(delta_time);

		if (packet.has_imu_measurements())
		{
			OrientationControlVectord control;
			control.set_angular_rates(packet.imu_gyroscope_rad_per_sec.cast<double>());

			filter->ukf.predict(filter->system_model, control);
		}
		else
		{
			filter->ukf.predict(filter->system_model);
		}

		// Apply any optical measurement to the filter
		if (packet.has_optical_measurement())
		{
			assert(packet.tracking_projection_area_px_sqr > 0.f);
			Eigen::Quaterniond optical_orientation= packet.optical_orientation.cast<double>();

			// If this is the first time we have seen an orientation measurement, 
			// snap the orientation state to a best fit alignment of the sensor measurements.
			if (!m_filter->bSeenOrientationMeasurement)
			{
				optical_measurement_model.update_world_orientation(optical_orientation);
				imu_measurement_model.update_world_orientation(optical_orientation);
				filter->set_world_quaternion(optical_orientation);
				filter->bSeenOrientationMeasurement = true;
			}

			OrientationMeasurementVectord measurement = OrientationMeasurementVectord::Zero();
			measurement.set_optical_quaterniond(optical_orientation);
			filter->ukf.update(optical_measurement_model, measurement);
		}

		// Apply any IMU measurement to the filter
		if (packet.has_imu_measurements())
		{
			assert(packet.has_accelerometer_measurement);

			GravMeasurementVectord measurement = GravMeasurementVectord::Zero();
			measurement.set_accelerometer(packet.imu_accelerometer_g_units.cast<double>());
			filter->ukf.update(imu_measurement_model, measurement);
		}

		// Apply the orientation error in the UKF state to the output quaternion.
		// Zero out the error in the UKF state vector.
		filter->apply_error_to_world_quaternion();

		// Update the measurement model with the latest estimate of the orientation (without error)
		// so that we can predict what the controller relative sensor measurements will be
		optical_measurement_model.update_world_orientation(filter->world_orientation);
		imu_measurement_model.update_world_orientation(filter->world_orientation);
    }
    else
    {
		m_filter->ukf.init(OrientationStateVectord::Identity());
        m_filter->bIsValid= true;
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
		OpticalOrientationMeasurementModel &optical_measurement_model = filter->optical_measurement_model;
		MagGravMeasurementModel &imu_measurement_model= filter->imu_measurement_model;

        // Predict state for current time-step using the filters
		filter->system_model.set_time_step(delta_time);

		if (packet.has_imu_measurements())
		{
			OrientationControlVectord control;
			control.set_angular_rates(packet.imu_gyroscope_rad_per_sec.cast<double>());

			filter->ukf.predict(filter->system_model, control);
		}
		else
		{
			filter->ukf.predict(filter->system_model);
		}

		// Apply any optical measurement to the filter
		if (packet.has_optical_measurement())
		{
			assert(packet.tracking_projection_area_px_sqr > 0.f);
			Eigen::Quaterniond optical_orientation= packet.optical_orientation.cast<double>();

			// If this is the first time we have seen an orientation measurement, 
			// snap the orientation state to a best fit alignment of the sensor measurements.
			if (!m_filter->bSeenOrientationMeasurement)
			{
				optical_measurement_model.update_world_orientation(optical_orientation);
				imu_measurement_model.update_world_orientation(optical_orientation);
				filter->set_world_quaternion(optical_orientation);
				filter->bSeenOrientationMeasurement = true;
			}

			OrientationMeasurementVectord measurement = OrientationMeasurementVectord::Zero();
			measurement.set_optical_quaterniond(optical_orientation);
			filter->ukf.update(optical_measurement_model, measurement);
		}

		// Apply any IMU measurement to the filter
		if (packet.has_imu_measurements())
		{
			assert(packet.has_accelerometer_measurement);

			MagGravMeasurementVectord measurement = MagGravMeasurementVectord::Zero();
			measurement.set_accelerometer(packet.imu_accelerometer_g_units.cast<double>());
			measurement.set_magnetometer(packet.imu_magnetometer_unit.cast<double>());
			filter->ukf.update(imu_measurement_model, measurement);
		}

		// Apply the orientation error in the UKF state to the output quaternion.
		// Zero out the error in the UKF state vector.
		filter->apply_error_to_world_quaternion();

		// Update the measurement model with the latest estimate of the orientation (without error)
		// so that we can predict what the controller relative sensor measurements will be
		optical_measurement_model.update_world_orientation(filter->world_orientation);
		imu_measurement_model.update_world_orientation(filter->world_orientation);
    }
    else
    {
		m_filter->ukf.init(OrientationStateVectord::Identity());
        m_filter->bIsValid= true;
    }
}
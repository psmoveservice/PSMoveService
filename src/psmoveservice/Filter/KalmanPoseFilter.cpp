//-- includes --
#include "KalmanPoseFilter.h"
#include "MathAlignment.h"

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

    STATE_PARAMETER_COUNT,
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

// Arbitrary tuning scale applied to the measurement noise
#define R_SCALE 10.0

// Arbitrary tuning scale applied to the process noise
#define Q_SCALE 10.0

// From: http://nbviewer.jupyter.org/github/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/10-Unscented-Kalman-Filter.ipynb#Reasonable-Choices-for-the-Parameters
// beta=2 is a good choice for Gaussian problems, 
// kappa=3-n where n is the size of x is a good choice for kappa, 
// 0<=alpha<=1 is an appropriate choice for alpha, 
// where a larger value for alpha spreads the sigma points further from the mean.
#define k_ukf_alpha 1.0
#define k_ukf_beta 2.0
#define k_ukf_kappa -1.0

//-- private methods ---
void process_3rd_order_noise(
	const double dT, const double var, const int state_index, 
	Eigen::Matrix<double, STATE_PARAMETER_COUNT, STATE_PARAMETER_COUNT> &Q);

void process_2nd_order_noise(
	const double dT, const double var, const int state_index, 
	Eigen::Matrix<double, STATE_PARAMETER_COUNT, STATE_PARAMETER_COUNT> &Q);

//-- private definitions --
class PoseStateVector : public Eigen::Matrix<double, STATE_PARAMETER_COUNT, 1>
{
public:
	PoseStateVector(void) : Eigen::Matrix<double, STATE_PARAMETER_COUNT, 1>()
	{ }

	template<typename OtherDerived>
	PoseStateVector(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Matrix<double, STATE_PARAMETER_COUNT, 1>(other)
	{ }

	template<typename OtherDerived>
	PoseStateVector& operator= (const Eigen::MatrixBase<OtherDerived>& other)
	{
		this->Base::operator=(other);
		return *this;
	}

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
	template <int RowsAtCompileTime>
	static Eigen::AngleAxisd extract_angle_axis(const Eigen::Matrix<double, RowsAtCompileTime, 1> &M) {
		Eigen::Vector3d axis = Eigen::Vector3d(M[ANGLE_AXIS_X], M[ANGLE_AXIS_Y], M[ANGLE_AXIS_Z]);
		const double angle = eigen_vector3d_normalize_with_default(axis, Eigen::Vector3d::Zero());
		return Eigen::AngleAxisd(angle, axis);
	}
    Eigen::AngleAxisd get_angle_axis() const {
        return extract_angle_axis<STATE_PARAMETER_COUNT>(*this);
    }
	template <int RowsAtCompileTime>
	static Eigen::Quaterniond extract_quaternion(const Eigen::Matrix<double, RowsAtCompileTime, 1> &M) {
		return Eigen::Quaterniond(extract_angle_axis<RowsAtCompileTime>(M));
	}
    Eigen::Quaterniond get_quaternion() const {
        return extract_quaternion<STATE_PARAMETER_COUNT>(*this);
    }
    Eigen::Vector3d get_angular_velocity() const {
        return Eigen::Vector3d((*this)[ANGULAR_VELOCITY_X], (*this)[ANGULAR_VELOCITY_Y], (*this)[ANGULAR_VELOCITY_Z]);
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
	template <int RowsAtCompileTime>
	static void apply_angle_axis(const Eigen::AngleAxisd &a, Eigen::Matrix<double, RowsAtCompileTime, 1> &M) {
		const double angle = a.angle();
		M[ANGLE_AXIS_X] = a.axis().x() * angle;
		M[ANGLE_AXIS_Y] = a.axis().y() * angle;
		M[ANGLE_AXIS_Z] = a.axis().z() * angle;
	}
    void set_angle_axis(const Eigen::AngleAxisd &a) {
		apply_angle_axis<STATE_PARAMETER_COUNT>(a, *this);
    }
	template <int RowsAtCompileTime>
	static void apply_quaternion(const Eigen::Quaterniond &q, Eigen::Matrix<double, RowsAtCompileTime, 1> &M) {
		const Eigen::AngleAxisd angle_axis(q);
		apply_angle_axis<RowsAtCompileTime>(angle_axis, M);
	}
    void set_quaternion(const Eigen::Quaterniond &q) {
		apply_quaternion<STATE_PARAMETER_COUNT>(q, *this);
    }
    void set_angular_velocity(const Eigen::Vector3d &v) {
        (*this)[ANGULAR_VELOCITY_X] = v.x(); (*this)[ANGULAR_VELOCITY_Y] = v.y(); (*this)[ANGULAR_VELOCITY_Z] = v.z();
    }

	template <int RowsAtCompileTime>
	static void special_state_add(
		const Eigen::Matrix<double, RowsAtCompileTime, 1> &A,
		const Eigen::Matrix<double, RowsAtCompileTime, 1> &B, 
		Eigen::Matrix<double, RowsAtCompileTime, 1> &result) 
	{
		// Extract the orientation quaternion from A (which is stored as an angle axis vector)
		const Eigen::Quaterniond orientation = extract_quaternion<RowsAtCompileTime>(A);

		// Extract the delta quaternion from B (which is also stored as an angle axis vector)
		const Eigen::Quaterniond delta = extract_quaternion<RowsAtCompileTime>(B);

		// Apply the delta to the orientation
		const Eigen::Quaterniond new_rotation = delta*orientation;

		// Save the net rotation rotation back in result
		apply_quaternion<RowsAtCompileTime>(new_rotation, result);
	}	

	template <int RowsAtCompileTime>
	static void special_state_subtract(
		const Eigen::Matrix<double, RowsAtCompileTime, 1> &A,
		const Eigen::Matrix<double, RowsAtCompileTime, 1> &B,
		Eigen::Matrix<double, RowsAtCompileTime, 1> &result)
	{
		// Extract the orientation quaternion from both states (which is stored as an angle axis vector)
		const Eigen::Quaterniond q1= extract_quaternion<RowsAtCompileTime>(A);
		const Eigen::Quaterniond q2= extract_quaternion<RowsAtCompileTime>(B);

		// Compute the "quaternion difference" i.e. rotation from q1 to q2
		const Eigen::Quaterniond q_diff= q2*q1.conjugate();

		apply_quaternion<RowsAtCompileTime>(q_diff, result);
	}

	template <int RowsAtCompileTime, int PointCount>
	static void special_state_mean(
		const Eigen::Matrix<double, RowsAtCompileTime, PointCount>& state_matrix,
		const Eigen::Matrix<double, PointCount, 1> weight_vector,
		Eigen::Matrix<double, RowsAtCompileTime, 1> &result)
	{
		// Extract the orientations from the states
		Eigen::Quaterniond orientations[PointCount];
		double weights[PointCount];
		for (int col_index = 0; col_index < PointCount; ++col_index)
		{
			Eigen::Quaterniond orientation = extract_quaternion<RowsAtCompileTime>(state_matrix.col(col_index));

			orientations[col_index]= orientation;
			weights[col_index]= weight_vector[col_index];
		}

		// Compute the average of the quaternions
		Eigen::Quaterniond average_quat;
		eigen_quaternion_compute_weighted_average(orientations, weights, PointCount, &average_quat);

		// Stomp the incorrect orientation average
		apply_quaternion<RowsAtCompileTime>(average_quat, result);
	}
};

class PSMove_MeasurementVector : public Eigen::Matrix<double, PSMOVE_MEASUREMENT_PARAMETER_COUNT, 1>
{
public:
	PSMove_MeasurementVector(void) : Eigen::Matrix<double, PSMOVE_MEASUREMENT_PARAMETER_COUNT, 1>()
	{ }

	template<typename OtherDerived>
	PSMove_MeasurementVector(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Matrix<double, PSMOVE_MEASUREMENT_PARAMETER_COUNT, 1>(other)
	{ }

	template<typename OtherDerived>
	PSMove_MeasurementVector& operator= (const Eigen::MatrixBase<OtherDerived>& other)
	{
		this->Base::operator=(other);
		return *this;
	}

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
    Eigen::Vector3d get_optical_position() const {
        return Eigen::Vector3d((*this)[PSMOVE_OPTICAL_POSITION_X], (*this)[PSMOVE_OPTICAL_POSITION_Y], (*this)[PSMOVE_OPTICAL_POSITION_Z]);
    }

    // Mutators
    void set_accelerometer(const Eigen::Vector3d &a) {
        (*this)[PSMOVE_ACCELEROMETER_X] = a.x(); (*this)[PSMOVE_ACCELEROMETER_Y] = a.y(); (*this)[PSMOVE_ACCELEROMETER_Z] = a.z();
    }
    void set_gyroscope(const Eigen::Vector3d &g) {
        (*this)[PSMOVE_GYROSCOPE_X] = g.x(); (*this)[PSMOVE_GYROSCOPE_Y] = g.y(); (*this)[PSMOVE_GYROSCOPE_Z] = g.z();
    }
    void set_optical_position(const Eigen::Vector3d &p) {
        (*this)[PSMOVE_OPTICAL_POSITION_X] = p.x(); (*this)[PSMOVE_OPTICAL_POSITION_Y] = p.y(); (*this)[PSMOVE_OPTICAL_POSITION_Z] = p.z();
    }
    void set_magnetometer(const Eigen::Vector3d &m) {
        (*this)[PSMOVE_MAGNETOMETER_X] = m.x(); (*this)[PSMOVE_MAGNETOMETER_Y] = m.y(); (*this)[PSMOVE_MAGNETOMETER_Z] = m.z();
    }

	PSMove_MeasurementVector difference(const PSMove_MeasurementVector &other) const
	{
		// for the PSMove measurement the difference can be computed 
		// with simple vector subtraction
		return (*this) - other;
	}

	template <int SIGMA_POINT_COUNT>
	static PSMove_MeasurementVector computeWeightedMeasurementAverage(
		const Eigen::Matrix<double, PSMOVE_MEASUREMENT_PARAMETER_COUNT, SIGMA_POINT_COUNT>& measurement_matrix,
		const Eigen::Matrix<double, SIGMA_POINT_COUNT, 1> &weight_vector)
	{
		// Use efficient matrix x vector computation to compute a weighted average of the sigma point samples
		// (No orientation stored in measurement means this can be simple)
		PSMove_MeasurementVector result= measurement_matrix * weight_vector;

		return result;
	}
};

class DS4_MeasurementVector : public Eigen::Matrix<double, DS4_MEASUREMENT_PARAMETER_COUNT, 1>
{
public:
	DS4_MeasurementVector(void) : Eigen::Matrix<double, DS4_MEASUREMENT_PARAMETER_COUNT, 1>()
	{ }

	template<typename OtherDerived>
	DS4_MeasurementVector(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Matrix<double, DS4_MEASUREMENT_PARAMETER_COUNT, 1>(other)
	{ }

	template<typename OtherDerived>
	DS4_MeasurementVector& operator= (const Eigen::MatrixBase<OtherDerived>& other)
	{
		this->Base::operator=(other);
		return *this;
	}

    // Accessors
    Eigen::Vector3d get_accelerometer() const {
        return Eigen::Vector3d((*this)[DS4_ACCELEROMETER_X], (*this)[DS4_ACCELEROMETER_Y], (*this)[DS4_ACCELEROMETER_Z]);
    }
    Eigen::Vector3d get_gyroscope() const {
        return Eigen::Vector3d((*this)[DS4_GYROSCOPE_X], (*this)[DS4_GYROSCOPE_Y], (*this)[DS4_GYROSCOPE_Z]);
    }
    Eigen::Vector3d get_optical_position() const {
        return Eigen::Vector3d((*this)[DS4_OPTICAL_POSITION_X], (*this)[DS4_OPTICAL_POSITION_Y], (*this)[DS4_OPTICAL_POSITION_Z]);
    }
    Eigen::AngleAxisd get_optical_angle_axis() const {
        Eigen::Vector3d axis= Eigen::Vector3d((*this)[DS4_OPTICAL_ANGLE_AXIS_X], (*this)[DS4_OPTICAL_ANGLE_AXIS_Y], (*this)[DS4_OPTICAL_ANGLE_AXIS_Z]);
        const double angle= eigen_vector3d_normalize_with_default(axis, Eigen::Vector3d::Zero());
        return Eigen::AngleAxisd(angle, axis);
    }
    Eigen::Quaterniond get_optical_quaternion() const {
        return Eigen::Quaterniond(get_optical_angle_axis());
    }

    // Mutators
    void set_accelerometer(const Eigen::Vector3d &a) {
        (*this)[DS4_ACCELEROMETER_X] = a.x(); (*this)[DS4_ACCELEROMETER_Y] = a.y(); (*this)[DS4_ACCELEROMETER_Z] = a.z();
    }
    void set_gyroscope(const Eigen::Vector3d &g) {
        (*this)[DS4_GYROSCOPE_X] = g.x(); (*this)[DS4_GYROSCOPE_Y] = g.y(); (*this)[DS4_GYROSCOPE_Z] = g.z();
    }
    void set_optical_position(const Eigen::Vector3d &p) {
        (*this)[DS4_OPTICAL_POSITION_X] = p.x(); (*this)[DS4_OPTICAL_POSITION_Y] = p.y(); (*this)[DS4_OPTICAL_POSITION_Z] = p.z();
    }
    void set_angle_axis(const Eigen::AngleAxisd &a) {        
        const double angle= a.angle();
        (*this)[DS4_OPTICAL_ANGLE_AXIS_X] = a.axis().x() * angle; 
        (*this)[DS4_OPTICAL_ANGLE_AXIS_Y] = a.axis().y() * angle;
        (*this)[DS4_OPTICAL_ANGLE_AXIS_Z] = a.axis().z() * angle;
    }
    void set_optical_quaternion(const Eigen::Quaterniond &q) {
        const Eigen::AngleAxisd angle_axis(q);
        set_angle_axis(angle_axis);
    }

	DS4_MeasurementVector difference(const DS4_MeasurementVector &other) const
	{
		DS4_MeasurementVector measurement_diff= (*this) - other;
		
		const Eigen::Quaterniond q1= this->get_optical_quaternion();
		const Eigen::Quaterniond q2= other.get_optical_quaternion();
		const Eigen::Quaterniond q_diff= q2*q1.conjugate();

		// Stomp the incorrect orientation difference computed by the vector subtraction
		measurement_diff.set_optical_quaternion(q_diff);

		return measurement_diff;
	}

	template <int SIGMA_POINT_COUNT>
	static DS4_MeasurementVector computeWeightedMeasurementAverage(
		const Eigen::Matrix<double, DS4_MEASUREMENT_PARAMETER_COUNT, SIGMA_POINT_COUNT>& measurement_matrix,
		Eigen::Matrix<double, SIGMA_POINT_COUNT, 1> weight_vector)
	{
		// Use efficient matrix x vector computation to compute a weighted average of the measurements
		// (the orientation portion will be wrong)
		DS4_MeasurementVector result= measurement_matrix * weight_vector;

		// Extract the orientations from the measurements
		Eigen::Quaterniond orientations[SIGMA_POINT_COUNT];
		double weights[SIGMA_POINT_COUNT];
		for (int col_index = 0; col_index <= SIGMA_POINT_COUNT; ++col_index)
		{
			const DS4_MeasurementVector measurement = measurement_matrix.col(col_index);
			Eigen::Quaterniond orientation = measurement.get_optical_quaternion();

			orientations[col_index]= orientation;
			weights[col_index]= weight_vector[col_index];
		}

		// Compute the average of the quaternions
		Eigen::Quaterniond average_quat;
		eigen_quaternion_compute_weighted_average(orientations, weights, SIGMA_POINT_COUNT, &average_quat);

		// Stomp the incorrect orientation average
		result.set_optical_quaternion(average_quat);

		return result;
	}
};

/**
* @brief Measurement model for measuring PSMove controller
*
* This is the measurement model for measuring the position and magnetometer of the PSMove controller.
* The measurement is given by the optical trackers.
*/
class PSMove_MeasurementModel
{
public:
    void init(const PoseFilterConstants &constants)
    {
		update_measurement_statistics(constants, 0.f);
        
		identity_gravity_direction= constants.orientation_constants.gravity_calibration_direction.cast<double>();
		identity_magnetometer_direction= constants.orientation_constants.magnetometer_calibration_direction.cast<double>();
    }

	void update_measurement_statistics(
		const PoseFilterConstants &constants,
		const float position_quality)
	{
        // Start off using the maximum standard deviation values
		const double accelerometer_std_dev= R_SCALE*sqrtf(constants.position_constants.accelerometer_variance);
		const double gyro_std_dev= R_SCALE*sqrtf(constants.orientation_constants.gyro_variance);
		const double magnetometer_std_dev= R_SCALE*sqrtf(constants.orientation_constants.magnetometer_variance);
        const double position_std_dev= 
			R_SCALE*sqrtf(lerp_clampf(
				constants.position_constants.max_position_variance,
				constants.position_constants.min_position_variance,
				position_quality));

		// TODO: For now we are assuming the noise is mean-zero
		R_mu = Eigen::Matrix<double, PSMOVE_MEASUREMENT_PARAMETER_COUNT, 1>::Zero();

        // Update the measurement covariance R
        R_cov = Eigen::Matrix<double, PSMOVE_MEASUREMENT_PARAMETER_COUNT, PSMOVE_MEASUREMENT_PARAMETER_COUNT>::Zero();

		// Only diagonals used so no need to compute Cholesky
		R_cov(PSMOVE_ACCELEROMETER_X, PSMOVE_ACCELEROMETER_X) = accelerometer_std_dev;
		R_cov(PSMOVE_ACCELEROMETER_Y, PSMOVE_ACCELEROMETER_Y) = accelerometer_std_dev;
		R_cov(PSMOVE_ACCELEROMETER_Z, PSMOVE_ACCELEROMETER_Z) = accelerometer_std_dev;
		R_cov(PSMOVE_GYROSCOPE_X, PSMOVE_GYROSCOPE_X)= gyro_std_dev;
		R_cov(PSMOVE_GYROSCOPE_Y, PSMOVE_GYROSCOPE_Y)= gyro_std_dev;
		R_cov(PSMOVE_GYROSCOPE_Z, PSMOVE_GYROSCOPE_Z)= gyro_std_dev;
        R_cov(PSMOVE_MAGNETOMETER_X, PSMOVE_MAGNETOMETER_X) = magnetometer_std_dev;
        R_cov(PSMOVE_MAGNETOMETER_Y, PSMOVE_MAGNETOMETER_Y) = magnetometer_std_dev;
        R_cov(PSMOVE_MAGNETOMETER_Z, PSMOVE_MAGNETOMETER_Z) = magnetometer_std_dev;
        R_cov(PSMOVE_OPTICAL_POSITION_X, PSMOVE_OPTICAL_POSITION_X) = position_std_dev;
        R_cov(PSMOVE_OPTICAL_POSITION_Y, PSMOVE_OPTICAL_POSITION_Y) = position_std_dev;
        R_cov(PSMOVE_OPTICAL_POSITION_Z, PSMOVE_OPTICAL_POSITION_Z) = position_std_dev;
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
    PSMove_MeasurementVector observation_function(const PoseStateVector& state, const PSMove_MeasurementVector &observation_noise) const
    {
        PSMove_MeasurementVector predicted_measurement;

		// Extract the observations noise
		const Eigen::Vector3d accel_noise = observation_noise.get_accelerometer();
		const Eigen::Vector3d mag_noise = observation_noise.get_magnetometer();
		const Eigen::Vector3d gyro_noise = observation_noise.get_gyroscope();
		const Eigen::Vector3d position_noise = observation_noise.get_optical_position();

        // Use the position and orientation from the state for predictions
        const Eigen::Vector3d position= state.get_position();
        const Eigen::Quaterniond orientation= state.get_quaternion();

        // Use the current linear acceleration from the state to predict
        // what the accelerometer reading will be (in world space)
        const Eigen::Vector3d gravity_accel_g_units= -identity_gravity_direction;
        const Eigen::Vector3d linear_accel_g_units= state.get_linear_acceleration() * k_ms2_to_g_units;
        const Eigen::Vector3d accel_world= linear_accel_g_units + gravity_accel_g_units;
        const Eigen::Quaterniond accel_world_quat(0.f, accel_world.x(), accel_world.y(), accel_world.z());

        // Put the accelerometer prediction into the local space of the controller
        const Eigen::Vector3d accel_local= orientation*(accel_world_quat*orientation.conjugate()).vec();

        // Use the angular velocity from the state to predict what the gyro reading will be
        const Eigen::Vector3d gyro_local= state.get_angular_velocity(); 

        // Use the orientation from the state to predict
        // what the magnetometer reading should be
        const Eigen::Vector3d &mag_world= identity_magnetometer_direction;
        const Eigen::Quaterniond mag_world_quat(0.f, mag_world.x(), mag_world.y(), mag_world.z());
        const Eigen::Vector3d mag_local= orientation*(mag_world_quat*orientation.conjugate()).vec();

        // Save the predictions into the measurement vector
        predicted_measurement.set_accelerometer(accel_local + accel_noise);
        predicted_measurement.set_magnetometer(mag_local + mag_noise);
        predicted_measurement.set_gyroscope(gyro_local + gyro_noise);
        predicted_measurement.set_optical_position(position + position_noise);

        return predicted_measurement;
    }

public:
    Eigen::Vector3d identity_gravity_direction;
    Eigen::Vector3d identity_magnetometer_direction;

	//! Measurement noise mean
	Eigen::Matrix<double, PSMOVE_MEASUREMENT_PARAMETER_COUNT, 1> R_mu;

	//! Measurement noise covariance
	Eigen::Matrix<double, PSMOVE_MEASUREMENT_PARAMETER_COUNT, PSMOVE_MEASUREMENT_PARAMETER_COUNT> R_cov;
};

/**
* @brief Measurement model for measuring DS4 controller
*
* This is the measurement model for measuring the position and orientation of the DS4 controller.
* The measurement is given by the optical trackers.
*/
class DS4_MeasurementModel
{
public:
    void init(const PoseFilterConstants &constants)
    {
		update_measurement_statistics(constants,0.f, 0.f);

		identity_gravity_direction= constants.orientation_constants.gravity_calibration_direction.cast<double>();
    }

	void update_measurement_statistics(
		const PoseFilterConstants &constants,
		const float position_quality,
		const float orientation_quality)
	{
        // Start off using the maximum variance values
		const double accelerometer_std_dev= R_SCALE*sqrtf(constants.position_constants.accelerometer_variance);
		const double gyro_std_dev= R_SCALE*sqrtf(constants.orientation_constants.gyro_variance);
        const double position_std_dev= 
			R_SCALE*sqrtf(lerp_clampf(
				constants.position_constants.max_position_variance,
				constants.position_constants.min_position_variance,
				position_quality));
		const double angle_axis_std_dev=
			R_SCALE*sqrtf(lerp_clampf(
				constants.orientation_constants.max_orientation_variance,
				constants.orientation_constants.min_orientation_variance,
				orientation_quality));

		// TODO: For now we are assuming the noise is mean-zero
		R_mu = Eigen::Matrix<double, PSMOVE_MEASUREMENT_PARAMETER_COUNT, 1>::Zero();

        // Update the measurement covariance R
        R_cov = Eigen::Matrix<double, PSMOVE_MEASUREMENT_PARAMETER_COUNT, PSMOVE_MEASUREMENT_PARAMETER_COUNT>::Zero();
		R_cov(DS4_ACCELEROMETER_X, DS4_ACCELEROMETER_X) = accelerometer_std_dev;
		R_cov(DS4_ACCELEROMETER_Y, DS4_ACCELEROMETER_Y) = accelerometer_std_dev;
		R_cov(DS4_ACCELEROMETER_Z, DS4_ACCELEROMETER_Z) = accelerometer_std_dev;
		R_cov(DS4_GYROSCOPE_X, DS4_GYROSCOPE_X)= gyro_std_dev;
		R_cov(DS4_GYROSCOPE_Y, DS4_GYROSCOPE_Y)= gyro_std_dev;
		R_cov(DS4_GYROSCOPE_Z, DS4_GYROSCOPE_Z)= gyro_std_dev;
        R_cov(DS4_OPTICAL_POSITION_X, DS4_OPTICAL_POSITION_X) = position_std_dev;
        R_cov(DS4_OPTICAL_POSITION_Y, DS4_OPTICAL_POSITION_Y) = position_std_dev;
        R_cov(DS4_OPTICAL_POSITION_Z, DS4_OPTICAL_POSITION_Z) = position_std_dev;
        R_cov(DS4_OPTICAL_ANGLE_AXIS_X, DS4_OPTICAL_ANGLE_AXIS_X) = angle_axis_std_dev; 
        R_cov(DS4_OPTICAL_ANGLE_AXIS_Y, DS4_OPTICAL_ANGLE_AXIS_Y) = angle_axis_std_dev;
        R_cov(DS4_OPTICAL_ANGLE_AXIS_Z, DS4_OPTICAL_ANGLE_AXIS_Z) = angle_axis_std_dev;
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
    DS4_MeasurementVector observation_function(const PoseStateVector& state, const DS4_MeasurementVector &observation_noise) const
    {
        DS4_MeasurementVector predicted_measurement;

		// Extract the observations noise
		const Eigen::Vector3d accel_noise= observation_noise.get_accelerometer();
		const Eigen::Vector3d gyro_noise= observation_noise.get_gyroscope();
		const Eigen::Vector3d position_noise= observation_noise.get_optical_position();
		const Eigen::Quaterniond orientation_noise= observation_noise.get_optical_quaternion();

        // Use the position and orientation from the state for predictions
        const Eigen::Vector3d position= state.get_position();
        const Eigen::Quaterniond orientation= state.get_quaternion();

        // Use the current linear acceleration from the state to predict
        // what the accelerometer reading will be (in world space)
        const Eigen::Vector3d gravity_accel_g_units= -identity_gravity_direction;
        const Eigen::Vector3d linear_accel_g_units= state.get_linear_acceleration() * k_ms2_to_g_units;
        const Eigen::Vector3d accel_world= linear_accel_g_units + gravity_accel_g_units;
        const Eigen::Quaterniond accel_world_quat(0.f, accel_world.x(), accel_world.y(), accel_world.z());

        // Put the accelerometer prediction into the local space of the controller
        const Eigen::Vector3d accel_local= orientation*(accel_world_quat*orientation.conjugate()).vec();

        // Use the angular velocity from the state to predict what the gyro reading will be
        const Eigen::Vector3d gyro_local= state.get_angular_velocity(); 

        // Save the predictions into the measurement vector
        predicted_measurement.set_accelerometer(accel_local + accel_noise);
        predicted_measurement.set_gyroscope(gyro_local + gyro_noise);
        predicted_measurement.set_optical_position(position + position_noise);
        predicted_measurement.set_optical_quaternion(orientation_noise*orientation);

        return predicted_measurement;
    }

public:
    Eigen::Vector3d identity_gravity_direction;

	//! Measurement noise mean
	Eigen::Matrix<double, DS4_MEASUREMENT_PARAMETER_COUNT, 1> R_mu;

	//! Measurement noise covariance
	Eigen::Matrix<double, DS4_MEASUREMENT_PARAMETER_COUNT, DS4_MEASUREMENT_PARAMETER_COUNT> R_cov;
};

template <int SIGMA_POINT_COUNT>
class SigmaPointWeights
{
public:
	/// Scaling factor for the sigma points
	double zeta;

	/// Sigma weights (m)
	Eigen::Matrix<double, SIGMA_POINT_COUNT, 1> wm;

	/// Sigma weights (c)
	Eigen::Matrix<double, SIGMA_POINT_COUNT, 1> wc;

	double w_qr;
	double w_cholup;

	SigmaPointWeights()
	{
		zeta = 0.0;
		wm = Eigen::Matrix<double, SIGMA_POINT_COUNT, 1>::Zero();
		wc = Eigen::Matrix<double, SIGMA_POINT_COUNT, 1>::Zero();
		w_qr = 0.0;
		w_cholup = 0.0;
	}

	/**
	* @param [in] alpha Scaling parameter for spread of sigma points (usually \f$ 1E-4 \leq \alpha \leq 1 \f$)
	* @param [in] beta Parameter for prior knowledge about the distribution (\f$ \beta = 2 \f$ is optimal for Gaussian)
	* @param [in] kappa Secondary scaling parameter (usually 0)
	*/
	void init(double alpha, double beta, double kappa)
	{	
		// Compute the augmented state size
		const double L = static_cast<double>((SIGMA_POINT_COUNT - 1) / 2);

		// For standard UKF, here are the weights...

		// compound scaling parameter
		double lambda = alpha * alpha * (L + kappa) - L;

		// Scaling factor for sigma points.
		zeta = sqrt(L + lambda);

		// Make sure L != -lambda to avoid division by zero
		assert(fabs(L + lambda) > 1e-6);

		// Make sure L != -kappa to avoid division by zero
		assert(fabs(L + kappa) > 1e-6);

		// Fill in the mean-weights
		double wm_0 = lambda / (L + lambda);
		double wm_rest = 0.5 / (L + lambda);
			
		// Make sure wm_rest > 0 to avoid square-root of negative number
		assert(wm_rest > 0.0);

		// wm = weights for calculating mean(both process and observation)
		wm[0] = wm_0;
		for (int point_index = 1; point_index < SIGMA_POINT_COUNT; ++point_index)
		{
			wm[point_index] = wm_rest;
		}

		// Fill in the covariance-weights
		double wc_0 = wm_0 + (1.0 - alpha*alpha + beta);
		double wc_rest = wm_rest;

		// wc = weights for calculating covariance(proc., obs., proc - obs)
		wc[0] = wc_0;
		for (int point_index = 1; point_index < SIGMA_POINT_COUNT; ++point_index)
		{
			wc[point_index] = wc_rest;
		}

		// For SRUKF, we also need sqrt of wc_rest for chol update.
		w_qr = sqrt(wc[2]);
		w_cholup = sqrt(fabs(wc(1)));
	}
};

// Specialized Square Root Unscented Kalman Filter (SR-UKF)
template<class MeasurementModelType, class Measurement>
class PoseSRUFK
{
public:
	static const int X_DIM = STATE_PARAMETER_COUNT;
	static const int O_DIM = Measurement::RowsAtCompileTime;
	static const int Q_DIM = STATE_PARAMETER_COUNT;
	static const int R_DIM = Measurement::RowsAtCompileTime;
	static const int L_DIM = X_DIM + Q_DIM + R_DIM;
	static const int SIGMA_POINT_COUNT = 2 * L_DIM + 1;

	//! Type of the state vector
	typedef PoseStateVector State;
        
	//! Estimated state
	State x;

	//! Lower-triangular Cholesky factor of state covariance
	Eigen::Matrix<double, STATE_PARAMETER_COUNT, STATE_PARAMETER_COUNT> S;

	//! Process noise mean
	Eigen::Matrix<double, STATE_PARAMETER_COUNT, 1> Q_mu;

	//! The "square root" of the process noise covariance a.k.a. the lower part of the Choleskly
	Eigen::Matrix<double, STATE_PARAMETER_COUNT, STATE_PARAMETER_COUNT> Q_cov;

	MeasurementModelType measurement_model;

	SigmaPointWeights<SIGMA_POINT_COUNT> W;

	// Sigma points at time t = k - 1
	Eigen::Matrix<double, L_DIM, SIGMA_POINT_COUNT> X_t;

	// Sigma points propagated through process function to time k
	Eigen::Matrix<double, X_DIM, SIGMA_POINT_COUNT> X_k;

	// State estimate = weighted sum of sigma points
	Eigen::Matrix<double, X_DIM, 1> x_k;

	// Propagated sigma point residuals = (sp - x_k)
	Eigen::Matrix<double, X_DIM, SIGMA_POINT_COUNT> X_k_r;

	// Upper - triangular of propagated sp covariance
	Eigen::Matrix<double, X_DIM, X_DIM > Sx_k;
                    
public:
	PoseSRUFK()
	{
		// Setup state and covariance
		x.setZero();
		S.setIdentity();
		Q_mu.setZero();
		Q_cov.setIdentity();
	}

	void init(const PoseFilterConstants &constants)
	{
		const double mean_position_dT = constants.position_constants.mean_update_time_delta;
		const double mean_orientation_dT = constants.position_constants.mean_update_time_delta;

		// Start off using the maximum variance values
		const double position_variance =
			(constants.position_constants.min_position_variance +
				constants.position_constants.max_position_variance) * 0.5f * Q_SCALE;
		const double angle_axis_variance =
			(constants.orientation_constants.min_orientation_variance +
				constants.orientation_constants.max_orientation_variance) * 0.5f* Q_SCALE;

		// TODO: Initial guess at state covariance square root from filter constants?
		S = Eigen::Matrix<double, STATE_PARAMETER_COUNT, STATE_PARAMETER_COUNT>::Identity() * 0.1;

		// Process noise should be mean-zero, I think.
		Q_mu = Eigen::Matrix<double, STATE_PARAMETER_COUNT, 1>::Zero();

		// Initialize the process covariance matrix Q
		Eigen::Matrix<double, STATE_PARAMETER_COUNT, STATE_PARAMETER_COUNT> Q_cov_init=
			Eigen::Matrix<double, STATE_PARAMETER_COUNT, STATE_PARAMETER_COUNT>::Zero();
		process_3rd_order_noise(mean_position_dT, position_variance, POSITION_X, Q_cov_init);
		process_3rd_order_noise(mean_position_dT, position_variance, POSITION_Y, Q_cov_init);
		process_3rd_order_noise(mean_position_dT, position_variance, POSITION_Z, Q_cov_init);
		process_2nd_order_noise(mean_orientation_dT, angle_axis_variance, ANGLE_AXIS_X, Q_cov_init);
		process_2nd_order_noise(mean_orientation_dT, angle_axis_variance, ANGLE_AXIS_Y, Q_cov_init);
		process_2nd_order_noise(mean_orientation_dT, angle_axis_variance, ANGLE_AXIS_Z, Q_cov_init);

		// Compute the std-deviation Q matrix a.k.a. the sqrt of Q_cov_init a.k.a the Cholesky
		Q_cov= Q_cov_init.llt().matrixL();

		// Initialize the measurement noise
		measurement_model.init(constants);

		//%% 1. Initialize the sigma point weights
		W.init(k_ukf_alpha, k_ukf_beta, k_ukf_kappa);
	}

	/**
	* @brief Definition of (non-linear) state transition function
	*
	* This function defines how the system state is propagated through time,
	* i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to
	* be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
	* the system control input \f$u\f$.
	*
	* @param [in] old_state The system state in current time-step
	* @param [in] process_noise The control vector input
	* @returns The (predicted) system state in the next time-step
	*/
	PoseStateVector process_function(
		const PoseStateVector& old_state,  
		const PoseStateVector& process_noise,
		const float deltaTime) const
	{
		//! Predicted state vector after transition
		PoseStateVector new_state;

		// Extract parameters from the old state
		const Eigen::Vector3d old_position = old_state.get_position();
		const Eigen::Vector3d old_linear_velocity = old_state.get_linear_velocity();
		const Eigen::Vector3d old_linear_acceleration = old_state.get_linear_acceleration();
		const Eigen::Quaterniond old_orientation = old_state.get_quaternion();
		const Eigen::Vector3d old_angular_velocity = old_state.get_angular_velocity();

		// Extract parameters from process noise
		const Eigen::Vector3d position_noise = process_noise.get_position();
		const Eigen::Vector3d linear_velocity_noise = process_noise.get_linear_velocity();
		const Eigen::Vector3d linear_acceleration_noise = process_noise.get_linear_acceleration();
		const Eigen::Quaterniond orientation_noise = process_noise.get_quaternion();
		const Eigen::Vector3d angular_velocity_noise = process_noise.get_angular_velocity();

		// Compute the position state update
		const Eigen::Vector3d new_position =
			old_position
			+ old_linear_velocity*deltaTime
			+ old_linear_acceleration*deltaTime*deltaTime*0.5f
			+ position_noise;
		const Eigen::Vector3d new_linear_velocity = old_linear_velocity + old_linear_acceleration*deltaTime + linear_velocity_noise;
		const Eigen::Vector3d &new_linear_acceleration = old_linear_acceleration + linear_acceleration_noise;

		// Compute the orientation update
		const Eigen::Quaterniond quaternion_derivative =
			eigen_angular_velocity_to_quaterniond_derivative(old_orientation, old_angular_velocity);
		const Eigen::Quaterniond new_orientation = 
			orientation_noise *
			Eigen::Quaterniond(
				old_orientation.coeffs()
				+ quaternion_derivative.coeffs()*deltaTime).normalized();

		const Eigen::Vector3d &new_angular_velocity = old_angular_velocity + angular_velocity_noise;

		// Save results to the new state
		new_state.set_position(new_position);
		new_state.set_linear_velocity(new_linear_velocity);
		new_state.set_linear_acceleration(new_linear_acceleration);
		new_state.set_quaternion(new_orientation);
		new_state.set_angular_velocity(new_angular_velocity);

		return new_state;
	}

	/**
	* @brief Perform filter prediction step using control input \f$u\f$ and corresponding system model
	*
	* @param [in] deltaTime Seconds since the last update
	*/
	void predict(const float deltaTime)
	{
		const int nsp = SIGMA_POINT_COUNT;

		// In the below variables, the subscripts are as follows
		// k is the next / predicted time point
		// t = k - 1 (time point of previous estimate)

		//%% 2. Build augmented state vector
		Eigen::Matrix<double, L_DIM, 1> x_t;
		x_t.segment<X_DIM>(0) = x;
		x_t.segment<Q_DIM>(X_DIM) = Q_mu;
		x_t.segment<R_DIM>(X_DIM + Q_DIM) = measurement_model.R_mu;

		//%% 3. Build augmented sqrt state covariance matrix
		const int X_inds = 0;
		const int Q_inds = X_inds+ X_DIM;
		const int R_inds = Q_inds+ Q_DIM;
		Eigen::Matrix<double, L_DIM, L_DIM> S_a= Eigen::Matrix<double, L_DIM, L_DIM>::Zero(); // = sqrt(state_covariance)
		S_a.block<X_DIM, X_DIM>(X_inds, X_inds) = S;
		S_a.block<Q_DIM, Q_DIM>(Q_inds, Q_inds) = Q_cov;
		S_a.block<R_DIM, R_DIM>(R_inds, R_inds) = measurement_model.R_cov;

		//%% 4. Calculate augmented sigma points
		Eigen::Matrix<double, L_DIM, L_DIM> zetaSa = S_a * W.zeta;

		// Note that for each column in X_t:
		// The first Xdim rows will be the state sigma points.
		// The next Qdim rows will be used for process noise.
		// The last Rdim rows will be used for observation noise.

		// Initially compute the sigma points assuming that that all state
		// is can added to or subtracted from (not true for the rotation portion)
		X_t.leftCols<1>() = x_t;
		// Set center block with x_t + zeta * S_a
		X_t.block<L_DIM, L_DIM>(0, 1) = zetaSa.colwise() + x_t;
		// Set right block with x - zeta * S_a
		X_t.rightCols<L_DIM>() = (-zetaSa).colwise() + x_t;

		// Handle the rotation portion next (stomping what was calculated before)
		for (int col_offset = 0; col_offset < L_DIM; ++col_offset)
		{
			const Eigen::Matrix<double, L_DIM, 1> zetaSa_State_i = zetaSa.block<L_DIM, 1>(0, col_offset);

			//TODO: See if there is a way to pass a block reference into a function
			Eigen::Matrix<double, L_DIM, 1> add_result = X_t.block<L_DIM, 1>(0, 1 + col_offset);
			PoseStateVector::special_state_add<L_DIM>(
				x_t,
				zetaSa_State_i,
				add_result);
			X_t.block<L_DIM, 1>(0, 1 + col_offset) = add_result;

			//TODO: See if there is a way to pass a block reference into a function
			Eigen::Matrix<double, L_DIM, 1> sub_result = X_t.block<L_DIM, 1>(0, 1 + col_offset);
			PoseStateVector::special_state_subtract<L_DIM>(
				x_t,
				zetaSa_State_i,
				sub_result);
			X_t.block<L_DIM, 1>(0, 1 + L_DIM + col_offset) = sub_result;
		}

		//%% 5. Propagate sigma points through process function
		// X_k = process_function(X_t(1:filt_struct.Xdim, : ), X_t(Q_inds, :), dt);
		for (int point_index = 0; point_index < nsp; ++point_index)
		{
			X_k.col(point_index) = 
				process_function(
					X_t.block<X_DIM, 1>(0, point_index), // Slice out the state vector for sigma point
					X_t.block<Q_DIM, 1>(Q_inds, point_index), // Slice out the covariance for the sigma point
					deltaTime);
		}

		// %% 6. Estimate mean state from weighted sum of propagated sigma points
		x_k = X_k * W.wm;
		PoseStateVector::special_state_mean<X_DIM, nsp>(X_k, W.wm, x_k);

		// %% 7. Get residuals
		X_k_r = X_k.colwise() - x_k;
		for (int col_offset = 0; col_offset < L_DIM; ++col_offset)
		{
			//TODO: See if there is a way to pass a block reference into a function
			Eigen::Matrix<double, X_DIM, 1> sub_result = X_k.col(col_offset);
			PoseStateVector::special_state_subtract<X_DIM>(
				X_k.col(col_offset),
				x_k,
				sub_result);
			X_k.col(col_offset) = sub_result;
		}

		//%% 8. Estimate state covariance(sqrt)
		// w_qr is scalar
		// QR update of state Cholesky factor.
		// w_qr and w_cholup cannot be negative
		Eigen::Matrix<double, nsp - 1, X_DIM > qr_input = (W.w_qr*X_k_r.rightCols<nsp - 1>()).transpose();

		// TODO: Use ColPivHouseholderQR
		Eigen::HouseholderQR<decltype(qr_input)> qr(qr_input);

		// Set R matrix as upper triangular square root
		Sx_k = qr.matrixQR().topRightCorner<X_DIM, X_DIM>().triangularView<Eigen::Upper>();

		// Perform additional rank 1 update
		Sx_k.selfadjointView<Eigen::Upper>().rankUpdate(X_k_r.leftCols<1>(), W.w_cholup);
	}

	/**
	* @brief Perform filter update step using measurement \f$z\f$ and corresponding measurement model
	*
	* @param [in] observation The measurement vector
	*/
	void update(const Measurement& observation)
	{

		//%% 1. Propagate sigma points through observation function.
		const int nsp = SIGMA_POINT_COUNT;
		const int X_inds = 0;
		const int Q_inds = X_inds + X_DIM;
		const int R_inds = Q_inds + Q_DIM;

		//Y_k = observation_function(filt_struct, X_k, X_t(R_inds, :));
		Eigen::Matrix<double, O_DIM, nsp> Y_k;
		for (int point_index = 0; point_index < nsp; ++point_index)
		{
			Y_k.col(point_index) =
				measurement_model.observation_function(
					X_k.col(point_index), // Slice out the state vector for sigma point
					X_t.block<R_DIM, 1>(R_inds, point_index)); // Slice out the measurement noise
		}

		//%% 2. Calculate observation mean.
		Measurement y_k = Measurement::computeWeightedMeasurementAverage<nsp>(Y_k, W.wm);

		//%% 3. Calculate y - residuals.
		// Used in observation covariance and state - observation cross - covariance for Kalman gain.
		Eigen::Matrix<double, O_DIM, nsp>  Y_k_r;
		for (int col_offset = 0; col_offset < nsp; ++col_offset)
		{
			Y_k_r.col(col_offset)= Measurement(Y_k.col(col_offset)).difference(y_k);
		}

		//%% 4. Calculate observation sqrt covariance
		// w_qr is scalar
		// QR update of state Cholesky factor.
		// w_qr and w_cholup cannot be negative
		Eigen::Matrix<double, nsp - 1, O_DIM> qr_input = (W.w_qr*Y_k_r.rightCols<nsp - 1>()).transpose();

		// TODO: Use ColPivHouseholderQR
		Eigen::HouseholderQR<decltype(qr_input)> qr(qr_input);

		// Set R matrix as upper triangular square root
		Eigen::Matrix<double, O_DIM, O_DIM > Sy_k = qr.matrixQR().topRightCorner<O_DIM, O_DIM>().triangularView<Eigen::Upper>();

		// Perform additional rank 1 update
		Sy_k.selfadjointView<Eigen::Upper>().rankUpdate(Y_k_r.leftCols<1>(), W.w_cholup);

		// We need the lower triangular Cholesky factor
		Sy_k = Sy_k.transpose();

		//%% 5. Calculate Kalman Gain
		Eigen::Matrix<double, X_DIM, O_DIM> Pxy;
		Pxy.setZero();
		for (int point_index = 0; point_index < nsp; ++point_index)
		{
			//%TODO : Should X_k_r axisAngles be multiplied directly like this ?
			Pxy = Pxy + W.wc(point_index) * (X_k_r.col(point_index) * Y_k_r.col(point_index).transpose());
		}

		// KG = (Pxy / Sy_k')/Sy_k, where "/" is the "mrdivide" operator in matlab
		// The following: http://stackoverflow.com/questions/31705168/armadillos-solvea-b-returning-different-answer-from-matlab-eigen
		// says A mrdivide B is equivalent to the following in Eigen:
		// A.transpose().colPivHouseholderQr().solve(B.transpose())
		// Therefore numerator= (Pxy / Sy_k') becomes:
		Eigen::Matrix<double, X_DIM, O_DIM> numerator= (Pxy.transpose().colPivHouseholderQr().solve(Sy_k));
		// and KG= numerator/Sy_k becomes:
		Eigen::Matrix<double, X_DIM, O_DIM> KG = (numerator.transpose().colPivHouseholderQr().solve(Sy_k.transpose()));

		// %% 6. Calculate innovation
		Measurement innov = observation.difference(y_k);

		// %% 7. State update / correct
		PoseStateVector upd(KG*innov);
		x = x_k + upd;
		PoseStateVector::special_state_add<X_DIM>(x_k, upd, x);

		// %% 8. Covariance update / correct
		// This is equivalent to : Px = Px_ - KG*Py*KG';
		Eigen::Matrix<double, X_DIM, O_DIM> cov_update_vectors = KG * Sy_k;
		for (int j = 0; j < O_DIM; ++j)
		{
			Sx_k.selfadjointView<Eigen::Lower>().rankUpdate(cov_update_vectors.col(j), -1.0);
		}		
		S = Sx_k.transpose();
	}
};

class KalmanPoseFilterImpl
{
public:
    /// Is the current fusion state valid
    bool bIsValid;

    /// True if we have seen a valid position measurement (>0 position quality)
    bool bSeenPositionMeasurement;

	/// True if we have seen a valid orientation measurement (>0 orientation quality)
	bool bSeenOrientationMeasurement;

    /// Quaternion measured when controller points towards camera 
    Eigen::Quaternionf reset_orientation;

    /// Position that's considered the origin position 
    Eigen::Vector3f origin_position; // meters

    /// The last published state from the filter
	PoseStateVector state;

	KalmanPoseFilterImpl()
    {
    }

    virtual void init(const PoseFilterConstants &constants)
    {
        bIsValid = false;
		bSeenPositionMeasurement= false;
		bSeenOrientationMeasurement= false;

        reset_orientation = Eigen::Quaternionf::Identity();
        origin_position = Eigen::Vector3f::Zero();
		state = PoseStateVector::Zero();
    }
};

class DS4KalmanPoseFilterImpl : public KalmanPoseFilterImpl
{
public:
	PoseSRUFK<DS4_MeasurementModel, DS4_MeasurementVector> srukf;

	void init(const PoseFilterConstants &constants) override
	{
		KalmanPoseFilterImpl::init(constants);
		srukf.init(constants);
	}
};

class PSMoveKalmanPoseFilterImpl : public KalmanPoseFilterImpl
{
public:
	PoseSRUFK<PSMove_MeasurementModel, PSMove_MeasurementVector> srukf;

	void init(const PoseFilterConstants &constants) override
	{
		KalmanPoseFilterImpl::init(constants);
		srukf.init(constants);
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
        const Eigen::Quaternionf state_orientation = m_filter->state.get_quaternion().cast<float>();
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
	Eigen::Vector3d ang_vel= m_filter->state.get_angular_velocity();
	
    return ang_vel.cast<float>();
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
        Eigen::Vector3f state_position= m_filter->state.get_position().cast<float>();
        Eigen::Vector3f predicted_position =
            is_nearly_zero(time)
            ? state_position
            : state_position + getVelocity() * time;

        result = (predicted_position - m_filter->origin_position) * k_meters_to_centimeters;
    }

    return result;
}

Eigen::Vector3f KalmanPoseFilter::getVelocity() const
{
	Eigen::Vector3d vel= m_filter->state.get_linear_velocity() * k_meters_to_centimeters;

    return vel.cast<float>();
}

Eigen::Vector3f KalmanPoseFilter::getAcceleration() const
{
    Eigen::Vector3d accel= m_filter->state.get_linear_acceleration() * k_meters_to_centimeters;

	return accel.cast<float>();
}

//-- KalmanFilterOpticalPoseARG --
bool KalmanPoseFilterDS4::init(const PoseFilterConstants &constants)
{
    KalmanPoseFilter::init(constants);

    DS4KalmanPoseFilterImpl *filter = new DS4KalmanPoseFilterImpl();
    filter->init(constants);
    m_filter = filter;

    return true;
}

void KalmanPoseFilterDS4::update(const float delta_time, const PoseFilterPacket &packet)
{
	// Get the DS4 implementation specific sigma point weights and measurement model
	DS4KalmanPoseFilterImpl *ds4FilterImpl = static_cast<DS4KalmanPoseFilterImpl *>(m_filter);
	PoseSRUFK<DS4_MeasurementModel, DS4_MeasurementVector> &srukf = ds4FilterImpl->srukf;
	DS4_MeasurementModel &measurement_model = srukf.measurement_model;

	if (m_filter->bIsValid)
    {
		// Predict state for current time-step using the filters
        srukf.predict(delta_time);

        // Project the current state onto a predicted measurement as a default
        // in case no observation is available
        DS4_MeasurementVector measurement = measurement_model.observation_function(srukf.x, DS4_MeasurementVector::Zero());

        // Accelerometer and gyroscope measurements are always available
        measurement.set_accelerometer(packet.imu_accelerometer.cast<double>());
        measurement.set_gyroscope(packet.imu_gyroscope.cast<double>());

        if (packet.optical_orientation_quality > 0.f || packet.optical_position_quality > 0.f)
        {
			// Adjust the amount we trust the optical measurements based on the quality parameters
            measurement_model.update_measurement_statistics(
				m_constants, 
				packet.optical_position_quality, 
				packet.optical_orientation_quality);

            // If available, use the optical orientation measurement
            if (packet.optical_orientation_quality > 0.f)
            {
                measurement.set_optical_quaternion(packet.optical_orientation.cast<double>());

				// If this is the first time we have seen the orientation, snap the orientation state
				if (!m_filter->bSeenOrientationMeasurement)
				{
					srukf.x.set_quaternion(packet.optical_orientation.cast<double>());
					m_filter->bSeenOrientationMeasurement= true;
				}
            }

            // If available, use the optical position
            if (packet.optical_position_quality > 0.f)
            {
				Eigen::Vector3f optical_position= packet.get_optical_position_in_meters();

                // State internally stores position in meters
                measurement.set_optical_position(optical_position.cast<double>());

				// If this is the first time we have seen the position, snap the position state
				if (!m_filter->bSeenPositionMeasurement)
				{
					srukf.x.set_position(optical_position.cast<double>());
					m_filter->bSeenPositionMeasurement= true;
				}
            }
        }

        // Update UKF
        srukf.update(measurement);
    }
    else
    {
        srukf.x.setZero();

		if (packet.optical_position_quality > 0.f)
		{
			Eigen::Vector3f optical_position= packet.get_optical_position_in_meters();

			srukf.x.set_position(optical_position.cast<double>());
			m_filter->bSeenPositionMeasurement= true;
		}
		else
		{
			srukf.x.set_position(Eigen::Vector3d::Zero());
		}

        if (packet.optical_orientation_quality > 0.f)
        {
            srukf.x.set_quaternion(packet.optical_orientation.cast<double>());
			m_filter->bSeenOrientationMeasurement= true;
        }
        else
        {
            srukf.x.set_quaternion(Eigen::Quaterniond::Identity());
        }

        m_filter->bIsValid= true;
    }

	// Publish the state from the filter
	m_filter->state = srukf.x;
}

//-- PSMovePoseKalmanFilter --
bool KalmanPoseFilterPSMove::init(const PoseFilterConstants &constants)
{
    KalmanPoseFilter::init(constants);

    PSMoveKalmanPoseFilterImpl *filter = new PSMoveKalmanPoseFilterImpl();
    filter->init(constants);
    m_filter = filter;

    return true;
}

void KalmanPoseFilterPSMove::update(const float delta_time, const PoseFilterPacket &packet)
{
	PSMoveKalmanPoseFilterImpl *psmoveFilterImpl = static_cast<PSMoveKalmanPoseFilterImpl *>(m_filter);
	PoseSRUFK<PSMove_MeasurementModel, PSMove_MeasurementVector> &srukf = psmoveFilterImpl->srukf;
	PSMove_MeasurementModel &measurement_model = srukf.measurement_model;

    if (m_filter->bIsValid)
    {
		// Predict state for current time-step using the filters
        srukf.predict(delta_time);

        // Project the current state onto a predicted measurement as a default
        // in case no observation is available
        PSMove_MeasurementVector measurement = measurement_model.observation_function(srukf.x, PSMove_MeasurementVector::Zero());

        // Accelerometer, magnetometer and gyroscope measurements are always available
        measurement.set_accelerometer(packet.imu_accelerometer.cast<double>());
        measurement.set_gyroscope(packet.imu_gyroscope.cast<double>());
        measurement.set_magnetometer(packet.imu_magnetometer.cast<double>());

        // If available, use the optical position
        if (packet.optical_position_quality > 0.f)
        {
			Eigen::Vector3f optical_position= packet.get_optical_position_in_meters();

			// Adjust the amount we trust the optical measurements based on the quality parameters
			measurement_model.update_measurement_statistics(m_constants, packet.optical_position_quality);

			// Assign the latest optical measurement from the packet
            measurement.set_optical_position(optical_position.cast<double>());

			// If this is the first time we have seen the position, snap the position state
			if (!m_filter->bSeenPositionMeasurement)
			{
				srukf.x.set_position(optical_position.cast<double>());
				m_filter->bSeenPositionMeasurement= true;
			}
        }

        // Update UKF
        srukf.update(measurement);
    }
    else
    {
        srukf.x.setZero();
        srukf.x.set_quaternion(Eigen::Quaterniond::Identity());

		// We always "see" the orientation measurements for the PSMove (MARG state)
		m_filter->bSeenOrientationMeasurement= true;

		if (packet.optical_position_quality > 0.f)
		{
			Eigen::Vector3f optical_position= packet.get_optical_position_in_meters();

			srukf.x.set_position(optical_position.cast<double>());
			m_filter->bSeenPositionMeasurement= true;
		}
		else
		{
			srukf.x.set_position(Eigen::Vector3d::Zero());
		}

        m_filter->bIsValid= true;
    }

	// Publish the state from the filter
	m_filter->state = srukf.x;
}

//-- Private functions --
void process_3rd_order_noise(
    const double dT,
    const double var,
    const int state_index,
	Eigen::Matrix<double, STATE_PARAMETER_COUNT, STATE_PARAMETER_COUNT> &Q)
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

void process_2nd_order_noise(
	const double dT, 
	const double var, 
	const int state_index, 
	Eigen::Matrix<double, STATE_PARAMETER_COUNT, STATE_PARAMETER_COUNT> &Q)
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
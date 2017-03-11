//-- includes --
#include "KalmanPoseFilter.h"
#include "MathAlignment.h"
#include <iostream>

// The kalman filter runs way to slow in a fully unoptimized build.
// But if we just unoptimize this file in a release build it seems to run ok.
#if defined(_MSC_VER) && defined(UNOPTIMIZE_KALMAN_FILTERS)
#pragma optimize( "", off )
#endif

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
	ORIENTATION_W,  // quaternion
    ORIENTATION_X, 
    ORIENTATION_Y,
    ORIENTATION_Z,
	ANGULAR_VELOCITY_X, // rad/s
	ANGULAR_VELOCITY_Y,
	ANGULAR_VELOCITY_Z,

    STATE_PARAMETER_COUNT,
};

enum ProcessNoiseEnum
{
	NOISE_POSITION_X, // meters
	NOISE_LINEAR_VELOCITY_X, // meters / s
	NOISE_LINEAR_ACCELERATION_X, // meters /s^2
	NOISE_POSITION_Y,
	NOISE_LINEAR_VELOCITY_Y,
	NOISE_LINEAR_ACCELERATION_Y,
	NOISE_POSITION_Z,
	NOISE_LINEAR_VELOCITY_Z,
	NOISE_LINEAR_ACCELERATION_Z,
	NOISE_ANGLE_AXIS_X,  // axis * radians
	NOISE_ANGULAR_VELOCITY_X, // rad/s
	NOISE_ANGLE_AXIS_Y,
	NOISE_ANGULAR_VELOCITY_Y,
	NOISE_ANGLE_AXIS_Z,
	NOISE_ANGULAR_VELOCITY_Z,

	NOISE_PARAMETER_COUNT,
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
#define k_ukf_alpha 0.6
#define k_ukf_beta 2.0
#define k_ukf_kappa 3 - STATE_PARAMETER_COUNT

//-- private methods ---
void process_3rd_order_noise(
	const double dT, const double var, const int state_index, 
	Eigen::Matrix<double, NOISE_PARAMETER_COUNT, NOISE_PARAMETER_COUNT> &Q);

void process_2nd_order_noise(
	const double dT, const double var, const int state_index, 
	Eigen::Matrix<double, NOISE_PARAMETER_COUNT, NOISE_PARAMETER_COUNT> &Q);

//-- private definitions --
class PoseNoiseVector : public Eigen::Matrix<double, NOISE_PARAMETER_COUNT, 1>
{
public:
	PoseNoiseVector(void) : Eigen::Matrix<double, NOISE_PARAMETER_COUNT, 1>()
	{ }

	template<typename OtherDerived>
	PoseNoiseVector(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Matrix<double, NOISE_PARAMETER_COUNT, 1>(other)
	{ }

	template<typename OtherDerived>
	PoseNoiseVector& operator= (const Eigen::MatrixBase<OtherDerived>& other)
	{
		this->Base::operator=(other);
		return *this;
	}

	// Accessors
	Eigen::Vector3d get_position_noise() const {
		return Eigen::Vector3d((*this)[NOISE_POSITION_X], (*this)[NOISE_POSITION_Y], (*this)[NOISE_POSITION_Z]);
	}
	Eigen::Vector3d get_linear_velocity_noise() const {
		return Eigen::Vector3d((*this)[NOISE_LINEAR_VELOCITY_X], (*this)[NOISE_LINEAR_VELOCITY_Y], (*this)[NOISE_LINEAR_VELOCITY_Z]);
	}
	Eigen::Vector3d get_linear_acceleration_noise() const {
		return Eigen::Vector3d((*this)[NOISE_LINEAR_ACCELERATION_X], (*this)[NOISE_LINEAR_ACCELERATION_Y], (*this)[NOISE_LINEAR_ACCELERATION_Z]);
	}
	template <int RowsAtCompileTime>
	static Eigen::AngleAxisd extract_angle_axis_noise(const Eigen::Matrix<double, RowsAtCompileTime, 1> &M) {
		Eigen::Vector3d axis = Eigen::Vector3d(M[NOISE_ANGLE_AXIS_X], M[NOISE_ANGLE_AXIS_Y], M[NOISE_ANGLE_AXIS_Z]);
		const double angle = eigen_vector3d_normalize_with_default(axis, Eigen::Vector3d::Zero());
		return Eigen::AngleAxisd(angle, axis);
	}
	Eigen::AngleAxisd get_angle_axis_noise() const {
		return extract_angle_axis_noise<NOISE_PARAMETER_COUNT>(*this);
	}
	template <int RowsAtCompileTime>
	static Eigen::Quaterniond extract_quaternion_noise(const Eigen::Matrix<double, RowsAtCompileTime, 1> &M) {
		return Eigen::Quaterniond(extract_angle_axis_noise<RowsAtCompileTime>(M));
	}
	Eigen::Quaterniond get_quaternion_noise() const {
		return extract_quaternion_noise<NOISE_PARAMETER_COUNT>(*this);
	}
	Eigen::Vector3d get_angular_velocity_noise() const {
		return Eigen::Vector3d((*this)[NOISE_ANGULAR_VELOCITY_X], (*this)[NOISE_ANGULAR_VELOCITY_Y], (*this)[NOISE_ANGULAR_VELOCITY_Z]);
	}

	// Mutators
	void set_position_noise(const Eigen::Vector3d &p) {
		(*this)[NOISE_POSITION_X] = p.x(); (*this)[NOISE_POSITION_Y] = p.y(); (*this)[NOISE_POSITION_Z] = p.z();
	}
	void set_linear_velocity_noise(const Eigen::Vector3d &v) {
		(*this)[NOISE_LINEAR_VELOCITY_X] = v.x(); (*this)[NOISE_LINEAR_VELOCITY_Y] = v.y(); (*this)[NOISE_LINEAR_VELOCITY_Z] = v.z();
	}
	void set_linear_acceleration_noise(const Eigen::Vector3d &a) {
		(*this)[NOISE_LINEAR_ACCELERATION_X] = a.x(); (*this)[NOISE_LINEAR_ACCELERATION_Y] = a.y(); (*this)[NOISE_LINEAR_ACCELERATION_Z] = a.z();
	}
	template <int RowsAtCompileTime>
	static void apply_angle_axis_noise(const Eigen::AngleAxisd &a, Eigen::Matrix<double, RowsAtCompileTime, 1> &M) {
		const double angle = a.angle();
		M[NOISE_ANGLE_AXIS_X] = a.axis().x() * angle;
		M[NOISE_ANGLE_AXIS_Y] = a.axis().y() * angle;
		M[NOISE_ANGLE_AXIS_Z] = a.axis().z() * angle;
	}
	void set_angle_axis_noise(const Eigen::AngleAxisd &a) {
		apply_angle_axis_noise<NOISE_PARAMETER_COUNT>(a, *this);
	}
	template <int RowsAtCompileTime>
	static void apply_quaternion_noise(const Eigen::Quaterniond &q, Eigen::Matrix<double, RowsAtCompileTime, 1> &M) {
		const Eigen::AngleAxisd angle_axis(q);
		apply_angle_axis_noise<RowsAtCompileTime>(angle_axis, M);
	}
	void set_quaternion_noise(const Eigen::Quaterniond &q) {
		apply_quaternion_noise<NOISE_PARAMETER_COUNT>(q, *this);
	}
	void set_angular_velocity_noise(const Eigen::Vector3d &v) {
		(*this)[NOISE_ANGULAR_VELOCITY_X] = v.x(); (*this)[NOISE_ANGULAR_VELOCITY_Y] = v.y(); (*this)[NOISE_ANGULAR_VELOCITY_Z] = v.z();
	}

	PoseNoiseVector negate() const
	{
		PoseNoiseVector neg_noise = -(*this);

		neg_noise.set_angle_axis_noise(this->get_angle_axis_noise().inverse());

		return neg_noise;
	}
};

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
    Eigen::Vector3d get_position_meters() const { 
        return Eigen::Vector3d((*this)[POSITION_X], (*this)[POSITION_Y], (*this)[POSITION_Z]); 
    }
    Eigen::Vector3d get_linear_velocity_m_per_sec() const {
        return Eigen::Vector3d((*this)[LINEAR_VELOCITY_X], (*this)[LINEAR_VELOCITY_Y], (*this)[LINEAR_VELOCITY_Z]);
    }
    Eigen::Vector3d get_linear_acceleration_m_per_sec_sqr() const {
        return Eigen::Vector3d((*this)[LINEAR_ACCELERATION_X], (*this)[LINEAR_ACCELERATION_Y], (*this)[LINEAR_ACCELERATION_Z]);
    }
	template <int RowsAtCompileTime>
	static Eigen::Quaterniond extract_quaternion(const Eigen::Matrix<double, RowsAtCompileTime, 1> &M) {
		return Eigen::Quaterniond(M[ORIENTATION_W], M[ORIENTATION_X], M[ORIENTATION_Y], M[ORIENTATION_Z]);
	}
    Eigen::Quaterniond get_quaternion() const {
        return extract_quaternion<STATE_PARAMETER_COUNT>(*this);
    }
    Eigen::Vector3d get_angular_velocity_rad_per_sec() const {
        return Eigen::Vector3d((*this)[ANGULAR_VELOCITY_X], (*this)[ANGULAR_VELOCITY_Y], (*this)[ANGULAR_VELOCITY_Z]);
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
	template <int RowsAtCompileTime>
	static void apply_quaternion(const Eigen::Quaterniond &q, Eigen::Matrix<double, RowsAtCompileTime, 1> &M) {
		M[ORIENTATION_W] = q.w(); M[ORIENTATION_X] = q.x(); M[ORIENTATION_Y] = q.y(); M[ORIENTATION_Z] = q.z();
	}
    void set_quaternion(const Eigen::Quaterniond &q) {
		apply_quaternion<STATE_PARAMETER_COUNT>(q, *this);
    }
    void set_angular_velocity_rad_per_sec(const Eigen::Vector3d &v) {
        (*this)[ANGULAR_VELOCITY_X] = v.x(); (*this)[ANGULAR_VELOCITY_Y] = v.y(); (*this)[ANGULAR_VELOCITY_Z] = v.z();
    }


	PoseStateVector operator + (const PoseStateVector &other) const
	{
		PoseStateVector result;

		// Add the first 9 rows (position, velocity, and acceleration) the usual way
		result.head<9>() = this->head<9>() + other.head<9>();
		// Add the last 3 rows (angular velocity) the usual was
		result.tail<3>() = this->tail<3>() + other.head<3>();

		// Extract the orientation quaternion from A (which is stored as an angle axis vector)
		const Eigen::Quaterniond orientation = this->get_quaternion();

		// Extract the delta quaternion from B (which is also stored as an angle axis vector)
		const Eigen::Quaterniond delta = other.get_quaternion();

		// Apply the delta to the orientation
		const Eigen::Quaterniond new_rotation = (orientation*delta).normalized();

		// Save the net rotation rotation back in result
		result.set_quaternion(new_rotation);

		return result;
	}	

	PoseStateVector operator + (const PoseNoiseVector &other) const
	{
		PoseStateVector result;

		// Add the first 9 rows (position, velocity, and acceleration) the usual way
		result.head<9>() = this->head<9>() + other.head<9>();

		// Extract the orientation quaternion from A (which is stored as an angle axis vector)
		const Eigen::Quaterniond orientation = this->get_quaternion();

		// Extract the delta noise quaternion from B (which is also stored as an angle axis vector)
		const Eigen::Quaterniond delta = other.get_quaternion_noise();

		// Apply the noise delta to the orientation
		const Eigen::Quaterniond new_rotation = (orientation*delta).normalized();

		// Save the net rotation rotation back in result
		result.set_quaternion(new_rotation);

		// Add the noise angular velocity to this angular velocity
		result.set_angular_velocity_rad_per_sec(this->get_angular_velocity_rad_per_sec() + other.get_angular_velocity_noise());

		return result;
	}

	PoseStateVector operator - (const PoseStateVector &other) const
	{
		PoseStateVector result;

		// Subtract the first 9 rows (position, velocity, and acceleration) the usual way
		result.head<9>() = this->head<9>() - other.head<9>();
		// Subtract the last 3 rows (angular velocity) the usual was
		result.tail<3>() = this->tail<3>() - other.head<3>();

		// Extract the orientation quaternion from both states (which is stored as an angle axis vector)
		const Eigen::Quaterniond q1= this->get_quaternion();
		const Eigen::Quaterniond q2= other.get_quaternion();

		// Compute the "quaternion difference" i.e. rotation from q1 to q2
		const Eigen::Quaterniond q_diff= (q1*q2.conjugate()).normalized();

		result.set_quaternion(q_diff);

		return result;
	}

	PoseStateVector operator - (const PoseNoiseVector &other) const
	{
		PoseStateVector result;

		// Subtract the first 9 rows (position, velocity, and acceleration) the usual way
		result.head<9>() = this->head<9>() - other.head<9>();

		// Extract the orientation quaternion from both states (which is stored as an angle axis vector)
		const Eigen::Quaterniond q1 = this->get_quaternion();
		const Eigen::Quaterniond q2 = other.get_quaternion_noise();

		// Compute the "quaternion difference" i.e. rotation from q1 to q2
		const Eigen::Quaterniond q_diff = (q1*q2.conjugate()).normalized();

		result.set_quaternion(q_diff);

		// Subtract the noise angular velocity from this angular velocity
		result.set_angular_velocity_rad_per_sec(this->get_angular_velocity_rad_per_sec() - other.get_angular_velocity_noise());


		return result;
	}

	template <int PointCount>
	static void special_state_mean(
		const Eigen::Matrix<double, STATE_PARAMETER_COUNT, PointCount>& state_matrix,
		const Eigen::Matrix<double, PointCount, 1> weight_vector,
		PoseStateVector &result)
	{
		Eigen::Quaterniond quat_0_inv = extract_quaternion<RowsAtCompileTime>(state_matrix.col(0)).conjugate();

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

PoseNoiseVector convert_state_to_noise_vector(const PoseStateVector &state_vector)
{
	PoseNoiseVector result;

	// Copy the linear portions straight over (position, velocity, acceleration)
	result.head<9>() = state_vector.head<9>();

	// Convert the quaternion in the state vector to an angle-axis vector
	result.set_angle_axis_noise(Eigen::AngleAxisd(state_vector.get_quaternion()));

	// Copy over the angular velocity vector
	result.set_angular_velocity_noise(state_vector.get_angular_velocity_rad_per_sec());

	return result;
}

PoseStateVector convert_noise_to_state_vector(const PoseNoiseVector &noise_vector)
{
	PoseStateVector result;

	// Copy the linear portions straight over (position, velocity, acceleration)
	result.head<9>() = noise_vector.head<9>();

	// Copy the angle-axis vector
	result.set_quaternion(Eigen::Quaterniond(noise_vector.get_angle_axis_noise()));

	// Copy over the angular velocity vector
	result.set_angular_velocity_rad_per_sec(noise_vector.get_angular_velocity_noise());

	return result;
}

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

	PSMove_MeasurementVector negate() const
	{
		// for the PSMove measurement the negation can be computed 
		// with simple vector negation
		return (*this) * -1.0;
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

	DS4_MeasurementVector operator - (const DS4_MeasurementVector &other) const
	{
		DS4_MeasurementVector measurement_diff;

		measurement_diff.head<9>() = this->head<9>() - other.head<9>();
		
		const Eigen::Quaterniond q1= this->get_optical_quaternion();
		const Eigen::Quaterniond q2= other.get_optical_quaternion();
		const Eigen::Quaterniond q_diff= q2*q1.conjugate();

		// Stomp the incorrect orientation difference computed by the vector subtraction
		measurement_diff.set_optical_quaternion(q_diff);

		return measurement_diff;
	}

	DS4_MeasurementVector negate() const
	{
		// for the DS4 measurement the negation can be computed 
		// with simple vector negation
		// (Safe to negate the optical angle axis)
		return (*this) * -1.0;
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
		for (int col_index = 0; col_index < SIGMA_POINT_COUNT; ++col_index)
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
		const float tracking_projection_area_px_sqr)
	{
        // Start off using the maximum standard deviation values
		const double position_variance_cm_sqr = static_cast<double>(constants.position_constants.position_variance_curve.evaluate(tracking_projection_area_px_sqr));
		// variance_meters = variance_cm * (0.01)^2 because ...
		// var(k*x) = sum(k*x_i - k*mu)^2/(N-1) = k^2*sum(x_i - mu)^2/(N-1)
		// where k = k_centimeters_to_meters = 0.01
		const double position_variance_m_sqr = k_centimeters_to_meters*k_centimeters_to_meters*position_variance_cm_sqr;

		// Update the biases
		Eigen::Vector3d acc_drift = constants.position_constants.accelerometer_drift.cast<double>();
		Eigen::Vector3d gyro_drift = constants.orientation_constants.gyro_drift.cast<double>();
		Eigen::Vector3d mag_drift = constants.orientation_constants.magnetometer_drift.cast<double>();
		R_mu(PSMOVE_ACCELEROMETER_X, 1) = acc_drift.x();
		R_mu(PSMOVE_ACCELEROMETER_Y, 1) = acc_drift.y();
		R_mu(PSMOVE_ACCELEROMETER_Z, 1) = acc_drift.z();
		R_mu(PSMOVE_GYROSCOPE_X, 1) = gyro_drift.x();
		R_mu(PSMOVE_GYROSCOPE_Y, 1) = gyro_drift.y();
		R_mu(PSMOVE_GYROSCOPE_Z, 1) = gyro_drift.z();
		R_mu(PSMOVE_MAGNETOMETER_X, 1) = mag_drift.x();
		R_mu(PSMOVE_MAGNETOMETER_Y, 1) = mag_drift.y();
		R_mu(PSMOVE_MAGNETOMETER_Z, 1) = mag_drift.z();
		R_mu(PSMOVE_OPTICAL_POSITION_X, 1) = 0.0;
		R_mu(PSMOVE_OPTICAL_POSITION_Y, 1) = 0.0;
		R_mu(PSMOVE_OPTICAL_POSITION_Z, 1) = 0.0;


        // Update the measurement covariance R
        R_cov = Eigen::Matrix<double, PSMOVE_MEASUREMENT_PARAMETER_COUNT, PSMOVE_MEASUREMENT_PARAMETER_COUNT>::Zero();

		// Only diagonals used so no need to compute Cholesky
		R_cov(PSMOVE_ACCELEROMETER_X, PSMOVE_ACCELEROMETER_X) = sqrt(R_SCALE*constants.position_constants.accelerometer_variance.x());
		R_cov(PSMOVE_ACCELEROMETER_Y, PSMOVE_ACCELEROMETER_Y) = sqrt(R_SCALE*constants.position_constants.accelerometer_variance.y());
		R_cov(PSMOVE_ACCELEROMETER_Z, PSMOVE_ACCELEROMETER_Z) = sqrt(R_SCALE*constants.position_constants.accelerometer_variance.z());
		R_cov(PSMOVE_GYROSCOPE_X, PSMOVE_GYROSCOPE_X)= sqrt(R_SCALE*constants.orientation_constants.gyro_variance.x());
		R_cov(PSMOVE_GYROSCOPE_Y, PSMOVE_GYROSCOPE_Y)= sqrt(R_SCALE*constants.orientation_constants.gyro_variance.y());
		R_cov(PSMOVE_GYROSCOPE_Z, PSMOVE_GYROSCOPE_Z)= sqrt(R_SCALE*constants.orientation_constants.gyro_variance.z());
		R_cov(PSMOVE_MAGNETOMETER_X, PSMOVE_MAGNETOMETER_X) = sqrt(R_SCALE*constants.orientation_constants.magnetometer_variance.x());
		R_cov(PSMOVE_MAGNETOMETER_Y, PSMOVE_MAGNETOMETER_Y) = sqrt(R_SCALE*constants.orientation_constants.magnetometer_variance.y());
		R_cov(PSMOVE_MAGNETOMETER_Z, PSMOVE_MAGNETOMETER_Z) = sqrt(R_SCALE*constants.orientation_constants.magnetometer_variance.z());
		R_cov(PSMOVE_OPTICAL_POSITION_X, PSMOVE_OPTICAL_POSITION_X) = sqrt(R_SCALE*position_variance_m_sqr);
		R_cov(PSMOVE_OPTICAL_POSITION_Y, PSMOVE_OPTICAL_POSITION_Y) = sqrt(R_SCALE*position_variance_m_sqr);
		R_cov(PSMOVE_OPTICAL_POSITION_Z, PSMOVE_OPTICAL_POSITION_Z) = sqrt(R_SCALE*position_variance_m_sqr);
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

		// Extract the observation bias
		const PSMove_MeasurementVector &observation_bias = R_mu;
		const Eigen::Vector3d accel_bias = observation_bias.get_accelerometer();
		const Eigen::Vector3d mag_bias = observation_bias.get_magnetometer();
		const Eigen::Vector3d gyro_bias = observation_bias.get_gyroscope();
		const Eigen::Vector3d position_bias = observation_bias.get_optical_position();

		// Extract the observation noise
		const Eigen::Vector3d accel_noise = observation_noise.get_accelerometer();
		const Eigen::Vector3d mag_noise = observation_noise.get_magnetometer();
		const Eigen::Vector3d gyro_noise = observation_noise.get_gyroscope();
		const Eigen::Vector3d position_noise = observation_noise.get_optical_position();

        // Use the position and orientation from the state for predictions
        const Eigen::Vector3d position= state.get_position_meters();
        const Eigen::Quaterniond orientation= state.get_quaternion();

        // Use the current linear acceleration from the state to predict
        // what the accelerometer reading will be (in world space)
        const Eigen::Vector3d gravity_accel_g_units= identity_gravity_direction;
        const Eigen::Vector3d linear_accel_g_units= state.get_linear_acceleration_m_per_sec_sqr() * k_ms2_to_g_units;
        const Eigen::Vector3d accel_world= linear_accel_g_units + gravity_accel_g_units;

        // Put the accelerometer prediction into the local space of the controller
		const Eigen::Quaterniond accel_world_quat(0.f, accel_world.x(), accel_world.y(), accel_world.z());
		const Eigen::Vector3d accel_local = orientation*(accel_world_quat*orientation.conjugate()).vec();

        // Use the angular velocity from the state to predict what the gyro reading will be
        const Eigen::Vector3d gyro_local= state.get_angular_velocity_rad_per_sec(); 

        // Use the orientation from the state to predict
        // what the magnetometer reading should be
        const Eigen::Vector3d &mag_world= identity_magnetometer_direction;
		const Eigen::Quaterniond mag_world_quat(0.f, mag_world.x(), mag_world.y(), mag_world.z());
        const Eigen::Vector3d mag_local= orientation*(mag_world_quat*orientation.conjugate()).vec();

        // Save the predictions into the measurement vector
        predicted_measurement.set_accelerometer(accel_local + accel_bias + accel_noise);
        predicted_measurement.set_magnetometer(mag_local + mag_bias + mag_noise);
        predicted_measurement.set_gyroscope(gyro_local + gyro_bias + gyro_noise);
        predicted_measurement.set_optical_position(position + position_bias + position_noise);

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
		update_measurement_statistics(constants, 0.f);

		identity_gravity_direction= constants.orientation_constants.gravity_calibration_direction.cast<double>();
    }

	void update_measurement_statistics(
		const PoseFilterConstants &constants,
		const float tracking_projection_area_px_sqr)
	{
		const double position_variance_cm_sqr = static_cast<double>(constants.position_constants.position_variance_curve.evaluate(tracking_projection_area_px_sqr));
		// variance_meters = variance_cm * (0.01)^2 because ...
		// var(k*x) = sum(k*x_i - k*mu)^2/(N-1) = k^2*sum(x_i - mu)^2/(N-1)
		// where k = k_centimeters_to_meters = 0.01
		const double position_variance_m_sqr = k_centimeters_to_meters*k_centimeters_to_meters*position_variance_cm_sqr;
		const double orientation_variance =
			constants.orientation_constants.orientation_variance_curve.evaluate(tracking_projection_area_px_sqr);

		const float position_drift = 0.f;

		const double angle_axis_std_dev= sqrt(R_SCALE*orientation_variance);
		const double angle_axis_drift = 0.f;

		// Update the biases
		const Eigen::Vector3d acc_drift = constants.position_constants.accelerometer_drift.cast<double>();
		const Eigen::Vector3d gyro_drift = constants.orientation_constants.gyro_drift.cast<double>();
		R_mu(DS4_ACCELEROMETER_X, 1) = acc_drift.x();
		R_mu(DS4_ACCELEROMETER_Y, 1) = acc_drift.y();
		R_mu(DS4_ACCELEROMETER_Z, 1) = acc_drift.z();
		R_mu(DS4_GYROSCOPE_X, 1) = gyro_drift.x();
		R_mu(DS4_GYROSCOPE_Y, 1) = gyro_drift.y();
		R_mu(DS4_GYROSCOPE_Z, 1) = gyro_drift.z();
		R_mu(DS4_OPTICAL_POSITION_X, 1) = position_drift;
		R_mu(DS4_OPTICAL_POSITION_Y, 1) = position_drift;
		R_mu(DS4_OPTICAL_POSITION_Z, 1) = position_drift;
		R_mu(DS4_OPTICAL_ANGLE_AXIS_X, 1) = angle_axis_drift;
		R_mu(DS4_OPTICAL_ANGLE_AXIS_Y, 1) = angle_axis_drift;
		R_mu(DS4_OPTICAL_ANGLE_AXIS_Z, 1) = angle_axis_drift;

        // Update the measurement covariance R
        R_cov = Eigen::Matrix<double, DS4_MEASUREMENT_PARAMETER_COUNT, DS4_MEASUREMENT_PARAMETER_COUNT>::Zero();
		R_cov(DS4_ACCELEROMETER_X, DS4_ACCELEROMETER_X) = sqrt(R_SCALE*constants.position_constants.accelerometer_variance.x());
		R_cov(DS4_ACCELEROMETER_Y, DS4_ACCELEROMETER_Y) = sqrt(R_SCALE*constants.position_constants.accelerometer_variance.y());
		R_cov(DS4_ACCELEROMETER_Z, DS4_ACCELEROMETER_Z) = sqrt(R_SCALE*constants.position_constants.accelerometer_variance.z());
		R_cov(DS4_GYROSCOPE_X, DS4_GYROSCOPE_X)= sqrt(R_SCALE*constants.orientation_constants.gyro_variance.x());
		R_cov(DS4_GYROSCOPE_Y, DS4_GYROSCOPE_Y)= sqrt(R_SCALE*constants.orientation_constants.gyro_variance.y());
		R_cov(DS4_GYROSCOPE_Z, DS4_GYROSCOPE_Z)= sqrt(R_SCALE*constants.orientation_constants.gyro_variance.z());
		R_cov(DS4_OPTICAL_POSITION_X, DS4_OPTICAL_POSITION_X) = sqrt(R_SCALE*position_variance_m_sqr);
		R_cov(DS4_OPTICAL_POSITION_Y, DS4_OPTICAL_POSITION_Y) = sqrt(R_SCALE*position_variance_m_sqr);
		R_cov(DS4_OPTICAL_POSITION_Z, DS4_OPTICAL_POSITION_Z) = sqrt(R_SCALE*position_variance_m_sqr);
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

		// Extract the observation bias
		const DS4_MeasurementVector &observation_bias = R_mu;
		const Eigen::Vector3d accel_bias = observation_bias.get_accelerometer();
		const Eigen::Vector3d gyro_bias = observation_bias.get_gyroscope();
		const Eigen::Vector3d position_bias = observation_bias.get_optical_position();
		const Eigen::Quaterniond orientation_bias = observation_bias.get_optical_quaternion();

		// Extract the observations noise
		const Eigen::Vector3d accel_noise= observation_noise.get_accelerometer();
		const Eigen::Vector3d gyro_noise= observation_noise.get_gyroscope();
		const Eigen::Vector3d position_noise= observation_noise.get_optical_position();
		const Eigen::Quaterniond orientation_noise= observation_noise.get_optical_quaternion();

        // Use the position and orientation from the state for predictions
        const Eigen::Vector3d position= state.get_position_meters();
        const Eigen::Quaterniond orientation= state.get_quaternion();

		// Accelerometer = (linear acceleration + gravity) transformed to controller frame.
        const Eigen::Vector3d gravity_accel_g_units= -identity_gravity_direction;
        const Eigen::Vector3d linear_accel_g_units= state.get_linear_acceleration_m_per_sec_sqr() * k_ms2_to_g_units;
        const Eigen::Vector3d accel_world= linear_accel_g_units + gravity_accel_g_units;
        const Eigen::Quaterniond accel_world_quat(0.f, accel_world.x(), accel_world.y(), accel_world.z());

        // Put the accelerometer prediction into the local space of the controller
        const Eigen::Vector3d accel_local= orientation*(accel_world_quat*orientation.conjugate()).vec();

        // Gyroscope = angular velocity (both rad/sec)
        const Eigen::Vector3d gyro_local= state.get_angular_velocity_rad_per_sec(); 

        // Save the predictions into the measurement vector
        predicted_measurement.set_accelerometer(accel_local + accel_bias + accel_noise);
        predicted_measurement.set_gyroscope(gyro_local + gyro_noise + gyro_noise);
        predicted_measurement.set_optical_position(position + position_bias + position_noise);
        predicted_measurement.set_optical_quaternion(
			(orientation_bias*orientation_noise*orientation).normalized());

        return predicted_measurement;
    }

public:
    Eigen::Vector3d identity_gravity_direction;

	//! Measurement noise mean
	Eigen::Matrix<double, DS4_MEASUREMENT_PARAMETER_COUNT, 1> R_mu;

	//! Measurement noise covariance
	Eigen::Matrix<double, DS4_MEASUREMENT_PARAMETER_COUNT, DS4_MEASUREMENT_PARAMETER_COUNT> R_cov;
};

template <int S_DIM, int Q_DIM, int R_DIM>
class SigmaPointWeights
{
public:
	static const int L_DIM = 1 + 2*S_DIM + 2*Q_DIM + 2*R_DIM;

	/// Scaling factor for the sigma points
	double zeta;

	/// Sigma weights (m)
	Eigen::Matrix<double, L_DIM, 1> wm;

	/// Sigma weights (c)
	Eigen::Matrix<double, L_DIM, 1> wc;

	double w_qr;
	double w_cholup;

	SigmaPointWeights()
	{
		zeta = 0.0;
		wm = Eigen::Matrix<double, L_DIM, 1>::Zero();
		wc = Eigen::Matrix<double, L_DIM, 1>::Zero();
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
		// TODO: this isn't the state size
		const double L = static_cast<double>(S_DIM + Q_DIM + R_DIM);

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
		for (int point_index = 1; point_index < L_DIM; ++point_index)
		{
			wm[point_index] = wm_rest;
		}

		// Fill in the covariance-weights
		double wc_0 = wm_0 + (1.0 - alpha*alpha + beta);
		double wc_rest = wm_rest;

		// wc = weights for calculating covariance(proc., obs., proc - obs)
		wc[0] = wc_0;
		for (int point_index = 1; point_index < L_DIM; ++point_index)
		{
			wc[point_index] = wc_rest;
		}

		// For SRUKF, we also need sqrt of wc_rest for chol update.
		w_qr = sqrt(wc[1]);
		w_cholup = sqrt(fabs(wc(0)));
	}
};

// Specialized Square Root Unscented Kalman Filter (SR-UKF)
template<class MeasurementModelType, class Measurement>
class PoseSRUFK
{
public:
	// State vector: posx, velx, accx, posy, vely, accy, posz, velz, accz, qw, qx, qy, qz, avelx, avely, avelz
	// Units: pos: m, vel: m/s, acc: m/s^2, orient. in quat, avel: rad/s
	static const int X_DIM = STATE_PARAMETER_COUNT;
	// [a.x, a.y, a.z, g.x, g.y, g.z, m.x, m.y, m.z, pos.x, pos.y, pos.z]
	static const int O_DIM = Measurement::RowsAtCompileTime;
	// State covariance.Rotational covariance represented with angle - axis instead of quaternion.See Qdim below.
	static const int S_DIM = NOISE_PARAMETER_COUNT;
	// Process noise vector : posx, velx, accx, posy, vely, accy, posz, velz, accz, angx, avelx, angy, avely, angz, avelz
	// Units: Same as state vector.
	static const int Q_DIM = NOISE_PARAMETER_COUNT;
	// Assume observation noise is same dimensionality as measurement vector.
	static const int R_DIM = Measurement::RowsAtCompileTime;
	static const int SIGMA_POINT_COUNT = 2 * S_DIM + 1;
	static const int L_DIM = SIGMA_POINT_COUNT + 2*Q_DIM + 2*R_DIM;

	//! Type of the state vector
	typedef PoseStateVector State;
        
	//! Estimated state
	State x;

	//! Lower-triangular Cholesky factor of state covariance
	Eigen::Matrix<double, S_DIM, S_DIM> S;

	//! Process noise mean
	PoseNoiseVector Q_mu;

	//! The "square root" of the process noise covariance a.k.a. the lower part of the Choleskly
	Eigen::Matrix<double, Q_DIM, Q_DIM> Q_cov;

	MeasurementModelType measurement_model;

	SigmaPointWeights<S_DIM, Q_DIM, R_DIM> W;

	// Sigma points at time t = k - 1
	Eigen::Matrix<double, X_DIM, SIGMA_POINT_COUNT> X_t;

	// Augmented Sigma points propagated through process function to time k
	Eigen::Matrix<double, X_DIM, L_DIM> X_k;

	// State estimate = weighted sum of sigma points
	PoseStateVector x_k;

	// Propagated sigma point residuals = (sp - x_k)
	Eigen::Matrix<double, S_DIM, L_DIM> X_k_r;

	// Upper - triangular of propagated sp covariance
	Eigen::Matrix<double, S_DIM, S_DIM > Sx_k;
                    
public:
	PoseSRUFK()
	{
		// Setup state and covariance
		x.setZero();
		S.setIdentity();
		Q_mu.setZero();
		Q_cov.setIdentity();
	}

	void init(
		const PoseFilterConstants &constants, 
		const Eigen::Vector3f &position, 
		const Eigen::Quaternionf &orientation)
	{
		const double mean_position_dT = constants.position_constants.mean_update_time_delta;
		//const double mean_orientation_dT = constants.position_constants.mean_update_time_delta;

		// Start off using the maximum variance values
		//const Eigen::Vector3f position_variance =
			//(constants.position_constants.min_position_variance +
			//	constants.position_constants.max_position_variance) * 0.5f * Q_SCALE;
		//const double angle_axis_variance =
			//(constants.orientation_constants.min_orientation_variance +
			//	constants.orientation_constants.max_orientation_variance) * 0.5f* Q_SCALE;

		// TODO: Initial guess at state covariance square root from filter constants?
		S = Eigen::Matrix<double, S_DIM, S_DIM>::Identity() * 0.01;

		// Process noise should be mean-zero, I think.
		Q_mu = Eigen::Matrix<double, Q_DIM, 1>::Zero();

		// Initialize the process covariance matrix Q
		Eigen::Matrix<double, Q_DIM, Q_DIM> Q_cov_init=
			Eigen::Matrix<double, Q_DIM, Q_DIM>::Zero();
		process_3rd_order_noise(mean_position_dT, Q_SCALE, POSITION_X, Q_cov_init);
		process_3rd_order_noise(mean_position_dT, Q_SCALE, POSITION_Y, Q_cov_init);
		process_3rd_order_noise(mean_position_dT, Q_SCALE, POSITION_Z, Q_cov_init);
		//process_2nd_order_noise(mean_orientation_dT, Q_SCALE, ANGLE_AXIS_X, Q_cov_init);
		//process_2nd_order_noise(mean_orientation_dT, Q_SCALE, ANGLE_AXIS_Y, Q_cov_init);
		//process_2nd_order_noise(mean_orientation_dT, Q_SCALE, ANGLE_AXIS_Z, Q_cov_init);
		//HACK: Overwrite orientation/angvel process noise.
//		Q_cov_init.block<6, 6>(9, 9) = Eigen::Matrix<double, 6, 6>::Identity()*0.1;

		// Compute the std-deviation Q matrix a.k.a. the sqrt of Q_cov_init a.k.a the Cholesky
		Q_cov= Q_cov_init.llt().matrixL();

		// Initialize the measurement noise
		measurement_model.init(constants);

		// Set the initial state
		x.setZero();
		x.set_position_meters(position.cast<double>());
		x.set_quaternion(orientation.cast<double>());

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
		const PoseNoiseVector& zQ_cov,
		const float deltaTime) const
	{
		//! Predicted state vector after transition
		PoseStateVector new_state;

		// Extract parameters from the old state
		const Eigen::Vector3d old_position = old_state.get_position_meters();
		const Eigen::Vector3d old_linear_velocity = old_state.get_linear_velocity_m_per_sec();
		const Eigen::Vector3d old_linear_acceleration = old_state.get_linear_acceleration_m_per_sec_sqr();
		const Eigen::Quaterniond old_orientation = old_state.get_quaternion();
		const Eigen::Vector3d old_angular_velocity = old_state.get_angular_velocity_rad_per_sec();

		// Extract parameters from process noise mean
		const Eigen::Vector3d position_bias = Q_mu.get_position_noise();
		const Eigen::Vector3d linear_velocity_bias = Q_mu.get_linear_velocity_noise();
		const Eigen::Vector3d linear_acceleration_bias = Q_mu.get_linear_acceleration_noise();
		const Eigen::Quaterniond orientation_bias = Q_mu.get_quaternion_noise();
		const Eigen::Vector3d angular_velocity_bias = Q_mu.get_angular_velocity_noise();

		// Extract parameters from process noise variance
		const Eigen::Vector3d position_noise = zQ_cov.get_position_noise();
		const Eigen::Vector3d linear_velocity_noise = zQ_cov.get_linear_velocity_noise();
		const Eigen::Vector3d linear_acceleration_noise = zQ_cov.get_linear_acceleration_noise();
		const Eigen::Quaterniond orientation_noise = zQ_cov.get_quaternion_noise();
		const Eigen::Vector3d angular_velocity_noise = zQ_cov.get_angular_velocity_noise();

		// Compute the position state update
		const Eigen::Vector3d new_position =
			old_position
			+ old_linear_velocity*deltaTime
			+ old_linear_acceleration*deltaTime*deltaTime*0.5f
			+ position_bias
			+ position_noise;
		const Eigen::Vector3d new_linear_velocity = 
			old_linear_velocity 
			+ old_linear_acceleration*deltaTime
			+ linear_velocity_bias
			+ linear_velocity_noise;
		const Eigen::Vector3d new_linear_acceleration = 
			old_linear_acceleration
			+ linear_acceleration_bias
			+ linear_acceleration_noise;
		const Eigen::Vector3d new_angular_velocity =
			old_angular_velocity
			+ angular_velocity_bias
			+ angular_velocity_noise;

		// Compute the orientation update
		// From Kraft or Enayati:
		const Eigen::Quaterniond q_delta = eigen_angle_axis_to_quaterniond(old_angular_velocity * deltaTime);
		const Eigen::Quaterniond new_orientation = 
			(old_orientation 
			* q_delta
			* orientation_bias
			* orientation_noise).normalized();

		// Save results to the new state
		new_state.set_position_meters(new_position);
		new_state.set_linear_velocity_m_per_sec(new_linear_velocity);
		new_state.set_linear_acceleration_m_per_sec_sqr(new_linear_acceleration);
		new_state.set_quaternion(new_orientation);
		new_state.set_angular_velocity_rad_per_sec(new_angular_velocity);

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

		// 2. Calculate sigma points
		// In the paper, the authors create an augmented state variable :
		//  [x; Q.mu; R.mu], with length L = Xdim + Q.dim + R.dim
		// Then, when constructing the sigma points, this state vector is repeated
		// 1 + 2 * L times(spmat size is L x 1 + 2 * L).
		// Added to this sigmapoint matrix 2 : end columns is the zeta - scaled
		// augmented covariance matrix :
		//  [S       zeros   zeros;
		//   zeros   Q.cov   zeros;
		//   zeros   zeros   R.cov]
		// First negative, then positive.So, the resulting sigma point matrix looks
		// like...
		//  [x	 x + zS	 x       x      x - zS	x       x;
		//   q   q       q + zQ	q       q       q - zQ	q;
		//   r   r       r       r + zR	r       r       r - zR]
		//
		// But only the upper Xdim rows are sent to the process function, and the
		// next Q.dim rows are sent separately as noise to be added to the sigma
		// points.Using() to indicate propagation, our propagated state vectors +
		// Q noise looks like :
		//  [(x)+q(x + zS) + q(x) + q + zQ(x) + q(x - zS) + q(x) + q - zQ(x) + q]
		// Notice that we are only propagating x, x + zS, and x - zS.Therefore, those
		// are really the only sigma points we need to create right now.

		// The 1st sigma - point is just the state vector.
		// Each remaining sigma - point is the state + / -the scaled sqrt covariance.
/*
		X_t.leftCols<1>() = x;
		for (int col_offset = 0; col_offset < S_DIM; ++col_offset)
		{
			// zS is scaled sqrt cov.
			const PoseNoiseVector zS = S.block<S_DIM, 1>(0, col_offset) * W.zeta;

			X_t.block<X_DIM, 1>(0, 1 + col_offset) = x + zS;
			X_t.block<X_DIM, 1>(0, 1 + S_DIM + col_offset) = x - zS;
		}
*/
		// We now have our minimal sigma points : [x x + zS x - zS]
		// Note : We could add Q(not sqrt) to P(= SS^T) before calculating S, and
		// before calculating the sigma points.This would eliminate the need to add
		// Q in the process function.

		// 3. Propagate sigma points through process function
		// Note that X_k is larger than X_t because the process noise added more state vectors.
		// [p(x_t) + Q_mu | p(x_t + zS) + Q_mu | p(x_t - zS) + Q_mu | p(x_t) + Q_mu + z*Q.cov | p(x_t) + Q_mu - z*Q_cov]
		for (int point_index = 0; point_index < nsp; ++point_index)
		{
			X_k.col(point_index) = 
				process_function(
					X_t.col(point_index), 
					PoseNoiseVector::Zero(),
					deltaTime);
		}
		for (int Q_col = 0; Q_col < Q_DIM; ++Q_col)
		{
			const PoseNoiseVector zQ = Q_cov.col(Q_col) * W.zeta;

			X_k.col(nsp + Q_col) =
				process_function(
					x,
					zQ,
					deltaTime);
			X_k.col(nsp + Q_DIM + Q_col) =
				process_function(
					x,
					zQ.negate(),
					deltaTime);
		}

		// Extend X_k with 2 * filt_struct.R.dim repeats of the first column
		// This emulates the rest of the augmented matrix. It's necessary to extend
		// it here because the weights only work with the correct number of columns.
		PoseStateVector X_k_0 = X_k.col(0);
//		X_k.rightCols<2 * R_DIM>() = X_k_0.replicate<1, 2*R_DIM>();

		// 4. Estimate mean state from weighted sum of propagated sigma points
		x_k = X_k * W.wm;
		PoseStateVector::special_state_mean<L_DIM>(X_k, W.wm, x_k);

		// 5. Get residuals in S - format
		for (int col_offset = 0; col_offset < L_DIM; ++col_offset)
		{
			// Subtract the states (with quaternion orientation) 
			// and then convert to a noise vector (with an angle axis orientation)
			X_k_r.col(col_offset) = convert_state_to_noise_vector(X_k.col(col_offset) - x_k);
		}

		// 6. Estimate state covariance(sqrt)
		// w_qr is scalar
		// QR update of state Cholesky factor.
		// w_qr and w_cholup cannot be negative
//		Eigen::Matrix<double, L_DIM - 1, S_DIM > qr_input = (W.w_qr*X_k_r.rightCols<L_DIM - 1>()).transpose();

		// TODO: Use ColPivHouseholderQR
//		Eigen::HouseholderQR<decltype(qr_input)> qr(qr_input);

		// Set R matrix as upper triangular square root
		// NOTE: R matrix is stored in upper triangular half
		// See: http://math.stackexchange.com/questions/1396308/qr-decomposition-results-in-eigen-library-differs-from-matlab
//		Sx_k = qr.matrixQR().topLeftCorner<S_DIM, S_DIM>().triangularView<Eigen::Upper>();

		// Perform additional rank 1 update
		float wc0_sign = static_cast<float>(sgn(W.wc(0)));
//		Sx_k.selfadjointView<Eigen::Upper>().rankUpdate(X_k_r.leftCols<1>(), W.w_cholup*wc0_sign);
	}

	/**
	* @brief Perform filter update step using measurement \f$z\f$ and corresponding measurement model
	*
	* @param [in] observation The measurement vector
	*/
	void update(const Measurement& observation)
	{
/*
		// 1. Propagate sigma points through observation function.
		const int nsp = SIGMA_POINT_COUNT;
		const int R_inds = (nsp - 2 * R_DIM);
		Eigen::Matrix<double, O_DIM, L_DIM> Y_k;

		// Pass the first 5 blocks of the sigma points through the observation function
		// with zero measurement covariance applied
		// obs([p(x_t) + Q_mu | p(x_t + zS) + Q_mu | p(x_t - zS) + Q_mu | p(x_t) + Q_mu + z*Q.cov | p(x_t) + Q_mu - z*Q_cov], 0)
		for (int point_index = 0; point_index < R_inds; ++point_index)
		{
			Y_k.col(point_index) =
				measurement_model.observation_function(
					X_k.col(point_index),
					Measurement::Zero()); // zero measurement covariance
		}
		// Pass the last two blocks of the sigma points through the observation function
		// with the scaled measurement covariance applied
		// obs([p(x_t) + Q_mu], +zR)
		// obs([p(x_t) + Q_mu], -zR)
		for (int R_col = 0; R_col < R_DIM; ++R_col)
		{
			const Measurement zR = measurement_model.R_cov.col(R_col) * W.zeta;

			Y_k.col(R_inds + R_col) =
				measurement_model.observation_function(
					X_k.col(R_inds + R_col),
					zR);
			Y_k.col(R_inds + R_DIM + R_col) =
				measurement_model.observation_function(
					X_k.col(R_inds + R_DIM + R_col),
					zR.negate());
		}

		// 2. Calculate observation mean.
		Measurement y_k = Measurement::computeWeightedMeasurementAverage<L_DIM>(Y_k, W.wm);

		// 3. Calculate y - residuals.
		// Used in observation covariance and state - observation cross - covariance for Kalman gain.

		Eigen::Matrix<double, O_DIM, L_DIM>  Y_k_r;
		for (int col_offset = 0; col_offset < L_DIM; ++col_offset)
		{
			Y_k_r.col(col_offset)= Measurement(Y_k.col(col_offset)) - y_k;
		}
        
		// 4. Calculate observation sqrt covariance
		// w_qr is scalar
		// QR update of state Cholesky factor.
		// w_qr and w_cholup cannot be negative
		Eigen::Matrix<double, L_DIM - 1, O_DIM> qr_input = (W.w_qr*Y_k_r.rightCols<L_DIM - 1>()).transpose();

		// TODO: Use ColPivHouseholderQR
		Eigen::HouseholderQR<decltype(qr_input)> qr(qr_input);

		// Set R matrix as upper triangular square root
		// NOTE: R matrix is stored in upper triangular half
		// See: http://math.stackexchange.com/questions/1396308/qr-decomposition-results-in-eigen-library-differs-from-matlab
		Eigen::Matrix<double, O_DIM, O_DIM > Sy_k = qr.matrixQR().topLeftCorner<O_DIM, O_DIM>().triangularView<Eigen::Upper>();

		// Perform additional rank 1 update
		float wc0_sign = (W.wc(0) > 0.f) ? 1.f : -1.f;
		Sy_k.selfadjointView<Eigen::Upper>().rankUpdate(Y_k_r.leftCols<1>(), W.w_cholup*wc0_sign);

		// 5. Calculate Kalman Gain
		//First calculate state - observation cross(sqrt) covariance
		const Eigen::Matrix<double, 1, L_DIM> wc = W.wc.transpose();
		const Eigen::Matrix<double, S_DIM, L_DIM> wc_repl = wc.replicate<S_DIM, 1>();
		const Eigen::Matrix<double, S_DIM, O_DIM> Pxy=
			X_k_r.cwiseProduct(wc_repl).eval() * Y_k_r.transpose();

		// In the Matlab code: KG = (Pxy / Sy_k')/Sy_k
		// where "/" is the "mrdivide" operator, 
		// x = B/A solves the system of linear equations A*x = B for x. 
		// I arrived at the following through trial and error
		Eigen::Matrix<double, O_DIM, S_DIM> numerator = Sy_k.transpose().colPivHouseholderQr().solve(Pxy.transpose());
		Eigen::Matrix<double, S_DIM, O_DIM> KG = Sy_k.colPivHouseholderQr().solve(numerator).transpose();

		//Eigen::Matrix<double, X_DIM, O_DIM> KG = Pxy * Sy_k.inverse();

		// 6. Calculate innovation
		Measurement innov = observation - y_k;

		// 7. State update / correct
		// ReBeL srukf doesn't do anything special for angles to get upd.
		PoseStateVector upd= convert_noise_to_state_vector(KG*innov);
		x = x_k + upd;

		// 8. Covariance update / correct
		// This is equivalent to : Px = Px_ - KG*Py*KG';
		Eigen::Matrix<double, S_DIM, O_DIM> cov_update_vectors = KG * Sy_k;
		for (int j = 0; j < O_DIM; ++j)
		{
			// Still UPPER
			Sx_k.selfadjointView<Eigen::Upper>().rankUpdate(cov_update_vectors.col(j), -1.0);
		}

		S = Sx_k.transpose(); // LOWER sqrt-covariance saved for next predict.
*/
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

	virtual void init(
		const PoseFilterConstants &constants)
	{
		bIsValid = false;
		bSeenPositionMeasurement = true;
		bSeenOrientationMeasurement = true;

		reset_orientation = Eigen::Quaternionf::Identity();
		origin_position = Eigen::Vector3f::Zero();
		state = PoseStateVector::Zero();
	}

	virtual void init(
		const PoseFilterConstants &constants,
		const Eigen::Vector3f &position,
		const Eigen::Quaternionf &orientation)
    {
        bIsValid = true;
		bSeenPositionMeasurement= false;
		bSeenOrientationMeasurement= false;

        reset_orientation = Eigen::Quaternionf::Identity();
        origin_position = Eigen::Vector3f::Zero();
		state = PoseStateVector::Zero();
		state.set_position_meters(position.cast<double>());
		state.set_quaternion(orientation.cast<double>());
    }
};

class DS4KalmanPoseFilterImpl : public KalmanPoseFilterImpl
{
public:
	PoseSRUFK<DS4_MeasurementModel, DS4_MeasurementVector> srukf;

	void init(
		const PoseFilterConstants &constants) override
	{
		KalmanPoseFilterImpl::init(constants);
		srukf.init(constants, Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity());
	}

	void init(
		const PoseFilterConstants &constants,
		const Eigen::Vector3f &position,
		const Eigen::Quaternionf &orientation) override
	{
		KalmanPoseFilterImpl::init(constants, position, orientation);
		srukf.init(constants, position, orientation);
	}
};

class PSMoveKalmanPoseFilterImpl : public KalmanPoseFilterImpl
{
public:
	PoseSRUFK<PSMove_MeasurementModel, PSMove_MeasurementVector> srukf;

	void init(
		const PoseFilterConstants &constants) override
	{
		KalmanPoseFilterImpl::init(constants);
		srukf.init(constants, Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity());
	}

	void init(
		const PoseFilterConstants &constants, 
		const Eigen::Vector3f &position,
		const Eigen::Quaternionf &orientation) override
	{
		KalmanPoseFilterImpl::init(constants, position, orientation);
		srukf.init(constants, position, orientation);
	}
};

//-- public interface --
//-- KalmanFilterOpticalPoseARG --
KalmanPoseFilter::KalmanPoseFilter() 
    : m_filter(nullptr)
{
	m_constants.clear();
}

bool KalmanPoseFilter::init(
	const PoseFilterConstants &constants)
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

bool KalmanPoseFilter::init(
	const PoseFilterConstants &constants,
	const Eigen::Vector3f &position, 
	const Eigen::Quaternionf &orientation)
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

void KalmanPoseFilter::recenterOrientation(const Eigen::Quaternionf& q_pose)
{
    Eigen::Quaternionf q_inverse = getOrientation().conjugate();

    eigen_quaternion_normalize_with_default(q_inverse, Eigen::Quaternionf::Identity());
    m_filter->reset_orientation = q_pose*q_inverse;
}

bool KalmanPoseFilter::getIsPositionStateValid() const
{
    return m_filter->bIsValid;
}

bool KalmanPoseFilter::getIsOrientationStateValid() const
{
    return m_filter->bIsValid;
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
                eigen_angular_velocity_to_quaternion_derivative(result, getAngularVelocityRadPerSec());

            predicted_orientation = Eigen::Quaternionf(
                state_orientation.coeffs()
                + quaternion_derivative.coeffs()*time).normalized();
        }

        result = m_filter->reset_orientation * predicted_orientation;
    }

    return result;
}

Eigen::Vector3f KalmanPoseFilter::getAngularVelocityRadPerSec() const
{
	Eigen::Vector3d ang_vel= m_filter->state.get_angular_velocity_rad_per_sec();
	
    return ang_vel.cast<float>();
}

Eigen::Vector3f KalmanPoseFilter::getAngularAccelerationRadPerSecSqr() const
{
    return Eigen::Vector3f::Zero();
}

Eigen::Vector3f KalmanPoseFilter::getPositionCm(float time) const
{
    Eigen::Vector3f result = Eigen::Vector3f::Zero();

    if (m_filter->bIsValid)
    {
        Eigen::Vector3f state_position_meters= m_filter->state.get_position_meters().cast<float>();
		Eigen::Vector3f state_vel_m_per_sec = m_filter->state.get_linear_velocity_m_per_sec().cast<float>();
        Eigen::Vector3f predicted_position_meters =
            is_nearly_zero(time)
            ? state_position_meters
            : state_position_meters + state_vel_m_per_sec * time;

        result = (predicted_position_meters - m_filter->origin_position) * k_meters_to_centimeters;
    }

    return result;
}

Eigen::Vector3f KalmanPoseFilter::getVelocityCmPerSec() const
{
	Eigen::Vector3d vel= m_filter->state.get_linear_velocity_m_per_sec() * k_meters_to_centimeters;

    return vel.cast<float>();
}

Eigen::Vector3f KalmanPoseFilter::getAccelerationCmPerSecSqr() const
{
    Eigen::Vector3d accel= m_filter->state.get_linear_acceleration_m_per_sec_sqr() * k_meters_to_centimeters;

	return accel.cast<float>();
}

//-- KalmanPoseFilterDS4 --
bool KalmanPoseFilterDS4::init(
	const PoseFilterConstants &constants)
{
	KalmanPoseFilter::init(constants);

	DS4KalmanPoseFilterImpl *filter = new DS4KalmanPoseFilterImpl();
	filter->init(constants);
	m_filter = filter;

	return true;
}

bool KalmanPoseFilterDS4::init(
	const PoseFilterConstants &constants,
	const Eigen::Vector3f &position, 
	const Eigen::Quaternionf &orientation)
{
    KalmanPoseFilter::init(constants, position, orientation);

    DS4KalmanPoseFilterImpl *filter = new DS4KalmanPoseFilterImpl();
    filter->init(constants, position, orientation);
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
        measurement.set_accelerometer(packet.imu_accelerometer_g_units.cast<double>());
        measurement.set_gyroscope(packet.imu_gyroscope_rad_per_sec.cast<double>());

		// Adjust the amount we trust the optical measurements based on the quality parameters
		measurement_model.update_measurement_statistics(
			m_constants,
			packet.tracking_projection_area_px_sqr);

        if (packet.tracking_projection_area_px_sqr > 0.f)
        {
			Eigen::Vector3f optical_position_meters = packet.get_optical_position_in_meters();

            // Use the optical orientation measurement
            measurement.set_optical_quaternion(packet.optical_orientation.cast<double>());

			// If this is the first time we have seen the orientation, snap the orientation state
			if (!m_filter->bSeenOrientationMeasurement)
			{
				srukf.x.set_quaternion(packet.optical_orientation.cast<double>());
				m_filter->bSeenOrientationMeasurement= true;
			}

            // Use the optical position
            // State internally stores position in meters
            measurement.set_optical_position(optical_position_meters.cast<double>());

			// If this is the first time we have seen the position, snap the position state
			if (!m_filter->bSeenPositionMeasurement)
			{
				srukf.x.set_position_meters(optical_position_meters.cast<double>());
				m_filter->bSeenPositionMeasurement= true;
			}
        }

        // Update UKF
        srukf.update(measurement);
    }
    else
    {
        srukf.x.setZero();

		if (packet.tracking_projection_area_px_sqr > 0.f)
		{
			Eigen::Vector3f optical_position_meters= packet.get_optical_position_in_meters();

			srukf.x.set_position_meters(optical_position_meters.cast<double>());
			m_filter->bSeenPositionMeasurement= true;

			srukf.x.set_quaternion(packet.optical_orientation.cast<double>());
			m_filter->bSeenOrientationMeasurement = true;
		}
		else
		{
			srukf.x.set_position_meters(Eigen::Vector3d::Zero());
			srukf.x.set_quaternion(Eigen::Quaterniond::Identity());
		}

        m_filter->bIsValid= true;
    }

	// Publish the state from the filter
	m_filter->state = srukf.x;
}

//-- PSMovePoseKalmanFilter --
bool KalmanPoseFilterPSMove::init(
	const PoseFilterConstants &constants)
{
	KalmanPoseFilter::init(constants);

	PSMoveKalmanPoseFilterImpl *filter = new PSMoveKalmanPoseFilterImpl();
	filter->init(constants);
	m_filter = filter;

	return true;
}

bool KalmanPoseFilterPSMove::init(
	const PoseFilterConstants &constants,
	const Eigen::Vector3f &position,
	const Eigen::Quaternionf &orientation)
{
    KalmanPoseFilter::init(constants, position, orientation);

    PSMoveKalmanPoseFilterImpl *filter = new PSMoveKalmanPoseFilterImpl();
    filter->init(constants, position, orientation);
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
        measurement.set_accelerometer(packet.imu_accelerometer_g_units.cast<double>());
        measurement.set_gyroscope(packet.imu_gyroscope_rad_per_sec.cast<double>());
        measurement.set_magnetometer(packet.imu_magnetometer_unit.cast<double>());

        // If available, use the optical position
        if (packet.tracking_projection_area_px_sqr > 0.f)
        {
			Eigen::Vector3f optical_position= packet.get_optical_position_in_meters();

			//TODO: Update measurement statistics once we get the filter working
			//// Adjust the amount we trust the optical measurements based on the quality parameters
			//measurement_model.update_measurement_statistics(m_constants, packet.tracking_projection_area);

			// Assign the latest optical measurement from the packet
            measurement.set_optical_position(optical_position.cast<double>());

			// If this is the first time we have seen the position, snap the position state
			//if (!m_filter->bSeenPositionMeasurement)
			//{
			//	srukf.x.set_position(optical_position.cast<double>());
			//	m_filter->bSeenPositionMeasurement= true;
			//}
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

		if (packet.tracking_projection_area_px_sqr > 0.f)
		{
			Eigen::Vector3f optical_position_meters= packet.get_optical_position_in_meters();

			srukf.x.set_position_meters(optical_position_meters.cast<double>());
			m_filter->bSeenPositionMeasurement= true;
		}
		else
		{
			srukf.x.set_position_meters(Eigen::Vector3d::Zero());
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
	Eigen::Matrix<double, NOISE_PARAMETER_COUNT, NOISE_PARAMETER_COUNT> &Q)
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
	Eigen::Matrix<double, NOISE_PARAMETER_COUNT, NOISE_PARAMETER_COUNT> &Q)
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

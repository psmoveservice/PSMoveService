#ifndef MATH_EIGEN_H
#define MATH_EIGEN_H

//-- includes -----
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "MathUtility.h"

//-- constants -----
extern const Eigen::Vector3f *k_eigen_vector3f_zero;
extern const Eigen::Vector3f *k_eigen_vector3f_one;
extern const Eigen::Quaternionf *k_eigen_quaternion_zero;

//-- macros -----
#define assert_eigen_vector3f_is_normalized(v) assert(is_nearly_equal(v.squaredNorm(), 1.f, k_normal_epsilon))
#define assert_eigen_quaternion_is_normalized(q) assert(is_nearly_equal(q.squaredNorm(), 1.f, k_normal_epsilon))
#define assert_eigen_quaternions_are_nearly_equal(q1, q2, eps) assert(is_nearly_equal(q1.dot(q2), 1.f, eps) || is_nearly_equal(q1.dot(Eigen::Quaternionf(q2.coeffs()*-1.f)), 1.f, eps))

#define assert_eigen_vector3d_is_normalized(v) assert(is_double_nearly_equal(v.squaredNorm(), 1.0, k_real64_normal_epsilon))
#define assert_eigen_quaterniond_is_normalized(q) assert(is_double_nearly_equal(q.squaredNorm(), 1.0, k_real64_normal_epsilon))
#define assert_eigen_quaternionds_are_nearly_equal(q1, q2, eps) assert(is_double_nearly_equal(q1.dot(q2), 1.0, eps) || is_double_nearly_equal(q1.dot(Eigen::Quaterniond(q2.coeffs()*-1.0)), 1.0, eps))

namespace Eigen
{
	//Euler Convention http://www.euclideanspace.com/maths/geometry/rotations/euler/index.htm
	// x - forward - bank axis - applied last
	// y - up - heading axis - applied first
	// z - right - attitude axis - applied second
	template <typename T>
    class EulerAngles : public Eigen::Matrix<T, 3, 1>
	{
	public:
		inline EulerAngles() : Eigen::Matrix<T, 3, 1>() {}
		inline EulerAngles(const Eigen::Matrix<T, 3, 1> &e)
		{
			set(e.x(), e.y(), e.z());
		}
		inline EulerAngles(T bank_radians, T heading_radians, T attitude_radians)
		{
			set(bank_radians, heading_radians, attitude_radians);
		}

        inline T get_x_radians() const { return (*this)(0,0);}
		inline T get_y_radians() const { return (*this)(1,0); }
		inline T get_z_radians() const { return (*this)(2,0); }

		inline T get_x_degrees() const { return static_cast<T>((*this)(0,0) * k_real64_radians_to_degreees); }
		inline T get_y_degrees() const { return static_cast<T>((*this)(1,0) * k_real64_radians_to_degreees); }
		inline T get_z_degrees() const { return static_cast<T>((*this)(2,0) * k_real64_radians_to_degreees); }

		inline T get_bank_radians() const { return get_x_radians(); }
		inline T get_heading_radians() const { return get_y_radians(); }
		inline T get_attitude_radians() const { return get_z_radians(); }

		inline T get_bank_degrees() const { return get_x_degrees(); }
		inline T get_heading_degrees() const { return get_y_degrees(); }
		inline T get_attitude_degrees() const { return get_z_degrees(); }

		inline void set(T bank_radians, T heading_radians, T attitude_radians)
		{
			(*this)(0,0) = (T)wrap_ranged((T)bank_radians, -k_real64_pi - k_real64_normal_epsilon, k_real64_pi + k_real64_normal_epsilon); // bank in range [-180,180], applied third
			(*this)(1,0) = (T)wrap_ranged((T)heading_radians, -k_real64_pi - k_real64_normal_epsilon, k_real64_pi + k_real64_normal_epsilon); // heading in range [-180,180], applied first
			(*this)(2,0) = (T)wrap_ranged((T)attitude_radians, -k_real64_half_pi - k_real64_normal_epsilon, k_real64_half_pi + k_real64_normal_epsilon); // attitude in range [-90,90], applied second
		}
	};
	typedef EulerAngles<float> EulerAnglesf;
	typedef EulerAngles<double> EulerAnglesd;
};

//-- interface -----
Eigen::Quaternionf
eigen_quaternion_from_forward_up(
	const Eigen::Vector3f &forward,
	const Eigen::Vector3f &up);

// Creates a quaternion that rotates clockwise about the axis for a positive angle
// when appied with psmove_vector_clockwise_rotate()
Eigen::Quaternionf
eigen_quaternion_angle_axis(float radians, const Eigen::Vector3f &axis);

template <typename T>
Eigen::Quaternion<T>
eigen_quaternion_inverse(const Eigen::Quaternion<T> &q)
{
	assert_eigen_quaterniond_is_normalized(q);
	return q.conjugate();
}

template <typename T>
Eigen::Quaternion<T>
eigen_quaternion_concatenate(const Eigen::Quaternion<T> &first, const Eigen::Quaternion<T> &second)
{
	return first * second;
}

Eigen::Quaternionf
eigen_quaternion_normalized_lerp(const Eigen::Quaternionf &a, const Eigen::Quaternionf &b, const float u);

Eigen::Quaternionf
eigen_quaternion_safe_divide_with_default(const Eigen::Quaternionf &q, const float divisor, const Eigen::Quaternionf &default_result);

Eigen::Quaterniond
eigen_quaterniond_safe_divide_with_default(const Eigen::Quaterniond &q, const double divisor, const Eigen::Quaterniond &default_result);

float
eigen_quaternion_normalize_with_default(Eigen::Quaternionf &inout_v, const Eigen::Quaternionf &default_result);

double
eigen_quaterniond_normalize_with_default(Eigen::Quaterniond &inout_v, const Eigen::Quaterniond &default_result);

bool
eigen_vector3f_is_valid(const Eigen::Vector3f &v);

bool
eigen_quaternion_is_valid(const Eigen::Quaternionf &q);

Eigen::Vector3f
eigen_vector3f_clockwise_rotate(const Eigen::Quaternionf &q, const Eigen::Vector3f &v);

Eigen::Vector3d
eigen_vector3d_clockwise_rotate(const Eigen::Quaterniond &q, const Eigen::Vector3d &v);

Eigen::Matrix3f
eigen_quaternion_to_clockwise_matrix3f(const Eigen::Quaternionf &q);

Eigen::Quaternionf
eigen_matrix3f_to_clockwise_quaternion(const Eigen::Matrix3f &m);

Eigen::Vector3f
eigen_vector3f_divide_by_vector_with_default(
    const Eigen::Vector3f &v, 
    const Eigen::Vector3f &divisor, 
    const Eigen::Vector3f &default_result);

float 
eigen_vector3f_normalize_with_default(Eigen::Vector3f &v, const Eigen::Vector3f &default_value);

double 
eigen_vector3d_normalize_with_default(Eigen::Vector3d &v, const Eigen::Vector3d &default_value);

float
eigen_quaternion_unsigned_angle_between(const Eigen::Quaternionf &a, const Eigen::Quaternionf &b);

Eigen::Quaternionf
eigen_angular_velocity_to_quaternion_derivative(
	const Eigen::Quaternionf &current_orientation,
	const Eigen::Vector3f &ang_vel);

Eigen::Quaterniond
eigen_angular_velocity_to_quaterniond_derivative(
	const Eigen::Quaterniond &current_orientation,
	const Eigen::Vector3d &ang_vel);

Eigen::Vector3f
eigen_quaternion_derivative_to_angular_velocity(
	const Eigen::Quaternionf &current_orientation,
	const Eigen::Quaternionf &quaternion_derivative);

Eigen::Vector3d
eigen_quaterniond_derivative_to_angular_velocity(
	const Eigen::Quaterniond &current_orientation,
	const Eigen::Quaterniond &quaternion_derivative);

Eigen::Quaterniond
eigen_angle_axis_to_quaterniond(const Eigen::Vector3d &angle_axis);

Eigen::Quaternionf
eigen_angle_axis_to_quaternion(const Eigen::Vector3f &angle_axis);

Eigen::Quaterniond
eigen_euler_angles_to_quaterniond(const Eigen::EulerAnglesd &euler_angles);

Eigen::Quaternionf
eigen_euler_angles_to_quaternionf(const Eigen::EulerAnglesf &euler_angles);

Eigen::EulerAnglesd
eigen_quaterniond_to_euler_angles(const Eigen::Quaterniond &q);

Eigen::EulerAnglesf
eigen_quaternionf_to_euler_angles(const Eigen::Quaternionf &q);


#endif // MATH_EIGEN_H

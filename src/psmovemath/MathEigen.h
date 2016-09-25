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

//-- interface -----
Eigen::Quaternionf
eigen_quaternion_yaw_pitch_roll(float yaw_radians, float pitch_radians, float roll_radians);

void
eigen_quaternion_get_yaw_pitch_roll(
    const Eigen::Quaternionf &q, float *out_yaw_radians, float *out_pitch_radians, float *out_roll_radians);

Eigen::Quaternionf
eigen_quaternion_from_forward_up(
	const Eigen::Vector3f &forward,
	const Eigen::Vector3f &up);

// Creates a quaternion that rotates clockwise about the axis for a positive angle
// when appied with psmove_vector_clockwise_rotate()
Eigen::Quaternionf
eigen_quaternion_angle_axis(float radians, const Eigen::Vector3f &axis);

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

Eigen::Quaterniond
eigen_angle_axis_to_quaterniond(const Eigen::Vector3d &angle_axis);

Eigen::Quaternionf
eigen_angle_axis_to_quaternion(const Eigen::Vector3f &angle_axis);

#endif // MATH_EIGEN_H
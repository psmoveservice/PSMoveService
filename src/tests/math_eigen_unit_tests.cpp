//-- includes -----
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "MathEigen.h"
#include "unit_test.h"

static bool
math_eigen_test_rotation_method_consistency(const Eigen::Quaternionf &q, const Eigen::Vector3f &v);

//-- public interface -----
bool run_math_eigen_unit_tests()
{
	UNIT_TEST_MODULE_BEGIN("math_eigen")
		UNIT_TEST_MODULE_CALL_TEST(math_eigen_test_inverse_multiplication);
		UNIT_TEST_MODULE_CALL_TEST(math_eigen_test_euler_angles);
		UNIT_TEST_MODULE_CALL_TEST(math_eigen_test_matrix_conversion);
		UNIT_TEST_MODULE_CALL_TEST(math_eigen_test_rotate_with_angle_axis_quaternion)
		UNIT_TEST_MODULE_CALL_TEST(math_eigen_test_rotate_with_arbitrary_quaternion)
		UNIT_TEST_MODULE_CALL_TEST(math_eigen_test_concatenation);
	UNIT_TEST_MODULE_END()
}

//-- private functions -----
bool math_eigen_test_inverse_multiplication()
{
	UNIT_TEST_BEGIN("inverse multiplication")
		Eigen::Quaternionf q = Eigen::Quaternionf(0.980671287f, 0.177366823f, 0.0705093816f, 0.0430502370f);
		Eigen::Quaternionf q_inverse = eigen_quaternion_inverse(q);
		Eigen::Quaternionf q_identity = q * q_inverse;

		success &= q_identity.isApprox(Eigen::Quaternionf::Identity(), k_normal_epsilon);
		assert(success);
	UNIT_TEST_COMPLETE()
}

bool math_eigen_test_euler_angles()
{
	UNIT_TEST_BEGIN("euler angles")
		Eigen::Quaternionf q_pitch90 = eigen_quaternion_angle_axis(k_real_half_pi, Eigen::Vector3f::UnitX());
		Eigen::Quaternionf q_yaw90 = eigen_quaternion_angle_axis(k_real_half_pi, Eigen::Vector3f::UnitY());
		Eigen::Quaternionf q_roll90 = eigen_quaternion_angle_axis(k_real_half_pi, Eigen::Vector3f::UnitZ());

		Eigen::Quaternionf q_euler_roll90 = eigen_euler_angles_to_quaternionf(Eigen::EulerAnglesf(k_real_half_pi, 0.f, 0.f));
		Eigen::Quaternionf q_euler_yaw90 = eigen_euler_angles_to_quaternionf(Eigen::EulerAnglesf(0.f, k_real_half_pi, 0.f));
		Eigen::Quaternionf q_euler_pitch90 = eigen_euler_angles_to_quaternionf(Eigen::EulerAnglesf(0.f, 0.f, k_real_half_pi));

		success &= q_pitch90.isApprox(q_euler_pitch90, k_normal_epsilon);
		assert(success);
		success &= q_yaw90.isApprox(q_euler_yaw90, k_normal_epsilon);
		assert(success);
		success &= q_roll90.isApprox(q_euler_roll90, k_normal_epsilon);
		assert(success);

		Eigen::EulerAnglesf euler_angles_pitch90= eigen_quaternionf_to_euler_angles(q_euler_pitch90);
		success &= is_nearly_equal(euler_angles_pitch90.get_heading_radians(), 0.f, k_normal_epsilon);
		success &= is_nearly_equal(euler_angles_pitch90.get_attitude_radians(), k_real_half_pi, k_normal_epsilon);
		success &= is_nearly_equal(euler_angles_pitch90.get_bank_radians(), 0.f, k_normal_epsilon);
		assert(success);

		Eigen::EulerAnglesf euler_angles_yaw90 = eigen_quaternionf_to_euler_angles(q_euler_yaw90);
		success &= is_nearly_equal(euler_angles_yaw90.get_heading_radians(), k_real_half_pi, k_normal_epsilon);
		success &= is_nearly_equal(euler_angles_yaw90.get_attitude_radians(), 0.f, k_normal_epsilon);
		success &= is_nearly_equal(euler_angles_yaw90.get_bank_radians(), 0.f, k_normal_epsilon);
		assert(success);

		Eigen::EulerAnglesf euler_angles_roll90 = eigen_quaternionf_to_euler_angles(q_euler_roll90);
		success &= is_nearly_equal(euler_angles_roll90.get_heading_radians(), 0.f, k_normal_epsilon);
		success &= is_nearly_equal(euler_angles_roll90.get_attitude_radians(), 0.f, k_normal_epsilon);
		success &= is_nearly_equal(euler_angles_roll90.get_bank_radians(), k_real_half_pi, k_normal_epsilon);
		assert(success);
	UNIT_TEST_COMPLETE()
}

bool math_eigen_test_matrix_conversion()
{
	UNIT_TEST_BEGIN("matrix conversion")
		Eigen::Quaternionf q = Eigen::Quaternionf(0.980671287f, 0.177366823f, 0.0705093816f, 0.0430502370f);
		assert_eigen_quaternion_is_normalized(q);

		Eigen::Matrix3f m = eigen_quaternion_to_clockwise_matrix3f(q);
		Eigen::Quaternionf q_copy = eigen_matrix3f_to_clockwise_quaternion(m);

		success &= q.isApprox(q_copy, k_normal_epsilon);
		assert(success);
	UNIT_TEST_COMPLETE()
}

bool math_eigen_test_rotate_with_angle_axis_quaternion()
{
	UNIT_TEST_BEGIN("rotate with angle axis quaternion")
		Eigen::Quaternionf q_pitch90 = eigen_quaternion_angle_axis(k_real_half_pi, Eigen::Vector3f::UnitX());
		Eigen::Quaternionf q_yaw90 = eigen_quaternion_angle_axis(k_real_half_pi, Eigen::Vector3f::UnitY());
		Eigen::Quaternionf q_roll90 = eigen_quaternion_angle_axis(k_real_half_pi, Eigen::Vector3f::UnitZ());
		Eigen::Quaternionf q_euler_roll90 = eigen_euler_angles_to_quaternionf(Eigen::EulerAnglesf(k_real_half_pi, 0.f, 0.f));
		Eigen::Quaternionf q_euler_yaw90 = eigen_euler_angles_to_quaternionf(Eigen::EulerAnglesf(0.f, k_real_half_pi, 0.f));
		Eigen::Quaternionf q_euler_pitch90 = eigen_euler_angles_to_quaternionf(Eigen::EulerAnglesf(0.f, 0.f, k_real_half_pi));
		Eigen::Vector3f v_x = Eigen::Vector3f::UnitX();
		Eigen::Vector3f v_y = Eigen::Vector3f::UnitY();
		Eigen::Vector3f v_z = Eigen::Vector3f::UnitZ();

		// Make sure we get the answers we expect with angle axis rotation
		Eigen::Vector3f v_x_rotated = eigen_vector3f_clockwise_rotate(q_yaw90, v_x);
		success &= v_x_rotated.isApprox(Eigen::Vector3f::UnitZ(), k_normal_epsilon);
		assert(success);
		Eigen::Vector3f v_y_rotated = eigen_vector3f_clockwise_rotate(q_roll90, v_y);
		success &= v_y_rotated.isApprox(Eigen::Vector3f::UnitX(), k_normal_epsilon);
		assert(success);
		Eigen::Vector3f v_z_rotated = eigen_vector3f_clockwise_rotate(q_pitch90, v_z);
		success &= v_z_rotated.isApprox(Eigen::Vector3f::UnitY(), k_normal_epsilon);
		assert(success);

		// Make sure we get the answers we expect with euler angle rotation
		v_x_rotated = eigen_vector3f_clockwise_rotate(q_euler_yaw90, v_x);
		success &= v_x_rotated.isApprox(Eigen::Vector3f::UnitZ(), k_normal_epsilon);
		assert(success);
		v_y_rotated = eigen_vector3f_clockwise_rotate(q_euler_roll90, v_y);
		success &= v_y_rotated.isApprox(Eigen::Vector3f::UnitX(), k_normal_epsilon);
		assert(success);
		v_z_rotated = eigen_vector3f_clockwise_rotate(q_euler_pitch90, v_z);
		success &= v_z_rotated.isApprox(Eigen::Vector3f::UnitY(), k_normal_epsilon);
		assert(success);

		// Make sure all rotation methods are consistent with each other
		success &= math_eigen_test_rotation_method_consistency(q_roll90, v_x);
		success &= math_eigen_test_rotation_method_consistency(q_pitch90, v_y);
		success &= math_eigen_test_rotation_method_consistency(q_yaw90, v_z);
	UNIT_TEST_COMPLETE()
}

bool math_eigen_test_rotate_with_arbitrary_quaternion()
{
	UNIT_TEST_BEGIN("rotate with arbitrary quaternion")
		Eigen::Quaternionf q = Eigen::Quaternionf(0.980671287f, 0.177366823f, 0.0705093816f, 0.0430502370f);
		Eigen::Vector3f v = Eigen::Vector3f(0.288588405f, -0.909195602f, 0.300133437f);

		success &= math_eigen_test_rotation_method_consistency(q, v);
	UNIT_TEST_COMPLETE()
}

bool math_eigen_test_concatenation()
{
	UNIT_TEST_BEGIN("quaternion concatenation")
		Eigen::Quaternionf q_euler_pitch90 = eigen_euler_angles_to_quaternionf(Eigen::EulerAnglesf(0.f, 0.f, k_real_half_pi));
		Eigen::Quaternionf q_euler_yaw90 = eigen_euler_angles_to_quaternionf(Eigen::EulerAnglesf(0.f, k_real_half_pi, 0.f));
		Eigen::Vector3f v_x = Eigen::Vector3f::UnitX();

		// Make sure we get the answers we expect with angle axis rotation
		Eigen::Vector3f v_x_y90 = eigen_vector3f_clockwise_rotate(q_euler_yaw90, v_x);
		Eigen::Vector3f v_x_y90p90 = eigen_vector3f_clockwise_rotate(q_euler_pitch90, v_x_y90);

		// Make sure we get the answers we expect with euler angle rotation
		Eigen::Quaternionf q_euler_yaw90pitch90 = eigen_quaternion_concatenate(q_euler_yaw90, q_euler_pitch90);
		Eigen::Vector3f v_x_qy90p90 = eigen_vector3f_clockwise_rotate(q_euler_yaw90pitch90, v_x);

		success &= v_x_y90p90.isApprox(v_x_qy90p90, k_normal_epsilon);
		assert(success);
	UNIT_TEST_COMPLETE()
}

static bool math_eigen_test_rotation_method_consistency(const Eigen::Quaternionf &q, const Eigen::Vector3f &v)
{
	bool success = true;

	assert_eigen_quaternion_is_normalized(q);

	// This is the same as computing the rotation by computing: q^-1*[0,v]*q, but cheaper
	Eigen::Vector3f v_rotated = eigen_vector3f_clockwise_rotate(q, v);

	// Make sure doing the matrix based rotation performs the same result
	{
		Eigen::Matrix3f m = eigen_quaternion_to_clockwise_matrix3f(q);
		Eigen::Vector3f v_test = m * v;

		success &= v_test.isApprox(v_rotated, k_normal_epsilon);
		assert(success);
	}

	// Make sure the Hamilton product style rotation matches
	{
		Eigen::Quaternionf v_as_quaternion = Eigen::Quaternionf(0.f, v.x(), v.y(), v.z());
		Eigen::Quaternionf q_inv = q.conjugate();
		Eigen::Quaternionf qinv_v = q_inv * v_as_quaternion;
		Eigen::Quaternionf qinv_v_q = qinv_v * q;

		success &=
			is_nearly_equal(qinv_v_q.w(), 0.f, k_normal_epsilon) &&
			is_nearly_equal(qinv_v_q.x(), v_rotated.x(), k_normal_epsilon) &&
			is_nearly_equal(qinv_v_q.y(), v_rotated.y(), k_normal_epsilon) &&
			is_nearly_equal(qinv_v_q.z(), v_rotated.z(), k_normal_epsilon);
		assert(success);
	}

	return success;
}
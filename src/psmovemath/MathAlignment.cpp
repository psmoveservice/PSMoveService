//-- includes -----
#include "MathAlignment.h"

//-- public methods -----
Eigen::Quaternionf
psmove_alignment_quaternion_between_vectors(const Eigen::Vector3f &from, const Eigen::Vector3f &to)
{
	assert_vector3f_is_normalized(from);
	assert_vector3f_is_normalized(to);

	return Eigen::Quaternionf::FromTwoVectors(from, to).conjugate();
}

// This formula comes from Sebastian O.H. Madgwick's 2010 paper:
// "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
// https://www.samba.org/tridge/UAV/madgwick_internal_report.pdf
void
psmove_alignment_compute_objective_vector(
	const Eigen::Quaternionf &q, const Eigen::Vector3f &d, const Eigen::Vector3f &s,
	Eigen::Matrix<float, 3, 1> &out_f, float *out_squared_error)
{
	// Computing f(q;d, s) = (q^-1 * d * q) - s
	Eigen::Vector3f d_rotated = eigen_vector3f_clockwise_rotate(q, d);
	Eigen::Vector3f f = d_rotated - s;

	out_f(0, 0) = f.x();
	out_f(1, 0) = f.y();
	out_f(2, 0) = f.z();

	if (out_squared_error)
	{
		*out_squared_error = f.norm();
	}
}

// This formula comes from Sebastian O.H. Madgwick's 2010 paper:
// "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
// https://www.samba.org/tridge/UAV/madgwick_internal_report.pdf
void
psmove_alignment_compute_objective_jacobian(
	const Eigen::Quaternionf &q, const Eigen::Vector3f &d, Eigen::Matrix<float, 4, 3> &J)
{
	/*
	* The Jacobian of a function is a matrix of partial derivatives that relates rates of changes in inputs to outputs
	*
	* In this case the inputs are the components of the quaternion q (qw, qx, qy, and qz)
	* and the outputs are the components of the error vector f(fx, fy, fz)
	* Where f= q^-1*[0 dx dy dz]*q - s, d= initial field vector, s= target field vector
	*
	* Since there are 4 input vars (q1,q2,q3,q4) and 3 output vars (fx, fy, fz)
	* The Jacobian is a 3x4 matrix that looks like this:
	*
	* | df_x/dq_1 df_x/dq_2 df_x/dq_3 df_x/dq_4 |
	* | df_y/dq_1 df_y/dq_2 df_y/dq_3 df_y/dq_4 |
	* | df_z/dq_1 df_z/dq_2 df_z/dq_3 df_z/dq_4 |
	*/

	const float two_dxq1 = 2.f*d.x()*q.w();
	const float two_dxq2 = 2.f*d.x()*q.x();
	const float two_dxq3 = 2.f*d.x()*q.y();
	const float two_dxq4 = 2.f*d.x()*q.z();

	const float two_dyq1 = 2.f*d.y()*q.w();
	const float two_dyq2 = 2.f*d.y()*q.x();
	const float two_dyq3 = 2.f*d.y()*q.y();
	const float two_dyq4 = 2.f*d.y()*q.z();

	const float two_dzq1 = 2.f*d.z()*q.w();
	const float two_dzq2 = 2.f*d.z()*q.x();
	const float two_dzq3 = 2.f*d.z()*q.y();
	const float two_dzq4 = 2.f*d.z()*q.z();

	J(0, 0) = two_dyq4 - two_dzq3;                 J(0, 1) = -two_dxq4 + two_dzq2;                J(0, 2) = two_dxq3 - two_dyq2;
	J(1, 0) = two_dyq3 + two_dzq4;                 J(1, 1) = two_dxq3 - 2.f*two_dyq2 + two_dzq1;  J(1, 2) = two_dxq4 - two_dyq1 - 2.f*two_dzq2;
	J(2, 0) = -2.f*two_dxq3 + two_dyq2 - two_dzq1; J(2, 1) = two_dxq2 + two_dzq4;                 J(2, 2) = two_dxq1 + two_dyq4 - 2.f*two_dzq3;
	J(3, 0) = -2.f*two_dxq4 + two_dyq1 + two_dzq2; J(3, 1) = -two_dxq1 - 2.f*two_dyq4 + two_dzq3; J(3, 2) = two_dxq2 + two_dyq3;
}

bool
psmove_alignment_quaternion_between_vector_frames(
	const Eigen::Vector3f* from[2], const Eigen::Vector3f* to[2], const float tolerance, const Eigen::Quaternionf &initial_q,
	Eigen::Quaternionf &out_q)
{
	bool success = true;

	Eigen::Quaternionf previous_q = initial_q;
	Eigen::Quaternionf q = initial_q;
	
    //const float tolerance_squared = tolerance*tolerance; //TODO: This variable is unused, but it should be. Need to re-test with this added since the below test should be: error_squared > tolerance_squared
	const int k_max_iterations = 32;
	float previous_error_squared = k_real_max;
	float error_squared = k_real_max;
	float squared_error_delta = k_real_max;
	float gamma = 0.5f;
	bool backtracked = false;

	for (int iteration = 0; 
		iteration < k_max_iterations && // Haven't exceeded iteration limit
		error_squared > tolerance && // Aren't within tolerance of the destination
		squared_error_delta > k_normal_epsilon && // Haven't reached a minima
		gamma > k_normal_epsilon; // Haven't reduced our step size to zero
		iteration++)
	{
		// Fill in the 6x1 objective function matrix |f_0|
		//                                           |f_1|
		float error_squared0, error_squared1;

		Eigen::Matrix<float, 3, 1> f_0;
		psmove_alignment_compute_objective_vector(q, *from[0], *to[0], f_0, &error_squared0);

		Eigen::Matrix<float, 3, 1> f_1;
		psmove_alignment_compute_objective_vector(q, *from[1], *to[1], f_1, &error_squared1);

		Eigen::Matrix<float, 6, 1> f;
		f.block<3, 1>(0, 0) = f_0;
		f.block<3, 1>(3, 0) = f_1;

		error_squared = error_squared0 + error_squared1;

		// Make sure this new step hasn't made the error worse
		if (error_squared <= previous_error_squared)
		{
			// We won't have a valid error derivative if we had to back track
			squared_error_delta = !backtracked ? fabsf(error_squared - previous_error_squared) : squared_error_delta;
			backtracked = false;

			// This is a good step.
			// Remember it in case the next one makes things worse
			previous_error_squared = error_squared;
			previous_q = q;

			// Fill in the 4x6 objective function Jacobian matrix: [J_0|J_1]
			Eigen::Matrix<float, 4, 3> J_0;
			psmove_alignment_compute_objective_jacobian(q, *from[0], J_0);

			Eigen::Matrix<float, 4, 3> J_1;
			psmove_alignment_compute_objective_jacobian(q, *from[1], J_1);

			Eigen::Matrix<float, 4, 6> J;
			J.block<4, 3>(0, 0) = J_0; J.block<4, 3>(0, 3) = J_1;

			// Compute the gradient of the objective function
			Eigen::Matrix<float, 4, 1> gradient_f = J*f;
			Eigen::Quaternionf gradient_q =
				Eigen::Quaternionf(gradient_f(0, 0), gradient_f(1, 0), gradient_f(2, 0), gradient_f(3, 0));

			// The gradient points toward the maximum, so we subtract it off to head toward the minimum.
			// The step scale 'gamma' is just a guess.			
			q = Eigen::Quaternionf(q.coeffs() - gradient_q.coeffs()*gamma); //q-= gradient_q*gamma;
			q.normalize();
		}
		else
		{
			// The step made the error worse.
			// Return to the previous orientation and half our step size.
			q = previous_q;
			gamma /= 2.f;
			backtracked = true;
		}
	}

	if (error_squared > tolerance)
	{
		// Make sure we didn't fail to converge on the goal
		success = false;
	}

	out_q= q;

	return success;
}
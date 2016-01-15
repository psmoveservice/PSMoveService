#ifndef MATH_ALIGNMNET_H
#define MATH_ALIGNMNET_H

//-- includes -----
#include "MathEigen.h"

//-- interface -----
Eigen::Quaternionf
psmove_alignment_quaternion_between_vectors(const Eigen::Vector3f &from, const Eigen::Vector3f &to);

void
psmove_alignment_compute_objective_vector(
	const Eigen::Quaternionf &q, const Eigen::Vector3f &d, const Eigen::Vector3f &s,
	Eigen::Matrix<float,3,1> &out_f, float *out_squared_error);

void
psmove_alignment_compute_objective_jacobian(
	const Eigen::Quaternionf &q, const Eigen::Vector3f &d, Eigen::Matrix<float, 4, 3> &J);

bool
psmove_alignment_quaternion_between_vector_frames(
	const Eigen::Vector3f* from[2], const Eigen::Vector3f* to[2], const float tolerance, const Eigen::Quaternionf &initial_q,
	Eigen::Quaternionf &out_q);

#endif // MATH_UTILITY_h
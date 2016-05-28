#ifndef MATH_ALIGNMNET_H
#define MATH_ALIGNMNET_H

//-- includes -----
#include "MathEigen.h"

//-- structs -----
struct EigenFitEllipsoid
{
    Eigen::Vector3f center;
    Eigen::Matrix3f basis;
    Eigen::Vector3f extents;
    float error;

    void clear()
    {
        center = Eigen::Vector3f::Zero();
        basis = Eigen::Matrix3f::Identity();
        extents = Eigen::Vector3f::Zero();
        error = 0.f;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EigenFitEllipse
{
    Eigen::Vector2f center;
    Eigen::Vector2f extents;
    float angle; // radians
    float error;

    void clear()
    {
        center = Eigen::Vector2f::Zero();
        extents = Eigen::Vector2f::Zero();
        angle = 0.f;
        error = 0.f;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//-- interface -----
Eigen::Quaternionf
eigen_alignment_quaternion_between_vectors(const Eigen::Vector3f &from, const Eigen::Vector3f &to);

void
eigen_alignment_compute_objective_vector(
	const Eigen::Quaternionf &q, const Eigen::Vector3f &d, const Eigen::Vector3f &s,
	Eigen::Matrix<float,3,1> &out_f, float *out_squared_error);

void
eigen_alignment_compute_objective_jacobian(
	const Eigen::Quaternionf &q, const Eigen::Vector3f &d, Eigen::Matrix<float, 4, 3> &J);

bool
eigen_alignment_quaternion_between_vector_frames(
	const Eigen::Vector3f* from[2], const Eigen::Vector3f* to[2], const float tolerance, const Eigen::Quaternionf &initial_q,
	Eigen::Quaternionf &out_q);

void
eigen_alignment_fit_bounding_box_ellipsoid(
    const Eigen::Vector3f *points, const int point_count,
    EigenFitEllipsoid &out_ellipsoid);

void
eigen_alignment_fit_min_volume_ellipsoid(
    const Eigen::Vector3f *points, const int point_count,
    const float tolerance,
    EigenFitEllipsoid &out_ellipsoid);

Eigen::Vector3f
eigen_alignment_project_point_on_ellipsoid_basis(
    const Eigen::Vector3f &point,
    const EigenFitEllipsoid &ellipsoid);

float
eigen_alignment_compute_ellipsoid_fit_error(
    const Eigen::Vector3f *points, const int point_count,
    const EigenFitEllipsoid &ellipsoid);

bool
eigen_alignment_fit_least_squares_ellipse(
    const Eigen::Vector2f *points,
    const int point_count,
    EigenFitEllipse &out_ellipse);

float
eigen_alignment_compute_ellipse_fit_error(
    const Eigen::Vector2f *points, const int point_count,
    const EigenFitEllipse &ellipsoid);

bool
eigen_alignment_fit_focal_cone_to_sphere(
    const Eigen::Vector2f *focal_plane_contour_points,
    const int focal_plane_contour_point_count,
    const float sphere_radius,
    const float camera_focal_length,
    Eigen::Vector3f *out_sphere_center);

#endif // MATH_UTILITY_h
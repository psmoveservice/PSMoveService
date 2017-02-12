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
    float area;
    float error;

    void clear()
    {
        center = Eigen::Vector2f::Zero();
        extents = Eigen::Vector2f::Zero();
        angle = 0.f;
        area = 0.f;
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

void
eigen_alignment_project_ellipse(Eigen::Vector3f *sphere_center,
                                float k,
                                float focal_length_proj,
                                float zz,
                                EigenFitEllipse *out_ellipse_projection);

// Method of cboulay
void
eigen_alignment_fit_focal_cone_to_sphere(
    const EigenFitEllipse &ellipse_projection,
    const float sphere_radius,
    const float focal_length_pts,
    Eigen::Vector3f *out_sphere_center);

// Method of Doc_ok
void
eigen_alignment_fit_focal_cone_to_sphere(
    const Eigen::Vector2f *points,
    const int point_count,
    const float sphere_radius,
    const float focal_length_pts, // a.k.a. "f_px"
    Eigen::Vector3f *out_sphere_center,
    EigenFitEllipse *out_ellipse_projection= nullptr);

// Compute the weighted average of multiple quaternions
// * All weights will be renormalized against the total weight
// * All input weights must be >= 0
bool
eigen_quaternion_compute_normalized_weighted_average(
    const Eigen::Quaternionf *quaternions,
    const float *weights,
    const int count,
    Eigen::Quaternionf *out_result);

// Compute the weighted average of multiple quaternions
// * Source weights are NOT renormalized
// * Source weights can be negative
bool
eigen_quaternion_compute_weighted_average(
    const Eigen::Quaterniond *quaternions,
    const double *weights,
    const int count,
    Eigen::Quaterniond *out_result);

void 
eigen_vector3f_compute_mean_and_variance(
	const Eigen::Vector3f *samples,
    const int sample_count,
	Eigen::Vector3f *out_mean,
    Eigen::Vector3f *out_variance);

// best fit line is of the form y(x) = out_line->x()*x + out_line->y()
bool
eigen_alignment_fit_least_squares_line(
	const Eigen::Vector2f *samples, const int sample_count,
	Eigen::Vector2f *out_line, float *out_correlation_coefficient);

// best fit curve of the form y(x) = out_curve->y()*exp(out_curve->x()*x), 
bool
eigen_alignment_fit_least_squares_exponential(
	const Eigen::Vector2f *samples, const int sample_count,
	Eigen::Vector2f *out_curve);

// Computes a best fit plane to the given set of data points
bool 
eigen_alignment_fit_least_squares_plane(
	const Eigen::Vector3f *samples, const int sample_count,
	Eigen::Vector3f *out_centroid, Eigen::Vector3f *out_normal);

// Project a point set onto a plane and compute the total distance error
float
eigen_alignment_project_points_on_plane(
	const Eigen::Vector3f &centroid, const Eigen::Vector3f &normal,
	Eigen::Vector3f *samples, const int sample_count);

// Compute the "Fundamental" camera matrix. 
// Used to convert a pixel location in one camera to pixel location on another camera.
void
eigen_alignment_compute_camera_fundamental_matrix(
	const Eigen::Vector3f &Ta, // world space translation of camera A
	const Eigen::Vector3f &Tb, // world space translation of camera B
	const Eigen::Quaternionf &Qa, // world space rotation of camera A
	const Eigen::Quaternionf &Qb, // world space rotation of camera B
	const Eigen::Matrix3f &Ka, // intrinsic matrix of camera A
	const Eigen::Matrix3f &Kb, // intrinsic matrix of camera B
	Eigen::Matrix3f &F_ab); // Output Fundamental matric F_ab

#endif // MATH_UTILITY_H

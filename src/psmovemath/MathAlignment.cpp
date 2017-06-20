//-- includes -----
#include "MathAlignment.h"
#include "Eigen/SVD"
#include "Eigen/Dense"
#include <iostream>

//-- public methods -----
Eigen::Quaternionf
eigen_alignment_quaternion_between_vectors(const Eigen::Vector3f &from, const Eigen::Vector3f &to)
{
    assert_eigen_vector3f_is_normalized(from);
    assert_eigen_vector3f_is_normalized(to);

    return Eigen::Quaternionf::FromTwoVectors(from, to).conjugate();
}

// This formula comes from Sebastian O.H. Madgwick's 2010 paper:
// "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
// https://www.samba.org/tridge/UAV/madgwick_internal_report.pdf
void
eigen_alignment_compute_objective_vector(
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
eigen_alignment_compute_objective_jacobian(
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
eigen_alignment_quaternion_between_vector_frames(
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
        eigen_alignment_compute_objective_vector(q, *from[0], *to[0], f_0, &error_squared0);

        Eigen::Matrix<float, 3, 1> f_1;
        eigen_alignment_compute_objective_vector(q, *from[1], *to[1], f_1, &error_squared1);

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
            eigen_alignment_compute_objective_jacobian(q, *from[0], J_0);

            Eigen::Matrix<float, 4, 3> J_1;
            eigen_alignment_compute_objective_jacobian(q, *from[1], J_1);

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

void
eigen_alignment_fit_bounding_box_ellipsoid(
    const Eigen::Vector3f *points, 
    const int point_count,
    EigenFitEllipsoid &out_ellipsoid)
{

    if (point_count > 0)
    {
        Eigen::Vector3f box_min(points[0].x(), points[0].y(), points[0].z());
        Eigen::Vector3f box_max = box_min;

        for (int point_index = 0; point_index < point_count; ++point_index)
        {
            const Eigen::Vector3f &point = points[point_index];
            
            box_min = box_min.cwiseMin(point);
            box_max = box_max.cwiseMax(point);
        }

        out_ellipsoid.center = (box_max + box_min) / 2.f;
        out_ellipsoid.extents = (box_max - box_min) / 2.f;
        out_ellipsoid.basis = Eigen::Matrix3f::Identity();
        out_ellipsoid.error = eigen_alignment_compute_ellipsoid_fit_error(points, point_count, out_ellipsoid);
    }
    else
    {
        out_ellipsoid.center = Eigen::Vector3f::Zero();
        out_ellipsoid.extents = Eigen::Vector3f::Zero();
        out_ellipsoid.basis = Eigen::Matrix3f::Identity();
        out_ellipsoid.error = 0.f;
    }
}

// See http://stackoverflow.com/questions/1768197/bounding-ellipse/1768440#1768440
// Relevant paper: http://www.seas.upenn.edu/~nima/papers/Mim_vol_ellipse.pdf
void
eigen_alignment_fit_min_volume_ellipsoid(
    const Eigen::Vector3f *points,
    const int point_count,
    const float tolerance,
    EigenFitEllipsoid &out_ellipsoid)
{
    const float POINT_DIMENSION = 3.f;

    if (point_count > POINT_DIMENSION)
    {
        const float N = static_cast<float>(point_count);

        //u is an Nx1 vector where each element is 1/N
        Eigen::VectorXf u(point_count);

        const int k_max_iteration_count = 100;
        float error = k_real_max;

        // Fill out 3xN and 4xN point matrices P & Q
        Eigen::MatrixXf P(3, point_count);
        Eigen::MatrixXf Q(4, point_count);

        int dest_row = 0;
        for (int point_index = 0; point_index < point_count; ++point_index)
        {
            const Eigen::Vector3f &point = points[point_index];

            P.col(dest_row) = point;
            Q.col(dest_row) = Eigen::Vector4f(point.x(), point.y(), point.z(), 1.f);
            ++dest_row;
        }

        // Run the Khachiyan Convex Optimization Algorithm
        u.setConstant(1.f / N);
        for (int iteration_count = 0; error > tolerance && iteration_count < k_max_iteration_count; ++iteration_count)
        {
            Eigen::Matrix4f X = Q*u.asDiagonal()*Q.transpose(); // (4xN)(NxN)(Nx4) = (4x4)
            Eigen::VectorXf M = (Q.transpose()*X.inverse()*Q).diagonal(); // [(Nx4)(4x4)(4xN)].diagonal() = (Nx1)

            // Find the max element and position in M
            int max_element_index = 0;
            float max_element = M[0];
            for (int element_index = 1; element_index < 4; ++element_index)
            {
                if (M[element_index] > max_element)
                {
                    max_element = M[element_index];
                    max_element_index = element_index;
                }
            }

            // Update u
            {
                // Calculate the step size for the ascent
                const float step_size = (max_element - POINT_DIMENSION - 1.f) / ((POINT_DIMENSION + 1.f)*(max_element - 1.f));
                Eigen::VectorXf new_u = (1.f - step_size)*u;

                new_u[max_element_index] = new_u[max_element_index] + step_size;

                error = (new_u - u).norm();

                u = new_u;
            }
        }

        // Compute the Ellipsoid A-matrix i.e. (X-c)'*A*(X-c)
		Eigen::Matrix3f PuP_trans= P*u.asDiagonal()*P.transpose();
		Eigen::Matrix3f PuPu_trans= (P*u)*(P*u).transpose();
        Eigen::Matrix3f A = (1.f / POINT_DIMENSION) * (PuP_trans - PuPu_trans).inverse();

        // Compute the singular values of A (where A = U*D*V)
        const Eigen::JacobiSVD<Eigen::Matrix3f> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        const Eigen::Vector3f D = svd.singularValues();

        // The orientation of the ellipse axes is the V matrix of the singular value decomposition
        out_ellipsoid.basis = svd.matrixV();

        // Compute the extents from the singular values
        out_ellipsoid.extents =
            Eigen::Vector3f(
            1.f / safe_sqrt_with_default(D(0), 100000),
            1.f / safe_sqrt_with_default(D(1), 100000),
            1.f / safe_sqrt_with_default(D(2), 100000));

        // Compute the center
        out_ellipsoid.center = P*u;

        // Compute the fit error
        out_ellipsoid.error = eigen_alignment_compute_ellipsoid_fit_error(points, point_count, out_ellipsoid);
    }
    else
    {
        eigen_alignment_fit_bounding_box_ellipsoid(points, point_count, out_ellipsoid);
    }
}

Eigen::Vector3f
eigen_alignment_project_point_on_ellipsoid_basis(
    const Eigen::Vector3f &point,
    const EigenFitEllipsoid &ellipsoid)
{
    // Compute a world space offset relative to the ellipsoid center
    const Eigen::Vector3f world_offset = point - ellipsoid.center;

    // Project the offset onto ellipse basis
    const Eigen::Vector3f ellipse_offset = ellipsoid.basis.transpose() * world_offset;

    // Scale the ellipsoid space into a sphere
    const Eigen::Vector3f sphere_offset =
        eigen_vector3f_divide_by_vector_with_default(ellipse_offset, ellipsoid.extents, Eigen::Vector3f::Zero());

    return sphere_offset;
}

float
eigen_alignment_compute_ellipsoid_fit_error(
    const Eigen::Vector3f *points,
    const int point_count,
    const EigenFitEllipsoid &ellipsoid)
{
    float error = 0.f;

    if (point_count > 1)
    {
        // Get the semi-axis lengths of the ellipse
        const float a_squared = ellipsoid.extents.x()*ellipsoid.extents.x();
        const float b_squared = ellipsoid.extents.y()*ellipsoid.extents.y();
        const float c_squared = ellipsoid.extents.z()*ellipsoid.extents.z();

        for (int point_index = 0; point_index < point_count; ++point_index)
        {
            // Compute the point relative to the ellipsoid center
            const Eigen::Vector3f &point = points[point_index];
            const Eigen::Vector3f offset = point - ellipsoid.center;

            // Project the offset onto basis
            const float x = offset.dot(ellipsoid.basis.col(0));
            const float y = offset.dot(ellipsoid.basis.col(1));
            const float z = offset.dot(ellipsoid.basis.col(2));

            // Compute the general ellipsoid equation: E(x, y, x)= (x^2/a^2) + (y^2/b^2) + (z^2/x^2) - 1
            // E(x, y, x) = 0 means the point is on the surface
            // E(x, y, x) > 0 means the point is above the surface
            // E(x, y, x) < 0 means the point is below the surface
            // |E(x, y, x)| gives us distance from the surface, which will treat as an error
            const float x_term= safe_divide_with_default(x*x, a_squared, 0.f);
            const float y_term= safe_divide_with_default(y*y, b_squared, 0.f);
            const float z_term= safe_divide_with_default(z*z, c_squared, 0.f);
            error += fabsf(x_term + y_term + z_term - 1.f);
        }
    }

    return error;
}

bool
eigen_alignment_fit_least_squares_ellipse(
    const Eigen::Vector2f *points,
    const int point_count,
    EigenFitEllipse &out_ellipse)
{
    float conic_params[6];
    bool bSuccess = false;

    // See http://autotrace.sourceforge.net/WSCG98.pdf
    Eigen::MatrixXf D1(point_count, 3);
    Eigen::MatrixXf D2(point_count, 3);
    for (int ix = 0; ix < point_count; ++ix) {
        D1.row(ix)[0] = points[ix].x() * points[ix].x();
        D1.row(ix)[1] = points[ix].x() * points[ix].y();
        D1.row(ix)[2] = points[ix].y() * points[ix].y();
        D2.row(ix)[0] = points[ix].x();
        D2.row(ix)[1] = points[ix].y();
        D2.row(ix)[2] = 1;
    }

    Eigen::Matrix3f S1 = D1.transpose() * D1;
    Eigen::Matrix3f S2 = D1.transpose() * D2;
    Eigen::Matrix3f S3 = D2.transpose() * D2;
    Eigen::Matrix3f T = -S3.colPivHouseholderQr().solve(S2.transpose());
    //                        Eigen::Matrix3f T = -S3.inverse() * S2.transpose();
    Eigen::Matrix3f M = S2*T + S1;
    Eigen::Matrix3f Mout;
    Mout.block<1, 3>(0, 0) = M.block<1, 3>(2, 0) / 2;
    Mout.block<1, 3>(1, 0) = -M.block<1, 3>(1, 0);
    Mout.block<1, 3>(2, 0) = M.block<1, 3>(0, 0) / 2;
    Eigen::EigenSolver<Eigen::Matrix3f> eigsolv(Mout);
    for (int row_ix = 0; row_ix<3; ++row_ix) 
    {
        Eigen::Vector3f evec = eigsolv.eigenvectors().col(row_ix).real();

        //cond = 4 * evec(1, :) .* evec(3, :) - evec(2, :).^2; % evaluate a'Ca
        if ((4.0 * evec[0] * evec[2]) - (evec[1] * evec[1]) > 0)
        {
            conic_params[0] = evec[0];
            conic_params[1] = evec[1];
            conic_params[2] = evec[2];

            Eigen::Vector3f Tevec = T*evec;
            conic_params[3] = Tevec[0];
            conic_params[4] = Tevec[1];
            conic_params[5] = Tevec[2];
        }
    }

    if (eigsolv.info() == Eigen::Success)
    {
        // Get the general quadratic curve parameters of the ellipse
        // solved for in the least square minimization
        // These express an ellipse of the form:
        // A*x^2 + 2*B*x*y + C*y^2 + 2*D*x + 2*F*y + G = 0
        float A = conic_params[0];
        float B = conic_params[1] / 2;
        float C = conic_params[2];
        float D = conic_params[3] / 2;
        float F = conic_params[4] / 2;
        float G = conic_params[5];

        // Convert to parametric parameters
        // These express an ellipse of the form:
        // x(t) = a*cos(t + tau) + h
        // y(t) = b*sin(t + tau) + k
        float BB = B*B;
        float AC = A*C;
        float FF = F*F;
        float off_d = BB - AC;
        float h = (C*D - B*F) / off_d;
        float k = (A*F - B*D) / off_d;
        float semi_n = A*FF + C*D*D + G*BB - 2 * B*D*F - AC*G;
        float dAC = A - C;
        float dACsq = dAC*dAC;
        float semi_d_2 = sqrtf(dACsq + 4 * BB);
        float semi_d_3 = -A - C;
        float a_sqrd = (2 * semi_n) / (off_d * (semi_d_3 + semi_d_2));
        float b_sqrd = (2 * semi_n) / (off_d * (semi_d_3 - semi_d_2));

        if (a_sqrd > k_real_epsilon && b_sqrd > k_real_epsilon)
        {
            // b and tau are only needed for drawing an ellipse.
            float a = sqrt(a_sqrd);
            float b = sqrt(b_sqrd);
            float tau = atan2f(2 * B, dAC) / 2.f;  //acot((A-C)/(2*B))/2;

            if (A > C)
            {
                tau += k_real_pi / 2.f;
            }

            out_ellipse.center = Eigen::Vector2f(h, k);
            out_ellipse.extents = Eigen::Vector2f(a, b);
            out_ellipse.angle = tau;
            out_ellipse.error = eigen_alignment_compute_ellipse_fit_error(points, point_count, out_ellipse);
            bSuccess = true;
        }
    }

    return bSuccess;
}

float
eigen_alignment_compute_ellipse_fit_error(
    const Eigen::Vector2f *points, const int point_count,
    const EigenFitEllipse &ellipse)
{
    float error = 0.f;

    // Get the semi-axis lengths of the ellipse
    const float a_squared = ellipse.extents.x()*ellipse.extents.x();
    const float b_squared = ellipse.extents.y()*ellipse.extents.y();

    Eigen::Vector2f basis_x(cosf(ellipse.angle), sinf(ellipse.angle));
    Eigen::Vector2f basis_y(-basis_x.y(), basis_x.x());

    for (int point_index = 0; point_index < point_count; ++point_index)
    {
        // Compute the point relative to the ellipsoid center
        const Eigen::Vector2f &point = points[point_index];
        const Eigen::Vector2f offset = point - ellipse.center;

        // Project the offset onto basis
        const float x = offset.dot(basis_x);
        const float y = offset.dot(basis_y);

        // Compute the general ellipsoid equation: E(x, y, x)= (x^2/a^2) + (y^2/b^2) + (z^2/x^2) - 1
        // E(x, y, x) = 0 means the point is on the surface
        // E(x, y, x) > 0 means the point is above the surface
        // E(x, y, x) < 0 means the point is below the surface
        // |E(x, y, x)| gives us distance from the surface, which will treat as an error
        error += fabsf((x*x / a_squared) + (y*y / b_squared) - 1.f);
    }

    return error;
}


void
eigen_alignment_project_ellipse(Eigen::Vector3f *sphere_center,
                                float k,
                                float focal_length_proj,
                                float zz,
                                EigenFitEllipse *out_ellipse_projection)
{
    // Get conical parameters
    float xx = sphere_center->x() * sphere_center->x();
    float yy = sphere_center->y() * sphere_center->y();
    float _zz = sphere_center->z() * sphere_center->z();
    float m = k*(xx + yy + _zz);
    float a = xx - m;
    float b = 2 * sphere_center->x()*sphere_center->y();
    float c = yy - m;
    float d = 2 * sphere_center->x()*sphere_center->z()*focal_length_proj;
    float f = 2 * sphere_center->y()*sphere_center->z()*focal_length_proj;
    float g = zz * (_zz - m);
    
    // Convert conical to parametric
    // http://mathworld.wolfram.com/Ellipse.html Eqns 19-23
    b /= 2;
    d /= 2;
    f /= 2;
    float bb = b*b;
    float off_d = bb - a*c;
    float h_ = (c*d - b*f) / off_d;
    float k_ = (a*f - b*d) / off_d;
    float semi_n = a*f*f + c*d*d + g*bb - 2 * b*d*f - a*c*g;
    float semi_d_2 = sqrt((a - c)*(a - c) + 4 * bb);
    float semi_d_3 = -1 * (a + c);
    float a_ = sqrt(2 * semi_n / (off_d * (semi_d_2 + semi_d_3)));
    float b_ = sqrt(2 * semi_n / (off_d * (-1 * semi_d_2 + semi_d_3)));
    float tau = 0;
    
    if (b != 0)
    {
        tau = atan2f(2 * b, a - c) / 2;
    }
    
    Eigen::Vector2f center;
    center << h_, k_;
    
    Eigen::Vector2f extents;
    extents << a_, b_;
    
    out_ellipse_projection->center = center;
    out_ellipse_projection->extents = extents;
    out_ellipse_projection->angle = tau;
    out_ellipse_projection->area = k_real_pi*a_*b_;
}


// Comments reference diagram in "Analytic solution after ellipse fitting"
// from: https://github.com/cboulay/PSMoveService/wiki/Optical-Tracker-Algorithms
void
eigen_alignment_fit_focal_cone_to_sphere(
    const EigenFitEllipse &ellipse_projection,
    const float sphere_radius, 
    const float camera_focal_length, // a.k.a. "f_px"
    Eigen::Vector3f *out_sphere_center)
{
    // The sphere can be thought of as a base of a cone whose vertex is at the camera focal point.
    // The camera's sensor plane can be thought of as a slice through the cone, creating an ellipse. 
    // We fit an ellipse to the blob. Then we can determine the sphere's 3D position by using similar 
    // triangles to relate known quantities(camera FOV, focal length, ellipse centre, major and minor axes) 
    // to unknown quantities.

    const float h = ellipse_projection.center.x();
    const float k = ellipse_projection.center.y();
    const float a = ellipse_projection.extents.x();

    // The length of the line from image centre to ellipse centre.
    const float L_px = sqrtf(h*h + k*k);

    // The green triangle goes from the camera pinhole (at origin)
    // to 0,0,f_px (centerpoint on focal plane),
    // to the center of the sphere on the focal plane (x_px, y_px, f_px)
    // The orange triangle extends the green triangle to go from pinhole,
    // to the middle of the sensor image, to the far edge of the ellipse (i.e. L_px + a_px)

    // Theta, the angle in the green triangle from the pinhole
    // to the center of the sphere on the image, off the focal axis:
    // theta = atan(m), where
    const float m = L_px / camera_focal_length;

    // We can now use another pair of similar (nested) triangles.
    // The outer triangle (blue+purple+orange) has base L_cm, side Z_cm, and hypotenuse D_cm.
    // The inner triangle (blue + some orange) has base L_px, size f_px, and hypotenuse D_px.
    // From the larger triangle, we get sin(gamma) = Z_cm / D_cm;
    // From the smaller triangle, we get tan(gamma) = f_px / L_px,
    // or gamma = atan( f_px / L_px );
    // then sin(gamma) = sin( atan( f_px / L_px ) ) = Z_cm / D_cm;
    // Again, using the sin-of-arctan identity
    // sin( atan( f_px / L_px ) ) = fl / sqrt( 1 + fl*fl ) = Z_cm / D_cm, where
    const float fl = camera_focal_length / L_px;

    // theta + alpha, the angle in the green+orange triangle
    // from the pinhole to the far edge of the ellipse, off the focal axis:
    // theta + alpha = atan( j ), where
    const float j = (L_px + a) / camera_focal_length;

    // Re-arranging for alpha:
    // alpha = atan(j) - atan(m);
    // Difference of atans (See 5.2.9 here:
    // http://www.mathamazement.com/Lessons/Pre-Calculus/05_Analytic-Trigonometry/sum-and-difference-formulas.html )
    // atan(j) - atan(m) = atan(l), where
    const float l = (j - m) / (1 + j*m);

    // The red+purple+orange triangle goes from camera pinhole,
    // to the edge of the sphere, to the center of the sphere.
    // sin(alpha) = R_cm / D_cm;
    // sin(atan(l)) = R_cm / D_cm;
    // sin of arctan (http://www.rapidtables.com/math/trigonometry/arctan/sin-of-arctan.htm )
    // sin(atan(l)) = l / sqrt( 1 + l*l )
    // R_cm / D_cm = l / sqrt( 1 + l*l )
    // Solve for D:
    const float D_cm = sphere_radius * sqrt(1 + l*l) / l;

    // Solve for Z_cm
    const float z = D_cm * fl / sqrt(1 + fl*fl);

    // Use a pair of similar triangles to find L_cm
    // 1: blue + purple + orange; tan(beta) = z_cm / L_cm
    // 2: inner blue + some orange; tan(beta) = f_px / L_px
    // Solve for L_cm
    const float L_cm = z * m;

    // We can now use the pair of gray triangles on the x-y plane to find x_cm and y_cm
    const float x = L_cm * h / L_px;
    const float y = L_cm * k / L_px;

    *out_sphere_center = Eigen::Vector3f(x, y, z);
}

void
eigen_alignment_fit_focal_cone_to_sphere(
    const Eigen::Vector2f *points,
    const int point_count,
    const float sphere_radius,
    const float focal_length_pts, // a.k.a. "f_px"
    Eigen::Vector3f *out_sphere_center,
    EigenFitEllipse *out_ellipse_projection)
{
    // Compute the sphere position whose projection on the focal plane
    // best fits the given convex contour
    float zz = focal_length_pts * focal_length_pts;

    Eigen::MatrixXf A(point_count, 3);
    for (int i = 0; i<point_count; ++i)
    {
        Eigen::Vector2f p = points[i];
        float norm_A = sqrt(p.x()*p.x() + p.y()*p.y() + zz);
        A(i, 0) = p.x();
        A(i, 1) = p.y();
        A(i, 2) = -norm_A;
    }

    Eigen::VectorXf b(point_count);
    b.fill(-zz);
    Eigen::Vector3f Bx_By_c = A.colPivHouseholderQr().solve(b);
    float norm_norm_B = sqrt(Bx_By_c[0] * Bx_By_c[0] +
        Bx_By_c[1] * Bx_By_c[1] +
        zz);
    float cos_theta = Bx_By_c[2] / norm_norm_B;
    float k = cos_theta * cos_theta;
    float norm_B = sphere_radius / sqrt(1 - k);

    *out_sphere_center << Bx_By_c[0], Bx_By_c[1], focal_length_pts;
    *out_sphere_center *= (norm_B / norm_norm_B);

    // Optionally compute the best fit ellipse
    if (out_ellipse_projection != nullptr)
    {
        eigen_alignment_project_ellipse(out_sphere_center, k,
                                        focal_length_pts, zz,
                                        out_ellipse_projection);
        
        out_ellipse_projection->error=
            eigen_alignment_compute_ellipse_fit_error(
                points, point_count, *out_ellipse_projection);
    }
}


bool
eigen_quaternion_compute_normalized_weighted_average(
    const Eigen::Quaternionf *quaternions,
    const float *weights,
    const int count,
    Eigen::Quaternionf *out_result)
{
    bool success = false;

    if (count == 1)
    {
        *out_result= quaternions[0];
        success= true;
    }
    else if (count == 2)
    {
		assert(weights == nullptr || weights[0] >= 0);
		assert(weights == nullptr || weights[1] >= 0);
		const float w0 = (weights != nullptr) ? weights[0] : 0.5f;
		const float w1 = (weights != nullptr) ? weights[1] : 0.5f;
        const float u= safe_divide_with_default(w1, w0 + w1, 0.f);

        *out_result= eigen_quaternion_normalized_lerp(quaternions[0], quaternions[1], u);
        success= true;
    }
    else if (count > 2)
    {
        // http://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
        Eigen::MatrixXf q(4, count);
        Eigen::MatrixXf q_transpose(count, 4);

        float total_weight= 0.f;
		if (weights != nullptr)
		{
			for (int index = 0; index < count; ++index)
			{
				assert(weights[index] >= 0);
				total_weight += weights[index];
			}
		}

        for (int index = 0; index < count; ++index)
        {
			// Normalize the weights against the total weight
            const Eigen::Quaternionf &sample = quaternions[index];
			const float weight = (weights != nullptr) ? weights[index] : 0.f;
            const float normalized_weight= safe_divide_with_default(weight, total_weight, 1.f);

            const float w= sample.w() * normalized_weight;
            const float x= sample.x() * normalized_weight;
            const float y= sample.y() * normalized_weight;
            const float z= sample.z() * normalized_weight;

            q(0, index) = w;
            q(1, index) = x;
            q(2, index) = y;
            q(3, index) = z;

            q_transpose(index, 0) = w;
            q_transpose(index, 1) = x;
            q_transpose(index, 2) = y;
            q_transpose(index, 3) = z;
        }

        Eigen::Matrix4f M= q*q_transpose;

        Eigen::EigenSolver<Eigen::Matrix4f> eigsolv(M);
        if (eigsolv.info() == Eigen::Success)
        {
            int largest_row = 0;
            float largest_eigenvalue = eigsolv.eigenvalues()[0].real();
            for (int row_ix = 1; row_ix < 4; ++row_ix) 
            {
                if (eigsolv.eigenvalues()[row_ix].real() > largest_eigenvalue)
                {
                    largest_eigenvalue = eigsolv.eigenvalues()[row_ix].real();
                    largest_row = row_ix;
                }                
            }

            Eigen::Vector4f largest_eigenvector = eigsolv.eigenvectors().col(largest_row).real();
            float w= largest_eigenvector(0);
            float x= largest_eigenvector(1);
            float y= largest_eigenvector(2);
            float z= largest_eigenvector(3);

            *out_result= Eigen::Quaternionf(w, x, y, z).normalized();
            success= true;
        }
    }

    return success;
}

bool
eigen_quaternion_compute_weighted_average(
    const Eigen::Quaterniond *quaternions,
    const double *weights,
    const int count,
    Eigen::Quaterniond *out_result)
{
    bool success = false;

    if (count == 1)
    {
        *out_result= quaternions[0];
        success= true;
    }
    else
    {
        // http://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
        Eigen::MatrixXd q(4, count);
        Eigen::MatrixXd q_transpose(count, 4);

        for (int index = 0; index < count; ++index)
        {
            const Eigen::Quaterniond &sample = quaternions[index];
			const double signed_weight= (weights != nullptr) ? weights[index] : 1.f;
            const double unsigned_weight= fabs(signed_weight);

            const double w= sample.w() * unsigned_weight;
			// For negative weights, use the conjugate of the quaternion 
			// (i.e. flip the rotation axis)
            const double x= sample.x() * signed_weight;
            const double y= sample.y() * signed_weight;
            const double z= sample.z() * signed_weight;

            q(0, index) = w;
            q(1, index) = x;
            q(2, index) = y;
            q(3, index) = z;

            q_transpose(index, 0) = w;
            q_transpose(index, 1) = x;
            q_transpose(index, 2) = y;
            q_transpose(index, 3) = z;
        }

        Eigen::Matrix4d M= q*q_transpose;

        Eigen::EigenSolver<Eigen::Matrix4d> eigsolv(M);
        if (eigsolv.info() == Eigen::Success)
        {
            int largest_row = 0;
            double largest_eigenvalue = eigsolv.eigenvalues()[0].real();
            for (int row_ix = 1; row_ix < 4; ++row_ix) 
            {
                if (eigsolv.eigenvalues()[row_ix].real() > largest_eigenvalue)
                {
                    largest_eigenvalue = eigsolv.eigenvalues()[row_ix].real();
                    largest_row = row_ix;
                }                
            }

            Eigen::Vector4d largest_eigenvector = eigsolv.eigenvectors().col(largest_row).real();
            double w= largest_eigenvector(0);
            double x= largest_eigenvector(1);
            double y= largest_eigenvector(2);
            double z= largest_eigenvector(3);

            *out_result= Eigen::Quaterniond(w, x, y, z).normalized();
            success= true;
        }
    }

    return success;
}

void 
eigen_vector3f_compute_mean_and_variance(
	const Eigen::Vector3f *samples,
    const int sample_count,
	Eigen::Vector3f *out_mean,
    Eigen::Vector3f *out_variance)
{
	assert(out_mean != nullptr || out_variance != nullptr);

	Eigen::Vector3f mean= Eigen::Vector3f::Zero();
	Eigen::Vector3f variance= Eigen::Vector3f::Zero();

	if (sample_count > 0.f)
	{
		const float N = static_cast<float>(sample_count);

		for (int sample_index = 0; sample_index < sample_count; sample_index++)
		{
			const Eigen::Vector3f &sample= samples[sample_index];

			mean+= sample;
		}
		mean/= N;

		// Compute the variance of the (unsigned) sample error, where "error" = abs(omega_sample)
		if (out_variance != nullptr)
		{
			for (int sample_index = 0; sample_index < sample_count; sample_index++)
			{
				const Eigen::Vector3f &sample = samples[sample_index];
				const Eigen::Vector3f diff_from_mean = sample - mean;

				variance += diff_from_mean.cwiseProduct(diff_from_mean);
			}
			variance /= (N - 1);
		}
	}
	
	if (out_mean != nullptr)
	{
		*out_mean = mean;
	}

	if (out_variance != nullptr)
	{
		*out_variance = variance;
	}
}

// From: http://stackoverflow.com/questions/5083465/fast-efficient-least-squares-fit-algorithm-in-c
bool 
eigen_alignment_fit_least_squares_line(
	const Eigen::Vector2f *samples, const int sample_count, 
	Eigen::Vector2f *out_line, float *out_correlation_coefficient)
{
	const float N = static_cast<float>(sample_count);

	float sumx = 0.f;
	float sumx2 = 0.f;
	float sumxy = 0.f;
	float sumy = 0.f;
	float sumy2 = 0.f;

	float m, b;
	bool bSuccess = false;

	for (int i = 0; i < sample_count; i++)
	{
		const Eigen::Vector2f &sample = samples[i];
		const float x_i = sample.x();
		const float y_i = sample.y();

		sumx += x_i;
		sumx2 += x_i*x_i;
		sumxy += x_i*y_i;
		sumy += y_i;
		sumy2 += y_i*y_i;
	}

	const float denom = (N*sumx2 - sumx*sumx);

	if (denom != 0)
	{
		m = (N*sumxy - sumx*sumy) / denom;
		b = (sumy*sumx2 - sumx*sumxy) / denom;

		if (out_correlation_coefficient != nullptr)
		{
			// compute correlation coeff
			*out_correlation_coefficient = (sumxy - sumx*sumy / N) / sqrtf((sumx2 - (sumx*sumx) / N) * (sumy2 - (sumy*sumy) / N));
		}

		bSuccess= true;
	}
	else
	{
		// singular matrix. can't solve the problem.
		m = 0;
		b = 0;

		if (out_correlation_coefficient != nullptr)
		{
			*out_correlation_coefficient = 0;
		}
	}

	if (out_line != nullptr)
	{
		*out_line = Eigen::Vector2f(m, b);
	}

	return bSuccess;
}

// From: http://mathworld.wolfram.com/LeastSquaresFittingExponential.html
bool
eigen_alignment_fit_least_squares_exponential(
	const Eigen::Vector2f *samples, const int sample_count,
	Eigen::Vector2f *out_curve)
{
	const float N = static_cast<float>(sample_count);

	float sum_lny = 0.f;
	float sum_x2 = 0.f;
	float sum_x = 0.f;
	float sum_xlny = 0.f;

	float a, b;
	bool bSuccess = false;

	for (int i = 0; i < sample_count; i++)
	{
		const Eigen::Vector2f &sample = samples[i];
		const float x_i = sample.x();
		const float y_i = sample.y();
		const float log_y_i = logf(y_i);

		sum_lny += log_y_i;
		sum_x2 += x_i*x_i;
		sum_x += x_i;
		sum_xlny += x_i*log_y_i;
	}

	const float denom = (N*sum_x2 - sum_x*sum_x);

	if (denom != 0)
	{
		a = static_cast<float>(exp((sum_lny*sum_x2 - sum_x*sum_xlny) / denom));
		b = (N*sum_xlny - sum_x*sum_lny) / denom;
		bSuccess = true;
	}
	else
	{
		// singular matrix. can't solve the problem.
		a = 0;
		b = 0;
	}

	if (out_curve != nullptr)
	{
		*out_curve = Eigen::Vector2f(b, a);
	}

	return bSuccess;
}

// Modified from: https://gist.github.com/ialhashim/0a2554076a6cf32831ca
bool 
eigen_alignment_fit_least_squares_plane(
	const Eigen::Vector3f *samples, const int sample_count,
	Eigen::Vector3f *out_centroid, Eigen::Vector3f *out_normal)
{
	bool bSuccess= false;

	if (sample_count > 3)
	{
		Eigen::Matrix< float, Eigen::Dynamic, Eigen::Dynamic > coord(3, sample_count);
		for (int i = 0; i < sample_count; ++i)
		{
			coord.col(i) = samples[i];
		}

		// calculate centroid
		Eigen::Vector3f centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

		// subtract centroid
		coord.row(0).array() -= centroid(0);
		coord.row(1).array() -= centroid(1); 
		coord.row(2).array() -= centroid(2);

		// we only need the left-singular matrix here
		//  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
		auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::Vector3f plane_normal = svd.matrixU().rightCols<1>();
		float length= plane_normal.norm();

		if (length > k_real_epsilon)
		{
			*out_centroid= centroid;
			*out_normal= plane_normal / length;
			bSuccess= true;
		}
	}
	else if (sample_count == 3)
	{
		Eigen::Vector3f centroid= (samples[0] + samples[1] + samples[2]) / 3.f;
		Eigen::Vector3f plane_normal= (samples[1] - samples[0]).cross((samples[2] - samples[0]));
		float length= plane_normal.norm();

		if (length > k_real_epsilon)
		{
			*out_centroid= centroid;
			*out_normal= plane_normal / length;
			bSuccess= true;
		}
	}

	return bSuccess;
}

float
eigen_alignment_project_points_on_plane(
	const Eigen::Vector3f &centroid, const Eigen::Vector3f &normal,
	Eigen::Vector3f *samples, const int sample_count)
{
	float total_error= 0.f;

	for (int sample_index = 0; sample_index < sample_count; ++sample_index)
	{
		const Eigen::Vector3f &sample= samples[sample_index];
		const Eigen::Vector3f centroidToSample= sample - centroid;
		const float signedDistanceToPlane= centroidToSample.dot(normal);

		// Move the sample onto the plane
		samples[sample_index]= sample - normal*signedDistanceToPlane;

		// Add up the total error distances from the plane
		total_error+= fabsf(signedDistanceToPlane);
	}

	return total_error;
}

void
eigen_alignment_compute_camera_fundamental_matrix(
	const Eigen::Vector3f &Ta,
	const Eigen::Vector3f &Tb,
	const Eigen::Quaternionf &Qa,
	const Eigen::Quaternionf &Qb,
	const Eigen::Matrix3f &Ka,
	const Eigen::Matrix3f &Kb,
	Eigen::Matrix3f &F_ab)
{
	// T = Translation from camera A to camera B
	Eigen::Vector3f T = Tb - Ta;
	Eigen::Matrix3f S;
	S << 0.f, T.z(), -T.y(),
		-T.z(), 0.f, T.x(),
		T.y(), -T.x(), 0.f;

	// R = Rotation matrix from camera A to camera B
	Eigen::Quaternionf R_quat = Qa.conjugate() * Qb;
	Eigen::Matrix3f R = R_quat.toRotationMatrix();

	// Essential Matrix from A to B, depending on extrinsic parameters
	Eigen::Matrix3f E = R * S;

	// Compute the fundamental matrix from camera A to camera B
	F_ab = Kb.inverse().transpose() * E * Ka.inverse();
}
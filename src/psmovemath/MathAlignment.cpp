//-- includes -----
#include "MathAlignment.h"
#include "Eigen/SVD"
#include "Eigen/Dense"

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
        Eigen::Matrix3f A = (1.f / POINT_DIMENSION) * (P*u.asDiagonal()*P.transpose() - (P*u)*(P*u).transpose()).inverse();

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
        error += fabsf((x*x / a_squared) + (y*y / b_squared) + (z*z / c_squared) - 1.f);
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
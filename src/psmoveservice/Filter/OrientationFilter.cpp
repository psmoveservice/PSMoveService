// -- includes -----
#include "OrientationFilter.h"
#include "MathAlignment.h"
#include "ServerLog.h"
#include <deque>

//-- constants -----
// Maximum we blend against the optically derived orientation
#define k_max_optical_orientation_weight 0.05f

// Complementary MARG Filter constants
#define k_base_earth_frame_align_weight 0.02f

// Max length of the orientation history we keep
#define k_orientation_history_max 16

// -- private definitions -----
struct OrientationFilterState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Is the current fusion state valid
    bool bIsValid;

    /* Physics State */
    Eigen::Quaternionf orientation;
    Eigen::Vector3f angular_velocity;
    Eigen::Vector3f angular_acceleration;

    /* Quaternion measured when controller points towards camera */
    Eigen::Quaternionf reset_orientation;

    void reset()
    {
        bIsValid= false;
        orientation= Eigen::Quaternionf::Identity();
        angular_velocity = Eigen::Vector3f::Zero();
        angular_acceleration = Eigen::Vector3f::Zero();
        reset_orientation= Eigen::Quaternionf::Identity();
    }

    void apply_state(
        const Eigen::Quaternionf &new_orientation,
        const Eigen::Vector3f &new_angular_velocity,
        const Eigen::Vector3f &new_angular_acceleration)
    {
        if (eigen_quaternion_is_valid(new_orientation))
        {
            orientation = new_orientation;
        }
        else
        {
            SERVER_LOG_WARNING("OrientationFilter") << "Orientation is NaN!";
        }

        if (eigen_vector3f_is_valid(new_angular_velocity))
        {
            angular_velocity= new_angular_velocity;
        }
        else
        {
            SERVER_LOG_WARNING("OrientationFilter") << "Angular Velocity is NaN!";
        }

        if (eigen_vector3f_is_valid(new_angular_acceleration))
        {
            angular_acceleration= new_angular_acceleration;
        }
        else
        {
            SERVER_LOG_WARNING("OrientationFilter") << "Angular Acceleration is NaN!";
        }

        // state is valid now that we have had an update
        bIsValid= true;
    }
};

// -- public interface -----
//-- Orientation Filter --
OrientationFilter::OrientationFilter() :
    m_state(new OrientationFilterState)
{
    memset(&m_constants, 0, sizeof(OrientationFilterConstants));
    resetState();
}

OrientationFilter::~OrientationFilter()
{
    delete m_state;
}

bool OrientationFilter::getIsStateValid() const
{
    return m_state->bIsValid;
}

void OrientationFilter::resetState()
{
    m_state->reset();
}

void OrientationFilter::recenterOrientation(const Eigen::Quaternionf& q_pose)
{
    Eigen::Quaternionf q_inverse = m_state->orientation.conjugate();

    eigen_quaternion_normalize_with_default(q_inverse, Eigen::Quaternionf::Identity());
    m_state->reset_orientation= q_pose*q_inverse;
}

bool OrientationFilter::init(const OrientationFilterConstants &constants)
{
    resetState();
    m_constants= constants;

    return true;
}

bool OrientationFilter::init(const OrientationFilterConstants &constants, const Eigen::Quaternionf &initial_orientation)
{
	resetState();
	m_constants = constants;
	m_state->orientation = initial_orientation;
	m_state->bIsValid = true;

	return true;
}

Eigen::Quaternionf OrientationFilter::getOrientation(float time) const
{
    Eigen::Quaternionf result = Eigen::Quaternionf::Identity();

    if (m_state->bIsValid)
    {
        Eigen::Quaternionf predicted_orientation = m_state->orientation;

        if (fabsf(time) > k_real_epsilon)
        {
            const Eigen::Quaternionf &quaternion_derivative=
                eigen_angular_velocity_to_quaternion_derivative(m_state->orientation, m_state->angular_velocity);

            predicted_orientation= Eigen::Quaternionf(
                m_state->orientation.coeffs()
                + quaternion_derivative.coeffs()*time).normalized();
        }

        result = m_state->reset_orientation * predicted_orientation;
    }

    return result;
}

Eigen::Vector3f OrientationFilter::getAngularVelocityRadPerSec() const
{
    return m_state->bIsValid ? m_state->angular_velocity : Eigen::Vector3f::Zero();
}

Eigen::Vector3f OrientationFilter::getAngularAccelerationRadPerSecSqr() const
{
    return m_state->bIsValid ? m_state->angular_acceleration : Eigen::Vector3f::Zero();
}

// -- OrientationFilterPassThru --
void OrientationFilterPassThru::update(const float delta_time, const PoseFilterPacket &packet)
{
	// Use the current orientation if the optical orientation is unavailable
    const Eigen::Quaternionf &new_orientation= 
		(packet.tracking_projection_area_px_sqr > 0.f) 
		? packet.optical_orientation
		: packet.current_orientation;

    const Eigen::Quaternionf orientation_derivative= 
        Eigen::Quaternionf((new_orientation.coeffs() - m_state->orientation.coeffs()) / delta_time);
    const Eigen::Vector3f new_angular_velocity = 
        Eigen::Vector3f::Zero(); //eigen_quaternion_derivative_to_angular_velocity(new_orientation, orientation_derivative);		
    const Eigen::Vector3f new_angular_accelertion = 
        Eigen::Vector3f::Zero(); //(new_angular_velocity - m_state->angular_velocity) / delta_time;

    m_state->apply_state(new_orientation, new_angular_velocity, new_angular_accelertion);
}

// -- OrientationFilterMadgwickARG --
// This algorithm comes from Sebastian O.H. Madgwick's 2010 paper:
// "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
// https://www.samba.org/tridge/UAV/madgwick_internal_report.pdf
void OrientationFilterMadgwickARG::update(const float delta_time, const PoseFilterPacket &packet)
{
    const Eigen::Vector3f &current_omega= packet.imu_gyroscope_rad_per_sec;

    Eigen::Vector3f current_g= packet.imu_accelerometer_g_units;
    eigen_vector3f_normalize_with_default(current_g, Eigen::Vector3f::Zero());

    // Current orientation from earth frame to sensor frame
    const Eigen::Quaternionf SEq = m_state->orientation;
    Eigen::Quaternionf SEq_new = SEq;

    // Compute the quaternion derivative measured by gyroscopes
    // Eqn 12) q_dot = 0.5*q*omega
    Eigen::Quaternionf omega = Eigen::Quaternionf(0.f, current_omega.x(), current_omega.y(), current_omega.z());
    Eigen::Quaternionf SEqDot_omega = Eigen::Quaternionf(SEq.coeffs() * 0.5f) *omega;

    if (!current_g.isApprox(Eigen::Vector3f::Zero(), k_normal_epsilon))
    {
        // Get the direction of the gravitational fields in the identity pose		
        Eigen::Vector3f k_identity_g_direction = m_constants.gravity_calibration_direction;

        // Eqn 15) Applied to the gravity vector
        // Fill in the 3x1 objective function matrix f(SEq, Sa) =|f_g|
        Eigen::Matrix<float, 3, 1> f_g;
        eigen_alignment_compute_objective_vector(SEq, k_identity_g_direction, current_g, f_g, NULL);

        // Eqn 21) Applied to the gravity vector
        // Fill in the 4x3 objective function Jacobian matrix: J_gb(SEq)= [J_g]
        Eigen::Matrix<float, 4, 3> J_g;
        eigen_alignment_compute_objective_jacobian(SEq, k_identity_g_direction, J_g);

        // Eqn 34) gradient_F= J_g(SEq)*f(SEq, Sa)
        // Compute the gradient of the objective function
        Eigen::Matrix<float, 4, 1> gradient_f = J_g * f_g;
        Eigen::Quaternionf SEqHatDot =
            Eigen::Quaternionf(gradient_f(0, 0), gradient_f(1, 0), gradient_f(2, 0), gradient_f(3, 0));

        // normalize the gradient
        eigen_quaternion_normalize_with_default(SEqHatDot, *k_eigen_quaternion_zero);

        // Compute the estimated quaternion rate of change
        // Eqn 43) SEq_est = SEqDot_omega - beta*SEqHatDot
        const float beta= sqrtf(3.0f / 4.0f) * fmaxf(fmaxf(m_constants.gyro_variance.x(), m_constants.gyro_variance.y()), m_constants.gyro_variance.z());
        Eigen::Quaternionf SEqDot_est = Eigen::Quaternionf(SEqDot_omega.coeffs() - SEqHatDot.coeffs()*beta);

        // Compute then integrate the estimated quaternion rate
        // Eqn 42) SEq_new = SEq + SEqDot_est*delta_t
        SEq_new = Eigen::Quaternionf(SEq.coeffs() + SEqDot_est.coeffs()*delta_time);
    }
    else
    {
        SEq_new = Eigen::Quaternionf(SEq.coeffs() + SEqDot_omega.coeffs()*delta_time);
    }

    // Make sure the net quaternion is a pure rotation quaternion
    SEq_new.normalize();

    // Save the new quaternion and first derivative back into the orientation state
    // Derive the second derivative
    {
        const Eigen::Quaternionf &new_orientation = SEq_new;
        const Eigen::Vector3f new_angular_velocity= Eigen::Vector3f::Zero(); // current_omega;
        const Eigen::Vector3f new_angular_acceleration= Eigen::Vector3f::Zero(); // (current_omega - m_state->angular_velocity) / delta_time;

        m_state->apply_state(new_orientation, new_angular_velocity, new_angular_acceleration);
    }
}

// -- OrientationFilterMadgwickMARG --
// This algorithm comes from Sebastian O.H. Madgwick's 2010 paper:
// "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
// https://www.samba.org/tridge/UAV/madgwick_internal_report.pdf
void OrientationFilterMadgwickMARG::resetState()
{
    OrientationFilterMadgwickARG::resetState();
    m_omega_bias_x= m_omega_bias_y= m_omega_bias_z= 0.f;
}

void OrientationFilterMadgwickMARG::update(const float delta_time, const PoseFilterPacket &packet)
{
    const Eigen::Vector3f &current_omega= packet.imu_gyroscope_rad_per_sec;

    Eigen::Vector3f current_g= packet.imu_accelerometer_g_units;
    eigen_vector3f_normalize_with_default(current_g, Eigen::Vector3f::Zero());

    Eigen::Vector3f current_m= packet.imu_magnetometer_unit;
    eigen_vector3f_normalize_with_default(current_m, Eigen::Vector3f::Zero());

    // If there isn't a valid magnetometer or accelerometer vector, fall back to the IMU style update
    if (current_g.isZero(k_normal_epsilon) || current_m.isZero(k_normal_epsilon))
    {
        OrientationFilterMadgwickARG::update(delta_time, packet);
        return;
    }

    // Current orientation from earth frame to sensor frame
    const Eigen::Quaternionf SEq = m_state->orientation;

    // Get the direction of the magnetic fields in the identity pose.	
    // NOTE: In the original paper we converge on this vector over time automatically (See Eqn 45 & 46)
    // but since we've already done the work in calibration to get this vector, let's just use it.
    // This also removes the last assumption in this function about what 
    // the orientation of the identity-pose is (handled by the sensor transform).
    Eigen::Vector3f k_identity_m_direction = m_constants.magnetometer_calibration_direction;

    // Get the direction of the gravitational fields in the identity pose
    Eigen::Vector3f k_identity_g_direction = m_constants.gravity_calibration_direction;

    // Eqn 15) Applied to the gravity and magnetometer vectors
    // Fill in the 6x1 objective function matrix f(SEq, Sa, Eb, Sm) =|f_g|
    //                                                               |f_b|
    Eigen::Matrix<float, 3, 1> f_g;
    eigen_alignment_compute_objective_vector(SEq, k_identity_g_direction, current_g, f_g, NULL);

    Eigen::Matrix<float, 3, 1> f_m;
    eigen_alignment_compute_objective_vector(SEq, k_identity_m_direction, current_m, f_m, NULL);

    Eigen::Matrix<float, 6, 1> f_gb;
    f_gb.block<3, 1>(0, 0) = f_g;
    f_gb.block<3, 1>(3, 0) = f_m;

    // Eqn 21) Applied to the gravity and magnetometer vectors
    // Fill in the 4x6 objective function Jacobian matrix: J_gb(SEq, Eb)= [J_g|J_b]
    Eigen::Matrix<float, 4, 3> J_g;
    eigen_alignment_compute_objective_jacobian(SEq, k_identity_g_direction, J_g);

    Eigen::Matrix<float, 4, 3> J_m;
    eigen_alignment_compute_objective_jacobian(SEq, k_identity_m_direction, J_m);

    Eigen::Matrix<float, 4, 6> J_gb;
    J_gb.block<4, 3>(0, 0) = J_g; J_gb.block<4, 3>(0, 3) = J_m;

    // Eqn 34) gradient_F= J_gb(SEq, Eb)*f(SEq, Sa, Eb, Sm)
    // Compute the gradient of the objective function
    Eigen::Matrix<float, 4, 1> gradient_f = J_gb*f_gb;
    Eigen::Quaternionf SEqHatDot =
        Eigen::Quaternionf(gradient_f(0, 0), gradient_f(1, 0), gradient_f(2, 0), gradient_f(3, 0));

    // normalize the gradient to estimate direction of the gyroscope error
    eigen_quaternion_normalize_with_default(SEqHatDot, *k_eigen_quaternion_zero);

    // Eqn 47) omega_err= 2*SEq*SEqHatDot
    // compute angular estimated direction of the gyroscope error
    Eigen::Quaternionf omega_err = Eigen::Quaternionf(SEq.coeffs()*2.f) * SEqHatDot;

    // Eqn 48) net_omega_bias+= zeta*omega_err
    // Compute the net accumulated gyroscope bias
    const float zeta= sqrtf(3.0f / 4.0f) * fmaxf(fmaxf(m_constants.gyro_variance.x(), m_constants.gyro_variance.y()), m_constants.gyro_variance.z());
    Eigen::Quaternionf omega_bias(0.f, m_omega_bias_x, m_omega_bias_y, m_omega_bias_z);
    omega_bias = Eigen::Quaternionf(omega_bias.coeffs() + omega_err.coeffs()*zeta*delta_time);
    m_omega_bias_x= omega_bias.x();
    m_omega_bias_y= omega_bias.y();
    m_omega_bias_z= omega_bias.z();

    // Eqn 49) omega_corrected = omega - net_omega_bias
    Eigen::Quaternionf omega = Eigen::Quaternionf(0.f, current_omega.x(), current_omega.y(), current_omega.z());
    Eigen::Quaternionf corrected_omega = Eigen::Quaternionf(omega.coeffs() - omega_bias.coeffs());

    // Compute the rate of change of the orientation purely from the gyroscope
    // Eqn 12) q_dot = 0.5*q*omega
    Eigen::Quaternionf SEqDot_omega = Eigen::Quaternionf(SEq.coeffs() * 0.5f) * corrected_omega;

    // Compute the estimated quaternion rate of change
    // Eqn 43) SEq_est = SEqDot_omega - beta*SEqHatDot
    const float beta= sqrtf(3.0f / 4.0f) * fmaxf(fmaxf(m_constants.gyro_variance.x(), m_constants.gyro_variance.y()), m_constants.gyro_variance.z());
    Eigen::Quaternionf SEqDot_est = Eigen::Quaternionf(SEqDot_omega.coeffs() - SEqHatDot.coeffs()*beta);

    // Compute then integrate the estimated quaternion rate
    // Eqn 42) SEq_new = SEq + SEqDot_est*delta_t
    Eigen::Quaternionf SEq_new = Eigen::Quaternionf(SEq.coeffs() + SEqDot_est.coeffs()*delta_time);

    // Make sure the net quaternion is a pure rotation quaternion
    SEq_new.normalize();

    // Save the new quaternion and first derivative back into the orientation state
    // Derive the second derivative
    {
        const Eigen::Quaternionf &new_orientation = SEq_new;
        const Eigen::Vector3f new_angular_velocity = Eigen::Vector3f::Zero(); //(corrected_omega.x(), corrected_omega.y(), corrected_omega.z());
        const Eigen::Vector3f new_angular_acceleration = Eigen::Vector3f::Zero(); //(new_angular_velocity - m_state->angular_velocity) / delta_time;

        m_state->apply_state(new_orientation, new_angular_velocity, new_angular_acceleration);
    }
}

// -- OrientationFilterComplementaryOpticalARG --
void OrientationFilterComplementaryOpticalARG::update(const float delta_time, const PoseFilterPacket &packet)
{
    if (packet.tracking_projection_area_px_sqr <= k_real_epsilon)
    {
        OrientationFilterMadgwickARG::update(delta_time, packet);
		return;
    }

    const Eigen::Vector3f &current_omega= packet.imu_gyroscope_rad_per_sec;

    Eigen::Vector3f current_g= packet.imu_accelerometer_g_units;
    eigen_vector3f_normalize_with_default(current_g, Eigen::Vector3f::Zero());

    // Current orientation from earth frame to sensor frame
    const Eigen::Quaternionf SEq = m_state->orientation;
    Eigen::Quaternionf SEq_new = SEq;

    // Compute the quaternion derivative measured by gyroscopes
    // Eqn 12) q_dot = 0.5*q*omega
    Eigen::Quaternionf omega = Eigen::Quaternionf(0.f, current_omega.x(), current_omega.y(), current_omega.z());
    Eigen::Quaternionf SEqDot_omega = Eigen::Quaternionf(SEq.coeffs() * 0.5f) *omega;

    if (!current_g.isApprox(Eigen::Vector3f::Zero(), k_normal_epsilon))
    {
        // Get the direction of the gravitational fields in the identity pose		
        Eigen::Vector3f k_identity_g_direction = m_constants.gravity_calibration_direction;

        // Eqn 15) Applied to the gravity vector
        // Fill in the 3x1 objective function matrix f(SEq, Sa) =|f_g|
        Eigen::Matrix<float, 3, 1> f_g;
        eigen_alignment_compute_objective_vector(SEq, k_identity_g_direction, current_g, f_g, NULL);

        // Eqn 21) Applied to the gravity vector
        // Fill in the 4x3 objective function Jacobian matrix: J_gb(SEq)= [J_g]
        Eigen::Matrix<float, 4, 3> J_g;
        eigen_alignment_compute_objective_jacobian(SEq, k_identity_g_direction, J_g);

        // Eqn 34) gradient_F= J_g(SEq)*f(SEq, Sa)
        // Compute the gradient of the objective function
        Eigen::Matrix<float, 4, 1> gradient_f = J_g * f_g;
        Eigen::Quaternionf SEqHatDot =
            Eigen::Quaternionf(gradient_f(0, 0), gradient_f(1, 0), gradient_f(2, 0), gradient_f(3, 0));

        // normalize the gradient
        eigen_quaternion_normalize_with_default(SEqHatDot, *k_eigen_quaternion_zero);

        // Compute the estimated quaternion rate of change
        // Eqn 43) SEq_est = SEqDot_omega - beta*SEqHatDot
        const float beta= sqrtf(3.0f / 4.0f) * fmaxf(fmaxf(m_constants.gyro_variance.x(), m_constants.gyro_variance.y()), m_constants.gyro_variance.z());
        Eigen::Quaternionf SEqDot_est = Eigen::Quaternionf(SEqDot_omega.coeffs() - SEqHatDot.coeffs()*beta);

        // Compute then integrate the estimated quaternion rate
        // Eqn 42) SEq_new = SEq + SEqDot_est*delta_t
        SEq_new = Eigen::Quaternionf(SEq.coeffs() + SEqDot_est.coeffs()*delta_time);
    }
    else
    {
        SEq_new = Eigen::Quaternionf(SEq.coeffs() + SEqDot_omega.coeffs()*delta_time);
    }

    // Make sure the net quaternion is a pure rotation quaternion
    SEq_new.normalize();

	// Blend with optical yaw
	Eigen::Quaternionf blended_orientation_new = SEq_new;
	if (packet.tracking_projection_area_px_sqr > 0)
    {
        // The final rotation is a blend between the integrated orientation and absolute optical orientation
		const float fraction_of_max_orientation_variance =
			safe_divide_with_default(
				m_constants.orientation_variance_curve.evaluate(packet.tracking_projection_area_px_sqr),
				m_constants.orientation_variance_curve.MaxValue,
				1.f);
		const float optical_orientation_quality = clampf01(1.f - fraction_of_max_orientation_variance);
        float optical_weight= 
			lerp_clampf(0, k_max_optical_orientation_weight, optical_orientation_quality);
        
        static float g_weight_override= -1.f;
        if (g_weight_override >= 0.f)
        {
            optical_weight= g_weight_override;
        }

		blended_orientation_new= eigen_quaternion_normalized_lerp(SEq_new, packet.optical_orientation, optical_weight);

		//const Eigen::EulerAnglesf optical_euler_angles = eigen_quaternionf_to_euler_angles(packet.optical_orientation);
		//const Eigen::EulerAnglesf SEeuler_new= eigen_quaternionf_to_euler_angles(SEq_new);

		//// Blend in the yaw from the optical orientation
		//const float blended_heading_radians= 
		//	wrap_lerpf(
		//		SEeuler_new.get_heading_radians(), 
		//		optical_euler_angles.get_heading_radians(), 
		//		optical_weight, 
		//		-k_real_pi, k_real_pi);
		//const Eigen::EulerAnglesf new_euler_angles(
		//	SEeuler_new.get_bank_radians(), blended_heading_radians, SEeuler_new.get_attitude_radians());

		//blended_orientation_new = eigen_euler_angles_to_quaternionf(new_euler_angles);
    }

	{
		// Compute the angular acceleration from the time derivative of the angular velocity
		const Eigen::Vector3f new_angular_velocity = Eigen::Vector3f::Zero(); //current_omega;
		const Eigen::Vector3f new_angular_acceleration = Eigen::Vector3f::Zero(); // (current_omega - m_state->angular_velocity) / delta_time;

		m_state->apply_state(blended_orientation_new, new_angular_velocity, new_angular_acceleration);
	}
}

// -- OrientationFilterComplementaryMARG --
void OrientationFilterComplementaryMARG::resetState()
{
    OrientationFilter::resetState();
    mg_weight= 1.f;
}

void OrientationFilterComplementaryMARG::update(const float delta_time, const PoseFilterPacket &packet)
{
    const Eigen::Vector3f &current_omega= packet.imu_gyroscope_rad_per_sec;

    Eigen::Vector3f current_g= packet.imu_accelerometer_g_units;
    eigen_vector3f_normalize_with_default(current_g, Eigen::Vector3f::Zero());

    Eigen::Vector3f current_m= packet.imu_magnetometer_unit;
    eigen_vector3f_normalize_with_default(current_m, Eigen::Vector3f::Zero());

    // Get the direction of the magnetic fields in the identity pose.	
    Eigen::Vector3f k_identity_m_direction = m_constants.magnetometer_calibration_direction;

    // Get the direction of the gravitational fields in the identity pose
    Eigen::Vector3f k_identity_g_direction = m_constants.gravity_calibration_direction;

    // Angular Rotation (AR) Update
    //-----------------------------
    // Compute the rate of change of the orientation purely from the gyroscope
    // q_dot = 0.5*q*omega
    Eigen::Quaternionf q_current= m_state->orientation;

    Eigen::Quaternionf q_omega = Eigen::Quaternionf(0.f, current_omega.x(), current_omega.y(), current_omega.z());
    Eigen::Quaternionf q_derivative = Eigen::Quaternionf(q_current.coeffs()*0.5f) * q_omega;

    // Integrate the rate of change to get a new orientation
    // q_new= q + q_dot*dT
    Eigen::Quaternionf q_step = Eigen::Quaternionf(q_derivative.coeffs() * delta_time);
    Eigen::Quaternionf ar_orientation = Eigen::Quaternionf(q_current.coeffs() + q_step.coeffs());

    // Make sure the resulting quaternion is normalized
    ar_orientation.normalize();

    // Magnetic/Gravity (MG) Update
    //-----------------------------
    const Eigen::Vector3f* mg_from[2] = { &k_identity_g_direction, &k_identity_m_direction };
    const Eigen::Vector3f* mg_to[2] = { &current_g, &current_m };
    Eigen::Quaternionf mg_orientation;

    // Always attempt to align with the identity_mg, even if we don't get within the alignment tolerance.
    // More often then not we'll be better off moving forward with what we have and trying to refine
    // the alignment next frame.
    eigen_alignment_quaternion_between_vector_frames(
        mg_from, mg_to, 0.1f, q_current, mg_orientation);

    // Blending Update
    //----------------
    // Save the new quaternion and first derivative back into the orientation state
    // Derive the second derivative
    {
        // The final rotation is a blend between the integrated orientation and absolute rotation from the earth-frame
        const Eigen::Quaternionf new_orientation = 
            eigen_quaternion_normalized_lerp(ar_orientation, mg_orientation, mg_weight);            
        const Eigen::Vector3f new_angular_velocity= Eigen::Vector3f::Zero(); // current_omega;
        const Eigen::Vector3f new_angular_acceleration = Eigen::Vector3f::Zero(); // (current_omega - m_state->angular_velocity) / delta_time;

        m_state->apply_state(new_orientation, new_angular_velocity, new_angular_acceleration);
    }

    // Update the blend weight
    // -- Exponential blend the MG weight from 1 down to k_base_earth_frame_align_weight
    mg_weight = lerp_clampf(mg_weight, k_base_earth_frame_align_weight, 0.9f);
}
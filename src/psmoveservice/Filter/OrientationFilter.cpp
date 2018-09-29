// -- includes -----
#include "OrientationFilter.h"
#include "MathAlignment.h"
#include "ServerLog.h"
#include <deque>

//-- constants -----
// Maximum we blend against the optically derived orientation
#define k_max_optical_orientation_weight 0.005f

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
    double time;

    /* Quaternion measured when controller points towards camera */
    Eigen::Quaternionf reset_orientation;

	/// The amount of time since the optical filter was updated
	double accumulated_optical_time_delta;

	/// The amount of time since the imu filter was updated
	double accumulated_imu_time_delta;

    void reset()
    {
        bIsValid= false;
        orientation= Eigen::Quaternionf::Identity();
        angular_velocity = Eigen::Vector3f::Zero();
        angular_acceleration = Eigen::Vector3f::Zero();
        reset_orientation= Eigen::Quaternionf::Identity();
        time= 0.0;
		accumulated_optical_time_delta= 0.f;
		accumulated_imu_time_delta= 0.f;
    }

    void apply_imu_state(
        const Eigen::Quaternionf &new_orientation,
        const Eigen::Vector3f &new_angular_velocity,
        const Eigen::Vector3f &new_angular_acceleration,
		const float delta_time)
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

		if (is_valid_float(delta_time))
		{
			time= accumulated_imu_time_delta + (double)delta_time;
			accumulated_imu_time_delta= 0.0;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "time delta is NaN!";
		}

        // state is valid now that we have had an update
        bIsValid= true;
    }

    void apply_optical_state(
        const Eigen::Quaternionf &new_orientation,
		const float delta_time)
    {
        if (eigen_quaternion_is_valid(new_orientation))
        {
            orientation = new_orientation;
        }
        else
        {
            SERVER_LOG_WARNING("OrientationFilter") << "Orientation is NaN!";
        }

		if (is_valid_float(delta_time))
		{
			time= accumulated_imu_time_delta + (double)delta_time;
			accumulated_imu_time_delta= 0.0;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "time delta is NaN!";
		}

        // state is valid now that we have had an update
        bIsValid= true;
    }

	void accumulate_optical_delta_time(const float delta_time)
	{
		if (is_valid_float(delta_time))
		{
			accumulated_optical_time_delta+= (double)delta_time;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "optical time delta is NaN!";
		}
	}

	void accumulate_imu_delta_time(const float delta_time)
	{
		if (is_valid_float(delta_time))
		{
			accumulated_imu_time_delta+= (double)delta_time;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "imu time delta is NaN!";
		}
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

double OrientationFilter::getTimeInSeconds() const
{
    return m_state->time;
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
	if (packet.has_optical_measurement())
	{
		const Eigen::Quaternionf &new_orientation= packet.optical_orientation;

		m_state->apply_optical_state(new_orientation, delta_time);
	}
	else
	{
		m_state->accumulate_optical_delta_time(delta_time);
	}
}

// -- OrientationFilterMadgwickARG --
// This algorithm comes from Sebastian O.H. Madgwick's 2010 paper:
// "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
// https://www.samba.org/tridge/UAV/madgwick_internal_report.pdf
void OrientationFilterMadgwickARG::update(const float delta_time, const PoseFilterPacket &packet)
{
	if (packet.has_imu_measurements())
	{
		// Time delta used for filter update is time delta passed in
		// plus the accumulated time since the packet hasn't has an IMU measurement
		const float total_delta_time= (float)m_state->accumulated_imu_time_delta + delta_time;

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
			// Eqn 42) SEq_new = SEq + SEqDot_est*total_delta_time
			SEq_new = Eigen::Quaternionf(SEq.coeffs() + SEqDot_est.coeffs()*total_delta_time);
		}
		else
		{
			SEq_new = Eigen::Quaternionf(SEq.coeffs() + SEqDot_omega.coeffs()*total_delta_time);
		}

		// Make sure the net quaternion is a pure rotation quaternion
		SEq_new.normalize();

		// Save the new quaternion and first derivative back into the orientation state
		// Derive the second derivative
		{
			const Eigen::Quaternionf &new_orientation = SEq_new;
			const Eigen::Vector3f new_angular_velocity= Eigen::Vector3f::Zero(); // current_omega;
			const Eigen::Vector3f new_angular_acceleration= Eigen::Vector3f::Zero(); // (current_omega - m_state->angular_velocity) / delta_time;

			m_state->apply_imu_state(new_orientation, new_angular_velocity, new_angular_acceleration, delta_time);
		}
	}
	else
	{
		m_state->accumulate_imu_delta_time(delta_time);
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
	if (packet.has_imu_measurements())
	{
		// Time delta used for filter update is time delta passed in
		// plus the accumulated time since the packet hasn't has an IMU measurement
		const float total_delta_time= (float)m_state->accumulated_imu_time_delta + delta_time;

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
		omega_bias = Eigen::Quaternionf(omega_bias.coeffs() + omega_err.coeffs()*zeta*total_delta_time);
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
		Eigen::Quaternionf SEq_new = Eigen::Quaternionf(SEq.coeffs() + SEqDot_est.coeffs()*total_delta_time);

		// Make sure the net quaternion is a pure rotation quaternion
		SEq_new.normalize();

		// Save the new quaternion and first derivative back into the orientation state
		// Derive the second derivative
		{
			const Eigen::Quaternionf &new_orientation = SEq_new;
			const Eigen::Vector3f new_angular_velocity = Eigen::Vector3f::Zero(); //(corrected_omega.x(), corrected_omega.y(), corrected_omega.z());
			const Eigen::Vector3f new_angular_acceleration = Eigen::Vector3f::Zero(); //(new_angular_velocity - m_state->angular_velocity) / delta_time;

			m_state->apply_imu_state(new_orientation, new_angular_velocity, new_angular_acceleration, delta_time);
		}
	}
	else
	{
		m_state->accumulate_imu_delta_time(delta_time);
	}
}

// -- OrientationFilterComplementaryOpticalARG --
#define COMPLEMENTARY_FILTER_YAW_ONLY_BLEND 0
void OrientationFilterComplementaryOpticalARG::update(const float delta_time, const PoseFilterPacket &packet)
{
	// Blend with optical yaw
	if (packet.has_optical_measurement())
    {
		Eigen::Quaternionf new_orientation;

		if (m_state->bIsValid)
		{
			Eigen::Quaternionf optical_orientation= Eigen::Quaternionf::Identity();
			float optical_weight= 0.f;

			if (m_constants.tracking_shape.shape_type == Sphere)
			{
				// Estimate the orientation of the controller 
				// by leveraging the fact that the bulb is offset from the IMU location
				CommonDevicePosition sphere_offset= m_constants.tracking_shape.shape.sphere.center_cm;
				if (getIsStateValid() && 
					(fabsf(sphere_offset.x) > 0 || fabsf(sphere_offset.y) > 0 || fabsf(sphere_offset.z) > 0))
				{
					// TODO: This assumes that the tracking sphere is 
					// aligned along the +Z axis from the IMU

					// Get the basis vectors for the last estimated IMU orientation
					const Eigen::Matrix3f imu_estimated_basis= m_state->orientation.toRotationMatrix();
					const Eigen::Vector3f imu_estimated_XAxis= imu_estimated_basis.col(0);
					const Eigen::Vector3f imu_estimated_YAxis= imu_estimated_basis.col(1);
					const Eigen::Vector3f imu_estimated_ZAxis= imu_estimated_basis.col(2);

					// Compute where the IMU position should be
					// using the last estimated bulb position
					const Eigen::Vector3f imu_estimated_position_cm=
						packet.current_position_cm // last estimated bulb position
						//- imu_estimated_XAxis*sphere_offset.x
						//- imu_estimated_YAxis*sphere_offset.y
						- imu_estimated_ZAxis*sphere_offset.z;

					// Compute an orientation aligned along the vector from the 
					// last estimated IMU position to the current bulb position
					const Eigen::Vector3f optical_ZAxis= packet.optical_position_cm - imu_estimated_position_cm;
					const Eigen::Vector3f optical_YAxis= optical_ZAxis.cross(imu_estimated_XAxis);
					optical_orientation= eigen_quaternion_from_ZY(optical_ZAxis, optical_YAxis);

					// Use the positional variance as the quality measure
					const float optical_variance= 
						m_constants.position_variance_curve.evaluate(packet.tracking_projection_area_px_sqr);
					const float fraction_of_max_orientation_variance =
						safe_divide_with_default(
							optical_variance,
							m_constants.position_variance_curve.MaxValue,
							1.f);
					const float optical_orientation_quality = clampf01(1.f - fraction_of_max_orientation_variance);

					optical_weight= 
						lerp_clampf(0, k_max_optical_orientation_weight, optical_orientation_quality);
				}
			}
			else
			{
				// Use the optical orientation fed into the filter packet
				optical_orientation= packet.optical_orientation;

				// Use the orientation variance as the quality measure
				const float optical_variance= 
					m_constants.orientation_variance_curve.evaluate(packet.tracking_projection_area_px_sqr);
				const float fraction_of_max_orientation_variance =
					safe_divide_with_default(
						optical_variance,
						m_constants.orientation_variance_curve.MaxValue,
						1.f);
				const float optical_orientation_quality = clampf01(1.f - fraction_of_max_orientation_variance);
				
				optical_weight= 
					lerp_clampf(0, k_max_optical_orientation_weight, optical_orientation_quality);
			}
        
			static float g_weight_override= -1.f;
			if (g_weight_override >= 0.f)
			{
				optical_weight= g_weight_override;
			}

			// Blend between the state's orientation and incoming optical orientation
			#if COMPLEMENTARY_FILTER_YAW_ONLY_BLEND
			Eigen::Quaternionf optical_yaw, optical_twist;
			eigen_quaternionf_to_swing_twist(
				packet.optical_orientation, Eigen::Vector3f(0.f, 1.f, 0.f), 
				optical_yaw, optical_twist);
			Eigen::Quaternionf optical_test= optical_yaw * optical_twist;

			Eigen::Quaternionf SEq_new_yaw, SEq_new_twist;
			eigen_quaternionf_to_swing_twist(
				SEq_new, Eigen::Vector3f(0.f, 1.f, 0.f), 
				SEq_new_yaw, SEq_new_twist);
			Eigen::Quaternionf SEq_new_test= SEq_new_yaw * SEq_new_twist;

			Eigen::Quaternionf blended_swing = 
				eigen_quaternion_normalized_lerp(SEq_new_yaw, optical_yaw, optical_weight);

			// Keep the twist from the filtered orientation
			// but use the blended yaw orientation
			new_orientation = blended_swing * SEq_new_twist;
			#else
			new_orientation= 
				eigen_quaternion_normalized_lerp(
					m_state->orientation, packet.optical_orientation, optical_weight);
			#endif
		}
		else
		{
			// Just use the packets optical orientation if the state is uninitialized
			new_orientation= packet.optical_orientation;
		}

		m_state->apply_optical_state(new_orientation, delta_time);
    }
	else
	{
		m_state->accumulate_optical_delta_time(delta_time);
	}

    OrientationFilterMadgwickARG::update(delta_time, packet);
}

// -- OrientationFilterComplementaryMARG --
void OrientationFilterComplementaryMARG::resetState()
{
    OrientationFilter::resetState();
    mg_weight= 1.f;
}

void OrientationFilterComplementaryMARG::update(const float delta_time, const PoseFilterPacket &packet)
{
	if (packet.has_imu_measurements())
	{
		// Time delta used for filter update is time delta passed in
		// plus the accumulated time since the packet hasn't has an IMU measurement
		const float total_delta_time= (float)m_state->accumulated_imu_time_delta + delta_time;

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
		Eigen::Quaternionf q_step = Eigen::Quaternionf(q_derivative.coeffs() * total_delta_time);
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

			m_state->apply_imu_state(new_orientation, new_angular_velocity, new_angular_acceleration, delta_time);
		}

		// Update the blend weight
		// -- Exponential blend the MG weight from 1 down to k_base_earth_frame_align_weight
		mg_weight = lerp_clampf(mg_weight, k_base_earth_frame_align_weight, 0.9f);
	}
	else
	{
		m_state->accumulate_imu_delta_time(delta_time);
	}
}
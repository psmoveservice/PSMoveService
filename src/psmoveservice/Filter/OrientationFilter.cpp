// -- includes -----
#include "OrientationFilter.h"
#include "MathAlignment.h"
#include "../ServerLog.h"
#include <deque>

//-- constants -----
// Calibration Pose transform
const Eigen::Matrix3f g_eigen_identity_pose_upright = Eigen::Matrix3f::Identity();
const Eigen::Matrix3f *k_eigen_identity_pose_upright = &g_eigen_identity_pose_upright;

const Eigen::Matrix3f g_eigen_identity_pose_laying_flat((Eigen::Matrix3f() << 1,0,0, 0,0,-1, 0,1,0).finished());
const Eigen::Matrix3f *k_eigen_identity_pose_laying_flat = &g_eigen_identity_pose_laying_flat;

//Sensor Transforms
const Eigen::Matrix3f g_eigen_sensor_transform_identity = Eigen::Matrix3f::Identity();
const Eigen::Matrix3f *k_eigen_sensor_transform_identity = &g_eigen_sensor_transform_identity;

const Eigen::Matrix3f g_eigen_sensor_transform_opengl((Eigen::Matrix3f() << 1,0,0, 0,0,1, 0,-1,0).finished());
const Eigen::Matrix3f *k_eigen_sensor_transform_opengl= &g_eigen_sensor_transform_opengl;

// Madgwick MARG Filter Constants
#define gyroMeasDrift 3.14159265358979f * (0.9f / 180.0f) // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
#define beta sqrtf(3.0f / 4.0f) * gyroMeasError // compute beta
#define gyroMeasError 3.14159265358979f * (1.5f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define zeta sqrtf(3.0f / 4.0f) * gyroMeasDrift // compute zeta

// Complementary ARG Filter constants
#define k_base_earth_frame_align_weight 0.02f

// Max length of the orientation history we keep
#define k_orientation_history_max 16

// -- private definitions -----
struct OrientationSample
{
    Eigen::Quaternionf orientation;
};

struct MadgwickMARGState
{
    // estimate gyroscope biases error
    Eigen::Quaternionf omega_bias;
};

struct ComplementaryMARGState
{
    float mg_weight;
};

struct OrientationSensorFusionState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Recent history of orientation readings
    std::deque<OrientationSample> orientationHistory;

    /* Output value as quaternion */
    Eigen::Quaternionf orientation;

    /* Quaternion measured when controller points towards camera */
    Eigen::Quaternionf reset_orientation;

    /* Per filter type data */
    struct 
    {
        MadgwickMARGState madgwick_marg_state;
        ComplementaryMARGState complementary_marg_state;
    } fusion_state;

    OrientationFilter::FusionType fusion_type;

    void initialize()
    {
        orientation= Eigen::Quaternionf::Identity();
        reset_orientation= Eigen::Quaternionf::Identity();
        fusion_type= OrientationFilter::FusionTypeComplementaryMARG;
        orientationHistory.clear();
    }
};

// -- globals -----

// -- private methods -----
static void orientation_fusion_imu_update(
    const float delta_time, const OrientationFilterSpace *filter_space, 
    const OrientationFilterPacket *filter_packet, OrientationSensorFusionState *fusion_state);
static void orientation_fusion_complementary_marg_update(
    const float delta_time, const OrientationFilterSpace *filter_space, 
    const OrientationFilterPacket *filter_packet, OrientationSensorFusionState *fusion_state);
static void orientation_fusion_madgwick_marg_update(
    const float delta_time, const OrientationFilterSpace *filter_space, 
    const OrientationFilterPacket *filter_packet, OrientationSensorFusionState *fusion_state);

// -- public interface -----

//-- Orientation Filter Space -----
OrientationFilterSpace::OrientationFilterSpace()
    : m_IdentityGravity(Eigen::Vector3f(0.f, 1.f, 0.f))
    , m_IdentityMagnetometer(Eigen::Vector3f(0.f, -1.f, 0.f))
    , m_CalibrationTransform(Eigen::Matrix3f::Identity())
    , m_SensorTransform(Eigen::Matrix3f::Identity())
{
}

OrientationFilterSpace::OrientationFilterSpace(
    const Eigen::Vector3f &identityGravity,
    const Eigen::Vector3f &identityMagnetometer,
    const Eigen::Matrix3f &calibrationTransform,
    const Eigen::Matrix3f &sensorTransform)
    : m_IdentityGravity(identityGravity)
    , m_IdentityMagnetometer(identityMagnetometer)
    , m_CalibrationTransform(calibrationTransform)
    , m_SensorTransform(sensorTransform)
{
}

Eigen::Vector3f OrientationFilterSpace::getGravityCalibrationDirection() const
{
	// First apply the calibration data transform.
	// This allows us to pretend the "identity pose" was some other orientation the vertical during calibration
    const Eigen::Vector3f calibrationSpaceVector= m_CalibrationTransform * m_IdentityGravity;

	// Next apply the sensor data transform.
	// This allows us to pretend the sensors are in some other coordinate system (like OpenGL where +Y is up)
    const Eigen::Vector3f filterSpaceVector= m_SensorTransform * calibrationSpaceVector;

    return filterSpaceVector;
}

Eigen::Vector3f OrientationFilterSpace::getMagnetometerCalibrationDirection() const
{
	// First apply the calibration data transform.
	// This allows us to pretend the "identity pose" was some other orientation the vertical during calibration
    const Eigen::Vector3f calibrationSpaceVector= m_CalibrationTransform * m_IdentityMagnetometer;

	// Next apply the sensor data transform.
	// This allows us to pretend the sensors are in some other coordinate system (like OpenGL where +Y is up)
    const Eigen::Vector3f filterSpaceVector= m_SensorTransform * calibrationSpaceVector;

    return filterSpaceVector;
}

void OrientationFilterSpace::convertSensorPacketToFilterPacket(
    const OrientationSensorPacket &sensorPacket,
    OrientationFilterPacket &outFilterPacket) const
{
    outFilterPacket.gyroscope= m_SensorTransform * sensorPacket.gyroscope;
    outFilterPacket.normalized_accelerometer= m_SensorTransform * sensorPacket.accelerometer;
    outFilterPacket.normalized_magnetometer= m_SensorTransform * sensorPacket.magnetometer;
        
    eigen_vector3f_normalize_with_default(outFilterPacket.normalized_accelerometer, Eigen::Vector3f());
    eigen_vector3f_normalize_with_default(outFilterPacket.normalized_magnetometer, Eigen::Vector3f());
}

//-- Orientation Filter -----
OrientationFilter::OrientationFilter()
    : m_FilterSpace()
    , m_FusionState(new OrientationSensorFusionState)
{
    m_FusionState->initialize();
}

OrientationFilter::~OrientationFilter()
{
    delete m_FusionState;
}

// Estimate the current orientation of the filter given a time offset
// Positive time values estimate into the future
// Negative time values get pose values from the past
Eigen::Quaternionf OrientationFilter::getOrientation(int msec_time)
{
    //###bwalker $TODO Use the orientation history to compute an orientation

    Eigen::Quaternionf result= m_FusionState->reset_orientation * m_FusionState->orientation;

    return result;
}

void OrientationFilter::setFilterSpace(const OrientationFilterSpace &filterSpace)
{
    m_FilterSpace= filterSpace;
    m_FusionState->initialize();
}

void OrientationFilter::setFusionType(OrientationFilter::FusionType fusionType)
{
    m_FusionState->fusion_type = fusionType;

    switch (m_FusionState->fusion_type)
    {
    case FusionTypeNone:
    case FusionTypeMadgwickIMU:
        // No initialization
        break;
    case FusionTypeMadgwickMARG:
        {
            MadgwickMARGState *marg_state = &m_FusionState->fusion_state.madgwick_marg_state;

            marg_state->omega_bias = *k_eigen_quaternion_zero;
        }
        break;
    case FusionTypeComplementaryMARG:
        {
            ComplementaryMARGState *marg_state = &m_FusionState->fusion_state.complementary_marg_state;

            // Start off fully using the rotation from earth-frame.
            // Then drop down 
            marg_state->mg_weight = 1.f;
        }
        break;
    default:
        break;
    }
}

void OrientationFilter::resetOrientation()
{
    Eigen::Quaternionf q_inverse = m_FusionState->orientation.conjugate();

    eigen_quaternion_normalize_with_default(q_inverse, Eigen::Quaternionf::Identity());
    m_FusionState->reset_orientation= q_inverse;
}

void OrientationFilter::update(
    const float delta_time, 
    const OrientationSensorPacket &sensorPacket)
{
    OrientationFilterPacket filterPacket;
    m_FilterSpace.convertSensorPacketToFilterPacket(sensorPacket, filterPacket);

    Eigen::Quaternionf orientation_backup= m_FusionState->orientation;

    switch(m_FusionState->fusion_type)
    {
    case FusionTypeNone:
        break;
    case FusionTypeMadgwickIMU:
        orientation_fusion_imu_update(delta_time, &m_FilterSpace, &filterPacket, m_FusionState);
        break;
    case FusionTypeMadgwickMARG:
        orientation_fusion_madgwick_marg_update(delta_time, &m_FilterSpace, &filterPacket, m_FusionState);
        break;
    case FusionTypeComplementaryMARG:
        orientation_fusion_complementary_marg_update(delta_time, &m_FilterSpace, &filterPacket, m_FusionState);
        break;
    }

    if (!eigen_quaternion_is_valid(m_FusionState->orientation)) 
    {
        SERVER_LOG_WARNING("OrientationFilter") << "Orientation is NaN!";
        m_FusionState->orientation = orientation_backup;
    }

    // Add the new orientation sample to the orientation history
    {
        OrientationSample sample;

        sample.orientation= m_FusionState->orientation;
        //###bwalker $TODO Timestamp?

        // Make room for new entry if at the max queue size
        if (m_FusionState->orientationHistory.size() >= k_orientation_history_max)
        {
            m_FusionState->orientationHistory.erase(
                m_FusionState->orientationHistory.begin(),
                m_FusionState->orientationHistory.begin() + m_FusionState->orientationHistory.size() - k_orientation_history_max);
        }

        m_FusionState->orientationHistory.push_back(sample);
    }
}

// -- Orientation Filters ----

// This algorithm comes from Sebastian O.H. Madgwick's 2010 paper:
// "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
// https://www.samba.org/tridge/UAV/madgwick_internal_report.pdf
static void 
orientation_fusion_imu_update(
    const float delta_time,
    const OrientationFilterSpace *filter_space,
    const OrientationFilterPacket *filter_packet,
    OrientationSensorFusionState *fusion_state)
{
    const Eigen::Vector3f &current_omega= filter_packet->gyroscope;
    const Eigen::Vector3f &current_g= filter_packet->normalized_accelerometer;

    // Current orientation from earth frame to sensor frame
    const Eigen::Quaternionf SEq = fusion_state->orientation;
    Eigen::Quaternionf SEq_new = SEq;

    // Compute the quaternion derivative measured by gyroscopes
    // Eqn 12) q_dot = 0.5*q*omega
    Eigen::Quaternionf omega = Eigen::Quaternionf(0.f, current_omega.x(), current_omega.y(), current_omega.z());
    Eigen::Quaternionf SEqDot_omega = Eigen::Quaternionf(SEq.coeffs() * 0.5f) *omega;

    if (current_g.isApprox(Eigen::Vector3f::Zero(), k_normal_epsilon))
    {
        // Get the direction of the gravitational fields in the identity pose		
        Eigen::Vector3f k_identity_g_direction = filter_space->getGravityCalibrationDirection();

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

    // Save the new quaternion back into the orientation state
    fusion_state->orientation= SEq_new;
}

// This algorithm comes from Sebastian O.H. Madgwick's 2010 paper:
// "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
// https://www.samba.org/tridge/UAV/madgwick_internal_report.pdf
static void 
orientation_fusion_complementary_marg_update(
    const float delta_time,
    const OrientationFilterSpace *filter_space,
    const OrientationFilterPacket *filter_packet,
    OrientationSensorFusionState *fusion_state)
{
    const Eigen::Vector3f &current_omega= filter_packet->gyroscope;
    const Eigen::Vector3f &current_g= filter_packet->normalized_accelerometer;
    const Eigen::Vector3f &current_m= filter_packet->normalized_magnetometer;

    // If there isn't a valid magnetometer or accelerometer vector, fall back to the IMU style update
    if (current_g.isZero(k_normal_epsilon) || current_m.isZero(k_normal_epsilon))
    {
        orientation_fusion_imu_update(
            delta_time,
            filter_space,
            filter_packet,
            fusion_state);
        return;
    }

    MadgwickMARGState *marg_state = &fusion_state->fusion_state.madgwick_marg_state;

    // Current orientation from earth frame to sensor frame
    const Eigen::Quaternionf SEq = fusion_state->orientation;

    // Get the direction of the magnetic fields in the identity pose.	
    // NOTE: In the original paper we converge on this vector over time automatically (See Eqn 45 & 46)
    // but since we've already done the work in calibration to get this vector, let's just use it.
    // This also removes the last assumption in this function about what 
    // the orientation of the identity-pose is (handled by the sensor transform).
    Eigen::Vector3f k_identity_m_direction = filter_space->getMagnetometerCalibrationDirection();

    // Get the direction of the gravitational fields in the identity pose
    Eigen::Vector3f k_identity_g_direction = filter_space->getGravityCalibrationDirection();

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
    Eigen::Quaternionf omega_bias= marg_state->omega_bias;
    omega_bias = Eigen::Quaternionf(omega_bias.coeffs() + omega_err.coeffs()*zeta*delta_time);
    omega_bias.w() = 0.f; // no bias should accumulate on the w-component
    marg_state->omega_bias= omega_bias;

    // Eqn 49) omega_corrected = omega - net_omega_bias
    Eigen::Quaternionf omega = Eigen::Quaternionf(0.f, current_omega.x(), current_omega.y(), current_omega.z());
    Eigen::Quaternionf corrected_omega = Eigen::Quaternionf(omega.coeffs() - omega_bias.coeffs());

    // Compute the rate of change of the orientation purely from the gyroscope
    // Eqn 12) q_dot = 0.5*q*omega
    Eigen::Quaternionf SEqDot_omega = Eigen::Quaternionf(SEq.coeffs() * 0.5f) * corrected_omega;

    // Compute the estimated quaternion rate of change
    // Eqn 43) SEq_est = SEqDot_omega - beta*SEqHatDot
    Eigen::Quaternionf SEqDot_est = Eigen::Quaternionf(SEqDot_omega.coeffs() - SEqHatDot.coeffs()*beta);

    // Compute then integrate the estimated quaternion rate
    // Eqn 42) SEq_new = SEq + SEqDot_est*delta_t
    Eigen::Quaternionf SEq_new = Eigen::Quaternionf(SEq.coeffs() + SEqDot_est.coeffs()*delta_time);

    // Make sure the net quaternion is a pure rotation quaternion
    SEq_new.normalize();

    // Save the new quaternion back into the orientation state
    fusion_state->orientation= SEq_new;
}

static void 
orientation_fusion_madgwick_marg_update(
    const float delta_time,
    const OrientationFilterSpace *filter_space,
    const OrientationFilterPacket *filter_packet,
    OrientationSensorFusionState *fusion_state)
{
    const Eigen::Vector3f &current_omega= filter_packet->gyroscope;
    const Eigen::Vector3f &current_g= filter_packet->normalized_accelerometer;
    const Eigen::Vector3f &current_m= filter_packet->normalized_magnetometer;

    // Get the direction of the magnetic fields in the identity pose.	
    Eigen::Vector3f k_identity_m_direction = filter_space->getMagnetometerCalibrationDirection();

    // Get the direction of the gravitational fields in the identity pose
    Eigen::Vector3f k_identity_g_direction = filter_space->getGravityCalibrationDirection();

    // Angular Rotation (AR) Update
    //-----------------------------
    // Compute the rate of change of the orientation purely from the gyroscope
    // q_dot = 0.5*q*omega
    Eigen::Quaternionf q_current= fusion_state->orientation;

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
    bool mg_align_success =
        eigen_alignment_quaternion_between_vector_frames(
            mg_from, mg_to, 0.1f, q_current, mg_orientation);

    // Blending Update
    //----------------
    if (mg_align_success)
    {
        // The final rotation is a blend between the integrated orientation and absolute rotation from the earth-frame
        float mg_wight = fusion_state->fusion_state.complementary_marg_state.mg_weight;
        fusion_state->orientation= eigen_quaternion_normalized_lerp(ar_orientation, mg_orientation, mg_wight);

        // Update the blend weight
        fusion_state->fusion_state.complementary_marg_state.mg_weight =
            lerp_clampf(mg_wight, k_base_earth_frame_align_weight, 0.9f);
    }
    else
    {
        fusion_state->orientation= ar_orientation;
    }
}
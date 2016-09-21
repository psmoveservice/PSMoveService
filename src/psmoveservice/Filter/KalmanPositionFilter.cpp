//-- includes --
#include "KalmanPositionFilter.h"
#include "MathAlignment.h"

#include <kalman/MeasurementModel.hpp>
#include <kalman/SystemModel.hpp>
#include <kalman/SquareRootBase.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>

//-- constants --
enum PositionFilterStateEnum
{
    POSITION_X, // meters
    LINEAR_VELOCITY_X, // meters / s
    LINEAR_ACCELERATION_X, // meters /s^2
    POSITION_Y,
    LINEAR_VELOCITY_Y,
    LINEAR_ACCELERATION_Y,
    POSITION_Z,
    LINEAR_VELOCITY_Z,
    LINEAR_ACCELERATION_Z,

    STATE_PARAMETER_COUNT
};

enum PositionFilterMeasurementEnum {
    ACCELEROMETER_X, // gravity units
    ACCELEROMETER_Y,
    ACCELEROMETER_Z,
	OPTICAL_POSITION_X,
	OPTICAL_POSITION_Y,
	OPTICAL_POSITION_Z,

    MEASUREMENT_PARAMETER_COUNT
};

// From: http://nbviewer.jupyter.org/github/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/10-Unscented-Kalman-Filter.ipynb#Reasonable-Choices-for-the-Parameters
// beta=2 is a good choice for Gaussian problems, 
// kappa=3-n where n is the size of x is a good choice for kappa, 
// 0<=alpha<=1 is an appropriate choice for alpha, 
// where a larger value for alpha spreads the sigma points further from the mean.
#define k_ukf_alpha 0.6
#define k_ukf_beta 2.0
#define k_ukf_kappa -6.0 // 3 - STATE_PARAMETER_COUNT

//-- private methods ---
template <class StateType>
void process_3rd_order_noise(const double dT, const double var, const int state_index, Kalman::Covariance<StateType> &Q);

template <class StateType>
void process_2nd_order_noise(const double dT, const double var, const int state_index, Kalman::Covariance<StateType> &Q);

//-- private definitions --
template<typename T>
class ControllerStateVector : public Kalman::Vector<T, STATE_PARAMETER_COUNT>
{
public:
    KALMAN_VECTOR(ControllerStateVector, T, STATE_PARAMETER_COUNT)

    // Accessors
    Eigen::Vector3d get_position() const { 
        return Eigen::Vector3d((*this)[POSITION_X], (*this)[POSITION_Y], (*this)[POSITION_Z]); 
    }
    Eigen::Vector3d get_linear_velocity() const {
        return Eigen::Vector3d((*this)[LINEAR_VELOCITY_X], (*this)[LINEAR_VELOCITY_Y], (*this)[LINEAR_VELOCITY_Z]);
    }
    Eigen::Vector3d get_linear_acceleration() const {
        return Eigen::Vector3d((*this)[LINEAR_ACCELERATION_X], (*this)[LINEAR_ACCELERATION_Y], (*this)[LINEAR_ACCELERATION_Z]);
    }

    // Mutators
    void set_position(const Eigen::Vector3d &p) {
        (*this)[POSITION_X] = p.x(); (*this)[POSITION_Y] = p.y(); (*this)[POSITION_Z] = p.z();
    }
    void set_linear_velocity(const Eigen::Vector3d &v) {
        (*this)[LINEAR_VELOCITY_X] = v.x(); (*this)[LINEAR_VELOCITY_Y] = v.y(); (*this)[LINEAR_VELOCITY_Z] = v.z();
    }
    void set_linear_acceleration(const Eigen::Vector3d &a) {
        (*this)[LINEAR_ACCELERATION_X] = a.x(); (*this)[LINEAR_ACCELERATION_Y] = a.y(); (*this)[LINEAR_ACCELERATION_Z] = a.z();
    }
};
typedef ControllerStateVector<double> ControllerStateVectord;

template<typename T>
class ControllerMeasurementVector : public Kalman::Vector<T, MEASUREMENT_PARAMETER_COUNT>
{
public:
    KALMAN_VECTOR(ControllerMeasurementVector, T, MEASUREMENT_PARAMETER_COUNT)

    // Accessors
    Eigen::Vector3d get_accelerometer() const {
        return Eigen::Vector3d((*this)[ACCELEROMETER_X], (*this)[ACCELEROMETER_Y], (*this)[ACCELEROMETER_Z]);
    }
    Eigen::Vector3d get_optical_position() const {
        return Eigen::Vector3d((*this)[OPTICAL_POSITION_X], (*this)[OPTICAL_POSITION_Y], (*this)[OPTICAL_POSITION_Z]);
    }

    // Mutators
    void set_accelerometer(const Eigen::Vector3d &a) {
        (*this)[ACCELEROMETER_X] = a.x(); (*this)[ACCELEROMETER_Y] = a.y(); (*this)[ACCELEROMETER_Z] = a.z();
    }
    void set_optical_position(const Eigen::Vector3d &p) {
        (*this)[OPTICAL_POSITION_X] = p.x(); (*this)[OPTICAL_POSITION_Y] = p.y(); (*this)[OPTICAL_POSITION_Z] = p.z();
    }
};
typedef ControllerMeasurementVector<double> ControllerMeasurementVectord;

/**
* @brief System model for a controller
*
* This is the system model defining how a controller advances from one
* time-step to the next, i.e. how the system state evolves over time.
*/
class ControllerSystemModel : public Kalman::SystemModel<ControllerStateVectord, Kalman::Vector<double, 0>, Kalman::SquareRootBase>
{
public:
    inline void set_time_step(const double dt) { m_time_step = dt; }

    void init(const PositionFilterConstants &constants)
    {
        const double mean_position_dT= constants.mean_update_time_delta;
        const double mean_orientation_dT= constants.mean_update_time_delta;

        // Start off using the maximum variance values
        const Eigen::Vector3f position_variance= 
			(constants.min_position_variance +
			constants.max_position_variance) * 0.5f;

        // Initialize the process covariance matrix Q
        Kalman::Covariance<ControllerStateVectord> Q = Kalman::Covariance<ControllerStateVectord>::Zero();
        process_3rd_order_noise<ControllerStateVectord>(mean_position_dT, position_variance.x(), POSITION_X, Q);
		process_3rd_order_noise<ControllerStateVectord>(mean_position_dT, position_variance.y(), POSITION_Y, Q);
		process_3rd_order_noise<ControllerStateVectord>(mean_position_dT, position_variance.z(), POSITION_Z, Q);
        setCovariance(Q);
    }

    /**
    * @brief Definition of (non-linear) state transition function
    *
    * This function defines how the system state is propagated through time,
    * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to
    * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
    * the system control input \f$u\f$.
    *
    * @param [in] x The system state in current time-step
    * @param [in] u The control vector input
    * @returns The (predicted) system state in the next time-step
    */
    ControllerStateVectord f(const ControllerStateVectord& old_state, const Kalman::Vector<double, 0>& control) const
    {
        //! Predicted state vector after transition
        ControllerStateVectord new_state;

        // Extract parameters from the old state
        const Eigen::Vector3d old_position = old_state.get_position();
        const Eigen::Vector3d old_linear_velocity = old_state.get_linear_velocity();
        const Eigen::Vector3d old_linear_acceleration = old_state.get_linear_acceleration();

        // Compute the position state update
        const Eigen::Vector3d new_position= 
            old_position 
            + old_linear_velocity*m_time_step 
            + old_linear_acceleration*m_time_step*m_time_step*0.5f;
        const Eigen::Vector3d new_linear_velocity= old_linear_velocity + old_linear_acceleration*m_time_step;
        const Eigen::Vector3d &new_linear_acceleration= old_linear_acceleration;

        // Save results to the new state
        new_state.set_position(new_position);
        new_state.set_linear_velocity(new_linear_velocity);
        new_state.set_linear_acceleration(new_linear_acceleration);

        return new_state;
    }

protected:
    double m_time_step;
};

/**
* @brief Measurement model for measuring PSMove controller
*
* This is the measurement model for measuring the position and magnetometer of the PSMove controller.
* The measurement is given by the optical trackers.
*/
class ControllerMeasurementModel : public Kalman::MeasurementModel<ControllerStateVectord, ControllerMeasurementVectord, Kalman::SquareRootBase>
{
public:
    void init(const PositionFilterConstants &constants)
    {
		update_measurement_covariance(constants, 0.f);
        
		m_identity_gravity_direction= constants.gravity_calibration_direction.cast<double>();
    }

	inline void set_current_orientation(const Eigen::Quaterniond &orientation)
	{
		m_current_orientation= orientation;
	}

	void update_measurement_covariance(
		const PositionFilterConstants &constants,
		const float position_quality)
	{
        // Start off using the maximum variance values
		const Eigen::Vector3f &accelerometer_variance= constants.accelerometer_variance;
		const Eigen::Vector3f position_variance =
			position_quality*constants.min_position_variance +
			(1.f - position_quality)*constants.max_position_variance;

        // Update the measurement covariance R
        Kalman::Covariance<ControllerMeasurementVectord> R = 
			Kalman::Covariance<ControllerMeasurementVectord>::Zero();
		R(ACCELEROMETER_X, ACCELEROMETER_X) = constants.accelerometer_variance.x();
		R(ACCELEROMETER_Y, ACCELEROMETER_Y) = constants.accelerometer_variance.y();
		R(ACCELEROMETER_Z, ACCELEROMETER_Z) = constants.accelerometer_variance.z();
        R(OPTICAL_POSITION_X, OPTICAL_POSITION_X) = position_variance.x();
        R(OPTICAL_POSITION_Y, OPTICAL_POSITION_Y) = position_variance.y();
        R(OPTICAL_POSITION_Z, OPTICAL_POSITION_Z) = position_variance.z();
        setCovariance(R);
	}

    /**
    * @brief Definition of (possibly non-linear) measurement function
    *
    * This function maps the system state to the measurement that is expected
    * to be received from the sensor assuming the system is currently in the
    * estimated state.
    *
    * @param [in] x The system state in current time-step
    * @returns The (predicted) sensor measurement for the system state
    */
    ControllerMeasurementVectord h(const ControllerStateVectord& x) const
    {
        ControllerMeasurementVectord predicted_measurement;

        // Use the position and orientation from the state for predictions
        const Eigen::Vector3d position= x.get_position();

        // Use the current linear acceleration from the state to predict
        // what the accelerometer reading will be (in world space)
        const Eigen::Vector3d gravity_accel_g_units= -m_identity_gravity_direction;
        const Eigen::Vector3d linear_accel_g_units= x.get_linear_acceleration() * k_ms2_to_g_units;
        const Eigen::Vector3d accel_world= linear_accel_g_units + gravity_accel_g_units;
        const Eigen::Quaterniond accel_world_quat(0.f, accel_world.x(), accel_world.y(), accel_world.z());

        // Put the accelerometer prediction into the local space of the controller
        const Eigen::Vector3d accel_local= m_current_orientation*(accel_world_quat*m_current_orientation.conjugate()).vec();

        // Save the predictions into the measurement vector
        predicted_measurement.set_accelerometer(accel_local);
        predicted_measurement.set_optical_position(position);

        return predicted_measurement;
    }

protected:
    Eigen::Vector3d m_identity_gravity_direction;
	Eigen::Quaterniond m_current_orientation;
};

/**
 * @brief Specialized Square Root Unscented Kalman Filter (SR-UKF)
 *
 * @note This is the square-root implementation variant of UnscentedKalmanFilter
 * 
 * This implementation is based upon [The square-root unscented Kalman filter for state and parameter-estimation](http://dx.doi.org/10.1109/ICASSP.2001.940586) by Rudolph van der Merwe and Eric A. Wan.
 * Whenever "the paper" is referenced within this file then this paper is meant.
 *
 * This is a parallel version of Kalman::SquareRootUnscentedKalmanFilter. 
 * Because the state and measurement has a weird rotation type in it we have to do 
 * special work when computing differences, adding deltas, and computing means.
 * 
 * @param StateType The vector-type of the system state (usually some type derived from Kalman::Vector)
 */
namespace Kalman {
	class PositionSRUFK
	{
	public:
		using Control = Vector<double, 0>;

		//! Type of the state vector
		typedef ControllerStateVectord State;

		//! The number of sigma points (depending on state dimensionality)
		static constexpr int SigmaPointCount = 2 * State::RowsAtCompileTime + 1;

        //! Unscented Kalman Filter base type
        typedef UnscentedKalmanFilterBase<ControllerStateVectord> UnscentedBase;
        
		//! Vector containing the sigma scaling weights
		typedef Vector<double, SigmaPointCount> SigmaWeights;

		//! Matrix type containing the sigma state or measurement points
		template<class Type>
		using SigmaPoints = Matrix<double, Type::RowsAtCompileTime, SigmaPointCount>;

		//! Kalman Gain Matrix Type
		template<class Measurement>
		using KalmanGain = Kalman::KalmanGain<State, Measurement>;

    protected:
		//! Estimated state
		State x;

		//! Covariance Square Root
		CovarianceSquareRoot<State> S;

		//! Sigma weights (m)
		SigmaWeights sigmaWeights_m;
		
		//! Sigma weights (c)
		SigmaWeights sigmaWeights_c;

		//! Sigma points (state)
		SigmaPoints<State> sigmaStatePoints;

		// Weight parameters
		double alpha;    //!< Scaling parameter for spread of sigma points (usually \f$ 1E-4 \leq \alpha \leq 1 \f$)
		double beta;     //!< Parameter for prior knowledge about the distribution (\f$ \beta = 2 \f$ is optimal for Gaussian)
		double kappa;    //!< Secondary scaling parameter (usually 0)
		double gamma;    //!< \f$ \gamma = \sqrt{L + \lambda} \f$ with \f$ L \f$ being the state dimensionality
		double lambda;   //!< \f$ \lambda = \alpha^2 ( L + \kappa ) - L\f$ with \f$ L \f$ being the state dimensionality
                     
	public:
		/**
			* Constructor
			* 
			* See paper for detailed parameter explanation
			* 
			* @param alpha Scaling parameter for spread of sigma points (usually \f$ 1E-4 \leq \alpha \leq 1 \f$)
			* @param beta Parameter for prior knowledge about the distribution (\f$ \beta = 2 \f$ is optimal for Gaussian)
			* @param kappa Secondary scaling parameter (usually 0)
			*/
		PositionSRUFK(double _alpha = 1.0, double _beta = 2.0, double _kappa = 0.0)
			: alpha(_alpha)
			, beta(_beta)
			, kappa(_kappa)
		{
			// Setup state and covariance
			x.setZero();

			// Init covariance to identity
			S.setIdentity();

			// Pre-compute all weights
			computeSigmaWeights();
		}

		void init(const State& initialState)
		{
			x = initialState;
		}

		/**
		* @brief Perform filter prediction step using control input \f$u\f$ and corresponding system model
		*
		* @param [in] s The System model
		* @param [in] u The Control input vector
		* @return The updated state estimate
		*/
		const State& predict( const ControllerSystemModel& system_model )
		{
			// No control parameters            
			Control u;
			u.setZero();

			// Compute sigma points
			computeSigmaPoints();

			// Pass each sigma point through non-linear state transition function
			// This covers equation (18) of Algorithm 3.1 in the Paper
			for (int point_index = 0; point_index < SigmaPointCount; ++point_index)
			{
				sigmaStatePoints.col(point_index) = system_model.f(sigmaStatePoints.col(point_index), u);
			}

			// Compute predicted state from predicted sigma points (weighted average of the sigma points)
			x = sigmaStatePoints * sigmaWeights_m;
            
			// Compute the Covariance Square root from sigma points and noise covariance
			// This covers equations (20) and (21) of Algorithm 3.1 in the Paper
			{
				const CovarianceSquareRoot<State> &noiseCov= system_model.getCovarianceSquareRoot();

				// -- Compute QR decomposition of (transposed) augmented matrix
				// Fill in the weighted sigma point portion of the augmented qr input matrix
				// Part of Eqn 19
				Matrix<double, 2*State::RowsAtCompileTime + State::RowsAtCompileTime, State::RowsAtCompileTime> qr_input;
				qr_input.template topRows<2*State::RowsAtCompileTime>() = 
					std::sqrt(this->sigmaWeights_c[1]) 
					* (sigmaStatePoints.template rightCols<SigmaPointCount-1>().colwise() - x).transpose();

				// Fill in the noise portion of the augmented qr input matrix
				// Part of Eqn 19
				qr_input.template bottomRows<State::RowsAtCompileTime>() = noiseCov.matrixU().toDenseMatrix();

				// TODO: Use ColPivHouseholderQR
				Eigen::HouseholderQR<decltype(qr_input)> qr( qr_input );
            
				// Set R matrix as upper triangular square root
				S.setU(qr.matrixQR().template topRightCorner<State::RowsAtCompileTime, State::RowsAtCompileTime>());
            
				// Perform additional rank 1 update
				// TODO: According to the paper (Section 3, "Cholesky factor updating") the update
				//       is defined using the square root of the scalar, however the correct result
				//       is obtained when using the weight directly rather than using the square root
				//       It should be checked whether this is a bug in Eigen or in the Paper
				// T nu = std::copysign( std::sqrt(std::abs(sigmaWeights_c[0])), sigmaWeights_c[0]);
				double nu = this->sigmaWeights_c[0];
				State firstSigmaPoint= sigmaStatePoints.template leftCols<1>();
				S.rankUpdate(firstSigmaPoint - x, nu );
            
				assert(S.info() == Eigen::Success);
			}
            
			// Return predicted state
			return x;
		}

		/**
		 * @brief Perform filter update step using measurement \f$z\f$ and corresponding measurement model
		 *
		 * @param [in] m The Measurement model
		 * @param [in] z The measurement vector
		 * @return The updated state estimate
		 */
		template<class MeasurementModelType, class Measurement>
		const State& update(
			const MeasurementModelType& measurement_model, 
			const Measurement& z )
		{
			SigmaPoints<Measurement> sigmaMeasurementPoints;
            
			// Compute sigma points (using predicted state)
			computeSigmaPoints();
            
			// Predict the expected sigma measurements from predicted sigma states using measurement model
			// This covers equation (23) of Algorithm 3.1 in the Paper
			for (int col_index = 0; col_index < SigmaPointCount; ++col_index)
			{
				sigmaMeasurementPoints.col(col_index) = measurement_model.h(sigmaStatePoints.col(col_index));
			}

			// Predict measurement from sigma measurement points (weighted average of the sigma points)
			Measurement y= sigmaMeasurementPoints * sigmaWeights_m;
           
			// Compute square root innovation covariance
			// This covers equations (23) and (24) of Algorithm 3.1 in the Paper
			CovarianceSquareRoot<Measurement> S_y;
			{
				const CovarianceSquareRoot<Measurement> noiseCov= measurement_model.getCovarianceSquareRoot();

				// -- Compute QR decomposition of (transposed) augmented matrix
				// Fill in the weighted sigma point portion of the augmented qr input matrix
				// Part of Eqn 23
				Matrix<double, 2*State::RowsAtCompileTime + Measurement::RowsAtCompileTime, Measurement::RowsAtCompileTime> qr_input;
				qr_input.template topRows<2*State::RowsAtCompileTime>() = 
					std::sqrt(this->sigmaWeights_c[1]) 
					* (sigmaMeasurementPoints.template rightCols<SigmaPointCount-1>().colwise() - y).transpose();

				// Fill in the noise portion of the augmented qr input matrix
				// Part of Eqn 23 
				qr_input.template bottomRows<Measurement::RowsAtCompileTime>() = noiseCov.matrixU().toDenseMatrix();

				// TODO: Use ColPivHouseholderQR
				Eigen::HouseholderQR<decltype(qr_input)> qr( qr_input );
	            
				// Set R matrix as upper triangular square root
				S_y.setU(qr.matrixQR().template topRightCorner<Measurement::RowsAtCompileTime, Measurement::RowsAtCompileTime>());
	            
				// Perform additional rank 1 update
				// TODO: According to the paper (Section 3, "Cholesky factor updating") the update
				//       is defined using the square root of the scalar, however the correct result
				//       is obtained when using the weight directly rather than using the square root
				//       It should be checked whether this is a bug in Eigen or in the Paper
				// T nu = std::copysign( std::sqrt(std::abs(sigmaWeights_c[0])), sigmaWeights_c[0]);
				double nu = this->sigmaWeights_c[0];
				Measurement firstSigmaMeasurementPoint= sigmaMeasurementPoints.template leftCols<1>();
				S_y.rankUpdate(firstSigmaMeasurementPoint - y, nu );
	            
				assert(S_y.info() == Eigen::Success);
			}
            		    
			// Compute the Kalman Gain from predicted measurement and sigma points and the innovation covariance.
			// 
			// This covers equations (27) and (28) of Algorithm 3.1 in the Paper
			//
			// We need to solve the equation \f$ K (S_y S_y^T) = P \f$ for \f$ K \f$ using backsubstitution.
			// In order to apply standard backsubstitution using the `solve` method we must bring the
			// equation into the form
			// \f[ AX = B \qquad \text{with } A = S_yS_y^T \f]
			// Thus we transpose the whole equation to obtain
			// \f{align*}{
			//   ( K (S_yS_y^T))^T &= P^T \Leftrightarrow \\
			//   (S_yS_y^T)^T K^T &= P^T \Leftrightarrow \\
			//   (S_yS_y^T) K^T &= P^T
			//\f}
			// Hence our \f$ X := K^T\f$ and \f$ B:= P^T \f$
			KalmanGain<Measurement> K;
			{
				// Note: The intermediate eval() is needed here (for now) due to a bug in Eigen that occurs
				// when Measurement::RowsAtCompileTime == 1 AND State::RowsAtCompileTime >= 8
				decltype(sigmaStatePoints) W = this->sigmaWeights_c.transpose().template replicate<State::RowsAtCompileTime,1>();
				Matrix<double, State::RowsAtCompileTime, Measurement::RowsAtCompileTime> P
						= (sigmaStatePoints.colwise() - x).cwiseProduct( W ).eval()
						* (sigmaMeasurementPoints.colwise() - y).transpose();
            
				K = S_y.solve(P.transpose()).transpose();
			}
            
			// Update state
			x += K * ( z - y );
            
			// Update the state covariance matrix using the Kalman Gain and the Innovation Covariance
			// This covers equations (29) and (30) of Algorithm 3.1 in the Paper
			{
				KalmanGain<Measurement> U = K * S_y.matrixL();

				for(int i = 0; i < U.cols(); ++i)
				{
					S.rankUpdate( U.col(i), -1 );
                
					assert( S.info() != Eigen::NumericalIssue );
				}            
			}
            
			return x;
		}

	protected:
		/**
		* @brief Compute sigma weights
		*/
		void computeSigmaWeights()
		{
			const double L = static_cast<double>(State::RowsAtCompileTime);

			lambda = alpha * alpha * (L + kappa) - L;
			gamma = sqrt(L + lambda);

			// Make sure L != -lambda to avoid division by zero
			assert(fabs(L + lambda) > 1e-6);

			// Make sure L != -kappa to avoid division by zero
			assert(fabs(L + kappa) > 1e-6);

			double W_m_0 = lambda / (L + lambda);
			double W_c_0 = W_m_0 + (1.0 - alpha*alpha + beta);
			double W_i = 1.0 / (2.0 * alpha*alpha * (L + kappa));

			// Make sure W_i > 0 to avoid square-root of negative number
			assert(W_i > 0.0);

			sigmaWeights_m[0] = W_m_0;
			sigmaWeights_c[0] = W_c_0;

			for (int point_index = 1; point_index < SigmaPointCount; ++point_index)
			{
				sigmaWeights_m[point_index] = W_i;
				sigmaWeights_c[point_index] = W_i;
			}
		}

		/**
		* @brief Compute sigma points from current state estimate and state covariance
		*
		* @note This covers equations (17) and (22) of Algorithm 3.1 in the Paper
		*/
		bool computeSigmaPoints()
		{
			// Get square root of covariance
			Matrix<double, State::RowsAtCompileTime, State::RowsAtCompileTime> _S = S.matrixL().toDenseMatrix();

			// Fill in the first column with the state vector 'x'
			sigmaStatePoints.template leftCols<1>() = x;
            // Set center block with x + gamma * S
            sigmaStatePoints.template block<State::RowsAtCompileTime, State::RowsAtCompileTime>(0,1)
                    = ( this->gamma * _S).colwise() + x;
            // Set right block with x - gamma * S
            sigmaStatePoints.template rightCols<State::RowsAtCompileTime>()
                    = (-this->gamma * _S).colwise() + x;

			return true;
		}
	};
}

class KalmanPositionFilterImpl
{
public:
    /// Is the current fusion state valid
    bool bIsValid;

    /// True if we have seen a valid position measurement (>0 position quality)
    bool bSeenPositionMeasurement;

    /// Position that's considered the origin position 
    Eigen::Vector3f origin_position; // meters

    /// All state parameters of the controller
    ControllerStateVectord state_vector;

    /// Used to model how the physics of the controller evolves
    ControllerSystemModel system_model;

	/// Used to project model onto predicted sensor measurements
	ControllerMeasurementModel measurement_model;

    /// Unscented Kalman Filter instance
	Kalman::PositionSRUFK ukf;

    KalmanPositionFilterImpl() 
		: ukf(k_ukf_alpha, k_ukf_beta, k_ukf_kappa)
    {
    }

    virtual void init(const PositionFilterConstants &constants)
    {
        bIsValid = false;
		bSeenPositionMeasurement= false;

        origin_position = Eigen::Vector3f::Zero();

        state_vector.setZero();

		measurement_model.init(constants);
        system_model.init(constants);
        ukf.init(state_vector);
    }
};

//-- public interface --
//-- KalmanFilterOpticalPoseARG --
KalmanPositionFilter::KalmanPositionFilter() 
    : m_filter(nullptr)
{
    memset(&m_constants, 0, sizeof(PositionFilterConstants));
}

KalmanPositionFilter::~KalmanPositionFilter()
{
    if (m_filter != nullptr)
    {
        delete m_filter;
        m_filter;
    }
}

bool KalmanPositionFilter::init(const PositionFilterConstants &constants)
{
    m_constants = constants;

    // cleanup any existing filter
    if (m_filter != nullptr)
    {
        delete m_filter;
        m_filter;
    }

	// Create and initialize the private filter implementation
    KalmanPositionFilterImpl *filter = new KalmanPositionFilterImpl();
    filter->init(constants);
    m_filter = filter;

    return true;
}

void KalmanPositionFilter::update(const float delta_time, const PoseFilterPacket &packet)
{
    if (m_filter->bIsValid)
    {
        // Predict state for current time-step using the filters
        m_filter->system_model.set_time_step(delta_time);
        m_filter->state_vector = m_filter->ukf.predict(m_filter->system_model);

        // Get the measurement model for the DS4 from the derived filter impl
        ControllerMeasurementModel &measurement_model = m_filter->measurement_model;

		// Update the measurement model with the latest controller orientation
		measurement_model.set_current_orientation(packet.current_orientation.cast<double>());

        // Project the current state onto a predicted measurement as a default
        // in case no observation is available
        ControllerMeasurementVectord measurement = measurement_model.h(m_filter->state_vector);

        // Accelerometer and gyroscope measurements are always available
        measurement.set_accelerometer(packet.imu_accelerometer.cast<double>());

        if (packet.optical_position_quality > 0.f)
        {
			Eigen::Vector3f optical_position= packet.get_optical_position_in_meters();

			// Adjust the amount we trust the optical measurements based on the quality parameters
            measurement_model.update_measurement_covariance(
				m_constants, 
				packet.optical_position_quality);

            // State internally stores position in meters
            measurement.set_optical_position(optical_position.cast<double>());

			// If this is the first time we have seen the position, snap the position state
			if (!m_filter->bSeenPositionMeasurement)
			{
				m_filter->state_vector.set_position(optical_position.cast<double>());
				m_filter->bSeenPositionMeasurement= true;
			}
        }

        // Update UKF
        m_filter->state_vector = m_filter->ukf.update(measurement_model, measurement);
    }
    else
    {
        m_filter->state_vector.setZero();

		if (packet.optical_position_quality > 0.f)
		{
			Eigen::Vector3f optical_position= packet.get_optical_position_in_meters();

			m_filter->state_vector.set_position(optical_position.cast<double>());
			m_filter->bSeenPositionMeasurement= true;
		}
		else
		{
			m_filter->state_vector.set_position(Eigen::Vector3d::Zero());
		}

        m_filter->bIsValid= true;
    }
}

bool KalmanPositionFilter::getIsStateValid() const
{
    return m_filter->bIsValid;
}

void KalmanPositionFilter::resetState()
{
    m_filter->init(m_constants);
}

void KalmanPositionFilter::recenterState()
{
    m_filter->origin_position = getPosition();
}

Eigen::Vector3f KalmanPositionFilter::getPosition(float time) const
{
    Eigen::Vector3f result = Eigen::Vector3f::Zero();

    if (m_filter->bIsValid)
    {
        Eigen::Vector3f state_position= m_filter->state_vector.get_position().cast<float>();
        Eigen::Vector3f predicted_position =
            is_nearly_zero(time)
            ? state_position
            : state_position + getVelocity() * time;

        result = (predicted_position - m_filter->origin_position) * k_meters_to_centimeters;
    }

    return result;
}

Eigen::Vector3f KalmanPositionFilter::getVelocity() const
{
	Eigen::Vector3d vel= m_filter->state_vector.get_linear_velocity() * k_meters_to_centimeters;

    return vel.cast<float>();
}

Eigen::Vector3f KalmanPositionFilter::getAcceleration() const
{
    Eigen::Vector3d accel= m_filter->state_vector.get_linear_acceleration() * k_meters_to_centimeters;

	return accel.cast<float>();
}

//-- Private functions --
template <class StateType>
void process_3rd_order_noise(
    const double dT,
    const double var,
    const int state_index,
    Kalman::Covariance<StateType> &Q)
{
    const double dT_2 = dT*dT;
	const double dT_3 = dT_2*dT;
	const double dT_4 = dT_2*dT_2;
	const double dT_5 = dT_3*dT_2;
	const double dT_6 = dT_3*dT_3;
	const double dT_7 = dT_4*dT_3;

    const double q7 = var * dT_7;
    const double q6 = var * dT_6;
    const double q5 = var * dT_5;
    const double q4 = var * dT_4;
    const double q3 = var * dT_3;

    const int &i= state_index;
    Q(i+0,i+0) = q7/252.0; Q(i+0,i+1) = q6/72.0; Q(i+0,i+2) = q5/30.0;
    Q(i+1,i+0) = q6/72.0;  Q(i+1,i+1) = q5/20.0; Q(i+1,i+2) = q4/8.0;
    Q(i+2,i+0) = q5/30.0;  Q(i+2,i+1) = q4/8.0;  Q(i+2,i+2) = q3/3.0;
}

template <class StateType>
void process_2nd_order_noise(
	const double dT, 
	const double var, 
	const int state_index, 
	Kalman::Covariance<StateType> &Q)
{
    const double dT_2 = dT*dT;
	const double dT_3 = dT_2*dT;
	const double dT_4 = dT_2*dT_2;
	const double dT_5 = dT_3*dT_2;

    const double q5 = var * dT_5;
    const double q4 = var * dT_4;
    const double q3 = var * dT_3;

    // Q = [.5dt^2, dt]*[.5dt^2, dt]^T * variance
    const int &i= state_index;
    Q(i+0,i+0) = q5/20.0; Q(i+0,i+1) = q4/8.0;
    Q(i+1,i+0) = q4/8.0;  Q(i+1,i+1) = q3/3.0;
}
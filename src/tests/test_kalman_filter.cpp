#include "KalmanPoseFilter.h"
#include "MathAlignment.h"

#if defined(__linux) || defined (__APPLE__)
#include <unistd.h>
#endif

#include <stdio.h>
#include <vector>

enum eControllerType
{
	Unknown,
	PSMove,
	DualShock4
};


enum eControllerSampleFields
{
	FIELD_ACCELEROMETER_X,
	FIELD_ACCELEROMETER_Y,
	FIELD_ACCELEROMETER_Z,
	FIELD_GYROSCOPE_X,
	FIELD_GYROSCOPE_Y,
	FIELD_GYROSCOPE_Z,
	FIELD_MAGNETOMETER_X,
	FIELD_MAGNETOMETER_Y,
	FIELD_MAGNETOMETER_Z,
	FIELD_POSITION_X,
	FIELD_POSITION_Y,
	FIELD_POSITION_Z,
	FIELD_ORIENTATION_W,
	FIELD_ORIENTATION_X,
	FIELD_ORIENTATION_Y,
	FIELD_ORIENTATION_Z,
	FIELD_TIME,

	FIELD_COUNT
};

const char *szColumnNames[FIELD_COUNT] = {
	"ACC_X",
	"ACC_Y",
	"ACC_Z",
	"GYRO_X",
	"GYRO_Y",
	"GYRO_Z",
	"MAG_X",
	"MAG_Y",
	"MAG_Z",
	"POS_X",
	"POS_Y",
	"POS_Z",
	"ORI_W",
	"ORI_X",
	"ORI_Y",
	"ORI_Z",
	"TIME"
};

struct ControllerSample
{
	// Sensor readings in the controller's reference frame
	float acc[3]; // g-units
	float gyro[3]; // rad/s
	float mag[3]; // unit vector

	// Optical readings in the world reference frame
	float pos[3]; // cm
	float ori[4];

	float time; // seconds
};
static_assert(sizeof(ControllerSample) == sizeof(float)*FIELD_COUNT, "incorrect field count");

class ControllerInputStream
{
public:
	ControllerInputStream(const char *filename)
		: m_sampleIndex(0)
		, m_controllerType(Unknown)
	{
		char line[512];
		float columns[FIELD_COUNT];

		FILE *fp = fopen(filename, "rt");
		if (fp != nullptr)
		{
			bool bSuccess = true;

			line[sizeof(line) - 1] = 0;
			m_controllerType = PSMove;
			//if (fgets(line, sizeof(line) - 1, fp))
			//{				
			//	if (strnicmp(line, "psmove", 6) == 0)
			//	{
			//		m_controllerType = PSMove;
			//		bSuccess = true;
			//	}
			//	else if (strnicmp(line, "dualshock4", 10) == 0)
			//	{
			//		m_controllerType = DualShock4;
			//		bSuccess = true;
			//	}
			//}

			//if (bSuccess)
			//{
			//	bSuccess = false;

			//	if (fgets(line, sizeof(line) - 1, fp) != nullptr)
			//	{
			//		size_t len = strlen(line);

			//		if (len > 0)
			//		{
			//			const char* last_start = &line[0];
			//			int valid_columns = 0;

			//			size_t cursor= 0;
			//			while (cursor < len && valid_columns < FIELD_COUNT)
			//			{
			//				if (line[cursor] == ',' || line[cursor] == '\n')
			//				{
			//					line[cursor] = '\0';
			//					if (strnicmp(last_start, szColumnNames[valid_columns], strlen(szColumnNames[valid_columns])) == 0)
			//					{
			//						cursor++;
			//						valid_columns++;
			//						last_start = &line[cursor];
			//					}
			//					else
			//					{
			//						break;
			//					}
			//				}

			//				cursor++;
			//			}

			//			if (valid_columns == FIELD_COUNT)
			//			{
			//				bSuccess = true;
			//			}
			//		}
			//	}
			//}

			if (bSuccess)
			{
				while (fgets(line, sizeof(line) - 1, fp) != nullptr)
				{
					size_t len = strlen(line);

					if (len > 0)
					{
						const char* last_start = &line[0];
						int valid_columns = 0;

						size_t cursor= 0;
						while (cursor < len && valid_columns < FIELD_COUNT)
						{
							if (line[cursor] == ',' || line[cursor] == '\n')
							{
								line[cursor] = '\0';
								columns[valid_columns] = static_cast<float>(atof(last_start));

								cursor++;
								valid_columns++;
								last_start = &line[cursor];
							}

							cursor++;
						}

						if (valid_columns == FIELD_COUNT)
						{
							ControllerSample sample;

							memcpy(&sample, columns, sizeof(float)*FIELD_COUNT);

							// Convert the samples in meters to centimeters
							sample.pos[0] *= k_centimeters_to_meters;
							sample.pos[1] *= k_centimeters_to_meters;
							sample.pos[2] *= k_centimeters_to_meters;

							// Normalize the magnetometer readings
							float mag_scale = sqrtf(
								sample.mag[0] * sample.mag[0] +
								sample.mag[1] * sample.mag[1] +
								sample.mag[2] * sample.mag[2]);
							sample.mag[0] /= mag_scale;
							sample.mag[1] /= mag_scale;
							sample.mag[2] /= mag_scale;

							// PSMoveService default orientation is with the controller vertical, bulb
							// to the sky, with the trigger to the camera.However, asking for the
							// rotation from PSMoveState.Pose.Orientation uses the bulb facing the
							// camera as the default orientation.We will use the provided orientations
							// for testing, so let's undo their rotations first.
							if (m_controllerType == PSMove)
							{
								Eigen::Quaternionf artificial_rotation(Eigen::AngleAxisf(-k_real_half_pi, Eigen::Vector3f(1.f, 0.f, 0.f)));
								Eigen::Quaternionf original_quat(sample.ori[0], sample.ori[1], sample.ori[2], sample.ori[3]);
								Eigen::Quaternionf rotated_quat= (original_quat * artificial_rotation).normalized();

								sample.ori[0] = rotated_quat.w();
								sample.ori[1] = rotated_quat.x();
								sample.ori[2] = rotated_quat.y();
								sample.ori[3] = rotated_quat.z();
							}

							m_samples.push_back(sample);
						}
					}
				}
			}

			fclose(fp);
		}
	}

	eControllerType getControllerType() const
	{
		return m_controllerType;
	}

	bool hasNext() const
	{
		return m_sampleIndex < m_samples.size();
	}

	const ControllerSample &next()
	{
		const ControllerSample &sample = m_samples.at(m_sampleIndex);
		++m_sampleIndex;

		return sample;
	}

	const ControllerSample &getSample(size_t index) const {
		return m_samples.at(index);
	}

	Eigen::Vector3f computeCovarianceSlice(const int field_index) const
	{
		assert(field_index >= FIELD_ACCELEROMETER_X && field_index <= FIELD_POSITION_X);
		assert(field_index % 3 == 0);

		std::vector<Eigen::Vector3f> sample_vectors;
		for (const ControllerSample &sample : m_samples)
		{
			const float *raw_sample = reinterpret_cast<const float *>(&sample);
			Eigen::Vector3f vector_sample(raw_sample[field_index], raw_sample[field_index + 1], raw_sample[field_index + 2]);

			sample_vectors.push_back(vector_sample);
		}

		Eigen::Vector3f mean, variance;
		eigen_vector3f_compute_mean_and_variance(
			sample_vectors.data(),
			static_cast<int>(sample_vectors.size()),
			&mean,
			&variance);

		return variance;
	}

	float computeMeanTimeDelta() const
	{
		float previous_time = -1.f;
		float mean_dt = 0.f;

		for (const ControllerSample &sample : m_samples)
		{
			if (previous_time >= 0.f)
			{
				float dt = sample.time - previous_time;

				mean_dt += dt;
			}

			previous_time = sample.time;
		}

		mean_dt /= static_cast<float>(m_samples.size() - 1);

		return mean_dt;
	}

private:
	std::vector<ControllerSample> m_samples;
	size_t m_sampleIndex;
	eControllerType m_controllerType;
};

class FilterOutputStream
{
public:
	FilterOutputStream(const char *filename)
	{
		m_fp = fopen(filename, "wt");

		if (m_fp != nullptr)
		{
			fprintf(m_fp, "TIME, POS_X, POS_Y, POS_Z, VEL_X, VEL_Y, VEL_Z, ACC_X, ACC_Y, ACC_Z, ORI_W, ORI_X, ORI_Y, ORI_Z, AVEL_X, AVEL_Y, AVEL_Z\n");
		}
	}

	~FilterOutputStream()
	{
		if (m_fp != nullptr)
		{
			fclose(m_fp);
		}
	}

	void writeFilterState(IPoseFilter *pose_filter, float time)
	{
		if (m_fp != nullptr)
		{
			Eigen::Vector3f pos= pose_filter->getPosition();
			Eigen::Vector3f vel = pose_filter->getVelocity();
			Eigen::Vector3f acc = pose_filter->getAcceleration();
			Eigen::Quaternionf quat = pose_filter->getOrientation();
			Eigen::Vector3f ang_vel = pose_filter->getAngularVelocity();

			fprintf(m_fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
				time,
				pos.x(), pos.y(), pos.z(),
				vel.x(), vel.y(), vel.z(),
				acc.x(), acc.y(), acc.z(),
				quat.w(), quat.x(), quat.y(), quat.z(),
				ang_vel.x(), ang_vel.y(), ang_vel.z());
		}
	}

private:
	FILE* m_fp;
};

static void init_filter_for_psdualshock4(
	const ControllerInputStream &stationary_stream,
	const Eigen::Vector3f &initial_position, const Eigen::Quaternionf &initial_orientation,
	PoseFilterSpace **out_pose_filter_space, IPoseFilter **out_pose_filter);
static void init_filter_for_psmove(
	const ControllerInputStream &stationary_stream,
	const Eigen::Vector3f &initial_position, const Eigen::Quaternionf &initial_orientation,
	PoseFilterSpace **out_pose_filter_space, IPoseFilter **out_pose_filter);

int main(int argc, char *argv[])
{   
	if (argc < 4)
	{
		printf("usage test_kalman_filter <stationary_file.csv> <movement_file.csv> <output_file.csv>");
		return -1;
	}

	ControllerInputStream stationary_stream(argv[1]);
	ControllerInputStream movement_stream(argv[1]);
	FilterOutputStream output_stream(argv[2]);

	PoseFilterSpace *pose_filter_space= nullptr;
	IPoseFilter *pose_filter = nullptr;

	const ControllerSample &initialSample = movement_stream.getSample(0);
	Eigen::Vector3f initial_pos(initialSample.pos[0], initialSample.pos[1], initialSample.pos[2]);
	Eigen::Quaternionf initial_ori(initialSample.ori[0], initialSample.ori[1], initialSample.ori[2], initialSample.ori[3]);

	switch (movement_stream.getControllerType())
	{
	case PSMove:
		init_filter_for_psmove(
			stationary_stream,
			initial_pos, initial_ori, 
			&pose_filter_space, &pose_filter);
		break;
	case DualShock4:
		init_filter_for_psdualshock4(
			stationary_stream,
			initial_pos, initial_ori, 
			&pose_filter_space, &pose_filter);
		break;
	default:
		break;
	}

	float lastTime= movement_stream.getSample(0).time - stationary_stream.computeMeanTimeDelta();
	while (movement_stream.hasNext())
	{
		ControllerSample sample= movement_stream.next();
		float dT = sample.time - lastTime;

		PoseSensorPacket sensorPacket;
		sensorPacket.imu_accelerometer = Eigen::Vector3f(sample.acc[0], sample.acc[1], sample.acc[2]);
		sensorPacket.imu_gyroscope = Eigen::Vector3f(sample.gyro[0], sample.gyro[1], sample.gyro[2]);
		sensorPacket.imu_magnetometer = Eigen::Vector3f(sample.mag[0], sample.mag[1], sample.mag[2]);
		sensorPacket.optical_orientation = Eigen::Quaternionf(sample.ori[0], sample.ori[1], sample.ori[2], sample.ori[3]);
		sensorPacket.optical_orientation_quality = 1.f; // sample.ori_qual;
		sensorPacket.optical_position_cm = Eigen::Vector3f(sample.pos[0], sample.pos[1], sample.pos[2]);
		sensorPacket.optical_position_quality = 1.f; // sample.pos_qual;

		PoseFilterPacket filterPacket;
		pose_filter_space->createFilterPacket(sensorPacket, pose_filter->getOrientation(), pose_filter->getPosition(), filterPacket);

		pose_filter->update(dT, filterPacket);

		output_stream.writeFilterState(pose_filter, sample.time);
	}

	if (pose_filter_space != nullptr)
	{
		delete pose_filter_space;
	}
	if (pose_filter != nullptr)
	{
		delete pose_filter;
	}

	return 0;
}

static void
init_filter_for_psmove(
	const ControllerInputStream &stationary_stream,
	const Eigen::Vector3f &initial_position,
	const Eigen::Quaternionf &initial_orientation,
	PoseFilterSpace **out_pose_filter_space,
	IPoseFilter **out_pose_filter)
{
	// Setup the space the orientation filter operates in
	PoseFilterSpace *pose_filter_space = new PoseFilterSpace();
	pose_filter_space->setIdentityGravity(Eigen::Vector3f(0.f, 0.f, -1.f));
	pose_filter_space->setIdentityMagnetometer(Eigen::Vector3f(0.737549126f, 0.675293505f, 1));
	pose_filter_space->setCalibrationTransform(*k_eigen_identity_pose_upright);
	pose_filter_space->setSensorTransform(*k_eigen_sensor_transform_identity);

	// Copy the pose filter constants from the controller config
	PoseFilterConstants constants;

	constants.orientation_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
	constants.orientation_constants.magnetometer_calibration_direction = pose_filter_space->getMagnetometerCalibrationDirection();
	constants.orientation_constants.gyro_drift = Eigen::Vector3f::Zero();
	constants.orientation_constants.gyro_variance = stationary_stream.computeCovarianceSlice(FIELD_GYROSCOPE_X);
	constants.orientation_constants.mean_update_time_delta = stationary_stream.computeMeanTimeDelta();
	constants.orientation_constants.min_orientation_variance = 1.0f; // from matlab
	constants.orientation_constants.max_orientation_variance = 1.0f; // from matlab
	constants.orientation_constants.min_orientation_drift = 0.0f; // from matlab
	constants.orientation_constants.max_orientation_drift = 0.0f; // from matlab
	constants.orientation_constants.magnetometer_drift = Eigen::Vector3f::Zero();
	constants.orientation_constants.magnetometer_variance = stationary_stream.computeCovarianceSlice(FIELD_MAGNETOMETER_X);

	constants.position_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
	constants.position_constants.accelerometer_drift = Eigen::Vector3f(0.0133424997f, 0.0107941628f, 0.0543990135f);
	constants.position_constants.accelerometer_variance = stationary_stream.computeCovarianceSlice(FIELD_ACCELEROMETER_X);
	constants.position_constants.accelerometer_noise_radius = 0.0139137721;
	constants.position_constants.max_velocity = 1.0f;
	constants.position_constants.mean_update_time_delta = stationary_stream.computeMeanTimeDelta();
	constants.position_constants.min_position_variance =
		constants.position_constants.max_position_variance =
			stationary_stream.computeCovarianceSlice(FIELD_POSITION_X);
	constants.position_constants.min_position_drift =
		constants.position_constants.max_position_drift =
			Eigen::Vector3f::Zero();

	KalmanPoseFilterPSMove *kalmanFilter = new KalmanPoseFilterPSMove();
	kalmanFilter->init(constants, initial_position, initial_orientation);

	*out_pose_filter_space = pose_filter_space;
	*out_pose_filter = kalmanFilter;
}

static void
init_filter_for_psdualshock4(
	const ControllerInputStream &stationary_stream,
	const Eigen::Vector3f &initial_position,
	const Eigen::Quaternionf &initial_orientation,
	PoseFilterSpace **out_pose_filter_space,
	IPoseFilter **out_pose_filter)
{
	// Setup the space the orientation filter operates in
	PoseFilterSpace *pose_filter_space = new PoseFilterSpace();
	pose_filter_space->setIdentityGravity(Eigen::Vector3f(0.f, 0.922760189f, -0.385374635f));
	pose_filter_space->setIdentityMagnetometer(Eigen::Vector3f::Zero());  // No magnetometer on DS4 :(
	pose_filter_space->setCalibrationTransform(*k_eigen_identity_pose_upright);
	pose_filter_space->setSensorTransform(*k_eigen_sensor_transform_identity);

	// Copy the pose filter constants from the controller config
	PoseFilterConstants constants;
	constants.orientation_constants.mean_update_time_delta = stationary_stream.computeMeanTimeDelta();
	constants.orientation_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
	constants.orientation_constants.magnetometer_calibration_direction = pose_filter_space->getMagnetometerCalibrationDirection();
	constants.orientation_constants.magnetometer_drift = Eigen::Vector3f::Zero(); // no magnetometer on ds4
	constants.orientation_constants.magnetometer_variance = Eigen::Vector3f::Zero(); // no magnetometer on ds4
	constants.orientation_constants.gyro_drift = Eigen::Vector3f(0.000705962884f, 0.000705962884f, 0.000705962884f);
	constants.orientation_constants.gyro_variance = stationary_stream.computeCovarianceSlice(FIELD_GYROSCOPE_X);
	// min variance at max screen area
	constants.orientation_constants.min_orientation_variance = 0.005f;
	constants.orientation_constants.min_orientation_drift = 0.f;
	// max variance at min screen area
	constants.orientation_constants.max_orientation_variance = 0.005f;
	constants.orientation_constants.max_orientation_drift = 0.f;

	constants.position_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
	constants.position_constants.accelerometer_drift = Eigen::Vector3f::Zero();
	constants.position_constants.accelerometer_variance = stationary_stream.computeCovarianceSlice(FIELD_ACCELEROMETER_X);
	constants.position_constants.accelerometer_noise_radius = 0.0148137454f;
	constants.position_constants.max_velocity = 1.f;
	constants.position_constants.mean_update_time_delta = stationary_stream.computeMeanTimeDelta();
	constants.position_constants.min_position_variance =
		constants.position_constants.max_position_variance =
			stationary_stream.computeCovarianceSlice(FIELD_POSITION_X);
	constants.position_constants.min_position_drift =
		constants.position_constants.max_position_drift = 
			Eigen::Vector3f::Zero();

	KalmanPoseFilterDS4 *kalmanFilter= new KalmanPoseFilterDS4();
	kalmanFilter->init(constants, initial_position, initial_orientation);

	*out_pose_filter_space = pose_filter_space;
	*out_pose_filter = kalmanFilter;
}
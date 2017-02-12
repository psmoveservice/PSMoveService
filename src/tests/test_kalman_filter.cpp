#include "DeviceInterface.h"
#include "KalmanPoseFilter.h"
#include "CompoundPoseFilter.h"
#include "MathAlignment.h"

#if defined(__linux) || defined (__APPLE__)
#include <unistd.h>
#endif

#include <stdio.h>
#include <vector>

#if _MSC_VER
#define strncasecmp(a, b, n) _strnicmp(a,b,n)
#endif

enum eControllerSampleFields
{
	FIELD_TIME,
	FIELD_POSITION_X,
	FIELD_POSITION_Y,
	FIELD_POSITION_Z,
	FIELD_AREA,
	FIELD_ORIENTATION_W,
	FIELD_ORIENTATION_X,
	FIELD_ORIENTATION_Y,
	FIELD_ORIENTATION_Z,
	FIELD_ACCELEROMETER_X,
	FIELD_ACCELEROMETER_Y,
	FIELD_ACCELEROMETER_Z,
	FIELD_MAGNETOMETER_X,
	FIELD_MAGNETOMETER_Y,
	FIELD_MAGNETOMETER_Z,
	FIELD_GYROSCOPE_X,
	FIELD_GYROSCOPE_Y,
	FIELD_GYROSCOPE_Z,

	FIELD_COUNT
};

const char *szColumnNames[FIELD_COUNT] = {
	"TIME",
	"POS_X",
	"POS_Y",
	"POS_Z",
	"AREA",
	"ORI_W",
	"ORI_X",
	"ORI_Y",
	"ORI_Z",
	"ACC_X",
	"ACC_Y",
	"ACC_Z",
	"MAG_X",
	"MAG_Y",
	"MAG_Z",
	"GYRO_X",
	"GYRO_Y",
	"GYRO_Z"
};

struct ControllerSample
{
	float time; // seconds

	// Optical readings in the world reference frame
	float pos[3]; // cm
	float area;
	float ori[4];

	// Sensor readings in the controller's reference frame
	float acc[3]; // g-units
	float mag[3]; // unit vector
	float gyro[3]; // rad/s
};
static_assert(sizeof(ControllerSample) == sizeof(float)*FIELD_COUNT, "incorrect field count");

class ControllerInputStream
{
public:
	ControllerInputStream(const char *filename)
		: m_sampleIndex(0)
		, m_controllerType(CommonDeviceState::PSMove)
	{
		char line[512];
		float columns[FIELD_COUNT];

		FILE *fp = fopen(filename, "rt");
		if (fp != nullptr)
		{
			bool bSuccess = true;

			line[sizeof(line) - 1] = 0;
			m_controllerType = CommonDeviceState::PSMove;
			if (fgets(line, sizeof(line) - 1, fp))
			{				
				if (strncasecmp(line, "psmove", 6) == 0)
				{
					m_controllerType = CommonDeviceState::PSMove;
					bSuccess = true;
				}
				else if (strncasecmp(line, "dualshock4", 10) == 0)
				{
					m_controllerType = CommonDeviceState::PSDualShock4;
					bSuccess = true;
				}
			}

			if (bSuccess)
			{
				bSuccess = false;

				if (fgets(line, sizeof(line) - 1, fp) != nullptr)
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
								if (strncasecmp(last_start, szColumnNames[valid_columns], strlen(szColumnNames[valid_columns])) == 0)
								{
									cursor++;
									valid_columns++;
									last_start = &line[cursor];
								}
								else
								{
									break;
								}
							}

							cursor++;
						}

						if (valid_columns == FIELD_COUNT)
						{
							bSuccess = true;
						}
					}
				}
			}

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

							// Convert the samples in centimeters to meters
							sample.pos[0] *= k_centimeters_to_meters;
							sample.pos[1] *= k_centimeters_to_meters;
							sample.pos[2] *= k_centimeters_to_meters;

							// Normalize the magnetometer readings
							float mag_scale = sqrtf(
								sample.mag[0] * sample.mag[0] +
								sample.mag[1] * sample.mag[1] +
								sample.mag[2] * sample.mag[2]);
							if (mag_scale > k_real_epsilon)
							{
								sample.mag[0] /= mag_scale;
								sample.mag[1] /= mag_scale;
								sample.mag[2] /= mag_scale;
							}

							// PSMoveService default orientation is with the controller vertical, bulb
							// to the sky, with the trigger to the camera.However, asking for the
							// rotation from PSMoveState.Pose.Orientation uses the bulb facing the
							// camera as the default orientation.We will use the provided orientations
							// for testing, so let's undo their rotations first.
							if (m_controllerType == CommonDeviceState::PSMove)
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

	CommonDeviceState::eDeviceType getControllerType() const
	{
		return m_controllerType;
	}

	size_t getSampleCount() const
	{
		return m_samples.size();
	}

	void reset()
	{
		m_sampleIndex = 0;
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

	void computeSliceStatistics(
		const int field_index,
		Eigen::Vector3f *out_mean,
		Eigen::Vector3f *out_variance) const
	{
		assert(field_index == FIELD_ACCELEROMETER_X || field_index == FIELD_MAGNETOMETER_X ||
			field_index == FIELD_GYROSCOPE_X || field_index == FIELD_POSITION_X);

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

		if (out_mean)
		{
			*out_mean = mean;
		}

		if (out_variance)
		{
			*out_variance = variance;
		}
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
	CommonDeviceState::eDeviceType m_controllerType;
};

class FilterOutputStream
{
public:
	FilterOutputStream(const char *filename_prefix, const char *filename_suffix)
	{
		std::string filename = filename_prefix;
		filename.append(filename_suffix);

		m_fp = fopen(filename.c_str(), "wt");

		if (m_fp != nullptr)
		{
			fprintf(m_fp, "TIME, RAW_POS_X, POS_X, RAW_POS_Y, POS_Y, RAW_POS_Z, POS_Z, VEL_X, VEL_Y, VEL_Z, ACC_X, ACC_Y, ACC_Z, RAW_ORI_P, ORI_P, RAW_ORI_Y, ORI_Y, RAW_ORI_R, ORI_R, AVEL_X, AVEL_Y, AVEL_Z\n");
		}
	}

	~FilterOutputStream()
	{
		if (m_fp != nullptr)
		{
			fclose(m_fp);
		}
	}

	void writeFilterState(ControllerSample &sample, IPoseFilter *pose_filter, float time)
	{
		if (m_fp != nullptr)
		{
			Eigen::Quaternionf raw_quat = Eigen::Quaternionf(sample.ori[0], sample.ori[1], sample.ori[2], sample.ori[3]);
			Eigen::EulerAnglesf raw_euler_angles= eigen_quaternionf_to_euler_angles(raw_quat);

			Eigen::Vector3f pos= pose_filter->getPositionCm();
			Eigen::Vector3f vel = pose_filter->getVelocityCmPerSec();
			Eigen::Vector3f acc = pose_filter->getAccelerationCmPerSecSqr();
			Eigen::Quaternionf quat = pose_filter->getOrientation();
			Eigen::EulerAnglesf angles = eigen_quaternionf_to_euler_angles(quat);
			Eigen::Vector3f ang_vel = pose_filter->getAngularVelocityRadPerSec();

			fprintf(m_fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
				time,
				sample.pos[0], pos.x(), sample.pos[1], pos.y(), sample.pos[2], pos.z(),
				vel.x(), vel.y(), vel.z(),
				acc.x(), acc.y(), acc.z(),
				raw_euler_angles.get_attitude_degrees(), angles.get_attitude_degrees(), 
				raw_euler_angles.get_heading_degrees(), angles.get_heading_degrees(), 
				raw_euler_angles.get_bank_degrees(), angles.get_bank_degrees(),
				ang_vel.x(), ang_vel.y(), ang_vel.z());
		}
	}

private:
	FILE* m_fp;
};

static void apply_filter(
	const bool bUseCompoundFilter,
	ControllerInputStream &stationary_stream,
	ControllerInputStream &movement_stream,
	FilterOutputStream &output_stream);
static void init_filter_for_psdualshock4(
	const ControllerInputStream &stationary_stream,
	const Eigen::Vector3f &initial_position, const Eigen::Quaternionf &initial_orientation,
	const bool bUseCompoundFilter,
	PoseFilterSpace **out_pose_filter_space, IPoseFilter **out_pose_filter);
static void init_filter_for_psmove(
	const ControllerInputStream &stationary_stream,
	const Eigen::Vector3f &initial_position, const Eigen::Quaternionf &initial_orientation,
	const bool bUseCompoundFilter,
	PoseFilterSpace **out_pose_filter_space, IPoseFilter **out_pose_filter);

int main(int argc, char *argv[])
{   
	if (argc < 4)
	{
		printf("usage test_kalman_filter <stationary_file.csv> <movement_file.csv> <output_file.csv>");
		return -1;
	}

	ControllerInputStream stationary_stream(argv[1]);
	if (stationary_stream.getSampleCount() <= 1)
	{
		printf("Stationary file: %s, doesn't contain more than one sample", argv[1]);
		return -1;
	}

	ControllerInputStream movement_stream(argv[2]);
	if (movement_stream.getSampleCount() <= 1)
	{
		printf("Movement file: %s, doesn't contain more than one sample", argv[2]);
		return -1;
	}

	FilterOutputStream compoundfilter_output_stream("compoundfilter_", argv[3]);
	apply_filter(
		true, // use compound orientation kalman + position kalman filter
		stationary_stream,
		movement_stream,
		compoundfilter_output_stream);

	//###HipsterSloth $TODO full pose kalman filter doesn't work yet
	//FilterOutputStream posefilter_output_stream("posefilter_", argv[3]);
	//apply_filter(
	//	false, // use full pose kalman filter
	//	stationary_stream,
	//	movement_stream,
	//	posefilter_output_stream);

	return 0;
}

static void
apply_filter(
	const bool bUseCompoundFilter,
	ControllerInputStream &stationary_stream,
	ControllerInputStream &movement_stream,
	FilterOutputStream &output_stream)
{
	PoseFilterSpace *pose_filter_space = nullptr;
	IPoseFilter *pose_filter = nullptr;

	const ControllerSample &initialSample = movement_stream.getSample(0);
	Eigen::Vector3f initial_pos(initialSample.pos[0], initialSample.pos[1], initialSample.pos[2]);
	Eigen::Quaternionf initial_ori(initialSample.ori[0], initialSample.ori[1], initialSample.ori[2], initialSample.ori[3]);

	switch (movement_stream.getControllerType())
	{
	case CommonDeviceState::PSMove:
		init_filter_for_psmove(
			stationary_stream,
			initial_pos, initial_ori,
			bUseCompoundFilter,
			&pose_filter_space, &pose_filter);
		break;
	case CommonDeviceState::PSDualShock4:
		init_filter_for_psdualshock4(
			stationary_stream,
			initial_pos, initial_ori,
			bUseCompoundFilter,
			&pose_filter_space, &pose_filter);
		break;
	default:
		break;
	}

	float lastTime = movement_stream.getSample(0).time - stationary_stream.computeMeanTimeDelta();

	movement_stream.reset();
	while (movement_stream.hasNext())
	{
		ControllerSample sample = movement_stream.next();
		float dT = sample.time - lastTime;

		PoseSensorPacket sensorPacket;
		sensorPacket.imu_accelerometer_g_units = Eigen::Vector3f(sample.acc[0], sample.acc[1], sample.acc[2]);
		sensorPacket.imu_gyroscope_rad_per_sec = Eigen::Vector3f(sample.gyro[0], sample.gyro[1], sample.gyro[2]);
		sensorPacket.imu_magnetometer_unit = Eigen::Vector3f(sample.mag[0], sample.mag[1], sample.mag[2]);
		sensorPacket.optical_orientation = Eigen::Quaternionf(sample.ori[0], sample.ori[1], sample.ori[2], sample.ori[3]);
		sensorPacket.tracking_projection_area_px_sqr = sample.area;
		sensorPacket.optical_position_cm = Eigen::Vector3f(sample.pos[0], sample.pos[1], sample.pos[2]);

		PoseFilterPacket filterPacket;
		pose_filter_space->createFilterPacket(sensorPacket, pose_filter, filterPacket);

		pose_filter->update(dT, filterPacket);

		output_stream.writeFilterState(sample, pose_filter, sample.time);
	}

	if (pose_filter_space != nullptr)
	{
		delete pose_filter_space;
	}
	if (pose_filter != nullptr)
	{
		delete pose_filter;
	}
}

static void
init_filter_for_psmove(
	const ControllerInputStream &stationary_stream,
	const Eigen::Vector3f &initial_position,
	const Eigen::Quaternionf &initial_orientation,
	const bool bUseCompoundFilter,
	PoseFilterSpace **out_pose_filter_space,
	IPoseFilter **out_pose_filter)
{
	// Setup the space the orientation filter operates in
	PoseFilterSpace *pose_filter_space = new PoseFilterSpace();
	pose_filter_space->setIdentityGravity(Eigen::Vector3f(0.f, 0.f, -1.f));
	pose_filter_space->setIdentityMagnetometer(Eigen::Vector3f(0.234017432f, 0.873125494f, 0.42765367f));
	pose_filter_space->setCalibrationTransform(*k_eigen_identity_pose_upright);
	pose_filter_space->setSensorTransform(*k_eigen_sensor_transform_identity);

	// Copy the pose filter constants from the controller config
	PoseFilterConstants constants;
	constants.clear();

	constants.orientation_constants.mean_update_time_delta = stationary_stream.computeMeanTimeDelta();
	constants.orientation_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
	constants.orientation_constants.magnetometer_calibration_direction = pose_filter_space->getMagnetometerCalibrationDirection();
	stationary_stream.computeSliceStatistics(
		FIELD_GYROSCOPE_X,
		&constants.orientation_constants.gyro_drift,
		&constants.orientation_constants.gyro_variance);
	constants.orientation_constants.magnetometer_drift = Eigen::Vector3f::Zero();
	stationary_stream.computeSliceStatistics(
		FIELD_MAGNETOMETER_X,
		nullptr,
		&constants.orientation_constants.magnetometer_variance);
	constants.orientation_constants.orientation_variance_curve.A = 0.0f;
	constants.orientation_constants.orientation_variance_curve.B = 0.0f;
	constants.orientation_constants.orientation_variance_curve.MaxValue = 0.0f;

	Eigen::Vector3f accelerometer_drift;
	stationary_stream.computeSliceStatistics(
		FIELD_ACCELEROMETER_X,
		&accelerometer_drift,
		&constants.position_constants.accelerometer_variance);
	constants.position_constants.accelerometer_drift =
		accelerometer_drift - Eigen::Vector3f(0.f, 1.f, 0.f);
	constants.position_constants.accelerometer_noise_radius = 0.0139137721f;
	constants.position_constants.max_velocity = 1.0f;

	Eigen::Vector3f position_variance;
	stationary_stream.computeSliceStatistics(
		FIELD_POSITION_X,
		nullptr, 
		&position_variance);
	constants.position_constants.position_variance_curve.A = 0.44888f;
	constants.position_constants.position_variance_curve.B = -0.00402f;
	constants.position_constants.position_variance_curve.MaxValue = 1.0f;
	constants.position_constants.mean_update_time_delta = stationary_stream.computeMeanTimeDelta();
	constants.position_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();

	if (bUseCompoundFilter)
	{
		CompoundPoseFilter *compoundFilter = new CompoundPoseFilter();
		compoundFilter->init(
			CommonDeviceState::PSMove, 
			OrientationFilterTypeKalman, PositionFilterTypeKalman, 
			constants,
			initial_position, initial_orientation);

		*out_pose_filter = compoundFilter;
	}
	else
	{
		KalmanPoseFilterPSMove *fullPoseFilter = new KalmanPoseFilterPSMove();
		fullPoseFilter->init(constants, initial_position, initial_orientation);

		*out_pose_filter = fullPoseFilter;
	}

	*out_pose_filter_space = pose_filter_space;
}

static void
init_filter_for_psdualshock4(
	const ControllerInputStream &stationary_stream,
	const Eigen::Vector3f &initial_position,
	const Eigen::Quaternionf &initial_orientation,
	const bool bUseCompoundFilter,
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
	constants.clear();

	constants.orientation_constants.mean_update_time_delta = stationary_stream.computeMeanTimeDelta();
	constants.orientation_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
	constants.orientation_constants.magnetometer_calibration_direction = pose_filter_space->getMagnetometerCalibrationDirection();
	constants.orientation_constants.magnetometer_drift = Eigen::Vector3f::Zero(); // no magnetometer on ds4
	constants.orientation_constants.magnetometer_variance = Eigen::Vector3f::Zero(); // no magnetometer on ds4
	stationary_stream.computeSliceStatistics(
		FIELD_GYROSCOPE_X,
		&constants.orientation_constants.gyro_drift,
		&constants.orientation_constants.gyro_variance);
	constants.orientation_constants.orientation_variance_curve.A = 0.44888f;
	constants.orientation_constants.orientation_variance_curve.B = -0.00402f;
	constants.orientation_constants.orientation_variance_curve.MaxValue = 1.0f;

	Eigen::Vector3f accelerometer_drift;
	stationary_stream.computeSliceStatistics(
		FIELD_ACCELEROMETER_X,
		&accelerometer_drift,
		&constants.position_constants.accelerometer_variance);
	constants.position_constants.accelerometer_drift =
		accelerometer_drift - Eigen::Vector3f(0.f, 1.f, 0.f);
	constants.position_constants.accelerometer_noise_radius = 0.0148137454f;
	constants.position_constants.max_velocity = 1.f;
	constants.position_constants.mean_update_time_delta = stationary_stream.computeMeanTimeDelta();
	constants.position_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();

	Eigen::Vector3f position_variance;
	stationary_stream.computeSliceStatistics(
		FIELD_POSITION_X,
		nullptr,
		&position_variance);
	constants.position_constants.position_variance_curve.A = 0.44888f;
	constants.position_constants.position_variance_curve.B = -0.00402f;
	constants.position_constants.position_variance_curve.MaxValue = 1.0f;

	if (bUseCompoundFilter)
	{
		CompoundPoseFilter *compoundFilter = new CompoundPoseFilter();
		compoundFilter->init(
			CommonDeviceState::PSDualShock4,
			OrientationFilterTypeKalman, PositionFilterTypeKalman,
			constants,
			initial_position, initial_orientation);

		*out_pose_filter = compoundFilter;
	}
	else
	{
		KalmanPoseFilterPSMove *fullPoseFilter = new KalmanPoseFilterPSMove();
		fullPoseFilter->init(constants, initial_position, initial_orientation);

		*out_pose_filter = fullPoseFilter;
	}

	*out_pose_filter_space = pose_filter_space;
}

#include "KalmanPoseFilter.h"

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
	FIELD_TIME,
	FIELD_POSITION_X,
	FIELD_POSITION_Y,
	FIELD_POSITION_Z,
	FIELD_POSITION_QUALITY,
	FIELD_ORIENTATION_W,
	FIELD_ORIENTATION_X,
	FIELD_ORIENTATION_Y,
	FIELD_ORIENTATION_Z,
	FIELD_ORIENTATION_QUALITY,
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
	"POS_QUAL",
	"ORI_W",
	"ORI_X",
	"ORI_Y",
	"ORI_Z",
	"ORI_QUAL",
	"ACC_X",
	"ACC_Y",
	"ACC_Z",
	"MAG_X",
	"MAG_Y",
	"MAG_Z",
	"GYRO_X",
	"GYRO_Y",
	"GYRO_Z",
};

struct ControllerSample
{
	float time; // seconds

	// Optical readings in the world reference frame
	float pos[3]; // cm
	float pos_qual; // [0, 1]

	float ori[4];
	float ori_qual; // [0, 1]

	// Sensor readings in the controller's reference frame
	float acc[3]; // g-units
	float mag[3]; // unit vector
	float gyro[3]; // rad/s
};
static_assert(sizeof(ControllerSample) == sizeof(float)*FIELD_COUNT, "incorrect field count");

class ControllerInputStream
{
public:
	static const int COLUMN_COUNT = 18;

	ControllerInputStream(const char *filename)
		: m_sampleIndex(0)
		, m_controllerType(Unknown)
	{
		char line[512];
		float columns[COLUMN_COUNT];

		FILE *fp = fopen(filename, "rt");
		if (fp != nullptr)
		{
			bool bSuccess = false;

			line[sizeof(line) - 1] = 0;
			if (fgets(line, sizeof(line) - 1, fp))
			{				
				if (stricmp(line, "psmove") == 0)
				{
					m_controllerType = PSMove;
					bSuccess = true;
				}
				else if (stricmp(line, "dualshock4") == 0)
				{
					m_controllerType = DualShock4;
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

						size_t cursor;
						while (cursor < len && valid_columns < COLUMN_COUNT)
						{
							if (line[cursor] == ',')
							{
								line[cursor] = '\0';
								if (stricmp(last_start, szColumnNames[valid_columns]) == 0)
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

						if (valid_columns == COLUMN_COUNT)
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

						size_t cursor;
						while (cursor < len && valid_columns < COLUMN_COUNT)
						{
							if (line[cursor] == ',')
							{
								line[cursor] = '\0';
								columns[valid_columns] = static_cast<float>(atof(last_start));

								cursor++;
								valid_columns++;
								last_start = &line[cursor];
							}

							cursor++;
						}

						if (valid_columns == COLUMN_COUNT)
						{
							ControllerSample sample;

							memcpy(&sample, columns, sizeof(float)*COLUMN_COUNT);
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
		return m_samples.at(m_sampleIndex);
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
			fprintf(m_fp, "TIME, POS_X, POS_Y, POS_Z, VEL_X, VEL_Y, VEL_Z, ACC_X, ACC_Y, ACC_Z, ORI_W, ORI_X, ORI_Y, ORI_Z, AVEL_X, AVEL_Y, AVEL_Z");
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

			fprintf(m_fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f",
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

static void init_filter_for_psdualshock4(PoseFilterSpace **out_pose_filter_space, IPoseFilter **out_pose_filter);
static void init_filter_for_psmove(PoseFilterSpace **out_pose_filter_space, IPoseFilter **out_pose_filter);

int main(int argc, char *argv[])
{   
	if (argc < 3)
	{
		printf("usage test_kalman_filter <input_file.csv> <output_file.csv>");
		return -1;
	}

	ControllerInputStream input_stream(argv[1]);
	FilterOutputStream output_stream(argv[2]);

	PoseFilterSpace *pose_filter_space= nullptr;
	IPoseFilter *pose_filter = nullptr;

	switch (input_stream.getControllerType())
	{
	case PSMove:
		init_filter_for_psmove(&pose_filter_space, &pose_filter);
		break;
	case DualShock4:
		init_filter_for_psdualshock4(&pose_filter_space, &pose_filter);
		break;
	default:
		break;
	}

	float lastTime= 0.f;
	while (input_stream.hasNext())
	{
		ControllerSample sample= input_stream.next();
		float dT = sample.time - lastTime;

		PoseSensorPacket sensorPacket;
		sensorPacket.imu_accelerometer = Eigen::Vector3f(sample.acc[0], sample.acc[1], sample.acc[2]);
		sensorPacket.imu_gyroscope = Eigen::Vector3f(sample.gyro[0], sample.gyro[1], sample.gyro[2]);
		sensorPacket.imu_magnetometer = Eigen::Vector3f(sample.mag[0], sample.mag[1], sample.mag[2]);
		sensorPacket.optical_orientation = Eigen::Quaternionf(sample.ori[0], sample.ori[1], sample.ori[2], sample.ori[3]);
		sensorPacket.optical_orientation_quality = sample.ori_qual;
		sensorPacket.optical_position_cm = Eigen::Vector3f(sample.pos[0], sample.pos[1], sample.pos[2]);
		sensorPacket.optical_position_quality = sample.pos_qual;

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
	PoseFilterSpace **out_pose_filter_space,
	IPoseFilter **out_pose_filter)
{
	// Setup the space the orientation filter operates in
	PoseFilterSpace *pose_filter_space = new PoseFilterSpace();
	pose_filter_space->setIdentityGravity(Eigen::Vector3f(0.f, 1.f, 0.f));
	pose_filter_space->setIdentityMagnetometer(Eigen::Vector3f(-0.364768296f, 0.930912316f, -0.0186111741f));
	pose_filter_space->setCalibrationTransform(*k_eigen_identity_pose_laying_flat);
	pose_filter_space->setSensorTransform(*k_eigen_sensor_transform_opengl);

	// Copy the pose filter constants from the controller config
	PoseFilterConstants constants;

	constants.orientation_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
	constants.orientation_constants.magnetometer_calibration_direction = pose_filter_space->getMagnetometerCalibrationDirection();
	constants.orientation_constants.gyro_drift = 0.0272777844f;
	constants.orientation_constants.gyro_variance = 0.000348962029f;
	constants.orientation_constants.mean_update_time_delta = 0.00833300035f;
	constants.orientation_constants.min_orientation_variance = 0.00499999989f;
	constants.orientation_constants.max_orientation_variance = 0.00499999989f;
	constants.orientation_constants.magnetometer_variance = 0.000590000011f;

	constants.position_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
	constants.position_constants.accelerometer_variance = 7.1999998e-06f;
	constants.position_constants.accelerometer_noise_radius = 0.0139137721;
	constants.position_constants.max_velocity = 1.0f;
	constants.position_constants.mean_update_time_delta = 0.00833300035f;
	// min variance at max screen area
	constants.position_constants.min_position_variance = 0.25f;
	// max variance at min screen area
	constants.position_constants.max_position_variance = 0.25f;

	KalmanPoseFilterPSMove *kalmanFilter = new KalmanPoseFilterPSMove();
	kalmanFilter->init(constants);

	*out_pose_filter_space = pose_filter_space;
	*out_pose_filter = kalmanFilter;
}

static void
init_filter_for_psdualshock4(
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
	constants.orientation_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
	constants.orientation_constants.magnetometer_calibration_direction = pose_filter_space->getMagnetometerCalibrationDirection();
	constants.orientation_constants.gyro_drift = 0.000705962884f;
	constants.orientation_constants.mean_update_time_delta = 0.0166669991f;
	constants.orientation_constants.magnetometer_variance = 0.f; // no magnetometer on ds4
	constants.orientation_constants.gyro_variance = 4.72827696e-06f;
	// min variance at max screen area
	constants.orientation_constants.min_orientation_variance = 0.005f;
	// max variance at min screen area
	constants.orientation_constants.max_orientation_variance = 0.005f;

	constants.position_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
	constants.position_constants.accelerometer_variance = 1.72511263e-05f;
	constants.position_constants.accelerometer_noise_radius = 0.0148137454f;
	constants.position_constants.max_velocity = 1.f;
	constants.position_constants.mean_update_time_delta = 0.0166669991;
	// min variance at max screen area
	constants.position_constants.min_position_variance = 0.25f;
	// max variance at min screen area
	constants.position_constants.max_position_variance = 0.25f;

	KalmanPoseFilterDS4 *kalmanFilter= new KalmanPoseFilterDS4();
	kalmanFilter->init(constants);

	*out_pose_filter_space = pose_filter_space;
	*out_pose_filter = kalmanFilter;
}
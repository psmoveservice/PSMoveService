//-- includes -----
#include "AppSubStage_CalibrateWithMat.h"
#include "AppStage_ComputeTrackerPoses.h"
#include "AppStage_TrackerSettings.h"
#include "App.h"
#include "AssetManager.h"
#include "Camera.h"
#include "GeometryUtility.h"
#include "Logger.h"
#include "MathAlignment.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "MathGLM.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"

#include "SDL_keycode.h"
#include "SDL_opengl.h"

#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <imgui.h>
#include <vector>

//#define COMPUTE_OPTICAL_VARIANCES

//-- constants -----
// Sample 5 points - The psmove standing on the 4 corners and the center of a sheet of paper
static const int k_mat_sample_location_count = 5;

// Take 60 samples at each location
static const int k_mat_calibration_sample_count = 60;

static const glm::vec3 k_psmove_frustum_color = glm::vec3(0.1f, 0.7f, 0.3f);

static const double k_stabilize_wait_time_ms = 1000.f;

static const float k_height_to_psmove_bulb_center = 17.7f; // cm - measured base to bulb center distance
static const float k_sample_x_location_offset = 14.f; // cm - Half the length of a 8.5'x11' sheet of paper
static const float k_sample_z_location_offset = 10.75f; // cm - Half the length of a 8.5'x11' sheet of paper

static const PSMovePosition k_sample_3d_locations[k_mat_sample_location_count] = {
    { k_sample_x_location_offset, k_height_to_psmove_bulb_center, k_sample_z_location_offset },
    { -k_sample_x_location_offset, k_height_to_psmove_bulb_center, k_sample_z_location_offset },
    { 0.f, k_height_to_psmove_bulb_center, 0.f },
    { -k_sample_x_location_offset, k_height_to_psmove_bulb_center, -k_sample_z_location_offset },
    { k_sample_x_location_offset, k_height_to_psmove_bulb_center, -k_sample_z_location_offset }
};
static const char *k_sample_location_names[k_mat_sample_location_count] = {
    "+X+Z Corner",
    "-X+Z Corner",
    "Center",
    "-X-Z Corner",
    "+X-Z Corner"
};

//-- private definitions -----
struct TrackerRelativePoseStatistics
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// N samples for the current sampling location
	PSMoveScreenLocation screenSpacePoints[k_mat_calibration_sample_count];
	float screenSpaceArea[k_mat_calibration_sample_count];
	Eigen::Vector3f trackerSpacePoints[k_mat_calibration_sample_count];
	Eigen::Vector3f trackerSpaceOrientations[k_mat_calibration_sample_count]; // angle-axis
	int sampleCount;

	// Average for each sampling location
	PSMoveScreenLocation avgScreenSpacePointAtLocation[k_mat_sample_location_count];
	float avgScreenSpaceArea[k_mat_sample_location_count];
	Eigen::Vector3f avgTrackerSpacePointAtLocation[k_mat_sample_location_count];
	float trackerSpacePointMaxVarianceAtLocation[k_mat_sample_location_count];
	float trackerSpaceOrientationMaxVarianceAtLocation[k_mat_sample_location_count];

	// Best fit lines for position and orientation variance 
	// as a linear function of tracking projection pixel area 
	Eigen::Vector2f positionVarianceLine;
	Eigen::Vector2f orientationVarianceLine;

	PSMovePose trackerPose;
	float reprojectionError;
	bool bValidTrackerPose;

	TrackerRelativePoseStatistics()
	{
		clearAll();
	}

	bool getIsComplete() const
	{
		return sampleCount >= k_mat_calibration_sample_count;
	}

	void clearLastSampleBatch()
	{
		memset(screenSpacePoints, 0, sizeof(PSMoveScreenLocation)*k_mat_calibration_sample_count);
		memset(screenSpaceArea, 0, sizeof(float)*k_mat_calibration_sample_count);
		memset(trackerSpacePoints, 0, sizeof(Eigen::Vector3f)*k_mat_calibration_sample_count);
		memset(trackerSpaceOrientations, 0, sizeof(Eigen::Vector3f)*k_mat_calibration_sample_count);
		sampleCount = 0;
	}

	void clearAll()
	{
		clearLastSampleBatch();

		memset(avgScreenSpacePointAtLocation, 0, sizeof(PSMoveScreenLocation)*k_mat_sample_location_count);
		memset(avgScreenSpaceArea, 0, sizeof(float)*k_mat_sample_location_count);
		memset(avgTrackerSpacePointAtLocation, 0, sizeof(Eigen::Vector3f)*k_mat_sample_location_count);
		memset(trackerSpacePointMaxVarianceAtLocation, 0, sizeof(float)*k_mat_sample_location_count);
		memset(trackerSpaceOrientationMaxVarianceAtLocation, 0, sizeof(float)*k_mat_sample_location_count);

		trackerPose = *k_psmove_pose_identity;
		reprojectionError = 0.f;
		bValidTrackerPose = false;
	}

	void addSample(const ClientTrackerView *trackerView, const ClientControllerView *controllerView, const int sampleLocationIndex)
	{
		const PSMoveRawTrackerData &trackerData= controllerView->GetRawTrackerData();
		const int trackerID= trackerView->getTrackerId();

		PSMoveScreenLocation screenSample;
		PSMoveTrackingProjection screenProjection;
		PSMovePosition trackerRelativePosition;
		PSMoveQuaternion trackerRelativeOrientation;
		
		if (!getIsComplete() &&
			trackerData.GetPixelLocationOnTrackerId(trackerID, screenSample) &&
			trackerData.GetProjectionOnTrackerId(trackerID, screenProjection) &&
			trackerData.GetPositionOnTrackerId(trackerID, trackerRelativePosition))
		{
			Eigen::Vector3f trackerRelativeScaledAngleAxis = Eigen::Vector3f::Zero();

			// Orientation isn't available on the psmove
			if (trackerData.GetOrientationOnTrackerId(trackerID, trackerRelativeOrientation))
			{
				Eigen::Quaternionf trackerRelativeQuaternion = psmove_quaternion_to_eigen_quaternionf(trackerRelativeOrientation);
				Eigen::AngleAxisf trackerRelativeAngleAxis(trackerRelativeQuaternion);
				
				trackerRelativeScaledAngleAxis = trackerRelativeAngleAxis.axis() * trackerRelativeAngleAxis.angle();
			}

			screenSpacePoints[sampleCount] = screenSample;
			screenSpaceArea[sampleCount] = screenProjection.get_projection_area();
			trackerSpacePoints[sampleCount] = psmove_float_vector3_to_eigen_vector3(trackerRelativePosition.toPSMoveFloatVector3());
			trackerSpaceOrientations[sampleCount] = trackerRelativeScaledAngleAxis;
			++sampleCount;

			if (getIsComplete())
			{
				const float N = static_cast<float>(k_mat_calibration_sample_count);

				// Compute the average screen space location
				{
					PSMoveFloatVector2 avg = PSMoveFloatVector2::create(0, 0);

					// Average together all the samples we captured
					for (int sampleIndex = 0; sampleIndex < k_mat_calibration_sample_count; ++sampleIndex)
					{
						const PSMoveScreenLocation &sample = screenSpacePoints[sampleIndex];

						avg = avg + sample.toPSMoveFloatVector2();
					}
					avg = avg.unsafe_divide(N);

					// Save the average sample for this tracker at this location
					avgScreenSpacePointAtLocation[sampleLocationIndex] =
						PSMoveScreenLocation::create(avg.i, avg.j);
				}

				// Compute the mean pixel tracking projection area
				{
					float avg = 0;

					for (int sampleIndex = 0; sampleIndex < k_mat_calibration_sample_count; ++sampleIndex)
					{
						avg += screenSpaceArea[sampleIndex];
					}

					avgScreenSpaceArea[sampleLocationIndex] = avg / N;
				}

#ifdef COMPUTE_OPTICAL_VARIANCES
				// Compute the mean and variance of the tracker relative position at the sample location
				Eigen::Vector3f trackerSpacePointVariance;
				eigen_vector3f_compute_mean_and_variance(
					trackerSpacePoints, 
					k_mat_calibration_sample_count, 
					&avgTrackerSpacePointAtLocation[sampleLocationIndex],
					&trackerSpacePointVariance);
				trackerSpacePointMaxVarianceAtLocation[sampleLocationIndex] = trackerSpacePointVariance.maxCoeff();

				// Compute the mean and variance of the tracker relative orientation at the sample location
				Eigen::Vector3f trackerSpaceOrientationVariance;
				eigen_vector3f_compute_mean_and_variance(
					trackerSpaceOrientations,
					k_mat_calibration_sample_count,
					nullptr, // Don't need the mean since we don't care how the controller was oriented during sampling
					&trackerSpaceOrientationVariance);
				trackerSpaceOrientationMaxVarianceAtLocation[sampleLocationIndex] = trackerSpaceOrientationVariance.maxCoeff();
#endif // COMPUTE_OPTICAL_VARIANCES
			}
		}
	}

	static bool computeVarianceBestFitLines(
		TrackerRelativePoseStatistics **trackerStatsList, int trackerCount,
		Eigen::Vector2f &out_position_variance_line, Eigen::Vector2f &out_orientation_variance_line)
	{
		Eigen::Vector2f area_positionvariance_samples[k_mat_sample_location_count*PSMOVESERVICE_MAX_TRACKER_COUNT];
		Eigen::Vector2f area_orientationvariance_samples[k_mat_sample_location_count*PSMOVESERVICE_MAX_TRACKER_COUNT];
		int sample_count = 0;

		for (int tracker_index = 0; tracker_index < trackerCount; ++tracker_index)
		{
			const TrackerRelativePoseStatistics *trackerStats = trackerStatsList[tracker_index];

			// Build lists mapping projection area on a given tracker to variance measurement
			for (int location_index = 0; location_index < k_mat_sample_location_count; ++location_index)
			{
				const float area = trackerStats->avgScreenSpaceArea[location_index];
				const float positionVariance = trackerStats->trackerSpacePointMaxVarianceAtLocation[location_index];
				const float orientationVariance = trackerStats->trackerSpaceOrientationMaxVarianceAtLocation[location_index];

				area_positionvariance_samples[sample_count] = Eigen::Vector2f(area, positionVariance);
				area_orientationvariance_samples[sample_count] = Eigen::Vector2f(area, orientationVariance);
				++sample_count;
			}
		}

		// Compute best fit lines for the area-variance tables		
		bool bSuccess = true;
		bSuccess&= eigen_alignment_fit_least_squares_line(
				area_positionvariance_samples, sample_count,
				&out_position_variance_line, nullptr);
		bSuccess&= eigen_alignment_fit_least_squares_line(
			area_orientationvariance_samples, sample_count,
			&out_orientation_variance_line, nullptr);

		return bSuccess;
	}
};

struct HMDTrackerPoseStatistics
{
	PSMovePosition worldSpacePoints[k_mat_calibration_sample_count];
	PSMoveQuaternion worldSpaceOrientations[k_mat_calibration_sample_count];
	int sampleCount;

	PSMovePosition avgHMDWorldSpacePoint;
	PSMoveQuaternion avgHMDWorldSpaceOrientation;

	HMDTrackerPoseStatistics()
	{
		clear();
	}

	bool getIsComplete() const
	{
		return sampleCount >= k_mat_calibration_sample_count;
	}

	void clear()
	{
		memset(this, 0, sizeof(HMDTrackerPoseStatistics));
	}

	void addSample(const PSMovePose &pose)
	{
		if (getIsComplete())
		{
			return;
		}

		worldSpacePoints[sampleCount] = pose.Position;
		worldSpaceOrientations[sampleCount] = pose.Orientation;
		++sampleCount;

		// See if we just read the last sample
		if (getIsComplete())
		{
			const float N = static_cast<float>(k_mat_sample_location_count);
			PSMoveFloatVector3 avgPosition = *k_psmove_float_vector3_zero;
			PSMoveQuaternion avgOrientation = *k_psmove_quaternion_identity;

			// Average together all the samples we captured
			for (int sampleIndex = 0; sampleIndex < k_mat_sample_location_count; ++sampleIndex)
			{
				const PSMovePosition &posSample = worldSpacePoints[sampleCount];
				const PSMoveQuaternion &orientationSample = worldSpaceOrientations[sampleCount];

				avgPosition = avgPosition + posSample.toPSMoveFloatVector3();
				avgOrientation = avgOrientation + orientationSample;
			}

			// Save the average sample for the HMD
			avgHMDWorldSpacePoint = avgPosition.unsafe_divide(N).castToPSMovePosition();

			// --Constrain the rotation to just be a yaw (rotation about y-axis) 
			// Compute the average pose quaternion
			PSMoveQuaternion avg_quaternion =
				avgOrientation.unsafe_divide(N).normalize_with_default(*k_psmove_quaternion_identity);
			glm::quat glm_quat = psmove_quaternion_to_glm_quat(avg_quaternion);

			// Convert the pose quaternion into a set of basis vectors
			glm::mat3 glm_mat3 = glm::mat3_cast(glm_quat);

			// Extract the forward (z-axis) vector from the basis
			glm::vec3 glm_forward = glm_mat3[2];

			// Project the forward vector onto the x-z plane
			glm::vec3 glm_forward2d = glm::normalize(glm::vec3(glm_forward.x, 0.f, glm_forward.z));

			// Compute the yaw angle (amount the z-axis has been rotated to it's current facing)
			float cos_yaw = glm::dot(glm_forward2d, glm::vec3(0.f, 0.f, 1.f));
			float half_yaw = acosf(clampf(cos_yaw, -1.f, 1.f)) / 2.f;
			glm::vec3 yaw_axis = glm::cross(glm::vec3(0.f, 0.f, 1.f), glm_forward2d);
			if (glm::dot(yaw_axis, glm::vec3(0.f, 1.f, 0.f)) < 0)
			{
				half_yaw = -half_yaw;
			}

			// Convert this yaw rotation back into a quaternion
			avgHMDWorldSpaceOrientation =
				PSMoveQuaternion::create(
					cosf(half_yaw), // w = cos(theta/2)
					0.f, sinf(half_yaw), 0.f); // (x, y, z) = sin(theta/2)*axis, where axis = (0, 1, 0)
		}
	}
};


//-- private methods -----
static bool computeTrackerCameraPose(
    const ClientTrackerView *trackerView,
    TrackerRelativePoseStatistics &trackerCoregData);

//-- public methods -----
AppSubStage_CalibrateWithMat::AppSubStage_CalibrateWithMat(
    AppStage_ComputeTrackerPoses *parentStage)
    : m_parentStage(parentStage)
    , m_menuState(AppSubStage_CalibrateWithMat::eMenuState::invalid)
    , m_bIsStable(false)
	, m_sampleLocationIndex(0)
{
	for (int location_index = 0; location_index < k_mat_sample_location_count; ++location_index)
	{
		m_psmoveTrackerPoseStats[location_index] = new TrackerRelativePoseStatistics;
	}
}

AppSubStage_CalibrateWithMat::~AppSubStage_CalibrateWithMat()
{
	for (int location_index = 0; location_index < k_mat_sample_location_count; ++location_index)
	{
		delete m_psmoveTrackerPoseStats[location_index];
	}
}

void AppSubStage_CalibrateWithMat::enter()
{
    setState(AppSubStage_CalibrateWithMat::eMenuState::initial);
}

void AppSubStage_CalibrateWithMat::exit()
{
    setState(AppSubStage_CalibrateWithMat::eMenuState::initial);
}

void AppSubStage_CalibrateWithMat::update()
{
    const ClientControllerView *ControllerView= m_parentStage->m_controllerView;

    switch (m_menuState)
    {
    case AppSubStage_CalibrateWithMat::eMenuState::initial:
        {
            // Go immediately to the initial place PSMove stage
            setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove);
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove:
        {
            if (ControllerView->GetIsStable() || m_bForceControllerStable)
            {
                std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();

                if (m_bIsStable)
                {
                    std::chrono::duration<double, std::milli> stableDuration = now - m_stableStartTime;

                    if (stableDuration.count() >= k_stabilize_wait_time_ms)
                    {
                        setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordPSMove);
                    }
                }
                else
                {
                    m_bIsStable = true;
                    m_stableStartTime = now;
                }
            }
            else
            {
                if (m_bIsStable)
                {
                    m_bIsStable = false;
                }
            }

            // Poll the next video frame from the tracker rendering
            m_parentStage->update_tracker_video();
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordPSMove:
        {
            const bool bIsStable = ControllerView->GetIsStable();

            // See if any tracker needs more samples
            bool bNeedMoreSamples = false;
            for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                iter != m_parentStage->m_trackerViews.end(); 
                ++iter)
            {
                const int trackerIndex = iter->second.listIndex;

                if (!m_psmoveTrackerPoseStats[trackerIndex]->getIsComplete())
                {
                    bNeedMoreSamples = true;
                    break;
                }
            }

            if (bNeedMoreSamples)
            {
                // Only record samples when the controller is stable
                if (bIsStable || m_bForceControllerStable)
                {
                    for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                        iter != m_parentStage->m_trackerViews.end();
                        ++iter)
                    {
                        const int trackerIndex = iter->second.listIndex;
                        const ClientTrackerView *trackerView = iter->second.trackerView;

                        if (ControllerView->GetIsCurrentlyTracking())
                        {
							m_psmoveTrackerPoseStats[trackerIndex]->addSample(trackerView, ControllerView, m_sampleLocationIndex);
                        }
                    }
                }
                else
                {
                    // Whoops! The controller got moved.
                    // Reset the sample count at this location for all trackers and wait for it 
                    setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove);
                }
            }
            else
            {
                // If we have completed sampling at this location, wait until the controller is picked up
                if (!bIsStable)
                {
                    // Move on to next sample location
                    ++m_sampleLocationIndex;

                    if (m_sampleLocationIndex < k_mat_sample_location_count)
                    {
                        // If there are more sample locations
                        // wait until the controller stabilizes at the new location
                        setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove);
                    }
                    else
                    {
                        setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepComputeTrackerPoses);
                    }
                }
            }

            // Poll the next video frame from the tracker rendering
            m_parentStage->update_tracker_video();
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepComputeTrackerPoses:
        {
            bool bSuccess = true;

            // Compute and the pose transform for each tracker
            for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                bSuccess && iter != m_parentStage->m_trackerViews.end();
                ++iter)
            {
                const int trackerIndex = iter->second.listIndex;
                const ClientTrackerView *trackerView = iter->second.trackerView;
                TrackerRelativePoseStatistics &trackerSampleData = *m_psmoveTrackerPoseStats[trackerIndex];

                bSuccess&= computeTrackerCameraPose(trackerView, trackerSampleData);
            }

            if (bSuccess)
            {
                // Update the poses on each local tracker view and notify the service of the new pose
                for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                    bSuccess && iter != m_parentStage->m_trackerViews.end();
                    ++iter)
                {
                    const int trackerIndex = iter->second.listIndex;
                    const TrackerRelativePoseStatistics &trackerSampleData = *m_psmoveTrackerPoseStats[trackerIndex];
                    const PSMovePose trackerPose = trackerSampleData.trackerPose;

                    ClientTrackerView *trackerView = iter->second.trackerView;

                    m_parentStage->request_set_tracker_pose(&trackerPose, trackerView);
                }
            }

			#ifdef COMPUTE_OPTICAL_VARIANCES
			if (bSuccess)
			{
				int trackerCount= static_cast<int>(m_parentStage->m_trackerViews.size());
				Eigen::Vector2f positionVarianceLine;
				Eigen::Vector2f orientationVarianceLine;

				if (TrackerRelativePoseStatistics::computeVarianceBestFitLines(
						m_psmoveTrackerPoseStats, trackerCount,
						positionVarianceLine, orientationVarianceLine))
				{
					RequestPtr request(new PSMoveProtocol::Request());
					request->set_type(PSMoveProtocol::Request_RequestType_SET_OPTICAL_NOISE_CALIBRATION);

					PSMoveProtocol::Request_RequestSetOpticalNoiseCalibration *set_calib_request =
						request->mutable_request_set_optical_noise_calibration();

					set_calib_request->set_controller_id(ControllerView->GetControllerID());
					set_calib_request->set_position_variance_gain(positionVarianceLine.x());
					set_calib_request->set_position_variance_bias(positionVarianceLine.y());
					set_calib_request->set_orientation_variance_gain(orientationVarianceLine.x());
					set_calib_request->set_orientation_variance_bias(orientationVarianceLine.y());

					ClientPSMoveAPI::eat_response(ClientPSMoveAPI::send_opaque_request(&request));
				}
			}
			#endif // COMPUTE_OPTICAL_VARIANCES

            if (bSuccess)
            {
                setState(AppSubStage_CalibrateWithMat::eMenuState::calibrateStepSuccess);
            }
            else
            {
                setState(AppSubStage_CalibrateWithMat::eMenuState::calibrateStepFailed);
            }
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepSuccess:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_CalibrateWithMat::render()
{
    switch (m_menuState)
    {
    case AppSubStage_CalibrateWithMat::eMenuState::initial:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordPSMove:
        {
            // Draw the video from the PoV of the current tracker
            m_parentStage->render_tracker_video();
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepComputeTrackerPoses:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepSuccess:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_CalibrateWithMat::renderUI()
{
    const float k_panel_width = 450.f;
    const char *k_window_title = "Compute Tracker Poses";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;
    const std::chrono::time_point<std::chrono::high_resolution_clock> now = 
        std::chrono::high_resolution_clock::now();

    switch (m_menuState)
    {
    case AppSubStage_CalibrateWithMat::eMenuState::initial:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Stand the PSMove upright on location #%d (%s)",
                m_sampleLocationIndex + 1, k_sample_location_names[m_sampleLocationIndex]);

            if (m_bIsStable)
            {
                std::chrono::duration<double, std::milli> stableDuration = now - m_stableStartTime;

                ImGui::Text("[stable for %d/%dms]", 
                    static_cast<int>(stableDuration.count()),
                    static_cast<int>(k_stabilize_wait_time_ms));
            }
            else
            {
                ImGui::Text("[Not stable and upright]");
            }

            ImGui::Separator();

            if (m_parentStage->get_tracker_count() > 1)
            {
                ImGui::Text("Tracker #%d", m_parentStage->get_render_tracker_index() + 1);

                if (ImGui::Button("Previous Tracker"))
                {
                    m_parentStage->go_previous_tracker();
                }
                ImGui::SameLine();
                if (ImGui::Button("Next Tracker"))
                {
                    m_parentStage->go_next_tracker();
                }
            }

            if (ImGui::Button("Trust me, it's stable"))
            {
                m_bForceControllerStable= true;
            }
            ImGui::SameLine();
            if (ImGui::Button("Restart Calibration"))
            {
                setState(AppSubStage_CalibrateWithMat::eMenuState::initial);
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel"))
            {
                m_parentStage->setState(AppStage_ComputeTrackerPoses::eMenuState::verifyTrackers);
            }

            ImGui::End();
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordPSMove:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 200));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Recording PSMove samples at location #%d (%s)",
                m_sampleLocationIndex + 1, k_sample_location_names[m_sampleLocationIndex]);

            bool bAnyTrackersSampling = false;
            for (int tracker_index = 0; tracker_index < m_parentStage->get_tracker_count(); ++tracker_index)
            {
                const int sampleCount = m_psmoveTrackerPoseStats[tracker_index]->sampleCount;

                if (sampleCount < k_mat_calibration_sample_count)
                {
                    ImGui::Text("Tracker %d: sample %d/%d", tracker_index + 1, sampleCount, k_mat_calibration_sample_count);
                    bAnyTrackersSampling = true;
                }
                else
                {
                    ImGui::Text("Tracker %d: COMPLETE", tracker_index + 1);
                }
            }

            if (!bAnyTrackersSampling)
            {
                ImGui::Text("Location sampling complete. Please pick up the controller.");
            }

            ImGui::Separator();

            if (m_parentStage->get_tracker_count() > 1)
            {
                ImGui::Text("Tracker #%d", m_parentStage->get_render_tracker_index() + 1);

                if (ImGui::Button("Previous Tracker"))
                {
                    m_parentStage->go_previous_tracker();
                }
                ImGui::SameLine();
                if (ImGui::Button("Next Tracker"))
                {
                    m_parentStage->go_next_tracker();
                }
            }

            if (ImGui::Button("Cancel"))
            {
                m_parentStage->setState(AppStage_ComputeTrackerPoses::eMenuState::verifyTrackers);
            }

            ImGui::End();
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepComputeTrackerPoses:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepSuccess:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

//-- private methods -----
void AppSubStage_CalibrateWithMat::setState(
    AppSubStage_CalibrateWithMat::eMenuState newState)
{
    if (newState != m_menuState)
    {
        onExitState(m_menuState);
        onEnterState(newState);
        m_menuState = newState;
    }
}

void AppSubStage_CalibrateWithMat::onExitState(
    AppSubStage_CalibrateWithMat::eMenuState oldState)
{
    switch (oldState)
    {
    case AppSubStage_CalibrateWithMat::eMenuState::invalid:
    case AppSubStage_CalibrateWithMat::eMenuState::initial:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordPSMove:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepComputeTrackerPoses:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepSuccess:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_CalibrateWithMat::onEnterState(
    AppSubStage_CalibrateWithMat::eMenuState newState)
{
    switch (newState)
    {
    case AppSubStage_CalibrateWithMat::eMenuState::initial:
        {
            for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                iter != m_parentStage->m_trackerViews.end();
                ++iter)
            {
                const int trackerIndex = iter->second.listIndex;

                m_psmoveTrackerPoseStats[trackerIndex]->clearAll();
            }

            m_sampleLocationIndex = 0;
            m_bIsStable = false;
            m_bForceControllerStable = false;
        }
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove:
        {
            for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                iter != m_parentStage->m_trackerViews.end();
                ++iter)
            {
                const int trackerIndex = iter->second.listIndex;

                m_psmoveTrackerPoseStats[trackerIndex]->clearLastSampleBatch();
            }

            m_bIsStable = false;
            m_bForceControllerStable= false;
        } break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordPSMove:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepComputeTrackerPoses:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepSuccess:
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

//-- math helper functions -----
static bool
computeTrackerCameraPose(
    const ClientTrackerView *trackerView,
    TrackerRelativePoseStatistics &trackerCoregData)
{
    // Get the pixel width and height of the tracker image
    const PSMoveFloatVector2 trackerPixelDimensions = trackerView->getTrackerPixelExtents();

    // Get the tracker "intrinsic" matrix that encodes the camera FOV
    const PSMoveMatrix3x3 cameraMatrix = trackerView->getTrackerIntrinsicMatrix();
    cv::Matx33f cvCameraMatrix = psmove_matrix3x3_to_cv_mat33f(cameraMatrix);

    // Copy the object/image point mappings into OpenCV format
    std::vector<cv::Point3f> cvObjectPoints;
    std::vector<cv::Point2f> cvImagePoints;
    for (int locationIndex = 0; locationIndex < k_mat_sample_location_count; ++locationIndex)
    {
        const PSMoveScreenLocation &screenPoint =
            trackerCoregData.avgScreenSpacePointAtLocation[locationIndex];
        const PSMovePosition &worldPoint =
            k_sample_3d_locations[locationIndex];

        // Add in the psmove calibration origin offset
        cvObjectPoints.push_back(cv::Point3f(worldPoint.x, worldPoint.y, worldPoint.z));

        // Convert the tracker screen locations in PSMoveScreenLocation space
        // i.e. [-frameWidth/2, -frameHeight/2]x[frameWidth/2, frameHeight/2] 
        // into OpenCV pixel space
        // i.e. [0, 0]x[frameWidth, frameHeight]
        cvImagePoints.push_back(
            cv::Point2f(
                screenPoint.x + (trackerPixelDimensions.i / 2), 
                screenPoint.y + (trackerPixelDimensions.j / 2)));
    }

    // Assume no distortion
    // TODO: Probably should get the distortion coefficients out of the tracker
    cv::Mat cvDistCoeffs(4, 1, cv::DataType<float>::type);
    cvDistCoeffs.at<float>(0) = 0;
    cvDistCoeffs.at<float>(1) = 0;
    cvDistCoeffs.at<float>(2) = 0;
    cvDistCoeffs.at<float>(3) = 0;

    // Solve the Project N-Point problem:
    // Given a set of 3D points and their corresponding 2D pixel projections,
    // solve for the cameras position and orientation that would allow
    // us to re-project the 3D points back onto the 2D pixel locations
    cv::Mat rvec(3, 1, cv::DataType<double>::type);
    cv::Mat tvec(3, 1, cv::DataType<double>::type);
    trackerCoregData.bValidTrackerPose = cv::solvePnP(cvObjectPoints, cvImagePoints, cvCameraMatrix, cvDistCoeffs, rvec, tvec);

    // Compute the re-projection error
    if (trackerCoregData.bValidTrackerPose)
    {
        std::vector<cv::Point2f> projectedPoints;
        cv::projectPoints(cvObjectPoints, rvec, tvec, cvCameraMatrix, cvDistCoeffs, projectedPoints);

        trackerCoregData.reprojectionError = 0.f;
        for (unsigned int i = 0; i < projectedPoints.size(); ++i)
        {
            const float xError = cvImagePoints[i].x - projectedPoints[i].x;
            const float yError = cvImagePoints[i].y - projectedPoints[i].y;
            const float squaredError = xError*xError + yError*yError;

            trackerCoregData.reprojectionError = squaredError;
        }
    }

    // Covert the rotation vector and translation into a GLM 4x4 transform
    if (trackerCoregData.bValidTrackerPose)
    {
        // Convert rvec to a rotation matrix
        cv::Mat R;
        cv::Rodrigues(rvec, R);

        float rotMat[9];
        for (int i = 0; i < 9; i++)
        {
            rotMat[i] = static_cast<float>(R.at<double>(i));
        }

        cv::Mat R_inv = R.t();
        cv::Mat tvecInv = -R_inv * tvec; // translation of the inverse R|t transform
        float tv[3];
        for (int i = 0; i < 3; i++)
        {
            tv[i] = static_cast<float>(tvecInv.at<double>(i));
        }

        float RTMat[] = {
            rotMat[0], rotMat[1], rotMat[2], 0.0f,
            rotMat[3], rotMat[4], rotMat[5], 0.0f,
            rotMat[6], rotMat[7], rotMat[8], 0.0f,
            tv[0], tv[1], tv[2], 1.0f };

        glm::mat4 trackerXform = glm::make_mat4(RTMat);

        // Save off the tracker pose in MultiCam Tracking space
        trackerCoregData.trackerPose = glm_mat4_to_psmove_pose(trackerXform);
    }

    return trackerCoregData.bValidTrackerPose;
}
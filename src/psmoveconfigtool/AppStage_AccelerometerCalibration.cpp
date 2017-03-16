//-- inludes -----
#include "AppStage_AccelerometerCalibration.h"
#include "AppStage_ControllerSettings.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "GeometryUtility.h"
#include "Logger.h"
#include "MathAlignment.h"
#include "MathGLM.h"
#include "MathEigen.h"
#include "MathUtility.h"
#include "PSMoveClient_CAPI.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include "Renderer.h"
#include "UIConstants.h"

#include "SDL_keycode.h"

#include <imgui.h>

#include <algorithm>

//-- statics ----
const char *AppStage_AccelerometerCalibration::APP_STAGE_NAME = "AcceleromterCalibration";

//-- constants -----
static const double k_stabilize_wait_time_ms = 1000.f;
static const int k_max_accelerometer_samples = 500;

static const float k_min_sample_distance = 1000.f;
static const float k_min_sample_distance_sq = k_min_sample_distance*k_min_sample_distance;

//-- definitions -----
struct AccelerometerStatistics
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PSMVector3f accelerometer_samples[k_max_accelerometer_samples];
	Eigen::Vector3f eigen_accelerometer_samples[k_max_accelerometer_samples];
    Eigen::Vector3f avg_accelerometer_sample;
	float noise_variance;
    float noise_radius;
    int sample_count;

    void clear()
    {        
        sample_count= 0;
        noise_radius= 0.f;
    }

	bool getIsComplete() const
	{
		return sample_count >= k_max_accelerometer_samples;
	}

	void addSample(const PSMVector3f &sample)
	{
		if (getIsComplete())
		{
			return;
		}

		accelerometer_samples[sample_count] = sample;
		eigen_accelerometer_samples[sample_count] = psm_vector3f_to_eigen_vector3(sample);
		++sample_count;

		if (getIsComplete())
		{
			// Compute the mean and variance of the accelerometer readings
			Eigen::Vector3f accelerometer_variance;
			eigen_vector3f_compute_mean_and_variance(
				eigen_accelerometer_samples, sample_count, 
				&avg_accelerometer_sample, &accelerometer_variance);
			noise_variance = accelerometer_variance.maxCoeff();

			// Compute the bounding radius of the accelerometer error
			noise_radius = 0;
			for (int sample_index = 0; sample_index < k_max_accelerometer_samples; ++sample_index)
			{
				Eigen::Vector3f error = eigen_accelerometer_samples[sample_index] - avg_accelerometer_sample;

				noise_radius = fmaxf(noise_radius, error.norm());
			}
		}
	}
};

//-- private methods -----
static void request_set_accelerometer_calibration(
    const int controller_id, const float noise_radius, const float noise_variance);
static void drawController(PSMController *controllerView, const glm::mat4 &transform);

//-- public methods -----
AppStage_AccelerometerCalibration::AppStage_AccelerometerCalibration(App *app)
    : AppStage(app)
    , m_menuState(AppStage_AccelerometerCalibration::inactive)
	, m_testMode(AppStage_AccelerometerCalibration::controllerRelative)
    , m_bBypassCalibration(false)
    , m_controllerView(nullptr)
    , m_isControllerStreamActive(false)
    , m_lastControllerSeqNum(-1)
    , m_noiseSamples(new AccelerometerStatistics)
{
}

AppStage_AccelerometerCalibration::~AppStage_AccelerometerCalibration()
{
    delete m_noiseSamples;
}

void AppStage_AccelerometerCalibration::enter()
{
    const AppStage_ControllerSettings *controllerSettings =
        m_app->getAppStage<AppStage_ControllerSettings>();
    const AppStage_ControllerSettings::ControllerInfo *controllerInfo =
        controllerSettings->getSelectedControllerInfo();

    // Reset the menu state
    m_app->setCameraType(_cameraOrbit);
    m_app->getOrbitCamera()->resetOrientation();
    m_app->getOrbitCamera()->setCameraOrbitRadius(1000.f); // zoom out to see the magnetometer data at scale

    m_menuState = eCalibrationMenuState::waitingForStreamStartResponse;

    m_noiseSamples->clear();

    // Initialize the controller state
    assert(controllerInfo->ControllerID != -1);
    assert(m_controllerView == nullptr);
	PSM_AllocateControllerListener(controllerInfo->ControllerID);
    m_controllerView = PSM_GetController(controllerInfo->ControllerID);

    m_lastCalibratedAccelerometer = *k_psm_float_vector3_zero;
    m_lastControllerSeqNum = -1;

    // Start streaming in controller data
    assert(!m_isControllerStreamActive);
	PSMRequestID request_id;
	PSM_StartControllerDataStreamAsync(m_controllerView->ControllerID, PSMStreamFlags_includeCalibratedSensorData, &request_id);
	PSM_RegisterCallback(request_id, &AppStage_AccelerometerCalibration::handle_acquire_controller, this);
}

void AppStage_AccelerometerCalibration::exit()
{
    assert(m_controllerView != nullptr);
    PSM_FreeControllerListener(m_controllerView->ControllerID);
    m_controllerView = nullptr;
    m_menuState = eCalibrationMenuState::inactive;

    // Reset the orbit camera back to default orientation and scale
    m_app->getOrbitCamera()->reset();
}

void AppStage_AccelerometerCalibration::update()
{
    bool bControllerDataUpdatedThisFrame = false;

    if (m_isControllerStreamActive && m_controllerView->OutputSequenceNum != m_lastControllerSeqNum)
    {
        switch(m_controllerView->ControllerType)
        {
        case PSMController_DualShock4:
            {
                const PSMDS4CalibratedSensorData &calibratedSensorData =
                    m_controllerView->ControllerState.PSDS4State.CalibratedSensorData;

                m_lastCalibratedAccelerometer = calibratedSensorData.Accelerometer;
            } break;
        case PSMController_Move:
            {
                const PSMPSMoveCalibratedSensorData &calibratedSensorData =
                    m_controllerView->ControllerState.PSMoveState.CalibratedSensorData;

                m_lastCalibratedAccelerometer = calibratedSensorData.Accelerometer;
            } break;
        default:
            assert(0 && "unreachable");
        }

        m_lastControllerSeqNum = m_controllerView->OutputSequenceNum;
        bControllerDataUpdatedThisFrame = true;
    }

    switch (m_menuState)
    {
    case eCalibrationMenuState::waitingForStreamStartResponse:
        {
            if (bControllerDataUpdatedThisFrame)
            {
                if (m_bBypassCalibration)
                {
                    m_app->getOrbitCamera()->resetOrientation();
                    m_menuState = AppStage_AccelerometerCalibration::test;
                }
                else
                {
                    m_menuState = AppStage_AccelerometerCalibration::placeController;
                }
            }
        } break;
    case eCalibrationMenuState::failedStreamStart:
    case eCalibrationMenuState::placeController:
        {
        } break;
    case eCalibrationMenuState::measureNoise:
        {
            if (bControllerDataUpdatedThisFrame && m_noiseSamples->sample_count < k_max_accelerometer_samples)
            {
                // Store the new sample
				m_noiseSamples->addSample(m_lastCalibratedAccelerometer);

                // See if we filled all of the samples for this pose
                if (m_noiseSamples->getIsComplete())
                {
                    // Tell the service what the new calibration constraints are
                    request_set_accelerometer_calibration(
                        m_controllerView->ControllerID,
                        m_noiseSamples->noise_radius,
						m_noiseSamples->noise_variance);

                    m_menuState = AppStage_AccelerometerCalibration::measureComplete;
                }
            }
        } break;
    case eCalibrationMenuState::measureComplete:
    case eCalibrationMenuState::test:
        {
			if (m_controllerView->ControllerType == PSMController_DualShock4 &&
				m_controllerView->ControllerState.PSDS4State.OptionsButton == PSMButtonState_PRESSED)
			{
				PSM_ResetControllerOrientationAsync(m_controllerView->ControllerID, k_psm_quaternion_identity, nullptr);
			}
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_AccelerometerCalibration::render()
{
    const float k_modelScale = 18.f;
    glm::mat4 defaultControllerTransform;

    switch(m_controllerView->ControllerType)
    {
    case PSMController_Move:
		defaultControllerTransform= 
			glm::rotate(
				glm::scale(glm::mat4(1.f), glm::vec3(k_modelScale, k_modelScale, k_modelScale)),
				90.f, glm::vec3(1.f, 0.f, 0.f));  
        break;
    case PSMController_DualShock4:
        defaultControllerTransform = glm::scale(glm::mat4(1.f), glm::vec3(k_modelScale, k_modelScale, k_modelScale));
        break;
    }

    switch (m_menuState)
    {
    case eCalibrationMenuState::waitingForStreamStartResponse:
    case eCalibrationMenuState::failedStreamStart:
        {
        } break;
    case eCalibrationMenuState::placeController:
        {
            // Draw the controller model in the pose we want the user place it in
            drawController(m_controllerView, defaultControllerTransform);
        } break;
    case eCalibrationMenuState::measureNoise:
    case eCalibrationMenuState::measureComplete:
        {
            const float sampleScale = 100.f;
            glm::mat4 sampleTransform = glm::scale(glm::mat4(1.f), glm::vec3(sampleScale, sampleScale, sampleScale));

            // Draw the controller in the middle            
            drawController(m_controllerView, defaultControllerTransform);

            // Draw the sample point cloud around the origin
            drawPointCloud(sampleTransform, glm::vec3(1.f, 1.f, 1.f), 
                reinterpret_cast<float *>(m_noiseSamples->accelerometer_samples), 
                m_noiseSamples->sample_count);

            // Draw the current raw accelerometer direction
            {
                glm::vec3 m_start = glm::vec3(0.f, 0.f, 0.f);
                glm::vec3 m_end = psm_vector3f_to_glm_vec3(m_lastCalibratedAccelerometer);

                drawArrow(sampleTransform, m_start, m_end, 0.1f, glm::vec3(1.f, 0.f, 0.f));
                drawTextAtWorldPosition(sampleTransform, m_end, "A");
            }
        } break;
    case eCalibrationMenuState::test:
        {
			const float k_sensorScale = 200.f;

			glm::mat4 controllerTransform;
			glm::mat4 sensorTransform;

			switch (m_testMode)
			{
			case eTestMode::controllerRelative:
				{
					controllerTransform = defaultControllerTransform;
					sensorTransform = glm::scale(glm::mat4(1.f), glm::vec3(k_sensorScale, k_sensorScale, k_sensorScale));
				} break;
			case eTestMode::worldRelative:
				{
					// Get the orientation of the controller in world space (OpenGL Coordinate System)            
					glm::quat q;

					switch(m_controllerView->ControllerType)
					{
					case PSMController_Move:
						q= psm_quatf_to_glm_quat(m_controllerView->ControllerState.PSMoveState.Pose.Orientation);
						break;
					case PSMController_DualShock4:
						q= psm_quatf_to_glm_quat(m_controllerView->ControllerState.PSDS4State.Pose.Orientation);
						break;
					}

					glm::mat4 worldSpaceOrientation = glm::mat4_cast(q);
					
					controllerTransform = glm::scale(worldSpaceOrientation, glm::vec3(k_modelScale, k_modelScale, k_modelScale));
					sensorTransform = glm::rotate(
						glm::scale(worldSpaceOrientation, glm::vec3(k_sensorScale, k_sensorScale, k_sensorScale)),
						-90.f, glm::vec3(1.f, 0.f, 0.f));
				} break;
			default:
				assert(0 && "unreachable");
			}

			// Draw the fixed world space axes
			drawTransformedAxes(glm::scale(glm::mat4(1.f), glm::vec3(k_modelScale, k_modelScale, k_modelScale)), 200.f);

			// Draw the controller
            drawController(m_controllerView, controllerTransform);
            drawTransformedAxes(controllerTransform, 200.f);

			// Draw the accelerometer
			{
				const float accel_g = PSM_Vector3fLength(&m_lastCalibratedAccelerometer);
				glm::vec3 m_start = glm::vec3(0.f);
				glm::vec3 m_end = psm_vector3f_to_glm_vec3(m_lastCalibratedAccelerometer);

				drawArrow(sensorTransform, m_start, m_end, 0.1f, glm::vec3(1.f, 1.f, 1.f));
				drawTextAtWorldPosition(sensorTransform, m_end, "A(%.1fg)", accel_g);
			}

        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_AccelerometerCalibration::renderUI()
{
    const float k_panel_width = 500;
    const char *k_window_title = "Controller Settings";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case eCalibrationMenuState::waitingForStreamStartResponse:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Waiting for controller stream to start...");

            ImGui::End();
        } break;
    case eCalibrationMenuState::failedStreamStart:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Failed to start controller stream!");

            if (ImGui::Button("Ok"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::SameLine();

            if (ImGui::Button("Return to Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::placeController:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

			switch(m_controllerView->ControllerType)
			{
			case PSMController_Move:
				ImGui::Text("Stand the controller on a level surface with the Move button facing you");
				break;
			case PSMController_DualShock4:
				ImGui::Text("Lay the controller flat on the table face up");
				break;
			}

            if (ImGui::Button("Start Sampling"))
            {
                m_menuState = eCalibrationMenuState::measureNoise;
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::measureNoise:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            float sampleFraction =
                static_cast<float>(m_noiseSamples->sample_count)
                / static_cast<float>(k_max_accelerometer_samples);

            ImGui::Text("Sampling accelerometer.");
            ImGui::ProgressBar(sampleFraction, ImVec2(250, 20));

            ImGui::End();
        } break;
    case eCalibrationMenuState::measureComplete:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::TextWrapped(
                "Sampling complete.\n" \
                "Press OK to continue or Redo to resample.");

            if (ImGui::Button("Ok"))
            {
				PSM_SetControllerLEDOverrideColor(m_controllerView->ControllerID, 0, 0, 0);
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }
            ImGui::SameLine();
            if (ImGui::Button("Redo"))
            {
                // Reset the sample info for the current pose
                m_noiseSamples->clear();
                m_menuState = eCalibrationMenuState::placeController;
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::test:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            if (m_bBypassCalibration)
            {
                ImGui::Text("Testing Calibration of Controller ID #%d", m_controllerView->ControllerID);
            }
            else
            {
                ImGui::Text("Calibration of Controller ID #%d complete!", m_controllerView->ControllerID);
            }

			if (m_testMode == eTestMode::controllerRelative)
			{
				if (ImGui::Button("World Relative"))
				{
					m_testMode = eTestMode::worldRelative;
				}
			}
			else if (m_testMode == eTestMode::worldRelative)
			{
				if (ImGui::Button("Controller Relative"))
				{
					m_testMode= eTestMode::controllerRelative;
				}
			}				

            if (ImGui::Button("Ok"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::SameLine();

            if (ImGui::Button("Return to Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    default:
        assert(0 && "unreachable");
    }
}

//-- private methods -----
static void request_set_accelerometer_calibration(
    const int controller_id,
    const float noise_radius,
	const float noise_variance)
{
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_CONTROLLER_ACCELEROMETER_CALIBRATION);

    PSMoveProtocol::Request_RequestSetControllerAccelerometerCalibration *calibration =
        request->mutable_set_controller_accelerometer_calibration_request();

    calibration->set_controller_id(controller_id);
    calibration->set_noise_radius(noise_radius);
	calibration->set_variance(noise_variance);

    PSM_SendOpaqueRequest(&request, nullptr);
}

void AppStage_AccelerometerCalibration::handle_acquire_controller(
    const PSMResponseMessage *response,
    void *userdata)
{
    AppStage_AccelerometerCalibration *thisPtr = reinterpret_cast<AppStage_AccelerometerCalibration *>(userdata);

    if (response->result_code == PSMResult_Success)
    {
        thisPtr->m_isControllerStreamActive = true;
        thisPtr->m_lastControllerSeqNum = -1;
        // Wait for the first controller packet to show up...
    }
    else
    {
        thisPtr->m_menuState = AppStage_AccelerometerCalibration::failedStreamStart;
    }
}

void AppStage_AccelerometerCalibration::request_exit_to_app_stage(const char *app_stage_name)
{
	PSM_StopControllerDataStreamAsync(m_controllerView->ControllerID, nullptr);
    m_isControllerStreamActive= false;
    m_app->setAppStage(app_stage_name);
}

//-- private methods -----
static void drawController(PSMController *controllerView, const glm::mat4 &transform)
{
    switch(controllerView->ControllerType)
    {
    case PSMController_Move:
        drawPSMoveModel(transform, glm::vec3(1.f, 1.f, 1.f));
        break;
    case PSMController_DualShock4:
        drawPSDualShock4Model(transform, glm::vec3(1.f, 1.f, 1.f));
        break;
    }
}

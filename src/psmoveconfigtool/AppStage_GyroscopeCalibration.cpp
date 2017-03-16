//-- inludes -----
#include "AppStage_GyroscopeCalibration.h"
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

#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include "Renderer.h"
#include "UIConstants.h"

#include "SDL_keycode.h"

#include <imgui.h>

//-- statics ----
const char *AppStage_GyroscopeCalibration::APP_STAGE_NAME = "GyroscopeCalibration";

//-- constants -----
const double k_stabilize_wait_time_ms = 1000.f;
const int k_desired_noise_sample_count = 1000;
const float k_desired_drift_sampling_time = 30.0*1000.f; // milliseconds

const int k_desired_scale_sample_count = 1000;

//-- definitions -----
struct GyroscopeNoiseSamples
{
    PSMVector3f omega_samples[k_desired_noise_sample_count];
    PSMVector3f drift_rotation;
    std::chrono::time_point<std::chrono::high_resolution_clock> sampleStartTime;
    int sample_count;

    float variance; // Max sensor variance (raw_sensor_units/s/s for DS4, rad/s/s for PSMove)
    float drift; // Max drift rate (raw_sensor_units/s for DS4, rad/s for PSMove)

    void clear()
    {
        drift_rotation= *k_psm_float_vector3_zero;
        sample_count= 0;
        variance= 0.f;
        drift= 0.f;
    }

    void computeStatistics(std::chrono::duration<float, std::milli> sampleDurationMilli)
    {
        const float sampleDurationSeconds= sampleDurationMilli.count() / 1000.f;
        const float N = static_cast<float>(sample_count);

        // Compute the mean of the error samples, where "error" = abs(omega_sample)
        // If we took the mean of the signed omega samples we'd get a value very 
        // close to zero since the the gyro at rest over a short period has mean-zero noise
        PSMVector3f mean_omega_error= *k_psm_float_vector3_zero;
        for (int sample_index = 0; sample_index < sample_count; sample_index++)
        {
            PSMVector3f error_sample= PSM_Vector3fAbs(&omega_samples[sample_index]);

            mean_omega_error= PSM_Vector3fAdd(&mean_omega_error, &error_sample);
        }
        mean_omega_error= PSM_Vector3fUnsafeScalarDivide(&mean_omega_error, N);

        // Compute the variance of the (unsigned) sample error, where "error" = abs(omega_sample)
        PSMVector3f var_omega= *k_psm_float_vector3_zero;
        for (int sample_index = 0; sample_index < sample_count; sample_index++)
        {
            PSMVector3f error_sample= PSM_Vector3fAbs(&omega_samples[sample_index]);
            PSMVector3f diff_from_mean= PSM_Vector3fSubtract(&error_sample, &mean_omega_error);
            PSMVector3f diff_from_mean_sqrd= PSM_Vector3fSquare(&diff_from_mean);

            var_omega= PSM_Vector3fAdd(&var_omega, &diff_from_mean);
        }
        var_omega= PSM_Vector3fUnsafeScalarDivide(&var_omega, N - 1);

        // Use the max variance of all three axes (should be close)
        variance= PSM_Vector3fMaxValue(&var_omega);

        // Compute the max drift rate we got across a three axis
        PSMVector3f drift_rate= PSM_Vector3fUnsafeScalarDivide(&drift_rotation, sampleDurationSeconds);
        PSMVector3f drift_rate_abs= PSM_Vector3fAbs(&drift_rate);
        drift= PSM_Vector3fMaxValue(&drift_rate_abs);
    }
};

//-- private methods -----
static void drawController(PSMController *controllerView, const glm::mat4 &transform);

//-- public methods -----
AppStage_GyroscopeCalibration::AppStage_GyroscopeCalibration(App *app)
    : AppStage(app)
    , m_menuState(AppStage_GyroscopeCalibration::inactive)
    , m_bBypassCalibration(false)
    , m_controllerView(nullptr)
    , m_isControllerStreamActive(false)
    , m_lastControllerSeqNum(-1)
    , m_lastRawGyroscope()
    , m_gyroNoiseSamples(new GyroscopeNoiseSamples)
{
}

AppStage_GyroscopeCalibration::~AppStage_GyroscopeCalibration()
{
    delete m_gyroNoiseSamples;
}

void AppStage_GyroscopeCalibration::enter()
{
    const AppStage_ControllerSettings *controllerSettings =
        m_app->getAppStage<AppStage_ControllerSettings>();
    const AppStage_ControllerSettings::ControllerInfo *controllerInfo =
        controllerSettings->getSelectedControllerInfo();

	m_menuState = eCalibrationMenuState::inactive;

    // Reset all of the sampling state
    m_gyroNoiseSamples->clear();

    m_lastRawGyroscope = *k_psm_int_vector3_zero;
    m_lastControllerSeqNum = -1;

    m_lastCalibratedAccelerometer = *k_psm_float_vector3_zero;
    m_lastCalibratedGyroscope = *k_psm_float_vector3_zero;

    m_stableStartTime = std::chrono::time_point<std::chrono::high_resolution_clock>();
    m_bIsStable= false;
	m_bForceControllerStable= false;

	// Initialize the controller state
	assert(controllerInfo->ControllerID != -1);
	assert(m_controllerView == nullptr);
	PSM_AllocateControllerListener(controllerInfo->ControllerID);
	m_controllerView= PSM_GetController(controllerInfo->ControllerID);

	// Get the tracking space settings first (for global forward reference)
	request_tracking_space_settings();
}

void AppStage_GyroscopeCalibration::exit()
{
    assert(m_controllerView != nullptr);
    PSM_FreeControllerListener(m_controllerView->ControllerID);
    m_controllerView = nullptr;
    setState(eCalibrationMenuState::inactive);
}

void AppStage_GyroscopeCalibration::update()
{
    bool bControllerDataUpdatedThisFrame = false;
    bool bTimeDeltaValid = false;
    std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float, std::milli> sampleTimeDeltaMilli(0);

    if (m_isControllerStreamActive && m_controllerView->OutputSequenceNum != m_lastControllerSeqNum)
    {
        switch(m_controllerView->ControllerType)
        {
        case PSMController_DualShock4:
            {
                const PSMDS4RawSensorData &rawSensorData =
                    m_controllerView->ControllerState.PSDS4State.RawSensorData;
                const PSMDS4CalibratedSensorData &calibratedSensorData =
                    m_controllerView->ControllerState.PSDS4State.CalibratedSensorData;

                m_lastRawGyroscope = rawSensorData.Gyroscope;
                m_lastCalibratedGyroscope = calibratedSensorData.Gyroscope;
                m_lastCalibratedAccelerometer = calibratedSensorData.Accelerometer;
            }
            break;
        case PSMController_Move:
            {
                const PSMPSMoveRawSensorData &rawSensorData =
                    m_controllerView->ControllerState.PSMoveState.RawSensorData;
                const PSMPSMoveCalibratedSensorData &calibratedSensorData =
                    m_controllerView->ControllerState.PSMoveState.CalibratedSensorData;

                m_lastRawGyroscope = rawSensorData.Gyroscope;
                m_lastCalibratedGyroscope = calibratedSensorData.Gyroscope;
                m_lastCalibratedAccelerometer = calibratedSensorData.Accelerometer;
            }
            break;
        }

        m_lastControllerSeqNum = m_controllerView->OutputSequenceNum;
        
        if (m_bLastSampleTimeValid)
        {
            sampleTimeDeltaMilli = now - m_lastSampleTime;
        }

        m_lastSampleTime= now;
        m_bLastSampleTimeValid= true;
        
        bControllerDataUpdatedThisFrame = true;
    }

    switch (m_menuState)
    {
	case eCalibrationMenuState::pendingTrackingSpaceSettings:
		{
		} break;
	case eCalibrationMenuState::waitingForStreamStartResponse:
        {
            if (bControllerDataUpdatedThisFrame)
            {
                if (m_bBypassCalibration)
                {
                    setState(AppStage_GyroscopeCalibration::test);
                }
                else
                {
                    setState(AppStage_GyroscopeCalibration::waitForStable);
                }
            }
        } break;
    case eCalibrationMenuState::failedStreamStart:
	case eCalibrationMenuState::failedTrackingSpaceSettings:
        {
        } break;
    case eCalibrationMenuState::waitForStable:
        {
			bool bIsStable;
			bool bCanBeStabilized= PSM_GetIsControllerStable(m_controllerView->ControllerID, &bIsStable) == PSMResult_Success;

            if ((bCanBeStabilized && bIsStable) || m_bForceControllerStable)
            {
                if (m_bIsStable || m_bForceControllerStable)
                {
                    std::chrono::duration<double, std::milli> stableDuration = now - m_stableStartTime;
    
                    if (stableDuration.count() >= k_stabilize_wait_time_ms)
                    {
                        m_gyroNoiseSamples->clear();
                        m_gyroNoiseSamples->sampleStartTime= now;
                        setState(eCalibrationMenuState::measureBiasAndDrift);
                    }
                }
                else
                {
                    m_bIsStable= true;
                    m_stableStartTime= now;
                }
            }
            else
            {
                if (m_bIsStable)
                {
                    m_bIsStable= false;
                }
            }
        } break;
    case eCalibrationMenuState::measureBiasAndDrift: // PSMove and DS4
        {
			bool bIsStable;
			bool bCanBeStabilized= PSM_GetIsControllerStable(m_controllerView->ControllerID, &bIsStable) == PSMResult_Success;

            if ((bCanBeStabilized && bIsStable) || m_bForceControllerStable)
            {
                const std::chrono::duration<float, std::milli> sampleDurationMilli = now - m_gyroNoiseSamples->sampleStartTime;
                const float deltaTimeSeconds= sampleTimeDeltaMilli.count()/1000.f;

                // Accumulate the drift total
                if (deltaTimeSeconds > 0.f)
                {
					m_gyroNoiseSamples->drift_rotation= 
						PSM_Vector3fScaleAndAdd(&m_lastCalibratedGyroscope, deltaTimeSeconds, &m_gyroNoiseSamples->drift_rotation);
                }

                // Record the next noise sample
                if (m_gyroNoiseSamples->sample_count < k_desired_noise_sample_count)
                {
                    m_gyroNoiseSamples->omega_samples[m_gyroNoiseSamples->sample_count]= m_lastCalibratedGyroscope;
                    ++m_gyroNoiseSamples->sample_count;
                }

                // See if we have completed the sampling period
                if (sampleDurationMilli.count() >= k_desired_drift_sampling_time)
                {
                    // Compute bias and drift statistics
                    m_gyroNoiseSamples->computeStatistics(sampleDurationMilli);

                    // Update the gyro config on the service
                    request_set_gyroscope_calibration(
                        m_gyroNoiseSamples->drift, 
                        m_gyroNoiseSamples->variance);

                    setState(eCalibrationMenuState::measureComplete);
                }
            }
            else
            {
                m_bIsStable= false;
                setState(AppStage_GyroscopeCalibration::waitForStable);
            }
        } break;
    case eCalibrationMenuState::measureComplete:
    case eCalibrationMenuState::test:
        {
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_GyroscopeCalibration::render()
{
    const float bigModelScale = 10.f;
    glm::mat4 scaleAndRotateModelX90= 
        glm::rotate(
            glm::scale(glm::mat4(1.f), glm::vec3(bigModelScale, bigModelScale, bigModelScale)),
            90.f, glm::vec3(1.f, 0.f, 0.f));  

    switch (m_menuState)
    {
	case eCalibrationMenuState::pendingTrackingSpaceSettings:
    case eCalibrationMenuState::waitingForStreamStartResponse:
    case eCalibrationMenuState::failedStreamStart:
	case eCalibrationMenuState::failedTrackingSpaceSettings:
        {
        } break;
    case eCalibrationMenuState::waitForStable:
        {
            drawController(m_controllerView, scaleAndRotateModelX90);

            // Draw the current direction of gravity
            {
                const float renderScale = 200.f;
                glm::mat4 renderScaleMatrix = 
                    glm::scale(glm::mat4(1.f), glm::vec3(renderScale, renderScale, renderScale));
                glm::vec3 g= -psm_vector3f_to_glm_vec3(m_lastCalibratedAccelerometer);

                drawArrow(
                    renderScaleMatrix,
                    glm::vec3(), g, 
                    0.1f, 
                    glm::vec3(0.f, 1.f, 0.f));
                drawTextAtWorldPosition(renderScaleMatrix, g, "G");
            }
        } break;
    case eCalibrationMenuState::measureBiasAndDrift:
        {
            drawController(m_controllerView, scaleAndRotateModelX90);
        } break;
    case eCalibrationMenuState::measureComplete:
        {
            drawController(m_controllerView, scaleAndRotateModelX90);
        } break;
    case eCalibrationMenuState::test:
        {
            // Get the orientation of the controller in world space (OpenGL Coordinate System)  
			PSMQuatf controllerQuat;
			if (PSM_GetControllerOrientation(m_controllerView->ControllerID, &controllerQuat) == PSMResult_Success)
			{
				glm::quat q= psm_quatf_to_glm_quat(controllerQuat);
				glm::mat4 worldSpaceOrientation= glm::mat4_cast(q);
				glm::mat4 worldTransform = glm::scale(worldSpaceOrientation, glm::vec3(1.f));

				drawController(m_controllerView, worldTransform);
				drawTransformedAxes(worldSpaceOrientation, 200.f);
				drawTransformedAxes(glm::mat4(1.f), 200.f);
			}
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_GyroscopeCalibration::renderUI()
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
	case eCalibrationMenuState::pendingTrackingSpaceSettings:
    case eCalibrationMenuState::waitingForStreamStartResponse:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Waiting for server response...");

            ImGui::End();
        } break;
    case eCalibrationMenuState::failedStreamStart:
	case eCalibrationMenuState::failedTrackingSpaceSettings:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Failed server request!");

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
    case eCalibrationMenuState::waitForStable:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::TextWrapped(
                "[Step 1 of 2: Measuring gyroscope drift and bias]\n" \
                "Set the controller down on a level surface.\n" \
                "Measurement will start once the controller is aligned with gravity and stable.");

            if (m_bIsStable || m_bForceControllerStable)
            {
                std::chrono::time_point<std::chrono::high_resolution_clock> now= std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::milli> stableDuration = now - m_stableStartTime;
                float fraction = static_cast<float>(stableDuration.count() / k_stabilize_wait_time_ms);

                ImGui::ProgressBar(fraction, ImVec2(250, 20));
                ImGui::Spacing();
            }
            else
            {
                ImGui::Text("Controller Destabilized! Waiting for stabilization..");
            }

            if (ImGui::Button("Trust me, it's stable"))
            {
                m_bForceControllerStable= true;
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::measureBiasAndDrift:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            std::chrono::time_point<std::chrono::high_resolution_clock> now= std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> stableDuration = now - m_gyroNoiseSamples->sampleStartTime;
            float timeFraction = static_cast<float>(stableDuration.count() / k_desired_drift_sampling_time);

            const float sampleFraction = 
                static_cast<float>(m_gyroNoiseSamples->sample_count)
                / static_cast<float>(k_desired_noise_sample_count);

            ImGui::TextWrapped(
                "[Step 1 of 2: Measuring gyroscope drift and bias]\n" \
                "Sampling Gyroscope...");
            ImGui::ProgressBar(fminf(sampleFraction, timeFraction), ImVec2(250, 20));

            if (ImGui::Button("Cancel"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::measureComplete:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::TextWrapped(
                "Sampling complete.\n" \
                "Press OK to continue or Redo to recalibration.");

            if (ImGui::Button("Ok"))
            {
				PSM_SetControllerLEDOverrideColor(m_controllerView->ControllerID, 0, 0, 0);
                setState(eCalibrationMenuState::test);
            }
            ImGui::SameLine();
            if (ImGui::Button("Redo"))
            {
                m_gyroNoiseSamples->clear();
                setState(eCalibrationMenuState::waitForStable);
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::test:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 140));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            if (m_bBypassCalibration)
            {
                ImGui::Text("Testing Calibration of Controller ID #%d", m_controllerView->ControllerID);
            }
            else
            {
                ImGui::Text("Calibration of Controller ID #%d complete!", m_controllerView->ControllerID);
            }

			PSMQuatf controllerQuat;
			if (PSM_GetControllerOrientation(m_controllerView->ControllerID, &controllerQuat) == PSMResult_Success)
			{
				const Eigen::Quaternionf eigen_quat = psm_quatf_to_eigen_quaternionf(controllerQuat);
				const Eigen::EulerAnglesf euler_angles = eigen_quaternionf_to_euler_angles(eigen_quat);

				ImGui::Text("Pitch(x): %.2f, Yaw(y): %.2f, Roll(z): %.2f",
					m_lastCalibratedGyroscope.x * k_radians_to_degreees, 
					m_lastCalibratedGyroscope.y * k_radians_to_degreees,
					m_lastCalibratedGyroscope.z * k_radians_to_degreees);
				ImGui::Text("Attitude: %.2f, Heading: %.2f, Bank: %.2f", 
					euler_angles.get_attitude_degrees(), euler_angles.get_heading_degrees(), euler_angles.get_bank_degrees());
			}

			if (m_controllerView->ControllerType == PSMController_DualShock4)
			{
				ImGui::TextWrapped(
					"[Press the Options button with controller pointed straight forward\n" \
					 "to recenter the controller]");
			}
			else if (m_controllerView->ControllerType == PSMController_Move)
			{
				ImGui::TextWrapped(
					"[Hold the Select button with controller pointed forward\n" \
					"to recenter the controller]");
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
void AppStage_GyroscopeCalibration::setState(eCalibrationMenuState newState)
{
	if (newState != m_menuState)
	{
		onExitState(m_menuState);
		onEnterState(newState);

		m_menuState = newState;
	}
}

void AppStage_GyroscopeCalibration::onExitState(eCalibrationMenuState newState)
{
	switch (m_menuState)
	{
	case eCalibrationMenuState::inactive:
	case eCalibrationMenuState::pendingTrackingSpaceSettings:
	case eCalibrationMenuState::failedTrackingSpaceSettings:
	case eCalibrationMenuState::waitingForStreamStartResponse:
	case eCalibrationMenuState::failedStreamStart:
	case eCalibrationMenuState::waitForStable:
	case eCalibrationMenuState::measureBiasAndDrift:
	case eCalibrationMenuState::measureComplete:
	case eCalibrationMenuState::test:
		break;
	default:
		assert(0 && "unreachable");
	}
}

void AppStage_GyroscopeCalibration::onEnterState(eCalibrationMenuState newState)
{
	switch (newState)
	{
	case eCalibrationMenuState::inactive:
		// Reset the orbit camera back to default orientation and scale
		m_app->getOrbitCamera()->reset();
		break;
	case eCalibrationMenuState::pendingTrackingSpaceSettings:
		// Reset the menu state
		m_app->setCameraType(_cameraOrbit);
		m_app->getOrbitCamera()->resetOrientation();
		m_app->getOrbitCamera()->setCameraOrbitRadius(1000.f); // zoom out to see the accelerometer data at scale
		break;
	case eCalibrationMenuState::failedTrackingSpaceSettings:
	case eCalibrationMenuState::waitingForStreamStartResponse:
	case eCalibrationMenuState::failedStreamStart:
		break;
	case eCalibrationMenuState::waitForStable:
		m_bForceControllerStable= false;
		m_bIsStable= false;
		break;
	case eCalibrationMenuState::measureBiasAndDrift:
		break;
	case eCalibrationMenuState::measureComplete:
		// Reset the menu state
		m_app->setCameraType(_cameraOrbit);
		m_app->getOrbitCamera()->resetOrientation();
		m_app->getOrbitCamera()->setCameraOrbitRadius(1000.f); // zoom out to see the accelerometer data at scale
		break;
	case eCalibrationMenuState::test:
		{
			switch (m_controllerView->ControllerType)
			{
			case PSMController_DualShock4:
				m_controllerView->ControllerState.PSDS4State.bPoseResetButtonEnabled= true;
				break;
			case PSMController_Move:
				m_controllerView->ControllerState.PSMoveState.bPoseResetButtonEnabled= true;
				break;
			}

			m_app->setCameraType(_cameraOrbit);
			m_app->getOrbitCamera()->reset();
			m_app->getOrbitCamera()->setCameraOrbitYaw(m_global_forward_degrees - k_camera_default_forward_degrees);
		}
		break;
	default:
		assert(0 && "unreachable");
	}
}

void AppStage_GyroscopeCalibration::request_tracking_space_settings()
{
	if (m_menuState != eCalibrationMenuState::pendingTrackingSpaceSettings)
	{
		setState(eCalibrationMenuState::pendingTrackingSpaceSettings);

		PSMRequestID request_id;
		PSM_GetTrackingSpaceSettingsAsync(&request_id);
		PSM_RegisterCallback(request_id, AppStage_GyroscopeCalibration::handle_tracking_space_settings_response, this);
	}
}

void AppStage_GyroscopeCalibration::handle_tracking_space_settings_response(
	const PSMResponseMessage *response_message,
	void *userdata)
{
	AppStage_GyroscopeCalibration *thisPtr = static_cast<AppStage_GyroscopeCalibration *>(userdata);

	switch (response_message->result_code)
	{
	case PSMResult_Success:
		{
			const AppStage_ControllerSettings *controllerSettings =
				thisPtr->m_app->getAppStage<AppStage_ControllerSettings>();
			const AppStage_ControllerSettings::ControllerInfo *controllerInfo =
				controllerSettings->getSelectedControllerInfo();

			assert(response_message->payload_type == PSMResponseMessage::_responsePayloadType_TrackingSpace);

			// Save the tracking space settings (used in rendering)
			thisPtr->m_global_forward_degrees = response_message->payload.tracking_space.global_forward_degrees;

			unsigned int stream_flags = 
				PSMStreamFlags_includeRawSensorData |
				PSMStreamFlags_includeCalibratedSensorData;

			if (controllerInfo->ControllerType == PSMController_DualShock4)
			{
				// Need to turn on optical tracking for the DS4 so that we have get optical orientation
				stream_flags |= PSMStreamFlags_includePositionData;
			}

			// Start streaming in controller data
			assert(!thisPtr->m_isControllerStreamActive);
			PSMRequestID request_id;
			PSM_StartControllerDataStreamAsync(thisPtr->m_controllerView->ControllerID, stream_flags, &request_id);
			PSM_RegisterCallback(request_id, &AppStage_GyroscopeCalibration::handle_acquire_controller, thisPtr);

			thisPtr->setState(eCalibrationMenuState::waitingForStreamStartResponse);
		} break;

	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
		{
			thisPtr->setState(eCalibrationMenuState::failedTrackingSpaceSettings);
		} break;
	}
}

void AppStage_GyroscopeCalibration::request_set_gyroscope_calibration(
    const float drift, 
    const float variance)
{
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_CONTROLLER_GYROSCOPE_CALIBRATION);

    PSMoveProtocol::Request_RequestSetControllerGyroscopeCalibration *calibration =
        request->mutable_set_controller_gyroscope_calibration_request();

    calibration->set_controller_id(m_controllerView->ControllerID);

    calibration->set_drift(drift);
    calibration->set_variance(variance);
	calibration->set_gyro_gain_setting(""); // keep existing gain

    PSM_SendOpaqueRequest(&request, nullptr);
}

void AppStage_GyroscopeCalibration::handle_acquire_controller(
    const PSMResponseMessage *response,
    void *userdata)
{
    AppStage_GyroscopeCalibration *thisPtr = reinterpret_cast<AppStage_GyroscopeCalibration *>(userdata);

    if (response->result_code == PSMResult_Success)
    {
        thisPtr->m_isControllerStreamActive = true;
        thisPtr->m_lastControllerSeqNum = -1;
        // Wait for the first controller packet to show up...
    }
    else
    {
        thisPtr->setState(AppStage_GyroscopeCalibration::failedStreamStart);
    }
}

void AppStage_GyroscopeCalibration::request_exit_to_app_stage(const char *app_stage_name)
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

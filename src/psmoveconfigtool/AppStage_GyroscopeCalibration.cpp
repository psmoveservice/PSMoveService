//-- inludes -----
#include "AppStage_GyroscopeCalibration.h"
#include "AppStage_ControllerSettings.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "ClientControllerView.h"
#include "GeometryUtility.h"
#include "Logger.h"
#include "MathAlignment.h"
#include "MathGLM.h"
#include "MathEigen.h"
#include "MathUtility.h"

#include "PSMoveProtocolInterface.h"
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
    PSMoveFloatVector3 omega_samples[k_desired_noise_sample_count];
    PSMoveFloatVector3 drift_rotation;
    std::chrono::time_point<std::chrono::high_resolution_clock> sampleStartTime;
    int sample_count;

    float variance; // Max sensor variance (raw_sensor_units/s/s for DS4, rad/s/s for PSMove)
    float drift; // Max drift rate (raw_sensor_units/s for DS4, rad/s for PSMove)

    void clear()
    {
        drift_rotation= PSMoveFloatVector3::create(0, 0, 0);
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
        PSMoveFloatVector3 mean_omega_error= PSMoveFloatVector3::create(0.f, 0.f, 0.f);
        for (int sample_index = 0; sample_index < sample_count; sample_index++)
        {
            PSMoveFloatVector3 error_sample= omega_samples[sample_index].abs();

            mean_omega_error= mean_omega_error + error_sample;
        }
        mean_omega_error= mean_omega_error.unsafe_divide(N);

        // Compute the variance of the (unsigned) sample error, where "error" = abs(omega_sample)
        PSMoveFloatVector3 var_omega= PSMoveFloatVector3::create(0.f, 0.f, 0.f);
        for (int sample_index = 0; sample_index < sample_count; sample_index++)
        {
            PSMoveFloatVector3 error_sample= omega_samples[sample_index].abs();
            PSMoveFloatVector3 diff_from_mean= error_sample - mean_omega_error;

            var_omega= var_omega + diff_from_mean.square();
        }
        var_omega= var_omega.unsafe_divide(N - 1);

        // Use the max variance of all three axes (should be close)
        variance= var_omega.maxValue();

        // Compute the max drift rate we got across a three axis
        PSMoveFloatVector3 drift_rate= drift_rotation.unsafe_divide(sampleDurationSeconds);
        drift= drift_rate.abs().maxValue();
    }
};

//-- private methods -----
static void drawController(ClientControllerView *controllerView, const glm::mat4 &transform);

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

    m_lastRawGyroscope = *k_psmove_int_vector3_zero;
    m_lastControllerSeqNum = -1;

    m_lastCalibratedAccelerometer = *k_psmove_float_vector3_zero;
    m_lastCalibratedGyroscope = *k_psmove_float_vector3_zero;

    m_stableStartTime = std::chrono::time_point<std::chrono::high_resolution_clock>();
    m_bIsStable= false;
	m_bForceControllerStable= false;

	// Initialize the controller state
	assert(controllerInfo->ControllerID != -1);
	assert(m_controllerView == nullptr);
	m_controllerView = ClientPSMoveAPI::allocate_controller_view(controllerInfo->ControllerID);

	// Get the tracking space settings first (for global forward reference)
	request_tracking_space_settings();
}

void AppStage_GyroscopeCalibration::exit()
{
    assert(m_controllerView != nullptr);
    ClientPSMoveAPI::free_controller_view(m_controllerView);
    m_controllerView = nullptr;
    setState(eCalibrationMenuState::inactive);
}

void AppStage_GyroscopeCalibration::update()
{
    bool bControllerDataUpdatedThisFrame = false;
    bool bTimeDeltaValid = false;
    std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float, std::milli> sampleTimeDeltaMilli(0);

    if (m_isControllerStreamActive && m_controllerView->GetOutputSequenceNum() != m_lastControllerSeqNum)
    {
        switch(m_controllerView->GetControllerViewType())
        {
        case ClientControllerView::PSDualShock4:
            {
                const PSDualShock4RawSensorData &rawSensorData =
                    m_controllerView->GetPSDualShock4View().GetRawSensorData();
                const PSDualShock4CalibratedSensorData &calibratedSensorData =
                    m_controllerView->GetPSDualShock4View().GetCalibratedSensorData();

                m_lastRawGyroscope = rawSensorData.Gyroscope;
                m_lastCalibratedGyroscope = calibratedSensorData.Gyroscope;
                m_lastCalibratedAccelerometer = calibratedSensorData.Accelerometer;
            }
            break;
        case ClientControllerView::PSMove:
            {
                const PSMoveRawSensorData &rawSensorData =
                    m_controllerView->GetPSMoveView().GetRawSensorData();
                const PSMoveCalibratedSensorData &calibratedSensorData =
                    m_controllerView->GetPSMoveView().GetCalibratedSensorData();

                m_lastRawGyroscope = rawSensorData.Gyroscope;
                m_lastCalibratedGyroscope = calibratedSensorData.Gyroscope;
                m_lastCalibratedAccelerometer = calibratedSensorData.Accelerometer;
            }
            break;
        }

        m_lastControllerSeqNum = m_controllerView->GetOutputSequenceNum();
        
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
            if (m_controllerView->GetIsStable() || m_bForceControllerStable)
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
            if (m_controllerView->GetIsStable() || m_bForceControllerStable)
            {
                const std::chrono::duration<float, std::milli> sampleDurationMilli = now - m_gyroNoiseSamples->sampleStartTime;
                const float deltaTimeSeconds= sampleTimeDeltaMilli.count()/1000.f;

                // Accumulate the drift total
                if (deltaTimeSeconds > 0.f)
                {
                    m_gyroNoiseSamples->drift_rotation= 
                        m_gyroNoiseSamples->drift_rotation
                        + m_lastCalibratedGyroscope*deltaTimeSeconds;
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
                glm::vec3 g= -psmove_float_vector3_to_glm_vec3(m_lastCalibratedAccelerometer);

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
            glm::quat q= psmove_quaternion_to_glm_quat(m_controllerView->GetOrientation());
            glm::mat4 worldSpaceOrientation= glm::mat4_cast(q);
            glm::mat4 worldTransform = glm::scale(worldSpaceOrientation, glm::vec3(1.f));

            drawController(m_controllerView, worldTransform);
            drawTransformedAxes(worldSpaceOrientation, 200.f);
            drawTransformedAxes(glm::mat4(1.f), 200.f);
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
                m_controllerView->SetLEDOverride(0, 0, 0);
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
                ImGui::Text("Testing Calibration of Controller ID #%d", m_controllerView->GetControllerID());
            }
            else
            {
                ImGui::Text("Calibration of Controller ID #%d complete!", m_controllerView->GetControllerID());
            }

			{
				const Eigen::Quaternionf eigen_quat = psmove_quaternion_to_eigen_quaternionf(m_controllerView->GetOrientation());
				const Eigen::EulerAnglesf euler_angles = eigen_quaternionf_to_euler_angles(eigen_quat);

				ImGui::Text("Pitch(x): %.2f, Yaw(y): %.2f, Roll(z): %.2f",
					m_lastCalibratedGyroscope.i * k_radians_to_degreees, 
					m_lastCalibratedGyroscope.j * k_radians_to_degreees,
					m_lastCalibratedGyroscope.k * k_radians_to_degreees);
				ImGui::Text("Attitude: %.2f, Heading: %.2f, Bank: %.2f", 
					euler_angles.get_attitude_degrees(), euler_angles.get_heading_degrees(), euler_angles.get_bank_degrees());
			}

			if (m_controllerView->GetControllerViewType() == ClientControllerView::PSDualShock4)
			{
				ImGui::TextWrapped(
					"[Press the Options button with controller pointed straight forward\n" \
					 "to recenter the controller]");
			}
			else if (m_controllerView->GetControllerViewType() == ClientControllerView::PSMove)
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
			switch (m_controllerView->GetControllerViewType())
			{
			case ClientControllerView::PSDualShock4:
				m_controllerView->GetPSDualShock4ViewMutable().SetPoseResetButtonEnabled(true);
				break;
			case ClientControllerView::PSMove:
				m_controllerView->GetPSMoveViewMutable().SetPoseResetButtonEnabled(true);
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

		ClientPSMoveAPI::register_callback(
			ClientPSMoveAPI::get_tracking_space_settings(),
			AppStage_GyroscopeCalibration::handle_tracking_space_settings_response, this);
	}
}

void AppStage_GyroscopeCalibration::handle_tracking_space_settings_response(
	const ClientPSMoveAPI::ResponseMessage *response_message,
	void *userdata)
{
	AppStage_GyroscopeCalibration *thisPtr = static_cast<AppStage_GyroscopeCalibration *>(userdata);

	switch (response_message->result_code)
	{
	case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
		{
			const AppStage_ControllerSettings *controllerSettings =
				thisPtr->m_app->getAppStage<AppStage_ControllerSettings>();
			const AppStage_ControllerSettings::ControllerInfo *controllerInfo =
				controllerSettings->getSelectedControllerInfo();

			assert(response_message->payload_type == ClientPSMoveAPI::_responsePayloadType_TrackingSpace);

			// Save the tracking space settings (used in rendering)
			thisPtr->m_global_forward_degrees = response_message->payload.tracking_space.global_forward_degrees;

			unsigned int stream_flags = 
				ClientPSMoveAPI::includeRawSensorData | 
				ClientPSMoveAPI::includeCalibratedSensorData;

			if (controllerInfo->ControllerType == ClientControllerView::PSDualShock4)
			{
				// Need to turn on optical tracking for the DS4 so that we have get optical orientation
				stream_flags |= ClientPSMoveAPI::includePositionData;
			}

			// Start streaming in controller data
			assert(!thisPtr->m_isControllerStreamActive);
			ClientPSMoveAPI::register_callback(
				ClientPSMoveAPI::start_controller_data_stream(
					thisPtr->m_controllerView,
					stream_flags),
				&AppStage_GyroscopeCalibration::handle_acquire_controller, thisPtr);

			thisPtr->setState(eCalibrationMenuState::waitingForStreamStartResponse);
		} break;

	case ClientPSMoveAPI::_clientPSMoveResultCode_error:
	case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
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

    calibration->set_controller_id(m_controllerView->GetControllerID());

    calibration->set_drift(drift);
    calibration->set_variance(variance);
	calibration->set_gyro_gain_setting(""); // keep existing gain

    ClientPSMoveAPI::eat_response(ClientPSMoveAPI::send_opaque_request(&request));
}

void AppStage_GyroscopeCalibration::handle_acquire_controller(
    const ClientPSMoveAPI::ResponseMessage *response,
    void *userdata)
{
    AppStage_GyroscopeCalibration *thisPtr = reinterpret_cast<AppStage_GyroscopeCalibration *>(userdata);

    if (response->result_code == ClientPSMoveAPI::_clientPSMoveResultCode_ok)
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
    ClientPSMoveAPI::eat_response(ClientPSMoveAPI::stop_controller_data_stream(m_controllerView));
    m_isControllerStreamActive= false;
    m_app->setAppStage(app_stage_name);
}

//-- private methods -----
static void drawController(ClientControllerView *controllerView, const glm::mat4 &transform)
{
    switch(controllerView->GetControllerViewType())
    {
    case ClientControllerView::PSMove:
        drawPSMoveModel(transform, glm::vec3(1.f, 1.f, 1.f));
        break;
    case ClientControllerView::PSDualShock4:
        drawPSDualShock4Model(transform, glm::vec3(1.f, 1.f, 1.f));
        break;
    }
}

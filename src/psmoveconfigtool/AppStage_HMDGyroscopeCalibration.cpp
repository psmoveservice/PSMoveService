//-- inludes -----
#include "AppStage_HMDGyroscopeCalibration.h"
#include "AppStage_HMDSettings.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "ClientHMDView.h"
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

#include <algorithm>

//-- statics ----
const char *AppStage_HMDGyroscopeCalibration::APP_STAGE_NAME = "HMDGyroscopeCalibration";

//-- constants -----
const double k_stabilize_wait_time_ms = 1000.f;
const int k_desired_noise_sample_count = 1000;
const float k_desired_drift_sampling_time = 30.0*1000.f; // milliseconds

//-- definitions -----
struct HMDGyroscopeErrorSamples
{
    PSMoveFloatVector3 raw_gyro_samples[k_desired_noise_sample_count];
	PSMoveFloatVector3 raw_gyro_bias;
    PSMoveFloatVector3 raw_total_gyro_drift;
    std::chrono::time_point<std::chrono::high_resolution_clock> sampleStartTime;
    int sample_count;

    float raw_variance; // Max sensor variance (raw_sensor_units/s/s for DS4, rad/s/s for PSMove)
    float raw_drift; // Max drift rate (raw_sensor_units/s for DS4, rad/s for PSMove)

    void clear()
    {
		raw_gyro_bias = PSMoveFloatVector3::create(0, 0, 0);
        raw_total_gyro_drift= PSMoveFloatVector3::create(0, 0, 0);
		raw_variance = 0.f;
		raw_drift = 0.f;
		sample_count= 0;
    }

    void computeStatistics(std::chrono::duration<float, std::milli> sampleDurationMilli)
    {
        const float sampleDurationSeconds= sampleDurationMilli.count() / 1000.f;
        const float N = static_cast<float>(sample_count);

        // Compute the mean of the error samples, where "error" = abs(omega_sample)
        // If we took the mean of the signed omega samples we'd get a value very 
        // close to zero since the the gyro at rest over a short period has mean-zero noise
        PSMoveFloatVector3 mean_gyro_abs_error= PSMoveFloatVector3::create(0.f, 0.f, 0.f);
		raw_gyro_bias = PSMoveFloatVector3::create(0.f, 0.f, 0.f);
        for (int sample_index = 0; sample_index < sample_count; sample_index++)
        {
			PSMoveFloatVector3 signed_error_sample = raw_gyro_samples[sample_index];
            PSMoveFloatVector3 unsigned_error_sample= signed_error_sample.abs();

            mean_gyro_abs_error= mean_gyro_abs_error + unsigned_error_sample;
			raw_gyro_bias = raw_gyro_bias + signed_error_sample;
        }
        mean_gyro_abs_error= mean_gyro_abs_error.unsafe_divide(N);
		raw_gyro_bias = raw_gyro_bias.unsafe_divide(N);

        // Compute the variance of the (unsigned) sample error, where "error" = abs(omega_sample)
        PSMoveFloatVector3 var_abs_error= PSMoveFloatVector3::create(0.f, 0.f, 0.f);
        for (int sample_index = 0; sample_index < sample_count; sample_index++)
        {
            PSMoveFloatVector3 unsigned_error_sample= raw_gyro_samples[sample_index].abs();
            PSMoveFloatVector3 diff_from_mean= unsigned_error_sample - mean_gyro_abs_error;

            var_abs_error= var_abs_error + diff_from_mean.square();
        }
        var_abs_error= var_abs_error.unsafe_divide(N - 1);

        // Use the max variance of all three axes (should be close)
        raw_variance= var_abs_error.maxValue();

        // Compute the max drift rate we got across a three axis
        PSMoveFloatVector3 drift_rate= raw_total_gyro_drift.unsafe_divide(sampleDurationSeconds);
        raw_drift= drift_rate.abs().maxValue();
    }
};

//-- private methods -----
static void drawHMD(ClientHMDView *hmdView, const glm::mat4 &transform);

//-- public methods -----
AppStage_HMDGyroscopeCalibration::AppStage_HMDGyroscopeCalibration(App *app)
    : AppStage(app)
    , m_menuState(AppStage_HMDGyroscopeCalibration::inactive)
    , m_bBypassCalibration(false)
    , m_hmdView(nullptr)
    , m_isHMDStreamActive(false)
    , m_lastHMDSeqNum(-1)
    , m_lastRawGyroscope()
    , m_errorSamples(new HMDGyroscopeErrorSamples)
{
}

AppStage_HMDGyroscopeCalibration::~AppStage_HMDGyroscopeCalibration()
{
    delete m_errorSamples;
}

void AppStage_HMDGyroscopeCalibration::enter()
{
    const AppStage_HMDSettings *hmdSettings = m_app->getAppStage<AppStage_HMDSettings>();
    const AppStage_HMDSettings::HMDInfo *hmdInfo = hmdSettings->getSelectedHmdInfo();

    // Reset the menu state
    m_app->setCameraType(_cameraOrbit);
    m_app->getOrbitCamera()->resetOrientation();
    m_app->getOrbitCamera()->setCameraOrbitRadius(1000.f); // zoom out to see the magnetometer data at scale

    m_menuState = eCalibrationMenuState::waitingForStreamStartResponse;

    // Reset all of the sampling state
    m_errorSamples->clear();

    // Initialize the hmd state
    assert(hmdInfo->HmdID != -1);
    assert(m_hmdView == nullptr);
    m_hmdView = ClientPSMoveAPI::allocate_hmd_view(hmdInfo->HmdID);

    m_lastRawGyroscope = *k_psmove_int_vector3_zero;
    m_lastHMDSeqNum = -1;

    m_lastCalibratedAccelerometer = *k_psmove_float_vector3_zero;
    m_lastCalibratedGyroscope = *k_psmove_float_vector3_zero;

    m_stableStartTime = std::chrono::time_point<std::chrono::high_resolution_clock>();
    m_bIsStable= false;

    // Start streaming in controller data
    assert(!m_isHMDStreamActive);
    ClientPSMoveAPI::register_callback(
        ClientPSMoveAPI::start_hmd_data_stream(
            m_hmdView, 
            ClientPSMoveAPI::includeRawSensorData | 
            ClientPSMoveAPI::includeCalibratedSensorData),
        &AppStage_HMDGyroscopeCalibration::handle_acquire_hmd, this);
}

void AppStage_HMDGyroscopeCalibration::exit()
{
    assert(m_hmdView != nullptr);
    ClientPSMoveAPI::free_hmd_view(m_hmdView);
    m_hmdView = nullptr;
    m_menuState = eCalibrationMenuState::inactive;

    // Reset the orbit camera back to default orientation and scale
    m_app->getOrbitCamera()->reset();
}

void AppStage_HMDGyroscopeCalibration::update()
{
    bool bControllerDataUpdatedThisFrame = false;
    bool bTimeDeltaValid = false;
    std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float, std::milli> sampleTimeDeltaMilli(0);

    if (m_isHMDStreamActive && m_hmdView->GetSequenceNum() != m_lastHMDSeqNum)
    {
        switch(m_hmdView->GetHmdViewType())
        {
        case ClientHMDView::Morpheus:
            {
                const MorpheusRawSensorData &rawSensorData =
                    m_hmdView->GetMorpheusView().GetRawSensorData();
                const MorpheusCalibratedSensorData &calibratedSensorData =
                    m_hmdView->GetMorpheusView().GetCalibratedSensorData();

                m_lastRawGyroscope = rawSensorData.Gyroscope;
                m_lastCalibratedGyroscope = calibratedSensorData.Gyroscope;
                m_lastCalibratedAccelerometer = calibratedSensorData.Accelerometer;
            }
            break;
        }

        m_lastHMDSeqNum = m_hmdView->GetSequenceNum();
        
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
    case eCalibrationMenuState::waitingForStreamStartResponse:
        {
            if (bControllerDataUpdatedThisFrame)
            {
                if (m_bBypassCalibration)
                {
                    m_app->getOrbitCamera()->resetOrientation();
                    m_menuState = AppStage_HMDGyroscopeCalibration::test;
                }
                else
                {
                    m_menuState = AppStage_HMDGyroscopeCalibration::waitForStable;
                }
            }
        } break;
    case eCalibrationMenuState::failedStreamStart:
        {
        } break;
    case eCalibrationMenuState::waitForStable:
        {
            if (m_hmdView->GetIsStable())
            {
                if (m_bIsStable)
                {
                    std::chrono::duration<double, std::milli> stableDuration = now - m_stableStartTime;
    
                    if (stableDuration.count() >= k_stabilize_wait_time_ms)
                    {
                        m_errorSamples->clear();
                        m_errorSamples->sampleStartTime= now;
                        m_menuState= eCalibrationMenuState::measureBiasAndDrift;
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
            if (m_hmdView->GetIsStable())
            {
                const std::chrono::duration<float, std::milli> sampleDurationMilli = now - m_errorSamples->sampleStartTime;
                const float deltaTimeSeconds= sampleTimeDeltaMilli.count()/1000.f;

                // Accumulate the drift total
                if (deltaTimeSeconds > 0.f)
                {
                    m_errorSamples->raw_total_gyro_drift= 
                        m_errorSamples->raw_total_gyro_drift
                        + m_lastRawGyroscope.castToFloatVector3()*deltaTimeSeconds;
                }

                // Record the next noise sample
                if (m_errorSamples->sample_count < k_desired_noise_sample_count)
                {
                    m_errorSamples->raw_gyro_samples[m_errorSamples->sample_count]= m_lastRawGyroscope.castToFloatVector3();
                    ++m_errorSamples->sample_count;
                }

                // See if we have completed the sampling period
                if (sampleDurationMilli.count() >= k_desired_drift_sampling_time)
                {
                    // Compute bias and drift statistics
                    m_errorSamples->computeStatistics(sampleDurationMilli);

                    // Update the gyro config on the service
                    request_set_gyroscope_calibration(
						m_errorSamples->raw_gyro_bias,
                        m_errorSamples->raw_drift, 
                        m_errorSamples->raw_variance);

                    m_menuState= eCalibrationMenuState::measureComplete;
                }
            }
            else
            {
                m_bIsStable= false;
                m_menuState = AppStage_HMDGyroscopeCalibration::waitForStable;
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

void AppStage_HMDGyroscopeCalibration::render()
{
	const float modelScale = 9.f;
	glm::mat4 hmdTransform;

	switch (m_hmdView->GetHmdViewType())
	{
	case ClientHMDView::Morpheus:
		hmdTransform = glm::scale(glm::mat4(1.f), glm::vec3(modelScale, modelScale, modelScale));
		break;
	}

    switch (m_menuState)
    {
    case eCalibrationMenuState::waitingForStreamStartResponse:
    case eCalibrationMenuState::failedStreamStart:
        {
        } break;
    case eCalibrationMenuState::waitForStable:
        {
            drawHMD(m_hmdView, hmdTransform);

            // Draw the current direction of gravity
            {
                const float renderScale = 200.f;
                glm::mat4 renderScaleMatrix = 
                    glm::scale(glm::mat4(1.f), glm::vec3(renderScale, renderScale, renderScale));
                glm::vec3 g= psmove_float_vector3_to_glm_vec3(m_lastCalibratedAccelerometer);

                drawArrow(
                    renderScaleMatrix,
                    glm::vec3(), g, 
                    0.1f, 
                    glm::vec3(0.f, 1.f, 0.f));
                drawTextAtWorldPosition(renderScaleMatrix, g, "G");
            }
        } break;
    case eCalibrationMenuState::measureBiasAndDrift:
	case eCalibrationMenuState::measureComplete:
		{
            drawHMD(m_hmdView, hmdTransform);
        } break;
    case eCalibrationMenuState::test:
        {
            // Get the orientation of the controller in world space (OpenGL Coordinate System)            
            glm::quat q= psmove_quaternion_to_glm_quat(m_hmdView->GetOrientation());
            glm::mat4 worldSpaceOrientation= glm::mat4_cast(q);
            glm::mat4 worldTransform = glm::scale(worldSpaceOrientation, glm::vec3(modelScale, modelScale, modelScale));

            drawHMD(m_hmdView, worldTransform);
            drawTransformedAxes(worldSpaceOrientation, 200.f);
            drawTransformedAxes(glm::mat4(1.f), 200.f);
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_HMDGyroscopeCalibration::renderUI()
{
    const float k_panel_width = 500;
    const char *k_window_title = "Gyroscope Calibration";
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

            ImGui::Text("Waiting for hmd stream to start...");

            ImGui::End();
        } break;
    case eCalibrationMenuState::failedStreamStart:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Failed to start hmd stream!");

            if (ImGui::Button("Ok"))
            {
                request_exit_to_app_stage(AppStage_HMDSettings::APP_STAGE_NAME);
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
                "Set the HMD down on a level surface.\n" \
                "Measurement will start once the hmd is aligned with gravity and stable.");

            if (m_bIsStable)
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

            if (ImGui::Button("Cancel"))
            {
                request_exit_to_app_stage(AppStage_HMDSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::measureBiasAndDrift:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            std::chrono::time_point<std::chrono::high_resolution_clock> now= std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> stableDuration = now - m_errorSamples->sampleStartTime;
            float timeFraction = static_cast<float>(stableDuration.count() / k_desired_drift_sampling_time);

            const float sampleFraction = 
                static_cast<float>(m_errorSamples->sample_count)
                / static_cast<float>(k_desired_noise_sample_count);

            ImGui::TextWrapped(
                "[Step 1 of 2: Measuring gyroscope drift and bias]\n" \
                "Sampling Gyroscope...");
            ImGui::ProgressBar(fminf(sampleFraction, timeFraction), ImVec2(250, 20));

            if (ImGui::Button("Cancel"))
            {
                request_exit_to_app_stage(AppStage_HMDSettings::APP_STAGE_NAME);
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
                m_menuState = eCalibrationMenuState::test;
            }
            ImGui::SameLine();
            if (ImGui::Button("Redo"))
            {
                m_errorSamples->clear();
                m_menuState = eCalibrationMenuState::waitForStable;
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel"))
            {
                request_exit_to_app_stage(AppStage_HMDSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::test:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 80));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            if (m_bBypassCalibration)
            {
                ImGui::Text("Testing Calibration of HMD ID #%d", m_hmdView->GetHmdID());
            }
            else
            {
                ImGui::Text("Calibration of HMD ID #%d complete!", m_hmdView->GetHmdID());
            }

            if (ImGui::Button("Ok"))
            {
                request_exit_to_app_stage(AppStage_HMDSettings::APP_STAGE_NAME);
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
void AppStage_HMDGyroscopeCalibration::request_set_gyroscope_calibration(
	const PSMoveFloatVector3 &raw_bias,
    const float raw_drift, 
    const float raw_variance)
{
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_HMD_GYROSCOPE_CALIBRATION);

    PSMoveProtocol::Request_RequestSetHMDGyroscopeCalibration *calibration =
        request->mutable_set_hmd_gyroscope_calibration_request();

    calibration->set_hmd_id(m_hmdView->GetHmdID());

	calibration->mutable_raw_bias()->set_i(raw_bias.i);
	calibration->mutable_raw_bias()->set_i(raw_bias.j);
	calibration->mutable_raw_bias()->set_i(raw_bias.k);
    calibration->set_raw_drift(raw_drift);
    calibration->set_raw_variance(raw_variance);

    ClientPSMoveAPI::eat_response(ClientPSMoveAPI::send_opaque_request(&request));
}

void AppStage_HMDGyroscopeCalibration::handle_acquire_hmd(
    const ClientPSMoveAPI::ResponseMessage *response,
    void *userdata)
{
    AppStage_HMDGyroscopeCalibration *thisPtr = reinterpret_cast<AppStage_HMDGyroscopeCalibration *>(userdata);

    if (response->result_code == ClientPSMoveAPI::_clientPSMoveResultCode_ok)
    {
        thisPtr->m_isHMDStreamActive = true;
        thisPtr->m_lastHMDSeqNum = -1;
        // Wait for the first controller packet to show up...
    }
    else
    {
        thisPtr->m_menuState = AppStage_HMDGyroscopeCalibration::failedStreamStart;
    }
}

void AppStage_HMDGyroscopeCalibration::request_exit_to_app_stage(const char *app_stage_name)
{
    ClientPSMoveAPI::eat_response(ClientPSMoveAPI::stop_hmd_data_stream(m_hmdView));
    m_isHMDStreamActive= false;
    m_app->setAppStage(app_stage_name);
}

//-- private methods -----
static void drawHMD(ClientHMDView *hmdView, const glm::mat4 &transform)
{
    switch(hmdView->GetHmdViewType())
    {
    case ClientHMDView::Morpheus:
        drawMorpheusModel(transform);
        break;
    }
}

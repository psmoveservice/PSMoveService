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

#include <algorithm>

//###HipsterSloth $NOTE Only used for trying to figure out raw gyro data scale
//#ifdef MEASURE_RAW_GYRO_SCALE

//-- statics ----
const char *AppStage_GyroscopeCalibration::APP_STAGE_NAME = "GyroscopeCalibration";

//-- constants -----
#define MILLI_PER_SECOND 1000
#define MICRO_PER_SECOND 1000000

const double k_stabilize_wait_time_ms = 1000.f;
const int k_desired_noise_sample_count = 1000;
const float k_desired_drift_sampling_time = 30.0*1000.f; // milliseconds

const int k_desired_scale_sample_count = 1000;

const double k_min_valid_scale_time_delta= 0.005; // seconds (200 fps)
const double k_max_valid_scale_time_delta= 0.05; // seconds (20 fps)

//-- definitions -----
struct GyroscopeErrorSamples
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

struct GyroscopeScaleSamples // DS4 only
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Eigen::Quaternionf lastOpticalOrientation;
    std::chrono::time_point<std::chrono::high_resolution_clock> lastSampleTime;
    bool lastSampleTimeValid;

    double scale_samples[k_desired_scale_sample_count];
    int sample_count;

    double raw_scale_mean;
    double raw_scale_variance;

    void clear()
    {
        lastSampleTimeValid= false;
        lastOpticalOrientation= Eigen::Quaternionf::Identity();
        sample_count= 0;
        raw_scale_mean= 0.f;
        raw_scale_variance= 0.f;
    }

    void computeStatistics()
    {
        const double N = static_cast<double>(sample_count);

        // Compute the mean of the samples
        raw_scale_mean= 0.f;
        for (int sample_index = 0; sample_index < sample_count; sample_index++)
        {
            raw_scale_mean+= scale_samples[sample_index];
        }
        raw_scale_mean/= N;

        // Compute the variance of the samples
        raw_scale_variance= 0.f;
        for (int sample_index = 0; sample_index < sample_count; sample_index++)
        {
            const double &sample= scale_samples[sample_index];
            double diff_from_mean= sample - raw_scale_mean;

            raw_scale_variance+= diff_from_mean*diff_from_mean;
        }
        raw_scale_variance/= N - 1.f;
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
    , m_errorSamples(new GyroscopeErrorSamples)
    , m_scaleSamples(new GyroscopeScaleSamples)
{
}

AppStage_GyroscopeCalibration::~AppStage_GyroscopeCalibration()
{
    delete m_errorSamples;
    delete m_scaleSamples;
}

void AppStage_GyroscopeCalibration::enter()
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

    // Reset all of the sampling state
    m_errorSamples->clear();
    m_scaleSamples->clear();

    // Initialize the controller state
    assert(controllerInfo->ControllerID != -1);
    assert(m_controllerView == nullptr);
    m_controllerView = ClientPSMoveAPI::allocate_controller_view(controllerInfo->ControllerID);

    m_lastRawGyroscope = *k_psmove_int_vector3_zero;
    m_lastControllerSeqNum = -1;

    m_lastCalibratedAccelerometer = *k_psmove_float_vector3_zero;
    m_lastCalibratedGyroscope = *k_psmove_float_vector3_zero;

    m_stableStartTime = std::chrono::time_point<std::chrono::high_resolution_clock>();
    m_bIsStable= false;

    // Start streaming in controller data
    assert(!m_isControllerStreamActive);
    ClientPSMoveAPI::register_callback(
        ClientPSMoveAPI::start_controller_data_stream(
            m_controllerView, 
            ClientPSMoveAPI::includePositionData | 
            ClientPSMoveAPI::includeRawSensorData | 
            ClientPSMoveAPI::includeCalibratedSensorData |
            ClientPSMoveAPI::includeRawTrackerData),
        &AppStage_GyroscopeCalibration::handle_acquire_controller, this);
}

void AppStage_GyroscopeCalibration::exit()
{
    assert(m_controllerView != nullptr);
    ClientPSMoveAPI::free_controller_view(m_controllerView);
    m_controllerView = nullptr;
    m_menuState = eCalibrationMenuState::inactive;

    // Reset the orbit camera back to default orientation and scale
    m_app->getOrbitCamera()->reset();
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
    case eCalibrationMenuState::waitingForStreamStartResponse:
        {
            if (bControllerDataUpdatedThisFrame)
            {
                if (m_bBypassCalibration)
                {
                    m_app->getOrbitCamera()->resetOrientation();
                    m_menuState = AppStage_GyroscopeCalibration::test;
                }
                else
                {
                    m_menuState = AppStage_GyroscopeCalibration::waitForStable;
                }
            }
        } break;
    case eCalibrationMenuState::failedStreamStart:
        {
        } break;
    case eCalibrationMenuState::waitForStable:
        {
            if (m_controllerView->GetIsStable())
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
            if (m_controllerView->GetIsStable())
            {
                const std::chrono::duration<float, std::milli> sampleDurationMilli = now - m_errorSamples->sampleStartTime;
                const float deltaTimeSeconds= sampleTimeDeltaMilli.count()/1000.f;

                // Accumulate the drift total
                if (deltaTimeSeconds > 0.f)
                {
                    m_errorSamples->drift_rotation= 
                        m_errorSamples->drift_rotation
                        + m_lastCalibratedGyroscope*deltaTimeSeconds;
                }

                // Record the next noise sample
                if (m_errorSamples->sample_count < k_desired_noise_sample_count)
                {
                    m_errorSamples->omega_samples[m_errorSamples->sample_count]= m_lastCalibratedGyroscope;
                    ++m_errorSamples->sample_count;
                }

                // See if we have completed the sampling period
                if (sampleDurationMilli.count() >= k_desired_drift_sampling_time)
                {
                    // Compute bias and drift statistics
                    m_errorSamples->computeStatistics(sampleDurationMilli);

                    // Start measuring the gyro scale
                    m_scaleSamples->clear();

                    // Update the gyro config on the service
                    request_set_gyroscope_calibration(
                        m_errorSamples->drift, 
                        m_errorSamples->variance);

                    #ifdef MEASURE_RAW_GYRO_SCALE
                    if (m_controllerView->GetControllerViewType() == ClientControllerView::PSDualShock4)
                    {
                        m_menuState= eCalibrationMenuState::measureScale;
                    }
                    else
                    #endif
                    {
                        m_menuState= eCalibrationMenuState::measureComplete;
                    }
                }
            }
            else
            {
                m_bIsStable= false;
                m_menuState = AppStage_GyroscopeCalibration::waitForStable;
            }
        } break;
    case eCalibrationMenuState::measureScale: // DS4 only
        {
            PSMoveQuaternion orientationOnTracker;

            assert(m_controllerView->GetControllerViewType() == ClientControllerView::PSDualShock4);

            if (!m_controllerView->GetIsStable() &&
                m_controllerView->GetIsCurrentlyTracking() &&
                m_controllerView->GetRawTrackerData().GetOrientationOnTrackerId(0, orientationOnTracker) &&
                m_scaleSamples->sample_count < k_desired_scale_sample_count)
            {
                std::chrono::time_point<std::chrono::high_resolution_clock> now = 
                    std::chrono::high_resolution_clock::now();
                const Eigen::Quaternionf orientation= 
                    psmove_quaternion_to_eigen_quaternionf(orientationOnTracker);

                if (m_scaleSamples->lastSampleTimeValid)
                {
                    std::chrono::duration<double, std::micro> sampleDurationMicro = 
                        now - m_scaleSamples->lastSampleTime;
                    double sampleDurationSeconds= sampleDurationMicro.count() / MICRO_PER_SECOND;

                    if (sampleDurationSeconds >= k_min_valid_scale_time_delta &&
                        sampleDurationSeconds <= k_max_valid_scale_time_delta)
                    {
                        const double optical_radians_between= 
                            eigen_quaternion_unsigned_angle_between(
                                m_scaleSamples->lastOpticalOrientation,
                                orientation);
                        const double optical_angular_speed= optical_radians_between / sampleDurationSeconds;
                        
                        // This assumes that the the length of the gyro vector
                        // (gyro vector = angular rates on each axis)
                        // equals the angular speed about the overall rotation axis 
                        // of the controller, which isn't strictly true
                        const float raw_gyro_speed= m_lastRawGyroscope.castToFloatVector3().length();

                        if (!is_nearly_zero(raw_gyro_speed))
                        {
                            const double raw_gyro_scale= optical_angular_speed / static_cast<double>(raw_gyro_speed);

                            m_scaleSamples->scale_samples[m_scaleSamples->sample_count]= raw_gyro_scale;
                            ++m_scaleSamples->sample_count;

                            if (m_scaleSamples->sample_count >= k_desired_scale_sample_count)
                            {
                                // Compute the scale statistics
                                m_scaleSamples->computeStatistics();

                                m_menuState= eCalibrationMenuState::measureComplete;
                            }
                        }
                    }
                }

                m_scaleSamples->lastSampleTime= now;
                m_scaleSamples->lastOpticalOrientation= 
                    psmove_quaternion_to_eigen_quaternionf(orientationOnTracker);
                m_scaleSamples->lastSampleTimeValid= true;
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
    const float modelScale = 18.f;
    glm::mat4 scaleAndRotateModelX90= 
        glm::rotate(
            glm::scale(glm::mat4(1.f), glm::vec3(modelScale, modelScale, modelScale)),
            90.f, glm::vec3(1.f, 0.f, 0.f));  

    switch (m_menuState)
    {
    case eCalibrationMenuState::waitingForStreamStartResponse:
    case eCalibrationMenuState::failedStreamStart:
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
        {
            drawController(m_controllerView, scaleAndRotateModelX90);
        } break;
    case eCalibrationMenuState::measureScale:
        {
            PSMoveQuaternion orientationOnTracker;
            
            // Show the optically derived orientation
            if (m_controllerView->GetIsCurrentlyTracking() &&
                m_controllerView->GetRawTrackerData().GetOrientationOnTrackerId(0, orientationOnTracker))
            {
                // Get the orientation of the controller in world space (OpenGL Coordinate System)            
                glm::quat q= psmove_quaternion_to_glm_quat(orientationOnTracker);
                glm::mat4 trackerSpaceOrientation= glm::mat4_cast(q);
                glm::mat4 trackerSpaceTransform = glm::scale(trackerSpaceOrientation, glm::vec3(modelScale, modelScale, modelScale));

                drawController(m_controllerView, trackerSpaceTransform);
            }
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
            glm::mat4 worldTransform = glm::scale(worldSpaceOrientation, glm::vec3(modelScale, modelScale, modelScale));

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
    case eCalibrationMenuState::waitForStable:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::TextWrapped(
                "[Step 1 of 2: Measuring gyroscope drift and bias]\n" \
                "Set the controller down on a level surface.\n" \
                "Measurement will start once the controller is aligned with gravity and stable.");

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
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::measureScale:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::TextWrapped(
                "[Step 2 of 2: Computing gyroscope sensor scale]\n" \
                "Pick up the controller and smoothly twist it around\n" \
                "with the light bar in view of the camera.");

            if (m_controllerView->GetIsStable())
            {
                ImGui::Text("[Controller stationary]");
            }
            else if (!m_controllerView->GetIsCurrentlyTracking())
            {
                ImGui::Text("[Light bar not in view of tracker 0]");
            }
            else
            {
                ImGui::Text("Sampling gyroscope scale...");                
            }

            const float sampleFraction = 
                static_cast<float>(m_scaleSamples->sample_count)
                / static_cast<float>(k_desired_scale_sample_count);
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
                "Press OK to continue or Redo to recalibration.");

            if (ImGui::Button("Ok"))
            {
                m_controllerView->SetLEDOverride(0, 0, 0);
                m_menuState = eCalibrationMenuState::test;
            }
            ImGui::SameLine();
            if (ImGui::Button("Redo"))
            {
                m_errorSamples->clear();
                m_scaleSamples->clear();
                m_menuState = eCalibrationMenuState::waitForStable;
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
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 80));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            if (m_bBypassCalibration)
            {
                ImGui::Text("Testing Calibration of Controller ID #%d", m_controllerView->GetControllerID());
            }
            else
            {
                ImGui::Text("Calibration of Controller ID #%d complete!", m_controllerView->GetControllerID());
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
        thisPtr->m_menuState = AppStage_GyroscopeCalibration::failedStreamStart;
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

//-- inludes -----
#include "AppStage_AccelerometerCalibration.h"
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

//-- statics ----
const char *AppStage_AccelerometerCalibration::APP_STAGE_NAME = "AcceleromterCalibration";

//-- constants -----
static const double k_stabilize_wait_time_ms = 1000.f;
static const int k_max_accelerometer_samples = 500;

static const float k_min_sample_distance = 1000.f;
static const float k_min_sample_distance_sq = k_min_sample_distance*k_min_sample_distance;

//-- definitions -----
struct AccelerometerPoseSamples
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PSMoveFloatVector3 accelerometer_samples[k_max_accelerometer_samples];
    PSMoveFloatVector3 avg_accelerometer_sample;
    float noise_radius;
    int sample_count;

    void clear()
    {        
        sample_count= 0;
        noise_radius= 0.f;
    }

    void computeStatistics()
    {
        avg_accelerometer_sample = PSMoveFloatVector3::create(0.f, 0.f, 0.f);
        for (int sample_index= 0; sample_index < k_max_accelerometer_samples; ++sample_index)
        {
            avg_accelerometer_sample= avg_accelerometer_sample + accelerometer_samples[sample_index];
        }
        avg_accelerometer_sample= avg_accelerometer_sample.unsafe_divide(static_cast<float>(k_max_accelerometer_samples));

        noise_radius= 0;
        for (int sample_index= 0; sample_index < k_max_accelerometer_samples; ++sample_index)
        {
            PSMoveFloatVector3 error= accelerometer_samples[sample_index] - avg_accelerometer_sample;

            noise_radius= fmaxf(noise_radius, error.length());
        }
    }
};

//-- private methods -----
static void request_set_accelerometer_calibration(
    const int controller_id,
    const float noise_radius);
static void drawController(ClientControllerView *controllerView, const glm::mat4 &transform);

//-- public methods -----
AppStage_AccelerometerCalibration::AppStage_AccelerometerCalibration(App *app)
    : AppStage(app)
    , m_menuState(AppStage_AccelerometerCalibration::inactive)
    , m_bBypassCalibration(false)
    , m_controllerView(nullptr)
    , m_isControllerStreamActive(false)
    , m_lastControllerSeqNum(-1)
    , m_noiseSamples(new AccelerometerPoseSamples)
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
    m_controllerView = ClientPSMoveAPI::allocate_controller_view(controllerInfo->ControllerID);

    m_lastCalibratedAccelerometer = *k_psmove_float_vector3_zero;
    m_lastControllerSeqNum = -1;

    // Start streaming in controller data
    assert(!m_isControllerStreamActive);
    ClientPSMoveAPI::register_callback(
        ClientPSMoveAPI::start_controller_data_stream(
            m_controllerView, 
            ClientPSMoveAPI::includeCalibratedSensorData | 
            ClientPSMoveAPI::includePhysicsData |
            ClientPSMoveAPI::includePositionData), // Needed so linear acceleration is computed
        &AppStage_AccelerometerCalibration::handle_acquire_controller, this);
}

void AppStage_AccelerometerCalibration::exit()
{
    assert(m_controllerView != nullptr);
    ClientPSMoveAPI::free_controller_view(m_controllerView);
    m_controllerView = nullptr;
    m_menuState = eCalibrationMenuState::inactive;

    // Reset the orbit camera back to default orientation and scale
    m_app->getOrbitCamera()->reset();
}

void AppStage_AccelerometerCalibration::update()
{
    bool bControllerDataUpdatedThisFrame = false;

    if (m_isControllerStreamActive && m_controllerView->GetOutputSequenceNum() != m_lastControllerSeqNum)
    {
        const PSMovePhysicsData &physicsData= m_controllerView->GetPhysicsData();

        switch(m_controllerView->GetControllerViewType())
        {
        case ClientControllerView::eControllerType::PSDualShock4:
            {
                const PSDualShock4CalibratedSensorData &calibratedSensorData =
                    m_controllerView->GetPSDualShock4View().GetCalibratedSensorData();

                m_lastCalibratedAccelerometer = calibratedSensorData.Accelerometer;
            } break;
        case ClientControllerView::eControllerType::PSMove:
            {
                const PSMoveCalibratedSensorData &calibratedSensorData =
                    m_controllerView->GetPSMoveView().GetCalibratedSensorData();

                m_lastCalibratedAccelerometer = calibratedSensorData.Accelerometer;
            } break;
        default:
            assert(0 && "unreachable");
        }

        m_lastAcceleration= physicsData.Acceleration;
        m_lastVelocity= physicsData.Velocity;
        m_lastControllerSeqNum = m_controllerView->GetOutputSequenceNum();
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
                m_noiseSamples->accelerometer_samples[m_noiseSamples->sample_count] = m_lastCalibratedAccelerometer;
                ++m_noiseSamples->sample_count;

                // See if we filled all of the samples for this pose
                if (m_noiseSamples->sample_count >= k_max_accelerometer_samples)
                {
                    // Compute the average gravity value in this pose.
                    // This assumes that the acceleration noise has a Gaussian distribution.
                    m_noiseSamples->computeStatistics();

                    // Tell the service what the new calibration constraints are
                    request_set_accelerometer_calibration(
                        m_controllerView->GetControllerID(),
                        m_noiseSamples->noise_radius);

                    m_menuState = AppStage_AccelerometerCalibration::measureComplete;
                }
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

void AppStage_AccelerometerCalibration::render()
{
    const float modelScale = 18.f;
    glm::mat4 controllerTransform = glm::scale(glm::mat4(1.f), glm::vec3(modelScale, modelScale, modelScale));

    switch (m_menuState)
    {
    case eCalibrationMenuState::waitingForStreamStartResponse:
    case eCalibrationMenuState::failedStreamStart:
        {
        } break;
    case eCalibrationMenuState::placeController:
        {
            // Draw the controller model in the pose we want the user place it in
            drawController(m_controllerView, controllerTransform);
        } break;
    case eCalibrationMenuState::measureNoise:
    case eCalibrationMenuState::measureComplete:
        {
            const float sampleScale = 100.f;
            glm::mat4 sampleTransform = glm::scale(glm::mat4(1.f), glm::vec3(sampleScale, sampleScale, sampleScale));

            // Draw the controller in the middle            
            drawController(m_controllerView, controllerTransform);

            // Draw the sample point cloud around the origin
            drawPointCloud(sampleTransform, glm::vec3(1.f, 1.f, 1.f), 
                reinterpret_cast<float *>(m_noiseSamples->accelerometer_samples), 
                m_noiseSamples->sample_count);

            // Draw the current raw accelerometer direction
            {
                glm::vec3 m_start = glm::vec3(0.f, 0.f, 0.f);
                glm::vec3 m_end = psmove_float_vector3_to_glm_vec3(m_lastCalibratedAccelerometer);

                drawArrow(sampleTransform, m_start, m_end, 0.1f, glm::vec3(1.f, 0.f, 0.f));
                drawTextAtWorldPosition(sampleTransform, m_end, "A");
            }
        } break;
    case eCalibrationMenuState::test:
        {
            const float sampleScale = 1.f;
            glm::mat4 sampleTransform = glm::scale(glm::mat4(1.f), glm::vec3(sampleScale, sampleScale, sampleScale));

            drawController(m_controllerView, controllerTransform);
            drawTransformedAxes(controllerTransform, 200.f);

            // Draw the current filtered acceleration direction
            {
                const float accel_cms2 = m_lastAcceleration.length();
                glm::vec3 m_start = glm::vec3(0.f);
                glm::vec3 m_end = psmove_float_vector3_to_glm_vec3(m_lastAcceleration);

                drawArrow(sampleTransform, m_start, m_end, 0.1f, glm::vec3(1.f, 0.f, 0.f));
                drawTextAtWorldPosition(sampleTransform, m_end, "A(%.1fcm/s^2)", accel_cms2);
            }

            // Draw the current filtered acceleration direction
            {
                const float vel_cms = m_lastVelocity.length();
                glm::vec3 m_start = glm::vec3(0.f);
                glm::vec3 m_end = psmove_float_vector3_to_glm_vec3(m_lastVelocity);

                drawArrow(sampleTransform, m_start, m_end, 0.1f, glm::vec3(0.f, 1.f, 0.f));
                drawTextAtWorldPosition(sampleTransform, m_end, "V(%.1fcm/s)", vel_cms);
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

            ImGui::Text("Lay the controller flat on the table face up");

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
                m_controllerView->SetLEDOverride(0, 0, 0);
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
static void request_set_accelerometer_calibration(
    const int controller_id,
    const float noise_radius)
{
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_ACCELEROMETER_CALIBRATION);

    PSMoveProtocol::Request_RequestSetAccelerometerCalibration *calibration =
        request->mutable_set_accelerometer_calibration_request();

    calibration->set_controller_id(controller_id);
    calibration->set_noise_radius(noise_radius);

    ClientPSMoveAPI::eat_response(ClientPSMoveAPI::send_opaque_request(&request));
}

void AppStage_AccelerometerCalibration::handle_acquire_controller(
    const ClientPSMoveAPI::ResponseMessage *response,
    void *userdata)
{
    AppStage_AccelerometerCalibration *thisPtr = reinterpret_cast<AppStage_AccelerometerCalibration *>(userdata);

    if (response->result_code == ClientPSMoveAPI::_clientPSMoveResultCode_ok)
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

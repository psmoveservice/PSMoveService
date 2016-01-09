//-- inludes -----
#include "AppStage_MagnetometerCalibration.h"
#include "AppStage_ControllerSettings.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "ClientPSMoveAPI.h"
#include "ClientControllerView.h"
#include "Logger.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"

#include "SDL_keycode.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>

//-- statics ----
const char *AppStage_MagnetometerCalibration::APP_STAGE_NAME= "MagnetometerCalibration";

//-- constants -----

//-- private methods -----
static bool is_move_stable_and_aligned_with_gravity(ClientControllerView *m_controllerView);

//-- public methods -----
AppStage_MagnetometerCalibration::AppStage_MagnetometerCalibration(App *app) 
    : AppStage(app)
    , m_menuState(AppStage_MagnetometerCalibration::inactive)
    , m_pendingAppStage(nullptr)
    , m_controllerView(nullptr)
    , m_isControllerStreamActive(false)
    , m_lastControllerSeqNum(-1)
{ }

void AppStage_MagnetometerCalibration::enter()
{
    const AppStage_ControllerSettings *controllerSettings= 
        m_app->getAppStage<AppStage_ControllerSettings>();
    const AppStage_ControllerSettings::ControllerInfo *controllerInfo=
        controllerSettings->getSelectedControllerInfo();

    m_app->setCameraType(_cameraFixed);

    assert(controllerInfo->ControllerID != -1);
    assert(m_controllerView == nullptr);
    m_controllerView= ClientPSMoveAPI::allocate_controller_view(controllerInfo->ControllerID);

    assert(!m_isControllerStreamActive);
    ClientPSMoveAPI::start_controller_data_stream(
        m_controllerView, 
        &AppStage_MagnetometerCalibration::handle_acquire_controller, 
        this);
}

void AppStage_MagnetometerCalibration::exit()
{
    assert(m_controllerView != nullptr);
    ClientPSMoveAPI::free_controller_view(m_controllerView);
    m_controllerView= nullptr;
}

void AppStage_MagnetometerCalibration::update()
{
    bool bControllerDataUpdatedThisFrame= false;

    if (m_isControllerStreamActive && m_controllerView->GetSequenceNum() != m_lastControllerSeqNum)
    {
        m_lastControllerSeqNum= m_controllerView->GetSequenceNum();
        bControllerDataUpdatedThisFrame= true;
    }

    switch (m_menuState)
    {
    case eCalibrationMenuState::waitingForStreamStartResponse:
        {
            if (bControllerDataUpdatedThisFrame)
            {
                if (m_controllerView->GetPSMoveView().GetHasValidHardwareCalibration())
                {
                    m_menuState= AppStage_MagnetometerCalibration::measureBExtents;
                }
                else
                {
                    m_menuState= AppStage_MagnetometerCalibration::failedBadCalibration;
                }
            }
        } break;
    case eCalibrationMenuState::failedStreamStart:
    case eCalibrationMenuState::failedBadCalibration:
        {
        } break;
    case eCalibrationMenuState::measureBExtents:
        {
        } break;
    case eCalibrationMenuState::waitForGravityAlignment:
        {
        } break;
    case eCalibrationMenuState::measureBDirection:
        {
        } break;
    case eCalibrationMenuState::complete:
        {
        } break;
    case eCalibrationMenuState::pendingExit:
        {
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_MagnetometerCalibration::render()
{
    glm::mat4 scale3= glm::scale(glm::mat4(1.f), glm::vec3(3.f, 3.f, 3.f));

    switch (m_menuState)
    {
    case eCalibrationMenuState::waitingForStreamStartResponse:
        {
        } break;
    case eCalibrationMenuState::failedStreamStart:
    case eCalibrationMenuState::failedBadCalibration:
        {
        } break;
    case eCalibrationMenuState::measureBExtents:
        {
            drawPSMoveModel(scale3, glm::vec3(1.f, 1.f, 1.f));
        } break;
    case eCalibrationMenuState::waitForGravityAlignment:
        {
            drawPSMoveModel(scale3, glm::vec3(1.f, 1.f, 1.f));
        } break;
    case eCalibrationMenuState::measureBDirection:
        {
            drawPSMoveModel(scale3, glm::vec3(1.f, 1.f, 1.f));
        } break;
    case eCalibrationMenuState::complete:
        {
        } break;
    case eCalibrationMenuState::pendingExit:
        {
        } break;
    default:
        assert(0 && "unreachable");
    }

}

void AppStage_MagnetometerCalibration::renderUI()
{
    const float k_panel_width= 300;
    const char *k_window_title= "Controller Settings";
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
            ImGui::Begin(k_window_title, nullptr, ImVec2(k_panel_width, 150), k_background_alpha, window_flags);

            ImGui::Text("Waiting for controller stream to start...");

            ImGui::End();
        } break;
    case eCalibrationMenuState::failedStreamStart:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, ImVec2(k_panel_width, 150), k_background_alpha, window_flags);

            ImGui::Text("Failed to start controller stream!");

            if (ImGui::Button("Ok"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            if (ImGui::Button("Return to Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::failedBadCalibration:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, ImVec2(k_panel_width, 150), k_background_alpha, window_flags);

            ImGui::TextWrapped(
                "Bad controller hardware calibration!\n" \
                "Try un-pairing and re-pairing the controller.");

            if (ImGui::Button("Ok"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            if (ImGui::Button("Return to Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::measureBExtents:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x/2.f - k_panel_width/2.f, 20.f));
            ImGui::Begin(k_window_title, nullptr, ImVec2(k_panel_width, 150), k_background_alpha, window_flags);

            ImGui::TextWrapped(
                "Calibrating Controller ID #%d\n" \
                "[Step 1 of 2: Measuring extents of the magnetometer]\n" \
                "Rotate the controller in all directions.", m_controllerView->GetControllerID());

            if (ImGui::Button("Cancel"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::waitForGravityAlignment:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x/2.f - k_panel_width/2.f, 20.f));
            ImGui::Begin(k_window_title, nullptr, ImVec2(k_panel_width, 150), k_background_alpha, window_flags);

            ImGui::TextWrapped(
                "[Step 2 of 2: Measuring reference magnetic field direction]\n" \
                "Stand the controller on a level surface with the Move button facing you.\n" \
                "This will be the default orientation of the move controller.\n" \
                "Measurement will start once the controller is aligned with gravity and stable.");

            if (ImGui::Button("Cancel"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::measureBDirection:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x/2.f - k_panel_width/2.f, 20.f));
            ImGui::Begin(k_window_title, nullptr, ImVec2(k_panel_width, 150), k_background_alpha, window_flags);

            ImGui::TextWrapped(
                "[Step 2 of 2: Measuring reference magnetic field direction]\n" \
                "Stand the controller on a level surface with the Move button facing you.\n"
                "This will be the default orientation of the move controller.\n"
                "Measurement will start once the controller is aligned with gravity and stable.");

            if (ImGui::Button("Cancel"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::complete:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, ImVec2(k_panel_width, 150), k_background_alpha, window_flags);

            ImGui::Text("Calibration of Controller ID #%d complete!", m_controllerView->GetControllerID());

            if (ImGui::Button("Ok"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            if (ImGui::Button("Return to Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::pendingExit:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, ImVec2(k_panel_width, 150), k_background_alpha, window_flags);

            ImGui::Text("Waiting for controller stream to stop...");

            ImGui::End();
        } break;
    default:
        assert(0 && "unreachable");
    }
}

//-- private methods -----
void AppStage_MagnetometerCalibration::handle_acquire_controller(
    ClientPSMoveAPI::eClientPSMoveResultCode resultCode,
    const ClientPSMoveAPI::t_request_id request_id, 
    ClientPSMoveAPI::t_response_handle opaque_response_handle,
    void *userdata)
{
    AppStage_MagnetometerCalibration *thisPtr= reinterpret_cast<AppStage_MagnetometerCalibration *>(userdata);

    if (resultCode == ClientPSMoveAPI::_clientPSMoveResultCode_ok)
    {
        assert(thisPtr->m_controllerView->GetControllerViewType() == ClientControllerView::PSMove);
        thisPtr->m_isControllerStreamActive= true;
        thisPtr->m_lastControllerSeqNum= -1;
    }
    else
    {
        thisPtr->m_menuState= AppStage_MagnetometerCalibration::failedStreamStart;
    }
}

void AppStage_MagnetometerCalibration::request_exit_to_app_stage(const char *app_stage_name)
{
    if (m_pendingAppStage == nullptr)
    {
        if (m_isControllerStreamActive)
        {
            m_pendingAppStage= app_stage_name;
            ClientPSMoveAPI::stop_controller_data_stream(
                m_controllerView, 
                &AppStage_MagnetometerCalibration::handle_acquire_controller, 
                this);
        }
        else
        {
            m_app->setAppStage(app_stage_name);
        }
    }
}

void AppStage_MagnetometerCalibration::handle_release_controller(
    ClientPSMoveAPI::eClientPSMoveResultCode resultCode,
    const ClientPSMoveAPI::t_request_id request_id, 
    ClientPSMoveAPI::t_response_handle opaque_response_handle,
    void *userdata)
{
    AppStage_MagnetometerCalibration *thisPtr= reinterpret_cast<AppStage_MagnetometerCalibration *>(userdata);

    if (resultCode != ClientPSMoveAPI::_clientPSMoveResultCode_ok)
    {
        Log_ERROR("AppStage_MagnetometerCalibration", "Failed to release controller on server!");
    }

    thisPtr->m_isControllerStreamActive= false;
    thisPtr->m_app->setAppStage(AppStage_ControllerSettings::APP_STAGE_NAME);
}

//-- private methods -----
static bool
is_move_stable_and_aligned_with_gravity(
    ClientControllerView *m_controllerView)
{
    const float k_cosine_10_degrees = 0.984808f;

    // Get the direction the gravity vector should be pointing 
    // while the controller is in cradle pose.
    const PSMoveFloatVector3 &gravity= m_controllerView->GetPSMoveView().GetIdentityGravityCalibrationDirection();
    glm::vec3 k_identity_gravity_vector= glm::vec3(gravity.i, gravity.j, gravity.k);;

    // Get the current acceleration reading from the controller
    const PSMoveFloatVector3 &acceleration= m_controllerView->GetPSMoveView().GetRawSensorData().Accelerometer;
    glm::vec3 acceleration_direction= glm::vec3(acceleration.i, acceleration.j, acceleration.k);
    const float acceleration_magnitude= glm_vec3_normalize_with_default(acceleration_direction, glm::vec3());

    const bool isOk =
        is_nearly_equal(1.f, acceleration_magnitude, 0.1f) &&
        glm::dot(k_identity_gravity_vector, acceleration_direction) >= k_cosine_10_degrees;

    return isOk;
}
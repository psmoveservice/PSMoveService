//-- inludes -----
#include "AppStage_TestRumble.h"
#include "AppStage_ControllerSettings.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "ClientPSMoveAPI.h"
#include "ClientControllerView.h"
#include "Logger.h"
#include "MathGLM.h"
#include "Renderer.h"
#include "UIConstants.h"

#include <imgui.h>

//-- statics ----
const char *AppStage_TestRumble::APP_STAGE_NAME = "TestRumble";

//-- constants -----


//-- private methods -----
static void drawController(ClientControllerView *controllerView, const glm::mat4 &transform, const glm::vec3 &bulb_color);

//-- public methods -----
AppStage_TestRumble::AppStage_TestRumble(App *app)
    : AppStage(app)
    , m_menuState(AppStage_TestRumble::inactive)
    , m_controllerView(nullptr)
{
}

AppStage_TestRumble::~AppStage_TestRumble()
{
}

void AppStage_TestRumble::enter()
{
    const AppStage_ControllerSettings *controllerSettings =
        m_app->getAppStage<AppStage_ControllerSettings>();
    const AppStage_ControllerSettings::ControllerInfo *controllerInfo =
        controllerSettings->getSelectedControllerInfo();

    m_app->setCameraType(_cameraOrbit);
    m_app->getOrbitCamera()->resetOrientation();
    m_app->getOrbitCamera()->setCameraOrbitRadius(1000.f); // zoom out to see the magnetometer data at scale

    assert(controllerInfo->ControllerID != -1);
    assert(m_controllerView == nullptr);
    m_controllerView = ClientPSMoveAPI::allocate_controller_view(controllerInfo->ControllerID);

    m_menuState = eMenuState::waitingForStreamStartResponse;

    ClientPSMoveAPI::register_callback(
        ClientPSMoveAPI::start_controller_data_stream(m_controllerView, ClientPSMoveAPI::defaultStreamOptions),
        &AppStage_TestRumble::handle_acquire_controller, this);
}

void AppStage_TestRumble::exit()
{
    assert(m_controllerView != nullptr);
    ClientPSMoveAPI::free_controller_view(m_controllerView);
    m_controllerView = nullptr;
    m_menuState = eMenuState::inactive;

    // Reset the orbit camera back to default orientation and scale
    m_app->getOrbitCamera()->reset();
}

void AppStage_TestRumble::update()
{
    switch (m_menuState)
    {
    case eMenuState::waitingForStreamStartResponse:
        {
        } break;
    case eMenuState::failedStreamStart:
        {
        } break;
    case eMenuState::idle:
        {
            set_rumble_amounts(get_left_trigger(), get_right_trigger());
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_TestRumble::render()
{
    const float modelScale = 18.f;
    glm::mat4 scaleAndRotateModelX90 =
        glm::rotate(
        glm::scale(glm::mat4(1.f), glm::vec3(modelScale, modelScale, modelScale)),
        90.f, glm::vec3(1.f, 0.f, 0.f));

    switch (m_menuState)
    {
    case eMenuState::waitingForStreamStartResponse:
        {
        } break;
    case eMenuState::failedStreamStart:
        {
        } break;
    case eMenuState::idle:
        {
            float bigRumbleAmount = get_big_rumble_amount();
			float smallRumbleAmount = get_small_rumble_amount();

            // Draw the psmove model in the middle
			const float red= fmaxf(bigRumbleAmount, smallRumbleAmount);
			drawController(m_controllerView, scaleAndRotateModelX90, glm::vec3(red, 0.f, 0.f));
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_TestRumble::renderUI()
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
    case eMenuState::waitingForStreamStartResponse:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Waiting for controller stream to start...");

            ImGui::End();
        } break;
    case eMenuState::failedStreamStart:
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
    case eMenuState::idle:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Press trigger to test rumble");

            if (ImGui::Button("Return to Controller Settings"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

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
void AppStage_TestRumble::handle_acquire_controller(
    const ClientPSMoveAPI::ResponseMessage *response,
    void *userdata)
{
    AppStage_TestRumble *thisPtr = reinterpret_cast<AppStage_TestRumble *>(userdata);

    if (response->result_code == ClientPSMoveAPI::_clientPSMoveResultCode_ok)
    {
        thisPtr->m_menuState = AppStage_TestRumble::idle;
    }
    else
    {
        thisPtr->m_menuState = AppStage_TestRumble::failedStreamStart;
    }
}

void AppStage_TestRumble::request_exit_to_app_stage(const char *app_stage_name)
{
    ClientPSMoveAPI::eat_response(ClientPSMoveAPI::stop_controller_data_stream(m_controllerView));

    m_app->setAppStage(app_stage_name);
}

float AppStage_TestRumble::get_left_trigger() const
{
    float trigger = 0.f;

    switch (m_controllerView->GetControllerViewType())
    {
    case ClientControllerView::eControllerType::PSMove:
        {
			// Only one trigger
            trigger = m_controllerView->GetPSMoveView().GetTriggerValue();
        } break;
    case ClientControllerView::eControllerType::PSNavi:
        {
            trigger = 0.f;
        } break;
    case ClientControllerView::eControllerType::PSDualShock4:
        {
            trigger = m_controllerView->GetPSDualShock4View().GetLeftTriggerValue();
        } break;
    }

    return trigger;
}

float AppStage_TestRumble::get_right_trigger() const
{
    float trigger = 0.f;

    switch (m_controllerView->GetControllerViewType())
    {
    case ClientControllerView::eControllerType::PSMove:
        {
			// Only one trigger
            trigger = m_controllerView->GetPSMoveView().GetTriggerValue();
        } break;
    case ClientControllerView::eControllerType::PSNavi:
        {
            trigger = 0.f;
        } break;
    case ClientControllerView::eControllerType::PSDualShock4:
        {
            trigger = m_controllerView->GetPSDualShock4View().GetRightTriggerValue();
        } break;
    }

    return trigger;
}

float AppStage_TestRumble::get_big_rumble_amount() const
{
    float rumble = 0.f;

    switch (m_controllerView->GetControllerViewType())
    {
    case ClientControllerView::eControllerType::PSMove:
        {
			// Only one rumble
            rumble = m_controllerView->GetPSMoveView().GetRumble();
        } break;
    case ClientControllerView::eControllerType::PSNavi:
        {
            rumble = 0.f;
        } break;
    case ClientControllerView::eControllerType::PSDualShock4:
        {
            rumble = m_controllerView->GetPSDualShock4View().GetBigRumble();
        } break;
    }

    return rumble;
}

float AppStage_TestRumble::get_small_rumble_amount() const
{
    float rumble = 0.f;

    switch (m_controllerView->GetControllerViewType())
    {
    case ClientControllerView::eControllerType::PSMove:
        {
			// Only one rumble
            rumble = m_controllerView->GetPSMoveView().GetRumble();
        } break;
    case ClientControllerView::eControllerType::PSNavi:
        {
            rumble = 0.f;
        } break;
    case ClientControllerView::eControllerType::PSDualShock4:
        {
            rumble = m_controllerView->GetPSDualShock4View().GetSmallRumble();
        } break;
    }

    return rumble;
}

void AppStage_TestRumble::set_rumble_amounts(float big_rumble, float small_rumble)
{
    switch (m_controllerView->GetControllerViewType())
    {
    case ClientControllerView::eControllerType::PSMove:
        {
            m_controllerView->GetPSMoveViewMutable().SetRumble(fmaxf(big_rumble, small_rumble));
            m_controllerView->GetPSMoveViewMutable().SetLEDOverride(static_cast<unsigned char>(big_rumble*255.f), 0, 0);
        } break;
    case ClientControllerView::eControllerType::PSNavi:
        {
            // No rumble
        } break;
    case ClientControllerView::eControllerType::PSDualShock4:
        {
			float red= fmaxf(big_rumble, small_rumble);

            m_controllerView->GetPSDualShock4ViewMutable().SetBigRumble(big_rumble);
			m_controllerView->GetPSDualShock4ViewMutable().SetSmallRumble(small_rumble);
            m_controllerView->GetPSDualShock4ViewMutable().SetLEDOverride(static_cast<unsigned char>(red*255.f), 0, 0);
        } break;
    }
}

//-- private methods -----
static void drawController(ClientControllerView *controllerView, const glm::mat4 &transform, const glm::vec3 &bulb_color)
{
    switch(controllerView->GetControllerViewType())
    {
    case ClientControllerView::PSMove:
        drawPSMoveModel(transform, bulb_color);
        break;
    case ClientControllerView::PSNavi:
        drawPSNaviModel(transform);
        break;
    case ClientControllerView::PSDualShock4:
        drawPSDualShock4Model(transform, bulb_color);
        break;
    }
}

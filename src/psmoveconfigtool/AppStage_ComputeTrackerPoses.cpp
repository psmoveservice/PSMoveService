//-- inludes -----
#include "AppStage_ComputeTrackerPoses.h"
#include "AppStage_MainMenu.h"
#include "AppStage_TrackerSettings.h"
#include "App.h"
#include "Camera.h"
#include "ClientHMDView.h"
#include "Logger.h"
#include "OpenVRContext.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include "SharedTrackerState.h"
#include "MathGLM.h"

#include "SDL_keycode.h"
#include "SDL_opengl.h"

#include <imgui.h>
#include <sstream>

//-- statics ----
const char *AppStage_ComputeTrackerPoses::APP_STAGE_NAME = "ComputeTrackerPoses";

//-- constants -----

//-- public methods -----
AppStage_ComputeTrackerPoses::AppStage_ComputeTrackerPoses(App *app)
    : AppStage(app)
    , m_menuState(AppStage_ComputeTrackerPoses::inactive)
    , m_hmdView(nullptr)
    , m_lastHmdSeqNum(-1)
{ }

void AppStage_ComputeTrackerPoses::enter()
{
    // Allocate a new HMD stream
    assert(m_hmdView == nullptr);

    if (m_app->getOpenVRContext()->getIsInitialized())
    {
        m_hmdView = m_app->getOpenVRContext()->allocateHmdView();

        if (m_hmdView != nullptr)
        {
            m_menuState = eMenuState::pendingHmdStartStreamRequest;
            m_lastHmdSeqNum = -1;
        }
        else
        {
            m_menuState = eMenuState::failedHmdStartStreamRequest;
        }
    }
    else
    {
        m_menuState = eMenuState::idle;
        m_lastHmdSeqNum = -1;
    }
    
    m_app->setCameraType(_cameraFixed);

    m_lastHmdSeqNum = -1;
}

void AppStage_ComputeTrackerPoses::exit()
{
    if (m_hmdView != nullptr)
    {
        m_app->getOpenVRContext()->freeHmdView(m_hmdView);
        m_hmdView = nullptr;
    }

    m_menuState = eMenuState::inactive;
}

void AppStage_ComputeTrackerPoses::update()
{
    bool bControllerDataUpdatedThisFrame = false;

    if (m_hmdView != nullptr && m_hmdView->getSequenceNum() != m_lastHmdSeqNum)
    {
        m_lastHmdSeqNum = m_hmdView->getSequenceNum();
        bControllerDataUpdatedThisFrame = true;

        if (m_menuState == eMenuState::pendingHmdStartStreamRequest)
        {
            m_menuState = eMenuState::idle;
        }
    }
}

void AppStage_ComputeTrackerPoses::render()
{
    if (m_menuState == eMenuState::idle)
    {
        if (m_hmdView != nullptr)
        {
            PSMovePose pose = m_hmdView->getHmdPose();
            glm::quat orientation(pose.Orientation.w, pose.Orientation.x, pose.Orientation.y, pose.Orientation.z);
            glm::vec3 position(pose.Position.x, pose.Position.y, pose.Position.z);

            glm::mat4 rot = glm::mat4_cast(orientation);
            glm::mat4 trans = glm::translate(glm::mat4(1.0f), position);
            glm::mat4 transform = trans * rot;

            drawDK2Model(transform);
            drawTransformedAxes(transform, 10.f);
        }
    }
}

void AppStage_ComputeTrackerPoses::renderUI()
{
    const float k_panel_width = 300.f;
    const char *k_window_title = "Compute Tracker Poses";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case eMenuState::idle:
    {
        ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 50));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        if (ImGui::Button("Return to Tracker Settings"))
        {
            request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
        }

        ImGui::End();
    } break;

    case eMenuState::pendingHmdStartStreamRequest:
    {
        ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 50));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Pending HMD stream start...");

        if (ImGui::Button("Return to Tracker Settings"))
        {
            request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
        }

        ImGui::End();
    } break;

    case eMenuState::failedHmdStartStreamRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Failed to start HMD stream!");

        if (ImGui::Button("Ok"))
        {
            m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
        }

        if (ImGui::Button("Return to Main Menu"))
        {
            m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
        }

        ImGui::End();
    } break;

    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ComputeTrackerPoses::request_exit_to_app_stage(const char *app_stage_name)
{
    if (m_hmdView != nullptr)
    {
        m_app->getOpenVRContext()->freeHmdView(m_hmdView);
        m_hmdView = nullptr;
    }

    m_app->setAppStage(app_stage_name);
}
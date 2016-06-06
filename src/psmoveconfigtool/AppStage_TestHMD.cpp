//-- inludes -----
#include "AppStage_TestHMD.h"
#include "AppStage_HMDSettings.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "ClientHMDView.h"
#include "GeometryUtility.h"
#include "Logger.h"
#include "OpenVRContext.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include "MathGLM.h"

#include "SDL_keycode.h"
#include "SDL_opengl.h"

#include <imgui.h>
#include <sstream>

//-- statics ----
const char *AppStage_TestHMD::APP_STAGE_NAME = "TestHMD";

//-- constants -----

//-- public methods -----
AppStage_TestHMD::AppStage_TestHMD(App *app)
    : AppStage(app)
    , m_menuState(AppStage_TestHMD::inactive)
    , m_hmdView(nullptr)
    , m_lastHmdSeqNum(-1)
{ }

void AppStage_TestHMD::enter()
{
    const AppStage_HMDSettings *hmdSettings = m_app->getAppStage<AppStage_HMDSettings>();
    const OpenVRHmdInfo *hmdInfo = hmdSettings->getSelectedHmdInfo();
    assert(hmdInfo->DeviceIndex != -1);
    
    // Allocate a new HMD stream
    assert(m_hmdView == nullptr);
    m_hmdView = m_app->getOpenVRContext()->allocateHmdView();
    if (m_hmdView != nullptr)
    {
        m_menuState = eHmdMenuState::pendingHmdStartStreamRequest;
        m_lastHmdSeqNum = -1;
    }
    else
    {
        m_menuState = eHmdMenuState::failedHmdStartStreamRequest;
    }

    m_app->setCameraType(_cameraOrbit);

    m_lastHmdSeqNum = -1;
}

void AppStage_TestHMD::exit()
{
    if (m_hmdView != nullptr)
    {
        m_app->getOpenVRContext()->freeHmdView(m_hmdView);
        m_hmdView = nullptr;
    }

    m_menuState = eHmdMenuState::inactive;
}

void AppStage_TestHMD::update()
{
    bool bControllerDataUpdatedThisFrame = false;

     if (m_hmdView != nullptr && m_hmdView->getHMDSequenceNum() != m_lastHmdSeqNum)
     {
         m_lastHmdSeqNum = m_hmdView->getHMDSequenceNum();
         bControllerDataUpdatedThisFrame = true;

         if (m_menuState == eHmdMenuState::pendingHmdStartStreamRequest)
         {
             m_menuState = eHmdMenuState::idle;
         }
     }
}

void AppStage_TestHMD::render()
{
     if (m_menuState == eHmdMenuState::idle)
     {
        // Render the HMD
        {
            PSMovePose pose = m_hmdView->getDisplayHmdPose();
            glm::quat orientation(pose.Orientation.w, pose.Orientation.x, pose.Orientation.y, pose.Orientation.z);
            glm::vec3 position(pose.Position.x, pose.Position.y, pose.Position.z);
            glm::mat4 transform = glm_mat4_from_pose(orientation, position);

            drawDK2Model(transform);
            drawTransformedAxes(transform, 10.f);
        }

        // Render the HMD tracking volume
        {
            PSMoveVolume volume;

            if (m_app->getOpenVRContext()->getHMDTrackingVolume(volume))
            {
                drawTransformedVolume(glm::mat4(1.f), &volume, glm::vec3(0.f, 1.f, 1.f));
            }
        }

         //###HipsterSloth $TODO render tracking cameras
     }
}

void AppStage_TestHMD::renderUI()
{
    const float k_panel_width = 300.f;
    const char *k_window_title = "HMD Test";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case eHmdMenuState::idle:
    {
        ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 50));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        if (ImGui::Button("Return to HMD Settings"))
        {
            request_exit_to_app_stage(AppStage_HMDSettings::APP_STAGE_NAME);
        }

        ImGui::End();
    } break;

    case eHmdMenuState::pendingHmdStartStreamRequest:
    {
        ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 50));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Pending HMD stream start...");

        if (ImGui::Button("Return to HMD Settings"))
        {
            request_exit_to_app_stage(AppStage_HMDSettings::APP_STAGE_NAME);
        }

        ImGui::End();
    } break;

    case eHmdMenuState::failedHmdStartStreamRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Failed to start HMD stream!");

        if (ImGui::Button("Ok"))
        {
            m_app->setAppStage(AppStage_HMDSettings::APP_STAGE_NAME);
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

void AppStage_TestHMD::request_exit_to_app_stage(const char *app_stage_name)
{
    if (m_hmdView != nullptr)
    {
        m_app->getOpenVRContext()->freeHmdView(m_hmdView);
        m_hmdView = nullptr;
    }

    m_app->setAppStage(app_stage_name);
}
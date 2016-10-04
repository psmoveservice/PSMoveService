//-- inludes -----
#include "AppStage_MainMenu.h"
#include "AppStage_TrackerSettings.h"
#include "AppStage_ControllerSettings.h"
#include "AppStage_ServiceSettings.h"
#include "AppStage_TestTracker.h"
#include "App.h"
#include "Camera.h"
#include "Renderer.h"
#include "UIConstants.h"

#include "SDL_keycode.h"

#include <imgui.h>

//-- statics ----
const char *AppStage_MainMenu::APP_STAGE_NAME= "MainMenu";

//-- public methods -----
AppStage_MainMenu::AppStage_MainMenu(App *app) 
    : AppStage(app)
    , m_menuState(AppStage_MainMenu::inactive)
{ }

bool AppStage_MainMenu::init(int argc, char** argv)
{
    // Always fallback to the main menu on disconnection
    m_app->registerEventFallbackAppStage<AppStage_MainMenu>(ClientPSMoveAPI::disconnectedFromService);

    return true;
}

void AppStage_MainMenu::enter()
{
    m_app->setCameraType(_cameraFixed);

    // Only set the menu state if it hasn't been set already
    if (m_menuState == AppStage_MainMenu::inactive)
    {
        if (ClientPSMoveAPI::has_started())
        {
            m_menuState= AppStage_MainMenu::connectedToService;
        }
        else
        {
            m_menuState= AppStage_MainMenu::pendingConnectToToService;
            m_app->reconnectToService();
        }
    }
}

void AppStage_MainMenu::exit()
{
    // Upon normal exit, set the state to inactive
    m_menuState= AppStage_MainMenu::inactive;
}

void AppStage_MainMenu::renderUI()
{
    switch(m_menuState)
    {
    case connectedToService:
        {
            ImGuiWindowFlags window_flags = 
                ImGuiWindowFlags_ShowBorders |
                ImGuiWindowFlags_NoResize | 
                ImGuiWindowFlags_NoMove |
                ImGuiWindowFlags_NoScrollbar |
                ImGuiWindowFlags_NoCollapse;
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin("Main Menu", nullptr, ImVec2(300, 400), k_background_alpha, window_flags);
      
            if (ImGui::Button("Controller Settings"))
            {
                m_app->setAppStage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }
    
            if (ImGui::Button("Tracker Settings"))
            {
                m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }
    
            if (ImGui::Button("Exit"))
            {
                m_app->requestShutdown();
            }
    
            ImGui::End();
        } break;
    case pendingConnectToToService:
        {
            ImGuiWindowFlags window_flags = 
                ImGuiWindowFlags_ShowBorders |
                ImGuiWindowFlags_NoResize | 
                ImGuiWindowFlags_NoMove |
                ImGuiWindowFlags_NoScrollbar |
                ImGuiWindowFlags_NoCollapse;
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin("Status", nullptr, ImVec2(300, 150), k_background_alpha, window_flags);
            ImGui::Text("Connecting to PSMoveService...");
            if (ImGui::Button("Exit"))
            {
                m_app->requestShutdown();
            }
            ImGui::End();
        } break;
    case failedConnectionToService:
        {
            ImGuiWindowFlags window_flags = 
                ImGuiWindowFlags_ShowBorders |
                ImGuiWindowFlags_NoResize | 
                ImGuiWindowFlags_NoMove |
                ImGuiWindowFlags_NoScrollbar |
                ImGuiWindowFlags_NoCollapse;
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin("Status", nullptr, ImVec2(300, 150), k_background_alpha, window_flags);
            ImGui::Text("Failed to connect to PSMoveService!");
            
            if (ImGui::Button("Retry"))
            {
                m_menuState= AppStage_MainMenu::pendingConnectToToService;
                m_app->reconnectToService();
            }

            if (ImGui::Button("Exit"))
            {
                m_app->requestShutdown();
            }

            ImGui::End();
        } break;
    case disconnectedFromService:
        {
            ImGuiWindowFlags window_flags = 
                ImGuiWindowFlags_ShowBorders |
                ImGuiWindowFlags_NoResize | 
                ImGuiWindowFlags_NoMove |
                ImGuiWindowFlags_NoScrollbar |
                ImGuiWindowFlags_NoCollapse;
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin("Status", nullptr, ImVec2(300, 150), k_background_alpha, window_flags);
            ImGui::Text("Disconnected from PSMoveService!");
            
            if (ImGui::Button("Retry"))
            {
                m_menuState= AppStage_MainMenu::pendingConnectToToService;
                m_app->reconnectToService();
            }

            if (ImGui::Button("Exit"))
            {
                m_app->requestShutdown();
            }

            ImGui::End();
        } break;
    default:
        assert(0 && "unreachable");
    }
}

bool AppStage_MainMenu::onClientAPIEvent(
    ClientPSMoveAPI::eEventType event_type,
    ClientPSMoveAPI::t_event_data_handle opaque_event_handle) 
{
    bool bHandeled= false;

    switch(event_type)
    {
    case ClientPSMoveAPI::connectedToService:
        {
            // Allow the user to access the menu now
            m_menuState= AppStage_MainMenu::connectedToService;
            bHandeled= true;
        } break;

    case ClientPSMoveAPI::failedToConnectToService:
        {
            // Tell the user that the connection attempt failed
            m_menuState= AppStage_MainMenu::failedConnectionToService;
            bHandeled= true;
        } break;

    case ClientPSMoveAPI::disconnectedFromService:
        {
            // Tell the user that we failed to connect
            m_menuState= AppStage_MainMenu::failedConnectionToService;

            // If we weren't running this stage, make sure we are now
            if (m_app->getCurrentAppStage() != this)
            {
                m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            bHandeled= true;
        } break;
    }

    return bHandeled; 
}
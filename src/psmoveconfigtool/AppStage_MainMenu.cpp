//-- inludes -----
#include "AppStage_MainMenu.h"
#include "AppStage_TrackerSettings.h"
#include "AppStage_ControllerSettings.h"
#include "AppStage_HMDSettings.h"
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
    m_app->registerEventFallbackAppStage<AppStage_MainMenu>(PSMEventMessage::PSMEvent_disconnectedFromService);

    return true;
}

void AppStage_MainMenu::enter()
{
    m_app->setCameraType(_cameraFixed);

    // Only set the menu state if it hasn't been set already
    if (m_menuState == AppStage_MainMenu::inactive)
    {
        if (PSM_GetIsInitialized())
        {
            m_menuState= AppStage_MainMenu::connectedToService;
        }
        else
        {
            m_menuState= AppStage_MainMenu::startConnectionToService;
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
    
            if (ImGui::Button("HMD Settings"))
            {
                m_app->setAppStage(AppStage_HMDSettings::APP_STAGE_NAME);
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
	case startConnectionToService:
    case failedConnectionToService:
	case disconnectedFromService:
        {
            ImGuiWindowFlags window_flags = 
                ImGuiWindowFlags_ShowBorders |
                ImGuiWindowFlags_NoResize | 
                ImGuiWindowFlags_NoMove |
                ImGuiWindowFlags_NoScrollbar |
                ImGuiWindowFlags_NoCollapse;
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin("Connect", nullptr, ImVec2(300, 150), k_background_alpha, window_flags);

			if (m_menuState == failedConnectionToService)
			{
	            ImGui::Text("Failed to connect to PSMoveService!");
			}
			else if (m_menuState == disconnectedFromService)
			{
				ImGui::Text("Disconnected from PSMoveService!");
			}
            
			ImGui::PushItemWidth(125.f);
			if (ImGui::InputText("Server Address", m_app->m_serverAddress, sizeof(m_app->m_serverAddress), ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank))
			{
				m_app->m_bIsServerLocal= (strncmp(m_app->m_serverAddress, PSMOVESERVICE_DEFAULT_ADDRESS, sizeof(m_app->m_serverAddress)) == 0);
			}

			ImGui::InputText("Server Port", m_app->m_serverPort, sizeof(m_app->m_serverPort), ImGuiInputTextFlags_CharsDecimal);
			ImGui::PopItemWidth();

            if (ImGui::Button("Connect"))
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
    PSMEventMessage::eEventType event, 
    PSMEventDataHandle opaque_event_handle)
{
    bool bHandeled= false;

    switch(event)
    {
    case PSMEventMessage::PSMEvent_connectedToService:
        {
            // Allow the user to access the menu now
            m_menuState= AppStage_MainMenu::connectedToService;
            bHandeled= true;
        } break;

    case PSMEventMessage::PSMEvent_failedToConnectToService:
        {
            // Tell the user that the connection attempt failed
            m_menuState= AppStage_MainMenu::failedConnectionToService;
            bHandeled= true;
        } break;

    case PSMEventMessage::PSMEvent_disconnectedFromService:
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
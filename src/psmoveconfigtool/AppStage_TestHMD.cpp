//-- inludes -----
#include "AppStage_TestHMD.h"
#include "AppStage_HMDSettings.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "Logger.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "PSMoveClient_CAPI.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include "SharedTrackerState.h"
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
    , m_pendingAppStage(nullptr)
    , m_hmdView(nullptr)
    , m_isHmdStreamActive(false)
    , m_lastHmdSeqNum(-1)
{ }

void AppStage_TestHMD::enter()
{
    const AppStage_HMDSettings *hmdSettings = m_app->getAppStage<AppStage_HMDSettings>();
    const AppStage_HMDSettings::HMDInfo *hmdInfo = hmdSettings->getSelectedHmdInfo();
    assert(hmdInfo->HmdID != -1);
    
    assert(m_hmdView == nullptr);
	PSM_AllocateHmdListener(hmdInfo->HmdID);
	m_hmdView = PSM_GetHmd(hmdInfo->HmdID);

    m_app->setCameraType(_cameraFixed);

    m_menuState = eHmdMenuState::pendingHmdStartStreamRequest;
    assert(!m_isHmdStreamActive);
    m_lastHmdSeqNum = -1;

	PSMRequestID requestId;
	PSM_StartHmdDataStreamAsync(
		m_hmdView->HmdID, 
		PSMStreamFlags_includeRawSensorData, 
		&requestId);
	PSM_RegisterCallback(requestId, &AppStage_TestHMD::handle_hmd_start_stream_response, this);
}

void AppStage_TestHMD::exit()
{
    assert(m_hmdView != nullptr);	 
    PSM_FreeHmdListener(m_hmdView->HmdID);
    m_hmdView = nullptr;

    m_menuState = eHmdMenuState::inactive;
}

void AppStage_TestHMD::update()
{
    bool bControllerDataUpdatedThisFrame = false;

    if (m_isHmdStreamActive && m_hmdView->OutputSequenceNum != m_lastHmdSeqNum)
    {
        m_lastHmdSeqNum = m_hmdView->OutputSequenceNum;
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
		PSMPosef pose;
		if (PSM_GetHmdPose(m_hmdView->HmdID, &pose) == PSMResult_Success)
		{
			glm::quat orientation(pose.Orientation.w, pose.Orientation.x, pose.Orientation.y, pose.Orientation.z);
			glm::vec3 position(pose.Position.x, pose.Position.y, pose.Position.z);

			glm::mat4 rot = glm::mat4_cast(orientation);
			glm::mat4 trans = glm::translate(glm::mat4(1.0f), position);
			glm::mat4 transform = trans * rot;

			drawMorpheusModel(transform);
			drawTransformedAxes(transform, 10.f);
		}
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
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 50));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Waiting for HMD stream to start...");

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

    case eHmdMenuState::pendingHmdStopStreamRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 50));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Waiting for HMD stream to stop...");

        ImGui::End();
    } break;

    case eHmdMenuState::failedHmdStopStreamRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Failed to stop HMD stream!");

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

void AppStage_TestHMD::handle_hmd_start_stream_response(
	const PSMResponseMessage *response,
	void *userdata)
{
    AppStage_TestHMD *thisPtr = static_cast<AppStage_TestHMD *>(userdata);

    switch (response->result_code)
    {
    case PSMResult_Success:
        {
            thisPtr->m_isHmdStreamActive = true;
            thisPtr->m_lastHmdSeqNum = -1;
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
	case PSMResult_Timeout:
        {
            thisPtr->m_menuState = AppStage_TestHMD::failedHmdStartStreamRequest;
        } break;
    }
}

void AppStage_TestHMD::request_exit_to_app_stage(const char *app_stage_name)
{
    if (m_pendingAppStage == nullptr)
    {
        if (m_isHmdStreamActive)
        {
            m_pendingAppStage = app_stage_name;

			PSMRequestID requestId;
			PSM_StopHmdDataStreamAsync(m_hmdView->HmdID, &requestId);
			PSM_RegisterCallback(requestId, &AppStage_TestHMD::handle_hmd_stop_stream_response, this);
        }
        else
        {
            m_app->setAppStage(app_stage_name);
        }
    }
}

void AppStage_TestHMD::handle_hmd_stop_stream_response(
	const PSMResponseMessage *response,
	void *userdata)
{
    AppStage_TestHMD *thisPtr = static_cast<AppStage_TestHMD *>(userdata);

    if (response->result_code != PSMResult_Success)
    {
        Log_ERROR("AppStage_TestHMD", "Failed to release HMD on server!");
    }

    thisPtr->m_isHmdStreamActive = false;    
    thisPtr->m_app->setAppStage(thisPtr->m_pendingAppStage);
    thisPtr->m_pendingAppStage = nullptr;
}
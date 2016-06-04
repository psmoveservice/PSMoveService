//-- inludes -----
#include "AppStage_HMDSettings.h"
#include "AppStage_MainMenu.h"
#include "AppStage_TestHMD.h"
#include "App.h"
#include "Camera.h"
#include "ClientHMDView.h"
#include "OpenVRContext.h"
#include "Renderer.h"
#include "UIConstants.h"

#include "SDL_keycode.h"

#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>

//-- statics ----
const char *AppStage_HMDSettings::APP_STAGE_NAME= "HMDSettings";

//-- constants -----
const int k_max_hmd_list_list = 4;

//-- public methods -----
AppStage_HMDSettings::AppStage_HMDSettings(App *app) 
    : AppStage(app)
    , m_menuState(AppStage_HMDSettings::inactive)
    , m_hmdInfos(nullptr)
    , m_hmdListCount(0)
    , m_selectedHmdIndex(-1)
{ 
    m_hmdInfos = new OpenVRHmdInfo[k_max_hmd_list_list];
}

AppStage_HMDSettings::~AppStage_HMDSettings()
{
    delete[] m_hmdInfos;
}

const OpenVRHmdInfo *AppStage_HMDSettings::getSelectedHmdInfo() const
{
    return
        (m_selectedHmdIndex != -1)
        ? &m_hmdInfos[m_selectedHmdIndex]
        : nullptr;
}

void AppStage_HMDSettings::enter()
{
    m_app->setCameraType(_cameraFixed);
    m_selectedHmdIndex = -1;

    request_hmd_list();
}

void AppStage_HMDSettings::exit()
{
    m_menuState = AppStage_HMDSettings::inactive;
}

void AppStage_HMDSettings::update()
{
}
    
void AppStage_HMDSettings::render()
{
    switch (m_menuState)
    {
    case eHmdMenuState::idle:
    {
        if (m_selectedHmdIndex >= 0)
        {
//            const OpenVRHmdInfo &hmdInfo = m_hmdInfos[m_selectedHmdIndex];

            //###HipsterSloth $TODO Render the model retrieved from OpenVR
            glm::mat4 scale3 = glm::scale(glm::mat4(1.f), glm::vec3(3.f, 3.f, 3.f));
            drawDK2Model(scale3);
        }
    } break;

    case eHmdMenuState::failedHmdListRequest:
        {
        } break;

    default:
        assert(0 && "unreachable");
    }
}

void AppStage_HMDSettings::renderUI()
{
    const char *k_window_title = "Tracker Settings";
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
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(300, 400));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        if (m_hmdListCount > 0)
        {
            const OpenVRHmdInfo &hmdInfo = m_hmdInfos[m_selectedHmdIndex];

            ImGui::Text("HMD: %d", m_selectedHmdIndex);
            ImGui::Text("  OpenVR DeviceIndex: %d", hmdInfo.DeviceIndex);
            ImGui::Text("  Manufacturer: %s", hmdInfo.ManufacturerName.c_str());
            ImGui::Text("  Tracking System Name: %s", hmdInfo.TrackingSystemName.c_str());
            ImGui::Text("  Model Number: %s", hmdInfo.ModelNumber.c_str());
            ImGui::Text("  Serial Number: %s", hmdInfo.SerialNumber.c_str());
            ImGui::Text("  Tracking Firmware Version: %s", hmdInfo.TrackingFirmwareVersion.c_str());
            ImGui::Text("  Hardware Revision: %s", hmdInfo.HardwareRevision.c_str());
            ImGui::Text("  EDID Vendor ID: 0x%x", hmdInfo.EdidVendorID);
            ImGui::Text("  EDID Product ID: 0x%x", hmdInfo.EdidProductID);

            if (m_selectedHmdIndex > 0)
            {
                if (ImGui::Button("Previous HMD"))
                {
                    --m_selectedHmdIndex;
                }
            }

            if (m_selectedHmdIndex + 1 < m_hmdListCount)
            {
                if (ImGui::Button("Next HMD"))
                {
                    ++m_selectedHmdIndex;
                }
            }

            if (ImGui::Button("Test HMD Tracking"))
            {
                m_app->setAppStage(AppStage_TestHMD::APP_STAGE_NAME);
            }
        }
        else
        {
            ImGui::Text("No HMDs");
        }

        if (ImGui::Button("Return to Main Menu"))
        {
            m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
        }

        ImGui::End();
    } break;
    case eHmdMenuState::failedHmdListRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(300, 150));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Failed to get HMD list!");

        if (ImGui::Button("Retry"))
        {
            request_hmd_list();
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

void AppStage_HMDSettings::request_hmd_list()
{
    if (m_app->getOpenVRContext()->getIsInitialized())
    {
        m_hmdListCount = m_app->getOpenVRContext()->getHmdList(m_hmdInfos, k_max_hmd_list_list);
        m_selectedHmdIndex = (m_hmdListCount > 0) ? 0 : -1;
        m_menuState = AppStage_HMDSettings::idle;
    }
    else
    {
        m_hmdListCount = 0;
        m_selectedHmdIndex = -1;
        m_menuState = AppStage_HMDSettings::failedHmdListRequest;
    }
}
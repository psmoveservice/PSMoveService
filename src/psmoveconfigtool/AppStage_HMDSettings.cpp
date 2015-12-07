//-- inludes -----
#include "AppStage_HMDSettings.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "Renderer.h"
#include "UIConstants.h"

#include "SDL_keycode.h"

#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>

//-- statics ----
const char *AppStage_HMDSettings::APP_STAGE_NAME= "HMDSettings";

//-- constants -----

//-- public methods -----
AppStage_HMDSettings::AppStage_HMDSettings(App *app) 
    : AppStage(app)
{ }

void AppStage_HMDSettings::enter()
{
    m_app->setCameraType(_cameraFixed);
}

void AppStage_HMDSettings::exit()
{
}

void AppStage_HMDSettings::update()
{
}
    
void AppStage_HMDSettings::render()
{
    drawDK2Model(glm::mat4(1.f));
}

void AppStage_HMDSettings::renderUI()
{
    ImGuiWindowFlags window_flags = 
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize | 
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;
    ImGui::SetNextWindowPosCenter();
    ImGui::Begin("HMD Settings", nullptr, ImVec2(300, 150), k_background_alpha, window_flags);

    if (ImGui::Button("Return to Main Menu"))
    {
        m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
    }

    ImGui::End();
}
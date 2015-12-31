//-- inludes -----
#include "AppStage_CameraSettings.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "Renderer.h"
#include "UIConstants.h"

#include "SDL_keycode.h"

#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>

//-- statics ----
const char *AppStage_CameraSettings::APP_STAGE_NAME= "CameraSettings";

//-- constants -----

//-- public methods -----
AppStage_CameraSettings::AppStage_CameraSettings(App *app) 
    : AppStage(app)
{ }

void AppStage_CameraSettings::enter()
{
    m_app->setCameraType(_cameraFixed);
}

void AppStage_CameraSettings::exit()
{
}

void AppStage_CameraSettings::update()
{
}
    
void AppStage_CameraSettings::render()
{
    glm::mat4 scale3= glm::scale(glm::mat4(1.f), glm::vec3(3.f, 3.f, 3.f));
    drawPS3EyeModel(scale3);
}

void AppStage_CameraSettings::renderUI()
{
    ImGuiWindowFlags window_flags = 
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize | 
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;
    ImGui::SetNextWindowPosCenter();
    ImGui::Begin("Camera Settings", nullptr, ImVec2(300, 150), k_background_alpha, window_flags);

    if (ImGui::Button("Return to Main Menu"))
    {
        m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
    }

    ImGui::End();
}
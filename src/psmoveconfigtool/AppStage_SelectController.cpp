//-- inludes -----
#include "AppStage_SelectController.h"
#include "App.h"
#include "Camera.h"
#include "Renderer.h"

#include "SDL_keycode.h"
#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>

//-- statics ----
const char *AppStage_SelectController::APP_STAGE_NAME= "SelectController";

//-- constants -----
static const float k_bg_alpha = 0.65f;

//-- public methods -----
AppStage_SelectController::AppStage_SelectController(App *app) 
    : AppStage(app)
{ }

void AppStage_SelectController::enter()
{
    m_app->setCameraType(_cameraFixed);
}

void AppStage_SelectController::exit()
{
}

void AppStage_SelectController::update()
{
}
    
void AppStage_SelectController::render()
{
    glm::mat4 scale2RotateX90= 
        glm::rotate(
            glm::scale(glm::mat4(1.f), glm::vec3(2.f, 2.f, 2.f)), 
            90.f, glm::vec3(1.f, 0.f, 0.f));    
    drawPSMoveModel(scale2RotateX90, glm::vec3(1.f, 1.f, 0.f));
}

void AppStage_SelectController::renderUI()
{
    ImGuiWindowFlags window_flags = 
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize | 
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;
    ImGui::SetNextWindowPosCenter();
    ImGui::Begin("Status", nullptr, ImVec2(300, 150), k_bg_alpha, window_flags);
    ImGui::Text("Here is some status...");
    ImGui::End();
}

void AppStage_SelectController::onKeyDown(SDL_Keycode keyCode)
{
    if (keyCode == SDLK_SPACE)
    {
        //m_app->setAppStage(...);
    }
}
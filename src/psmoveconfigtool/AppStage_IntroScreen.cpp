//-- inludes -----
#include "AppStage_IntroScreen.h"
#include "App.h"
#include "Camera.h"
#include "Renderer.h"

#include "SDL_keycode.h"

#include <imgui.h>

//-- statics ----
const char *AppStage_IntroScreen::APP_STAGE_NAME= "IntroScreen";

//-- globals -----
bool g_show_test_window = true;
bool g_show_another_window = false;
ImVec4 g_clear_color = ImColor(114, 144, 154);

//-- public methods -----
AppStage_IntroScreen::AppStage_IntroScreen(App *app) 
    : AppStage(app)
{ }

void AppStage_IntroScreen::enter()
{
    m_app->getOrbitCamera()->setCameraOrbitLocation(-3.f, 0.f, 500.f); // yaw degrees, pitch degrees, radius cm
    m_app->setCameraType(_cameraOrbit);
}

void AppStage_IntroScreen::exit()
{
}

void AppStage_IntroScreen::update()
{
}
    
void AppStage_IntroScreen::render()
{
    drawTransformedAxes(glm::mat4(1.0f), 100.f);
}

void AppStage_IntroScreen::renderUI()
{
    static float f = 0.0f;
    ImGui::Text("Hello, world!");
    ImGui::SliderFloat("float", &f, 0.0f, 1.0f);
    ImGui::ColorEdit3("clear color", (float*)&g_clear_color);

    if (ImGui::Button("Test Window")) g_show_test_window ^= 1;
    if (ImGui::Button("Another Window")) g_show_another_window ^= 1;
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

    // 2. Show another simple window, this time using an explicit Begin/End pair
    if (g_show_another_window)
    {
        ImGui::SetNextWindowSize(ImVec2(200,100), ImGuiSetCond_FirstUseEver);
        ImGui::Begin("Another Window", &g_show_another_window);
        ImGui::Text("Hello");
        ImGui::End();
    }

    // 3. Show the ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
    if (g_show_test_window)
    {
        ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
        ImGui::ShowTestWindow(&g_show_test_window);
    }
}

void AppStage_IntroScreen::onKeyDown(SDL_Keycode keyCode)
{
    if (keyCode == SDLK_SPACE)
    {
        //m_app->setAppStage(...);
    }
}
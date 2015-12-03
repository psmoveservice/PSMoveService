//-- inludes -----
#include "AppStage_IntroScreen.h"
#include "App.h"
#include "Camera.h"
#include "Renderer.h"

#include "SDL_keycode.h"

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
}

void AppStage_IntroScreen::onKeyDown(SDL_Keycode keyCode)
{
    if (keyCode == SDLK_SPACE)
    {
        //m_app->setAppStage(...);
    }
}
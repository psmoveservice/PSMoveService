//-- includes -----
#include "App.h"
#include "AppStage_IntroScreen.h"
#include "Logger.h"

//-- public methods -----
App::App()
    : m_renderer()
    , m_assetManager()
    , m_cameraType(_cameraNone)
    , m_camera(NULL)
    , m_orbitCamera(&m_renderer)
    , m_fixedCamera(&m_renderer)
    , m_appStageType(_appStageIntroScreen)
    , m_appStage(nullptr)
    , m_appStage_IntroScreen(new AppStage_IntroScreen(this))
{
}

App::~App()
{
    delete m_appStage_IntroScreen;
}

int App::exec(int argc, char** argv)
{
    int result= 0;

    if (init(argc, argv))
    {
        SDL_Event e;

        while (true) 
        {
            if (SDL_PollEvent(&e)) 
            {
                if (e.type == SDL_QUIT || 
                    (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE)) 
                {
                    Log_INFO("App::exec", "QUIT message received");
                    break;
                }
                else 
                {
                    onSDLEvent(e);
                }
            }

            update();
            render();
        }
    }
    else
    {
        Log_ERROR("App::exec", "Failed to initialize application!");
        result= -1;
    }

    destroy();

    return result;
}

void App::setCameraType(eCameraType cameraType)
{
    switch (cameraType)
    {
    case _cameraNone:
        m_camera= NULL;
        break;
    case _cameraOrbit:
        m_camera= &m_orbitCamera;
        break;
    case _cameraFixed:
        m_camera= &m_fixedCamera;
        break;
    }

    m_cameraType= cameraType;

    if (m_camera != NULL)
    {
        m_camera->publishCameraViewMatrix();
    }
    else
    {
        m_renderer.setCameraViewMatrix(glm::mat4(1.f));
    }
}

void App::setAppStage(eAppStageType appStageType)
{
    if (m_appStage != NULL)
    {
        m_appStage->exit();
    }

    switch (appStageType)
    {
    case _appStageNone:
        m_appStage= NULL;
        break;
    case _appStageIntroScreen:
        m_appStage = m_appStage_IntroScreen;
        break;
    }

    m_appStageType= appStageType;

    if (m_appStage != NULL)
    {
        m_appStage->enter();
    }
}

//-- private methods -----
bool App::init(int argc, char** argv)
{
    bool success= true;

    if (success && !m_renderer.init())
    {
        Log_ERROR("App::init", "Failed to initialize renderer!");
        success= false;
    }

    if (success && !m_assetManager.init())
    {
        Log_ERROR("App::init", "Failed to initialize asset manager!");
        success= false;
    }

    if (success)
    {
        m_orbitCamera.setIsLocked(false);
        m_fixedCamera.setIsLocked(true);

        setAppStage(_appStageIntroScreen);
    }

    return success;
}

void App::destroy()
{
    setAppStage(_appStageNone);
    m_assetManager.destroy();
    m_renderer.destroy();
}
    
void App::onSDLEvent(const SDL_Event &e)
{
    if (m_appStage != NULL)
    {
        switch(e.type)
        {
        case SDL_KEYDOWN:
            m_appStage->onKeyDown(e.key.keysym.sym);
            break;
        }
    }

    if (m_camera != NULL)
    {
        switch(e.type)
        {
        case SDL_MOUSEMOTION:
            m_camera->onMouseMotion((int)e.motion.xrel, (int)e.motion.yrel);
            break;
        case SDL_MOUSEBUTTONDOWN:
            m_camera->onMouseButtonDown((int)e.button.button);
            break;
        case SDL_MOUSEBUTTONUP:
            m_camera->onMouseButtonUp((int)e.button.button);
            break;
        case SDL_MOUSEWHEEL:
            m_camera->onMouseWheel((int)e.wheel.y);
            break;
        }
    }
}

void App::update()
{
    if (m_appStage != NULL)
    {
        m_appStage->update();
    }
}

void App::render()
{
    m_renderer.renderBegin();

    m_renderer.renderStageBegin();
    if (m_appStage != NULL)
    {
        m_appStage->render();
    }
    m_renderer.renderStageEnd();

    m_renderer.renderUIBegin();
    if (m_appStage != NULL)
    {
        m_appStage->renderUI();
    }
    m_renderer.renderUIEnd();

    m_renderer.renderEnd();
}
//-- includes -----
#include "App.h"
#include "AppStage.h"
#include "AssetManager.h"
#include "Renderer.h"
#include "Logger.h"

//-- public methods -----
App::App()
    : m_renderer(new Renderer())
    , m_assetManager(new AssetManager())
    , m_cameraType(_cameraNone)
    , m_camera(NULL)
    , m_orbitCamera(m_renderer)
    , m_fixedCamera(m_renderer)
    , m_appStageName(nullptr)
    , m_appStage(nullptr)
{
}

App::~App()
{
    for (t_app_stage_map::const_iterator iter= m_nameToAppStageMap.begin(); iter != m_nameToAppStageMap.end(); ++iter)
    {
        delete iter->second;
    }

    delete m_renderer;
    delete m_assetManager;
}

int App::exec(int argc, char** argv, const char *initial_state_name)
{
    int result= 0;

    if (init(argc, argv))
    {
        SDL_Event e;

        setAppStage(initial_state_name);

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

bool App::reconnectToService()
{
    if (ClientPSMoveAPI::has_started())
    {
        ClientPSMoveAPI::shutdown();
    }

    bool success= 
        ClientPSMoveAPI::startup(
            "localhost", "9512", //###bwalker $TODO put in config 
            &App::onClientPSMoveEvent, this, 
            _log_severity_level_info); //###bwalker $TODO put in config 

    return success;
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

    if (m_camera != nullptr)
    {
        m_camera->publishCameraViewMatrix();
    }
    else
    {
        m_renderer->setCameraViewMatrix(glm::mat4(1.f));
    }
}

void App::setAppStage(const char *appStageName)
{
    if (m_appStage != nullptr)
    {
        m_appStage->exit();
    }    

    m_appStageName= appStageName;
    m_appStage = (appStageName != nullptr) ? m_nameToAppStageMap[appStageName] : nullptr;

    if (m_appStage != nullptr)
    {
        m_appStage->enter();
    }
}

//-- private methods -----
bool App::init(int argc, char** argv)
{
    bool success= true;

    if (success && !m_renderer->init())
    {
        Log_ERROR("App::init", "Failed to initialize renderer!");
        success= false;
    }

    if (success && !m_assetManager->init())
    {
        Log_ERROR("App::init", "Failed to initialize asset manager!");
        success= false;
    }

    if (success)
    {
        for (t_app_stage_map::const_iterator iter= m_nameToAppStageMap.begin(); iter != m_nameToAppStageMap.end(); ++iter)
        {
            if (!iter->second->init(argc, argv))
            {
                Log_ERROR("App::init", "Failed to initialize app stage %s!", iter->first);
                success= false;
                break;
            }
        }
    }

    if (success)
    {
        m_orbitCamera.setIsLocked(false);
        m_fixedCamera.setIsLocked(true);
    }

    return success;
}

void App::destroy()
{
    setAppStage(nullptr);

    for (t_app_stage_map::const_iterator iter= m_nameToAppStageMap.begin(); iter != m_nameToAppStageMap.end(); ++iter)
    {
        iter->second->destroy();
    }

    m_assetManager->destroy();
    m_renderer->destroy();
}
    
void App::onSDLEvent(const SDL_Event &e)
{
    m_renderer->onSDLEvent(&e);

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

void App::onClientPSMoveEvent(
    ClientPSMoveAPI::eClientPSMoveAPIEvent event_type,
    ClientPSMoveAPI::t_event_data_handle opaque_event_handle,
    void *userdata)
{
    App *thisPtr= reinterpret_cast<App *>(userdata);

    // Try giving the event to the current AppStage first
    if (!thisPtr->m_appStage->onClientAPIEvent(event_type, opaque_event_handle))
    {
        t_app_stage_event_map::iterator entry= thisPtr->m_eventToFallbackAppStageMap.find(event_type);

        if (entry != thisPtr->m_eventToFallbackAppStageMap.end() && 
            entry->second != thisPtr->m_appStage)
        {
            // If the current stage doesn't care about the event,
            // hand it off to another app stage registered to care about the event
            entry->second->onClientAPIEvent(event_type, opaque_event_handle);
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
    m_renderer->renderBegin();

    m_renderer->renderStageBegin();
    if (m_appStage != NULL)
    {
        m_appStage->render();
    }
    m_renderer->renderStageEnd();

    m_renderer->renderUIBegin();
    if (m_appStage != NULL)
    {
        m_appStage->renderUI();
    }
    m_renderer->renderUIEnd();

    m_renderer->renderEnd();
}
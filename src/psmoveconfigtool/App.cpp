//-- includes -----
#include "App.h"
#include "AppStage.h"
#include "AssetManager.h"
#include "Renderer.h"
#include "Logger.h"

#include "AppStage_MainMenu.h"

#include "PSMoveProtocol.pb.h"

class InputParser {
public:
	InputParser(int &argc, char **argv) {
		for (int i = 1; i < argc; ++i)
			this->tokens.push_back(std::string(argv[i]));
	}
	/// @author iain
	const std::string& getCmdOption(const std::string &option) const {
		std::vector<std::string>::const_iterator itr;
		itr = std::find(this->tokens.begin(), this->tokens.end(), option);
		if (itr != this->tokens.end() && ++itr != this->tokens.end()) {
			return *itr;
		}
		static const std::string empty_string("");
		return empty_string;
	}
	/// @author iain
	bool cmdOptionExists(const std::string &option) const {
		return std::find(this->tokens.begin(), this->tokens.end(), option)
			!= this->tokens.end();
	}
private:
	std::vector <std::string> tokens;
};


//-- public methods -----
App::App()
    : m_renderer(new Renderer())
    , m_assetManager(new AssetManager())
    , m_cameraType(_cameraNone)
    , m_camera(NULL)
    , m_orbitCamera(m_renderer)
    , m_fixedCamera(m_renderer)
    , m_appStage(nullptr)
    , m_bShutdownRequested(false)
{
	strncpy(m_serverAddress, PSMOVESERVICE_DEFAULT_ADDRESS, sizeof(m_serverAddress));
	strncpy(m_serverPort, PSMOVESERVICE_DEFAULT_PORT, sizeof(m_serverPort));
	m_bIsServerLocal= true;

	autoConnect = false;

	excludePositionSettings = false;
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

void App::initCommandLine(int argc, char** argv) {
	InputParser inputParser(argc, argv);

	if (inputParser.cmdOptionExists("\\autoConnect")) {
		autoConnect = true;
	}

	if (inputParser.cmdOptionExists("\\excludePositionSettings")) {
		excludePositionSettings = true;
	}

	initialStage = inputParser.getCmdOption("\\initialStage");
	
}


int App::exec(int argc, char** argv)
{
    int result= 0;

	initCommandLine(argc, argv);

    if (init(argc, argv))
    {
        SDL_Event e;

		if (autoConnect) {
			reconnectToServiceSync();
		}

		if (initialStage.length() > 0) {
			setAppStage(initialStage.c_str());
		}
		
		if(m_appStageName.length() == 0)
		{
			setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
		}


        while (!m_bShutdownRequested) 
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
    if (PSM_GetIsInitialized())
    {
		PSM_Shutdown();
    }

    bool success= 
		PSM_InitializeAsync(
            m_serverAddress,
            m_serverPort) == PSMResult_Success;

    return success;
}

bool App::reconnectToServiceSync()
{
	if (PSM_GetIsInitialized())
	{
		PSM_Shutdown();
	}

	bool success =
		PSM_Initialize(
			m_serverAddress,
			m_serverPort,2000) == PSMResult_Success;

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

    if (appStageName != nullptr)
    {
	    if (m_nameToAppStageMap.find(appStageName) != m_nameToAppStageMap.end()) {

		    m_appStageName = appStageName;
		    m_appStage = (appStageName != nullptr) ? m_nameToAppStageMap[appStageName] : nullptr;

		    if (m_appStage != nullptr)
		    {
			    m_appStage->enter();
		    }
	    }
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
                Log_ERROR("App::init", "Failed to initialize app stage %s!", iter->first.c_str());
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
    const PSMEventMessage *event)
{
    PSMEventMessageType event_type = event->event_type;
    PSMEventDataHandle opaque_event_handle = event->event_data_handle;

    // Try giving the event to the current AppStage first
    if (!m_appStage->onClientAPIEvent(event_type, opaque_event_handle))
    {
        t_app_stage_event_map::iterator entry= m_eventToFallbackAppStageMap.find(event_type);

        if (entry != m_eventToFallbackAppStageMap.end() && 
            entry->second != m_appStage)
        {
            // If the current stage doesn't care about the event,
            // hand it off to another app stage registered to care about the event
            entry->second->onClientAPIEvent(event_type, opaque_event_handle);
        }
    }
}

void App::onClientPSMoveResponse(
    const PSMResponseMessage *response)
{
    PSMRequestID request_id= response->request_id;
    const PSMoveProtocol::Response *protocol_response = GET_PSMOVEPROTOCOL_RESPONSE(response->opaque_response_handle);
    PSMoveProtocol::Response_ResponseType protocol_response_type= protocol_response->type();
    const std::string& protocol_response_type_name = PSMoveProtocol::Response_ResponseType_Name(protocol_response_type);

    // All responses should have been handled by a response handler
    Log_ERROR("App::onClientPSMoveResponse", "Unhandled response type:%s (request id: %d)!", 
        protocol_response_type_name.c_str(), request_id);
}

void App::update()
{
	if (PSM_GetIsInitialized())
	{
		// Poll any events from the service
		PSM_UpdateNoPollMessages();

		// Poll events queued up by the call to ClientPSMoveAPI::update()
		PSMMessage message;
		while (PSM_PollNextMessage(&message) == PSMResult_Success)
		{
			switch (message.payload_type)
			{
			case PSMMessageType::_messagePayloadType_Response:
				onClientPSMoveResponse(&message.response_data);
				break;
			case PSMMessageType::_messagePayloadType_Event:
				onClientPSMoveEvent(&message.event_data);
				break;
			}
		}
	}

    // Update the current app stage last
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
#ifndef APP_H
#define APP_H

//-- includes -----
#include "ClientPSMoveAPI.h"

#include "SDL_events.h"

#include "Camera.h"

#include <map>
#include <utility> // std::pair

//-- definitions -----
class App
{
public:
    App();
    virtual ~App();

    inline class Renderer *getRenderer()
    { return m_renderer; }

    inline class AssetManager *getAssetManager()
    { return m_assetManager; }

    inline Camera *getOrbitCamera()
    { return &m_orbitCamera; }
    inline Camera *getFixedCamera()
    { return &m_fixedCamera; }

    inline class AppStage *getCurrentAppStage()
    { return m_appStage; }
    inline const char *getCurrentAppStageName()
    { return m_appStageName; }
    inline class AppStage *getAppStageByName(const char *name)
    { return m_nameToAppStageMap[name]; }

    int exec(int argc, char** argv, const char *initial_state_name);

    bool reconnectToService();
    void setCameraType(eCameraType cameraType);
    void setAppStage(const char *appStageName);

    template <typename t_app_stage>
    inline void registerAppStage()
    {
        // The app stages contained in the map get cleaned up in the App destructor
        m_nameToAppStageMap.insert(t_app_stage_map_entry(t_app_stage::APP_STAGE_NAME, new t_app_stage(this)));
    }

    template <typename t_app_stage>
    inline void registerEventFallbackAppStage(ClientPSMoveAPI::eClientPSMoveAPIEvent event_type)
    {
        t_app_stage *app_stage= getAppStageByName(t_app_stage::APP_STAGE_NAME);
        m_eventToFallbackAppStageMap.insert(t_app_stage_event_map_entry(event_type, app_stage));
    }

protected:
    bool init(int argc, char** argv);
    void destroy();
   
    void onSDLEvent(const SDL_Event &e);
    static void onClientPSMoveEvent(
        ClientPSMoveAPI::eClientPSMoveAPIEvent event_type,
        ClientPSMoveAPI::t_event_data_handle opaque_event_handle,
        void *userdata);

    void update();
    void render();

private:
    // Contexts
    class Renderer *m_renderer;

    // Assets (textures, sounds)
    class AssetManager *m_assetManager;

    // Cameras
    eCameraType m_cameraType;
    Camera *m_camera;
    Camera m_orbitCamera;
    Camera m_fixedCamera;

    // App Stages
    const char *m_appStageName;
    class AppStage *m_appStage;

    typedef std::map<const char *, class AppStage *> t_app_stage_map;
    typedef std::pair<const char *, class AppStage *> t_app_stage_map_entry;

    typedef std::map<ClientPSMoveAPI::eClientPSMoveAPIEvent, class AppStage *> t_app_stage_event_map;
    typedef std::pair<ClientPSMoveAPI::eClientPSMoveAPIEvent, class AppStage *> t_app_stage_event_map_entry;

    t_app_stage_map m_nameToAppStageMap;

    // If the current app stage doesn't handle the event,
    // we look here to find an app state that could handle it
    t_app_stage_event_map m_eventToFallbackAppStageMap;
};

#endif // APP_H
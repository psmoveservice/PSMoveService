#ifndef APP_H
#define APP_H

//-- includes -----
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

    int exec(int argc, char** argv, const char *initial_state_name);

    void setCameraType(eCameraType cameraType);
    void setAppStage(const char *appStageName);

    template <typename t_app_stage>
    inline void registerAppStage()
    {
        // This mapping gets cleaned up in the destructor
        m_nameToAppStageMap.insert(t_app_stage_map_entry(t_app_stage::APP_STAGE_NAME, new t_app_stage(this)));
    }

protected:
    bool init(int argc, char** argv);
    void destroy();
   
    void onSDLEvent(const SDL_Event &e);

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

    t_app_stage_map m_nameToAppStageMap;
};

#endif // APP_H
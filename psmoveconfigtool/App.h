#ifndef APP_H
#define APP_H

//-- includes -----
#include "Camera.h"
#include "Renderer.h"
#include "AssetManager.h"

#include "SDL_events.h"

//-- constants -----
enum eAppStageType
{
    _appStageNone,
    _appStageIntroScreen,
};

//-- definitions -----
class App
{
public:
    App();
    virtual ~App();

    inline Renderer *getRenderer()
    { return &m_renderer; }

    inline AssetManager *getAssetManager()
    { return &m_assetManager; }

    inline Camera *getOrbitCamera()
    { return &m_orbitCamera; }
    inline Camera *getFixedCamera()
    { return &m_fixedCamera; }

    int exec(int argc, char** argv);

    void setCameraType(eCameraType cameraType);
    void setAppStage(eAppStageType appStageType);

protected:
    bool init(int argc, char** argv);
    void destroy();
   
    void onSDLEvent(const SDL_Event &e);

    void update();
    void render();

private:
    // Contexts
    class Renderer m_renderer;

    // Assets (textures, sounds)
    class AssetManager m_assetManager;

    // Cameras
    eCameraType m_cameraType;
    Camera *m_camera;
    Camera m_orbitCamera;
    Camera m_fixedCamera;

    // App Stages
    eAppStageType m_appStageType;
    class AppStage *m_appStage;
    class AppStage_IntroScreen *m_appStage_IntroScreen;
};

#endif // APP_H
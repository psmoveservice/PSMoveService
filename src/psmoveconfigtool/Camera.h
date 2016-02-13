#ifndef CAMERA_H
#define CAMERA_H

//-- includes -----
#include <glm/glm.hpp>

//-- constants -----
enum eCameraType
{
    _cameraNone,
    _cameraOrbit,
    _cameraFixed
};

//-- definitions -----
class Camera
{
public:
    Camera(class Renderer *renderer) 
        : m_renderer(renderer)
        , m_cameraOrbitYawDegrees(0.f)
        , m_cameraOrbitPitchDegrees(0.f)
        , m_cameraOrbitRadius(100.f)
        , m_cameraPosition(0.f, 0.f, 100.f)
        , m_isPanningOrbitCamera(false)
        , m_isLocked(false)
    { }

    void onMouseMotion(int deltaX, int deltaY);
    void onMouseButtonDown(int buttonIndex);
    void onMouseButtonUp(int buttonIndex);
    void onMouseWheel(int scrollAmount);

    void setIsLocked(bool locked);
    void setCameraOrbitLocation(float yawDegrees, float pitchDegrees, float radius);
    void setCameraOrbitRadius(float radius);
    void resetOrientation();
    void reset();
    void publishCameraViewMatrix();

private:
    class Renderer *m_renderer;
    float m_cameraOrbitYawDegrees;
    float m_cameraOrbitPitchDegrees;
    float m_cameraOrbitRadius;
    glm::vec3 m_cameraPosition;
    bool m_isPanningOrbitCamera;
    bool m_isLocked;
};

#endif //CAMERA_H
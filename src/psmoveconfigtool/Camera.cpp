//-- includes -----
#include "Camera.h"
#include "MathUtility.h"
#include "Renderer.h"

#include "SDL_mouse.h"

#include <glm/gtc/matrix_transform.hpp>

//-- constants -----
static const float k_camera_mouse_zoom_scalar= 50.f;
static const float k_camera_mouse_pan_scalar= 0.5f;
static const float k_camera_min_zoom= 100.f;
static const float k_camera_y_offset = 1.f;

//-- public methods -----
void Camera::onMouseMotion(int deltaX, int deltaY)
{
    if (!m_isLocked && m_isPanningOrbitCamera)
    {
        const float deltaYaw= -(float)deltaX * k_camera_mouse_pan_scalar;
        const float deltaPitch= (float)deltaY * k_camera_mouse_pan_scalar;

        setCameraOrbitLocation(
            m_cameraOrbitYawDegrees+deltaYaw, 
            m_cameraOrbitPitchDegrees+deltaPitch, 
            m_cameraOrbitRadius);
    }
}

void Camera::onMouseButtonDown(int buttonIndex)
{
    if (!m_isLocked && buttonIndex == SDL_BUTTON_LEFT)
    {
        m_isPanningOrbitCamera= true;
    }
}

void Camera::onMouseButtonUp(int buttonIndex)
{
    if (!m_isLocked && buttonIndex == SDL_BUTTON_LEFT)
    {
        m_isPanningOrbitCamera= false;
    }
}

void Camera::onMouseWheel(int scrollAmount)
{
    if (!m_isLocked)
    {
        const float deltaRadius= (float)scrollAmount * k_camera_mouse_zoom_scalar;

        setCameraOrbitLocation(
            m_cameraOrbitYawDegrees, 
            m_cameraOrbitPitchDegrees, 
            m_cameraOrbitRadius+deltaRadius);
    }
}

void Camera::setIsLocked(bool locked)
{
    if (locked)
    {
        m_isLocked= true;
        m_isPanningOrbitCamera= false;
    }
    else
    {
        m_isLocked= false;
    }
}

void Camera::setCameraOrbitLocation(float yawDegrees, float pitchDegrees, float radius)
{
    m_cameraOrbitYawDegrees= wrap_degrees(yawDegrees);
    m_cameraOrbitPitchDegrees= clampf(pitchDegrees, 0.f, 60.f);
    m_cameraOrbitRadius= fmaxf(radius, k_camera_min_zoom);

    const float yawRadians= degrees_to_radians(m_cameraOrbitYawDegrees);
    const float pitchRadians= degrees_to_radians(m_cameraOrbitPitchDegrees);
    const float xzRadiusAtPitch= m_cameraOrbitRadius*cosf(pitchRadians);

    m_cameraPosition= 
        glm::vec3(
            xzRadiusAtPitch*sinf(yawRadians),
            m_cameraOrbitRadius*sinf(pitchRadians),
            xzRadiusAtPitch*cosf(yawRadians))
        + glm::vec3(0.f, k_camera_y_offset, 0.f);

    publishCameraViewMatrix();
}

void Camera::setCameraOrbitYaw(float yawDegrees)
{
	setCameraOrbitLocation(yawDegrees, m_cameraOrbitPitchDegrees, m_cameraOrbitRadius);
}

void Camera::setCameraOrbitPitch(float pitchDegrees)
{
	setCameraOrbitLocation(m_cameraOrbitYawDegrees, pitchDegrees, m_cameraOrbitRadius);
}

void Camera::setCameraOrbitRadius(float radius)
{
    setCameraOrbitLocation(m_cameraOrbitYawDegrees, m_cameraOrbitPitchDegrees, radius);
}

void Camera::resetOrientation()
{
    setCameraOrbitLocation(0.f, 0.f, m_cameraOrbitRadius);
}

void Camera::reset()
{
    setCameraOrbitLocation(0.f, 0.f, k_camera_min_zoom);
}

void Camera::publishCameraViewMatrix()
{
    m_renderer->setCameraViewMatrix(
        glm::lookAt(
            m_cameraPosition,
            glm::vec3(0, 0, 0), // Look at tracking origin
            glm::vec3(0, 1, 0)));    // Up is up.
}

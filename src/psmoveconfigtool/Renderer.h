#ifndef RENDERER_H
#define RENDERER_H

//-- includes -----
#include <glm/glm.hpp>

//-- typedefs -----
typedef union SDL_Event SDL_Event;

//-- definitions -----
class Renderer 
{
public:
    Renderer();
    ~Renderer();

    bool init();
    void destroy();

    bool onSDLEvent(const SDL_Event *event);

    void renderBegin();
    void renderStageBegin();
    void renderStageEnd();
    void renderUIBegin();
    void renderUIEnd();
    void renderEnd();

    static bool getIsRenderingStage() 
    { return m_instance != nullptr && m_instance->m_isRenderingStage; }
    static bool getIsRenderingUI()
    { return m_instance != nullptr && m_instance->m_isRenderingUI; }
    float getWindowAspectRatio() const
    { return static_cast<float>(m_windowWidth) / static_cast<float>(m_windowHeight); }

    void setProjectionMatrix(const glm::mat4 &matrix)
    { m_projectionMatrix= matrix; }
    void setCameraViewMatrix(const glm::mat4 &matrix)
    { m_cameraViewMatrix= matrix; }

private:
    bool m_sdlapi_initialized;
    
    struct SDL_Window *m_window;
    int m_windowWidth, m_windowHeight;

    void *m_glContext;

    glm::mat4 m_projectionMatrix;
    glm::mat4 m_cameraViewMatrix;

    bool m_isRenderingStage;
    bool m_isRenderingUI;

    // imgui state
    double m_Time;
    bool m_MousePressed[3];
    float m_MouseWheel;
    unsigned int m_FontTexture;

    static Renderer *m_instance;
};

//-- drawing methods -----
void drawTransformedAxes(const glm::mat4 &transform, float scale);
void drawTransformedBox(const glm::mat4 &transform, const glm::vec3 &half_extents, const glm::vec3 &color);
void drawTransformedTexturedCube(const glm::mat4 &transform, int textureId, float scale);
void drawDK2Model(const glm::mat4 &transform);
void drawPSMoveModel(const glm::mat4 &transform, const glm::vec3 &color);
void drawPSNaviModel(const glm::mat4 &transform);
void drawPS3EyeModel(const glm::mat4 &transform);

#endif // RENDERER_H

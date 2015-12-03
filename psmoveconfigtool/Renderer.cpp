//-- includes -----
#include "Renderer.h"
#include "Logger.h"
#include "SDL.h"
#include "SDL_opengl.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

//-- constants -----
static const int k_window_pixel_width= 800;
static const int k_window_pixel_height= 600;

static const float k_camera_vfov= 35.f;
static const float k_camera_z_near= 0.1f;
static const float k_camera_z_far= 5000.f;

//-- statics -----
Renderer *Renderer::m_instance= NULL;

//-- public methods -----
Renderer::Renderer()
    : m_sdlapi_initialized(false)
    , m_window(NULL)
    , m_windowWidth(0)
    , m_windowHeight(0)
    , m_glContext(NULL)
    , m_projectionMatrix()
    , m_cameraViewMatrix()
    , m_isRenderingStage(false)
    , m_isRenderingUI(false)
{
}

Renderer::~Renderer()
{
    assert(!m_sdlapi_initialized);
    assert(m_instance == NULL);
}

bool Renderer::init()
{
    bool success = true;

    Log_INFO("Renderer::init()", "Initializing Renderer Context");

    if (SDL_Init(SDL_INIT_VIDEO) == 0) 
    {
        m_sdlapi_initialized= true;
    }
    else
    {
        Log_ERROR("Renderer::init", "Unable to initialize SDL: %s", SDL_GetError());
        success= false;
    }

    if (success)
    {
        m_window = SDL_CreateWindow("PSMove Config Tool",
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            k_window_pixel_width, k_window_pixel_height,
            SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
        m_windowWidth= k_window_pixel_width;
        m_windowHeight= k_window_pixel_height;

        if (m_window == NULL) 
        {
            Log_ERROR("Renderer::init", "Unable to initialize window: %s", SDL_GetError());
            success= false;
        }
    }

    if (success)
    {
        m_glContext = SDL_GL_CreateContext(m_window);
        if (m_glContext == NULL) 
        {
            Log_ERROR("Renderer::init", "Unable to initialize window: %s", SDL_GetError());
            success= false;
        }
    }

    if (success)
    {
        glClearColor(7.f/255.f, 34.f/255.f, 66.f/255.f, 1.f);
        glViewport(0, 0, m_windowWidth, m_windowHeight);

        glEnable(GL_LIGHT0);
        glEnable(GL_TEXTURE_2D);
        //glShadeModel(GL_SMOOTH);
        //glClearDepth(1.0f);
        glEnable(GL_DEPTH_TEST);
        //glDepthFunc(GL_LEQUAL);
        //glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        setProjectionMatrix(
            glm::perspective(k_camera_vfov, getWindowAspectRatio(), k_camera_z_near, k_camera_z_far));

        m_instance= this;
    }

    return success;
}

void Renderer::destroy()
{
    if (m_glContext != NULL)
    {
        SDL_GL_DeleteContext(m_glContext);
        m_glContext= NULL;
    }

    if (m_window != NULL)
    {
        SDL_DestroyWindow(m_window);
        m_window= NULL;
    }

    if (m_sdlapi_initialized)
    {
        SDL_Quit();
        m_sdlapi_initialized= false;
    }

    m_instance= NULL;
}

void Renderer::renderBegin()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::renderStageBegin()
{
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(glm::value_ptr(m_projectionMatrix));

    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(glm::value_ptr(m_cameraViewMatrix));

    m_isRenderingStage= true;
}

void Renderer::renderStageEnd()
{
    m_isRenderingStage= false;
}

void Renderer::renderUIBegin()
{
    const glm::mat4 ortho_projection= glm::ortho(
        0.f, (float)m_windowWidth, // left, right
        (float)m_windowHeight, 0.f, // bottom, top
        -1.0f, 1.0f); // zNear, zFar

    glClear(GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(glm::value_ptr(ortho_projection));

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    m_isRenderingUI= true;
}

void Renderer::renderUIEnd()
{
    m_isRenderingUI= false;
}

void Renderer::renderEnd()
{
    SDL_GL_SwapWindow(m_window);
}

//-- Drawing Methods -----
void drawTransformedAxes(const glm::mat4 &transform, float scale)
{
    assert(Renderer::getIsRenderingStage());

    glm::vec3 origin(0.f, 0.f, 0.f);
    glm::vec3 xAxis(scale, 0.f, 0.f);
    glm::vec3 yAxis(0.f, scale, 0.f);
    glm::vec3 zAxis(0.f, 0.f, scale);
   
    glPushMatrix();
        glMultMatrixf(glm::value_ptr(transform));
        glBegin(GL_LINES);

        glColor3ub(255, 0, 0);
        glVertex3fv(glm::value_ptr(origin)); glVertex3fv(glm::value_ptr(xAxis));

        glColor3ub(0, 255, 0);
        glVertex3fv(glm::value_ptr(origin)); glVertex3fv(glm::value_ptr(yAxis));

        glColor3ub(0, 0, 255);
        glVertex3fv(glm::value_ptr(origin)); glVertex3fv(glm::value_ptr(zAxis));

        glEnd();
    glPopMatrix();
}

void drawTransformedBox(const glm::mat4 &transform, const glm::vec3 &half_extents, const glm::vec3 &color)
{
    assert(Renderer::getIsRenderingStage());

    glm::vec3 v0(half_extents.x, half_extents.y, half_extents.z);
    glm::vec3 v1(-half_extents.x, half_extents.y, half_extents.z);
    glm::vec3 v2(-half_extents.x, half_extents.y, -half_extents.z);
    glm::vec3 v3(half_extents.x, half_extents.y, -half_extents.z);
    glm::vec3 v4(half_extents.x, -half_extents.y, half_extents.z);
    glm::vec3 v5(-half_extents.x, -half_extents.y, half_extents.z);
    glm::vec3 v6(-half_extents.x, -half_extents.y, -half_extents.z);
    glm::vec3 v7(half_extents.x, -half_extents.y, -half_extents.z);

    glPushMatrix();
        glMultMatrixf(glm::value_ptr(transform));
        glColor3fv(glm::value_ptr(color));

        glBegin(GL_LINES);

        glVertex3fv(glm::value_ptr(v0)); glVertex3fv(glm::value_ptr(v1));
        glVertex3fv(glm::value_ptr(v1)); glVertex3fv(glm::value_ptr(v2));
        glVertex3fv(glm::value_ptr(v2)); glVertex3fv(glm::value_ptr(v3));
        glVertex3fv(glm::value_ptr(v3)); glVertex3fv(glm::value_ptr(v0));

        glVertex3fv(glm::value_ptr(v4)); glVertex3fv(glm::value_ptr(v5));
        glVertex3fv(glm::value_ptr(v5)); glVertex3fv(glm::value_ptr(v6));
        glVertex3fv(glm::value_ptr(v6)); glVertex3fv(glm::value_ptr(v7));
        glVertex3fv(glm::value_ptr(v7)); glVertex3fv(glm::value_ptr(v4));

        glVertex3fv(glm::value_ptr(v0)); glVertex3fv(glm::value_ptr(v4));
        glVertex3fv(glm::value_ptr(v1)); glVertex3fv(glm::value_ptr(v5));
        glVertex3fv(glm::value_ptr(v2)); glVertex3fv(glm::value_ptr(v6));
        glVertex3fv(glm::value_ptr(v3)); glVertex3fv(glm::value_ptr(v7));

        glEnd();
    glPopMatrix();
}

void drawTransformedTexturedCube(const glm::mat4 &transform, int textureId, float scale)
{
    assert(Renderer::getIsRenderingStage());

    glBindTexture(GL_TEXTURE_2D, textureId);
    glColor3f(1.f, 1.f, 1.f);

    glBegin(GL_QUADS);
        glMultMatrixf(glm::value_ptr(transform));
        // Front Face
        glTexCoord2f(0.0f, 0.0f); glVertex3f(-scale, -scale,  scale);
        glTexCoord2f(1.0f, 0.0f); glVertex3f( scale, -scale,  scale);
        glTexCoord2f(1.0f, 1.0f); glVertex3f( scale,  scale,  scale);
        glTexCoord2f(0.0f, 1.0f); glVertex3f(-scale,  scale,  scale);
        // Back Face
        glTexCoord2f(1.0f, 0.0f); glVertex3f(-scale, -scale, -scale);
        glTexCoord2f(1.0f, 1.0f); glVertex3f(-scale,  scale, -scale);
        glTexCoord2f(0.0f, 1.0f); glVertex3f( scale,  scale, -scale);
        glTexCoord2f(0.0f, 0.0f); glVertex3f( scale, -scale, -scale);
        // Top Face
        glTexCoord2f(0.0f, 1.0f); glVertex3f(-scale,  scale, -scale);
        glTexCoord2f(0.0f, 0.0f); glVertex3f(-scale,  scale,  scale);
        glTexCoord2f(1.0f, 0.0f); glVertex3f( scale,  scale,  scale);
        glTexCoord2f(1.0f, 1.0f); glVertex3f( scale,  scale, -scale);
        // Bottom Face
        glTexCoord2f(1.0f, 1.0f); glVertex3f(-scale, -scale, -scale);
        glTexCoord2f(0.0f, 1.0f); glVertex3f( scale, -scale, -scale);
        glTexCoord2f(0.0f, 0.0f); glVertex3f( scale, -scale,  scale);
        glTexCoord2f(1.0f, 0.0f); glVertex3f(-scale, -scale,  scale);
        // Right face
        glTexCoord2f(1.0f, 0.0f); glVertex3f( scale, -scale, -scale);
        glTexCoord2f(1.0f, 1.0f); glVertex3f( scale,  scale, -scale);
        glTexCoord2f(0.0f, 1.0f); glVertex3f( scale,  scale,  scale);
        glTexCoord2f(0.0f, 0.0f); glVertex3f( scale, -scale,  scale);
        // Left Face
        glTexCoord2f(0.0f, 0.0f); glVertex3f(-scale, -scale, -scale);
        glTexCoord2f(1.0f, 0.0f); glVertex3f(-scale, -scale,  scale);
        glTexCoord2f(1.0f, 1.0f); glVertex3f(-scale,  scale,  scale);
        glTexCoord2f(0.0f, 1.0f); glVertex3f(-scale,  scale, -scale);
    glEnd();

    // rebind the default texture
    glBindTexture(GL_TEXTURE_2D, 0); 
}
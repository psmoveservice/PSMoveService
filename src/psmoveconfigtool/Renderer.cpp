//-- includes -----
#include "Renderer.h"
#include "ClientGeometry_CAPI.h"
#include "AssetManager.h"
#include "Logger.h"
#include "ProtocolVersion.h"
#include "UIConstants.h"

#include "SDL.h"
#include "SDL_events.h"
#include "SDL_opengl.h"
#include "SDL_syswm.h"

#include "GeometryUtility.h"
#include "MathUtility.h"
#include "MathGLM.h"

#include <imgui.h>

#include "psmovebody_3dmodel.h"
#include "psmovebulb_3dmodel.h"
#include "psnavi_3dmodel.h"
#include "ps3eye_3dmodel.h"
#include "ds4body_3dmodel.h"
#include "ds4lightbar_3dmodel.h"
#include "morpheus_3dmodel.h"

#include <algorithm>

#ifdef _MSC_VER
#pragma warning (disable: 4505) // unreferenced local function has been removed (stb stuff)
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': strcpy, strdup, sprintf, vsnprintf, sscanf, fopen
#define snprintf _snprintf
#endif

//-- constants -----
static const int k_window_pixel_width= 800;
static const int k_window_pixel_height= 600;

static const float k_camera_vfov= 35.f;
static const float k_camera_z_near= 0.1f;
static const float k_camera_z_far= 5000.f;

static const ImVec4 k_clear_color = ImColor(114, 144, 154);

static const glm::vec3 k_psmove_frustum_color = glm::vec3(0.1f, 0.7f, 0.3f);

//-- statics -----
Renderer *Renderer::m_instance= NULL;

//-- prototypes -----
static const char* ImGui_ImplSdl_GetClipboardText();
static void ImGui_ImplSdl_SetClipboardText(const char* text);
static void ImGui_ImplSdl_RenderDrawLists(ImDrawData* draw_data);

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
    , m_Time(0.0f)
    , m_MouseWheel(0.0f)
    , m_FontTexture(0)
{
    memset(&m_MousePressed, 0, sizeof(m_MousePressed));
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
		char szWindowTitle[128];

        SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
        SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
        SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);

		snprintf(szWindowTitle, sizeof(szWindowTitle), "PSMove Config Tool v%s", PSM_DETAILED_VERSION_STRING);
        m_window = SDL_CreateWindow(szWindowTitle,
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

    // Setup ImGui key-bindings and callback functions
    if (success)
    {
        ImGuiIO& io = ImGui::GetIO();
        io.KeyMap[ImGuiKey_Tab] = SDLK_TAB;                     // Keyboard mapping. ImGui will use those indices to peek into the io.KeyDown[] array.
        io.KeyMap[ImGuiKey_LeftArrow] = SDL_SCANCODE_LEFT;
        io.KeyMap[ImGuiKey_RightArrow] = SDL_SCANCODE_RIGHT;
        io.KeyMap[ImGuiKey_UpArrow] = SDL_SCANCODE_UP;
        io.KeyMap[ImGuiKey_DownArrow] = SDL_SCANCODE_DOWN;
        io.KeyMap[ImGuiKey_PageUp] = SDL_SCANCODE_PAGEUP;
        io.KeyMap[ImGuiKey_PageDown] = SDL_SCANCODE_PAGEDOWN;
        io.KeyMap[ImGuiKey_Home] = SDL_SCANCODE_HOME;
        io.KeyMap[ImGuiKey_End] = SDL_SCANCODE_END;
        io.KeyMap[ImGuiKey_Delete] = SDLK_DELETE;
        io.KeyMap[ImGuiKey_Backspace] = SDLK_BACKSPACE;
        io.KeyMap[ImGuiKey_Enter] = SDLK_RETURN;
        io.KeyMap[ImGuiKey_Escape] = SDLK_ESCAPE;
        io.KeyMap[ImGuiKey_A] = SDLK_a;
        io.KeyMap[ImGuiKey_C] = SDLK_c;
        io.KeyMap[ImGuiKey_V] = SDLK_v;
        io.KeyMap[ImGuiKey_X] = SDLK_x;
        io.KeyMap[ImGuiKey_Y] = SDLK_y;
        io.KeyMap[ImGuiKey_Z] = SDLK_z;

        io.RenderDrawListsFn = ImGui_ImplSdl_RenderDrawLists;   // Alternatively you can set this to NULL and call ImGui::GetDrawData() after ImGui::Render() to get the same ImDrawData pointer.
        io.SetClipboardTextFn = ImGui_ImplSdl_SetClipboardText;
        io.GetClipboardTextFn = ImGui_ImplSdl_GetClipboardText;

    #ifdef _WIN32
        SDL_SysWMinfo wmInfo;
        SDL_VERSION(&wmInfo.version);
        SDL_GetWindowWMInfo(m_window, &wmInfo);
        io.ImeWindowHandle = wmInfo.info.win.window;
    #endif

        m_Time= 0.0f;
        m_MouseWheel= 0.0f;
        m_FontTexture= 0;
        memset(&m_MousePressed, 0, sizeof(m_MousePressed));
    }

    if (success)
    {
        glClearColor(k_clear_color.x, k_clear_color.y, k_clear_color.z, k_clear_color.w);
        glViewport(0, 0, m_windowWidth, m_windowHeight);

        glEnable(GL_LIGHT0);
        glEnable(GL_TEXTURE_2D);        
        //glClearDepth(1.0f);
        glEnable(GL_DEPTH_TEST);
        //glDepthFunc(GL_LEQUAL);
        //glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        m_instance = this;
        setProjectionMatrix(
            glm::perspective(k_camera_vfov, Renderer::getWindowAspectRatio(), k_camera_z_near, k_camera_z_far));
    }

    return success;
}

void Renderer::destroy()
{
    if (m_FontTexture)
    {
        glDeleteTextures(1, &m_FontTexture);
        ImGui::GetIO().Fonts->TexID = 0;
        m_FontTexture = 0;
    }

    ImGui::Shutdown();

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

bool Renderer::onSDLEvent(const SDL_Event *event)
{
    ImGuiIO& io = ImGui::GetIO();

    switch (event->type)
    {
    case SDL_MOUSEWHEEL:
        {
            if (event->wheel.y > 0)
                m_MouseWheel = 1;
            if (event->wheel.y < 0)
                m_MouseWheel = -1;

            return true;
        }
    case SDL_MOUSEBUTTONDOWN:
        {
            if (event->button.button == SDL_BUTTON_LEFT) m_MousePressed[0] = true;
            if (event->button.button == SDL_BUTTON_RIGHT) m_MousePressed[1] = true;
            if (event->button.button == SDL_BUTTON_MIDDLE) m_MousePressed[2] = true;

            return true;
        }
    case SDL_TEXTINPUT:
        {
            ImGuiIO& io = ImGui::GetIO();

            io.AddInputCharactersUTF8(event->text.text);

            return true;
        }
    case SDL_KEYDOWN:
    case SDL_KEYUP:
        {
            int key = event->key.keysym.sym & ~SDLK_SCANCODE_MASK;

            io.KeysDown[key] = (event->type == SDL_KEYDOWN);
            io.KeyShift = ((SDL_GetModState() & KMOD_SHIFT) != 0);
            io.KeyCtrl = ((SDL_GetModState() & KMOD_CTRL) != 0);
            io.KeyAlt = ((SDL_GetModState() & KMOD_ALT) != 0);

            return true;
        }
    }

    return false;
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
    ImGuiIO& io = ImGui::GetIO();

    if (!m_FontTexture)
    {
        // Build texture
        unsigned char* pixels;
        int width, height;
        io.Fonts->GetTexDataAsAlpha8(&pixels, &width, &height);

        // Create texture
        glGenTextures(1, &m_FontTexture);
        glBindTexture(GL_TEXTURE_2D, m_FontTexture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, width, height, 0, GL_ALPHA, GL_UNSIGNED_BYTE, pixels);

        // Store our identifier
        io.Fonts->TexID = (void *)(intptr_t)m_FontTexture;

        // Cleanup (don't clear the input data if you want to append new fonts later)
        io.Fonts->ClearInputData();
        io.Fonts->ClearTexData();
    }

    // Setup display size (every frame to accommodate for window resizing)
    int w, h;
    SDL_GetWindowSize(m_window, &w, &h);
    io.DisplaySize = ImVec2((float)w, (float)h);

    // Setup time step
    Uint32 time = SDL_GetTicks();
    double current_time = time / 1000.0;
    io.DeltaTime = m_Time > 0.0 ? (float)(current_time - m_Time) : (float)(1.0f/60.0f);
    m_Time = current_time;

    // Setup inputs
    // (we already got mouse wheel, keyboard keys & characters from glfw callbacks polled in glfwPollEvents())
    int mx, my;
    Uint32 mouseMask = SDL_GetMouseState(&mx, &my);
    if (SDL_GetWindowFlags(m_window) & SDL_WINDOW_MOUSE_FOCUS)
    {
    	io.MousePos = ImVec2((float)mx, (float)my);   // Mouse position, in pixels (set to -1,-1 if no mouse / on another screen, etc.)
    }
    else
    {
    	io.MousePos = ImVec2(-1,-1);
    }
   
	io.MouseDown[0] = m_MousePressed[0] || (mouseMask & SDL_BUTTON(SDL_BUTTON_LEFT)) != 0;		// If a mouse press event came, always pass it as "mouse held this frame", so we don't miss click-release events that are shorter than 1 frame.
	io.MouseDown[1] = m_MousePressed[1] || (mouseMask & SDL_BUTTON(SDL_BUTTON_RIGHT)) != 0;
	io.MouseDown[2] = m_MousePressed[2] || (mouseMask & SDL_BUTTON(SDL_BUTTON_MIDDLE)) != 0;
    m_MousePressed[0] = m_MousePressed[1] = m_MousePressed[2] = false;

    io.MouseWheel = m_MouseWheel;
    m_MouseWheel = 0.0f;

    // Hide OS mouse cursor if ImGui is drawing it
    SDL_ShowCursor(io.MouseDrawCursor ? 0 : 1);

    // Start the frame
    ImGui::NewFrame();

    m_isRenderingUI= true;
}

void Renderer::renderUIEnd()
{
    glViewport(0, 0, (int)ImGui::GetIO().DisplaySize.x, (int)ImGui::GetIO().DisplaySize.y);    
    glClear(GL_DEPTH_BUFFER_BIT);
    ImGui::Render();

    m_isRenderingUI= false;
}

void Renderer::renderEnd()
{
    SDL_GL_SwapWindow(m_window);
}

//-- Drawing Methods -----
void drawArrow(
    const glm::mat4 &transform,
    const glm::vec3 &start, 
    const glm::vec3 &end, 
    const float headFraction, 
    const glm::vec3 &color)
{
    assert(Renderer::getIsRenderingStage());

    const glm::vec3 headAxis= end-start;
    const float headSize= headAxis.length()*headFraction*0.1f;
    const glm::vec3 headOrigin= glm_vec3_lerp(end, start, headFraction);

    const glm::vec3 worldUp= glm::vec3(0, 1, 0);
    const glm::vec3 headForward= glm::normalize(headAxis);
    const glm::vec3 headLeft= glm::normalize(glm::cross(worldUp, headForward));
    const glm::vec3 headUp= glm::normalize(glm::cross(headForward, headLeft));

    const glm::vec3 headXPos= headOrigin - headLeft*headSize;
    const glm::vec3 headXNeg= headOrigin + headLeft*headSize;
    const glm::vec3 headYPos= headOrigin + headUp*headSize;
    const glm::vec3 headYNeg= headOrigin - headUp*headSize;
   
    glColor3fv(glm::value_ptr(color));

    glPushMatrix();
    glMultMatrixf(glm::value_ptr(transform));

    glBegin(GL_LINES);

    glVertex3fv(glm::value_ptr(start)); glVertex3fv(glm::value_ptr(end));
        
    glVertex3fv(glm::value_ptr(headXPos)); glVertex3fv(glm::value_ptr(headYPos));
    glVertex3fv(glm::value_ptr(headYPos)); glVertex3fv(glm::value_ptr(headXNeg));
    glVertex3fv(glm::value_ptr(headXNeg)); glVertex3fv(glm::value_ptr(headYNeg));
    glVertex3fv(glm::value_ptr(headYNeg)); glVertex3fv(glm::value_ptr(headXPos));

    glVertex3fv(glm::value_ptr(headXPos)); glVertex3fv(glm::value_ptr(end));
    glVertex3fv(glm::value_ptr(headYPos)); glVertex3fv(glm::value_ptr(end));
    glVertex3fv(glm::value_ptr(headXNeg)); glVertex3fv(glm::value_ptr(end));
    glVertex3fv(glm::value_ptr(headYNeg)); glVertex3fv(glm::value_ptr(end));

    glVertex3fv(glm::value_ptr(headXPos)); glVertex3fv(glm::value_ptr(headXNeg));
    glVertex3fv(glm::value_ptr(headYPos)); glVertex3fv(glm::value_ptr(headYNeg));

    glEnd();

    glPopMatrix();
}

void drawTextAtWorldPosition(
    const glm::mat4 &transform, 
    const glm::vec3 &position, 
    const char *format, 
    ...)
{
    assert(Renderer::getIsRenderingStage());

    // Render with the default font
    const FontAsset *font= AssetManager::getInstance()->getDefaultFont();

    // Convert the world space coordinates into screen space
    const glm::vec3 transformed_position= glm::vec3(transform * glm::vec4(position, 1.f));
    const int screenWidth= static_cast<int>(ImGui::GetIO().DisplaySize.x);
    const int screenHeight= static_cast<int>(ImGui::GetIO().DisplaySize.y);
    glm::vec3 screenCoords =
        glm::project(
            transformed_position, 
            Renderer::getCurrentCameraViewMatrix(), 
            Renderer::getCurrentProjectionMatrix(),
            glm::vec4(0, screenHeight, screenWidth, -screenHeight));
    const float initial_x= screenCoords.x;

    // Bake out the text string
    char text[1024];
    va_list args;
    va_start(args, format);
    int w = vsnprintf(text, sizeof(text), format, args);
    text[sizeof(text)-1] = 0;
    va_end(args);

    // Save a back up of the projection matrix and replace with an orthographic projection,
    // Where units = screen pixels, origin at top left
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    const glm::mat4 ortho_projection= glm::ortho(
        0.f, (float)screenWidth, // left, right
        (float)screenHeight, 0.f, // bottom, top
        -1.0f, 1.0f); // zNear, zFar
    glLoadMatrixf(glm::value_ptr(ortho_projection));

    // Save a backup of the modelview matrix and replace with the identity matrix
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // Bind the font texture
    glBindTexture(GL_TEXTURE_2D, font->texture_id);
    glColor3f(1.f, 1.f, 1.f);

    // Render the text quads
    glBegin(GL_QUADS);
    char *next_character= text;
    while (*next_character) 
    {
        char ascii_character= *next_character;

        if (int(ascii_character) >= 32 && int(ascii_character) < 128)
        {
            stbtt_aligned_quad glyph_quad;
            int char_index= (int)ascii_character - 32;

            stbtt_GetBakedQuad(
                const_cast<stbtt_bakedchar *>(font->cdata), 
                font->texture_width, font->texture_height, 
                char_index, 
                &screenCoords.x, &screenCoords.y, // x position advances with character by the glyph pixel widthorbit
                &glyph_quad,
                1); // opengl_fillrule= true
            glTexCoord2f(glyph_quad.s0,glyph_quad.t0); glVertex2f(glyph_quad.x0,glyph_quad.y0);
            glTexCoord2f(glyph_quad.s1,glyph_quad.t0); glVertex2f(glyph_quad.x1,glyph_quad.y0);
            glTexCoord2f(glyph_quad.s1,glyph_quad.t1); glVertex2f(glyph_quad.x1,glyph_quad.y1);
            glTexCoord2f(glyph_quad.s0,glyph_quad.t1); glVertex2f(glyph_quad.x0,glyph_quad.y1);
        }
        else if (ascii_character == '\n')
        {
            screenCoords.x= initial_x;
            screenCoords.y+= font->glyphPixelHeight;
        }

        ++next_character;
    }
    glEnd();

    // rebind the default texture
    glBindTexture(GL_TEXTURE_2D, 0);

    // Restore the projection matrix
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    // Restore the modelview matrix
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

void drawFullscreenTexture(const unsigned int texture_id)
{
    // Save a backup of the projection matrix and replace with the identity matrix
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    // Save a backup of the modelview matrix and replace with the identity matrix
    glMatrixMode(GL_MODELVIEW); 
    glPushMatrix();
    glLoadIdentity();

    // Clear the screen and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Bind the texture to draw
    glBindTexture(GL_TEXTURE_2D, texture_id);

    // Fill the screen with the texture
    glColor3f(1.f, 1.f, 1.f);
    glBegin(GL_QUADS);
        glTexCoord2f(0.f, 1.f); glVertex2f(-1.f, -1.f);
        glTexCoord2f(1.f, 1.f); glVertex2f(1.f, -1.f);
        glTexCoord2f(1.f, 0.f); glVertex2f(1.f, 1.f);
        glTexCoord2f(0.f, 0.f); glVertex2f(-1.f, 1.f);
    glEnd();

    // rebind the default texture
    glBindTexture(GL_TEXTURE_2D, 0);

    // Clear the depth buffer to allow overdraw 
    glClear(GL_DEPTH_BUFFER_BIT);

    // Restore the projection matrix
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    // Restore the modelview matrix
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

void drawPointCloudProjection(
	const PSMVector2f *points,
	const int point_count, 
	const float point_size,
	const glm::vec3 &color,
	const float trackerWidth, 
	const float trackerHeight)
{
	assert(Renderer::getIsRenderingStage());

	// Clear the depth buffer to allow overdraw 
	glClear(GL_DEPTH_BUFFER_BIT);

	// Save a backup of the projection matrix 
	// and replace with a projection that maps the tracker image coordinates over the whole screen
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0.f, trackerWidth - 1.f, trackerHeight - 1.f, 0.f, 1.0f, -1.0f);

	// Save a backup of the modelview matrix and replace with the identity matrix
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	// Draw a small color "+" for each point in the point count
	glLineWidth(2.f);
	for (int point_index = 0; point_index < point_count; ++point_index)
	{
		const PSMVector2f *point = &points[point_index];

		glLineWidth(2.f);
		glBegin(GL_LINES);
		glColor3fv(glm::value_ptr(color));
		glVertex3f(point->x - point_size, point->y, 0.5f);
		glVertex3f(point->x + point_size, point->y, 0.5f);
		glVertex3f(point->x, point->y + point_size, 0.5f);
		glVertex3f(point->x, point->y - point_size, 0.5f);
		glEnd();
	}
	glLineWidth(1.f);

	// Restore the projection matrix
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	// Restore the modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

void drawTransformedVolume(const glm::mat4 &transform, const PSMVolume *volume, const glm::vec3 &color)
{
    assert(Renderer::getIsRenderingStage());

    glPushMatrix();
        glMultMatrixf(glm::value_ptr(transform));
        glColor3fv(glm::value_ptr(color));

        glBegin(GL_LINES);

        int previous_index= volume->vertex_count - 1;
        for (int index = 0; index < volume->vertex_count; ++index)
        {
            const PSMVector3f &previous_vert = volume->vertices[previous_index];
            const PSMVector3f &vert = volume->vertices[index];

            glm::vec3 prev_lower(previous_vert.x, previous_vert.y, previous_vert.z);
            glm::vec3 prev_upper(previous_vert.x, previous_vert.y + volume->up_height, previous_vert.z);
            glm::vec3 lower(vert.x, vert.y, vert.z);
            glm::vec3 upper(vert.x, vert.y + volume->up_height, vert.z);

            glVertex3fv(glm::value_ptr(prev_lower)); glVertex3fv(glm::value_ptr(lower));
            glVertex3fv(glm::value_ptr(prev_upper)); glVertex3fv(glm::value_ptr(upper));
            glVertex3fv(glm::value_ptr(lower)); glVertex3fv(glm::value_ptr(upper));

            previous_index= index;
        }

        glEnd();
    glPopMatrix();
}

void drawTransformedAxes(const glm::mat4 &transform, float scale)
{
    drawTransformedAxes(transform, scale, scale, scale);
}

void drawTransformedAxes(const glm::mat4 &transform, float xScale, float yScale, float zScale)
{
    assert(Renderer::getIsRenderingStage());

    glm::vec3 origin(0.f, 0.f, 0.f);
    glm::vec3 xAxis(xScale, 0.f, 0.f);
    glm::vec3 yAxis(0.f, yScale, 0.f);
    glm::vec3 zAxis(0.f, 0.f, zScale);
   
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
    drawTransformedBox(transform, -half_extents, half_extents, color);
}

void drawTransformedBox(const glm::mat4 &transform, const glm::vec3 &box_min, const glm::vec3 &box_max, const glm::vec3 &color)
{
    assert(Renderer::getIsRenderingStage());

    glm::vec3 v0(box_max.x, box_max.y, box_max.z);
    glm::vec3 v1(box_min.x, box_max.y, box_max.z);
    glm::vec3 v2(box_min.x, box_max.y, box_min.z);
    glm::vec3 v3(box_max.x, box_max.y, box_min.z);
    glm::vec3 v4(box_max.x, box_min.y, box_max.z);
    glm::vec3 v5(box_min.x, box_min.y, box_max.z);
    glm::vec3 v6(box_min.x, box_min.y, box_min.z);
    glm::vec3 v7(box_max.x, box_min.y, box_min.z);

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

void drawTransformedFrustum(const glm::mat4 &transform, const PSMFrustum *frustum, const glm::vec3 &color)
{
    assert(Renderer::getIsRenderingStage());

    const float HRatio = tanf(frustum->HFOV / 2.f);
    const float VRatio = tanf(frustum->VFOV / 2.f);

    glm::vec3 left(frustum->left.x, frustum->left.y, frustum->left.z);
    glm::vec3 up(frustum->up.x, frustum->up.y, frustum->up.z);
    glm::vec3 forward(frustum->forward.x, frustum->forward.y, frustum->forward.z);
    glm::vec3 origin(frustum->origin.x, frustum->origin.y, frustum->origin.z);

    glm::vec3 nearX = left*frustum->zNear*HRatio;
    glm::vec3 farX = left*frustum->zFar*HRatio;

    glm::vec3 nearY = up*frustum->zNear*VRatio;
    glm::vec3 farY = up*frustum->zFar*VRatio;

    glm::vec3 nearZ = forward*frustum->zNear;
    glm::vec3 farZ = forward*frustum->zFar;

    glm::vec3 nearCenter = origin + nearZ;
    glm::vec3 near0 = origin + nearX + nearY + nearZ;
    glm::vec3 near1 = origin - nearX + nearY + nearZ;
    glm::vec3 near2 = origin - nearX - nearY + nearZ;
    glm::vec3 near3 = origin + nearX - nearY + nearZ;

    glm::vec3 far0 = origin + farX + farY + farZ;
    glm::vec3 far1 = origin - farX + farY + farZ;
    glm::vec3 far2 = origin - farX - farY + farZ;
    glm::vec3 far3 = origin + farX - farY + farZ;

    glPushMatrix();
    glMultMatrixf(glm::value_ptr(transform));
    glColor3fv(glm::value_ptr(color));

    glBegin(GL_LINES);

    glVertex3fv(glm::value_ptr(near0)); glVertex3fv(glm::value_ptr(near1));
    glVertex3fv(glm::value_ptr(near1)); glVertex3fv(glm::value_ptr(near2));
    glVertex3fv(glm::value_ptr(near2)); glVertex3fv(glm::value_ptr(near3));
    glVertex3fv(glm::value_ptr(near3)); glVertex3fv(glm::value_ptr(near0));

    glVertex3fv(glm::value_ptr(far0)); glVertex3fv(glm::value_ptr(far1));
    glVertex3fv(glm::value_ptr(far1)); glVertex3fv(glm::value_ptr(far2));
    glVertex3fv(glm::value_ptr(far2)); glVertex3fv(glm::value_ptr(far3));
    glVertex3fv(glm::value_ptr(far3)); glVertex3fv(glm::value_ptr(far0));

    glVertex3fv(glm::value_ptr(origin)); glVertex3fv(glm::value_ptr(far0));
    glVertex3fv(glm::value_ptr(origin)); glVertex3fv(glm::value_ptr(far1));
    glVertex3fv(glm::value_ptr(origin)); glVertex3fv(glm::value_ptr(far2));
    glVertex3fv(glm::value_ptr(origin)); glVertex3fv(glm::value_ptr(far3));

    glVertex3fv(glm::value_ptr(origin));
    glColor3ub(0, 255, 0);
    glVertex3fv(glm::value_ptr(nearCenter));

    glEnd();

    glPopMatrix();
}

void drawPointCloud(const glm::mat4 &transform, const glm::vec3 &color, const float *points, const int point_count)
{
    assert(Renderer::getIsRenderingStage());

    glColor3fv(glm::value_ptr(color));

    glPushMatrix();
    glMultMatrixf(glm::value_ptr(transform));
    glEnableClientState(GL_VERTEX_ARRAY);
    glPointSize(5);
    glVertexPointer(3, GL_FLOAT, 0, points);
    glDrawArrays(GL_POINTS, 0, point_count);
    glDisableClientState(GL_VERTEX_ARRAY);
    glPopMatrix();
}

void drawEllipsoid(
    const glm::mat4 &transform,
    const glm::vec3 &color,
    const glm::mat3 &basis,
    const glm::vec3 &center,
    const glm::vec3 &extents,
    const int subdiv)
{
    assert(subdiv >= 3);
    assert(Renderer::getIsRenderingStage());

    glColor3fv(glm::value_ptr(color));
    glPushMatrix();
    glMultMatrixf(glm::value_ptr(transform));

    const float angleStep = k_real_two_pi / static_cast<float>(subdiv);
    float angle;

    const glm::vec3 x_axis = basis[0];
    const glm::vec3 y_axis = basis[1];
    const glm::vec3 z_axis = basis[2];

    const float x_extent = extents[0];
    const float y_extent = extents[1];
    const float z_extent = extents[2];

    glBegin(GL_LINE_STRIP);
    angle = 0.f;
    for (int index = 0; index <= subdiv; ++index)
    {
        glm::vec3 point = x_extent*x_axis*cosf(angle) + y_extent*y_axis*sinf(angle) + center;

        glVertex3fv(glm::value_ptr(point));
        angle += angleStep;
    }
    glEnd();

    glBegin(GL_LINE_STRIP);
    angle = 0.f;
    for (int index = 0; index <= subdiv; ++index)
    {
        glm::vec3 point = x_extent*x_axis*cosf(angle) + z_extent*z_axis*sinf(angle) + center;

        glVertex3fv(glm::value_ptr(point));
        angle += angleStep;
    }
    glEnd();

    glBegin(GL_LINE_STRIP);
    angle = 0.f;
    for (int index = 0; index <= subdiv; ++index)
    {
        glm::vec3 point = y_extent*y_axis*cosf(angle) + z_extent*z_axis*sinf(angle) + center;

        glVertex3fv(glm::value_ptr(point));
        angle += angleStep;
    }
    glEnd();

    glPopMatrix();
}

void drawLineStrip(const glm::mat4 &transform, const glm::vec3 &color, const float *points, const int point_count)
{
    assert(Renderer::getIsRenderingStage());

    glColor3fv(glm::value_ptr(color));
    glPushMatrix();
        glMultMatrixf(glm::value_ptr(transform));

        glBegin(GL_LINE_STRIP);
        for (int sampleIndex= 0; sampleIndex < point_count; ++sampleIndex)
        {
            glVertex3fv(&points[sampleIndex*3]);
        }
        glEnd();
    glPopMatrix();
}

void drawQuadList2d(const float trackerWidth, const float trackerHeight, const glm::vec3 &color, const float *points2d, const int point_count)
{
    assert(Renderer::getIsRenderingStage());
    assert((point_count % 4) == 0);

    // Clear the depth buffer to allow overdraw 
    glClear(GL_DEPTH_BUFFER_BIT);

    // Save a backup of the projection matrix 
    // and replace with a projection that maps the tracker image coordinates over the whole screen
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.f, trackerWidth, trackerHeight, 0, 1.0f, -1.0f);

    // Save a backup of the modelview matrix and replace with the identity matrix
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // Draw line strip connecting all of the points on the line strip
    glColor3fv(glm::value_ptr(color));
    glBegin(GL_LINES);
    for (int sampleIndex= 0; sampleIndex < point_count; sampleIndex+=4)
    {
        glVertex3f(points2d[sampleIndex*2+0], points2d[sampleIndex*2+1], 0.5f);
        glVertex3f(points2d[sampleIndex*2+2], points2d[sampleIndex*2+3], 0.5f);

        glVertex3f(points2d[sampleIndex*2+2], points2d[sampleIndex*2+3], 0.5f);
        glVertex3f(points2d[sampleIndex*2+4], points2d[sampleIndex*2+5], 0.5f);

        glVertex3f(points2d[sampleIndex*2+4], points2d[sampleIndex*2+5], 0.5f);
        glVertex3f(points2d[sampleIndex*2+6], points2d[sampleIndex*2+7], 0.5f);

        glVertex3f(points2d[sampleIndex*2+6], points2d[sampleIndex*2+7], 0.5f);
        glVertex3f(points2d[sampleIndex*2+0], points2d[sampleIndex*2+1], 0.5f);
    }
    glEnd();

    // Restore the projection matrix
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    // Restore the modelview matrix
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

void drawOpenCVChessBoard(const float trackerWidth, const float trackerHeight, const float *points2d, const int point_count, bool validPoints)
{
    assert(Renderer::getIsRenderingStage());

    // Clear the depth buffer to allow overdraw 
    glClear(GL_DEPTH_BUFFER_BIT);

    // Save a backup of the projection matrix 
    // and replace with a projection that maps the tracker image coordinates over the whole screen
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.f, trackerWidth, trackerHeight, 0, 1.0f, -1.0f);

    // Save a backup of the modelview matrix and replace with the identity matrix
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // Draw line strip connecting all of the corners on the chessboard
    glBegin(GL_LINE_STRIP);
    for (int sampleIndex= 0; sampleIndex < point_count; ++sampleIndex)
    {
        if (validPoints)
        {
            // Match how OpenCV colors the line strip (red -> blue i.e. hue angle 0 to 255 degrees)
            const float hue= static_cast<float>(sampleIndex * 255 / point_count);
            float r, g, b;

            HSVtoRGB(hue, 1.f, 1.f, r, g, b);
            glColor3f(r, g, b);
        }
        else
        {
            glColor3f(1.f, 0.f, 0.f);
        }

        glVertex3f(points2d[sampleIndex*2+0], points2d[sampleIndex*2+1], 0.5f);
    }
    glEnd();

    // Draw circles at each corner
    for (int sampleIndex= 0; sampleIndex < point_count; ++sampleIndex)
    {
        const float radius= 2.f;
        const int subdiv = 8;
        const float angleStep = k_real_two_pi / static_cast<float>(subdiv);
        float angle = 0.f;

        if (validPoints)
        {
            // Match how OpenCV colors the line strip (red -> blue i.e. hue angle 0 to 255 degrees)
            const float hue= static_cast<float>(sampleIndex * 255 / point_count);
            float r, g, b;

            HSVtoRGB(hue, 1.f, 1.f, r, g, b);
            glColor3f(r, g, b);
        }
        else
        {
            glColor3f(1.f, 0.f, 0.f);
        }

        glBegin(GL_LINE_STRIP);
        for (int index = 0; index <= subdiv; ++index)
        {
            glm::vec3 point(
                radius*cosf(angle) + points2d[sampleIndex*2+0],
                radius*sinf(angle) + points2d[sampleIndex*2+1],
                0.5f);

            glVertex3fv(glm::value_ptr(point));
            angle += angleStep;
        }
        glEnd();
    }

    // Restore the projection matrix
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    // Restore the modelview matrix
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

void drawPoseArrayStrip(const PSMPosef *poses, const int poseCount, const glm::vec3 &color)
{
    glColor3fv(glm::value_ptr(color));
    glBegin(GL_LINE_STRIP);

    for (int sampleIndex = 0; sampleIndex < poseCount; ++sampleIndex)
    {
        const PSMPosef &pose = poses[sampleIndex];

        glVertex3f(pose.Position.x, pose.Position.y, pose.Position.z);
    }

    glEnd();
}

void drawPS3EyeModel(const glm::mat4 &transform)
{
    assert(Renderer::getIsRenderingStage());

    int textureID= AssetManager::getInstance()->getPS3EyeTextureAsset()->texture_id;

    glBindTexture(GL_TEXTURE_2D, textureID);

    glColor3f(1.f, 1.f, 1.f);

    glPushMatrix();
        glMultMatrixf(glm::value_ptr(transform));
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        glVertexPointer(3, GL_FLOAT, 0, ps3eyeVerts);
        glTexCoordPointer(2, GL_FLOAT, 0, ps3eyeTexCoords);
        glDrawArrays(GL_TRIANGLES, 0, ps3eyeNumVerts);
        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glPopMatrix();

    // rebind the default texture
    glBindTexture(GL_TEXTURE_2D, 0); 
}

void drawPSMoveModel(const glm::mat4 &transform, const glm::vec3 &color)
{
    assert(Renderer::getIsRenderingStage());

    int textureID= AssetManager::getInstance()->getPSMoveTextureAsset()->texture_id;

    glBindTexture(GL_TEXTURE_2D, textureID);

    glPushMatrix();
        glMultMatrixf(glm::value_ptr(transform));

        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        
        glColor3f(1.f, 1.f, 1.f);
        glVertexPointer(3, GL_FLOAT, 0, psmovebodyVerts);
        glTexCoordPointer(2, GL_FLOAT, 0, psmovebodyTexCoords);
        glDrawArrays(GL_TRIANGLES, 0, psmovebodyNumVerts);

        glColor3fv(glm::value_ptr(color));
        glVertexPointer(3, GL_FLOAT, 0, psmovebulbVerts);
        glTexCoordPointer(2, GL_FLOAT, 0, psmovebulbTexCoords);
        glDrawArrays(GL_TRIANGLES, 0, psmovebulbNumVerts);

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);

    glPopMatrix();

    // rebind the default texture
    glBindTexture(GL_TEXTURE_2D, 0); 
}

void drawPSNaviModel(const glm::mat4 &transform)
{
    assert(Renderer::getIsRenderingStage());

    int textureID= AssetManager::getInstance()->getPSNaviTextureAsset()->texture_id;

    glBindTexture(GL_TEXTURE_2D, textureID);
    glColor3f(1.f, 1.f, 1.f);

    glPushMatrix();
        glMultMatrixf(glm::value_ptr(transform));
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        glVertexPointer(3, GL_FLOAT, 0, psnaviVerts);
        glTexCoordPointer(2, GL_FLOAT, 0, psnaviTexCoords);
        glDrawArrays(GL_TRIANGLES, 0, psnaviNumVerts);
        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glPopMatrix();

    // rebind the default texture
    glBindTexture(GL_TEXTURE_2D, 0); 
}

void drawPSDualShock4Model(const glm::mat4 &transform, const glm::vec3 &color)
{
    assert(Renderer::getIsRenderingStage());

    int textureID = AssetManager::getInstance()->getPSDualShock4TextureAsset()->texture_id;

    glBindTexture(GL_TEXTURE_2D, textureID);
    glColor3f(1.f, 1.f, 1.f);

    glPushMatrix();
        glMultMatrixf(glm::value_ptr(transform));
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        
		glVertexPointer(3, GL_FLOAT, 0, ds4bodyVerts);
        glTexCoordPointer(2, GL_FLOAT, 0, ds4bodyTexCoords);
        glDrawArrays(GL_TRIANGLES, 0, ds4bodyNumVerts);

		glDisableClientState(GL_TEXTURE_COORD_ARRAY);

		glColor3fv(glm::value_ptr(color));
		glVertexPointer(3, GL_FLOAT, 0, ds4lightbarVerts);
		glDrawArrays(GL_TRIANGLES, 0, ds4lightbarNumVerts);

        glDisableClientState(GL_VERTEX_ARRAY);
    glPopMatrix();

    // rebind the default texture
    glBindTexture(GL_TEXTURE_2D, 0); 
}

void drawTrackerList(const PSMClientTrackerInfo *trackerList, const int trackerCount)
{
	glm::mat4 psmove_tracking_space_to_chaperone_space = glm::mat4(1.f);


	// Draw the frustum for each tracking camera.
	// The frustums are defined in PSMove tracking space.
	// We need to transform them into chaperone space to display them along side the HMD.
	for (int tracker_index = 0; tracker_index < trackerCount; ++tracker_index)
	{
		const PSMClientTrackerInfo *trackerInfo= &trackerList[tracker_index];
		const PSMPosef tracker_pose = trackerInfo->tracker_pose;
		const glm::mat4 chaperoneSpaceTransform = psm_posef_to_glm_mat4(tracker_pose);

		PSMFrustum frustum;

		PSM_FrustumSetPose(&frustum, &tracker_pose);

		// Convert the FOV angles to radians for rendering purposes
		frustum.HFOV = trackerInfo->tracker_hfov * k_degrees_to_radians;
		frustum.VFOV = trackerInfo->tracker_vfov * k_degrees_to_radians;
		frustum.zNear = trackerInfo->tracker_znear;
		frustum.zFar = trackerInfo->tracker_zfar;

		drawTransformedFrustum(psmove_tracking_space_to_chaperone_space, &frustum, k_psmove_frustum_color);
		drawTransformedAxes(chaperoneSpaceTransform, 20.f);
	}
}

void drawMorpheusModel(const glm::mat4 &transform)
{
    assert(Renderer::getIsRenderingStage());

    int textureID= AssetManager::getInstance()->getMorpheusTextureAsset()->texture_id;

    glBindTexture(GL_TEXTURE_2D, textureID);

    glPushMatrix();
        glMultMatrixf(glm::value_ptr(transform));

        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        
        glColor3f(1.f, 1.f, 1.f);
        glVertexPointer(3, GL_FLOAT, 0, morpheusVerts);
        glTexCoordPointer(2, GL_FLOAT, 0, morpheusTexCoords);
        glDrawArrays(GL_TRIANGLES, 0, morpheusNumVerts);

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);

    glPopMatrix();

    // rebind the default texture
    glBindTexture(GL_TEXTURE_2D, 0); 
}

// -- IMGUI Callbacks -----
static const char* ImGui_ImplSdl_GetClipboardText()
{
	return SDL_GetClipboardText();
}

static void ImGui_ImplSdl_SetClipboardText(const char* text)
{
    SDL_SetClipboardText(text);
}

// This is the main rendering function that you have to implement and provide to ImGui (via setting up 'RenderDrawListsFn' in the ImGuiIO structure)
// If text or lines are blurry when integrating ImGui in your engine:
// - in your Render function, try translating your projection matrix by (0.5f,0.5f) or (0.375f,0.375f)
static void ImGui_ImplSdl_RenderDrawLists(ImDrawData* draw_data)
{
    // We are using the OpenGL fixed pipeline to make the example code simpler to read!
    // Setup render state: alpha-blending enabled, no face culling, no depth testing, scissor enabled, vertex/texcoord/color pointers.
    GLint last_texture; glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
    GLint last_viewport[4]; glGetIntegerv(GL_VIEWPORT, last_viewport);
    glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_TRANSFORM_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_SCISSOR_TEST);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    glEnable(GL_TEXTURE_2D);
    //glUseProgram(0); // You may want this if using this code in an OpenGL 3+ context

    // Handle cases of screen coordinates != from framebuffer coordinates (e.g. retina displays)
    ImGuiIO& io = ImGui::GetIO();
    float fb_height = io.DisplaySize.y * io.DisplayFramebufferScale.y;
    draw_data->ScaleClipRects(io.DisplayFramebufferScale);

    // Setup viewport, orthographic projection matrix
    glViewport(0, 0, (GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0f, io.DisplaySize.x, io.DisplaySize.y, 0.0f, -1.0f, +1.0f);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // Render command lists
    #define OFFSETOF(TYPE, ELEMENT) ((size_t)&(((TYPE *)0)->ELEMENT))
    for (int n = 0; n < draw_data->CmdListsCount; n++)
    {
        const ImDrawList* cmd_list = draw_data->CmdLists[n];
        const unsigned char* vtx_buffer = (const unsigned char*)&cmd_list->VtxBuffer.front();
        const ImDrawIdx* idx_buffer = &cmd_list->IdxBuffer.front();
        glVertexPointer(2, GL_FLOAT, sizeof(ImDrawVert), (void*)(vtx_buffer + OFFSETOF(ImDrawVert, pos)));
        glTexCoordPointer(2, GL_FLOAT, sizeof(ImDrawVert), (void*)(vtx_buffer + OFFSETOF(ImDrawVert, uv)));
        glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(ImDrawVert), (void*)(vtx_buffer + OFFSETOF(ImDrawVert, col)));

        for (int cmd_i = 0; cmd_i < cmd_list->CmdBuffer.size(); cmd_i++)
        {
            const ImDrawCmd* pcmd = &cmd_list->CmdBuffer[cmd_i];
            if (pcmd->UserCallback)
            {
                pcmd->UserCallback(cmd_list, pcmd);
            }
            else
            {
                glBindTexture(GL_TEXTURE_2D, (GLuint)(intptr_t)pcmd->TextureId);
                glScissor((int)pcmd->ClipRect.x, (int)(fb_height - pcmd->ClipRect.w), (int)(pcmd->ClipRect.z - pcmd->ClipRect.x), (int)(pcmd->ClipRect.w - pcmd->ClipRect.y));
                glDrawElements(GL_TRIANGLES, (GLsizei)pcmd->ElemCount, sizeof(ImDrawIdx) == 2 ? GL_UNSIGNED_SHORT : GL_UNSIGNED_INT, idx_buffer);
            }
            idx_buffer += pcmd->ElemCount;
        }
    }
    #undef OFFSETOF

    // Restore modified state
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
    glBindTexture(GL_TEXTURE_2D, last_texture);
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
    glViewport(last_viewport[0], last_viewport[1], (GLsizei)last_viewport[2], (GLsizei)last_viewport[3]);
}


//-- Utilities -----
// From: https://www.cs.rit.edu/~ncs/color/t_convert.html
// r,g,b values are from 0 to 1
// h = [0,360], s = [0,1], v = [0,1]
//		if s == 0, then h = -1 (undefined)
void RGBtoHSV(float r, float g, float b, float &h, float &s, float &v)
{
    float min = std::min(std::min(r, g), b);
    float max = std::max(std::max(r, g), b);

    v = max;				// v
    float delta = max - min;
    if( max != 0 )
    {
        s = delta / max;		// s
    }
    else 
    {
        // r = g = b = 0		// s = 0, v is undefined
        s = 0;
        h = -1;
        return;
    }

    if( r == max )
    {
        h = ( g - b ) / delta;		// between yellow & magenta
    }
    else if( g == max )
    {
        h = 2 + ( b - r ) / delta;	// between cyan & yellow
    }
    else
    {
        h = 4 + ( r - g ) / delta;	// between magenta & cyan
    }

    h *= 60;				// degrees
    if( h < 0 )
    {
        h += 360;
    }
}

// From: https://www.cs.rit.edu/~ncs/color/t_convert.html
// r,g,b values are from 0 to 1
// h = [0,360], s = [0,1], v = [0,1]
//		if s == 0, then h = -1 (undefined)
void HSVtoRGB(float h, float s, float v, float &r, float &g, float &b)
{
    if( s == 0 ) 
    {
        // achromatic (grey)
        r = g = b = v;
        return;
    }

    const float sector = h / 60;			// sector 0 to 5
    const int i = static_cast<int>(floorf( sector ));
    const float f = sector - i;			// factorial part of h
    const float p = v * ( 1 - s );
    const float q = v * ( 1 - s * f );
    const float t = v * ( 1 - s * ( 1 - f ) );

    switch( i ) 
    {
    case 0:
        r = v;
        g = t;
        b = p;
        break;
    case 1:
        r = q;
        g = v;
        b = p;
        break;
    case 2:
        r = p;
        g = v;
        b = t;
        break;
    case 3:
        r = p;
        g = q;
        b = v;
        break;
    case 4:
        r = t;
        g = p;
        b = v;
        break;
    default:		// case 5:
        r = v;
        g = p;
        b = q;
        break;
    }
}
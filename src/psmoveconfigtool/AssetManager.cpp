//-- includes -----
#include "AssetManager.h"
#include "Logger.h"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#include "stb_image.h"

#include "SDL_error.h"
#include "SDL_opengl.h"

#include <imgui.h>

//-- constants -----
static const char *k_dk2_texture_filename= "./assets/textures/DK2diffuse.jpg";
static const char *k_psmove_texture_filename= "./assets/textures/PSMoveDiffuse.jpg";
static const char *k_psnavi_texture_filename= "./assets/textures/PSNaviDiffuse.jpg";

static const char *k_default_font_filename= "./assets/fonts/OpenSans-Regular.ttf";
static const float k_default_font_pixel_height= 24.f;

//-- statics -----
AssetManager *AssetManager::m_instance= NULL;

//-- public methods -----
AssetManager::AssetManager()
    : m_dk2TextureId(0)
    //, m_defaultFont()
{
}

AssetManager::~AssetManager()
{
    assert(m_instance== NULL);
}

bool AssetManager::init()
{
    bool success= true;

    if (success)
    {
        success= loadTexture(k_dk2_texture_filename, &m_dk2TextureId);
    }

    if (success)
    {
        success= loadTexture(k_psmove_texture_filename, &m_psmoveTextureId);
    }

    if (success)
    {
        success= loadTexture(k_psnavi_texture_filename, &m_psnaviTextureId);
    }

    if (success)
    {
        // Load Fonts
        ImGuiIO& io = ImGui::GetIO();

        io.Fonts->AddFontDefault();
        io.Fonts->AddFontFromFileTTF(k_default_font_filename, k_default_font_pixel_height);
    }

    if (success)
    {
        m_instance= this;
    }

    return success;
}

void AssetManager::destroy()
{
    if (m_dk2TextureId != 0)
    {
        glDeleteTextures(1, &m_dk2TextureId);
        m_dk2TextureId= 0;
    }

    if (m_psmoveTextureId != 0)
    {
        glDeleteTextures(1, &m_psmoveTextureId);
        m_psmoveTextureId= 0;
    }

    if (m_psnaviTextureId != 0)
    {
        glDeleteTextures(1, &m_psnaviTextureId);
        m_psnaviTextureId= 0;
    }

    m_instance= NULL;
}

//-- private methods -----
bool AssetManager::loadTexture(const char *filename, GLuint *textureId)
{
    bool success= false;

    int pixelWidth=0, pixelHeight=0, channelCount=0;
    stbi_uc *image_buffer= stbi_load(filename, &pixelWidth, &pixelHeight, &channelCount, 3);

    if (image_buffer != NULL)
    {
        GLint glPixelFormat= -1;

        if (channelCount == 3)
        {
            glGenTextures(1, textureId);

            // Typical Texture Generation Using Data From The Bitmap
            glBindTexture(GL_TEXTURE_2D, *textureId);
            glTexImage2D(GL_TEXTURE_2D, 0, 3, pixelWidth, pixelHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, image_buffer);
            glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);

            success= true;
        }
        else
        {
            Log_ERROR("AssetManager::loadTexture", "Image isn't RGB24 pixel format!");
        }

        stbi_image_free(image_buffer);
    }
    else
    {
        Log_ERROR("AssetManager::loadTexture", "Failed to load: %s(%s)", filename, SDL_GetError());
    }

    return success;
}
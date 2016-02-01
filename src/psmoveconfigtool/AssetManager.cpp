//-- includes -----
#include "AssetManager.h"
#include "Logger.h"

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"
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

static const size_t k_kilo= 1<<10;
static const size_t k_meg= 1<<20;

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
        success= loadFont(k_default_font_filename, k_default_font_pixel_height, &m_defaultFont);
    }

    if (success)
    {
        // Load IMGUI Fonts
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

    if (m_defaultFont.textureId != 0)
    {
        glDeleteTextures(1, &m_defaultFont.textureId);
        m_defaultFont.textureId= 0;
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

bool AssetManager::loadFont(const char *filename, const float pixelHeight, AssetManager::FontAsset *fontAsset)
{
    unsigned char *temp_ttf_buffer = NULL;
    unsigned char *temp_bitmap = NULL;

    bool success= true;

    // For now assume all font sprite sheets fit in a 512x512 texture
    fontAsset->textureWidth= 512;
    fontAsset->textureHeight= 512;
    fontAsset->glyphPixelHeight= pixelHeight;

    // Allocate scratch buffers
    temp_ttf_buffer = NULL;
    temp_bitmap = new unsigned char[fontAsset->textureWidth*fontAsset->textureHeight];

    // Load the True Type Font data into memory
    if (success)
    {
        FILE *fp= fopen(k_default_font_filename, "rb");
        if (fp != NULL)
        {
            // obtain file size
            fseek (fp , 0 , SEEK_END);
            size_t fileSize = ftell (fp);
            rewind (fp);

            if (fileSize > 0 && fileSize < 10*k_meg)
            {
                temp_ttf_buffer= new unsigned char[fileSize];
                size_t bytes_read= fread(temp_ttf_buffer, 1, fileSize, fp);

                if (bytes_read != fileSize)
                {
                    Log_ERROR("AssetManager::loadFont", "Failed to load font (%s): failed to read expected # of bytes.", filename);
                    success= false;
                }
            }
            else
            {
                Log_ERROR("AssetManager::loadFont", "Failed to load font (%s): file size invalid", filename);
                success= false;
            }

            fclose(fp);
        }
        else
        {
            Log_ERROR("AssetManager::loadFont", "Failed to open font file (%s)", filename);
            success= false;
        }
    }

    // Build the sprite sheet for the font
    if (success)
    {
        if (stbtt_BakeFontBitmap(
            temp_ttf_buffer, 0, 
            pixelHeight, 
            temp_bitmap, fontAsset->textureWidth, fontAsset->textureHeight, 
            32,96, fontAsset->cdata) <= 0)
        {
            Log_ERROR("AssetManager::loadFont", "Failed to fit font(%s) into %dx%d sprite texture", 
                filename, fontAsset->textureWidth, fontAsset->textureHeight);
            success= false;
        }
    }
    
    // Generate the texture for the font sprite sheet
    if (success)
    {
        glGenTextures(1, &fontAsset->textureId);
        glBindTexture(GL_TEXTURE_2D, fontAsset->textureId);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, 512,512, 0, GL_ALPHA, GL_UNSIGNED_BYTE, temp_bitmap);            
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    }

    // free scratch buffers
    delete[] temp_bitmap;
    if (temp_ttf_buffer != NULL) delete[] temp_ttf_buffer;

    return success;
}

//-- font asset -----
AssetManager::FontAsset::FontAsset()
{
    memset(this, 0, sizeof(FontAsset));
}
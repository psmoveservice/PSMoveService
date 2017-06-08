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
static const char *k_ps3eye_texture_filename= "./assets/textures/PS3EyeDiffuse.jpg";
static const char *k_psmove_texture_filename= "./assets/textures/PSMoveDiffuse.jpg";
static const char *k_psnavi_texture_filename= "./assets/textures/PSNaviDiffuse.jpg";
static const char *k_psdualshock4_texture_filename = "./assets/textures/PSDS4Diffuse.jpg";
static const char *k_virtual_texture_filename = "./assets/textures/VirtualDiffuse.jpg";
static const char *k_morpheus_texture_filename = "./assets/textures/MorpheusDiffuse.jpg";
static const char *k_dk2_texture_filename = "./assets/textures/DK2Diffuse.jpg";

static const char *k_default_font_filename= "./assets/fonts/OpenSans-Regular.ttf";
static const float k_default_font_pixel_height= 24.f;

static const unsigned int k_font_texture_width = 512;
static const unsigned int k_font_texture_height = 512;

static const size_t k_kilo= 1<<10;
static const size_t k_meg= 1<<20;

//-- statics -----
AssetManager *AssetManager::m_instance= NULL;

//-- public methods -----
AssetManager::AssetManager()
    : m_ps3eyeTexture()
	, m_psmoveTexture()
    , m_psnaviTexture()
    , m_psdualshock4Texture()
    , m_morpheusTexture()
    , m_dk2Texture()
    , m_defaultFont()
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
		success= loadTexture(k_ps3eye_texture_filename, &m_ps3eyeTexture);		
	}

    if (success)
    {
        success= loadTexture(k_psmove_texture_filename, &m_psmoveTexture);
    }

    if (success)
    {
        success= loadTexture(k_psnavi_texture_filename, &m_psnaviTexture);
    }

    if (success)
    {
        success = loadTexture(k_psdualshock4_texture_filename, &m_psdualshock4Texture);
    }

    if (success)
    {
        success = loadTexture(k_virtual_texture_filename, &m_virtualTexture);
    }

    if (success)
    {
        success = loadTexture(k_morpheus_texture_filename, &m_morpheusTexture);
    }

    if (success)
    {
        success = loadTexture(k_dk2_texture_filename, &m_dk2Texture);
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
	m_ps3eyeTexture.dispose();
    m_psmoveTexture.dispose();
    m_psnaviTexture.dispose();
    m_psdualshock4Texture.dispose();
    m_morpheusTexture.dispose();
    m_dk2Texture.dispose();
    m_defaultFont.dispose();

    m_instance= NULL;
}

//-- private methods -----
bool AssetManager::loadTexture(const char *filename, TextureAsset *textureAsset)
{
    bool success= false;

    int pixelWidth=0, pixelHeight=0, channelCount=0;
    stbi_uc *image_buffer= stbi_load(filename, &pixelWidth, &pixelHeight, &channelCount, 3);

    if (image_buffer != NULL)
    {
//        GLint glPixelFormat= -1;

        if (channelCount == 3)
        {
            success = textureAsset->init(pixelWidth, pixelHeight, GL_RGB, GL_RGB, image_buffer);
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

bool AssetManager::loadFont(const char *filename, const float pixelHeight, FontAsset *fontAsset)
{
    unsigned char *temp_ttf_buffer = NULL;

    bool success= true;

    // Scratch buffer for true type font data loaded from file
    temp_ttf_buffer = NULL;

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
    if (success && !fontAsset->init(temp_ttf_buffer, pixelHeight))
    {
        Log_ERROR("AssetManager::loadFont", "Failed to fit font(%s) into %dx%d sprite texture", 
            filename, k_font_texture_width, k_font_texture_height);
        success = false;
    }
    
    // Free true type font scratch buffers
    if (temp_ttf_buffer != NULL)
    {
        delete[] temp_ttf_buffer;
    }

    return success;
}

//-- Font Asset -----
bool TextureAsset::init(
    unsigned int width,
    unsigned int height,
    unsigned int texture_format,
    unsigned int buffer_format,
    unsigned char *buffer)
{
    bool success = false;

    if (width > 0 && height > 0 && texture_format > 0 && buffer_format > 0)
    {
        this->texture_width = width;
        this->texture_height = height;
        this->texture_format = texture_format;
        this->buffer_format = buffer_format;

        // Setup the OpenGL texture to render the video frame into
        glGenTextures(1, &texture_id);
        glBindTexture(GL_TEXTURE_2D, texture_id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            texture_format,
            width,
            height,
            0,
            buffer_format,
            GL_UNSIGNED_BYTE,
            buffer);
        glBindTexture(GL_TEXTURE_2D, 0);

        success = true;
    }

    return success;
}

void TextureAsset::copyBufferIntoTexture(const unsigned char *pixels)
{
    if (texture_id != 0)
    {
        glPixelStorei(GL_UNPACK_SWAP_BYTES, GL_FALSE);
        glPixelStorei(GL_UNPACK_LSB_FIRST, GL_TRUE);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
        glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        glBindTexture(GL_TEXTURE_2D, texture_id);
        glTexSubImage2D(
            GL_TEXTURE_2D,
            0,
            0,
            0,
            texture_width,
            texture_height,
            buffer_format,
            GL_UNSIGNED_BYTE,
            pixels);
        glBindTexture(GL_TEXTURE_2D, 0);
    }
}

void TextureAsset::dispose()
{
    // Free the OpenGL video texture
    if (texture_id != 0)
    {
        glDeleteTextures(1, &texture_id);
        texture_id = 0;
        texture_width = 0;
        texture_height = 0;
        texture_format = 0;
        buffer_format = 0;
    }
}

//-- Font Asset -----
bool FontAsset::init(
    unsigned char *ttf_buffer,
    float pixel_height)
{
    bool success = false;

    // Temp buffer to bake the font texture into
    unsigned char *texture_buffer = new unsigned char[k_font_texture_width*k_font_texture_height];

    glyphPixelHeight = pixel_height;

    // Generate the texture for the font sprite sheet
    if (stbtt_BakeFontBitmap(
            ttf_buffer, 0,
            pixel_height,
            texture_buffer, k_font_texture_width, k_font_texture_height,
            32, 96, cdata) > 0)
    {
        // Load the texture into video memory
        success = TextureAsset::init(k_font_texture_width, k_font_texture_height, GL_ALPHA, GL_ALPHA, texture_buffer);
    }

    // Free the font texture buffer
    delete[] texture_buffer;

    return success;
}

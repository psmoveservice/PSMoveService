#ifndef ASSET_MANAGER_H
#define ASSET_MANAGER_H

#include "stb_truetype.h"

class TextureAsset
{
public:
    unsigned int texture_id;
    unsigned int texture_width;
    unsigned int texture_height;
    unsigned int texture_format;
    unsigned int buffer_format;

    TextureAsset()
        : texture_id(0)
        , texture_width(0)
        , texture_height(0)
        , texture_format(0)
        , buffer_format(0)
    {}
    ~TextureAsset()
    { dispose(); }

    bool init(unsigned int width, unsigned int height, unsigned int texture_format, unsigned int buffer_format, unsigned char *buffer);
    void copyBufferIntoTexture(const unsigned char *pixels);
    void dispose();
};

class FontAsset : public TextureAsset
{
public:
    float glyphPixelHeight;
    stbtt_bakedchar cdata[96]; // ASCII 32..126 is 95 glyphs

    FontAsset()
        : TextureAsset()
        , glyphPixelHeight(0.f)
    {}

    bool init(unsigned char *ttf_buffer, float pixel_height);
};

class AssetManager
{
public:
    AssetManager();
    ~AssetManager();

    bool init();
    void destroy();

    static AssetManager *getInstance()
    { return m_instance; }

    const TextureAsset *getPS3EyeTextureAsset()
    { return &m_ps3eyeTexture; }

    const TextureAsset *getPSMoveTextureAsset()
    { return &m_psmoveTexture; }

    const TextureAsset *getPSDualShock4TextureAsset()
    { return &m_psdualshock4Texture; }

    const TextureAsset *getVirtualControllerTextureAsset()
    { return &m_virtualTexture; }

    const TextureAsset *getPSNaviTextureAsset()
    { return &m_psnaviTexture; }

    const TextureAsset *getMorpheusTextureAsset()
    { return &m_morpheusTexture; }    

    const TextureAsset *getDK2TextureAsset()
    { return &m_dk2Texture; }    
    
    const FontAsset *getDefaultFont()
    { return &m_defaultFont; }

private:
    bool loadTexture(const char *filename, TextureAsset *textureAsset);
    bool loadFont(const char *filename, float pixelHeight, FontAsset *fontAsset);

    // Utility Textures
	TextureAsset m_ps3eyeTexture;
    TextureAsset m_psmoveTexture;
    TextureAsset m_psnaviTexture;
    TextureAsset m_psdualshock4Texture;
    TextureAsset m_virtualTexture;
    TextureAsset m_morpheusTexture;
    TextureAsset m_dk2Texture;

    // Font Rendering
    FontAsset m_defaultFont;

    static AssetManager *m_instance;
};

#endif // ASSET_MANAGER_H
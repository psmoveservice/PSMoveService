#ifndef ASSET_MANAGER_H
#define ASSET_MANAGER_H

#include "stb_truetype.h"

class AssetManager
{
public:
    struct FontAsset
    {
        unsigned int textureId;
        int textureWidth, textureHeight;
        float glyphPixelHeight;
        stbtt_bakedchar cdata[96]; // ASCII 32..126 is 95 glyphs

        FontAsset();
    };

    AssetManager();
    ~AssetManager();

    bool init();
    void destroy();

    static AssetManager *getInstance()
    { return m_instance; }

    unsigned int getDK2TextureId()
    { return m_dk2TextureId; }

    unsigned int getPSMoveTextureId()
    { return m_psmoveTextureId; }

    unsigned int getPSNaviTextureId()
    { return m_psnaviTextureId; }

    const FontAsset *getDefaultFont()
    { return &m_defaultFont; }

private:
    bool loadTexture(const char *filename, unsigned int *textureId);
    bool loadFont(const char *filename, float pixelHeight, AssetManager::FontAsset *fontAsset);

    // Utility Textures
    unsigned int m_dk2TextureId;
    unsigned int m_psmoveTextureId;
    unsigned int m_psnaviTextureId;

    // Font Rendering
    FontAsset m_defaultFont;

    static AssetManager *m_instance;
};

#endif // ASSET_MANAGER_H
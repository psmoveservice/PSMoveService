#ifndef ASSET_MANAGER_H
#define ASSET_MANAGER_H

class AssetManager
{
public:
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

private:
    bool loadTexture(const char *filename, unsigned int *textureId);

    // Utility Textures
    unsigned int m_dk2TextureId;
    unsigned int m_psmoveTextureId;
    unsigned int m_psnaviTextureId;

    static AssetManager *m_instance;
};

#endif // ASSET_MANAGER_H
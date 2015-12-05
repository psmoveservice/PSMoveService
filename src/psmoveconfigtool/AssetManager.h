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

private:
    bool loadTexture(const char *filename, unsigned int *textureId);

    // Utility Textures
    unsigned int m_dk2TextureId;

    static AssetManager *m_instance;
};

#endif // ASSET_MANAGER_H
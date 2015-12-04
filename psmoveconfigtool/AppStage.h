#ifndef APP_STAGE_H
#define APP_STAGE_H

class AppStage
{
public:
    AppStage(class App *app) 
        : m_app(app)
    { }

    virtual bool init(int argc, char** argv) { return true; }
    virtual void destroy() {}

    virtual void enter() {}
    virtual void exit() {}
    virtual void update() {}
    virtual void render() = 0;
    virtual void renderUI() {}

    virtual void onKeyDown(int keyCode) {}

protected:
    class App *m_app;
};

#endif // APP_STAGE_H
#ifndef CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_H

//-- includes -----
#include <memory>

//-- constants -----
static const int k_max_controllers= 5;

//-- typedefs -----
class ServerControllerView;
typedef std::shared_ptr<ServerControllerView> ServerControllerViewPtr;

//-- definitions -----
class ControllerManager 
{
public:
    ControllerManager();
    virtual ~ControllerManager();

    bool startup();
    void update();
    void shutdown();

    ServerControllerViewPtr getController(int controller_id);

    bool setControllerRumble(int controller_id, int rumble_amount);
    bool resetPose(int controller_id);    

private:
    // private implementation - same lifetime as the ControllerManager
    class ControllerManagerImpl *m_implementation_ptr;
};

#endif  // CONTROLLER_MANAGER_H

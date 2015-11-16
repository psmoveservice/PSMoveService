#ifndef CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_H

//-- includes -----

//-- constants -----
static const size_t k_max_psmove_controllers= 5;

//-- definitions -----
class ControllerManager 
{
public:
    ControllerManager();
    virtual ~ControllerManager();

    bool startup();
    void update();
    void shutdown();

    bool setControllerRumble(int psmove_id, int rumble_amount);
    bool resetPose(int psmove_id);

private:
    // private implementation - same lifetime as the ControllerManager
    class ControllerManagerImpl *m_implementation_ptr;
};

#endif  // CONTROLLER_MANAGER_H

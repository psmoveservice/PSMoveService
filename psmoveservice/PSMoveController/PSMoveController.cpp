#include "PSMoveController.h"

#define PSMOVE_VID 0x054c
#define PSMOVE_PID 0x03d5

PSMoveController::PSMoveController()
{
    // TODO: enumerate devices
    // TODO: get index of first free device
    // TODO: cleanup
    
    // TODO: call constructor with index
}

PSMoveController::PSMoveController(int index)
{
    // Mac and Linux: hid_init()
    // Windows:
}

PSMoveController::~PSMoveController()
{
    // TODO: Disconnect
    // Mac and Linux: hid_exit();
    // TODO: Free memory
}

psmovePosef
PSMoveController::getPose(int msec_time)
{
    psmovePosef nullPose;
    return nullPose;
}

int
PSMoveController::getButtonState()
{
    return 0;
}

void
PSMoveController::setRumbleValue(int value)
{

}

PSMoveControllerHIDAPI::PSMoveControllerHIDAPI(int index)
{
    if ((serial == NULL) && (!device_path.empty()))
    {
        const char * path = device_path.c_str();
        hid_open_path(path);
    }
    else
    {
        hid_open(PSMOVE_VID, PSMOVE_PID, serial);
    }
}

void
PSMoveControllerHIDAPI::enumerateControllers()
{
    struct hid_device_info *devs, *cur_dev;
    devs = hid_enumerate(PSMOVE_VID, PSMOVE_PID);
}
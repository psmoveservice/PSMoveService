#include "PSMoveDataFrame.h"
#include <string>
#include "hidapi.h"

// TODO: Make class virtual
class PSMoveController {
public:
    PSMoveController();
    PSMoveController(int index);
    ~PSMoveController();
    psmovePosef getPose(int msec_time);             // Used for prediction
    psmovePosef getPose() { return getPose(0); }    // Latest but no prediction
    int getButtonState();                           // TODO: A real 'state'
    void setRumbleValue(int value);
    void enumerateControllers();                    // TODO: Virtual function. Static. pass-by-ref.
    PSMoveDataFrame dataFrame;
};

class PSMoveControllerHIDAPI: public PSMoveController{
public:
    PSMoveControllerHIDAPI(int index);
    void enumerateControllers();
    std::string device_path;
    hid_device *handle;
    wchar_t *serial;
};
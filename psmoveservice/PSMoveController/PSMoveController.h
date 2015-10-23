#include "PSMoveDataFrame.h"
#include <string>
#include "hidapi.h"

struct PSMoveHIDDetails {
    std::string device_path;
    std::string device_path_addr; // only needed by Win > 8.1, otherwise ignored.
    hid_device *handle;
    hid_device *handle_addr; // only needed by Win > 8.1, otherwise ignored.
    std::string bt_addr;
};

class PSMoveController {
public:
    PSMoveController(const int next_ith = 1);       // next_ith beyond s_nOpened
    ~PSMoveController();
    
    psmovePosef getPose(int msec_time = 0);         // getPose msec_time in the future
    int getButtonState();                           // TODO: A real 'state', not int
    void setRumbleValue(unsigned char value);
	bool isOpen();
    
    PSMoveDataFrame dataFrame;
    static int s_nOpened;                           // Total number of opened controllers
    
private:
    int getBTAddress(std::string& host, std::string& controller);
    void writeDataOut();
    
    PSMoveHIDDetails HIDDetails;
    bool isBluetooth;
    int index;
	unsigned char ledr, ledg, ledb; //0x00..0xff
	unsigned char rumble; //0x00..0xff
};
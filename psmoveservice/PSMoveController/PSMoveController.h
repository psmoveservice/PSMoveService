#include "PSMoveDataFrame.h"
#include <string>
#include "hidapi.h"

typedef unsigned char PSMove_Data_BTAddr[6];

struct PSMoveHIDDetails {
    std::string device_path;
    std::string device_path_addr; // only needed by Win > 8.1, otherwise ignored.
    hid_device *handle;
    hid_device *handle_addr; // only needed by Win > 8.1, otherwise ignored.
    std::wstring bt_addr;
};

class PSMoveController {
public:
    PSMoveController(const int next_ith = 1);
    ~PSMoveController();
    
    psmovePosef getPose(int msec_time);             // Used for prediction
    psmovePosef getPose() { return getPose(0); }    // Latest but no prediction
    int getButtonState();                           // TODO: A real 'state'
    void setRumbleValue(unsigned char value);
	bool isOpen();
    
    PSMoveDataFrame dataFrame;
    static int s_nOpened;
    
private:
    int getBTAddress(PSMove_Data_BTAddr *host, PSMove_Data_BTAddr *controller);
    void writeDataOut();
    
    PSMoveHIDDetails HIDDetails;
    bool isBluetooth;
    int index;
	unsigned char ledr, ledg, ledb; //0x00..0xff
	unsigned char rumble; //0x00..0xff
};
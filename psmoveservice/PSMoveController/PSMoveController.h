#include "PSMoveDataFrame.h"
#include <string>
#include "hidapi.h"

struct PSMoveSensorTwoFrame {
	int oldFrame[3]; // TODO: vec3
	int newFrame[3]; // TODO: vec3
};

struct PSMoveState {
	// For each button,
	// ==32 if currently down,
	// ==64 if previously down,
	// ==96 if currently AND previously down
	unsigned int Triangle;
	unsigned int Circle;
	unsigned int Cross;
	unsigned int Square;
	unsigned int Select;
	unsigned int Start;
	unsigned int PS;
	unsigned int Move;

	unsigned char Trigger;  // 0-255. Average of last two frames.

	PSMoveSensorTwoFrame accel;
	PSMoveSensorTwoFrame gyro;
	int mag[3]; // TODO: vec3
	//TODO: oldTimestamp
	//TODO: newTimestamp
};

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
	bool isOpen();
    
	// TODO: Getters
	psmovePosef getPose(int msec_time = 0);         // getPose msec_time in the future
	PSMoveState getState();							//

	// Setters
	bool setLED(unsigned char r, unsigned char g, unsigned char b); // 0x00..0xff
    bool setRumbleIntensity(unsigned char value);	// 0x00..0xff
	bool setLEDPWMFrequency(unsigned long freq);	// 733..24e6
    
    PSMoveDataFrame dataFrame;
    static int s_nOpened;                           // Total number of opened controllers
    bool isBluetooth;
    
private:
	bool readDataIn();
    int getBTAddress(std::string& host, std::string& controller);
    bool writeDataOut();							// Setters will call this
    
    PSMoveHIDDetails HIDDetails;
    int index;
	unsigned char ledr, ledg, ledb; 
	unsigned char rumble;
	unsigned long ledpwmf;
	unsigned int lastButtons;
	PSMoveState lastState;
};
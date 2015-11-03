#include "PSMoveDataFrame.h"
#include "PSMoveConfig.h"
#include "hidapi.h"
#include <string>
#include <vector>
#include <deque>

enum PSMoveButtonState {
    Button_UP = 0x01,       // Not pressed
    Button_PRESSED = 0x01,  // Down for one frame only
    Button_DOWN = 0x11,     // Down for >1 frame
    Button_RELEASED = 0x10, // Up for one frame only
};

enum PSMoveBatteryLevel {
    Batt_MIN = 0x00, /*!< Battery is almost empty (< 20%) */
    Batt_20Percent = 0x01, /*!< Battery has at least 20% remaining */
    Batt_40Percent = 0x02, /*!< Battery has at least 40% remaining */
    Batt_60Percent = 0x03, /*!< Battery has at least 60% remaining */
    Batt_80Percent = 0x04, /*!< Battery has at least 80% remaining */
    Batt_MAX = 0x05, /*!< Battery is fully charged (not on charger) */
    Batt_CHARGING = 0xEE, /*!< Battery is currently being charged */
    Batt_CHARGING_DONE = 0xEF, /*!< Battery is fully charged (on charger) */
};

struct PSMoveState {
	PSMoveButtonState Triangle;
	PSMoveButtonState Circle;
	PSMoveButtonState Cross;
	PSMoveButtonState Square;
	PSMoveButtonState Select;
	PSMoveButtonState Start;
	PSMoveButtonState PS;
	PSMoveButtonState Move;

	unsigned char Trigger;  // 0-255. Average of last two frames.

    std::vector< std::vector<float> > Accel;    // Two frames of 3 dimensions
    std::vector< std::vector<float> > Gyro;     // Two frames of 3 dimensions
    std::vector<int> Mag;                       // One frame of 3 dimensions

    int Sequence;                               // 4-bit (1..16).
                                                // Sometimes frames are dropped.
    enum PSMoveBatteryLevel Battery;
    unsigned int TimeStamp;                     // 16-bit (time since ?, units?)
                                                // About 1150 between in-order frames.
    int TempRaw;
    unsigned int AllButtons;                       // all-buttons, used to detect changes
    
    //TODO: high-precision timestamp. Need to do in hidapi?
};

struct PSMoveHIDDetails {
    std::string Device_path;
    hid_device *Handle;
    std::string Device_path_addr; // only needed by Win > 8.1, otherwise ignored.
    hid_device *Handle_addr; // only needed by Win > 8.1, otherwise ignored.
    std::string Bt_addr;
};

struct PSMoveDataInput;  // See .cpp for full declaration

struct PSMoveKB {
	float k = 1.0f;	// Slope
	float b = 0.0f;	// Offset
};

struct PSMoveCalib
{
	PSMoveKB X;
	PSMoveKB Y;
	PSMoveKB Z;
};

class PSMoveControllerConfig : public PSMoveConfig
{
public:
	PSMoveControllerConfig(const std::string &fnamebase = "PSMoveControllerConfig")
		: PSMoveConfig(fnamebase){}

	struct PSMoveControllerCalib
	{
		PSMoveCalib Accel;
		PSMoveCalib Gyro;
	} Calibration;

	virtual const boost::property_tree::ptree config2ptree();
	virtual void ptree2config(const boost::property_tree::ptree &pt);
};

class PSMoveController {
public:
    PSMoveController(const int next_ith = 1);       // next_ith beyond s_nOpened
    ~PSMoveController();
    bool isOpen();                                  // returns true if hidapi opened successfully
    
	// Getters
	psmovePosef getPose(int msec_time = 0);         // TODO: Move this to a TrackerAndSensorFusion class
	const PSMoveState getState(int lookBack = 0);
    float getTempCelsius();

	// Setters
	bool setLED(unsigned char r, unsigned char g, unsigned char b); // 0x00..0xff. TODO: vec3
    bool setRumbleIntensity(unsigned char value);	// 0x00..0xff
	bool setLEDPWMFrequency(unsigned long freq);	// 733..24e6
    
    static int s_nOpened;                           // Total number of opened controllers
    bool IsBluetooth;                               // true if valid serial number on device opening
	PSMoveControllerConfig cfg;
    
private:
    
    bool readDataIn();                              // Called by Getters
    int getBTAddress(std::string& host, std::string& controller);
    void loadCalibration();                         // Use USB or file if on BT
    
    bool writeDataOut();							// Setters will call this
    
    PSMoveHIDDetails HIDDetails;
    int Index;
	unsigned char LedR, LedG, LedB;
	unsigned char Rumble;
	unsigned long LedPWMF;
    std::deque<PSMoveState> ControllerStates;
    PSMoveDataInput* InData;                      // Buffer to copy hidapi reports into
    PSMoveDataFrame DataFrame;                      // TODO: Move this to a TrackerAndSensorFusion class
};
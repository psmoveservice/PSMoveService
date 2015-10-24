#include "PSMoveDataFrame.h"
#include <string>
#include "hidapi.h"

/*! Battery charge level.
 * Charge level of the battery. Charging is indicated when the controller is
 * connected via USB, or when the controller is sitting in the charging dock.
 * In all other cases (Bluetooth, not in charging dock), the charge level is
 * indicated.
 *
 * Used by psmove_get_battery().
 **/
enum PSMove_Battery_Level {
    Batt_MIN = 0x00, /*!< Battery is almost empty (< 20%) */
    Batt_20Percent = 0x01, /*!< Battery has at least 20% remaining */
    Batt_40Percent = 0x02, /*!< Battery has at least 40% remaining */
    Batt_60Percent = 0x03, /*!< Battery has at least 60% remaining */
    Batt_80Percent = 0x04, /*!< Battery has at least 80% remaining */
    Batt_MAX = 0x05, /*!< Battery is fully charged (not on charger) */
    Batt_CHARGING = 0xEE, /*!< Battery is currently being charged */
    Batt_CHARGING_DONE = 0xEF, /*!< Battery is fully charged (on charger) */
};

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
    unsigned int TimeStamp;

	int Sequence;
    
    enum PSMove_Battery_Level Battery;
};

struct PSMoveHIDDetails {
    std::string device_path;
    std::string device_path_addr; // only needed by Win > 8.1, otherwise ignored.
    hid_device *handle;
    hid_device *handle_addr; // only needed by Win > 8.1, otherwise ignored.
    std::string bt_addr;
};

struct PSMove_Data_Input;  // Forward-declare so it can be referenced in a member variable.

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
    PSMove_Data_Input* inData;
};
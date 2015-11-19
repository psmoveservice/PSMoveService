#ifndef PSMOVE_CONTROLLER_H
#define PSMOVE_CONTROLLER_H

#include "PSMoveDataFrame.h"
#include "PSMoveConfig.h"
#include "hidapi.h"
#include <string>
#include <vector>
#include <deque>

enum PSMoveButtonState {
    Button_UP = 0x00,       // (00b) Not pressed
    Button_PRESSED = 0x01,  // (01b) Down for one frame only
    Button_DOWN = 0x03,     // (11b) Down for >1 frame
    Button_RELEASED = 0x02, // (10b) Up for one frame only
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
    
    PSMoveState()
    {
        clear();
    }

    void clear()
    {
        Triangle = Button_UP;
        Circle = Button_UP;
        Cross = Button_UP;
        Square = Button_UP;
        Select = Button_UP;
        Start = Button_UP;
        PS = Button_UP;
        Move = Button_UP;

        Trigger= 0;

        Accel = { {0, 0, 0}, {0, 0, 0} };
        Gyro = { {0, 0, 0}, {0, 0, 0} };
        Mag = {0, 0, 0};

        Sequence= 0;
        Battery= Batt_MAX;
        TimeStamp= 0;
        TempRaw= 0;
        AllButtons= 0;
    }
};

struct PSMoveHIDDetails {
    std::string Device_path;
    hid_device *Handle;
    std::string Device_path_addr; // only needed by Win > 8.1, otherwise ignored.
    hid_device *Handle_addr; // only needed by Win > 8.1, otherwise ignored.
    std::string Bt_addr;
};

struct PSMoveDataInput;  // See .cpp for full declaration

class PSMoveControllerConfig : public PSMoveConfig
{
public:
    PSMoveControllerConfig(const std::string &fnamebase = "PSMoveControllerConfig")
        : PSMoveConfig(fnamebase),
        cal_ag_xyz_kb(2, std::vector<std::vector<float>>(3, std::vector<float>(2, 0.f)))
    {};

    virtual const boost::property_tree::ptree config2ptree();
    virtual void ptree2config(const boost::property_tree::ptree &pt);
    std::vector<std::vector<std::vector<float>>> cal_ag_xyz_kb;
};

class PSMoveDeviceEnumerator
{
public:
    PSMoveDeviceEnumerator();
    ~PSMoveDeviceEnumerator();

    bool is_valid() const;
    bool next();

    const char *get_path() const;
    const wchar_t *get_serial_number() const;

private:
    struct hid_device_info *devs, *cur_dev;
};

class PSMoveController {
public:
    enum eReadDataResult
    {
        _ReadDataResultSuccessNoData,
        _ReadDataResultSuccessNewData,
        _ReadDataResultFailure,
    };

    PSMoveController(const int psmove_id);
    ~PSMoveController();

    bool open();                                             // Opens the first HID device for the controller
    bool open(const PSMoveDeviceEnumerator &enumerator);     // Opens the HID device for the controller at the given enumerator
    eReadDataResult readDataIn();                            // Polls for new controller data
    void close();                                            // Closes the HID device for the controller
    
    // Getters
    inline static int getOpenedControllerCount() 
    { return s_nOpened; }

    inline const PSMoveControllerConfig &getConfig() const
    { return cfg; }

    inline int getPSMoveID() const 
    { return PSMove_ID; }

    inline bool getIsBluetooth() const
    { return IsBluetooth; }

    bool getIsOpen() const;                            // Returns true if hidapi opened successfully
    psmovePosef getPose(int msec_time = 0) const;      // TODO: Move this to a TrackerAndSensorFusion class
    float getTempCelsius() const;
    const PSMoveState getState(int lookBack = 0) const;
    
    bool matchesDeviceEnumerator(const PSMoveDeviceEnumerator &enumerator) const; // return true if device path matches

    // Setters
    inline void setPSMoveID(int id)
    { PSMove_ID= id; }

    bool setLED(unsigned char r, unsigned char g, unsigned char b); // 0x00..0xff. TODO: vec3
    bool setRumbleIntensity(unsigned char value);   // 0x00..0xff
    bool setLEDPWMFrequency(unsigned long freq);    // 733..24e6
        
private:    
    bool getBTAddress(std::string& host, std::string& controller);
    void loadCalibration();                         // Use USB or file if on BT
    
    bool writeDataOut();                            // Setters will call this
    
    // Constant while a controller is open
    PSMoveControllerConfig cfg;
    PSMoveHIDDetails HIDDetails;
    int PSMove_ID;
    bool IsBluetooth;                               // true if valid serial number on device opening

    // Cached Setter State
    unsigned char LedR, LedG, LedB;
    unsigned char Rumble;
    unsigned long LedPWMF;

    // Read Controller State
    std::deque<PSMoveState> ControllerStates;
    PSMoveDataInput* InData;                        // Buffer to copy hidapi reports into

    // Derived Controller Data
    PSMoveDataFrame DataFrame;                      // TODO: Move this to a TrackerAndSensorFusion class

    // Data shared amongst all controllers
    static int s_nOpened;                           // Total number of opened controllers
};
#endif // PSMOVE_CONTROLLER_H
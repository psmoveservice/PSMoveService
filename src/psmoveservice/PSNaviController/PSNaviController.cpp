//-- includes -----
#include "PSNaviController.h"
#include "ControllerDeviceEnumerator.h"
#include "ServerLog.h"
#include "ServerUtility.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <vector>
#include <cstdlib>
#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif
#include <math.h>

//-- constants -----
//###bwalker $TODO This haven't been tested yet
#define PSNAVI_BTADDR_GET_SIZE 16
#define PSNAVI_BTADDR_SIZE 6
#define PSNAVI_BTADDR_SET_SIZE 7
#define PSNAVI_STATE_BUFFER_MAX 16

//###bwalker $TODO This haven't been tested yet
// https://code.google.com/p/moveonpc/wiki/HIDReports
enum PSNaviRequestType {
    PSNavi_Req_GetInput = 0x01,
    PSNavi_Req_GetBTAddr = 0xF2,
    PSNavi_Req_SetBTAddr = 0xF5,
};

enum PSNaviButton {
    // https://code.google.com/p/moveonpc/wiki/NavigationInputReport

    // buttons 2
    Btn_CROSS = 1 << 0,   // Blue cross
    Btn_CIRCLE = 1 << 2,	// Red circle
    Btn_L1 = 1 << 5,
    Btn_L2 = 1 << 6,

    // Buttons 1
    Btn_L3 = 1 << 9,
    Btn_UP = 1 << 12,
    Btn_RIGHT = 1 << 13,
    Btn_DOWN = 1 << 14,
    Btn_LEFT = 1 << 15,

    // Buttons 3
    Btn_PS = 1 << 16,		// PS button, front center
};

// -- private definitions -----
struct PSNaviDataInput {
    unsigned char type; /* message type, must be PSNavi_Req_GetInput */
    unsigned char buttons1; // (L3, D-pad)
    unsigned char buttons2; // (X, Circle, L1, L2)
    unsigned char buttons3; // (PS Button)
    unsigned char stick_xaxis; // stick value; 0..255 (subtract 0x80 to get signed values)
    unsigned char stick_yaxis; // stick value; 0..255 (subtract 0x80 to get signed values)
    unsigned char analog_dpad_up; // 0x00 (not pressed) to 0xff (fully pressed)
    unsigned char analog_dpad_right; // 0x00 (not pressed) to 0xff (fully pressed)
    unsigned char analog_dpad_down; // 0x00 (not pressed) to 0xff (fully pressed)
    unsigned char analog_dpad_left; // 0x00 (not pressed) to 0xff (fully pressed)
    unsigned char analog_l2; // 0x00 (not pressed) to 0xff (fully pressed)
    unsigned char analog_l1; // 0x00 (not pressed) to 0xff (fully pressed)
    unsigned char analog_circle; // 0x00 (not pressed) to 0xff (fully pressed)
    unsigned char analog_cross; // 0x00 (not pressed) to 0xff (fully pressed)
    unsigned char battery; // 0x05 = fully charged, 0xEE = charging or 0xEF = fully charged
};

// -- private prototypes -----
static std::string btAddrUcharToString(const unsigned char* addr_buff);
static bool stringToBTAddrUchar(const std::string &addr, unsigned char *addr_buff, const int addr_buf_size);
inline enum CommonControllerState::ButtonState getButtonState(unsigned int buttons, unsigned int lastButtons, int buttonMask);
inline bool hid_error_mbs(hid_device *dev, char *out_mb_error, size_t mb_buffer_size);

// -- public methods

// -- PSMove Controller Config
const boost::property_tree::ptree
PSNaviControllerConfig::config2ptree()
{
    boost::property_tree::ptree pt;

    pt.put("max_poll_failute_count", max_poll_failure_count);

    return pt;
}

void
PSNaviControllerConfig::ptree2config(const boost::property_tree::ptree &pt)
{
    max_poll_failure_count = pt.get<long>("max_poll_failute_count", 100);
}

// -- PSMove Controller -----
PSNaviController::PSNaviController()
{
    HIDDetails.Handle = nullptr;
    HIDDetails.Handle_addr = nullptr;
    
    NextPollSequenceNumber= 0;
    InData = new PSNaviDataInput;
    InData->type = PSNavi_Req_GetInput;
}

PSNaviController::~PSNaviController()
{
    if (getIsOpen())
    {
        SERVER_LOG_ERROR("~PSNaviController") << "Controller deleted without calling close() first!";
    }

    delete InData;
}

bool PSNaviController::open()
{
    ControllerDeviceEnumerator enumerator(CommonControllerState::PSNavi);
    bool success= false;

    if (enumerator.is_valid())
    {
        success= open(&enumerator);
    }

    return success;
}

bool PSNaviController::open(
    const DeviceEnumerator *enumerator)
{
    const ControllerDeviceEnumerator *pEnum = static_cast<const ControllerDeviceEnumerator *>(enumerator);
    
    const char *cur_dev_path= pEnum->get_path();
    bool success= false;

    if (getIsOpen())
    {
        SERVER_LOG_WARNING("PSNaviController::open") << "PSNavoController(" << cur_dev_path << ") already open. Ignoring request.";
        success= true;
    }
    else
    {
        char cur_dev_serial_number[256];

        SERVER_LOG_INFO("PSNaviController::open") << "Opening PSNaviController(" << cur_dev_path << ")";

        if (pEnum->get_serial_number(cur_dev_serial_number, sizeof(cur_dev_serial_number)))
        {
            SERVER_LOG_INFO("PSNaviController::open") << "  with serial_number: " << cur_dev_serial_number;
        }
        else
        {
            cur_dev_serial_number[0]= '\0';
            SERVER_LOG_INFO("PSNaviController::open") << "  with EMPTY serial_number";
        }

        HIDDetails.Device_path = cur_dev_path;
    #ifdef _WIN32
        HIDDetails.Device_path_addr = HIDDetails.Device_path;
        HIDDetails.Device_path_addr.replace(HIDDetails.Device_path_addr.find("&col01#"), 7, "&col02#");
        HIDDetails.Device_path_addr.replace(HIDDetails.Device_path_addr.find("&0000#"), 6, "&0001#");
        HIDDetails.Handle_addr = hid_open_path(HIDDetails.Device_path_addr.c_str());
        hid_set_nonblocking(HIDDetails.Handle_addr, 1);
    #endif
        HIDDetails.Handle = hid_open_path(HIDDetails.Device_path.c_str());
        hid_set_nonblocking(HIDDetails.Handle, 1);
                
        IsBluetooth = (strlen(cur_dev_serial_number) > 0);

        if (getIsOpen())  // Controller was opened and has an index
        {
            // Get the bluetooth address
    #ifndef _WIN32
            // On my Mac, getting the bt feature report when connected via
            // bt crashes the controller. So we simply copy the serial number.
            // It gets modified in getBTAddress.
            // TODO: Copy this over anyway even in Windows. Check getBTAddress
            // comments for handling windows serial_number.
            // Once done, we can remove the ifndef above.
            std::string mbs(cur_dev_serial_number);
            HIDDetails.Bt_addr = mbs;
    #endif
            if (getBTAddress(HIDDetails.Host_bt_addr, HIDDetails.Bt_addr))
            {
                // Load the config file
                std::string btaddr = HIDDetails.Bt_addr;
                std::replace(btaddr.begin(), btaddr.end(), ':', '_');
                cfg = PSNaviControllerConfig(btaddr);
                cfg.load();

                // TODO: Other startup.

                success= true;
            }
            else
            {
                // If serial is still bad, maybe we have a disconnected
                // controller still showing up in hidapi
                SERVER_LOG_ERROR("PSNaviController::open") << "Failed to get bluetooth address of PSNaviController(" << cur_dev_path << ")";
                success= false;
            }

            // Reset the polling sequence counter
            NextPollSequenceNumber= 0;
        }
        else
        {
            SERVER_LOG_ERROR("PSNaviController::open") << "Failed to open PSNaviController(" << cur_dev_path << ")";
            success= false;
        }
    }

    return success;
}

void PSNaviController::close()
{
    if (getIsOpen())
    {
        SERVER_LOG_INFO("PSNaviController::close") << "Closing PSNaviController(" << HIDDetails.Device_path << ")";

        if (HIDDetails.Handle != nullptr)
        {
            hid_close(HIDDetails.Handle);
            HIDDetails.Handle= nullptr;
        }

        if (HIDDetails.Handle_addr != nullptr)
        {
            hid_close(HIDDetails.Handle_addr);
            HIDDetails.Handle_addr= nullptr;
        }
    }
    else
    {
        SERVER_LOG_INFO("PSNaviController::close") << "PSNaviController(" << HIDDetails.Device_path << ") already closed. Ignoring request.";
    }
}

bool 
PSNaviController::setHostBluetoothAddress(const std::string &new_host_bt_addr)
{
    bool success= false;
    unsigned char bts[PSNAVI_BTADDR_SET_SIZE];

    memset(bts, 0, sizeof(bts));
    bts[0] = PSNavi_Req_SetBTAddr;
    bts[1] = 0x01;
    bts[2] = 0x00;

    unsigned char addr[6];
    if (stringToBTAddrUchar(new_host_bt_addr, addr, sizeof(addr)))
    {
        int res;

        /* Copy 6 bytes from addr into bts[3]..bts[8] */
        memcpy(&bts[3], addr, sizeof(addr));

        /* _WIN32 only has move->handle_addr for getting bluetooth address. */
        if (HIDDetails.Handle_addr) 
        {
            res = hid_send_feature_report(HIDDetails.Handle_addr, bts, sizeof(bts));
        } 
        else 
        {
            res = hid_send_feature_report(HIDDetails.Handle, bts, sizeof(bts));
        }

        if (res == sizeof(bts))
        {
            success= true;
        }
        else
        {
            char hidapi_err_mbs[256];
            bool valid_error_mesg= false;
            
            if (HIDDetails.Handle_addr)
            {
                valid_error_mesg = hid_error_mbs(HIDDetails.Handle_addr, hidapi_err_mbs, sizeof(hidapi_err_mbs));
            }
            else
            {
                valid_error_mesg = hid_error_mbs(HIDDetails.Handle, hidapi_err_mbs, sizeof(hidapi_err_mbs));
            }

            if (valid_error_mesg)
            {
                SERVER_LOG_ERROR("PSNaviController::setBTAddress") << "HID ERROR: " << hidapi_err_mbs;
            }            
        }
    }
    else
    {
        SERVER_LOG_ERROR("PSNaviController::setBTAddress") << "Malformed address: " << new_host_bt_addr;
    }

    return success;
}


// Getters
bool 
PSNaviController::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    const ControllerDeviceEnumerator *pEnum = static_cast<const ControllerDeviceEnumerator *>(enumerator);
    bool matches= false;

    if (pEnum->get_device_type() == CommonControllerState::PSNavi)
    {
        const char *enumerator_path= pEnum->get_path();
        const char *dev_path= HIDDetails.Device_path.c_str();

    #ifdef _WIN32
        matches= _stricmp(dev_path, enumerator_path) == 0;
    #else
        matches= strcmp(dev_path, enumerator_path) == 0;
    #endif
    }

    return matches;
}

bool 
PSNaviController::getIsBluetooth() const
{ 
    return IsBluetooth; 
}

bool
PSNaviController::getIsReadyToPoll() const
{
    return (getIsOpen() && getIsBluetooth());
}

std::string 
PSNaviController::getUSBDevicePath() const
{
    return HIDDetails.Device_path;
}

std::string 
PSNaviController::getSerial() const
{
    return HIDDetails.Bt_addr;
}

std::string 
PSNaviController::getAssignedHostBluetoothAddress() const
{
    return HIDDetails.Host_bt_addr;
}

bool
PSNaviController::getIsOpen() const
{
    return (HIDDetails.Handle != nullptr);
}

CommonDeviceState::eDeviceType
PSNaviController::getDeviceType() const
{
    return CommonControllerState::PSNavi;
}

bool
PSNaviController::getBTAddress(std::string& host, std::string& controller)
{
    bool success = false;

    if (IsBluetooth && !controller.empty())
    {
        std::replace(controller.begin(), controller.end(), '-', ':');
        std::transform(controller.begin(), controller.end(), controller.begin(), ::tolower);
        
        //TODO: If the third entry is not : and length is PSNAVI_BTADDR_SIZE
//        std::stringstream ss;
//        ss << controller.substr(0, 2) << ":" << controller.substr(2, 2) <<
//        ":" << controller.substr(4, 2) << ":" << controller.substr(6, 2) <<
//        ":" << controller.substr(8, 2) << ":" << controller.substr(10, 2);
//        controller = ss.str();
        
        success = true;
    }
    else
    {
        int res;
        
        unsigned char btg[PSNAVI_BTADDR_GET_SIZE];
        unsigned char ctrl_char_buff[PSNAVI_BTADDR_SIZE];

        memset(btg, 0, sizeof(btg));
        btg[0] = PSNavi_Req_GetBTAddr;
        /* _WIN32 only has move->handle_addr for getting bluetooth address. */
        if (HIDDetails.Handle_addr) {
            res = hid_get_feature_report(HIDDetails.Handle_addr, btg, sizeof(btg));
        }
        else {
            res = hid_get_feature_report(HIDDetails.Handle, btg, sizeof(btg));
        }

        if (res == sizeof(btg)) {

            memcpy(ctrl_char_buff, btg + 2, PSNAVI_BTADDR_SIZE);
            controller = btAddrUcharToString(ctrl_char_buff);

            success = true;
        }
        else
        {
            char hidapi_err_mbs[256];
            bool valid_error_mesg= false;
            
            if (HIDDetails.Handle_addr)
            {
                valid_error_mesg = hid_error_mbs(HIDDetails.Handle_addr, hidapi_err_mbs, sizeof(hidapi_err_mbs));
            }
            else
            {
                valid_error_mesg = hid_error_mbs(HIDDetails.Handle, hidapi_err_mbs, sizeof(hidapi_err_mbs));
            }

            if (valid_error_mesg)
            {
                SERVER_LOG_ERROR("PSNaviController::getBTAddress") << "HID ERROR: " << hidapi_err_mbs;
            }
        }
    }

    return success;
}

IControllerInterface::ePollResult
PSNaviController::poll()
{
    IControllerInterface::ePollResult result= IControllerInterface::_PollResultFailure;
      
    if (!getIsBluetooth())
    {
        // Don't bother polling when connected via usb
        result = IControllerInterface::_PollResultSuccessNoData;
    }
    else if (getIsOpen())
    {
        static const int k_max_iterations= 32;        

        for (int iteration= 0; iteration < k_max_iterations; ++iteration)
        {
            // Attempt to read the next update packet from the controller
            int res = hid_read(HIDDetails.Handle, (unsigned char*)InData, sizeof(PSNaviDataInput));

            if (res == 0)
            {
                // Device still in valid state
                result= (iteration == 0) 
                    ? IControllerInterface::_PollResultSuccessNoData 
                    : IControllerInterface::_PollResultSuccessNewData;
                
                // No more data available. Stop iterating.
                break;
            }
            else if (res < 0)
            {
                char hidapi_err_mbs[256];
                bool valid_error_mesg = hid_error_mbs(HIDDetails.Handle, hidapi_err_mbs, sizeof(hidapi_err_mbs));

                // Device no longer in valid state.
                if (valid_error_mesg)
                {
                    SERVER_LOG_ERROR("PSMoveController::readDataIn") << "HID ERROR: " << hidapi_err_mbs;
                }
                result= IControllerInterface::_PollResultFailure;

                // No more data available. Stop iterating.
                break;
            }
            else
            {
                // New data available. Keep iterating.
                result= IControllerInterface::_PollResultFailure;
            }
        
            // https://github.com/nitsch/moveonpc/wiki/Input-report
            PSNaviControllerState newState;
        
            // Increment the sequence for every new polling packet
            newState.PollSequenceNumber= NextPollSequenceNumber;
            ++NextPollSequenceNumber;

            // Buttons
            newState.AllButtons = 
                (InData->buttons2) |               // |-|L2|L1|-|-|Circle|-|Cross
                (InData->buttons1 << 8) |          // |Left|Down|Right|Up|-|-|L3|-
                ((InData->buttons3 & 0x01) << 16); // |-|-|-|-|-|-|-|PS
        
            unsigned int lastButtons = ControllerStates.empty() ? 0 : ControllerStates.back().AllButtons;

            newState.Circle = getButtonState(newState.AllButtons, lastButtons, Btn_CIRCLE);
            newState.Cross = getButtonState(newState.AllButtons, lastButtons, Btn_CROSS);
            newState.PS = getButtonState(newState.AllButtons, lastButtons, Btn_PS);
            newState.Trigger = InData->analog_l2;
            newState.Stick_XAxis= InData->stick_xaxis;
            newState.Stick_YAxis= InData->stick_yaxis;
                           
            // Other
            newState.Battery = static_cast<CommonControllerState::BatteryLevel>(InData->battery);

            // Make room for new entry if at the max queue size
            if (ControllerStates.size() >= PSNAVI_STATE_BUFFER_MAX)
            {
                ControllerStates.erase(ControllerStates.begin(),
                                        ControllerStates.begin()+ControllerStates.size()-PSNAVI_STATE_BUFFER_MAX);
            }

            ControllerStates.push_back(newState);
        }
    }

    return result;
}

const CommonDeviceState *
PSNaviController::getState(
    int lookBack) const
{
    const int queueSize= static_cast<int>(ControllerStates.size());
    const CommonDeviceState * result=
        (lookBack < queueSize) ? &ControllerStates.at(queueSize - lookBack - 1) : nullptr;

    return result;
}

long 
PSNaviController::getMaxPollFailureCount() const
{
    return cfg.max_poll_failure_count;
}

const std::tuple<unsigned char, unsigned char, unsigned char> 
PSNaviController::getColour() const
{
    // Navi doesn't have an LED
    return std::make_tuple(0, 0, 0);
}

void 
PSNaviController::getTrackingShape(CommonDeviceTrackingShape &outTrackingShape) const
{
    // Navi isn't a tracked controller
    outTrackingShape.shape_type= eCommonTrackingShapeType::INVALID_SHAPE;
}
    
// -- private helper functions -----
static std::string
btAddrUcharToString(const unsigned char* addr_buff)
{
    // http://stackoverflow.com/questions/11181251/saving-hex-values-to-a-c-string
    // NOTE: unlike with the Move controller, the address is not stored backwards in the report
    std::ostringstream stream;
    for (int buff_ind = 0; buff_ind < 6; ++buff_ind)
    {
        stream << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(addr_buff[buff_ind]);
        if (buff_ind > 0)
        {
            stream << ":";
        }
    }
    return stream.str();
}

static bool
stringToBTAddrUchar(const std::string &addr, unsigned char *addr_buff, const int addr_buf_size)
{
    bool success= false;

    if (addr.length() >= 17 && addr_buf_size >= 6)
    {
        const char *raw_string= addr.c_str();
        int octets[6];

        success= 
            sscanf(raw_string, "%x:%x:%x:%x:%x:%x", 
                &octets[0],
                &octets[1],
                &octets[2],
                &octets[3],
                &octets[4],
                &octets[5]) == 6;
        //TODO: Make sscanf safe. (sscanf_s is not portable)

        if (success)
        {
            for (int i= 0; i < 6; ++i)
            {
                addr_buff[i]= ServerUtility::int32_to_int8_verify(octets[i]);
            }
        }
    }

    return success;
}

inline enum CommonControllerState::ButtonState
getButtonState(unsigned int buttons, unsigned int lastButtons, int buttonMask)
{
    return (enum CommonControllerState::ButtonState)((((lastButtons & buttonMask) > 0) << 1) + ((buttons & buttonMask)>0));
}

inline bool hid_error_mbs(hid_device *dev, char *out_mb_error, size_t mb_buffer_size)
{
    return ServerUtility::convert_wcs_to_mbs(hid_error(dev), out_mb_error, mb_buffer_size);
}
//-- includes -----
#include "PSNaviController.h"
#include "ControllerDeviceEnumerator.h"
#include "ControllerUSBDeviceEnumerator.h"
#include "ControllerGamepadEnumerator.h"
#include "ServerLog.h"
#include "ServerUtility.h"
#include "USBDeviceManager.h"

#include "gamepad/Gamepad.h"

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

#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': sscanf
#endif

// NOTE: Details concerning low level communication with the Navi came from the SCP library 
// Forum Post: http://forums.pcsx2.net/Thread-XInput-Wrapper-for-DS3-and-Play-com-USB-Dual-DS2-Controller
// source: http://forums.pcsx2.net/attachment.php?aid=49785

//-- constants -----
#define CONTROL_TRANSFER_TIMEOUT	  500 /* timeout in ms */
#define INTERRUPT_TRANSFER_TIMEOUT	  500 /* timeout in ms */

#define PSNAVI_VENDOR_ID 0x054c
#define PSNAVI_PRODUCT_ID 0x042F

#define PSNAVI_CONFIGURATION_VALUE  1
#define PSNAVI_ENDPOINT_IN 0x80

#define PSNAVI_HID_INTERFACE 0

#define PSNAVI_USB_INTERFACES_MASK_TO_CLAIM ( \
	(1 << PSNAVI_HID_INTERFACE) \
)

#define PSNAVI_CNTLR_BTADDR_BUF_SIZE 17
#define PSNAVI_HOST_BTADDR_BUF_SIZE 9
#define PSNAVI_BTADDR_SIZE 6
#define PSNAVI_STATE_BUFFER_MAX 16

// https://github.com/nitsch/moveonpc/wiki/HID-reports
enum PSNaviRequestType {
    PSNavi_Req_GetInput = 0x01,
    PSNavi_Req_GetControllerBTAddr = 0xF2,
	PSNavi_Req_GetHostBTAddr = 0xF5,

	PSNavi_Req_SetInputStreamEnabled = 0xF4,
	PSNavi_Req_SetHostBTAddr = 0xF5,
};

enum PSNaviButton {
    // https://github.com/nitsch/moveonpc/wiki/Navigation-Input-report

	// Buttons 1
	Btn_L3 = 1 << 1,
	Btn_UP = 1 << 4,
	Btn_RIGHT = 1 << 5,
	Btn_DOWN = 1 << 6,
	Btn_LEFT = 1 << 7,

    // buttons 2
	Btn_L2 = 1 << 8,
	Btn_L1 = 1 << 10,
	Btn_CIRCLE = 1 << 13,	// Red circle
	Btn_CROSS = 1 << 14,   // Blue cross

    // Buttons 3
    Btn_PS = 1 << 16,		// PS button, front center
};

// -- private definitions -----
class PSNaviAPIContext
{
public:
	int vendor_id;
	int product_id;
	std::string device_identifier;
	std::string host_bluetooth_address;
	std::string controller_bluetooth_address;

	// LibUSB state
	std::string usb_device_path;
	t_usb_device_handle usb_device_handle;

	// Gamepad state
	std::string gamepad_device_path;
	int gamepad_index;

	PSNaviAPIContext()
	{
		Reset();
	}

	void Reset()
	{
		vendor_id= -1;
		product_id= -1;
		device_identifier = "";
		host_bluetooth_address = "";
		controller_bluetooth_address = "";
		usb_device_path = "";
		usb_device_handle = k_invalid_usb_device_handle;
		gamepad_device_path = "";
		gamepad_index = -1;
	}
};

struct PSNaviDataInput {
    unsigned char type; /* message type, must be PSNavi_Req_GetInput */
	unsigned char _unk0[1];
    unsigned char buttons1; // (L3, D-pad)
    unsigned char buttons2; // (X, Circle, L1, L2)
    unsigned char buttons3; // (PS Button)
	unsigned char _unk1[1];
    unsigned char stick_xaxis; // stick value; 0..255 (subtract 0x80 to get signed values)
    unsigned char stick_yaxis; // stick value; 0..255 (subtract 0x80 to get signed values)
	unsigned char _unk2[6];
    unsigned char analog_dpad_up; // 0x00 (not pressed) to 0xff (fully pressed)
    unsigned char analog_dpad_right; // 0x00 (not pressed) to 0xff (fully pressed)
    unsigned char analog_dpad_down; // 0x00 (not pressed) to 0xff (fully pressed)
    unsigned char analog_dpad_left; // 0x00 (not pressed) to 0xff (fully pressed)
    unsigned char analog_l2; // 0x00 (not pressed) to 0xff (fully pressed)
	unsigned char _unk3[1];
    unsigned char analog_l1; // 0x00 (not pressed) to 0xff (fully pressed)
	unsigned char _unk4[2];
    unsigned char analog_circle; // 0x00 (not pressed) to 0xff (fully pressed)
    unsigned char analog_cross; // 0x00 (not pressed) to 0xff (fully pressed)
	unsigned char _unk[5];
    unsigned char battery; // 0x05 = fully charged, 0xEE = charging or 0xEF = fully charged
};

// -- private prototypes -----
static std::string NaviBTAddrUcharToString(const unsigned char* addr_buff);
static bool stringToNaviBTAddrUchar(const std::string &addr, unsigned char *addr_buff, const int addr_buf_size);
inline void setButtonBit(unsigned int buttons, unsigned int button_bit, bool is_pressed);
inline enum CommonControllerState::ButtonState getButtonState(unsigned int buttons, unsigned int lastButtons, int buttonMask);

static int psnavi_get_usb_feature_report(t_usb_device_handle device_handle, unsigned char *report_bytes, size_t report_size);
static int psnavi_send_usb_feature_report(t_usb_device_handle device_handle, unsigned char *report_bytes, size_t report_size);
static int psnavi_read_usb_interrupt_pipe(t_usb_device_handle device_handle, unsigned char *buffer, size_t max_buffer_size);

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
	APIContext = new PSNaviAPIContext();

    // Make sure there is an initial empty state in the tracker queue
    {     
        PSNaviControllerState empty_state;

        empty_state.clear();
        ControllerStates.push_back(empty_state);
    }

    NextPollSequenceNumber= 0;
}

PSNaviController::~PSNaviController()
{
    if (getIsOpen())
    {
        SERVER_LOG_ERROR("~PSNaviController") << "Controller deleted without calling close() first!";
    }

	delete APIContext;
}

bool PSNaviController::open()
{
    ControllerDeviceEnumerator enumerator(ControllerDeviceEnumerator::CommunicationType_USB, CommonControllerState::PSNavi);
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
        SERVER_LOG_WARNING("PSNaviController::open") << "PSNaviController(" << cur_dev_path << ") already open. Ignoring request.";
        success= true;
    }
    else
    {
        SERVER_LOG_INFO("PSNaviController::open") << "Opening PSNaviController(" << cur_dev_path << ")";

		APIContext->vendor_id = pEnum->get_vendor_id();
		APIContext->product_id = pEnum->get_product_id();

		if (pEnum->get_api_type() == ControllerDeviceEnumerator::CommunicationType_USB)
		{
			const ControllerUSBDeviceEnumerator *usbControllerEnum= pEnum->get_usb_controller_enumerator();
			assert(usbControllerEnum != nullptr);
			const t_usb_device_handle usb_device_handle = usb_device_open(usbControllerEnum->get_usb_device_enumerator());

			if (usb_device_handle != k_invalid_usb_device_handle)
			{
				SERVER_LOG_INFO("PSNaviController::open") << "  Successfully opened USB handle " << usb_device_handle;
				APIContext->usb_device_path = cur_dev_path;
				APIContext->usb_device_handle = usb_device_handle;
			}
			else
			{
				SERVER_LOG_ERROR("PSNaviController::open") << "  Failed to open USB handle " << usb_device_handle;
			}
		}
		else if (pEnum->get_api_type() == ControllerDeviceEnumerator::CommunicationType_GAMEPAD)
		{
			const ControllerGamepadEnumerator *gamepadControllerEnum = pEnum->get_gamepad_controller_enumerator();
			assert(gamepadControllerEnum != nullptr);
			const int gamepad_index = gamepadControllerEnum->get_contoller_index();

			if (gamepad_index != -1)
			{
				const Gamepad_device * gamepad = Gamepad_deviceAtIndex(static_cast<unsigned int>(gamepad_index));

				char device_path[255];
				ServerUtility::format_string(device_path, sizeof(device_path), "%s #%d", gamepad->description, gamepad_index);

				SERVER_LOG_INFO("PSNaviController::open") << "  Successfully opened gamepad: " << device_path;
				APIContext->gamepad_index = gamepad_index;
				APIContext->gamepad_device_path = device_path;

				// Sadly this info available to us through the gamepad api
				APIContext->host_bluetooth_address= "00:00:00:00:00:00";
				APIContext->controller_bluetooth_address= "00:00:00:00:00:00";
			}
			else
			{
				SERVER_LOG_ERROR("PSNaviController::open") << "  Failed to open gamepad";
			}
		}

        if (getIsOpen())  // Controller was opened and has an index
        {
			if (pEnum->get_api_type() == ControllerDeviceEnumerator::CommunicationType_USB)
			{
				// Get the bluetooth address
				if (getHostBTAddressOverUSB(APIContext->host_bluetooth_address) &&
					getControllerBTAddressOverUSB(APIContext->controller_bluetooth_address))
				{
					// Build a unique name for the config file using bluetooth address of the controller
					char szConfigSuffix[18];
					ServerUtility::bluetooth_cstr_address_normalize(
						APIContext->controller_bluetooth_address.c_str(), true, '_',
						szConfigSuffix, sizeof(szConfigSuffix));

					std::string config_name("psnavi_");
					config_name += szConfigSuffix;

					// Load the config file
					cfg = PSNaviControllerConfig(config_name);
					cfg.load();

					// Save it back out again in case any defaults changed
					cfg.save();

					// Tell the controller to start sending input data packets
					setInputStreamEnabledOverUSB();

					success = true;
				}
				else
				{
					// If serial is still bad, maybe we have a disconnected
					// controller still showing up in hidapi
					SERVER_LOG_ERROR("PSNaviController::open") << "Failed to get bluetooth address of PSNaviController(" << cur_dev_path << ")";
					success = false;
				}
			}
			else if (pEnum->get_api_type() == ControllerDeviceEnumerator::CommunicationType_GAMEPAD)
			{
				char config_name[255];
				ServerUtility::format_string(config_name, sizeof(config_name), "psnavi_gamepad_%d", APIContext->gamepad_index);

				// Load the config file
				cfg = PSNaviControllerConfig(std::string(config_name));
				cfg.load();

				// Save it back out again in case any defaults changed
				cfg.save();

				success = true;
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
		if (APIContext->usb_device_handle != k_invalid_usb_device_handle)
		{
			USBDeviceManager *usbMgr = USBDeviceManager::getInstance();

			SERVER_LOG_INFO("PSNaviController::close") << "Closing PSNaviController(" << APIContext->usb_device_path << ")";
			usb_device_close(APIContext->usb_device_handle);
			APIContext->usb_device_handle = k_invalid_usb_device_handle;
		}
		else if (APIContext->gamepad_index != -1)
		{
			SERVER_LOG_INFO("PSNaviController::close") << "Closing PSNaviController(" << APIContext->gamepad_index << ")";
			APIContext->gamepad_index = -1;
		}

		APIContext->Reset();
    }
    else
    {
        SERVER_LOG_INFO("PSNaviController::close") << "PSNaviController(" << APIContext->usb_device_path << ") already closed. Ignoring request.";
    }
}

bool
PSNaviController::setTrackingColorID(const eCommonTrackingColorID tracking_color_id)
{
	return false;
}

bool 
PSNaviController::setHostBluetoothAddress(const std::string &new_host_bt_addr)
{
    bool success= false;

	if (APIContext->usb_device_handle != k_invalid_usb_device_handle)
	{
		unsigned char bts[PSNAVI_HOST_BTADDR_BUF_SIZE];

		memset(bts, 0, sizeof(bts));
		bts[0] = PSNavi_Req_SetHostBTAddr;
		bts[1] = 0x00;
		bts[2] = 0x00;

		unsigned char addr[6];
		if (stringToNaviBTAddrUchar(new_host_bt_addr, addr, sizeof(addr)))
		{
			int res;

			/* Copy 6 bytes from addr into bts[3]..bts[8] */
			memcpy(&bts[3], addr, sizeof(addr));

			res = psnavi_send_usb_feature_report(APIContext->usb_device_handle, bts, sizeof(bts));
			if (res == sizeof(bts) )
			{
				success= true;
			}
		}
		else
		{
			SERVER_LOG_ERROR("PSNaviController::setBTAddress") << "Malformed address: " << new_host_bt_addr;
		}
	}
	else
	{
		SERVER_LOG_ERROR("PSNaviController::setBTAddress") << "Can't set bluetooth address using gampad api";
	}

    return success;
}


// Getters
bool 
PSNaviController::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    bool matches= false;

    if (enumerator->get_device_type() == CommonControllerState::PSNavi)
    {
		const char *enumerator_path = enumerator->get_path();
		const char *dev_path = nullptr;

		if (APIContext->usb_device_handle != k_invalid_usb_device_handle)
		{
			dev_path = APIContext->usb_device_path.c_str();
		}
		else if (APIContext->gamepad_index != -1)
		{
			dev_path = APIContext->gamepad_device_path.c_str();
		}
		else
		{
			dev_path = "<INVALID>";
		}

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
	// Assumed bluetooth connection if the controller is connected through the gamepad interface
    return APIContext->gamepad_index != -1;
}

bool
PSNaviController::getIsReadyToPoll() const
{
    return getIsOpen();
}

std::string 
PSNaviController::getUSBDevicePath() const
{
    return APIContext->usb_device_path;
}

int
PSNaviController::getVendorID() const
{
	return APIContext->vendor_id;
}

int
PSNaviController::getProductID() const
{
	return APIContext->product_id;
}

std::string 
PSNaviController::getSerial() const
{
    return APIContext->controller_bluetooth_address;
}

std::string 
PSNaviController::getAssignedHostBluetoothAddress() const
{
    return APIContext->host_bluetooth_address;
}

bool
PSNaviController::getIsOpen() const
{
    return (APIContext->usb_device_handle != k_invalid_usb_device_handle || APIContext->gamepad_index != -1);
}

CommonDeviceState::eDeviceType
PSNaviController::getDeviceType() const
{
    return CommonControllerState::PSNavi;
}

bool
PSNaviController::setInputStreamEnabledOverUSB()
{ 
	bool bSuccess = false;

	unsigned char report_bytes[5];

	// Command used to enable the Dualshock 3 and Navigation controller to send data via USB
	report_bytes[0] = PSNavi_Req_SetInputStreamEnabled; // Report ID
	report_bytes[1] = 0x42; // Special PS3 Controller enable commands
	report_bytes[2] = 0x0c;
	report_bytes[3] = 0x00;
	report_bytes[4] = 0x00;

	int res = psnavi_send_usb_feature_report(APIContext->usb_device_handle, report_bytes, sizeof(report_bytes));
	if (res == sizeof(report_bytes))
	{
		bSuccess = true;
	}

	return bSuccess;
}

bool
PSNaviController::getHostBTAddressOverUSB(std::string& host)
{
    bool bSuccess = false;
        
    unsigned char btg[64];
    unsigned char ctrl_char_buff[PSNAVI_BTADDR_SIZE];

    memset(btg, 0, sizeof(btg));
    btg[0] = PSNavi_Req_GetHostBTAddr;

	int res = psnavi_get_usb_feature_report(APIContext->usb_device_handle, btg, sizeof(btg));
	if (res == sizeof(btg))
	{
		bSuccess = true;
	}

	if (bSuccess)
	{
		memcpy(ctrl_char_buff, btg + 2, PSNAVI_BTADDR_SIZE);
		host = NaviBTAddrUcharToString(ctrl_char_buff);
	}
	
    return bSuccess;
}

bool
PSNaviController::getControllerBTAddressOverUSB(std::string& controller)
{
	bool bSuccess = false;

	unsigned char btg[64];
	unsigned char ctrl_char_buff[PSNAVI_BTADDR_SIZE];

	memset(btg, 0, sizeof(btg));
	btg[0] = PSNavi_Req_GetControllerBTAddr;

	int res = psnavi_get_usb_feature_report(APIContext->usb_device_handle, btg, sizeof(btg));
	if (res == sizeof(btg))
	{
		bSuccess = true;
	}

	if (bSuccess)
	{
		memcpy(ctrl_char_buff, btg + 4, PSNAVI_BTADDR_SIZE);
		controller = NaviBTAddrUcharToString(ctrl_char_buff);
	}

	return bSuccess;
}

IControllerInterface::ePollResult
PSNaviController::poll()
{
    IControllerInterface::ePollResult result= IControllerInterface::_PollResultFailure;
      
    if (getIsOpen())
    {
		if (APIContext->usb_device_handle != k_invalid_usb_device_handle)
		{
			result = pollUSB();			
		}
		else if (APIContext->gamepad_index != -1)
		{
			// For the gamepad case, just consider the current state is new data
			result = pollGamepad();
		}
    }

    return result;
}

IControllerInterface::ePollResult
PSNaviController::pollGamepad()
{
	assert(getIsOpen());

	const Gamepad_device * gamepad = Gamepad_deviceAtIndex(static_cast<unsigned int>(APIContext->gamepad_index));
	PSNaviControllerState newState;

	// Increment the sequence for every new polling packet
	newState.PollSequenceNumber = NextPollSequenceNumber;
	++NextPollSequenceNumber;

	// New Button State
	bool bIsCrossPressed= gamepad->buttonStates[0];
	bool bIsCirclePressed= gamepad->buttonStates[1];
	bool bIsL1Pressed= gamepad->buttonStates[4];
	bool bIsL2Pressed= gamepad->axisStates[2] >= .9f;
	bool bIsL3Pressed= gamepad->buttonStates[8];
	bool bIsDPadLeftPressed= gamepad->axisStates[5] <= -0.99f;
	bool bIsDPadRightPressed= gamepad->axisStates[5] >= 0.99f;
	bool bIsDPadUpPressed= gamepad->axisStates[6] <= -0.99f;
	bool bIsDPadDownPressed= gamepad->axisStates[6] >= 0.99f;

	newState.AllButtons = 0;
	setButtonBit(newState.AllButtons, Btn_UP, bIsDPadUpPressed);
	setButtonBit(newState.AllButtons, Btn_DOWN, bIsDPadDownPressed);
	setButtonBit(newState.AllButtons, Btn_LEFT, bIsDPadLeftPressed);
	setButtonBit(newState.AllButtons, Btn_RIGHT, bIsDPadRightPressed);
	setButtonBit(newState.AllButtons, Btn_CROSS, bIsCrossPressed);
	setButtonBit(newState.AllButtons, Btn_CIRCLE, bIsCirclePressed);
	setButtonBit(newState.AllButtons, Btn_L1, bIsL1Pressed);
	setButtonBit(newState.AllButtons, Btn_L2, bIsL2Pressed);
	setButtonBit(newState.AllButtons, Btn_L3, bIsL3Pressed);

	// Button de-bounce
	unsigned int lastButtons = ControllerStates.empty() ? 0 : ControllerStates.back().AllButtons;
	newState.DPad_Up = getButtonState(newState.AllButtons, lastButtons, Btn_UP);
	newState.DPad_Down = getButtonState(newState.AllButtons, lastButtons, Btn_DOWN);
	newState.DPad_Left = getButtonState(newState.AllButtons, lastButtons, Btn_LEFT);
	newState.DPad_Right = getButtonState(newState.AllButtons, lastButtons, Btn_RIGHT);
	newState.Circle = getButtonState(newState.AllButtons, lastButtons, Btn_CIRCLE);
	newState.Cross = getButtonState(newState.AllButtons, lastButtons, Btn_CROSS);
	newState.PS = getButtonState(newState.AllButtons, lastButtons, Btn_PS);
	newState.L1 = getButtonState(newState.AllButtons, lastButtons, Btn_L1);
	newState.L2 = getButtonState(newState.AllButtons, lastButtons, Btn_L2);
	newState.L3 = getButtonState(newState.AllButtons, lastButtons, Btn_L3);

	// Analog triggers
	newState.Stick_XAxis = static_cast<unsigned char>((gamepad->axisStates[0] + 1.f) * 127.f);
	newState.Stick_YAxis = static_cast<unsigned char>((gamepad->axisStates[1] + 1.f) * 127.f);
	newState.Trigger = static_cast<unsigned char>(gamepad->axisStates[2] * 255.f);

	// Can't report the true battery state
	newState.Battery = CommonControllerState::Batt_MAX;

	// Make room for new entry if at the max queue size
	if (ControllerStates.size() >= PSNAVI_STATE_BUFFER_MAX)
	{
		ControllerStates.erase(ControllerStates.begin(),
			ControllerStates.begin() + ControllerStates.size() - PSNAVI_STATE_BUFFER_MAX);
	}

	ControllerStates.push_back(newState);

	return IControllerInterface::_PollResultSuccessNewData;
}

IControllerInterface::ePollResult
PSNaviController::pollUSB()
{
	assert(getIsOpen());
	assert(!getIsBluetooth());

	IControllerInterface::ePollResult poll_result;
	int res = psnavi_read_usb_interrupt_pipe(APIContext->usb_device_handle, InBuffer, sizeof(InBuffer));

	if (res < 0)
	{
		poll_result = IControllerInterface::_PollResultFailure;
	}
	else if (res == 0)
	{
		poll_result = IControllerInterface::_PollResultSuccessNoData;
	}
	else
	{
		parseInputData();
		poll_result = IControllerInterface::_PollResultSuccessNewData;
	}

	return poll_result;
}

void 
PSNaviController::parseInputData()
{
	PSNaviDataInput* InData = reinterpret_cast<PSNaviDataInput *>(InBuffer);

	// https://github.com/nitsch/moveonpc/wiki/Input-report
	PSNaviControllerState newState;

	// Increment the sequence for every new polling packet
	newState.PollSequenceNumber = NextPollSequenceNumber;
	++NextPollSequenceNumber;

	// Buttons
	newState.AllButtons =
		(InData->buttons1) |               // |Left|Down|Right|Up|-|-|L3|-
		(InData->buttons2 << 8) |          // |Cross|-|Circle|-|L1|-|-|L2|
		((InData->buttons3 & 0x01) << 16); // |-|-|-|-|-|-|-|PS|

	unsigned int lastButtons = ControllerStates.empty() ? 0 : ControllerStates.back().AllButtons;

	newState.DPad_Up = getButtonState(newState.AllButtons, lastButtons, Btn_UP);
	newState.DPad_Down = getButtonState(newState.AllButtons, lastButtons, Btn_DOWN);
	newState.DPad_Left = getButtonState(newState.AllButtons, lastButtons, Btn_LEFT);
	newState.DPad_Right = getButtonState(newState.AllButtons, lastButtons, Btn_RIGHT);
	newState.Circle = getButtonState(newState.AllButtons, lastButtons, Btn_CIRCLE);
	newState.Cross = getButtonState(newState.AllButtons, lastButtons, Btn_CROSS);
	newState.PS = getButtonState(newState.AllButtons, lastButtons, Btn_PS);
	newState.L1 = getButtonState(newState.AllButtons, lastButtons, Btn_L1);
	newState.L2 = getButtonState(newState.AllButtons, lastButtons, Btn_L2);
	newState.L3 = getButtonState(newState.AllButtons, lastButtons, Btn_L3);
	newState.Trigger = InData->analog_l2;
	newState.Stick_XAxis = InData->stick_xaxis;
	newState.Stick_YAxis = InData->stick_yaxis;

	// Other
	newState.Battery = static_cast<CommonControllerState::BatteryLevel>(InData->battery);

	// Make room for new entry if at the max queue size
	if (ControllerStates.size() >= PSNAVI_STATE_BUFFER_MAX)
	{
		ControllerStates.erase(ControllerStates.begin(),
			ControllerStates.begin() + ControllerStates.size() - PSNAVI_STATE_BUFFER_MAX);
	}

	ControllerStates.push_back(newState);
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

bool 
PSNaviController::getTrackingColorID(eCommonTrackingColorID &out_tracking_color_id) const
{
	out_tracking_color_id = eCommonTrackingColorID::INVALID_COLOR;
	return false;
}

float PSNaviController::getIdentityForwardDegrees() const
{
	// Controller model points down the -Z axis when it has the identity orientation
	return 270.f;
}

float PSNaviController::getPredictionTime() const
{
	return 0.f; // No state prediction on the psnavi
}
    
// -- private helper functions -----
static std::string
NaviBTAddrUcharToString(const unsigned char* addr_buff)
{
	// https://github.com/nitsch/moveonpc/wiki/HID-reports#readable-reports-1
    // NOTE: unlike with the Move controller, the address is not stored backwards in the report

	// http://stackoverflow.com/questions/11181251/saving-hex-values-to-a-c-string
    std::ostringstream stream;
    for (int buff_ind = 0; buff_ind < 6; ++buff_ind)
    {
        stream << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(addr_buff[buff_ind]);
        if (buff_ind < 5)
        {
            stream << ":";
        }
    }
    return stream.str();
}

static bool
stringToNaviBTAddrUchar(const std::string &addr, unsigned char *addr_buff, const int addr_buf_size)
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
			// https://github.com/nitsch/moveonpc/wiki/HID-reports#writable-reports-1
			// "NOTE: unlike with the Move controller, the address is not to be written backwards in the report"
            for (int i= 0; i < 6; ++i)
            {
                addr_buff[i]= ServerUtility::int32_to_int8_verify(octets[i]);
            }
        }
    }

    return success;
}

inline void 
setButtonBit(unsigned int buttons, unsigned int button_bit, bool is_pressed)
{
	if (is_pressed)
	{
		buttons|= (1 << button_bit);
	}
	else
	{
		buttons&= ~(1 << button_bit);
	}
}

inline enum CommonControllerState::ButtonState
getButtonState(unsigned int buttons, unsigned int lastButtons, int buttonMask)
{
    return (enum CommonControllerState::ButtonState)((((lastButtons & buttonMask) > 0) << 1) + ((buttons & buttonMask)>0));
}

static int 
psnavi_get_usb_feature_report(
	t_usb_device_handle device_handle,
	unsigned char *report_bytes,
	size_t report_size)
{
	USBDeviceManager *usbMgr= USBDeviceManager::getInstance();
	int result = 0;
	
	USBTransferRequest transfer_request;
	transfer_request.request_type = _USBRequestType_ControlTransfer;

	USBRequestPayload_ControlTransfer &control_transfer= transfer_request.payload.control_transfer;
	control_transfer.usb_device_handle = device_handle;
	control_transfer.bmRequestType = USB_CTRL_IN;
	control_transfer.bRequest = HID_GET_REPORT;
	control_transfer.wValue = (HID_REPORT_TYPE_FEATURE << 8) | report_bytes[0];
	control_transfer.wIndex = 0;
	control_transfer.wLength = static_cast<uint16_t>(report_size);
	control_transfer.timeout = CONTROL_TRANSFER_TIMEOUT;

	USBTransferResult transfer_result= usb_device_submit_transfer_request_blocking(transfer_request);
	assert(transfer_result.result_type == _USBResultType_ControlTransfer);

	if (transfer_result.payload.control_transfer.result_code == _USBResultCode_Completed)
	{
		size_t byte_count = std::min(static_cast<size_t>(transfer_result.payload.control_transfer.dataLength), report_size);
		
		if (byte_count > 0)
		{
			memcpy(report_bytes, transfer_result.payload.control_transfer.data, byte_count);
		}

		result = transfer_result.payload.control_transfer.dataLength;
	}
	else
	{
		const char * error_text = usb_device_get_error_string(transfer_result.payload.control_transfer.result_code);
		SERVER_LOG_ERROR("psnavi_get_usb_feature_report") << "Control transfer failed with error: " << error_text;
		result= -static_cast<int>(transfer_result.payload.control_transfer.result_code);
	}

	return result;
}

static int
psnavi_send_usb_feature_report(
	t_usb_device_handle device_handle,
	unsigned char *report_bytes,
	size_t report_size)
{
	USBDeviceManager *usbMgr = USBDeviceManager::getInstance();
	int result = 0;

	USBTransferRequest transfer_request;
	transfer_request.request_type = _USBRequestType_ControlTransfer;

	USBRequestPayload_ControlTransfer &control_transfer = transfer_request.payload.control_transfer;
	control_transfer.usb_device_handle = device_handle;
	control_transfer.bmRequestType = USB_CTRL_OUT;
	control_transfer.bRequest = HID_SET_REPORT;
	control_transfer.wValue = (HID_REPORT_TYPE_FEATURE << 8) | report_bytes[0];
	control_transfer.wIndex = 0;
	memcpy(control_transfer.data, report_bytes + 1, report_size - 1);
	control_transfer.wLength = static_cast<uint16_t>(report_size-1); // don't include the report id in the size
	control_transfer.timeout = CONTROL_TRANSFER_TIMEOUT;

	USBTransferResult transfer_result = usb_device_submit_transfer_request_blocking(transfer_request);
	assert(transfer_result.result_type == _USBResultType_ControlTransfer);

	if (transfer_result.payload.control_transfer.result_code == _USBResultCode_Completed)
	{
		result = transfer_result.payload.control_transfer.dataLength + 1;
	}
	else
	{
		const char * error_text = usb_device_get_error_string(transfer_result.payload.control_transfer.result_code);
		SERVER_LOG_ERROR("psnavi_send_usb_feature_report") << "Control transfer failed with error: " << error_text;
		result = -static_cast<int>(transfer_result.payload.control_transfer.result_code);
	}

	return result;
}

static int
psnavi_read_usb_interrupt_pipe(
	t_usb_device_handle device_handle,
	unsigned char *buffer,
	size_t max_buffer_size)
{
	USBDeviceManager *usbMgr = USBDeviceManager::getInstance();
	int result = 0;

	USBTransferRequest transfer_request;
	transfer_request.request_type = _USBRequestType_InterruptTransfer;

	USBRequestPayload_InterruptTransfer &interrupt_transfer = transfer_request.payload.interrupt_transfer;
	interrupt_transfer.usb_device_handle = device_handle;
	interrupt_transfer.endpoint = 0x81;
	interrupt_transfer.length = static_cast<unsigned int>(max_buffer_size);
	interrupt_transfer.timeout = INTERRUPT_TRANSFER_TIMEOUT;

	USBTransferResult transfer_result = usb_device_submit_transfer_request_blocking(transfer_request);
	assert(transfer_result.result_type == _USBResultType_InterrupTransfer);

	if (transfer_result.payload.interrupt_transfer.result_code == _USBResultCode_Completed)
	{
		size_t byte_count = std::min(static_cast<size_t>(transfer_result.payload.interrupt_transfer.dataLength), max_buffer_size);

		if (byte_count > 0)
		{
			memcpy(buffer, transfer_result.payload.interrupt_transfer.data, byte_count);
		}

		result = transfer_result.payload.interrupt_transfer.dataLength;
	}
	else
	{
		const char * error_text = usb_device_get_error_string(transfer_result.payload.interrupt_transfer.result_code);
		SERVER_LOG_ERROR("psnavi_read_usb_interrupt_pipe") << "interrupt transfer failed with error: " << error_text;
		result = -static_cast<int>(transfer_result.payload.interrupt_transfer.result_code);
	}

	return result;
}
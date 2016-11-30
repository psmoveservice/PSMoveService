//-- includes -----
#include "PSNaviController.h"
#include "ControllerDeviceEnumerator.h"
#include "ServerLog.h"
#include "ServerUtility.h"

#include "hidapi.h"
#include "libusb.h"

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
// HID Class-Specific Requests values. See section 7.2 of the HID specifications 
#define HID_GET_REPORT                0x01 
#define HID_GET_IDLE                  0x02 
#define HID_GET_PROTOCOL              0x03 
#define HID_SET_REPORT                0x09 
#define HID_SET_IDLE                  0x0A 
#define HID_SET_PROTOCOL              0x0B 
#define HID_REPORT_TYPE_INPUT         0x01 
#define HID_REPORT_TYPE_OUTPUT        0x02 
#define HID_REPORT_TYPE_FEATURE       0x03 

#define CTRL_IN        LIBUSB_ENDPOINT_IN|LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE 
#define CTRL_OUT    LIBUSB_ENDPOINT_OUT|LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE 

#define CONTROL_TRANSFER_TIMEOUT	  500 /* timeout in ms */

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
class PSNaviUSBContext
{
public:
	std::string device_identifier;
	std::string host_bluetooth_address;
	std::string controller_bluetooth_address;

	// HIDApi state
	std::string hid_device_path;
	hid_device *hid_device_handle;

	// LibUSB state
	libusb_context *usb_context;
	libusb_device_handle *usb_device_handle;
	libusb_config_descriptor *usb_device_descriptor;
	unsigned int usb_claimed_interface_mask;

	PSNaviUSBContext()
	{
		Reset();
	}

	void Reset()
	{
		device_identifier = "";
		host_bluetooth_address = "";
		controller_bluetooth_address = "";
		hid_device_path = "";
		hid_device_handle = nullptr;
		usb_context = nullptr;
		usb_device_handle = nullptr;
		usb_device_descriptor = nullptr;
		usb_claimed_interface_mask = 0;
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
inline enum CommonControllerState::ButtonState getButtonState(unsigned int buttons, unsigned int lastButtons, int buttonMask);

static int navi_hid_get_feature_report(hid_device *dev, unsigned char *data, size_t length);
inline bool hid_error_mbs(hid_device *dev, char *out_mb_error, size_t mb_buffer_size);

static bool psnavi_open_usb_device(PSNaviUSBContext *navi_context);
static void psnavi_close_usb_device(PSNaviUSBContext *navi_context);
static int psnavi_get_usb_feature_report(PSNaviUSBContext *navi_context, unsigned char *report_bytes, size_t report_size);
static int psnavi_send_usb_feature_report(PSNaviUSBContext *navi_context, unsigned char *report_bytes, size_t report_size);
static int psnavi_read_usb_interrupt_pipe(PSNaviUSBContext *navi_context, unsigned char *buffer, size_t max_buffer_size);

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
	USBContext = new PSNaviUSBContext();

    NextPollSequenceNumber= 0;
}

PSNaviController::~PSNaviController()
{
    if (getIsOpen())
    {
        SERVER_LOG_ERROR("~PSNaviController") << "Controller deleted without calling close() first!";
    }

	delete USBContext;
}

bool PSNaviController::open()
{
    ControllerDeviceEnumerator enumerator(ControllerDeviceEnumerator::CommunicationType_LIBUSB, CommonControllerState::PSNavi);
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
        char cur_dev_serial_number[256];

        SERVER_LOG_INFO("PSNaviController::open") << "Opening PSNaviController(" << cur_dev_path << ")";

		if (pEnum->get_api_type() == ControllerDeviceEnumerator::CommunicationType_HID)
		{
			if (pEnum->get_serial_number(cur_dev_serial_number, sizeof(cur_dev_serial_number)))
			{
				SERVER_LOG_INFO("PSNaviController::open") << "  with serial_number: " << cur_dev_serial_number;
			}
			else
			{
				cur_dev_serial_number[0] = '\0';
				SERVER_LOG_INFO("PSNaviController::open") << "  with EMPTY serial_number";
			}

			USBContext->hid_device_path = cur_dev_path;
			USBContext->hid_device_handle = hid_open_path(USBContext->hid_device_path.c_str());
			hid_set_nonblocking(USBContext->hid_device_handle, 1);
			IsBluetooth = strlen(cur_dev_serial_number) > 0;
		}
		else
		{
			IsBluetooth = false;
			psnavi_open_usb_device(USBContext);
		}

        if (getIsOpen())  // Controller was opened and has an index
        {
			// Get the bluetooth address
            if (getHostBTAddress(USBContext->host_bluetooth_address) &&
				getControllerBTAddress(USBContext->controller_bluetooth_address))
            {
				// Build a unique name for the config file using bluetooth address of the controller
				char szConfigSuffix[18];
				ServerUtility::bluetooth_cstr_address_normalize(
					USBContext->controller_bluetooth_address.c_str(), true, '_',
					szConfigSuffix, sizeof(szConfigSuffix));

				std::string config_name("psnavi_");
				config_name += szConfigSuffix;

				// Load the config file
				cfg = PSNaviControllerConfig(config_name);
				cfg.load();

				// Save it back out again in case any defaults changed
				cfg.save();

				// Tell the controller to start sending input data packets
				setInputStreamEnabled();

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
        SERVER_LOG_INFO("PSNaviController::close") << "Closing PSNaviController(" << USBContext->hid_device_path << ")";

        if (USBContext->hid_device_handle != nullptr)
        {
            hid_close(USBContext->hid_device_handle);
			USBContext->hid_device_handle= nullptr;
        }

		psnavi_close_usb_device(USBContext);

		USBContext->Reset();
    }
    else
    {
        SERVER_LOG_INFO("PSNaviController::close") << "PSNaviController(" << USBContext->hid_device_path << ") already closed. Ignoring request.";
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

        res = psnavi_send_usb_feature_report(USBContext, bts, sizeof(bts));
        if (res == sizeof(bts))
        {
            success= true;
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
        const char *dev_path= USBContext->hid_device_path.c_str();

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
    return USBContext->hid_device_path;
}

std::string 
PSNaviController::getSerial() const
{
    return USBContext->controller_bluetooth_address;
}

std::string 
PSNaviController::getAssignedHostBluetoothAddress() const
{
    return USBContext->host_bluetooth_address;
}

bool
PSNaviController::getIsOpen() const
{
    return (USBContext->hid_device_handle != nullptr) || (USBContext->usb_device_handle != nullptr);
}

CommonDeviceState::eDeviceType
PSNaviController::getDeviceType() const
{
    return CommonControllerState::PSNavi;
}

bool
PSNaviController::setInputStreamEnabled()
{ 
	bool bSuccess = false;

	if (getIsBluetooth())
	{
		unsigned char report_bytes[6];

		report_bytes[0] = 0x53; // HID BT Set_report (0x50) | Report Type (Feature 0x03)
		report_bytes[1] = PSNavi_Req_SetInputStreamEnabled; // Report ID
		report_bytes[2] = 0x42; // Special PS3 Controller enable commands
		report_bytes[3] = 0x03;
		report_bytes[4] = 0x00;
		report_bytes[5] = 0x00;

		int res = hid_send_feature_report(USBContext->hid_device_handle, report_bytes, sizeof(report_bytes));
		if (res == sizeof(report_bytes))
		{
			bSuccess = true;
		}
		else
		{
			char hidapi_err_mbs[256];

			if (hid_error_mbs(USBContext->hid_device_handle, hidapi_err_mbs, sizeof(hidapi_err_mbs)))
			{
				SERVER_LOG_ERROR("PSNaviController::sendDataStreamEnableCommand") << "HID ERROR: " << hidapi_err_mbs;
			}
		}
	}
	else
	{
		unsigned char report_bytes[5];

		// Command used to enable the Dualshock 3 and Navigation controller to send data via USB
		report_bytes[0] = PSNavi_Req_SetInputStreamEnabled; // Report ID
		report_bytes[1] = 0x42; // Special PS3 Controller enable commands
		report_bytes[2] = 0x0c;
		report_bytes[3] = 0x00;
		report_bytes[4] = 0x00;

		int res = psnavi_send_usb_feature_report(USBContext, report_bytes, sizeof(report_bytes));
		if (res == sizeof(report_bytes))
		{
			bSuccess = true;
		}
	}

	return bSuccess;
}

bool
PSNaviController::getHostBTAddress(std::string& host)
{
    bool bSuccess = false;
        
    unsigned char btg[64];
    unsigned char ctrl_char_buff[PSNAVI_BTADDR_SIZE];

    memset(btg, 0, sizeof(btg));
    btg[0] = PSNavi_Req_GetHostBTAddr;

	if (getIsBluetooth())
	{
		int res = hid_get_feature_report(USBContext->hid_device_handle, btg, sizeof(btg));
		if (res == sizeof(btg))
		{
			bSuccess = true;
		}
		else
		{
			char hidapi_err_mbs[256];

			if (hid_error_mbs(USBContext->hid_device_handle, hidapi_err_mbs, sizeof(hidapi_err_mbs)))
			{
				SERVER_LOG_ERROR("PSNaviController::getHostBTAddress") << "HID ERROR: " << hidapi_err_mbs;
			}
		}
	}
	else
	{
		int res = psnavi_get_usb_feature_report(USBContext, btg, sizeof(btg));
		if (res == sizeof(btg))
		{
			bSuccess = true;
		}
	}

	if (bSuccess)
	{
		memcpy(ctrl_char_buff, btg + 2, PSNAVI_BTADDR_SIZE);
		host = NaviBTAddrUcharToString(ctrl_char_buff);
	}
	
    return bSuccess;
}

bool
PSNaviController::getControllerBTAddress(std::string& controller)
{
	bool bSuccess = false;

	unsigned char btg[64];
	unsigned char ctrl_char_buff[PSNAVI_BTADDR_SIZE];

	memset(btg, 0, sizeof(btg));
	btg[0] = PSNavi_Req_GetControllerBTAddr;

	if (getIsBluetooth())
	{
		int res = hid_get_feature_report(USBContext->hid_device_handle, btg, sizeof(btg));
		if (res == sizeof(btg))
		{
			bSuccess = true;
		}
		else
		{
			char hidapi_err_mbs[256];

			if (hid_error_mbs(USBContext->hid_device_handle, hidapi_err_mbs, sizeof(hidapi_err_mbs)))
			{
				SERVER_LOG_ERROR("PSNaviController::getHostBTAddress") << "HID ERROR: " << hidapi_err_mbs;
			}
		}
	}
	else
	{
		int res = psnavi_get_usb_feature_report(USBContext, btg, sizeof(btg));
		if (res == sizeof(btg))
		{
			bSuccess = true;
		}
	}

	if (bSuccess)
	{
		memcpy(ctrl_char_buff, btg + 3, PSNAVI_BTADDR_SIZE);
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
		if (getIsBluetooth())
		{
			result = pollBluetooth();
		}
		else
		{
			result = pollUSB();
		}
    }

    return result;
}

IControllerInterface::ePollResult
PSNaviController::pollUSB()
{
	assert(getIsOpen());
	assert(!getIsBluetooth());

	IControllerInterface::ePollResult poll_result;
	int res = psnavi_read_usb_interrupt_pipe(USBContext, InBuffer, sizeof(InBuffer));

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

IControllerInterface::ePollResult
PSNaviController::pollBluetooth()
{
	IControllerInterface::ePollResult poll_result = IControllerInterface::_PollResultFailure;

	static const int k_max_iterations = 32;

	for (int iteration = 0; iteration < k_max_iterations; ++iteration)
	{
		// Attempt to read the next update packet from the controller
		InBuffer[0]= PSNavi_Req_GetInput;
		int res = hid_read(USBContext->hid_device_handle, InBuffer, sizeof(PSNaviDataInput));

		if (res == 0)
		{
			// Device still in valid state
			poll_result = (iteration == 0)
				? IControllerInterface::_PollResultSuccessNoData
				: IControllerInterface::_PollResultSuccessNewData;

			// No more data available. Stop iterating.
			break;
		}
		else if (res < 0)
		{
			char hidapi_err_mbs[256];
			bool valid_error_mesg = hid_error_mbs(USBContext->hid_device_handle, hidapi_err_mbs, sizeof(hidapi_err_mbs));

			// Device no longer in valid state.
			if (valid_error_mesg)
			{
				SERVER_LOG_ERROR("PSMoveController::readDataIn") << "HID ERROR: " << hidapi_err_mbs;
			}

			poll_result = IControllerInterface::_PollResultFailure;

			// No more data available. Stop iterating.
			break;
		}
		else
		{
			// New data available. Keep iterating.
			parseInputData();
			poll_result = IControllerInterface::_PollResultSuccessNewData;
		}
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
		(InData->buttons1) |          // |Left|Down|Right|Up|-|-|L3|-
		(InData->buttons2 << 8) |               // |Cross|-|Circle|-|L1|-|-|L2|
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

inline enum CommonControllerState::ButtonState
getButtonState(unsigned int buttons, unsigned int lastButtons, int buttonMask)
{
    return (enum CommonControllerState::ButtonState)((((lastButtons & buttonMask) > 0) << 1) + ((buttons & buttonMask)>0));
}

inline bool hid_error_mbs(hid_device *dev, char *out_mb_error, size_t mb_buffer_size)
{
    return ServerUtility::convert_wcs_to_mbs(hid_error(dev), out_mb_error, mb_buffer_size);
}

static bool 
psnavi_open_usb_device(PSNaviUSBContext *navi_context)
{
	bool bSuccess = true;
	if (libusb_init(&navi_context->usb_context) == LIBUSB_SUCCESS)
	{
		libusb_set_debug(navi_context->usb_context, 3);
	}
	else
	{
		SERVER_LOG_ERROR("psnavi_open_usb_device") << "libusb context initialization failed!";
		bSuccess = false;
	}

	if (bSuccess)
	{
		//###HipsterSloth $HACK This approach only works for one connected controller
		navi_context->usb_device_handle =
			libusb_open_device_with_vid_pid(
				navi_context->usb_context,
				PSNAVI_VENDOR_ID, PSNAVI_PRODUCT_ID);

		if (navi_context->usb_device_handle == nullptr)
		{
			SERVER_LOG_ERROR("psnavi_open_usb_device") << "PSNavi USB device not found!";
			bSuccess = false;
		}
	}

	if (bSuccess)
	{
		libusb_device *device = libusb_get_device(navi_context->usb_device_handle);
		int result = libusb_get_config_descriptor_by_value(
			device,
			PSNAVI_CONFIGURATION_VALUE,
			&navi_context->usb_device_descriptor);

		if (result != LIBUSB_SUCCESS)
		{
			SERVER_LOG_ERROR("psnavi_open_usb_device") << "Failed to retrieve Morpheus usb config descriptor";
			bSuccess = false;
		}
	}

	for (int interface_index = 0;
		bSuccess && interface_index < navi_context->usb_device_descriptor->bNumInterfaces;
		interface_index++)
	{
		int mask = 1 << interface_index;

		if (PSNAVI_USB_INTERFACES_MASK_TO_CLAIM & mask)
		{
			int result = 0;

#ifndef _WIN32
			result = libusb_kernel_driver_active(navi_context->usb_device_handle, interface_index);
			if (result < 0)
			{
				SERVER_LOG_ERROR("psnavi_open_usb_device") << "USB Interface #" << interface_index << " driver status failed";
				bSuccess = false;
			}

			if (bSuccess && result == 1)
			{
				SERVER_LOG_ERROR("morpeus_open_usb_device") << "Detach kernel driver on interface #" << interface_index;

				result = libusb_detach_kernel_driver(navi_context->usb_device_handle, interface_index);
				if (result != LIBUSB_SUCCESS)
				{
					SERVER_LOG_ERROR("psnavi_open_usb_device") << "Interface #" << interface_index << " detach failed";
					bSuccess = false;
				}
			}
#endif //_WIN32

			result = libusb_claim_interface(navi_context->usb_device_handle, interface_index);
			if (result == LIBUSB_SUCCESS)
			{
				navi_context->usb_claimed_interface_mask |= mask;
			}
			else
			{
				SERVER_LOG_ERROR("psnavi_open_usb_device") << "Interface #" << interface_index << " claim failed";
				bSuccess = false;
			}
		}
	}

	if (!bSuccess)
	{
		psnavi_close_usb_device(navi_context);
	}

	return bSuccess;
}

static void 
psnavi_close_usb_device(PSNaviUSBContext *navi_context)
{
	for (int interface_index = 0; navi_context->usb_claimed_interface_mask != 0; ++interface_index)
	{
		int interface_mask = 1 << interface_index;

		if ((navi_context->usb_claimed_interface_mask & interface_mask) != 0)
		{
			libusb_release_interface(navi_context->usb_device_handle, interface_index);
			navi_context->usb_claimed_interface_mask &= ~interface_mask;
		}
	}

	if (navi_context->usb_device_descriptor != nullptr)
	{
		libusb_free_config_descriptor(navi_context->usb_device_descriptor);
		navi_context->usb_device_descriptor = nullptr;
	}

	if (navi_context->usb_device_handle != nullptr)
	{
		libusb_close(navi_context->usb_device_handle);
		navi_context->usb_device_handle = nullptr;
	}

	if (navi_context->usb_context != nullptr)
	{
		libusb_exit(navi_context->usb_context);
		navi_context->usb_context = nullptr;
	}
}

static int 
psnavi_get_usb_feature_report(
	PSNaviUSBContext *navi_context,
	unsigned char *report_bytes,
	size_t report_size)
{
	int result = libusb_control_transfer(
		navi_context->usb_device_handle, 
		CTRL_IN, 
		HID_GET_REPORT, 
		(HID_REPORT_TYPE_FEATURE << 8) | report_bytes[0],
		0, 
		report_bytes,
		static_cast<uint16_t>(report_size), 
		CONTROL_TRANSFER_TIMEOUT);

	if (result < LIBUSB_SUCCESS) 
	{
		const char * error_text = libusb_strerror(static_cast<libusb_error>(result));
		SERVER_LOG_ERROR("psnavi_get_usb_hid_report") << "Control transfer failed with error: " << error_text;
	}

	return result;
}

static int
psnavi_send_usb_feature_report(
	PSNaviUSBContext *navi_context,
	unsigned char *report_bytes,
	size_t report_size)
{
	int result = libusb_control_transfer(
		navi_context->usb_device_handle, 
		CTRL_OUT, 
		HID_SET_REPORT, 
		(HID_REPORT_TYPE_FEATURE << 8) | report_bytes[0],
		0,
		report_bytes+1, // don't send the report id
		static_cast<uint16_t>(report_size-1), // don't include the report id in the size
		CONTROL_TRANSFER_TIMEOUT);

	if (result >= 0)
	{
		// add the report id size back in the result
		++result;
	}
	else
	{
		const char * error_text = libusb_strerror(static_cast<libusb_error>(result));
		SERVER_LOG_ERROR("psnavi_get_usb_hid_report") << "Control transfer failed with error: " << error_text;
	}

	return result;
}

static int
psnavi_read_usb_interrupt_pipe(
	PSNaviUSBContext *navi_context,
	unsigned char *buffer,
	size_t max_buffer_size)
{
	int actual_length = 0;
	int result = libusb_interrupt_transfer(
		navi_context->usb_device_handle, 
		0x81,
		buffer, 
		max_buffer_size,
		&actual_length, 
		0);

	if (result >= 0)
	{
		result = actual_length;
	}
	else
	{
		const char * error_text = libusb_strerror(static_cast<libusb_error>(result));
		SERVER_LOG_ERROR("psnavi_read_usb_interrupt_pipe") << "Control transfer failed with error: " << error_text;
	}

	return result;
}
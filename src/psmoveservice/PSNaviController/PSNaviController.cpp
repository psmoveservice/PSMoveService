//-- includes -----
#include "AtomicPrimitives.h"
#include "PSNaviController.h"
#include "ControllerDeviceEnumerator.h"
#include "ControllerUSBDeviceEnumerator.h"
#include "ControllerHidDeviceEnumerator.h"
#include "ControllerGamepadEnumerator.h"
#include "ServerLog.h"
#include "ServerUtility.h"
#include "USBDeviceManager.h"
#include "WorkerThread.h"

#include "hidapi.h"

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
#define HID_READ_TIMEOUT			  100 /* timeout in ms */

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

	// HIDApi state
    std::string hid_device_path;
	std::string hid_serial_number;
	int hid_interface_number;
    hid_device *hid_device_handle;

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
		hid_device_path = "";
		hid_serial_number = "";
		hid_interface_number = -1;
		hid_device_handle = nullptr;
	}
};

struct PSNaviDataInputRawUSB {
    uint8_t type; /* message type, must be PSNavi_Req_GetInput */
	uint8_t _unk0[1];
    uint8_t buttons1; // (L3, D-pad)
    uint8_t buttons2; // (X, Circle, L1, L2)
    uint8_t buttons3; // (PS Button)
	uint8_t _unk1[1];
    uint8_t stick_xaxis; // stick value; 0..255 (subtract 0x80 to get signed values)
    uint8_t stick_yaxis; // stick value; 0..255 (subtract 0x80 to get signed values)
	uint8_t _unk2[6];
    uint8_t analog_dpad_up; // 0x00 (not pressed) to 0xff (fully pressed)
    uint8_t analog_dpad_right; // 0x00 (not pressed) to 0xff (fully pressed)
    uint8_t analog_dpad_down; // 0x00 (not pressed) to 0xff (fully pressed)
    uint8_t analog_dpad_left; // 0x00 (not pressed) to 0xff (fully pressed)
    uint8_t analog_l2; // 0x00 (not pressed) to 0xff (fully pressed)
	uint8_t _unk3[1];
    uint8_t analog_l1; // 0x00 (not pressed) to 0xff (fully pressed)
	uint8_t _unk4[2];
    uint8_t analog_circle; // 0x00 (not pressed) to 0xff (fully pressed)
    uint8_t analog_cross; // 0x00 (not pressed) to 0xff (fully pressed)
	uint8_t _unk[5];
    uint8_t battery; // 0x05 = fully charged, 0xEE = charging or 0xEF = fully charged
};

struct PSNaviDataInputHID {
	uint8_t X : 1;
	uint8_t O : 1;
	uint8_t L1 : 1;
	uint8_t L2 : 1;
	uint8_t L3 : 1;
	uint8_t PS : 1;
	uint8_t DPAD : 4;
	int16_t JX;
	int16_t JY;
	int16_t Trigger;
};
 
enum PSNaviHidDPAD {
	CENTER = 0x08,
	UP = 0x00,
	UP_RIGHT = 0x01,
	RIGHT = 0x02,
	DOWN_RIGHT = 0x03,
	DOWN = 0x04,
	DOWN_LEFT = 0x05,
	LEFT = 0x06,
	UP_LEFT = 0x07
};

// -- private prototypes -----
static std::string NaviBTAddrUcharToString(const unsigned char* addr_buff);
static bool stringToNaviBTAddrUchar(const std::string &addr, unsigned char *addr_buff, const int addr_buf_size);
inline void setButtonBit(unsigned int &buttons, unsigned int button_mask, bool is_pressed);
inline enum CommonControllerState::ButtonState getButtonState(unsigned int buttons, unsigned int lastButtons, int buttonMask);

inline uint8_t signedInt16ToUInt8(const int16_t int16_value);

static int psnavi_get_usb_feature_report(t_usb_device_handle device_handle, unsigned char *report_bytes, size_t report_size);
static int psnavi_send_usb_feature_report(t_usb_device_handle device_handle, unsigned char *report_bytes, size_t report_size);
static int psnavi_read_usb_interrupt_pipe(t_usb_device_handle device_handle, unsigned char *buffer, size_t max_buffer_size);

// -- PSNaviHidPacketProcessor --
class PSNaviHidPacketProcessor : public WorkerThread
{
public:
	PSNaviHidPacketProcessor() 
		: WorkerThread("PSNaviHIDProcessor")
		, m_hidDevice(nullptr)
	{
		PSNaviDataInputHID rawHIDPacket;
		memset(&rawHIDPacket, 0, sizeof(PSNaviDataInputHID));
		m_publishedHIDPacket.storeValue(rawHIDPacket);
	}

	void fetchLatestHIDPacket(PSNaviDataInputHID &input_state)
	{
		m_publishedHIDPacket.fetchValue(input_state);
	}

    void start(hid_device *in_hid_device)
    {
		if (!hasThreadStarted())
		{
			m_hidDevice= in_hid_device;

			// Perform blocking reads on the worker thread
			hid_set_nonblocking(m_hidDevice, 0);

			// Fire up the worker thread
			WorkerThread::startThread();
		}
    }

	void stop()
	{
		WorkerThread::stopThread();
	}

protected:
	virtual bool doWork() override
    {
		// Attempt to read the next sensor update packet from 
		PSNaviDataInputHID rawHIDPacket;
		memset(&rawHIDPacket, 0, sizeof(PSNaviDataInputHID));
		int res = hid_read_timeout(m_hidDevice, (unsigned char*)&rawHIDPacket, sizeof(PSNaviDataInputHID), HID_READ_TIMEOUT);

		if (res > 0)
		{
			// Publish the new state to the main thread
			m_publishedHIDPacket.storeValue(rawHIDPacket);
		}
		else if (res < 0)
		{
			char hidapi_err_mbs[256];
			bool valid_error_mesg = 
				ServerUtility::convert_wcs_to_mbs(hid_error(m_hidDevice), hidapi_err_mbs, sizeof(hidapi_err_mbs));

			// Device no longer in valid state.
			if (valid_error_mesg)
			{
				SERVER_MT_LOG_ERROR("PSMoveSensorProcessor::doWork") << "HID ERROR: " << hidapi_err_mbs;
			}

			// Halt the worker threads
			return false;
		}

		return true;
    }

    // Multi-threaded state
	hid_device *m_hidDevice;
	AtomicObject<PSNaviDataInputHID> m_publishedHIDPacket;
};

// -- public methods

// -- PSMove Controller Config
const boost::property_tree::ptree
PSNaviControllerConfig::config2ptree()
{
    boost::property_tree::ptree pt;

    pt.put("max_poll_failute_count", max_poll_failure_count);
	pt.put("attached_to_controller", attached_to_controller);

    return pt;
}

void
PSNaviControllerConfig::ptree2config(const boost::property_tree::ptree &pt)
{
    max_poll_failure_count = pt.get<long>("max_poll_failute_count", 100);
	attached_to_controller= pt.get<std::string>("attached_to_controller", "");
}

// -- PSMove Controller -----
PSNaviController::PSNaviController()
{   
	APIContext = new PSNaviAPIContext();
	m_HIDPacketProcessor= nullptr;

	m_cachedInputState.clear();
	NextPollSequenceNumber= 0;
}

PSNaviController::~PSNaviController()
{
    if (getIsOpen())
    {
        SERVER_LOG_ERROR("~PSNaviController") << "Controller deleted without calling close() first!";
    }

	if (m_HIDPacketProcessor)
	{
		delete m_HIDPacketProcessor;
	}

	delete APIContext;
}

bool PSNaviController::open()
{
    ControllerDeviceEnumerator enumerator(ControllerDeviceEnumerator::CommunicationType_ALL, CommonControllerState::PSNavi);
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
		else if (pEnum->get_api_type() == ControllerDeviceEnumerator::CommunicationType_HID)
		{
			const ControllerHidDeviceEnumerator *hidEnum = pEnum->get_hid_controller_enumerator();
			assert(hidEnum != nullptr);

			APIContext->vendor_id = pEnum->get_vendor_id();
			APIContext->product_id = pEnum->get_product_id();
			APIContext->hid_device_path= pEnum->get_path();
			APIContext->hid_interface_number= hidEnum->get_interface_number();
			APIContext->hid_device_handle = hid_open_path(APIContext->hid_device_path.c_str());

			char szSerialNo[64];
			if (hidEnum->get_serial_number(szSerialNo, sizeof(szSerialNo)))
			{
				APIContext->hid_serial_number= szSerialNo;
			}
			else
			{
				APIContext->hid_serial_number= "";
			}

			if (APIContext->hid_device_handle != nullptr)
			{
				SERVER_LOG_INFO("PSNaviController::open") << "  Successfully opened HID path: " << pEnum->get_path();

				// Sadly this info available to us through the HID api
				APIContext->host_bluetooth_address= "00:00:00:00:00:00";
				APIContext->controller_bluetooth_address= "00:00:00:00:00:00";
			}
			else
			{
				SERVER_LOG_ERROR("PSNaviController::open") << "  Failed to open HID controller";
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

				if (gamepad != nullptr)
				{
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
					SERVER_LOG_ERROR("PSNaviController::open") << "  Failed to open gamepad (gamepad disconnected)";
				}
			}
			else
			{
				SERVER_LOG_ERROR("PSNaviController::open") << "  Failed to open gamepad (invalid game index)";
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
			else if (pEnum->get_api_type() == ControllerDeviceEnumerator::CommunicationType_HID)
			{
				char config_name[255];
				ServerUtility::format_string(config_name, sizeof(config_name), "psnavi_%s_%d", APIContext->hid_serial_number.c_str(), APIContext->hid_interface_number);

				// Load the config file
				cfg = PSNaviControllerConfig(std::string(config_name));
				cfg.load();

				// Create the sensor processor thread
				m_HIDPacketProcessor= new PSNaviHidPacketProcessor();
				m_HIDPacketProcessor->start(APIContext->hid_device_handle);

				// Save it back out again in case any defaults changed
				cfg.save();

				success = true;
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
		else if (APIContext->hid_device_handle != nullptr)
		{
			SERVER_LOG_INFO("PSNaviController::close") << "Closing PSNaviController(" << APIContext->hid_device_path << ")";

			if (m_HIDPacketProcessor != nullptr)
			{
				// halt the HID packet processing thread
				m_HIDPacketProcessor->stop();
				delete m_HIDPacketProcessor;
				m_HIDPacketProcessor= nullptr;
			}

			hid_close(APIContext->hid_device_handle);
			APIContext->hid_device_handle = nullptr;
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

void PSNaviController::setControllerListener(IControllerListener *listener)
{
	// Do nothing. PSNavi doesn't provide IMU data.
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
		else if (APIContext->hid_device_handle != nullptr)
		{
			dev_path = APIContext->hid_device_path.c_str();
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
	// Assumed bluetooth connection if the controller is connected through the gamepad or HID interfaces
    return APIContext->hid_device_handle != nullptr || APIContext->gamepad_index != -1;
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
    return (APIContext->usb_device_handle != k_invalid_usb_device_handle || APIContext->hid_device_handle != nullptr || APIContext->gamepad_index != -1);
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
		else if (APIContext->hid_device_handle != nullptr)
		{
			// For the gamepad case, just consider the current state is new data
			result = pollHid();
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
PSNaviController::pollHid()
{
	assert(getIsOpen());

	if (m_HIDPacketProcessor != nullptr && !m_HIDPacketProcessor->hasThreadEnded())
	{
		PSNaviDataInputHID rawHIDPacket;
		m_HIDPacketProcessor->fetchLatestHIDPacket(rawHIDPacket);

		// Copy the current input state off to the previous state for comparison
		PSNaviControllerInputState previousInputState= m_cachedInputState;
		m_cachedInputState.clear();

		// Increment the sequence for every new polling packet
		m_cachedInputState.PollSequenceNumber = NextPollSequenceNumber;
		++NextPollSequenceNumber;

		// New Button State
		bool bIsDPadUpPressed= 
			rawHIDPacket.DPAD == PSNaviHidDPAD::UP || 
			rawHIDPacket.DPAD == PSNaviHidDPAD::UP_LEFT ||
			rawHIDPacket.DPAD == PSNaviHidDPAD::UP_RIGHT;
		bool bIsDPadDownPressed=
			rawHIDPacket.DPAD == PSNaviHidDPAD::DOWN || 
			rawHIDPacket.DPAD == PSNaviHidDPAD::DOWN_LEFT ||
			rawHIDPacket.DPAD == PSNaviHidDPAD::DOWN_RIGHT;
		bool bIsDPadLeftPressed= 
			rawHIDPacket.DPAD == PSNaviHidDPAD::UP_LEFT ||
			rawHIDPacket.DPAD == PSNaviHidDPAD::LEFT || 
			rawHIDPacket.DPAD == PSNaviHidDPAD::DOWN_LEFT;
		bool bIsDPadRightPressed=
			rawHIDPacket.DPAD == PSNaviHidDPAD::UP_RIGHT ||
			rawHIDPacket.DPAD == PSNaviHidDPAD::RIGHT || 
			rawHIDPacket.DPAD == PSNaviHidDPAD::DOWN_RIGHT;
		bool bIsL1Pressed= rawHIDPacket.L1 != 0;
		bool bIsL2Pressed= rawHIDPacket.L2 != 0;
		bool bIsL3Pressed= rawHIDPacket.L3 != 0;
		bool bIsCrossPressed= rawHIDPacket.X != 0;
		bool bIsCirclePressed= rawHIDPacket.O != 0;
		bool bIsPSPressed = rawHIDPacket.PS != 0;

		// Create a new state bitmask
		m_cachedInputState.AllButtons = 0;
		setButtonBit(m_cachedInputState.AllButtons, Btn_UP, bIsDPadUpPressed);
		setButtonBit(m_cachedInputState.AllButtons, Btn_DOWN, bIsDPadDownPressed);
		setButtonBit(m_cachedInputState.AllButtons, Btn_LEFT, bIsDPadLeftPressed);
		setButtonBit(m_cachedInputState.AllButtons, Btn_RIGHT, bIsDPadRightPressed);
		setButtonBit(m_cachedInputState.AllButtons, Btn_CROSS, bIsCrossPressed);
		setButtonBit(m_cachedInputState.AllButtons, Btn_CIRCLE, bIsCirclePressed);
		setButtonBit(m_cachedInputState.AllButtons, Btn_L1, bIsL1Pressed);
		setButtonBit(m_cachedInputState.AllButtons, Btn_L2, bIsL2Pressed);
		setButtonBit(m_cachedInputState.AllButtons, Btn_L3, bIsL3Pressed);
		setButtonBit(m_cachedInputState.AllButtons, Btn_PS, bIsPSPressed);

		// Compare the new button state against the old button state
		unsigned int lastButtons = previousInputState.AllButtons;
		m_cachedInputState.DPad_Up = getButtonState(m_cachedInputState.AllButtons, lastButtons, Btn_UP);
		m_cachedInputState.DPad_Down = getButtonState(m_cachedInputState.AllButtons, lastButtons, Btn_DOWN);
		m_cachedInputState.DPad_Left = getButtonState(m_cachedInputState.AllButtons, lastButtons, Btn_LEFT);
		m_cachedInputState.DPad_Right = getButtonState(m_cachedInputState.AllButtons, lastButtons, Btn_RIGHT);
		m_cachedInputState.Circle = getButtonState(m_cachedInputState.AllButtons, lastButtons, Btn_CIRCLE);
		m_cachedInputState.Cross = getButtonState(m_cachedInputState.AllButtons, lastButtons, Btn_CROSS);
		m_cachedInputState.PS = getButtonState(m_cachedInputState.AllButtons, lastButtons, Btn_PS);
		m_cachedInputState.L1 = getButtonState(m_cachedInputState.AllButtons, lastButtons, Btn_L1);
		m_cachedInputState.L2 = getButtonState(m_cachedInputState.AllButtons, lastButtons, Btn_L2);
		m_cachedInputState.L3 = getButtonState(m_cachedInputState.AllButtons, lastButtons, Btn_L3);

		// Analog triggers
		//###HipsterSloth $TODO Long term these should be floats of unit size
		m_cachedInputState.Stick_XAxis = signedInt16ToUInt8(rawHIDPacket.JX);
		m_cachedInputState.Stick_YAxis = signedInt16ToUInt8(rawHIDPacket.JY);
		m_cachedInputState.Trigger = signedInt16ToUInt8(rawHIDPacket.Trigger);

		// Can't report the true battery state
		m_cachedInputState.Battery = CommonControllerState::Batt_MAX;

		return IDeviceInterface::_PollResultSuccessNewData;
	}
	else
	{
		return IDeviceInterface::_PollResultFailure;
	}
}

IControllerInterface::ePollResult
PSNaviController::pollGamepad()
{
	assert(getIsOpen());

	const Gamepad_device * gamepad = Gamepad_deviceAtIndex(static_cast<unsigned int>(APIContext->gamepad_index));
	IControllerInterface::ePollResult result= IControllerInterface::_PollResultSuccessNewData;

	if (gamepad != nullptr)
	{
		PSNaviControllerInputState newState;

		// Increment the sequence for every new polling packet
		newState.PollSequenceNumber = NextPollSequenceNumber;
		++NextPollSequenceNumber;

		// New Button State
		bool bIsDPadUpPressed= gamepad->buttonStates[0];
		bool bIsDPadDownPressed= gamepad->buttonStates[1];
		bool bIsDPadLeftPressed= gamepad->buttonStates[2];
		bool bIsDPadRightPressed= gamepad->buttonStates[3];
		bool bIsL2Pressed= gamepad->axisStates[4] >= .9f;
		bool bIsL3Pressed= gamepad->buttonStates[6];
		bool bIsL1Pressed= gamepad->buttonStates[8];
		bool bIsCrossPressed= gamepad->buttonStates[10];
		bool bIsCirclePressed= gamepad->buttonStates[11];
		bool bIsPSPressed = gamepad->buttonStates[14];

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
		setButtonBit(newState.AllButtons, Btn_PS, bIsPSPressed);

		// Button de-bounce
		unsigned int lastButtons = m_cachedInputState.AllButtons;
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
		newState.Trigger = static_cast<unsigned char>((gamepad->axisStates[4] + 1.f) * 127.f);

		// Can't report the true battery state
		newState.Battery = CommonControllerState::Batt_MAX;

		// Cache the newest controller state
		m_cachedInputState= newState;
	}
	else
	{
		result= IControllerInterface::_PollResultFailure;
	}

	return result;
}

IControllerInterface::ePollResult
PSNaviController::pollUSB()
{
	assert(getIsOpen());
	assert(!getIsBluetooth());

	IControllerInterface::ePollResult poll_result;
    
	PSNaviDataInputRawUSB InData;
	int res = psnavi_read_usb_interrupt_pipe(APIContext->usb_device_handle, (unsigned char *)&InData, sizeof(PSNaviDataInputRawUSB));

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
		// https://github.com/nitsch/moveonpc/wiki/Input-report
		PSNaviControllerInputState newState;
		newState.clear();

		// Increment the sequence for every new polling packet
		newState.PollSequenceNumber = NextPollSequenceNumber;
		++NextPollSequenceNumber;

		// Buttons
		newState.AllButtons =
			(InData.buttons1) |               // |Left|Down|Right|Up|-|-|L3|-
			(InData.buttons2 << 8) |          // |Cross|-|Circle|-|L1|-|-|L2|
			((InData.buttons3 & 0x01) << 16); // |-|-|-|-|-|-|-|PS|

		unsigned int lastButtons = m_cachedInputState.AllButtons;

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
		newState.Trigger = InData.analog_l2;
		newState.Stick_XAxis = InData.stick_xaxis;
		newState.Stick_YAxis = InData.stick_yaxis;

		// Other
		newState.Battery = static_cast<CommonControllerState::BatteryLevel>(InData.battery);

		// Cache the new controller state
		m_cachedInputState= newState;
		poll_result = IControllerInterface::_PollResultSuccessNewData;
	}

	return poll_result;
}

const CommonDeviceState *
PSNaviController::getState(
	int lookBack) const
{
    return &m_cachedInputState;
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

bool PSNaviController::getWasSystemButtonPressed() const
{
    const PSNaviControllerInputState *psnavi_state= static_cast<const PSNaviControllerInputState *>(getState());
    bool bWasPressed= false;

    if (psnavi_state != nullptr)
    {
        bWasPressed= psnavi_state->PS == CommonControllerState::Button_PRESSED;
    }

    return bWasPressed;
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
setButtonBit(unsigned int &buttons, unsigned int button_mask, bool is_pressed)
{
	if (is_pressed)
	{
		buttons|= button_mask;
	}
	else
	{
		buttons&= ~button_mask;
	}
}

inline enum CommonControllerState::ButtonState
getButtonState(unsigned int buttons, unsigned int lastButtons, int buttonMask)
{
    return (enum CommonControllerState::ButtonState)((((lastButtons & buttonMask) > 0) << 1) + ((buttons & buttonMask)>0));
}

inline uint8_t signedInt16ToUInt8(const int16_t int16_value)
{
	const int32_t int32_value= (int32_t)int16_value;
	const float unit_value= (float)(int32_value + 32767) / 65535.f;
	const uint8_t uint8_value= (uint8_t)(unit_value * 255.f);

	return uint8_value;
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
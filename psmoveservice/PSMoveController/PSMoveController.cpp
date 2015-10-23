#include "PSMoveController.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>



#define PSMOVE_VID 0x054c
#define PSMOVE_PID 0x03d5
#define PSMOVE_BUFFER_SIZE 49 /* Buffer size for writing LEDs and reading sensor data */
#define PSMOVE_EXT_DATA_BUF_SIZE 5
#ifdef _WIN32
#define PSMOVE_BTADDR_GET_SIZE 17
#else
#define PSMOVE_BTADDR_GET_SIZE 16
#endif
#define PSMOVE_BTADDR_SIZE 6

enum PSMove_Request_Type {
    PSMove_Req_GetInput = 0x01,
    PSMove_Req_SetLEDs = 0x02,
    PSMove_Req_SetLEDPWMFrequency = 0x03,
    PSMove_Req_GetBTAddr = 0x04,
    PSMove_Req_SetBTAddr = 0x05,
    PSMove_Req_GetCalibration = 0x10,
    PSMove_Req_SetAuthChallenge = 0xA0,
    PSMove_Req_GetAuthResponse = 0xA1,
    PSMove_Req_GetExtDeviceInfo = 0xE0,
    PSMove_Req_SetExtDeviceInfo = 0xE0,
    PSMove_Req_SetDFUMode = 0xF2,
    PSMove_Req_GetFirmwareInfo = 0xF9,
    
    /**
     * Permanently set LEDs via USB
     *
     * Writing USB report 0xFA controls the LEDs. But unlike BT report
     * 0x02 this one keeps the sphere glowing as long as the USB cable
     * is connected, i.e. no refresh updates need to be send. Not sure
     * why that one exists. Might be useful for debugging though.
     *
     * TODO: Can't get this to work, but I could imagine it be useful for
     * things like notification apps that don't have to be running all the
     * time. Maybe it only works in specific controller firmware versions?
     *
     * http://lists.ims.tuwien.ac.at/pipermail/psmove/2013-March/000335.html
     * https://github.com/thp/psmoveapi/issues/55
     **/
    PSMove_Req_SetLEDsPermanentUSB = 0xFA,
};

struct PSMove_Data_Out {
    unsigned char type;     /* message type, must be PSMove_Req_SetLEDs */
    unsigned char _zero;    /* must be zero */
    unsigned char r;        /* red value, 0x00..0xff */
    unsigned char g;        /* green value, 0x00..0xff */
    unsigned char b;        /* blue value, 0x00..0xff */
    unsigned char rumble2;  /* unknown, should be 0x00 for now */
    unsigned char rumble;   /* rumble value, 0x00..0xff */
    unsigned char _padding[PSMOVE_BUFFER_SIZE-7]; /* must be zero */
};

struct PSMove_Data_In {
    unsigned char type;             /* message type, must be PSMove_Req_GetInput */
    unsigned char buttons1;
    unsigned char buttons2;
    unsigned char buttons3;
    unsigned char buttons4;
    unsigned char trigger;          /* trigger value; 0..255 */
    unsigned char trigger2;         /* trigger value, 2nd frame */
    unsigned char _unk7;
    unsigned char _unk8;
    unsigned char _unk9;
    unsigned char _unk10;
    unsigned char timehigh;         /* high byte of timestamp */
    unsigned char battery;          /* battery level; 0x05 = max, 0xEE = USB charging */
    unsigned char aXlow;            /* low byte of accelerometer X value */
    unsigned char aXhigh;           /* high byte of accelerometer X value */
    unsigned char aYlow;
    unsigned char aYhigh;
    unsigned char aZlow;
    unsigned char aZhigh;
    unsigned char aXlow2;           /* low byte of accelerometer X value, 2nd frame */
    unsigned char aXhigh2;          /* high byte of accelerometer X value, 2nd frame */
    unsigned char aYlow2;
    unsigned char aYhigh2;
    unsigned char aZlow2;
    unsigned char aZhigh2;
    unsigned char gXlow;            /* low byte of gyro X value */
    unsigned char gXhigh;           /* high byte of gyro X value */
    unsigned char gYlow;
    unsigned char gYhigh;
    unsigned char gZlow;
    unsigned char gZhigh;
    unsigned char gXlow2;           /* low byte of gyro X value, 2nd frame */
    unsigned char gXhigh2;          /* high byte of gyro X value, 2nd frame */
    unsigned char gYlow2;
    unsigned char gYhigh2;
    unsigned char gZlow2;
    unsigned char gZhigh2;
    unsigned char temphigh;         /* temperature (bits 12-5) */
    unsigned char templow_mXhigh;   /* temp (bits 4-1); magneto X (bits 12-9) */
    unsigned char mXlow;            /* magnetometer X (bits 8-1) */
    unsigned char mYhigh;           /* magnetometer Y (bits 12-5) */
    unsigned char mYlow_mZhigh;     /* magnetometer: Y (bits 4-1), Z (bits 12-9) */
    unsigned char mZlow;            /* magnetometer Z (bits 8-1) */
    unsigned char timelow;          /* low byte of timestamp */
    unsigned char extdata[PSMOVE_EXT_DATA_BUF_SIZE]; /* external device data (EXT port) */
};

int PSMoveController::s_nOpened = 0;

PSMoveController::PSMoveController(int next_ith)
	: index(0), ledr(255), ledg(0), ledb(0), rumble(0)
{
	HIDDetails.handle = nullptr;
	HIDDetails.handle_addr = nullptr;

	std::cout << "Looking for PSMoveController(" << s_nOpened+next_ith << ")" << std::endl;
    struct hid_device_info *devs, *cur_dev;
    int count = 0;
    
    devs = hid_enumerate(PSMOVE_VID, PSMOVE_PID);
	if (devs)
	{
		cur_dev = devs;
		while (cur_dev) {
			std::cout << "Found device path: " << cur_dev->path << std::endl;
			if (cur_dev->serial_number != NULL)
			{
				std::wcout << "with serial_number: " << cur_dev->serial_number << std::endl;
			}
			else
			{
				std::wcout << "with NULL serial_number" << std::endl;
			}
			isBluetooth = (cur_dev->serial_number != NULL);
			// On my Mac, using bluetooth,
			// cur_dev->path = Bluetooth_054c_03d5_779732e8
			// cur_dev->serial_number = 00-06-f7-97-32-e8
			// On my Mac, using USB,
			// cur_dev->path = USB_054c_03d5_14100000
			// cur_dev->serial_number =   (NULL)

			// On my Windows 10 box (different controller), using bluetooth
			// cur_dev->path = \\?\hid#{00001124-0000-1000-8000-00805f9b34fb}_vid&0002054c_pid&03d5&col01#9&456a2d2&2&0000#{4d1e55b2-f16f-11cf-88cb-001111000030}
			// cur_dev->serial_number = 0006f718cdf3
			// Using USB
			// cur_dev->path = \\?\hid#vid_054c&pid_03d5&col01#6&7773e57&0&0000#{4d1e55b2-f16f-11cf-88cb-001111000030}
			// cur_dev->serial_number = (NULL)

#ifdef _WIN32
			/**
			* Windows Quirk: Each dev is enumerated 3 times.
			* The one with "&col01#" in the path is the one we will get most of our data from. Only count this one.
			* The one with "&col02#" in the path is the one we will get the bluetooth address from.
			**/
			if (strstr(cur_dev->path, "&col01#") == NULL) {
				count--;
			}
#endif

			count++;
			if (count == (s_nOpened + next_ith))
			{
				index = count;
				HIDDetails.device_path = cur_dev->path;

#ifdef _WIN32
				HIDDetails.device_path_addr = HIDDetails.device_path;
				HIDDetails.device_path_addr.replace(HIDDetails.device_path_addr.find("&col01#"), 7, "&col02#");
				HIDDetails.device_path_addr.replace(HIDDetails.device_path_addr.find("&0000#"), 6, "&0001#");
				HIDDetails.handle_addr = hid_open_path(HIDDetails.device_path_addr.c_str());
				hid_set_nonblocking(HIDDetails.handle_addr, 1);
#endif

				// Open the device using the serial_number if available, else use the path
				if ((cur_dev->serial_number == NULL) && (!HIDDetails.device_path.empty()))
				{
					HIDDetails.handle = hid_open_path(HIDDetails.device_path.c_str());
				}
				else
				{
					HIDDetails.handle = hid_open(PSMOVE_VID, PSMOVE_PID, cur_dev->serial_number);
				}
				hid_set_nonblocking(HIDDetails.handle, 1);
				s_nOpened++;
				break;
			}
			else
			{
				cur_dev = cur_dev->next;
			}

		}
		hid_free_enumeration(devs);

		if (isOpen())  // Controller was opened and has an index
		{
			// Get the bluetooth address
			std::string host;
			int success = getBTAddress(host, HIDDetails.bt_addr);
			if (!success)
			{
				// TODO: If serial is still bad, maybe we have a disconnected controller still showing up in hidapi
			}
			// TODO: Other startup.
			// Load calibration from file
		}
	}
	else
	{
		std::cerr << "No PSMoveControllers found." << std::endl;
	}
}

PSMoveController::~PSMoveController()
{
	if (isOpen())
	{
		hid_close(HIDDetails.handle);
		s_nOpened--;
	}
	if ((HIDDetails.handle_addr != nullptr) && (HIDDetails.handle_addr != NULL))
    {
        hid_close(HIDDetails.handle_addr);
    }
}

bool
PSMoveController::isOpen()
{
	return ((index > 0) && (HIDDetails.handle != nullptr) && (HIDDetails.handle != NULL));
}

psmovePosef
PSMoveController::getPose(int msec_time)
{
    psmovePosef nullPose;
    return nullPose;
}

int
PSMoveController::getButtonState()
{
    return 0;
}

void
PSMoveController::writeDataOut()
{
    PSMove_Data_Out data_out = PSMove_Data_Out();  // 0-initialized
	data_out.type = PSMove_Req_SetLEDs;
	data_out.rumble2 = 0x00;
    data_out.r = ledr;
    data_out.g = ledg;
    data_out.b = ledb;
    data_out.rumble = rumble;
    
    hid_write(HIDDetails.handle, (unsigned char*)(&data_out),
                        sizeof(data_out));
}

void
PSMoveController::setRumbleValue(unsigned char value)
{
    rumble = value;
    writeDataOut();
}

static std::string
btAddrUcharToString(const unsigned char* addr_buff)
{
	// http://stackoverflow.com/questions/11181251/saving-hex-values-to-a-c-string
	std::ostringstream stream;
	int buff_ind = 5;
	for (buff_ind = 5; buff_ind >= 0; buff_ind--)
	{
		stream << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(addr_buff[buff_ind]);
		if (buff_ind > 0)
		{
			stream << ":";
		}
	}
	return stream.str();
}

int
PSMoveController::getBTAddress(std::string& host, std::string& controller)
{
    int res;
    int success = 0;
    unsigned char btg[PSMOVE_BTADDR_GET_SIZE];
    unsigned char ctrl_char_buff[PSMOVE_BTADDR_SIZE];
    unsigned char host_char_buff[PSMOVE_BTADDR_SIZE];
    
    memset(btg, 0, sizeof(btg));
    btg[0] = PSMove_Req_GetBTAddr;
    /* _WIN32 only has move->handle_addr for getting bluetooth address. */
    if (HIDDetails.handle_addr) {
        res = hid_get_feature_report(HIDDetails.handle_addr, btg, sizeof(btg));
    } else {
        res = hid_get_feature_report(HIDDetails.handle, btg, sizeof(btg));
    }
    
    if (res == sizeof(btg)) {
        
        memcpy(host_char_buff, btg+10, PSMOVE_BTADDR_SIZE);
        host = btAddrUcharToString(host_char_buff);
        
        memcpy(ctrl_char_buff, btg+1, PSMOVE_BTADDR_SIZE);
        controller = btAddrUcharToString(ctrl_char_buff);

        success = 1;
    }
    return success;
}
#include "PSMoveController.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <vector>
#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif
#include <math.h>

#define PSMOVE_VID 0x054c
#define PSMOVE_PID 0x03d5
#define PSMOVE_BUFFER_SIZE 49 /* Buffer size for writing LEDs and reading sensor data */
#define PSMOVE_EXT_DATA_BUF_SIZE 5
#define PSMOVE_BTADDR_GET_SIZE 16
#define PSMOVE_BTADDR_SIZE 6
#define PSMOVE_CALIBRATION_SIZE 49 /* Buffer size for calibration data */
#define PSMOVE_CALIBRATION_BLOB_SIZE (PSMOVE_CALIBRATION_SIZE*3 - 2*2) /* Three blocks, minus header (2 bytes) for blocks 2,3 */
#define PSMOVE_STATE_BUFFER_MAX 16

/* Decode 12-bit signed value (assuming two's complement) */
#define TWELVE_BIT_SIGNED(x) (((x) & 0x800)?(-(((~(x)) & 0xFFF) + 1)):(x))

enum PSMoveRequestType {
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

enum PSMoveButton {
	// https://github.com/nitsch/moveonpc/wiki/Input-report
	Btn_TRIANGLE = 1 << 4,	// Green triangle
	Btn_CIRCLE = 1 << 5,	// Red circle
	Btn_CROSS = 1 << 6,		// Blue cross
	Btn_SQUARE = 1 << 7,	// Pink square
	Btn_SELECT = 1 << 8,	// Select button, left side
	Btn_START = 1 << 11,	// Start button, right side
	Btn_PS = 1 << 16,		// PS button, front center
	Btn_MOVE = 1 << 19,		// Move button, big front button
	Btn_T = 1 << 20,		// Trigger, on the back

#if 0
	/* Not used for now - only on Sixaxis/DS3 or nav controller */
	Btn_L2 = 1 << 0x00,
	Btn_R2 = 1 << 0x01,
	Btn_L1 = 1 << 0x02,
	Btn_R1 = 1 << 0x03,
	Btn_L3 = 1 << 0x09,
	Btn_R3 = 1 << 0x0A,
	Btn_UP = 1 << 0x0C,
	Btn_RIGHT = 1 << 0x0D,
	Btn_DOWN = 1 << 0x0E,
	Btn_LEFT = 1 << 0x0F,
#endif
};

struct PSMoveDataOutput {
    unsigned char type;     /* message type, must be PSMove_Req_SetLEDs */
    unsigned char _zero;    /* must be zero */
    unsigned char r;        /* red value, 0x00..0xff */
    unsigned char g;        /* green value, 0x00..0xff */
    unsigned char b;        /* blue value, 0x00..0xff */
    unsigned char rumble2;  /* unknown, should be 0x00 for now */
    unsigned char rumble;   /* rumble value, 0x00..0xff */
    unsigned char _padding[PSMOVE_BUFFER_SIZE-7]; /* must be zero */
};

struct PSMoveDataInput {
	unsigned char type; /* message type, must be PSMove_Req_GetInput */
	unsigned char buttons1;
	unsigned char buttons2;
	unsigned char buttons3;
	unsigned char buttons4;
	unsigned char trigger; /* trigger value; 0..255 */
	unsigned char trigger2; /* trigger value, 2nd frame */
	unsigned char _unk7;
	unsigned char _unk8;
	unsigned char _unk9;
	unsigned char _unk10;
	unsigned char timehigh; /* high byte of timestamp */
	unsigned char battery; /* battery level; 0x05 = max, 0xEE = USB charging */
	unsigned char aXlow; /* low byte of accelerometer X value */
	unsigned char aXhigh; /* high byte of accelerometer X value */
	unsigned char aYlow;
	unsigned char aYhigh;
	unsigned char aZlow;
	unsigned char aZhigh;
	unsigned char aXlow2; /* low byte of accelerometer X value, 2nd frame */
	unsigned char aXhigh2; /* high byte of accelerometer X value, 2nd frame */
	unsigned char aYlow2;
	unsigned char aYhigh2;
	unsigned char aZlow2;
	unsigned char aZhigh2;
	unsigned char gXlow; /* low byte of gyro X value */
	unsigned char gXhigh; /* high byte of gyro X value */
	unsigned char gYlow;
	unsigned char gYhigh;
	unsigned char gZlow;
	unsigned char gZhigh;
	unsigned char gXlow2; /* low byte of gyro X value, 2nd frame */
	unsigned char gXhigh2; /* high byte of gyro X value, 2nd frame */
	unsigned char gYlow2;
	unsigned char gYhigh2;
	unsigned char gZlow2;
	unsigned char gZhigh2;
	unsigned char temphigh; /* temperature (bits 12-5) */
	unsigned char templow_mXhigh; /* temp (bits 4-1); magneto X (bits 12-9) */
	unsigned char mXlow; /* magnetometer X (bits 8-1) */
	unsigned char mYhigh; /* magnetometer Y (bits 12-5) */
	unsigned char mYlow_mZhigh; /* magnetometer: Y (bits 4-1), Z (bits 12-9) */
	unsigned char mZlow; /* magnetometer Z (bits 8-1) */
	unsigned char timelow; /* low byte of timestamp */
	unsigned char extdata[PSMOVE_EXT_DATA_BUF_SIZE]; /* external device data (EXT port) */
};

int PSMoveController::s_nOpened = 0;

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

const boost::property_tree::ptree
PSMoveControllerConfig::config2ptree()
{
	boost::property_tree::ptree pt;
	
	pt.put("Calibration.Accel.X.k", cal_ag_xyz_kb[0][0][0]);
	pt.put("Calibration.Accel.X.b", cal_ag_xyz_kb[0][0][1]);
	pt.put("Calibration.Accel.Y.k", cal_ag_xyz_kb[0][1][0]);
	pt.put("Calibration.Accel.Y.b", cal_ag_xyz_kb[0][1][1]);
	pt.put("Calibration.Accel.Z.k", cal_ag_xyz_kb[0][2][0]);
	pt.put("Calibration.Accel.Z.b", cal_ag_xyz_kb[0][2][1]);
	pt.put("Calibration.Gyro.X.k", cal_ag_xyz_kb[1][0][0]);
	pt.put("Calibration.Gyro.X.b", cal_ag_xyz_kb[1][0][1]);
	pt.put("Calibration.Gyro.Y.k", cal_ag_xyz_kb[1][1][0]);
	pt.put("Calibration.Gyro.Y.b", cal_ag_xyz_kb[1][1][1]);
	pt.put("Calibration.Gyro.Z.k", cal_ag_xyz_kb[1][2][0]);
	pt.put("Calibration.Gyro.Z.b", cal_ag_xyz_kb[1][2][1]);

	return pt;
}

void
PSMoveControllerConfig::ptree2config(const boost::property_tree::ptree &pt)
{
	cal_ag_xyz_kb[0][0][0] = pt.get<float>("Calibration.Accel.X.k", 1.0f);
	cal_ag_xyz_kb[0][0][1] = pt.get<float>("Calibration.Accel.X.b", 0.0f);
	cal_ag_xyz_kb[0][1][0] = pt.get<float>("Calibration.Accel.Y.k", 1.0f);
	cal_ag_xyz_kb[0][1][1] = pt.get<float>("Calibration.Accel.Y.b", 0.0f);
	cal_ag_xyz_kb[0][2][0] = pt.get<float>("Calibration.Accel.Z.k", 1.0f);
	cal_ag_xyz_kb[0][2][1] = pt.get<float>("Calibration.Accel.Z.b", 0.0f);

	cal_ag_xyz_kb[1][0][0] = pt.get<float>("Calibration.Gyro.X.k", 1.0f);
	cal_ag_xyz_kb[1][0][1] = pt.get<float>("Calibration.Gyro.X.b", 0.0f);
	cal_ag_xyz_kb[1][1][0] = pt.get<float>("Calibration.Gyro.Y.k", 1.0f);
	cal_ag_xyz_kb[1][1][1] = pt.get<float>("Calibration.Gyro.Y.b", 0.0f);
	cal_ag_xyz_kb[1][2][0] = pt.get<float>("Calibration.Gyro.Z.k", 1.0f);
	cal_ag_xyz_kb[1][2][1] = pt.get<float>("Calibration.Gyro.Z.b", 0.0f);
}

PSMoveController::PSMoveController(int next_ith)
	: Index(0), LedR(255), LedG(0), LedB(0), Rumble(0)
{
	hid_init();
	HIDDetails.Handle = nullptr;
	HIDDetails.Handle_addr = nullptr;
    
    InData = new PSMoveDataInput;
    InData->type = PSMove_Req_GetInput;

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
            IsBluetooth = !((cur_dev->serial_number == NULL) || (wcslen(cur_dev->serial_number) == 0));
			// On my Mac, using bluetooth,
			// cur_dev->path = Bluetooth_054c_03d5_779732e8
			// cur_dev->serial_number = 00-06-f7-97-32-e8
			// On my Mac, using USB,
			// cur_dev->path = USB_054c_03d5_14100000
			// cur_dev->serial_number = "" (not null, just empty)

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
				Index = count;
				HIDDetails.Device_path = cur_dev->path;
#ifdef _WIN32
				HIDDetails.Device_path_addr = HIDDetails.Device_path;
				HIDDetails.Device_path_addr.replace(HIDDetails.Device_path_addr.find("&col01#"), 7, "&col02#");
				HIDDetails.Device_path_addr.replace(HIDDetails.Device_path_addr.find("&0000#"), 6, "&0001#");
				HIDDetails.Handle_addr = hid_open_path(HIDDetails.Device_path_addr.c_str());
				hid_set_nonblocking(HIDDetails.Handle_addr, 1);
#endif
                HIDDetails.Handle = hid_open_path(HIDDetails.Device_path.c_str());
                hid_set_nonblocking(HIDDetails.Handle, 1);
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
#ifndef _WIN32
            // On my Mac, getting the bt feature report when connected via
            // bt crashes the controller. So we simply copy the serial number.
            // It gets modified in getBTAddress.
            // TODO: Copy this over anyway even in Windows. Check getBTAddress
            // comments for handling windows serial_number.
            // Once done, we can remove the ifndef above.
            std::wstring ws(cur_dev->serial_number);
            std::string mbs(ws.begin(), ws.end());
            HIDDetails.Bt_addr = mbs;
#endif
			std::string host;
			int success = getBTAddress(host, HIDDetails.Bt_addr);
			if (!success)
			{
				// TODO: If serial is still bad, maybe we have a disconnected
                // controller still showing up in hidapi
			}
            
			// Load the config file
			std::string btaddr = HIDDetails.Bt_addr;
			std::replace(btaddr.begin(), btaddr.end(), ':', '_');
			cfg = PSMoveControllerConfig(btaddr);
			cfg.load();

			if (!IsBluetooth)
			{
				// Load calibration from controller internal memory.
				loadCalibration();
			}
            
            
            // TODO: Other startup.
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
		hid_close(HIDDetails.Handle);
		s_nOpened--;
	}
	if ((HIDDetails.Handle_addr != nullptr) && (HIDDetails.Handle_addr != NULL))
    {
        hid_close(HIDDetails.Handle_addr);
    }
	if (s_nOpened == 0)
	{
		hid_exit();
	}
    delete InData;
}

bool
PSMoveController::isOpen()
{
	return ((Index > 0) && (HIDDetails.Handle != nullptr) && (HIDDetails.Handle != NULL));
}

// Getters

int
PSMoveController::getBTAddress(std::string& host, std::string& controller)
{
    int success = 0;

    if (IsBluetooth && !controller.empty())
    {
        std::replace(controller.begin(), controller.end(), '-', ':');
        std::transform(controller.begin(), controller.end(), controller.begin(), ::tolower);
        
        //TODO: If the third entry is not : and length is PSMOVE_BTADDR_SIZE
//        std::stringstream ss;
//        ss << controller.substr(0, 2) << ":" << controller.substr(2, 2) <<
//        ":" << controller.substr(4, 2) << ":" << controller.substr(6, 2) <<
//        ":" << controller.substr(8, 2) << ":" << controller.substr(10, 2);
//        controller = ss.str();
        
        success = 1;
    }
    else
    {
        int res;
        
        unsigned char btg[PSMOVE_BTADDR_GET_SIZE];
        unsigned char ctrl_char_buff[PSMOVE_BTADDR_SIZE];
        unsigned char host_char_buff[PSMOVE_BTADDR_SIZE];

        memset(btg, 0, sizeof(btg));
        btg[0] = PSMove_Req_GetBTAddr;
        /* _WIN32 only has move->handle_addr for getting bluetooth address. */
        if (HIDDetails.Handle_addr) {
            res = hid_get_feature_report(HIDDetails.Handle_addr, btg, sizeof(btg));
        }
        else {
            res = hid_get_feature_report(HIDDetails.Handle, btg, sizeof(btg));
        }

        if (res == sizeof(btg)) {

            memcpy(host_char_buff, btg + 10, PSMOVE_BTADDR_SIZE);
            host = btAddrUcharToString(host_char_buff);

            memcpy(ctrl_char_buff, btg + 1, PSMOVE_BTADDR_SIZE);
            controller = btAddrUcharToString(ctrl_char_buff);

            success = 1;
        }
        else
        {
            const wchar_t* hidapi_err;
            if (HIDDetails.Handle_addr)
            {
                hidapi_err = hid_error(HIDDetails.Handle_addr);
            }
            else
            {
                hidapi_err = hid_error(HIDDetails.Handle);
            }
            if (hidapi_err)
            {
                std::wcout << std::endl;
                std::wcout << hidapi_err << std::endl;
            }
        }
    }
	return success;
}
    
static int
decodeCalibration(char *data, int offset)
{
    unsigned char low = data[offset] & 0xFF;
    unsigned char high = (data[offset+1]) & 0xFF;
    return (low | (high << 8)) - 0x8000;
}
    
void
PSMoveController::loadCalibration()
{
	// The calibration provides a scale factor (k) and offset (b) to convert
	// raw accelerometer and gyroscope readings into something more useful.
	// https://github.com/nitsch/moveonpc/wiki/Calibration-data

	// calibration data storage - loaded from file (if bluetooth) or usb
	char usb_calibration[PSMOVE_CALIBRATION_BLOB_SIZE];

	// Default values are pass-through (raw*1 + 0)
	cfg.cal_ag_xyz_kb = { { { 1, 0 }, { 1, 0 }, { 1, 0 } }, { { 1, 0 }, { 1, 0 }, { 1, 0 } } };

    // Load the calibration from the controller itself.
    unsigned char hid_cal[PSMOVE_CALIBRATION_BLOB_SIZE];
    unsigned char cal[PSMOVE_CALIBRATION_SIZE];
    int res;
    int x;
    int dest_offset;
    int src_offset;
    for (x=0; x<3; x++) {
        memset(cal, 0, sizeof(cal));
        cal[0] = PSMove_Req_GetCalibration;
        res = hid_get_feature_report(HIDDetails.Handle, cal, sizeof(cal));
        if (cal[1] == 0x00) {
            /* First block */
            dest_offset = 0;
            src_offset = 0;
        } else if (cal[1] == 0x01) {
            /* Second block */
            dest_offset = PSMOVE_CALIBRATION_SIZE;
            src_offset = 2;
        } else if (cal[1] == 0x82) {
            /* Third block */
            dest_offset = 2*PSMOVE_CALIBRATION_SIZE - 2;
            src_offset = 2;
        }
        memcpy(hid_cal+dest_offset, cal+src_offset,
                sizeof(cal)-src_offset);
    }
	memcpy(usb_calibration, hid_cal, PSMOVE_CALIBRATION_BLOB_SIZE);
        
    // Convert the calibration blob into constant & offset for each accel dim.
    std::vector< std::vector<int> > dim_lohi = { {1, 3}, {5, 4}, {2, 0} };
    std::vector<int> res_lohi(2, 0);
    int dim_ix = 0;
    int lohi_ix = 0;
    for (dim_ix = 0; dim_ix < 3; dim_ix++)
    {
        for (lohi_ix = 0; lohi_ix < 2; lohi_ix++)
        {
            res_lohi[lohi_ix] = decodeCalibration(usb_calibration, 0x04 + 6*dim_lohi[dim_ix][lohi_ix] + 2*dim_ix);
        }
		cfg.cal_ag_xyz_kb[0][dim_ix][0] = 2.f / (float)(res_lohi[1] - res_lohi[0]);
		cfg.cal_ag_xyz_kb[0][dim_ix][1] = -(cfg.cal_ag_xyz_kb[0][dim_ix][0] * (float)res_lohi[0]) - 1.f;
    }
        
    // Convert the calibration blob into constant for each gyro dim.
    float factor = (float)(2.0 * M_PI * 80.0) / 60.0;
    for (dim_ix = 0; dim_ix < 3; dim_ix++)
    {
		cfg.cal_ag_xyz_kb[1][dim_ix][0] = factor / (float)(decodeCalibration(usb_calibration, 0x46 + 10 * dim_ix)
                                                - decodeCalibration(usb_calibration, 0x2a + 2*dim_ix));
        // No offset for gyroscope
    }

	cfg.save();
}

/* Decode 16-bit signed value from data pointer and offset */
int
psmove_decode_16bit(char *data, int offset)
{
	unsigned char low = data[offset] & 0xFF;
	unsigned char high = (data[offset + 1]) & 0xFF;
	return (low | (high << 8)) - 0x8000;
}

inline enum PSMoveButtonState
getButtonState(unsigned int buttons, unsigned int lastButtons, int buttonMask)
{
    return (enum PSMoveButtonState)((((lastButtons & buttonMask) > 0) << 1) + ((buttons & buttonMask)>0));
}
    
bool
PSMoveController::readDataIn()
{
	bool success = false;

	// TODO: Rate-limiting
		
	int res = hid_read(HIDDetails.Handle, (unsigned char*)InData, sizeof(PSMoveDataInput));
	while (res == sizeof(PSMoveDataInput))
	{
        // https://github.com/nitsch/moveonpc/wiki/Input-report
        
        PSMoveState newState;
        
        // Buttons
		newState.AllButtons = (InData->buttons2) | (InData->buttons1 << 8) |
			((InData->buttons3 & 0x01) << 16) | ((InData->buttons4 & 0xF0) << 13);
        
        unsigned int lastButtons = ControllerStates.empty() ? 0 : ControllerStates.back().AllButtons;

        newState.Triangle = getButtonState(newState.AllButtons, lastButtons, Btn_TRIANGLE);
        newState.Circle = getButtonState(newState.AllButtons, lastButtons, Btn_CIRCLE);
        newState.Cross = getButtonState(newState.AllButtons, lastButtons, Btn_CROSS);
        newState.Square = getButtonState(newState.AllButtons, lastButtons, Btn_SQUARE);
        newState.Select = getButtonState(newState.AllButtons, lastButtons, Btn_SELECT);
        newState.Start = getButtonState(newState.AllButtons, lastButtons, Btn_START);
        newState.PS = getButtonState(newState.AllButtons, lastButtons, Btn_PS);
        newState.Move = getButtonState(newState.AllButtons, lastButtons, Btn_MOVE);
		newState.Trigger = (InData->trigger + InData->trigger2) / 2; // TODO: store each frame separately
        
		// Sensors (Accel/Gyro/Mag)
		char* data = (char *)InData;

        // Do Accel and Gyro together.
        std::vector< std::vector< std::vector<float> > > ag_12_xyz = { { {0, 0, 0}, {0, 0, 0} }, { {0, 0, 0}, {0, 0, 0} } };
        std::vector<int> sensorOffsets = { offsetof(PSMoveDataInput, aXlow),
            offsetof(PSMoveDataInput, gXlow) };
        std::vector<int> frameOffsets = {0, 6};
        std::vector<int>::size_type s_ix, f_ix;
        int d_ix;
        int totalOffset;
        int val;
        
        for (s_ix = 0; s_ix != sensorOffsets.size(); s_ix++) //accel, gyro
        {
            for (f_ix = 0; f_ix != frameOffsets.size(); f_ix++) //old, new
            {
                for (d_ix = 0; d_ix < 3; d_ix++)  //x, y, z
                {
                    totalOffset = sensorOffsets[s_ix] + frameOffsets[f_ix] + 2*d_ix;
                    val = ((data[totalOffset] & 0xFF) | (((data[totalOffset + 1]) & 0xFF) << 8)) - 0x8000;
					ag_12_xyz[s_ix][f_ix][d_ix] = (float)val*cfg.cal_ag_xyz_kb[s_ix][d_ix][0] + cfg.cal_ag_xyz_kb[s_ix][d_ix][1];
                }
            }
        }
        newState.Accel = ag_12_xyz[0];
        newState.Gyro = ag_12_xyz[1];
        
        // Mag
        newState.Mag = {0, 0, 0};
		newState.Mag[0] = TWELVE_BIT_SIGNED(((InData->templow_mXhigh & 0x0F) << 8) | InData->mXlow);
		newState.Mag[1] = TWELVE_BIT_SIGNED((InData->mYhigh << 4) | (InData->mYlow_mZhigh & 0xF0) >> 4);
		newState.Mag[2] = TWELVE_BIT_SIGNED(((InData->mYlow_mZhigh & 0x0F) << 8) | InData->mZlow);

        
        // Other
		newState.Sequence = (InData->buttons4 & 0x0F);
        newState.Battery = (enum PSMoveBatteryLevel)(InData->battery);
        newState.TimeStamp = InData->timelow | (InData->timehigh << 8);
        newState.TempRaw = (InData->temphigh << 4) | ((InData->templow_mXhigh & 0xF0) >> 4);
        
        ControllerStates.push_back(newState);
        
        success = true;
        
        res = hid_read(HIDDetails.Handle, (unsigned char*)InData, sizeof(PSMoveDataInput));
	}
	if (res < 0)
	{
		const wchar_t* hidapi_err = hid_error(HIDDetails.Handle);
        std::wcout << std::endl;
        std::wcout << hidapi_err << std::endl;
	}
    if (ControllerStates.size() > PSMOVE_STATE_BUFFER_MAX)
    {
        ControllerStates.erase(ControllerStates.begin(),
                               ControllerStates.begin()+ControllerStates.size()-PSMOVE_STATE_BUFFER_MAX);
    }

	return success;
}

psmovePosef
PSMoveController::getPose(int msec_time)
{
    psmovePosef nullPose;
    return nullPose;
}

const PSMoveState
PSMoveController::getState(int lookBack)
{
	readDataIn();
    if (ControllerStates.empty())
    {
        PSMoveState emptyState = PSMoveState();
        emptyState.Accel = { {0, 0, 0}, {0, 0, 0} };
        emptyState.Gyro = { {0, 0, 0}, {0, 0, 0} };
        emptyState.Mag = {0, 0, 0};
        return emptyState;  // Returns a null state.
    }
    else
    {
        lookBack = (lookBack < (int)ControllerStates.size()) ? lookBack : ControllerStates.size();
        return ControllerStates.at(ControllerStates.size() - lookBack - 1);
    }
}
    
float
PSMoveController::getTempCelsius()
{
    PSMoveState lastState = getState();
    /**
     * The Move uses this table in Debug mode. Even though the resulting values
     * are not labeled "degree Celsius" in the Debug output, measurements
     * indicate that it is close enough.
     **/
    static int const temperature_lookup[80] = {
        0x1F6, 0x211, 0x22C, 0x249, 0x266, 0x284, 0x2A4, 0x2C4,
        0x2E5, 0x308, 0x32B, 0x34F, 0x374, 0x399, 0x3C0, 0x3E8,
        0x410, 0x439, 0x463, 0x48D, 0x4B8, 0x4E4, 0x510, 0x53D,
        0x56A, 0x598, 0x5C6, 0x5F4, 0x623, 0x651, 0x680, 0x6AF,
        0x6DE, 0x70D, 0x73C, 0x76B, 0x79A, 0x7C9, 0x7F7, 0x825,
        0x853, 0x880, 0x8AD, 0x8D9, 0x905, 0x930, 0x95B, 0x985,
        0x9AF, 0x9D8, 0xA00, 0xA28, 0xA4F, 0xA75, 0xA9B, 0xAC0,
        0xAE4, 0xB07, 0xB2A, 0xB4B, 0xB6D, 0xB8D, 0xBAD, 0xBCB,
        0xBEA, 0xC07, 0xC24, 0xC40, 0xC5B, 0xC75, 0xC8F, 0xCA8,
        0xCC1, 0xCD8, 0xCF0, 0xD06, 0xD1C, 0xD31, 0xD46, 0xD5A,
    };
    
    int i;
    
    for (i = 0; i < 80; i++) {
        if (temperature_lookup[i] > lastState.TempRaw) {
            return (float)(i - 10);
        }
    }
    
    return 70;
}

// Setters

bool
PSMoveController::writeDataOut()
{
    PSMoveDataOutput data_out = PSMoveDataOutput();  // 0-initialized
	data_out.type = PSMove_Req_SetLEDs;
	data_out.rumble2 = 0x00;
    data_out.r = LedR;
    data_out.g = LedG;
    data_out.b = LedB;
    data_out.rumble = Rumble;
    
    int res = hid_write(HIDDetails.Handle, (unsigned char*)(&data_out),
                        sizeof(data_out));
	return (res == sizeof(data_out));
}

bool
PSMoveController::setLED(unsigned char r, unsigned char g, unsigned char b)
{
	bool success = true;
	if ((LedR != r) || (LedG != g) || (LedB != b))
	{
		LedR = r;
		LedG = g;
		LedB = b;
		success = writeDataOut();
	}
	return success;
}

bool
PSMoveController::setRumbleIntensity(unsigned char value)
{
	bool success = true;
	if (Rumble != value)
	{
		Rumble = value;
		success = writeDataOut();
	}
	return success;
}

bool
PSMoveController::setLEDPWMFrequency(unsigned long freq)
{
	bool success = false;
	if ((freq >= 733) && (freq <= 24e6) && (freq != LedPWMF))
	{
		unsigned char buf[7];
		
		memset(buf, 0, sizeof(buf));
		buf[0] = PSMove_Req_SetLEDPWMFrequency;
		buf[1] = 0x41;  /* magic value, report is ignored otherwise */
		buf[2] = 0;     /* command byte, values 1..4 are internal frequency presets */
		/* The 32-bit frequency value must be stored in Little-Endian byte order */
		buf[3] = freq & 0xFF;
		buf[4] = (freq >> 8) & 0xFF;
		buf[5] = (freq >> 16) & 0xFF;
		buf[6] = (freq >> 24) & 0xFF;
		int res = hid_send_feature_report(HIDDetails.Handle, buf, sizeof(buf));
		success = (res == sizeof(buf));
        LedPWMF = freq;
	}
	return success;
}
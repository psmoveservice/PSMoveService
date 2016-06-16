//-- includes -----
#include "PSDualShock4Controller.h"
#include "ControllerDeviceEnumerator.h"
#include "ServerLog.h"
#include "ServerUtility.h"
#include "BluetoothQueries.h"
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
#define PSDS4_INPUT_REPORT_LENGTH 547

#define PSDS4_BTADDR_GET_SIZE 16
#define PSDS4_BTADDR_SET_SIZE 23
#define PSDS4_BTADDR_SIZE 6
#define PSDS4_STATE_BUFFER_MAX 16

#define PSDS4_TRACKING_SHAPE_WIDTH  5.f // The width of the DS4 tracking bar in cm
#define PSDS4_TRACKING_SHAPE_HEIGHT  1.1f // The height of the DS4 tracking bar in cm

#define PSDS4_RUMBLE_ENABLED 0xf3
#define PSDS4_RUMBLE_DISABLED 0xf0

#define PSDS4_CALIBRATION_SIZE 49 /* Buffer size for calibration data */
#define PSDS4_CALIBRATION_BLOB_SIZE (PSDS4_CALIBRATION_SIZE*3 - 2*2) /* Three blocks, minus header (2 bytes) for blocks 2,3 */

/* Minimum time (in milliseconds) psmove write updates */
#define PSDS4_WRITE_DATA_INTERVAL_MS 120

enum ePSDualShock4_RequestType {
    PSDualShock4_BTReport_Input = 0x00,
    PSDualShock4_BTReport_Output = 0x11,
    PSDualShock4_Req_GetCalibration = 0x10,
    PSDualShock4_Req_GetBTAddr = 0x12,
    PSDualShock4_Req_SetBTAddr = 0x13,
};

enum ePSDualShock4_DPad
{
    PSDualShock4DPad_N=         0,
    PSDualShock4DPad_NE=        1,
    PSDualShock4DPad_E=         2,
    PSDualShock4DPad_SE=        3,
    PSDualShock4DPad_S=         4,
    PSDualShock4DPad_SW=        5,
    PSDualShock4DPad_W=         6,
    PSDualShock4DPad_NW=        7,
    PSDualShock4DPad_Released=  8
};

enum ePSDS4_Button {
    // buttons1
    Btn_DPAD_LEFT = 1 << 0,
    Btn_DPAD_RIGHT = 1 << 1,
    Btn_DPAD_UP = 1 << 2,
    Btn_DPAD_DOWN = 1 << 3,
    Btn_SQUARE = 1 << 4,
    Btn_CROSS = 1 << 5,
    Btn_CIRCLE = 1 << 6,
    Btn_TRIANGLE = 1 << 7,

    // buttons2
    Btn_L1 = 1 << 8,
    Btn_R1 = 1 << 9,
    Btn_L2 = 1 << 10,
    Btn_R2 = 1 << 11,
    Btn_SHARE = 1 << 12,
    Btn_OPTION = 1 << 13,
    Btn_L3 = 1 << 14,
    Btn_R3 = 1 << 15,

    // buttons3
    Btn_PS = 1 << 17,
    Btn_TPAD = 1 << 18,
};

// The link key used when binding a bluetooth host address on the DS4
static unsigned char k_ps4_bluetooth_link_key[16] = {
    0x56,
    0xE8,
    0x81,
    0x38,
    0x08,
    0x06,
    0x51,
    0x41,
    0xC0,
    0x7F,
    0x12,
    0xAA,
    0xD9,
    0x66,
    0x3C,
    0xCE
};
static size_t k_ps4_bluetooth_link_key_length = sizeof(k_ps4_bluetooth_link_key);

// -- private definitions -----

// http://eleccelerator.com/wiki/index.php?title=DualShock_4
struct PS4DualShock4TouchPacket
{
    unsigned char packet_counter;           // byte 0

    unsigned char finger1_id : 7;           // byte 1, bit 0-6, id of finger touch 1
    unsigned char finger1_active_low : 1;   // byte 1, bit 7, 1 = false, 0 = true

    unsigned char finger1_coordinates[3];   // byte 2-5,  [Y2_hi|Y2_lo|X2_hi|X2_lo|Y1_hi|Y1_lo|X1_hi|X1_lo]

    unsigned char finger2_id : 7;           // byte 6, bit 0-6, id of finger touch 2
    unsigned char finger2_active_low : 1;   // byte 6, bit 7, 1 = false, 0 = true 

    unsigned char finger2_coordinates[3];   // byte 7-9,  [Y2_hi|Y2_lo|X2_hi|X2_lo|Y1_hi|Y1_lo|X1_hi|X1_lo]
};

// http://eleccelerator.com/wiki/index.php?title=DualShock_4
struct PSDualShock4DataInput
{
    unsigned char hid_report_type : 2;      // byte 0, bit 0-1 (0x01=INPUT)
    unsigned char hid_parameter : 2;        // byte 0, bit 2-3 (0x00)
    unsigned char hid_transaction_type : 4; // byte 0, bit 4-7 (0x0a=DATA)
    unsigned char hid_protocol_code;        // byte 1 (0x11)
    unsigned char hid_unknown;              // byte 2 (0xc2)

    unsigned char report_id;            // byte 3 report ID, must be PSDualShock4_BTReport_Input(0x00)
    unsigned char left_stick_x;         // byte 4, 0 = left
    unsigned char left_stick_y;         // byte 5, 0 = up
    unsigned char right_stick_x;        // byte 6, 0 = left
    unsigned char right_stick_y;        // byte 7, 0 = up

    union
    {
        struct
        {
            unsigned char dpad_enum : 4;        // byte 8, bit 0-3, enum PSDualShock4DPad
            unsigned char btn_square : 1;       // byte 8, bit 4
            unsigned char btn_x : 1;            // byte 8, bit 5
            unsigned char btn_circle : 1;       // byte 8, bit 6
            unsigned char btn_triangle : 1;     // byte 8, bit 7
        } state;
        unsigned char raw;
    } buttons1;

    union
    {
        struct  
        {
            unsigned char btn_l1 : 1;           // byte 9, bit 0
            unsigned char btn_r1 : 1;           // byte 9, bit 1
            unsigned char btn_l2 : 1;           // byte 9, bit 2
            unsigned char btn_r2 : 1;           // byte 9, bit 3
            unsigned char btn_share : 1;        // byte 9, bit 4
            unsigned char btn_option : 1;       // byte 9, bit 5
            unsigned char btn_l3 : 1;           // byte 9, bit 6
            unsigned char btn_r3 : 1;           // byte 9, bit 7
        } state;
        unsigned char raw;
    } buttons2;

    union
    {
        struct
        {
            unsigned char btn_ps : 1;           // byte 10, bit 0
            unsigned char btn_tpad : 1;         // byte 10, bit 1
            unsigned char counter : 6;          // byte 10, bit 2-7
        } state;
        unsigned char raw;
    } buttons3;

    unsigned char left_trigger;         // byte 11, 0 = release, 0xff = fully pressed
    unsigned char right_trigger;        // byte 12, 0 = release, 0xff = fully pressed

    // A common increment value between two reports is 188 (at full rate the report period is 1.25ms). 
    // This timestamp is used by the PS4 to process acceleration and gyroscope data.
    unsigned short timestamp;           // byte 13-14

    unsigned char battery;              // byte 15, 0x00 to 0xff

    unsigned char accel_y[2];           // byte 16-17, 12-bit signed x-accelerometer reading
    unsigned char accel_x[2];           // byte 18-19, 12-bit signed y-accelerometer reading
    unsigned char accel_z[2];           // byte 20-21, 12-bit signed z-accelerometer reading
    unsigned char gyro_x[2];            // byte 22-23, 16-bit signed x-gyroscope reading
    unsigned char gyro_y[2];            // byte 24-25, 16-bit signed x-gyroscope reading
    unsigned char gyro_z[2];            // byte 26-27, 16-bit signed x-gyroscope reading

    unsigned char _unknown1[5];         // byte 28-32, Unknown (seems to be always 0x00)

    unsigned char batteryLevel : 4;     // byte 33, bit 0-3, enum BatteryLevel?
    unsigned char usb : 1;              // byte 33, bit 4, is usb connected?
    unsigned char mic : 1;              // byte 33, bit 5, is mic connected
    unsigned char phone : 1;            // byte 33, bit 6, is paired with phone?
    unsigned char _zeroPad : 1;         // byte 33, bit 7, always zero

    unsigned char _unknown2[2];         // byte 34-35, Unknown (seems to be always 0x00)

    unsigned char trackPadPktCount;     // byte 36, Number of trackpad packets (0x00 to 0x04)
    PS4DualShock4TouchPacket trackPadPackets[4];    // byte 37-72, 4 tracker packets (4*9bytes)

    unsigned char _unknown3[2];         // byte 73-74, Unknown 0x00 0x00 or 0x00 0x01
    unsigned char crc32[4];             // byte 75-78, CRC-32 of the first 75 bytes
};

// 78 bytes
struct PSDualShock4DataOutput 
{
    unsigned char hid_report_type : 2;      // byte 0, bit 0-1 (0x02=OUTPUT)
    unsigned char hid_parameter : 2;        // byte 0, bit 2-3 (0x00)
    unsigned char hid_transaction_type : 4; // byte 0, bit 4-7 (0x0a=DATA)
    unsigned char hid_protocol_code;        // byte 1 (0x11)

    unsigned char _unknown1[2];             // byte 2-3, must be: [0x00|0xff]
    unsigned char rumbleFlags;              // byte 4, 	0xf0 disables the rumble motors, 0xf3 enables them
    unsigned char _unknown2[2];             // byte 5-6
    unsigned char rumble_right;             // byte 7, 0x00 to 0xff
    unsigned char rumble_left;              // byte 8, 0x00 to 0xff
    unsigned char led_r;                    // byte 9, red value, 0x00 to 0xff 
    unsigned char led_g;                    // byte 10, green value, 0x00 to 0xff
    unsigned char led_b;                    // byte 11, blue value, 0x00 to 0xff
    unsigned char led_flash_on;             // byte 12, flash on duration
    unsigned char led_flash_off;            // byte 13, flash off duration
    unsigned char _unknown3[8];             // byte 14-21
    unsigned char volume_left;              // byte 22
    unsigned char volume_right;             // byte 23
    unsigned char volume_mic;               // byte 24
    unsigned char volume_speaker;           // byte 25
    unsigned char _unknown4[49];            // byte 26-74
    unsigned char crc32[4];                 // byte 75-78, CRC-32 of the first 75 bytes
};

// -- private prototypes -----
static std::string btAddrUcharToString(const unsigned char* addr_buff);
static bool stringToBTAddrUchar(const std::string &addr, unsigned char *addr_buff, const int addr_buf_size);
static int decodeCalibration(char *data, int offset);
inline enum CommonControllerState::ButtonState getButtonState(unsigned int buttons, unsigned int lastButtons, int buttonMask);
inline bool hid_error_mbs(hid_device *dev, char *out_mb_error, size_t mb_buffer_size);

// -- public methods

// -- PSMove Controller Config
// Bump this version when you are making a breaking config change.
// Simply adding or removing a field is ok and doesn't require a version bump.
const int PSDualShock4ControllerConfig::CONFIG_VERSION = 2;

const boost::property_tree::ptree
PSDualShock4ControllerConfig::config2ptree()
{
    boost::property_tree::ptree pt;

    pt.put("is_valid", is_valid);
    pt.put("version", PSDualShock4ControllerConfig::CONFIG_VERSION);

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

    pt.put("prediction_time", prediction_time);
    pt.put("max_poll_failure_count", max_poll_failure_count);

    return pt;
}

void
PSDualShock4ControllerConfig::ptree2config(const boost::property_tree::ptree &pt)
{
    version = pt.get<int>("version", 0);

    if (version == PSDualShock4ControllerConfig::CONFIG_VERSION)
    {
        is_valid = pt.get<bool>("is_valid", false);
        prediction_time = pt.get<float>("prediction_time", 0.f);
        max_poll_failure_count = pt.get<long>("max_poll_failure_count", 100);

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
    else
    {
        SERVER_LOG_WARNING("PSDualShock4ControllerConfig") <<
            "Config version " << version << " does not match expected version " <<
            PSDualShock4ControllerConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

// -- PSMove Controller -----
PSDualShock4Controller::PSDualShock4Controller()
    : LedR(0)
    , LedG(0)
    , LedB(0)
    , RumbleRight(0)
    , RumbleLeft(0)
    , LedPWMF(0)
    , bWriteStateDirty(false)
    , NextPollSequenceNumber(0)
{
    HIDDetails.Handle = nullptr;
    HIDDetails.Handle_addr = nullptr;

    InData = new PSDualShock4DataInput;
    memset(InData, 0, sizeof(PSDualShock4DataInput));
    InData->hid_report_type= 0x01; // INPUT
    InData->hid_parameter= 0x00;
    InData->hid_transaction_type= 0x0a; // DATA
    InData->hid_protocol_code= 0x11;
    InData->hid_unknown= 0xc2;

    OutData = new PSDualShock4DataOutput;
    memset(OutData, 0, sizeof(PSDualShock4DataOutput));
    OutData->hid_report_type= 0x02; // OUTPUT
    OutData->hid_parameter= 0x00;
    OutData->hid_transaction_type= 0x0a; //DATA
    OutData->hid_protocol_code= 0x11;
    OutData->_unknown1[0]= 0x80; // Unknown why this this is needed, copied from DS4Windows
    OutData->_unknown1[1] = 0x00;
    OutData->rumbleFlags = PSDS4_RUMBLE_ENABLED;

    // Make sure there is an initial empty state in the tracker queue
    {
        PSDualShock4ControllerState empty_state;

        empty_state.clear();
        ControllerStates.push_back(empty_state);
    }
}

PSDualShock4Controller::~PSDualShock4Controller()
{
    if (getIsOpen())
    {
        SERVER_LOG_ERROR("~PSDualShock4Controller") << "Controller deleted without calling close() first!";
    }

    delete InData;
}

bool PSDualShock4Controller::open()
{
    ControllerDeviceEnumerator enumerator(CommonControllerState::PSMove);
    bool success = false;

    if (enumerator.is_valid())
    {
        success = open(&enumerator);
    }

    return success;
}

bool PSDualShock4Controller::open(
    const DeviceEnumerator *enumerator)
{
    const ControllerDeviceEnumerator *pEnum = static_cast<const ControllerDeviceEnumerator *>(enumerator);

    const char *cur_dev_path = pEnum->get_path();
    bool success = false;

    if (getIsOpen())
    {
        SERVER_LOG_WARNING("PSDualShock4Controller::open") << "PSDualShock4Controller(" << cur_dev_path << ") already open. Ignoring request.";
        success = true;
    }
    else
    {
        char cur_dev_serial_number[256];

        SERVER_LOG_INFO("PSDualShock4Controller::open") << "Opening PSDualShock4Controller(" << cur_dev_path << ")";

        if (pEnum->get_serial_number(cur_dev_serial_number, sizeof(cur_dev_serial_number)))
        {
            SERVER_LOG_INFO("PSDualShock4Controller::open") << "  with serial_number: " << cur_dev_serial_number;
        }
        else
        {
            cur_dev_serial_number[0] = '\0';
            SERVER_LOG_INFO("PSDualShock4Controller::open") << "  with EMPTY serial_number";
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
        // cur_dev->serial_number = (null)
        IsBluetooth = (strlen(cur_dev_serial_number) > 0);

        if (getIsOpen())  // Controller was opened and has an index
        {
            // Get the bluetooth address
#ifdef __APPLE__
            // On my Mac, getting the bt feature report when connected via
            // bt crashes the controller. So we simply copy the serial number.
            // It gets modified in getBTAddress.
            // TODO: Copy this over anyway even in Windows. Check getBTAddress
            // comments for handling windows serial_number.
            // Once done, we can remove the ifndef above.
            std::string mbs(cur_dev_serial_number);
            HIDDetails.Bt_addr = mbs;

            if (!bluetooth_get_host_address(HIDDetails.Host_bt_addr))
            {
                HIDDetails.Host_bt_addr = "00:00:00:00:00:00";
            }
#endif
            if (getBTAddress(HIDDetails.Host_bt_addr, HIDDetails.Bt_addr))
            {
                // Load the config file
                std::string btaddr = HIDDetails.Bt_addr;
                std::replace(btaddr.begin(), btaddr.end(), ':', '_');
                cfg = PSDualShock4ControllerConfig(btaddr);
                cfg.load();

                if (!IsBluetooth || !cfg.is_valid)
                {
                    if (!cfg.is_valid)
                    {
                        SERVER_LOG_ERROR("PSDualShock4Controller::open") << "PSDualShock4Controller(" << cur_dev_path << ") has invalid calibration. Reloading.";
                    }

                    // Load calibration from controller internal memory.
                    loadCalibration();
                }

                success = true;
            }
            else
            {
                // If serial is still bad, maybe we have a disconnected
                // controller still showing up in hidapi
                SERVER_LOG_ERROR("PSDualShock4Controller::open") << "Failed to get bluetooth address of PSDualShock4Controller(" << cur_dev_path << ")";
                success = false;
            }

            // Reset the polling sequence counter
            NextPollSequenceNumber = 0;
        }
        else
        {
            SERVER_LOG_ERROR("PSDualShock4Controller::open") << "Failed to open PSDualShock4Controller(" << cur_dev_path << ")";
            success = false;
        }
    }

    return success;
}

void PSDualShock4Controller::close()
{
    if (getIsOpen())
    {
        SERVER_LOG_INFO("PSDualShock4Controller::close") << "Closing PSDualShock4Controller(" << HIDDetails.Device_path << ")";

        if (HIDDetails.Handle != nullptr)
        {
            hid_close(HIDDetails.Handle);
            HIDDetails.Handle = nullptr;
        }

        if (HIDDetails.Handle_addr != nullptr)
        {
            hid_close(HIDDetails.Handle_addr);
            HIDDetails.Handle_addr = nullptr;
        }
    }
    else
    {
        SERVER_LOG_INFO("PSDualShock4Controller::close") << "PSDualShock4Controller(" << HIDDetails.Device_path << ") already closed. Ignoring request.";
    }
}

bool
PSDualShock4Controller::setHostBluetoothAddress(const std::string &new_host_bt_addr)
{
    bool success = false;
    unsigned char bts[PSDS4_BTADDR_SET_SIZE];

    memset(bts, 0, sizeof(bts));
    bts[0] = PSMove_Req_SetBTAddr;

    unsigned char addr[6];
    if (stringToBTAddrUchar(new_host_bt_addr, addr, sizeof(addr)))
    {
        int res;

        // Copy 6 bytes from addr into bts[1]..bts[6]
        memcpy(&bts[1], addr, sizeof(addr));

        // Copy the bluetooth link key index backwards into the bluetooth report
        for (size_t key_byte_index = 0; key_byte_index < k_ps4_bluetooth_link_key_length; ++key_byte_index)
        {
            bts[7 + key_byte_index] = k_ps4_bluetooth_link_key[k_ps4_bluetooth_link_key_length - key_byte_index - 1];
        }

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
            success = true;
        }
        else
        {
            char hidapi_err_mbs[256];
            bool valid_error_mesg = false;

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
                SERVER_LOG_ERROR("PSDualShock4Controller::setBTAddress") << "HID ERROR: " << hidapi_err_mbs;
            }
        }
    }
    else
    {
        SERVER_LOG_ERROR("PSDualShock4Controller::setBTAddress") << "Malformed address: " << new_host_bt_addr;
    }

    return success;
}

// Getters
bool
PSDualShock4Controller::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    // Down-cast the enumerator so we can use the correct get_path.
    const ControllerDeviceEnumerator *pEnum = static_cast<const ControllerDeviceEnumerator *>(enumerator);

    bool matches = false;

    if (pEnum->get_device_type() == CommonControllerState::PSMove)
    {
        const char *enumerator_path = pEnum->get_path();
        const char *dev_path = HIDDetails.Device_path.c_str();

#ifdef _WIN32
        matches = _stricmp(dev_path, enumerator_path) == 0;
#else
        matches = strcmp(dev_path, enumerator_path) == 0;
#endif
    }

    return matches;
}

bool
PSDualShock4Controller::getIsBluetooth() const
{
    return IsBluetooth;
}

bool
PSDualShock4Controller::getIsReadyToPoll() const
{
    return (getIsOpen() && getIsBluetooth());
}

std::string
PSDualShock4Controller::getUSBDevicePath() const
{
    return HIDDetails.Device_path;
}

std::string
PSDualShock4Controller::getSerial() const
{
    return HIDDetails.Bt_addr;
}

std::string
PSDualShock4Controller::getAssignedHostBluetoothAddress() const
{
    return HIDDetails.Host_bt_addr;
}

bool
PSDualShock4Controller::getIsOpen() const
{
    return (HIDDetails.Handle != nullptr);
}

CommonDeviceState::eDeviceType
PSDualShock4Controller::getDeviceType() const
{
    return CommonDeviceState::PSMove;
}

bool
PSDualShock4Controller::getBTAddress(std::string& host, std::string& controller)
{
    bool success = false;

    if (IsBluetooth && !controller.empty() && !host.empty())
    {
        std::replace(controller.begin(), controller.end(), '-', ':');
        std::transform(controller.begin(), controller.end(), controller.begin(), ::tolower);

        std::replace(host.begin(), host.end(), '-', ':');
        std::transform(host.begin(), host.end(), host.begin(), ::tolower);

        //TODO: If the third entry is not : and length is PSDS4_BTADDR_SIZE
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

        unsigned char btg[PSDS4_BTADDR_GET_SIZE];
        unsigned char ctrl_char_buff[PSDS4_BTADDR_SIZE];
        unsigned char host_char_buff[PSDS4_BTADDR_SIZE];

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

            memcpy(host_char_buff, btg + 10, PSDS4_BTADDR_SIZE);
            host = btAddrUcharToString(host_char_buff);

            memcpy(ctrl_char_buff, btg + 1, PSDS4_BTADDR_SIZE);
            controller = btAddrUcharToString(ctrl_char_buff);

            success = true;
        }
        else
        {
            char hidapi_err_mbs[256];
            bool valid_error_mesg = false;

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
                SERVER_LOG_ERROR("PSDualShock4Controller::getBTAddress") << "HID ERROR: " << hidapi_err_mbs;
            }
        }
    }

    return success;
}

void
PSDualShock4Controller::loadCalibration()
{
    bool is_valid = true;

    // The calibration provides a scale factor (k) and offset (b) to convert
    // raw accelerometer and gyroscope readings into something more useful.
    // https://github.com/nitsch/moveonpc/wiki/Calibration-data

    // calibration data storage - loaded from file (if bluetooth) or usb
    char usb_calibration[PSDS4_CALIBRATION_BLOB_SIZE];

    // Default values are pass-through (raw*1 + 0)
    cfg.cal_ag_xyz_kb = { { { 1, 0 }, { 1, 0 }, { 1, 0 } }, { { 1, 0 }, { 1, 0 }, { 1, 0 } } };

    // Load the calibration from the controller itself.
    unsigned char hid_cal[PSDS4_CALIBRATION_BLOB_SIZE];

    for (int block_index = 0; is_valid && block_index<3; block_index++)
    {
        unsigned char cal[PSDS4_CALIBRATION_SIZE];
        int dest_offset;
        int src_offset;

        memset(cal, 0, sizeof(cal));
        cal[0] = PSDualShock4_Req_GetCalibration;

        int res = hid_get_feature_report(HIDDetails.Handle, cal, sizeof(cal));

        if (res == PSDS4_CALIBRATION_SIZE)
        {
            if (cal[1] == 0x00)
            {
                /* First block */
                dest_offset = 0;
                src_offset = 0;
            }
            else if (cal[1] == 0x01)
            {
                /* Second block */
                dest_offset = PSDS4_CALIBRATION_SIZE;
                src_offset = 2;
            }
            else if (cal[1] == 0x82)
            {
                /* Third block */
                dest_offset = 2 * PSDS4_CALIBRATION_SIZE - 2;
                src_offset = 2;
            }
            else
            {
                SERVER_LOG_ERROR("PSMoveController::loadCalibration")
                    << "Unexpected calibration block id(0x" << std::hex << std::setfill('0') << std::setw(2) << cal[1]
                    << " on block #" << block_index;
                is_valid = false;
            }
        }
        else
        {
            char hidapi_err_mbs[256];
            bool valid_error_mesg = hid_error_mbs(HIDDetails.Handle, hidapi_err_mbs, sizeof(hidapi_err_mbs));

            // Device no longer in valid state.
            if (valid_error_mesg)
            {
                SERVER_LOG_ERROR("PSMoveController::loadCalibration") << "HID ERROR: " << hidapi_err_mbs;
            }

            is_valid = false;
        }

        if (is_valid)
        {
            memcpy(hid_cal + dest_offset, cal + src_offset, sizeof(cal) - src_offset);
        }
    }

    if (is_valid)
    {
        memcpy(usb_calibration, hid_cal, PSDS4_CALIBRATION_BLOB_SIZE);

        // Convert the calibration blob into constant & offset for each accel dim.
        std::vector< std::vector<int> > dim_lohi = { { 1, 3 }, { 5, 4 }, { 2, 0 } };
        std::vector<int> res_lohi(2, 0);
        int dim_ix = 0;
        int lohi_ix = 0;
        for (dim_ix = 0; dim_ix < 3; dim_ix++)
        {
            for (lohi_ix = 0; lohi_ix < 2; lohi_ix++)
            {
                res_lohi[lohi_ix] = decodeCalibration(usb_calibration, 0x04 + 6 * dim_lohi[dim_ix][lohi_ix] + 2 * dim_ix);
            }
            cfg.cal_ag_xyz_kb[0][dim_ix][0] = 2.f / (float)(res_lohi[1] - res_lohi[0]);
            cfg.cal_ag_xyz_kb[0][dim_ix][1] = -(cfg.cal_ag_xyz_kb[0][dim_ix][0] * (float)res_lohi[0]) - 1.f;
        }

        // Convert the calibration blob into constant for each gyro dim.
        float factor = (float)(2.0 * M_PI * 80.0) / 60.0f;
        for (dim_ix = 0; dim_ix < 3; dim_ix++)
        {
            cfg.cal_ag_xyz_kb[1][dim_ix][0] = factor / (float)(decodeCalibration(usb_calibration, 0x46 + 10 * dim_ix)
                - decodeCalibration(usb_calibration, 0x2a + 2 * dim_ix));
            // No offset for gyroscope
        }
    }

    cfg.is_valid = is_valid;
    cfg.save();
}

IControllerInterface::ePollResult
PSDualShock4Controller::poll()
{
    IControllerInterface::ePollResult result = IControllerInterface::_PollResultFailure;

    if (!getIsBluetooth())
    {
        // Don't bother polling when connected via usb
        result = IControllerInterface::_PollResultSuccessNoData;
    }
    else if (getIsOpen())
    {
        static const int k_max_iterations = 32;

        for (int iteration = 0; iteration < k_max_iterations; ++iteration)
        {
            // Attempt to read the next update packet from the controller
            int res = hid_read(HIDDetails.Handle, (unsigned char*)InData, sizeof(PSDualShock4DataInput));

            if (res == 0)
            {
                // Device still in valid state
                result = (iteration == 0)
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
                    SERVER_LOG_ERROR("PSDualShock4Controller::readDataIn") << "HID ERROR: " << hidapi_err_mbs;
                }
                result = IControllerInterface::_PollResultFailure;

                // No more data available. Stop iterating.
                break;
            }
            else
            {
                // New data available. Keep iterating.
                result = IControllerInterface::_PollResultSuccessNewData;
            }

            // https://github.com/nitsch/moveonpc/wiki/Input-report
            PSDualShock4ControllerState newState;

            // Increment the sequence for every new polling packet
            newState.PollSequenceNumber = NextPollSequenceNumber;
            ++NextPollSequenceNumber;

            // Smush the button state into one unsigned 32-bit variable
            newState.AllButtons = 
                (((unsigned int)InData->buttons3 & 0x3) << 16) | // Get the 1st two bits of buttons: [0|0|0|0|0|0|PS|TPad]
                (unsigned int)(InData->buttons2 << 8) | // [R3|L3|Option|Share|R2|L2|R1|L1]
                ((unsigned int)InData->buttons1 & 0xF8); // Mask out the dpad enum (1st four bits): [tri|cir|x|sq|0|0|0|0]

            // Converts the dpad enum to independent bit flags
            {
                ePSDualShock4_DPad dpad_enum = static_cast<ePSDualShock4_DPad>(InData->buttons1 & 0xF);
                unsigned int dpad_bits= 0;

                switch (dpad_enum)
                {
                case ePSDualShock4_DPad::PSDualShock4DPad_N:
                    dpad_bits |= ePSDS4_Button::Btn_DPAD_UP;
                    break;
                case ePSDualShock4_DPad::PSDualShock4DPad_NE:
                    dpad_bits |= ePSDS4_Button::Btn_DPAD_UP;
                    dpad_bits |= ePSDS4_Button::Btn_DPAD_RIGHT;
                    break;
                case ePSDualShock4_DPad::PSDualShock4DPad_E:
                    dpad_bits |= ePSDS4_Button::Btn_DPAD_RIGHT;
                    break;
                case ePSDualShock4_DPad::PSDualShock4DPad_SE:
                    dpad_bits |= ePSDS4_Button::Btn_DPAD_DOWN;
                    dpad_bits |= ePSDS4_Button::Btn_DPAD_RIGHT;
                    break;
                case ePSDualShock4_DPad::PSDualShock4DPad_S:
                    dpad_bits |= ePSDS4_Button::Btn_DPAD_DOWN;
                    break;
                case ePSDualShock4_DPad::PSDualShock4DPad_SW:
                    dpad_bits |= ePSDS4_Button::Btn_DPAD_DOWN;
                    dpad_bits |= ePSDS4_Button::Btn_DPAD_LEFT;
                    break;
                case ePSDualShock4_DPad::PSDualShock4DPad_W:
                    dpad_bits |= ePSDS4_Button::Btn_DPAD_LEFT;
                    break;
                case ePSDualShock4_DPad::PSDualShock4DPad_NW:
                    dpad_bits |= ePSDS4_Button::Btn_DPAD_UP;
                    dpad_bits |= ePSDS4_Button::Btn_DPAD_LEFT;
                    break;
                }

                // Append in the DPad bits
                newState.AllButtons |= (dpad_bits & 0xf);
            }

            // Update the button state enum
            {
                unsigned int lastButtons = ControllerStates.empty() ? 0 : ControllerStates.back().AllButtons;

                newState.DPad_Up = getButtonState(newState.AllButtons, lastButtons, Btn_DPAD_UP);
                newState.DPad_Down = getButtonState(newState.AllButtons, lastButtons, Btn_DPAD_DOWN);
                newState.DPad_Left = getButtonState(newState.AllButtons, lastButtons, Btn_DPAD_LEFT);
                newState.DPad_Right = getButtonState(newState.AllButtons, lastButtons, Btn_DPAD_RIGHT);
                newState.Square = getButtonState(newState.AllButtons, lastButtons, Btn_SQUARE);
                newState.Cross = getButtonState(newState.AllButtons, lastButtons, Btn_CROSS);
                newState.Circle = getButtonState(newState.AllButtons, lastButtons, Btn_CIRCLE);
                newState.Triangle = getButtonState(newState.AllButtons, lastButtons, Btn_TRIANGLE);

                newState.L1 = getButtonState(newState.AllButtons, lastButtons, Btn_L1);
                newState.R1 = getButtonState(newState.AllButtons, lastButtons, Btn_R1);
                newState.L2 = getButtonState(newState.AllButtons, lastButtons, Btn_L2);
                newState.R2 = getButtonState(newState.AllButtons, lastButtons, Btn_R2);
                newState.Share = getButtonState(newState.AllButtons, lastButtons, Btn_SHARE);
                newState.Options = getButtonState(newState.AllButtons, lastButtons, Btn_OPTION);
                newState.L3 = getButtonState(newState.AllButtons, lastButtons, Btn_L3);
                newState.R3 = getButtonState(newState.AllButtons, lastButtons, Btn_R3);


                newState.PS = getButtonState(newState.AllButtons, lastButtons, Btn_PS);
                newState.TrackPadButton = getButtonState(newState.AllButtons, lastButtons, Btn_TPAD);
            }

            // Remap the analog sticks from [0,255] -> [-1.f,1.f]
            newState.LeftAnalogX= ((static_cast<float>(InData->left_stick_x) / 255.f) - 0.5f) * 2.f;
            newState.LeftAnalogY = ((static_cast<float>(InData->left_stick_y) / 255.f) - 0.5f) * 2.f;
            newState.RightAnalogX = ((static_cast<float>(InData->right_stick_x) / 255.f) - 0.5f) * 2.f;
            newState.RightAnalogY = ((static_cast<float>(InData->right_stick_y) / 255.f) - 0.5f) * 2.f;

            // Remap the analog triggers from [0,255] -> [0.f,1.f]
            newState.LeftTrigger = static_cast<float>(InData->left_trigger) / 255.f;
            newState.RightTrigger = static_cast<float>(InData->right_trigger) / 255.f;

            // Processes the IMU data
            {
                // Piece together the 12-bit accelerometer data
                short raw_accelX = (short)((unsigned short)(InData->accel_x[0] << 8) | InData->accel_x[1]);
                short raw_accelY = (short)((unsigned short)(InData->accel_y[0] << 8) | InData->accel_y[1]);
                short raw_accelZ = (short)((unsigned short)(InData->accel_z[0] << 8) | InData->accel_z[1]);

                // Piece together the 16-bit gyroscope data
                short raw_gyroX = (short)((unsigned short)(InData->gyro_x[0] << 8) | InData->gyro_x[1]);
                short raw_gyroY = (short)((unsigned short)(InData->gyro_y[0] << 8) | InData->gyro_y[1]);
                short raw_gyroZ = (short)((unsigned short)(InData->gyro_z[0] << 8) | InData->gyro_z[1]);

                // TODO: Convert to g/s^2
                newState.Accel.i = (float)raw_accelX;
                newState.Accel.j = (float)raw_accelY;
                newState.Accel.k = (float)raw_accelZ;

                // TODO: Convert to rad/s
                newState.Gyro.i = (float)raw_gyroX;
                newState.Gyro.j = (float)raw_gyroY;
                newState.Gyro.k = (float)raw_gyroZ;
            }

            // Sequence and timestamp
            newState.RawSequence = InData->buttons3.state.counter;
            newState.RawTimeStamp = InData->timestamp;

            // Convert the 0-10 battery level into the batter level
            switch (InData->batteryLevel)
            {
            case 0:
                newState.Battery = CommonControllerState::BatteryLevel::Batt_CHARGING;
            case 1:
                newState.Battery = CommonControllerState::BatteryLevel::Batt_MIN;
            case 2:
            case 3:
                newState.Battery = CommonControllerState::BatteryLevel::Batt_20Percent;
            case 4:
            case 5:
                newState.Battery = CommonControllerState::BatteryLevel::Batt_40Percent;
            case 6:
            case 7:
                newState.Battery = CommonControllerState::BatteryLevel::Batt_60Percent;
            case 8:
            case 9:
                newState.Battery = CommonControllerState::BatteryLevel::Batt_80Percent;
            case 10:
            default:
                newState.Battery = CommonControllerState::BatteryLevel::Batt_MAX;
                break;
            }            

            // Make room for new entry if at the max queue size
            if (ControllerStates.size() >= PSDS4_STATE_BUFFER_MAX)
            {
                ControllerStates.erase(ControllerStates.begin(),
                    ControllerStates.begin() + ControllerStates.size() - PSDS4_STATE_BUFFER_MAX);
            }

            ControllerStates.push_back(newState);
        }

        // Update recurrent writes on a regular interval
        {
            std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();

            // See if it's time to update the LED/rumble state
            std::chrono::duration<double, std::milli> led_update_diff = now - lastWriteStateTime;
            if (led_update_diff.count() >= PSDS4_WRITE_DATA_INTERVAL_MS)
            {
                writeDataOut();
                lastWriteStateTime = now;
            }
        }
    }

    return result;
}

const CommonDeviceState *
PSDualShock4Controller::getState(
int lookBack) const
{
    const int queueSize = static_cast<int>(ControllerStates.size());
    const CommonDeviceState * result =
        (lookBack < queueSize) ? &ControllerStates.at(queueSize - lookBack - 1) : nullptr;

    return result;
}

const std::tuple<unsigned char, unsigned char, unsigned char>
PSDualShock4Controller::getColour() const
{
    return std::make_tuple(LedR, LedG, LedB);
}

void
PSDualShock4Controller::getTrackingShape(CommonDeviceTrackingShape &outTrackingShape) const
{
    outTrackingShape.shape_type = eCommonTrackingShapeType::PlanarBlob;
    outTrackingShape.shape.planar_blob.width = PSDS4_TRACKING_SHAPE_WIDTH;
    outTrackingShape.shape.planar_blob.height = PSDS4_TRACKING_SHAPE_HEIGHT;
}

long PSDualShock4Controller::getMaxPollFailureCount() const
{
    return cfg.max_poll_failure_count;
}

// Setters

bool
PSDualShock4Controller::writeDataOut()
{
    OutData->led_r = LedR;
    OutData->led_g = LedG;
    OutData->led_b = LedB;
    OutData->led_flash_on= LedOnDuration;
    OutData->led_flash_off = LedOffDuration;
    OutData->rumble_right = RumbleRight;
    OutData->rumble_left = RumbleLeft;

    // Keep writing state out until the desired LED and Rumble are 0 
    bWriteStateDirty = 
        LedR != 0 || LedG != 0 || LedB != 0 || 
        RumbleRight != 0 || RumbleLeft != 0;

    int res = hid_write(HIDDetails.Handle, (unsigned char*)(&OutData), sizeof(OutData));
    return (res == sizeof(OutData));
}

bool
PSDualShock4Controller::setLED(unsigned char r, unsigned char g, unsigned char b)
{
    bool success = true;
    if ((LedR != r) || (LedG != g) || (LedB != b))
    {
        LedR = r;
        LedG = g;
        LedB = b;
        bWriteStateDirty = true;
        success = writeDataOut();
    }
    return success;
}

bool
PSDualShock4Controller::setRumbleIntensity(unsigned char value)
{
    bool success = true;
    if (Rumble != value)
    {
        Rumble = value;
        bWriteStateDirty = true;
        success = writeDataOut();
    }
    return success;
}

// -- private helper functions -----
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

static bool
stringToBTAddrUchar(const std::string &addr, unsigned char *addr_buff, const int addr_buf_size)
{
    bool success = false;

    if (addr.length() >= 17 && addr_buf_size >= 6)
    {
        const char *raw_string = addr.c_str();
        unsigned int octets[6];

        success =
            sscanf(raw_string, "%x:%x:%x:%x:%x:%x",
            &octets[5],
            &octets[4],
            &octets[3],
            &octets[2],
            &octets[1],
            &octets[0]) == 6;
        //TODO: Make safe (sscanf_s is not portable)

        if (success)
        {
            for (int i = 0; i < 6; ++i)
            {
                addr_buff[i] = ServerUtility::int32_to_int8_verify(octets[i]);
            }
        }
    }

    return success;
}

static int
decodeCalibration(char *data, int offset)
{
    unsigned char low = data[offset] & 0xFF;
    unsigned char high = (data[offset + 1]) & 0xFF;
    return (low | (high << 8)) - 0x8000;
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
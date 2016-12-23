//-- includes -----
#include "PSDualShock4Controller.h"
#include "ControllerDeviceEnumerator.h"
#include "ControllerManager.h"
#include "DeviceManager.h"
#include "MathUtility.h"
#include "ServerLog.h"
#include "ServerUtility.h"
#include "BluetoothQueries.h"
#include <algorithm>
#include <vector>
#include <cstdlib>
#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif
#include <math.h>

#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': strcpy, strdup, sprintf, vsnprintf, sscanf, fopen
#endif

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <Hidsdi.h> // for HidD_SetOutputReport
#endif

//-- constants -----
#define PSDS4_INPUT_REPORT_LENGTH 547

#define PSDS4_BTADDR_GET_SIZE 16
#define PSDS4_BTADDR_SET_SIZE 23
#define PSDS4_BTADDR_SIZE 6
#define PSDS4_STATE_BUFFER_MAX 16

#define PSDS4_TRACKING_TRIANGLE_WIDTH  .9386f // The width of a triangle enclosed in the DS4 tracking bar in cm
#define PSDS4_TRACKING_TRIANGLE_HEIGHT  .6548f // The height of a triangle enclosed in the DS4 tracking bar in cm

#define PSDS4_TRACKING_QUAD_WIDTH  5.2f // The width of a quad that encloses the DS4 tracking bar in cm
#define PSDS4_TRACKING_QUAD_HEIGHT  1.1f // The height of a quad that encloses the DS4 tracking bar in cm

#define PSDS4_TRACKING_SHAPE_PITCH  30.f // How much the DS4 light bar is tilted inward in degrees

#define PSDS4_RUMBLE_ENABLED 0xff
#define PSDS4_RUMBLE_DISABLED 0xf0

#define PSDS4_CALIBRATION_SIZE 49 /* Buffer size for calibration data */
#define PSDS4_CALIBRATION_BLOB_SIZE (PSDS4_CALIBRATION_SIZE*3 - 2*2) /* Three blocks, minus header (2 bytes) for blocks 2,3 */

/* Minimum time (in milliseconds) psmove write updates */
#define PSDS4_WRITE_DATA_INTERVAL_MS 120

enum ePSDualShock4_RequestType {
    PSDualShock4_BTReport_Input = 0x00,
    PSDualShock4_BTReport_Output = 0x11,
    PSDualShock4_USBReport_GetBTAddr = 0x12,
    PSDualShock4_USBReport_SetBTAddr = 0x13,
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
    Btn_PS = 1 << 16,
    Btn_TPAD = 1 << 17,
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
    //unsigned char hid_report_type : 2;      // byte 0, bit 0-1 (0x01=INPUT)
    //unsigned char hid_parameter : 2;        // byte 0, bit 2-3 (0x00)
    //unsigned char hid_transaction_type : 4; // byte 0, bit 4-7 (0x0a=DATA)
    unsigned char hid_protocol_code;        // byte 1 (0x11)
    unsigned char hid_unknown;              // byte 2 (0xc0)

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

    unsigned char gyro_x[2];            // byte 16-17, 16-bit signed x-gyroscope reading
    unsigned char gyro_y[2];            // byte 18-19, 16-bit signed y-gyroscope reading
    unsigned char gyro_z[2];            // byte 20-21, 16-bit signed z-gyroscope reading
    unsigned char accel_x[2];           // byte 22-23, 12-bit signed x-accelerometer reading
    unsigned char accel_y[2];           // byte 24-25, 12-bit signed y-accelerometer reading
    unsigned char accel_z[2];           // byte 26-27, 12-bit signed z-accelerometer reading

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
    unsigned char hid_protocol_code;        // byte 0 (0x11)

    unsigned char _unknown1[2];             // byte 1-2, must be: [0x80|0x00]
    unsigned char rumbleFlags;              // byte 3, 	0xf0 disables the rumble motors, 0xf3 enables them
    unsigned char _unknown2[2];             // byte 4-5, [0x00, 0x00]
    unsigned char rumble_right;             // byte 6, 0x00 to 0xff
    unsigned char rumble_left;              // byte 7, 0x00 to 0xff
    unsigned char led_r;                    // byte 8, red value, 0x00 to 0xff 
    unsigned char led_g;                    // byte 9, green value, 0x00 to 0xff
    unsigned char led_b;                    // byte 10, blue value, 0x00 to 0xff
    unsigned char led_flash_on;             // byte 11, flash on duration
    unsigned char led_flash_off;            // byte 12, flash off duration
    unsigned char _unknown3[8];             // byte 13-20
    unsigned char volume_left;              // byte 21
    unsigned char volume_right;             // byte 22
    unsigned char volume_mic;               // byte 23
    unsigned char volume_speaker;           // byte 24
    unsigned char _unknown4[49];            // byte 25-73
    unsigned char crc32[4];                 // byte 74-77, CRC-32 of the first 75 bytes
};

// -- private prototypes -----
inline enum CommonControllerState::ButtonState getButtonState(unsigned int buttons, unsigned int lastButtons, int buttonMask);
inline bool hid_error_mbs(hid_device *dev, char *out_mb_error, size_t mb_buffer_size);
#ifdef _WIN32
static int hid_set_output_report(hid_device *dev, const unsigned char *data, size_t length);
#endif // _WIN32

// -- public methods

// -- PSMove Controller Config
// Bump this version when you are making a breaking config change.
// Simply adding or removing a field is ok and doesn't require a version bump.
const int PSDualShock4ControllerConfig::CONFIG_VERSION = 3;

const boost::property_tree::ptree
PSDualShock4ControllerConfig::config2ptree()
{
    boost::property_tree::ptree pt;

    pt.put("is_valid", is_valid);
    pt.put("version", PSDualShock4ControllerConfig::CONFIG_VERSION);

    pt.put("Calibration.Accel.X.k", accelerometer_gain.i);
    pt.put("Calibration.Accel.Y.k", accelerometer_gain.j);
    pt.put("Calibration.Accel.Z.k", accelerometer_gain.k);
    pt.put("Calibration.Accel.X.b", accelerometer_bias.i);
    pt.put("Calibration.Accel.Y.b", accelerometer_bias.j);
    pt.put("Calibration.Accel.Z.b", accelerometer_bias.k);
    pt.put("Calibration.Accel.NoiseRadius", accelerometer_noise_radius);
	pt.put("Calibration.Accel.Variance", accelerometer_variance);
    pt.put("Calibration.Gyro.Gain", gyro_gain);
    pt.put("Calibration.Gyro.Variance", gyro_variance);
    pt.put("Calibration.Gyro.Drift", gyro_drift);
    pt.put("Calibration.Identity.Gravity.X", identity_gravity_direction.i);
    pt.put("Calibration.Identity.Gravity.Y", identity_gravity_direction.j);
    pt.put("Calibration.Identity.Gravity.Z", identity_gravity_direction.k);
	pt.put("Calibration.Position.VarianceExpFitA", position_variance_exp_fit_a);
	pt.put("Calibration.Position.VarianceExpFitB", position_variance_exp_fit_b);
	pt.put("Calibration.Orientation.VarianceExpFitA", orientation_variance_exp_fit_a);
	pt.put("Calibration.Orientation.VarianceExpFitB", orientation_variance_exp_fit_b);
	pt.put("Calibration.Time.MeanUpdateTime", mean_update_time_delta);

	pt.put("OrientationFilter.FilterType", orientation_filter_type);

	pt.put("PositionFilter.FilterType", position_filter_type);
    pt.put("PositionFilter.MaxVelocity", max_velocity);

	pt.put("PoseFilter.MinScreenProjectionArea", min_screen_projection_area);

    pt.put("prediction_time", prediction_time);
    pt.put("max_poll_failure_count", max_poll_failure_count);

	writeTrackingColor(pt, tracking_color_id);

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

        // Use the current accelerometer values (constructor defaults) as the default values
        accelerometer_gain.i = pt.get<float>("Calibration.Accel.X.k", accelerometer_gain.i);
        accelerometer_gain.j = pt.get<float>("Calibration.Accel.Y.k", accelerometer_gain.j);
        accelerometer_gain.k = pt.get<float>("Calibration.Accel.Z.k", accelerometer_gain.k);
        accelerometer_bias.i = pt.get<float>("Calibration.Accel.X.b", accelerometer_bias.i);
        accelerometer_bias.j = pt.get<float>("Calibration.Accel.Y.b", accelerometer_bias.j);
        accelerometer_bias.k = pt.get<float>("Calibration.Accel.Z.b", accelerometer_bias.k);
        accelerometer_noise_radius = pt.get<float>("Calibration.Accel.NoiseRadius", accelerometer_noise_radius);
		accelerometer_variance= pt.get<float>("Calibration.Accel.Variance", accelerometer_variance);
		position_variance_exp_fit_a= pt.get<float>("Calibration.Position.VarianceExpFitA", position_variance_exp_fit_a);
		position_variance_exp_fit_b= pt.get<float>("Calibration.Position.VarianceExpFitB", position_variance_exp_fit_b);
		orientation_variance_exp_fit_a= pt.get<float>("Calibration.Orientation.VarianceExpFitA", orientation_variance_exp_fit_a);
		orientation_variance_exp_fit_b= pt.get<float>("Calibration.Orientation.VarianceExpFitB", orientation_variance_exp_fit_b);
		mean_update_time_delta= pt.get<float>("Calibration.Time.MeanUpdateTime", mean_update_time_delta);

        // Use the current gyroscope values (constructor defaults) as the default values
        gyro_gain= pt.get<float>("Calibration.Gyro.Gain", gyro_gain);
        gyro_variance= pt.get<float>("Calibration.Gyro.Variance", gyro_variance);
        gyro_drift= pt.get<float>("Calibration.Gyro.Drift", gyro_drift);

        // Get the orientation filter parameters
		orientation_filter_type= pt.get<std::string>("OrientationFilter.FilterType", orientation_filter_type);

        // Get the position filter parameters
		position_filter_type= pt.get<std::string>("PositionFilter.FilterType", position_filter_type);
        max_velocity= pt.get<float>("PositionFilter.MaxVelocity", max_velocity);

		// Get shared filter parameters
		min_screen_projection_area = pt.get<float>("PoseFilter.MinScreenProjectionArea", min_screen_projection_area);

        // Get the calibration direction for "down"
        identity_gravity_direction.i= pt.get<float>("Calibration.Identity.Gravity.X", identity_gravity_direction.i);
        identity_gravity_direction.j= pt.get<float>("Calibration.Identity.Gravity.Y", identity_gravity_direction.j);
        identity_gravity_direction.k= pt.get<float>("Calibration.Identity.Gravity.Z", identity_gravity_direction.k);

		// Read the tracking color
		tracking_color_id = static_cast<eCommonTrackingColorID>(readTrackingColor(pt));
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
    , bWriteStateDirty(false)
    , NextPollSequenceNumber(0)
{
	HIDDetails.vendor_id = -1;
	HIDDetails.product_id = -1;
    HIDDetails.Handle = nullptr;

    InData = new PSDualShock4DataInput;
    memset(InData, 0, sizeof(PSDualShock4DataInput));

    OutData = new PSDualShock4DataOutput;
    memset(OutData, 0, sizeof(PSDualShock4DataOutput));
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
    ControllerDeviceEnumerator enumerator(ControllerDeviceEnumerator::CommunicationType_HID, CommonControllerState::PSDualShock4);
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

        // Attempt to open the controller 
		HIDDetails.vendor_id = pEnum->get_vendor_id();
		HIDDetails.product_id = pEnum->get_product_id();
        HIDDetails.Device_path = cur_dev_path;
        HIDDetails.Handle = hid_open_path(HIDDetails.Device_path.c_str());

        if (HIDDetails.Handle != nullptr)  // Controller was opened and has an index
        {             
            // Don't block on hid report requests
            hid_set_nonblocking(HIDDetails.Handle, 1);

            /* -USB or Bluetooth Device-

                On my Mac, using bluetooth,
                cur_dev->path = Bluetooth_054c_03d5_779732e8
                cur_dev->serial_number = 00-06-f7-97-32-e8
                On my Mac, using USB,
                cur_dev->path = USB_054c_03d5_14100000
                cur_dev->serial_number = "" (not null, just empty)

                On my Windows 10 box (different controller), using bluetooth
                cur_dev->path = \\?\hid#{00001124-0000-1000-8000-00805f9b34fb}_vid&0002054c_pid&05c4#8&217a4584&0&0000#{4d1e55b2-f16f-11cf-88cb-001111000030}
                cur_dev->serial_number = 1c666d2c8deb
                Using USB
                cur_dev->path = \\?\hid#vid_054c&pid_03d5&col01#6&7773e57&0&0000#{4d1e55b2-f16f-11cf-88cb-001111000030}
                cur_dev->serial_number = (null)
            */

            // Bluetooth connected
            if (strlen(cur_dev_serial_number) > 0)
            {
                IsBluetooth = true;

                // Convert the serial number into a normalized bluetooth address of the form: "xx:xx:xx:xx:xx:xx"
                char szNormalizedControllerAddress[18];
                ServerUtility::bluetooth_cstr_address_normalize(
                    cur_dev_serial_number, true, ':',
                    szNormalizedControllerAddress, sizeof(szNormalizedControllerAddress));

                // Save the controller address as a std::string
                HIDDetails.Bt_addr = std::string(szNormalizedControllerAddress);

                // Use the cached host bluetooth address at service startup.
                // On windows, calling bluetooth_get_host_address() can sometimes be a really long blocking call.
                HIDDetails.Host_bt_addr = DeviceManager::getInstance()->m_controller_manager->getCachedBluetoothHostAddress();

                success = true;
            }
            // USB Connected
            else
            {
                IsBluetooth = false;
                
                // Fetch the bluetooth host and controller addresses via USB HID report request
                success = getBTAddressesViaUSB(HIDDetails.Host_bt_addr, HIDDetails.Bt_addr);

                if (!success)
                {
                    // If serial is still bad, maybe we have a disconnected
                    // controller still showing up in hidapi
                    SERVER_LOG_ERROR("PSDualShock4Controller::open") << "Failed to get bluetooth address of PSDualShock4Controller(" << cur_dev_path << ")";
                }
            }

            if (success)
            {
                // Build a unique name for the config file using bluetooth address of the controller
                char szConfigSuffix[18];
                ServerUtility::bluetooth_cstr_address_normalize(
                    HIDDetails.Bt_addr.c_str(), true, '_',
                    szConfigSuffix, sizeof(szConfigSuffix));

                std::string config_name("dualshock4_");
                config_name += szConfigSuffix;

                // Load the config file
                cfg = PSDualShock4ControllerConfig(config_name);
                cfg.load();

				// Save it back out again in case any defaults changed
				cfg.save();
            }

            // Reset the polling sequence counter
            NextPollSequenceNumber = 0;

            // Write out the initial controller state
            if (success && IsBluetooth)
            {
                bWriteStateDirty= true;
                writeDataOut();
            }
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
            if (IsBluetooth)
            {
                clearAndWriteDataOut();
            }

            hid_close(HIDDetails.Handle);
            HIDDetails.Handle = nullptr;
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
    bts[0] = PSDualShock4_USBReport_SetBTAddr;

    unsigned char addr[6];
    if (ServerUtility::bluetooth_string_address_to_bytes(new_host_bt_addr, addr, sizeof(addr)))
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
        res = hid_send_feature_report(HIDDetails.Handle, bts, sizeof(bts));

        if (res == sizeof(bts))
        {
            success = true;
        }
        else
        {
            char hidapi_err_mbs[256];
            bool valid_error_mesg = false;

            valid_error_mesg = hid_error_mbs(HIDDetails.Handle, hidapi_err_mbs, sizeof(hidapi_err_mbs));

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

bool 
PSDualShock4Controller::setTrackingColorID(const eCommonTrackingColorID tracking_color_id)
{
	bool bSuccess = false;

	if (getIsOpen() && getIsBluetooth())
	{
		cfg.tracking_color_id = tracking_color_id;
		cfg.save();
		bSuccess = true;
	}

	return bSuccess;
}

// Getters
bool
PSDualShock4Controller::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    // Down-cast the enumerator so we can use the correct get_path.
    const ControllerDeviceEnumerator *pEnum = static_cast<const ControllerDeviceEnumerator *>(enumerator);

    bool matches = false;

    if (pEnum->get_device_type() == CommonControllerState::PSDualShock4)
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

int
PSDualShock4Controller::getVendorID() const
{
	return HIDDetails.vendor_id;
}

int 
PSDualShock4Controller::getProductID() const
{
	return HIDDetails.product_id;
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
    return CommonDeviceState::PSDualShock4;
}

bool
PSDualShock4Controller::getBTAddressesViaUSB(std::string& host, std::string& controller)
{
    bool success = false;
    int res;

    unsigned char btg[PSDS4_BTADDR_GET_SIZE + 1];
    unsigned char ctrl_char_buff[PSDS4_BTADDR_SIZE];
    unsigned char host_char_buff[PSDS4_BTADDR_SIZE];

    memset(btg, 0, sizeof(btg));
    btg[0] = PSDualShock4_USBReport_GetBTAddr;
    res = hid_get_feature_report(HIDDetails.Handle, btg, sizeof(btg));

    if (res == sizeof(btg))
    {
        memcpy(host_char_buff, btg + 10, PSDS4_BTADDR_SIZE);
        host = ServerUtility::bluetooth_byte_addr_to_string(host_char_buff);

        memcpy(ctrl_char_buff, btg + 1, PSDS4_BTADDR_SIZE);
        controller = ServerUtility::bluetooth_byte_addr_to_string(ctrl_char_buff);

        success = true;
    }
    else
    {
        char hidapi_err_mbs[256];
        bool valid_error_mesg = false;

        valid_error_mesg = hid_error_mbs(HIDDetails.Handle, hidapi_err_mbs, sizeof(hidapi_err_mbs));

        if (valid_error_mesg)
        {
            SERVER_LOG_ERROR("PSDualShock4Controller::getBTAddress") << "HID ERROR: " << hidapi_err_mbs;
        }
    }

    return success;
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
                //SERVER_LOG_WARNING("PSDualShock4Controller::readDataIn") << "Read Bytes: " << res;

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
                //SERVER_LOG_WARNING("PSDualShock4Controller::readDataIn") << "Read Bytes: " << res;

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
                (((unsigned int)InData->buttons3.raw & 0x3) << 16) | // Get the 1st two bits of buttons: [0|0|0|0|0|0|PS|TPad]
                (unsigned int)(InData->buttons2.raw << 8) | // [R3|L3|Option|Share|R2|L2|R1|L1]
                ((unsigned int)InData->buttons1.raw & 0xF0); // Mask out the dpad enum (1st four bits): [tri|cir|x|sq|0|0|0|0]

            // Converts the dpad enum to independent bit flags
            {
                ePSDualShock4_DPad dpad_enum = static_cast<ePSDualShock4_DPad>(InData->buttons1.raw & 0xF);
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
                short raw_accelX = static_cast<short>((InData->accel_x[1] << 8) | InData->accel_x[0]) >> 4;
                short raw_accelY = static_cast<short>((InData->accel_y[1] << 8) | InData->accel_y[0]) >> 4;
                short raw_accelZ = static_cast<short>((InData->accel_z[1] << 8) | InData->accel_z[0]) >> 4;

                // Piece together the 16-bit gyroscope data
                short raw_gyroX = static_cast<short>((InData->gyro_x[1] << 8) | InData->gyro_x[0]);
                short raw_gyroY = static_cast<short>((InData->gyro_y[1] << 8) | InData->gyro_y[0]);
                short raw_gyroZ = static_cast<short>((InData->gyro_z[1] << 8) | InData->gyro_z[0]);

                // Save the raw accelerometer values
                newState.RawAccelerometer[0] = static_cast<int>(raw_accelX);
                newState.RawAccelerometer[1] = static_cast<int>(raw_accelY);
                newState.RawAccelerometer[2] = static_cast<int>(raw_accelZ);

                // Save the raw gyro values
                newState.RawGyro[0] = static_cast<int>(raw_gyroX);
                newState.RawGyro[1] = static_cast<int>(raw_gyroY);
                newState.RawGyro[2] = static_cast<int>(raw_gyroZ);

                // calibrated_acc= raw_acc*acc_gain + acc_bias
                newState.CalibratedAccelerometer.i = 
                    static_cast<float>(newState.RawAccelerometer[0]) * cfg.accelerometer_gain.i 
                    + cfg.accelerometer_bias.i;
                newState.CalibratedAccelerometer.j =
                    static_cast<float>(newState.RawAccelerometer[1]) * cfg.accelerometer_gain.j
                    + cfg.accelerometer_bias.j;
                newState.CalibratedAccelerometer.k =
                    static_cast<float>(newState.RawAccelerometer[2]) * cfg.accelerometer_gain.k
                    + cfg.accelerometer_bias.k;

                // calibrated_gyro= raw_gyro*gyro_gain + gyro_bias
                newState.CalibratedGyro.i = static_cast<float>(newState.RawGyro[0]) * cfg.gyro_gain;
                newState.CalibratedGyro.j = static_cast<float>(newState.RawGyro[1]) * cfg.gyro_gain;
                newState.CalibratedGyro.k = static_cast<float>(newState.RawGyro[2]) * cfg.gyro_gain;
            }

            // Sequence and timestamp
            newState.RawSequence = InData->buttons3.state.counter;
            newState.RawTimeStamp = InData->timestamp;

            // Convert the 0-10 battery level into the batter level
            switch (InData->batteryLevel)
            {
            case 0:
                newState.Battery = CommonControllerState::BatteryLevel::Batt_CHARGING;
                break;
            case 1:
                newState.Battery = CommonControllerState::BatteryLevel::Batt_MIN;
                break;
            case 2:
            case 3:
                newState.Battery = CommonControllerState::BatteryLevel::Batt_20Percent;
                break;
            case 4:
            case 5:
                newState.Battery = CommonControllerState::BatteryLevel::Batt_40Percent;
                break;
            case 6:
            case 7:
                newState.Battery = CommonControllerState::BatteryLevel::Batt_60Percent;
                break;
            case 8:
            case 9:
                newState.Battery = CommonControllerState::BatteryLevel::Batt_80Percent;
                break;
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
    const float quad_half_x = PSDS4_TRACKING_QUAD_WIDTH / 2.f;
    const float quad_half_y = PSDS4_TRACKING_QUAD_HEIGHT / 2.f;

    const float tri_half_x = PSDS4_TRACKING_TRIANGLE_WIDTH / 2.f;
    const float tri_lower_half_y = PSDS4_TRACKING_TRIANGLE_HEIGHT - quad_half_y;

    // We define the origin to be the center of the light bar.
    // The light bar on the DS4 is tilted inward 30 degrees.
    // The coordinate system on the DS4 is defined as follows:
    // x-axis= from the center toward the circle button
    // y-axis= from the center up through the track pad
    // z-axis= from the center out through the extension port

    // The triangle connects the mid-points of each light-bar edge (lower right, lower left, upper middle)
    outTrackingShape.shape.light_bar.triangle[CommonDeviceTrackingShape::TriVertexLowerRight] = 
		{ tri_half_x, -tri_lower_half_y, 0.f };
    outTrackingShape.shape.light_bar.triangle[CommonDeviceTrackingShape::TriVertexLowerLeft] = 
		{ -tri_half_x, -tri_lower_half_y, 0.f };
    outTrackingShape.shape.light_bar.triangle[CommonDeviceTrackingShape::TriVertexUpperMiddle] = 
		{ 0.f, quad_half_y, 0.f };

    // The quad bounds the light-bar (upper right, upper left, lower left, lower right)
    outTrackingShape.shape.light_bar.quad[CommonDeviceTrackingShape::QuadVertexUpperRight] = 
		{ quad_half_x, quad_half_y, 0.f };
    outTrackingShape.shape.light_bar.quad[CommonDeviceTrackingShape::QuadVertexUpperLeft] = 
		{ -quad_half_x, quad_half_y, 0.f };
    outTrackingShape.shape.light_bar.quad[CommonDeviceTrackingShape::QuadVertexLowerLeft] = 
		{ -quad_half_x, -quad_half_y, 0.f };
    outTrackingShape.shape.light_bar.quad[CommonDeviceTrackingShape::QuadVertexLowerRight] = 
		{ quad_half_x, -quad_half_y, 0.f };

    outTrackingShape.shape_type = eCommonTrackingShapeType::LightBar;
}

bool
PSDualShock4Controller::getTrackingColorID(eCommonTrackingColorID &out_tracking_color_id) const
{
	bool bSuccess = false;

	if (getIsOpen() && getIsBluetooth())
	{
		out_tracking_color_id = cfg.tracking_color_id;
		bSuccess = true;
	}

	return bSuccess;
}

float PSDualShock4Controller::getIdentityForwardDegrees() const
{
	// Controller model points down the +Z axis when it has the identity orientation
	return 270.f;
}

float PSDualShock4Controller::getPredictionTime() const
{
	return getConfig()->prediction_time;
}

long PSDualShock4Controller::getMaxPollFailureCount() const
{
    return cfg.max_poll_failure_count;
}

// Setters
void 
PSDualShock4Controller::clearAndWriteDataOut()
{
    LedR = LedG = LedB = 0;
    RumbleLeft = RumbleRight = 0;
    bWriteStateDirty= true;
    writeDataOut();
}

bool
PSDualShock4Controller::writeDataOut()
{
    bool bSuccess= true;

    if (bWriteStateDirty)
    {
        const bool bLedIsOn = LedR != 0 || LedG != 0 || LedB != 0;
        const bool bIsRumbleOn = RumbleRight != 0 || RumbleLeft != 0;

        // Light bar color
        OutData->led_r = LedR;
        OutData->led_g = LedG;
        OutData->led_b = LedB;
        
        // Set off interval to 0% and the on interval to 100%. 
        // There are no discos in PSMoveService.
        OutData->led_flash_on = bLedIsOn ? 0xff : 0x00;
        OutData->led_flash_off = 0x00; 

        // a.k.a Soft Rumble Motor
        OutData->rumble_right = RumbleRight;
        // a.k.a Hard Rumble Motor
        OutData->rumble_left = RumbleLeft;

        // Keep writing state out until the desired LED and Rumble are 0 
        bWriteStateDirty = bLedIsOn || bIsRumbleOn;

        // Unfortunately in windows simply writing to the HID device, via WriteFile() internally, 
        // doesn't appear to actually set the data on the controller (despite returning successfully).
        // In the DS4 implementation they use the HidD_SetOutputReport() Win32 API call instead. 
        // Unfortunately HIDAPI doesn't have any equivalent call, so we have to make our own.
        #ifdef _WIN32
        int res = hid_set_output_report(HIDDetails.Handle, (unsigned char*)OutData, sizeof(PSDualShock4DataOutput));
        #else
        int res = hid_write(HIDDetails.Handle, (unsigned char*)OutData, sizeof(PSDualShock4DataOutput));
        #endif
        bSuccess = res > 0;

        if (!bSuccess)
        {
            char szErrorMessage[256];

            if (hid_error_mbs(HIDDetails.Handle, szErrorMessage, sizeof(szErrorMessage)))
            {
                SERVER_LOG_ERROR("PSDualShock4Controller::writeDataOut") << "HID ERROR: " << szErrorMessage;
            }
        }
    }

    return bSuccess;
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
PSDualShock4Controller::setLeftRumbleIntensity(unsigned char value)
{
    bool success = true;
    if (RumbleLeft != value)
    {
        RumbleLeft = value;
        bWriteStateDirty = true;
        success = writeDataOut();
    }
    return success;
}

bool
PSDualShock4Controller::setRightRumbleIntensity(unsigned char value)
{
    bool success = true;
    if (RumbleRight != value)
    {
        RumbleRight = value;
        bWriteStateDirty = true;
        success = writeDataOut();
    }
    return success;
}

// -- private helper functions -----
inline enum CommonControllerState::ButtonState
getButtonState(unsigned int buttons, unsigned int lastButtons, int buttonMask)
{
    return (enum CommonControllerState::ButtonState)((((lastButtons & buttonMask) > 0) << 1) + ((buttons & buttonMask)>0));
}

inline bool hid_error_mbs(hid_device *dev, char *out_mb_error, size_t mb_buffer_size)
{
    return ServerUtility::convert_wcs_to_mbs(hid_error(dev), out_mb_error, mb_buffer_size);
}

#ifdef _WIN32
static int hid_set_output_report(hid_device *dev, const unsigned char *data, size_t length)
{
    // This copies the private structure inside the windows implementation of HidAPI
    struct hid_device_ {
        HANDLE device_handle;
        BOOL blocking;
        USHORT output_report_length;
        size_t input_report_length;
        void *last_error_str;
        DWORD last_error_num;
        BOOL read_pending;
        char *read_buf;
        OVERLAPPED ol;
    };
    hid_device_ *dev_internal = (hid_device_ *)dev;

    DWORD bytes_written;
    unsigned char *buf;
    BOOL res;

    if (length >= dev_internal->output_report_length)
    {
        // The user passed the right number of bytes. Use the buffer as-is.
        buf = (unsigned char *)data;
    }
    else
    {
        // Create a temporary buffer and copy the user's data into it, padding the rest with zeros.
        buf = (unsigned char *)malloc(dev_internal->output_report_length);
        memcpy(buf, data, length);
        memset(buf + length, 0, dev_internal->output_report_length - length);
        length = dev_internal->output_report_length;
    }

    res = HidD_SetOutputReport(dev_internal->device_handle, buf, length);

    if (res == TRUE)
    {
        bytes_written = length;
    }
    else
    {
        const DWORD error_code = GetLastError();

        if (error_code != ERROR_IO_PENDING)
        {
            WCHAR *error_msg;

            FormatMessageW(
                FORMAT_MESSAGE_ALLOCATE_BUFFER |
                FORMAT_MESSAGE_FROM_SYSTEM |
                FORMAT_MESSAGE_IGNORE_INSERTS,
                NULL,
                error_code,
                MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                (LPWSTR)&error_msg, 0/*sz*/,
                NULL);

            /* Store the message off in the Device entry so that
            the hid_error() function can pick it up. */
            LocalFree(dev_internal->last_error_str);
            dev_internal->last_error_str = error_msg;

            bytes_written = -1;
        }
    }

    if (buf != data)
    {
        free(buf);
    }

    return bytes_written;
}
#endif
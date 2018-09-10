// A large chunk of this file was adapted from PSMoveAPI.
// Reproducing the license here:
/*
The PS Move API library is licensed under the terms of the license below.
However, some optional third party libraries might have a different license.
Be sure to read the README file for details on third party licenses.

====

Copyright (c) 2011, 2012 Thomas Perl <m@thp.io>
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

//-- includes -----
#include "AtomicPrimitives.h"
#include "PSMoveController.h"
#include "ControllerDeviceEnumerator.h"
#include "ServerLog.h"
#include "ServerUtility.h"
#include "BluetoothQueries.h"
#include "MathAlignment.h"
#include "WorkerThread.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <vector>
#include <cstdlib>
#include <chrono>
#include <thread>
#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif
#include <math.h>

//-- constants -----
#define PSMOVE_BUFFER_SIZE 49 /* Buffer size for writing LEDs and reading sensor data */
#define PSMOVE_EXT_DATA_BUF_SIZE 5
#define PSMOVE_BTADDR_GET_ZCM1_SIZE 16 
#define PSMOVE_BTADDR_GET_ZCM2_SIZE 21
#define PSMOVE_BTADDR_GET_MAX_SIZE PSMOVE_BTADDR_GET_ZCM2_SIZE
#define PSMOVE_BTADDR_SET_SIZE 23
#define PSMOVE_BTADDR_SIZE 6
#define PSMOVE_FW_GET_SIZE 13
#define PSMOVE_CALIBRATION_SIZE 49 /* Buffer size for calibration data */
#define PSMOVE_ZCM1_CALIBRATION_BLOB_SIZE (PSMOVE_CALIBRATION_SIZE*3 - 2*2) /* Three blocks, minus header (2 bytes) for blocks 2,3 */
#define PSMOVE_ZCM2_CALIBRATION_BLOB_SIZE (PSMOVE_CALIBRATION_SIZE*2 - 2*1) /* Three blocks, minus header (2 bytes) for block 2 */
#define PSMOVE_STATE_BUFFER_MAX 16

#define PSMOVE_TRACKING_BULB_RADIUS  2.25f // The radius of the psmove tracking bulb in cm

/* Minimum time (in milliseconds) psmove write updates */
#define PSMOVE_WRITE_DATA_INTERVAL_MS 120

/* Decode 12-bit signed value (assuming two's complement) */
#define TWELVE_BIT_SIGNED(x) (((x) & 0x800)?(-(((~(x)) & 0xFFF) + 1)):(x))

enum PSNaviRequestType {
    PSMove_Req_GetInput = 0x01,
    PSMove_Req_SetLEDs = 0x06,
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
};

// -- private definitions -----
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

struct PSMoveDataInputCommon
{
    unsigned char type; /* message type, must be PSMove_Req_GetInput */
    unsigned char buttons1;
    unsigned char buttons2;
    unsigned char buttons3;
    unsigned char buttons4;
    unsigned char trigger; /* trigger value; 0..255 */
    unsigned char trigger2; /* trigger value, 2nd frame */
    unsigned char _unused0;
    unsigned char _unused1;
    unsigned char _unused2;
    unsigned char _unused3;
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
};

struct PSMoveDataInputZCM1 : PSMoveDataInputCommon
{
    unsigned char temphigh; /* temperature (bits 12-5) */
    unsigned char templow_mXhigh; /* temp (bits 4-1); magneto X (bits 12-9) */
    unsigned char mXlow; /* magnetometer X (bits 8-1) */
    unsigned char mYhigh; /* magnetometer Y (bits 12-5) */
    unsigned char mYlow_mZhigh; /* magnetometer: Y (bits 4-1), Z (bits 12-9) */
    unsigned char mZlow; /* magnetometer Z (bits 8-1) */
    unsigned char timelow; /* low byte of timestamp */
    unsigned char extdata[PSMOVE_EXT_DATA_BUF_SIZE]; /* external device data (EXT port) */
};

struct PSMoveDataInputZCM2  : PSMoveDataInputCommon
{
    unsigned char temphigh; /* temperature (bits 12-5) */
    unsigned char templow; /* temp (bits 4-1); */
    unsigned char timelow; /* low byte of timestamp */
	unsigned char _unused4; /* Always 0 */
	unsigned char _unused5; /* Always 0 */
    unsigned char _unused6; /* Always 0 */
};

class PSMoveHidPacketProcessor : public WorkerThread
{
public:
	PSMoveHidPacketProcessor(const PSMoveControllerConfig &cfg, PSMoveControllerModelPID model) 
		: WorkerThread("PSMoveSensorProcessor")
		, m_cfg(cfg)
		, m_model(model)
		, m_hidDevice(nullptr)
		, m_controllerListener(nullptr)
		, m_bSupportsMagnetometer(false)
		, m_nextPollSequenceNumber(0)
		, m_previousHIDInputPacket(nullptr)
		, m_currentHIDInputPacket(nullptr)
	{
		if (m_model == _psmove_controller_ZCM2)
		{
			m_previousHIDInputPacket = new PSMoveDataInputZCM2;
			m_currentHIDInputPacket = new PSMoveDataInputZCM2;
			memset(m_previousHIDInputPacket, 0, sizeof(PSMoveDataInputZCM2));
			memset(m_currentHIDInputPacket, 0, sizeof(PSMoveDataInputZCM2));
		}
		else
		{
			m_previousHIDInputPacket = new PSMoveDataInputZCM1;
			m_currentHIDInputPacket = new PSMoveDataInputZCM1;
			memset(m_previousHIDInputPacket, 0, sizeof(PSMoveDataInputZCM1));
			memset(m_currentHIDInputPacket, 0, sizeof(PSMoveDataInputZCM1));
		}
		m_previousHIDInputPacket->type = PSMove_Req_GetInput;
		m_currentHIDInputPacket->type = PSMove_Req_GetInput;

		memset(&m_previousOutputState, 0, sizeof(PSMoveControllerOutputState));
	}

	~PSMoveHidPacketProcessor()
	{
		delete m_currentHIDInputPacket;
	}

	bool getSupportsMagnetometer() const
	{
		return m_bSupportsMagnetometer;
	}

	void fetchLatestInputData(PSMoveControllerInputState &input_state)
	{
		m_currentInputState.fetchValue(input_state);
	}

	void postOutputState(const PSMoveControllerOutputState &output_state)
	{
		m_currentOutputState.storeValue(output_state);
	}

    void start(hid_device *in_hid_device, IControllerListener *controller_listener)
    {
		if (!hasThreadStarted())
		{
			m_hidDevice= in_hid_device;
			m_controllerListener= controller_listener;

			// See if this controller has a functional magnetometer
			testMagnetometer();

			// Fire up the worker thread
			WorkerThread::startThread();
		}
    }

	void stop()
	{
		WorkerThread::stopThread();
	}

protected:
	void testMagnetometer()
	{
		if (m_model == _psmove_controller_ZCM1)
		{
			const int k_max_poll_attempts = 10;
			int poll_count = 0;

			for (poll_count = 0; poll_count < k_max_poll_attempts; ++poll_count)
			{
				PSMoveDataInputZCM1 rawHIDPacket;
				int res = hid_read(m_hidDevice, (unsigned char*)&rawHIDPacket, sizeof(PSMoveDataInputZCM1));

				if (res > 0)
				{
					PSMoveControllerInputState newState;
					newState.parseDataInput(&m_cfg, m_model, nullptr, &rawHIDPacket);

					// See if we are getting valid magnetometer data
					m_bSupportsMagnetometer = 
						newState.RawMag[0] != 0 || newState.RawMag[1] != 0 || newState.RawMag[2] != 0;
					break;
				}
				else
				{
					const std::chrono::milliseconds k_WaitForDataMilliseconds(5);

					std::this_thread::sleep_for(k_WaitForDataMilliseconds);
				}
			}

			if (poll_count >= k_max_poll_attempts)
			{
				SERVER_LOG_ERROR("testMagnetometer") << "Failed to open read initial controller state after " << k_max_poll_attempts << " attempts.";
			}
		}
	}

	virtual bool doWork() override
    {
		// Attempt to read the next sensor update packet from the HMD
        int res = -1;
		if (m_model == _psmove_controller_ZCM2)
		{
			memcpy(m_previousHIDInputPacket, m_currentHIDInputPacket, sizeof(PSMoveDataInputZCM2));
			res= hid_read_timeout(m_hidDevice, (unsigned char*)m_currentHIDInputPacket, sizeof(PSMoveDataInputZCM2), m_cfg.poll_timeout_ms);
		}
		else
		{
			memcpy(m_previousHIDInputPacket, m_currentHIDInputPacket, sizeof(PSMoveDataInputZCM1));
			res= hid_read_timeout(m_hidDevice, (unsigned char*)m_currentHIDInputPacket, sizeof(PSMoveDataInputZCM1), m_cfg.poll_timeout_ms);
		}

		if (res > 0)
		{
			// https://github.com/hrl7/node-psvr/blob/master/lib/psvr.js
			PSMoveControllerInputState newState;

			// Increment the sequence for every new polling packet
			newState.PollSequenceNumber = m_nextPollSequenceNumber;
			++m_nextPollSequenceNumber;

			// Processes the IMU data
			newState.parseDataInput(&m_cfg, m_model, m_previousHIDInputPacket, m_currentHIDInputPacket);

			// Store a copy of the parsed input date for functions
			// that want to query input state off of the worker thread
			m_currentInputState.storeValue(newState);

			// Send the sensor data for processing by filter
			if (m_controllerListener != nullptr)
			{
				m_controllerListener->notifySensorDataReceived(&newState);
			}
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

			return true;
		}

        // Don't send output writes too frequently
        {
            std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();

            // See if it's time to update the LED/rumble state
            std::chrono::duration<double, std::milli> led_update_diff = now - m_lastHIDOutputTimestamp;
            if (led_update_diff.count() >= PSMOVE_WRITE_DATA_INTERVAL_MS)
            {
				PSMoveControllerOutputState output_state;
				m_currentOutputState.fetchValue(output_state);

				const bool bWriteStateChanged=
					output_state.r != m_previousOutputState.r ||
					output_state.g != m_previousOutputState.g ||
					output_state.b != m_previousOutputState.b ||
					output_state.rumble != m_previousOutputState.rumble;
				const bool bWriteStateNonZero=
					output_state.r != 0 ||
					output_state.g != 0 ||
					output_state.b != 0 ||
					output_state.rumble != 0;

				if (bWriteStateChanged || bWriteStateNonZero)
				{
					PSMoveDataOutput data_out;
					memset(&data_out, 0, sizeof(PSMoveDataOutput));
					data_out.type = PSMove_Req_SetLEDs;
					data_out.r = output_state.r;
					data_out.g = output_state.g;
					data_out.b = output_state.b;
					data_out.rumble = output_state.rumble;
					data_out.rumble2 = 0x00;

					res = hid_write(m_hidDevice, (unsigned char*)(&data_out), sizeof(data_out));
					if (res > 0)
					{
						m_previousOutputState= output_state;
						m_lastHIDOutputTimestamp = now;
					}
					else
					{
						char hidapi_err_mbs[256];
						bool valid_error_mesg = 
							ServerUtility::convert_wcs_to_mbs(hid_error(m_hidDevice), hidapi_err_mbs, sizeof(hidapi_err_mbs));

						// Device no longer in valid state.
						if (valid_error_mesg)
						{
							SERVER_MT_LOG_ERROR("PSMoveSensorProcessor::doWork") << "HID ERROR: " << hidapi_err_mbs;
						}
					}
				}
            }
        }

		return true;
    }

    // Multi-threaded state
	const PSMoveControllerConfig m_cfg;
	PSMoveControllerModelPID m_model;
	hid_device *m_hidDevice;
	IControllerListener *m_controllerListener;
	bool m_bSupportsMagnetometer;
	AtomicObject<PSMoveControllerInputState> m_currentInputState;
	AtomicObject<PSMoveControllerOutputState> m_currentOutputState;

    // Worker thread state
    int m_nextPollSequenceNumber;
	PSMoveDataInputCommon *m_previousHIDInputPacket;
    PSMoveDataInputCommon *m_currentHIDInputPacket;
	std::chrono::time_point<std::chrono::high_resolution_clock> m_lastHIDOutputTimestamp;
	PSMoveControllerOutputState m_previousOutputState;
};

// -- private prototypes -----
static std::string PSMoveBTAddrUcharToString(const unsigned char* addr_buff);
static bool stringToPSMoveBTAddrUchar(const std::string &addr, unsigned char *addr_buff, const int addr_buf_size);
static short decode16bitSigned(char *data, int offset);
static int decode16bitUnsignedToSigned(char *data, int offset);
static int decode16bitTwosCompliment(char *data, int offset);
inline enum CommonControllerState::ButtonState getButtonState(unsigned int buttons, unsigned int lastButtons, int buttonMask);
inline unsigned int make_psmove_button_bitmask(const PSMoveDataInputCommon *hid_packet);
inline bool hid_error_mbs(hid_device *dev, char *out_mb_error, size_t mb_buffer_size);

// -- public methods

// -- PSMove Controller Config
// Bump this version when you are making a breaking config change.
// Simply adding or removing a field is ok and doesn't require a version bump.
const int PSMoveControllerConfig::CONFIG_VERSION= 2;

const boost::property_tree::ptree
PSMoveControllerConfig::config2ptree()
{
    boost::property_tree::ptree pt;

    pt.put("is_valid", is_valid);
    pt.put("version", PSMoveControllerConfig::CONFIG_VERSION);

	pt.put("firmware_version", firmware_version);
	pt.put("bt_firmware_version", bt_firmware_version);
	pt.put("firmware_revision", firmware_revision);

    pt.put("prediction_time", prediction_time);
	pt.put("max_poll_failure_count", max_poll_failure_count);
    pt.put("poll_timeout_ms", poll_timeout_ms);
    
    pt.put("Calibration.Accel.X.k", cal_ag_xyz_kbd[0][0][0]);
    pt.put("Calibration.Accel.X.b", cal_ag_xyz_kbd[0][0][1]);
    pt.put("Calibration.Accel.Y.k", cal_ag_xyz_kbd[0][1][0]);
    pt.put("Calibration.Accel.Y.b", cal_ag_xyz_kbd[0][1][1]);
    pt.put("Calibration.Accel.Z.k", cal_ag_xyz_kbd[0][2][0]);
    pt.put("Calibration.Accel.Z.b", cal_ag_xyz_kbd[0][2][1]);

	pt.put("Calibration.Accel.Variance", accelerometer_variance);
    pt.put("Calibration.Accel.NoiseRadius", accelerometer_noise_radius);

    pt.put("Calibration.Gyro.X.k", cal_ag_xyz_kbd[1][0][0]);
    pt.put("Calibration.Gyro.X.b", cal_ag_xyz_kbd[1][0][1]);
	pt.put("Calibration.Gyro.X.d", cal_ag_xyz_kbd[1][0][2]);
    pt.put("Calibration.Gyro.Y.k", cal_ag_xyz_kbd[1][1][0]);
    pt.put("Calibration.Gyro.Y.b", cal_ag_xyz_kbd[1][1][1]);
	pt.put("Calibration.Gyro.Y.d", cal_ag_xyz_kbd[1][1][2]);
    pt.put("Calibration.Gyro.Z.k", cal_ag_xyz_kbd[1][2][0]);
    pt.put("Calibration.Gyro.Z.b", cal_ag_xyz_kbd[1][2][1]);
	pt.put("Calibration.Gyro.Z.d", cal_ag_xyz_kbd[1][2][2]);

    pt.put("Calibration.Gyro.Variance", gyro_variance);
    pt.put("Calibration.Gyro.Drift", gyro_drift);

    pt.put("Calibration.Magnetometer.Center.X", magnetometer_center.i);
    pt.put("Calibration.Magnetometer.Center.Y", magnetometer_center.j);
    pt.put("Calibration.Magnetometer.Center.Z", magnetometer_center.k);

    pt.put("Calibration.Magnetometer.BasisX.X", magnetometer_basis_x.i);
    pt.put("Calibration.Magnetometer.BasisX.Y", magnetometer_basis_x.j);
    pt.put("Calibration.Magnetometer.BasisX.Z", magnetometer_basis_x.k);
    pt.put("Calibration.Magnetometer.BasisY.X", magnetometer_basis_y.i);
    pt.put("Calibration.Magnetometer.BasisY.Y", magnetometer_basis_y.j);
    pt.put("Calibration.Magnetometer.BasisY.Z", magnetometer_basis_y.k);
    pt.put("Calibration.Magnetometer.BasisZ.X", magnetometer_basis_z.i);
    pt.put("Calibration.Magnetometer.BasisZ.Y", magnetometer_basis_z.j);
    pt.put("Calibration.Magnetometer.BasisZ.Z", magnetometer_basis_z.k);

    pt.put("Calibration.Magnetometer.Extents.X", magnetometer_extents.i);
    pt.put("Calibration.Magnetometer.Extents.Y", magnetometer_extents.j);
    pt.put("Calibration.Magnetometer.Extents.Z", magnetometer_extents.k);

    pt.put("Calibration.Magnetometer.Identity.X", magnetometer_identity.i);
    pt.put("Calibration.Magnetometer.Identity.Y", magnetometer_identity.j);
    pt.put("Calibration.Magnetometer.Identity.Z", magnetometer_identity.k);

    pt.put("Calibration.Magnetometer.Error", magnetometer_fit_error);
	pt.put("Calibration.Magnetometer.Variance", magnetometer_variance);

	pt.put("Calibration.Position.VarianceExpFitA", position_variance_exp_fit_a);
	pt.put("Calibration.Position.VarianceExpFitB", position_variance_exp_fit_b);

	pt.put("Calibration.Orientation.Variance", orientation_variance);

	pt.put("Calibration.Time.MeanUpdateTime", mean_update_time_delta);

	pt.put("OrientationFilter.FilterType", orientation_filter_type);

	pt.put("PositionFilter.FilterType", position_filter_type);
    pt.put("PositionFilter.MaxVelocity", max_velocity);

	pt.put("hand", hand);

	writeTrackingColor(pt, tracking_color_id);

    return pt;
}

void
PSMoveControllerConfig::ptree2config(const boost::property_tree::ptree &pt)
{
    version = pt.get<int>("version", 0);

    if (version == PSMoveControllerConfig::CONFIG_VERSION)
    {
        is_valid = pt.get<bool>("is_valid", false);

		firmware_version = pt.get<unsigned short>("firmware_version", 0);
		bt_firmware_version = pt.get<unsigned short>("bt_firmware_version", 0);
		firmware_revision = pt.get<unsigned short>("firmware_revision", 0);

        prediction_time = pt.get<float>("prediction_time", 0.f);
		max_poll_failure_count = pt.get<long>("max_poll_failure_count", 100);
        poll_timeout_ms = pt.get<long>("poll_timeout_ms", 1000);

        cal_ag_xyz_kbd[0][0][0] = pt.get<float>("Calibration.Accel.X.k", 1.0f);
        cal_ag_xyz_kbd[0][0][1] = pt.get<float>("Calibration.Accel.X.b", 0.0f);
		cal_ag_xyz_kbd[0][0][2] = 0.f; // No drift
        cal_ag_xyz_kbd[0][1][0] = pt.get<float>("Calibration.Accel.Y.k", 1.0f);
        cal_ag_xyz_kbd[0][1][1] = pt.get<float>("Calibration.Accel.Y.b", 0.0f);
		cal_ag_xyz_kbd[0][1][2] = 0.f; // No drift
        cal_ag_xyz_kbd[0][2][0] = pt.get<float>("Calibration.Accel.Z.k", 1.0f);
        cal_ag_xyz_kbd[0][2][1] = pt.get<float>("Calibration.Accel.Z.b", 0.0f);
		cal_ag_xyz_kbd[0][2][2] = 0.f; // No drift

		accelerometer_variance = pt.get<float>("Calibration.Accel.Variance", accelerometer_variance);
        accelerometer_noise_radius = pt.get<float>("Calibration.Accel.NoiseRadius", 0.0f);

        cal_ag_xyz_kbd[1][0][0] = pt.get<float>("Calibration.Gyro.X.k", 1.0f);
        cal_ag_xyz_kbd[1][0][1] = pt.get<float>("Calibration.Gyro.X.b", 0.0f);
		cal_ag_xyz_kbd[1][0][2] = pt.get<float>("Calibration.Gyro.X.d", 0.0f);
        cal_ag_xyz_kbd[1][1][0] = pt.get<float>("Calibration.Gyro.Y.k", 1.0f);
        cal_ag_xyz_kbd[1][1][1] = pt.get<float>("Calibration.Gyro.Y.b", 0.0f);
		cal_ag_xyz_kbd[1][1][2] = pt.get<float>("Calibration.Gyro.Y.d", 0.0f);
        cal_ag_xyz_kbd[1][2][0] = pt.get<float>("Calibration.Gyro.Z.k", 1.0f);
        cal_ag_xyz_kbd[1][2][1] = pt.get<float>("Calibration.Gyro.Z.b", 0.0f);
		cal_ag_xyz_kbd[1][2][2] = pt.get<float>("Calibration.Gyro.Z.d", 0.0f);

        gyro_variance= pt.get<float>("Calibration.Gyro.Variance", gyro_variance);
        gyro_drift= pt.get<float>("Calibration.Gyro.Drift", gyro_drift);

        magnetometer_center.i = pt.get<float>("Calibration.Magnetometer.Center.X", 0.f);
        magnetometer_center.j = pt.get<float>("Calibration.Magnetometer.Center.Y", 0.f);
        magnetometer_center.k = pt.get<float>("Calibration.Magnetometer.Center.Z", 0.f);

        magnetometer_basis_x.i = pt.get<float>("Calibration.Magnetometer.BasisX.X", 1.f);
        magnetometer_basis_x.j = pt.get<float>("Calibration.Magnetometer.BasisX.Y", 0.f);
        magnetometer_basis_x.k = pt.get<float>("Calibration.Magnetometer.BasisX.Z", 0.f);

        magnetometer_basis_y.i = pt.get<float>("Calibration.Magnetometer.BasisY.X", 0.f);
        magnetometer_basis_y.j = pt.get<float>("Calibration.Magnetometer.BasisY.Y", 1.f);
        magnetometer_basis_y.k = pt.get<float>("Calibration.Magnetometer.BasisY.Z", 0.f);

        magnetometer_basis_z.i = pt.get<float>("Calibration.Magnetometer.BasisZ.X", 0.f);
        magnetometer_basis_z.j = pt.get<float>("Calibration.Magnetometer.BasisZ.Y", 0.f);
        magnetometer_basis_z.k = pt.get<float>("Calibration.Magnetometer.BasisZ.Z", 1.f);

        magnetometer_extents.i = pt.get<float>("Calibration.Magnetometer.Extents.X", 0.f);
        magnetometer_extents.j = pt.get<float>("Calibration.Magnetometer.Extents.Y", 0.f);
        magnetometer_extents.k = pt.get<float>("Calibration.Magnetometer.Extents.Z", 0.f);

        magnetometer_identity.i = pt.get<float>("Calibration.Magnetometer.Identity.X", 0.f);
        magnetometer_identity.j = pt.get<float>("Calibration.Magnetometer.Identity.Y", 0.f);
        magnetometer_identity.k = pt.get<float>("Calibration.Magnetometer.Identity.Z", 0.f);

        magnetometer_fit_error= pt.get<float>("Calibration.Magnetometer.Error", 0.f);
		magnetometer_variance= pt.get<float>("Calibration.Magnetometer.Variance", magnetometer_variance);

		position_variance_exp_fit_a= pt.get<float>("Calibration.Position.VarianceExpFitA", position_variance_exp_fit_a);
		position_variance_exp_fit_b= pt.get<float>("Calibration.Position.VarianceExpFitB", position_variance_exp_fit_b);

		orientation_variance= pt.get<float>("Calibration.Orientation.Variance", orientation_variance);

		mean_update_time_delta= pt.get<float>("Calibration.Time.MeanUpdateTime", mean_update_time_delta);

		orientation_filter_type= pt.get<std::string>("OrientationFilter.FilterType", orientation_filter_type);

		position_filter_type= pt.get<std::string>("PositionFilter.FilterType", position_filter_type);
        max_velocity= pt.get<float>("PositionFilter.MaxVelocity", max_velocity);

		tracking_color_id= static_cast<eCommonTrackingColorID>(readTrackingColor(pt));

		hand= pt.get<std::string>("hand", hand);
    }
    else
    {
        SERVER_LOG_WARNING("PSMoveControllerConfig") << 
            "Config version " << version << " does not match expected version " << 
            PSMoveControllerConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

void
PSMoveControllerConfig::getMagnetometerEllipsoid(struct EigenFitEllipsoid *out_ellipsoid) const
{
    out_ellipsoid->center =
        Eigen::Vector3f(magnetometer_center.i, magnetometer_center.j, magnetometer_center.k);
    out_ellipsoid->extents =
        Eigen::Vector3f(magnetometer_extents.i, magnetometer_extents.j, magnetometer_extents.k);
    out_ellipsoid->basis.col(0) =
        Eigen::Vector3f(magnetometer_basis_x.i, magnetometer_basis_x.j, magnetometer_basis_x.k);
    out_ellipsoid->basis.col(1) =
        Eigen::Vector3f(magnetometer_basis_y.i, magnetometer_basis_y.j, magnetometer_basis_y.k);
    out_ellipsoid->basis.col(2) =
        Eigen::Vector3f(magnetometer_basis_z.i, magnetometer_basis_z.j, magnetometer_basis_z.k);
    out_ellipsoid->error= magnetometer_fit_error;
}

// -- PSMoveControllerInputState -----
PSMoveControllerInputState::PSMoveControllerInputState()
{
    clear();
}

void PSMoveControllerInputState::clear()
{
    CommonControllerState::clear();

    RawSequence = 0;
    RawTimeStamp = 0;

    DeviceType = PSMove;

    Triangle = CommonControllerState::Button_UP;
    Circle = CommonControllerState::Button_UP;
    Cross = CommonControllerState::Button_UP;
    Square = CommonControllerState::Button_UP;
    Select = CommonControllerState::Button_UP;
    Start = CommonControllerState::Button_UP;
    PS = CommonControllerState::Button_UP;
    Move = CommonControllerState::Button_UP;
    Trigger = CommonControllerState::Button_UP;

    TriggerValue= 0;

    CalibratedAccel = {{
        {{0, 0, 0}}, 
        {{0, 0, 0}} 
    }};
    CalibratedGyro = {{
        {{0, 0, 0}}, 
        {{0, 0, 0}}
    }};
    CalibratedMag = {{0, 0, 0}};

    TempRaw= 0;
}

void PSMoveControllerInputState::parseDataInput(
	const PSMoveControllerConfig *config,
	const PSMoveControllerModelPID model,
	const PSMoveDataInputCommon *previous_hid_packet,
	const PSMoveDataInputCommon *current_hid_input)
{
    // https://github.com/nitsch/moveonpc/wiki/Input-report
        
    // Buttons
	unsigned int prev_button_bitmask = previous_hid_packet != nullptr ? make_psmove_button_bitmask(previous_hid_packet) : 0;
    AllButtons = make_psmove_button_bitmask(current_hid_input);       
    Triangle = getButtonState(AllButtons, prev_button_bitmask, Btn_TRIANGLE);
    Circle = getButtonState(AllButtons, prev_button_bitmask, Btn_CIRCLE);
    Cross = getButtonState(AllButtons, prev_button_bitmask, Btn_CROSS);
    Square = getButtonState(AllButtons, prev_button_bitmask, Btn_SQUARE);
    Select = getButtonState(AllButtons, prev_button_bitmask, Btn_SELECT);
    Start = getButtonState(AllButtons, prev_button_bitmask, Btn_START);
    PS = getButtonState(AllButtons, prev_button_bitmask, Btn_PS);
    Move = getButtonState(AllButtons, prev_button_bitmask, Btn_MOVE);
    Trigger = getButtonState(AllButtons, prev_button_bitmask, Btn_T);

	if (model == _psmove_controller_ZCM2)
		TriggerValue = current_hid_input->trigger;
	else
		TriggerValue = (current_hid_input->trigger + current_hid_input->trigger2) / 2; // TODO: store each frame separately
    BatteryValue = (current_hid_input->battery);

    // Update raw and calibrated accelerometer and gyroscope state
    {
        // Access raw Accel and Gyro state from the DataInput struct as a byte array
        char* data = (char *)current_hid_input;

        // Extract Accelerometer and Gyroscope readings into in a set of two update frames.
        // Note: The double brackets are an oddity of C++11 static array initialization.
        std::array<std::array<std::array<int, 3>, 2>, 2> ag_raw_xyz = {{
            {{ {{ 0, 0, 0 }}, {{ 0, 0, 0 }} }},
            {{ {{ 0, 0, 0 }}, {{ 0, 0, 0 }} }}
        }};
        std::array<std::array<std::array<float, 3>, 2>, 2> ag_calibrated_xyz = {{
            {{ {{ 0, 0, 0 }}, {{ 0, 0, 0 }} }},
            {{ {{ 0, 0, 0 }}, {{ 0, 0, 0 }} }}
        }};
        std::array<int, 2> sensorOffsets = {{
            offsetof(PSMoveDataInputCommon, aXlow),
            offsetof(PSMoveDataInputCommon, gXlow)
        }};

		if (model == _psmove_controller_ZCM2)
		{
			for (std::array<int, 2>::size_type s_ix = 0; s_ix != sensorOffsets.size(); s_ix++) //accel, gyro
			{
				for (int d_ix = 0; d_ix < 3; d_ix++)  //x, y, z
				{
					// Offset into PSMoveDataInput
					const int totalOffset = sensorOffsets[s_ix] + 2 * d_ix;

					// Extract the raw signed 16-bit sensor value from the PSMoveDataInput packet
					const int raw_val = decode16bitTwosCompliment(data, totalOffset);

					// Get the calibration parameters for this sensor value
					const float k = config->cal_ag_xyz_kbd[s_ix][d_ix][0]; // calibration scale
					const float b = config->cal_ag_xyz_kbd[s_ix][d_ix][1]; // calibration offset
					const float d = config->cal_ag_xyz_kbd[s_ix][d_ix][2]; // calibration drift
					const float calibrated_val= (static_cast<float>(raw_val) - d)*k + b;

					// Save the raw sensor value
					// Frame 0 and Frame 1 are the same
					ag_raw_xyz[s_ix][0][d_ix] = raw_val;
					ag_raw_xyz[s_ix][1][d_ix] = raw_val;

					// Compute the calibrated sensor value
					// Frame 0 and Frame 1 are the same
					ag_calibrated_xyz[s_ix][0][d_ix] = calibrated_val;
					ag_calibrated_xyz[s_ix][1][d_ix] = calibrated_val;
				}
			}
		}
		else
		{
	        std::array<int, 2> frameOffsets = {{ 0, 6 }};

			for (std::array<int, 2>::size_type s_ix = 0; s_ix != sensorOffsets.size(); s_ix++) //accel, gyro
			{
				for (std::array<int, 2>::size_type f_ix = 0; f_ix != frameOffsets.size(); f_ix++) //older, newer
				{
					for (int d_ix = 0; d_ix < 3; d_ix++)  //x, y, z
					{
						// Offset into PSMoveDataInput
						const int totalOffset = sensorOffsets[s_ix] + frameOffsets[f_ix] + 2 * d_ix;

						// Extract the raw signed 16-bit sensor value from the PSMoveDataInput packet
						const int raw_val = decode16bitUnsignedToSigned(data, totalOffset);

						// Get the calibration parameters for this sensor value
						const float k = config->cal_ag_xyz_kbd[s_ix][d_ix][0]; // calibration scale
						const float b = config->cal_ag_xyz_kbd[s_ix][d_ix][1]; // calibration offset
						const float d = config->cal_ag_xyz_kbd[s_ix][d_ix][2]; // calibration drift
						const float calibrated_val= (static_cast<float>(raw_val) - d)*k + b;

						// Save the raw sensor value
						ag_raw_xyz[s_ix][f_ix][d_ix] = raw_val;

						// Compute the calibrated sensor value
						ag_calibrated_xyz[s_ix][f_ix][d_ix] = calibrated_val;
					}
				}
			}
		}

        RawAccel = ag_raw_xyz[0];
        RawGyro = ag_raw_xyz[1];

        CalibratedAccel = ag_calibrated_xyz[0];
        CalibratedGyro = ag_calibrated_xyz[1];
    }

	// Update the raw and calibrated magnetometer
	if (model == _psmove_controller_ZCM2)
	{
		PSMoveDataInputZCM2 *InDataZCM2= (PSMoveDataInputZCM2 *)current_hid_input;

		// ZCM2 - Doesn't have a magnetometer
		RawMag[0] = RawMag[1] = RawMag[2]= 0;
		CalibratedMag[0]= CalibratedMag[1] = CalibratedMag[2] = 0.f;

		// Other
		RawSequence = (InDataZCM2->buttons4 & 0x0F);
		Battery = static_cast<CommonControllerState::BatteryLevel>(InDataZCM2->battery);
		RawTimeStamp = InDataZCM2->timelow | (InDataZCM2->timehigh << 8);
		TempRaw = (InDataZCM2->temphigh << 4) | ((InDataZCM2->templow & 0xF0) >> 4);
	}
	else // ZCM1 - Might have a magnetometer
    {
		PSMoveDataInputZCM1 *InDataZCM1= (PSMoveDataInputZCM1 *)current_hid_input;

        Eigen::Vector3f raw_mag, calibrated_mag;
        EigenFitEllipsoid ellipsoid;

        // Save the Raw Magnetometer sensor value (signed 12-bit values)
        RawMag[0] = TWELVE_BIT_SIGNED(((InDataZCM1->templow_mXhigh & 0x0F) << 8) | InDataZCM1->mXlow);
        // The magnetometer y-axis is flipped compared to the accelerometer and gyro.
        // Flip it back around to get it into the same space.
        RawMag[1] = -TWELVE_BIT_SIGNED((InDataZCM1->mYhigh << 4) | (InDataZCM1->mYlow_mZhigh & 0xF0) >> 4);
        RawMag[2] = TWELVE_BIT_SIGNED(((InDataZCM1->mYlow_mZhigh & 0x0F) << 8) | InDataZCM1->mZlow);

        // Project the raw magnetometer sample into the space of the ellipsoid
        raw_mag = 
            Eigen::Vector3f(
                static_cast<float>(RawMag[0]), static_cast<float>(RawMag[1]), static_cast<float>(RawMag[2]));
        config->getMagnetometerEllipsoid(&ellipsoid);
        calibrated_mag= eigen_alignment_project_point_on_ellipsoid_basis(raw_mag, ellipsoid);

        // Normalize the projected measurement (any deviation from unit length is error)
		eigen_vector3f_normalize_with_default(calibrated_mag, Eigen::Vector3f(0.f, 1.f, 0.f));

        // Save the calibrated magnetometer vector
        CalibratedMag[0] = calibrated_mag.x();
        CalibratedMag[1] = calibrated_mag.y();
        CalibratedMag[2] = calibrated_mag.z();

		// Other
		RawSequence = (InDataZCM1->buttons4 & 0x0F);
		Battery = static_cast<CommonControllerState::BatteryLevel>(InDataZCM1->battery);
		RawTimeStamp = InDataZCM1->timelow | (InDataZCM1->timehigh << 8);
		TempRaw = (InDataZCM1->temphigh << 4) | ((InDataZCM1->templow_mXhigh & 0xF0) >> 4);
    }
}

// -- PSMoveControllerOutputState -----
PSMoveControllerOutputState::PSMoveControllerOutputState()
{
	clear();
}

void PSMoveControllerOutputState::clear()
{
	r = g = b= 0;
	rumble= 0;
}

// -- PSMove Controller -----
PSMoveController::PSMoveController()
    : SupportsMagnetometer(false)
	, m_HIDPacketProcessor(nullptr)
	, m_controllerListener(nullptr)
{
	HIDDetails.vendor_id = -1;
	HIDDetails.product_id = -1;
    HIDDetails.Handle = nullptr;
    HIDDetails.Handle_addr = nullptr;
	memset(&m_cachedInputState, 0, sizeof(PSMoveControllerInputState));
	memset(&m_cachedOutputState, 0, sizeof(PSMoveControllerOutputState));
}

PSMoveController::~PSMoveController()
{
    if (getIsOpen())
    {
        SERVER_LOG_ERROR("~PSMoveController") << "Controller deleted without calling close() first!";
    }

	if (m_HIDPacketProcessor)
	{
		delete m_HIDPacketProcessor;
	}
}

bool PSMoveController::open()
{
	ControllerDeviceEnumerator enumerator(ControllerDeviceEnumerator::CommunicationType_HID, CommonControllerState::PSMove);
	while (enumerator.is_valid())
    {
		if (open(&enumerator))
		{
			return true;
		}

		enumerator.next();
    }

    return false;

}

bool PSMoveController::open(PSMoveControllerModelPID model)
{
	ControllerDeviceEnumerator enumerator(ControllerDeviceEnumerator::CommunicationType_HID, CommonControllerState::PSMove);
	while (enumerator.is_valid())
    {
		if (enumerator.get_product_id() == (int)model)
		{
			if (open(&enumerator))
			{
				return true;
			}
		}

		enumerator.next();
    }

    return false;
}

bool PSMoveController::open(
	const DeviceEnumerator *enumerator)
{
	const ControllerDeviceEnumerator *pEnum = static_cast<const ControllerDeviceEnumerator *>(enumerator);
	const char *cur_dev_path= pEnum->get_path();

    bool success= false;

    if (getIsOpen())
    {
        SERVER_LOG_WARNING("PSMoveController::open") << "PSMoveController(" << cur_dev_path << ") already open. Ignoring request.";
        success= true;
    }
    else
    {
        char cur_dev_serial_number[256];

        SERVER_LOG_INFO("PSMoveController::open") << "Opening PSMoveController(" << cur_dev_path << ")";

        if (pEnum->get_serial_number(cur_dev_serial_number, sizeof(cur_dev_serial_number)))
        {
            SERVER_LOG_INFO("PSMoveController::open") << "  with serial_number: " << cur_dev_serial_number;
        }
        else
        {
            cur_dev_serial_number[0]= '\0';
            SERVER_LOG_INFO("PSMoveController::open") << "  with EMPTY serial_number";
        }

		HIDDetails.vendor_id = pEnum->get_vendor_id();
		HIDDetails.product_id = pEnum->get_product_id();
        HIDDetails.Device_path = cur_dev_path;
    #ifdef _WIN32
        HIDDetails.Device_path_addr = HIDDetails.Device_path;
        HIDDetails.Device_path_addr.replace(HIDDetails.Device_path_addr.find("&col01#"), 7, "&col02#");
		//HIDDetails.Device_path_addr.replace(HIDDetails.Device_path_addr.find("&Col01#"), 7, "&Col02#");
        HIDDetails.Device_path_addr.replace(HIDDetails.Device_path_addr.find("&0000#"), 6, "&0001#");
        HIDDetails.Handle_addr = hid_open_path(HIDDetails.Device_path_addr.c_str());
        hid_set_nonblocking(HIDDetails.Handle_addr, 1);
    #endif
        HIDDetails.Handle = hid_open_path(HIDDetails.Device_path.c_str());
		// Perform blocking reads in the IMU thread
        hid_set_nonblocking(HIDDetails.Handle, 0);
         
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
			// Get the firmware revision being used
			bool bSaveConfig= loadFirmwareInfo();

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
                HIDDetails.Host_bt_addr= "00:00:00:00:00:00";
            }
    #endif
            if (getBTAddress(HIDDetails.Host_bt_addr, HIDDetails.Bt_addr))
            {
                // Load the config file
                std::string btaddr = HIDDetails.Bt_addr;
                std::replace(btaddr.begin(), btaddr.end(), ':', '_');
                cfg = PSMoveControllerConfig(btaddr);
                cfg.load();

                if (!IsBluetooth || !cfg.is_valid)
                {
                    if (!cfg.is_valid)
                    {
                        SERVER_LOG_ERROR("PSMoveController::open") << "PSMoveController(" << cur_dev_path << ") has invalid calibration. Reloading.";
                    }

                    // Load calibration from controller internal memory.
					if (getIsPS4Controller())
						loadCalibrationZCM2();
					else
						loadCalibrationZCM1();
                }

				// Always save the config back out in case some defaults changed
				bSaveConfig = true;

                success= true;
            }
            else
            {
                // If serial is still bad, maybe we have a disconnected
                // controller still showing up in hidapi
                SERVER_LOG_ERROR("PSMoveController::open") << "Failed to get bluetooth address of PSMoveController(" << cur_dev_path << ")";
                success= false;
            }

			// Create the sensor processor thread
			m_HIDPacketProcessor= new PSMoveHidPacketProcessor(cfg, (PSMoveControllerModelPID)HIDDetails.product_id);
			m_HIDPacketProcessor->start(HIDDetails.Handle, m_controllerListener);

			if (bSaveConfig)
			{
				cfg.save();
			}
        }
        else
        {
            SERVER_LOG_ERROR("PSMoveController::open") << "Failed to open PSMoveController(" << cur_dev_path << ")";
            success= false;
        }
    }

    return success;
}

void PSMoveController::close()
{
    if (getIsOpen())
    {
        SERVER_LOG_INFO("PSMoveController::close") << "Closing PSMoveController(" << HIDDetails.Device_path << ")";

		if (m_HIDPacketProcessor != nullptr)
		{
			// halt the HID packet processing thread
			m_HIDPacketProcessor->stop();
			delete m_HIDPacketProcessor;
			m_HIDPacketProcessor= nullptr;
		}

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
        SERVER_LOG_INFO("PSMoveController::close") << "PSMoveController(" << HIDDetails.Device_path << ") already closed. Ignoring request.";
    }
}

bool 
PSMoveController::setHostBluetoothAddress(const std::string &new_host_bt_addr)
{
    bool success= false;
    unsigned char bts[PSMOVE_BTADDR_SET_SIZE];

    memset(bts, 0, sizeof(bts));
    bts[0] = PSMove_Req_SetBTAddr;

    unsigned char addr[6];
    if (stringToPSMoveBTAddrUchar(new_host_bt_addr, addr, sizeof(addr)))
    {
        int res;

        /* Copy 6 bytes from addr into bts[1]..bts[6] */
        memcpy(&bts[1], addr, sizeof(addr));

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
                SERVER_LOG_ERROR("PSMoveController::setBTAddress") << "HID ERROR: " << hidapi_err_mbs;
            }            
        }
    }
    else
    {
        SERVER_LOG_ERROR("PSMoveController::setBTAddress") << "Malformed address: " << new_host_bt_addr;
    }

    return success;
}

bool
PSMoveController::setTrackingColorID(const eCommonTrackingColorID tracking_color_id)
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
PSMoveController::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    // Down-cast the enumerator so we can use the correct get_path.
    const ControllerDeviceEnumerator *pEnum = static_cast<const ControllerDeviceEnumerator *>(enumerator);
    
    bool matches= false;

    if (pEnum->get_device_type() == CommonControllerState::PSMove)
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
PSMoveController::getIsBluetooth() const
{ 
    return IsBluetooth; 
}

bool
PSMoveController::getIsReadyToPoll() const
{
    return (getIsOpen() && getIsBluetooth());
}

long 
PSMoveController::getMaxPollFailureCount() const
{
	return cfg.max_poll_failure_count;
}

IDeviceInterface::ePollResult 
PSMoveController::poll()
{
	if (m_HIDPacketProcessor != nullptr)
	{
		int LastRawSequence= m_cachedInputState.RawSequence;

		m_HIDPacketProcessor->fetchLatestInputData(m_cachedInputState);

		if (m_cachedInputState.RawSequence != LastRawSequence)
		{
			return IDeviceInterface::_PollResultSuccessNewData;
		}
		else
		{
			return IDeviceInterface::_PollResultSuccessNoData;
		}
	}
	else
	{
		return IDeviceInterface::_PollResultFailure;
	}
}

std::string 
PSMoveController::getUSBDevicePath() const
{
    return HIDDetails.Device_path;
}

int
PSMoveController::getVendorID() const
{
	return HIDDetails.vendor_id;
}

int
PSMoveController::getProductID() const
{
	return HIDDetails.product_id;
}

std::string 
PSMoveController::getSerial() const
{
    return HIDDetails.Bt_addr;
}

std::string 
PSMoveController::getAssignedHostBluetoothAddress() const
{
    return HIDDetails.Host_bt_addr;
}

bool
PSMoveController::getIsOpen() const
{
    return (HIDDetails.Handle != nullptr);
}

bool
PSMoveController::getBTAddress(std::string& host, std::string& controller)
{
    bool success = false;

    if (IsBluetooth && !controller.empty() && !host.empty())
    {
        std::replace(controller.begin(), controller.end(), '-', ':');
        std::transform(controller.begin(), controller.end(), controller.begin(), ::tolower);
        
        std::replace(host.begin(), host.end(), '-', ':');
        std::transform(host.begin(), host.end(), host.begin(), ::tolower);
        
        //TODO: If the third entry is not : and length is PSMOVE_BTADDR_SIZE
//        std::stringstream ss;
//        ss << controller.substr(0, 2) << ":" << controller.substr(2, 2) <<
//        ":" << controller.substr(4, 2) << ":" << controller.substr(6, 2) <<
//        ":" << controller.substr(8, 2) << ":" << controller.substr(10, 2);
//        controller = ss.str();
        
        success = true;
    }
    else
    {
        unsigned char btg[PSMOVE_BTADDR_GET_MAX_SIZE+1];
        unsigned char ctrl_char_buff[PSMOVE_BTADDR_SIZE];
        unsigned char host_char_buff[PSMOVE_BTADDR_SIZE];
        
        int res;
        int expected_res = getIsPS4Controller() ? PSMOVE_BTADDR_GET_ZCM2_SIZE : PSMOVE_BTADDR_GET_ZCM1_SIZE;
        unsigned char *p = btg;
        
        memset(btg, 0, sizeof(btg));
        btg[0] = PSMove_Req_GetBTAddr;
        //Unlike the firmware request, this request always returns the request
        //key in the first byte.
        p = btg + 1;
        
        //Only in Windows does the res value reflect that the first byte is the request key.
#if defined (_WIN32)
        expected_res++;
#endif
        
        /* _WIN32 only has move->handle_addr for getting bluetooth address. */
        if (HIDDetails.Handle_addr) {
            res = hid_get_feature_report(HIDDetails.Handle_addr, btg, expected_res+1);
        }
        else {
            res = hid_get_feature_report(HIDDetails.Handle, btg, expected_res+1);
        }
        
        if (res > 0)
		{
            memcpy(ctrl_char_buff, p, PSMOVE_BTADDR_SIZE);
            controller = PSMoveBTAddrUcharToString(ctrl_char_buff);
            
            memcpy(host_char_buff, p + 9, PSMOVE_BTADDR_SIZE);
            host = PSMoveBTAddrUcharToString(host_char_buff);

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
                SERVER_LOG_ERROR("PSMoveController::getBTAddress") << "HID ERROR: " << hidapi_err_mbs;
            }
        }
    }

    return success;
}

void
PSMoveController::loadCalibrationZCM1()
{
    bool is_valid= true;

    // The calibration provides a scale factor (k) and offset (b) to convert
    // raw accelerometer and gyroscope readings into something more useful.
    // https://github.com/nitsch/moveonpc/wiki/Calibration-data

    // calibration data storage - loaded from file (if bluetooth) or usb
    char usb_calibration[PSMOVE_ZCM1_CALIBRATION_BLOB_SIZE];

    // Default values are pass-through (raw*1 + 0)
    cfg.cal_ag_xyz_kbd = {{ 
            {{ {{ 1, 0, 0 }}, {{ 1, 0, 0 }}, {{ 1, 0, 0 }} }}, 
            {{ {{ 1, 0, 0 }}, {{ 1, 0, 0 }}, {{ 1, 0, 0 }} }} 
        }};

    // Load the calibration from the controller itself.
    unsigned char hid_cal[PSMOVE_ZCM1_CALIBRATION_BLOB_SIZE];

    for (int block_index=0; is_valid && block_index<3; block_index++) 
    {
        unsigned char cal[PSMOVE_CALIBRATION_SIZE+1]; // +1 for report id at start
        int dest_offset;
        int src_offset;
        int expected_res = PSMOVE_CALIBRATION_SIZE;
#if defined(_WIN32)
        expected_res++;
#endif

        memset(cal, 0, sizeof(cal));
        cal[0] = PSMove_Req_GetCalibration;

        int res = hid_get_feature_report(HIDDetails.Handle, cal, sizeof(cal));

        if (res == expected_res)
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
                dest_offset = PSMOVE_CALIBRATION_SIZE;
                src_offset = 2;
            }
            else if (cal[1] == 0x82) 
            {
                /* Third block */
                dest_offset = 2*PSMOVE_CALIBRATION_SIZE - 2;
                src_offset = 2;
            }
            else
            {
                SERVER_LOG_ERROR("PSMoveController::loadCalibration") 
                    << "Unexpected calibration block id(0x" << std::hex << std::setfill('0') << std::setw(2) << cal[1] 
                    << " on block #" << block_index;
                is_valid= false;
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

            is_valid= false;
        }

        if (is_valid)
        {
            memcpy(hid_cal+dest_offset, cal+src_offset, sizeof(cal)-src_offset-1);
        }
    }

    if (is_valid)
    {
        memcpy(usb_calibration, hid_cal, PSMOVE_ZCM1_CALIBRATION_BLOB_SIZE);
        
        // Convert the calibration blob into constant & offset for each accel dim.
        std::vector< std::vector<int> > dim_lohi = { {1, 3}, {5, 4}, {2, 0} };
        std::vector<int> res_lohi(2, 0);
        int dim_ix = 0;
        int lohi_ix = 0;
        for (dim_ix = 0; dim_ix < 3; dim_ix++)
        {
            for (lohi_ix = 0; lohi_ix < 2; lohi_ix++)
            {
                res_lohi[lohi_ix] = decode16bitUnsignedToSigned(usb_calibration, 0x04 + 6*dim_lohi[dim_ix][lohi_ix] + 2*dim_ix);
            }
            cfg.cal_ag_xyz_kbd[0][dim_ix][0] = 2.f / (float)(res_lohi[1] - res_lohi[0]);
            cfg.cal_ag_xyz_kbd[0][dim_ix][1] = -(cfg.cal_ag_xyz_kbd[0][dim_ix][0] * (float)res_lohi[0]) - 1.f;
			// No drift for accelerometer
        }
        
        // Convert the calibration blob into constant for each gyro dim.
        float factor = (float)(2.0 * M_PI * 80.0) / 60.0f;
        for (dim_ix = 0; dim_ix < 3; dim_ix++)
        {
            cfg.cal_ag_xyz_kbd[1][dim_ix][0] = factor / (float)(decode16bitUnsignedToSigned(usb_calibration, 0x46 + 10 * dim_ix)
                                                    - decode16bitUnsignedToSigned(usb_calibration, 0x2a + 2*dim_ix));
            // No offset for gyroscope
			// No drift for gyroscope
        }
    }

    cfg.is_valid= is_valid;
}

void
PSMoveController::loadCalibrationZCM2()
{
    bool is_valid= true;

    // The calibration provides a scale factor (k) and offset (b) to convert
    // raw accelerometer and gyroscope readings into something more useful.
    // https://github.com/nitsch/moveonpc/wiki/Calibration-data

    // calibration data storage - loaded from file (if bluetooth) or usb
    char usb_calibration[PSMOVE_ZCM2_CALIBRATION_BLOB_SIZE];

    // Default values are pass-through ((raw-0)*1 + 0)
    cfg.cal_ag_xyz_kbd = {{ 
            {{ {{ 1, 0, 0 }}, {{ 1, 0, 0 }}, {{ 1, 0, 0 }} }}, 
            {{ {{ 1, 0, 0 }}, {{ 1, 0, 0 }}, {{ 1, 0, 0 }} }} 
        }};

    // Load the calibration from the controller itself.
    unsigned char hid_cal[PSMOVE_ZCM2_CALIBRATION_BLOB_SIZE];

    for (int block_index=0; is_valid && block_index<2; block_index++) 
    {
        unsigned char cal[PSMOVE_CALIBRATION_SIZE+1]; // +1 for report id at start
        int dest_offset;
        int src_offset;
        int expected_res = PSMOVE_CALIBRATION_SIZE;
#if defined(_WIN32)
        expected_res++;
#endif

        memset(cal, 0, sizeof(cal));
        cal[0] = PSMove_Req_GetCalibration;

        int res = hid_get_feature_report(HIDDetails.Handle, cal, sizeof(cal));

        if (res == expected_res)
        {
            if (cal[1] == 0x00) 
            {
                /* First block */
                dest_offset = 0;
                src_offset = 0;
            }
            else if (cal[1] == 0x81) 
            {
                /* Second block */
                dest_offset = PSMOVE_CALIBRATION_SIZE;
                src_offset = 2;
            }
            else
            {
                SERVER_LOG_ERROR("PSMoveController::loadCalibration") 
                    << "Unexpected calibration block id(0x" << std::hex << std::setfill('0') << std::setw(2) << cal[1] 
                    << " on block #" << block_index;
                is_valid= false;
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

            is_valid= false;
        }

        if (is_valid)
        {
            memcpy(hid_cal+dest_offset, cal+src_offset, sizeof(cal)-src_offset-1);
        }
    }

    if (is_valid)
    {
        memcpy(usb_calibration, hid_cal, PSMOVE_ZCM2_CALIBRATION_BLOB_SIZE);
        
        // Convert the calibration blob into gain and bias values for each axis.
		// The calibration stores 6 accelerometer recorded when the controller is
		// aligned on each axis:
		// Example values
		// 4137,   -19,    85
		//-4059,    13,    -6
		//   65,  4093,    25
		//   16, -4103,    53
		//   -3,     5,  4152
		//   82,   -28, -4086
        std::vector< std::vector<int> > accel_dim_lohi = { {1, 0}, {3, 2}, {5, 4} };
        for (int dim_ix = 0; dim_ix < 3; dim_ix++)
        {
			// Read the low and high values for each axis
	        std::vector<short> res_lohi(2, 0);
            for (int lohi_ix = 0; lohi_ix < 2; lohi_ix++)
            {
                res_lohi[lohi_ix] = decode16bitSigned(usb_calibration, 0x02 + 6*accel_dim_lohi[dim_ix][lohi_ix] + 2*dim_ix);
            }

			// Compute the gain value as 1/((max-min)/2)
            cfg.cal_ag_xyz_kbd[0][dim_ix][0] = 2.f / (float)(res_lohi[1] - res_lohi[0]);

			// Compute the bias as the amount the min reading differs from -1.0g
            cfg.cal_ag_xyz_kbd[0][dim_ix][1] = -(cfg.cal_ag_xyz_kbd[0][dim_ix][0] * (float)res_lohi[0]) - 1.f;

			// No Drift
			cfg.cal_ag_xyz_kbd[0][dim_ix][2] = 0.f;
        }
        
        // Convert the calibration blob into constant for each gyro axis.
		// The calibration stores 6 gyroscope readings when the controller is spun at 90RPM
		// on the +X, +Y, +Z, -X, -Y, and -Z axis.
		
		// Example values:
		// 6706,   -18,    58
		//  -50,  6820,    52
		// -114,    28,  6830
		//-6826,    46,   -34
		//   42, -6792,   -28
		//  129,     0, -6808

		// There is also one vector containing drift compensation values.
		// When the controller is sitting still each axis will report a small constant non-zero value.
		// Subtracting these drift values per-frame will give more accurate gyro readings.

		// Example drift values:
		// -3	13	11

        const float k_rpm_to_rad_per_sec = (2.0f * k_real_pi) / 60.0f;
		const float k_calibration_rpm= 90.f;
		const float y_hi= k_calibration_rpm * k_rpm_to_rad_per_sec;
		const float y_low= -k_calibration_rpm * k_rpm_to_rad_per_sec;
		std::vector< std::vector<int> > gyro_dim_lohi = { {3, 0}, {4, 1}, {5, 2} };
        for (int dim_ix = 0; dim_ix < 3; dim_ix++)
        {
			// Read the low and high values for each axis
			std::vector<short> res_lohi(2, 0);
            for (int lohi_ix = 0; lohi_ix < 2; lohi_ix++)
            {
                res_lohi[lohi_ix] = decode16bitSigned(usb_calibration, 0x30 + 6*gyro_dim_lohi[dim_ix][lohi_ix] + 2*dim_ix);
            }
			short raw_gyro_drift= decode16bitSigned(usb_calibration, 0x26 + 2*dim_ix);

			// Compute the gain value (the slope of the gyro reading/angular speed line)
			const float x_low= res_lohi[0];
			const float x_hi= res_lohi[1];
			const float m= (y_hi - y_low) / (x_hi - x_low);
            cfg.cal_ag_xyz_kbd[1][dim_ix][0] = m;

            // Compute the bias value (the y-intercept of the gyro reading/angular speed line)
			cfg.cal_ag_xyz_kbd[1][dim_ix][1] = y_hi - m*x_hi;

			//TODO:
			// Store off the drift value
			//cfg.cal_ag_xyz_kbd[1][dim_ix][2] = raw_gyro_drift;

			// The final result:
			// rad/s = (raw_gyro_value-drift)*gain + bias
        }
    }

    cfg.is_valid= is_valid;
}

bool
PSMoveController::loadFirmwareInfo()
{
	bool bFirmwareInfoValid = false;
    
    if (!getIsBluetooth())
    {
        unsigned char buf[PSMOVE_FW_GET_SIZE+1];
        int res;
        int expected_res = sizeof(buf) - 1;
        unsigned char *p = buf;

        memset(buf, 0, sizeof(buf));
        buf[0] = PSMove_Req_GetFirmwareInfo;

        res = hid_get_feature_report(HIDDetails.Handle, buf, sizeof(buf));

        /**
        * The Bluetooth report contains the Report ID as additional first byte
        * while the USB report does not. So we need to check the current connection
        * type in order to determine the correct offset for reading from the report
        * buffer.
        **/
	
		expected_res += 1;
		p = buf + 1;
        
        if (res == expected_res)
        {
            // NOTE: Each field in the report is stored in Big-Endian byte order
            cfg.firmware_version = (p[0] << 8) | p[1];
            cfg.firmware_revision = (p[2] << 8) | p[3];
            cfg.bt_firmware_version = (p[4] << 8) | p[5];
            
            bFirmwareInfoValid = true;
        }
	}
    
	return bFirmwareInfoValid;
}

bool
PSMoveController::enableDFUMode()
{
	unsigned char buf[10];
	int res;
	char mode_magic_val;

	if (getIsBluetooth())
	{
		mode_magic_val = 0x43;
	}
	else
	{
		mode_magic_val = 0x42;
	}

	memset(buf, 0, sizeof(buf));
	buf[0] = PSMove_Req_SetDFUMode;
	buf[1] = mode_magic_val;
	res = hid_send_feature_report(HIDDetails.Handle, buf, sizeof(buf));

	return (res == sizeof(buf));
}

void PSMoveController::setControllerListener(IControllerListener *listener)
{
	m_controllerListener= listener;
}

const CommonDeviceState * 
PSMoveController::getState(
	int lookBack) const
{
    return &m_cachedInputState;
}

const std::tuple<unsigned char, unsigned char, unsigned char>
PSMoveController::getColour() const
{	
    return std::make_tuple(m_cachedOutputState.r, m_cachedOutputState.g, m_cachedOutputState.b);
}

void 
PSMoveController::getTrackingShape(CommonDeviceTrackingShape &outTrackingShape) const
{
    outTrackingShape.shape_type= eCommonTrackingShapeType::Sphere;
    outTrackingShape.shape.sphere.radius_cm = PSMOVE_TRACKING_BULB_RADIUS;
}

bool
PSMoveController::getTrackingColorID(eCommonTrackingColorID &out_tracking_color_id) const
{
	bool bSuccess = false;

	if (getIsOpen() && getIsBluetooth())
	{
		out_tracking_color_id = cfg.tracking_color_id;
		bSuccess = true;
	}

	return bSuccess;
}

float PSMoveController::getIdentityForwardDegrees() const
{
	// Controller model points down the -Z axis when it has the identity orientation
	return 270.f;
}

float PSMoveController::getPredictionTime() const
{
	return getConfig()->prediction_time;
}

bool PSMoveController::getWasSystemButtonPressed() const
{
    const PSMoveControllerInputState *psmove_state= static_cast<const PSMoveControllerInputState *>(getState());
    bool bWasPressed= false;

    if (psmove_state != nullptr)
    {
        bWasPressed= psmove_state->PS == CommonControllerState::Button_PRESSED;
    }

    return bWasPressed;
}

float
PSMoveController::getTempCelsius() const
{
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
        if (temperature_lookup[i] > m_cachedInputState.TempRaw) {
            return (float)(i - 10);
        }
    }
    
    return 70;
}

// Setters
bool
PSMoveController::setLED(unsigned char r, unsigned char g, unsigned char b)
{
    bool success = true;

    if (m_HIDPacketProcessor != nullptr &&
		(m_cachedOutputState.r != r) || (m_cachedOutputState.g != g) || (m_cachedOutputState.b != b))
    {
        m_cachedOutputState.r = r;
        m_cachedOutputState.g = g;
        m_cachedOutputState.b = b;
		m_HIDPacketProcessor->postOutputState(m_cachedOutputState);
        
		success= true;
    }

    return success;
}

bool
PSMoveController::setRumbleIntensity(unsigned char value)
{
    bool success = true;
    if (m_cachedOutputState.rumble != value)
    {
        m_cachedOutputState.rumble = value;
		m_HIDPacketProcessor->postOutputState(m_cachedOutputState);

        success = true;
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

// -- private helper functions -----
static std::string
PSMoveBTAddrUcharToString(const unsigned char* addr_buff)
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
stringToPSMoveBTAddrUchar(const std::string &addr, unsigned char *addr_buff, const int addr_buf_size)
{
    bool success= false;

    if (addr.length() >= 17 && addr_buf_size >= 6)
    {
        const char *raw_string= addr.c_str();
        unsigned int octets[6];

        success= 
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
            for (int i= 0; i < 6; ++i)
            {
                addr_buff[i]= ServerUtility::int32_to_int8_verify(octets[i]);
            }
        }
    }

    return success;
}

static short
decode16bitSigned(char *data, int offset)
{
    unsigned short low = data[offset] & 0xFF;
    unsigned short high = (data[offset+1]) & 0xFF;
    return (short)(low | (high << 8));
}

static int
decode16bitUnsignedToSigned(char *data, int offset)
{
    unsigned char low = data[offset] & 0xFF;
    unsigned char high = (data[offset+1]) & 0xFF;
    return (low | (high << 8)) - 0x8000;
}

static int
decode16bitTwosCompliment(char *data, int offset)
{
    unsigned char low = data[offset] & 0xFF;
    unsigned char high = (data[offset+1]) & 0xFF;
    int value= (low | (high << 8));

	return (value & 0x8000) ? (-(~value & 0xFFFF) + 1) : value;
}

inline enum CommonControllerState::ButtonState
getButtonState(unsigned int buttons, unsigned int lastButtons, int buttonMask)
{
    return (enum CommonControllerState::ButtonState)((((lastButtons & buttonMask) > 0) << 1) + ((buttons & buttonMask)>0));
}

inline unsigned int
make_psmove_button_bitmask(const PSMoveDataInputCommon *hid_packet)
{
	return
		(hid_packet->buttons2) | (hid_packet->buttons1 << 8) |
		((hid_packet->buttons3 & 0x01) << 16) | ((hid_packet->buttons4 & 0xF0) << 13);
}

inline bool hid_error_mbs(hid_device *dev, char *out_mb_error, size_t mb_buffer_size)
{
    return ServerUtility::convert_wcs_to_mbs(hid_error(dev), out_mb_error, mb_buffer_size);
}

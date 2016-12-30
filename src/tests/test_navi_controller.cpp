#include "PSNaviController.h"
#include "ServerLog.h"
#include "USBDeviceManager.h"
#include "hidapi.h"
#include "stdio.h"
#include "gamepad/Gamepad.h"
#include <string>

// For sleep
#ifdef _WIN32
#include <cstdlib>
#else
#include <unistd.h>
#endif

int main()
{
    log_init("info");

	Gamepad_init();
	Gamepad_processEvents();

    if (hid_init() == -1)
    {
        printf("Failed to initialize hidapi\n");
        return -1;
    }

	// Manages all control and bulk transfer requests in another thread
	USBDeviceManager usb_device_manager(_USBApiType_LibUSB);
	if (!usb_device_manager.startup())
	{
		printf("Failed to initialize usb device manager\n");
		return -1;
	}

    PSNaviController navi;

	printf("Attempting to open first Navi Controller...\n");
	if (navi.open())
	{
		printf("Successfully opened Navi Controller!\n");

		const PSNaviControllerState *navi_state = nullptr;

		if (navi.getIsBluetooth())
		{
			printf("Connected via Bluetooth\n");
		}
		else
		{
			std::string hostBTAddress = navi.getAssignedHostBluetoothAddress();

			printf("Connected via USB\n");
			printf("Assigned BT host address: %s\n", hostBTAddress.c_str());
		}		

		navi.poll();
		navi_state = static_cast<const PSNaviControllerState *>(navi.getState());

		while (navi_state->PS != CommonControllerState::Button_DOWN)
		{
			Gamepad_processEvents();

			navi.poll();
			navi_state = static_cast<const PSNaviControllerState *>(navi.getState());

			printf("\rButtons: L1=%d,L2=%d,L3=%d,O=%d,X=%d,Up=%d,Dn=%d,Lt=%d,Rt=%d,Trg=%3d,SX=%3d,SY=%3d", 
				navi_state->L1 == CommonControllerState::Button_DOWN ? 1 : 0,
				navi_state->L2 == CommonControllerState::Button_DOWN ? 1 : 0, 
				navi_state->L3 == CommonControllerState::Button_DOWN ? 1 : 0,
				navi_state->Circle == CommonControllerState::Button_DOWN ? 1 : 0,
				navi_state->Cross == CommonControllerState::Button_DOWN ? 1 : 0, 
				navi_state->DPad_Up == CommonControllerState::Button_DOWN ? 1 : 0,
				navi_state->DPad_Down == CommonControllerState::Button_DOWN ? 1 : 0,
				navi_state->DPad_Left == CommonControllerState::Button_DOWN ? 1 : 0,
				navi_state->DPad_Right == CommonControllerState::Button_DOWN ? 1 : 0,
				navi_state->Trigger,
				navi_state->Stick_XAxis,
				navi_state->Stick_YAxis);

			usb_device_manager.update();
#ifdef _WIN32
			_sleep(5); // 5 msec
#else
			usleep(5000);
#endif
		}

		printf("Closing Navi Controller...\n");
        navi.close();
	}
	else
	{
		printf("Failed to open Navi Controller...\n");
	}
    
	usb_device_manager.shutdown();

    // Tear-down hid api
    hid_exit();

	// Shutdown the gamepad api
	Gamepad_shutdown();

	log_dispose();
    
    return 0;
}
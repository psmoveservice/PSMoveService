#include "PSDualShock4Controller.h"
#include "ServerLog.h"
#include "hidapi.h"
#include "stdio.h"
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

    if (hid_init() == -1)
    {
        printf("Failed to initialize hidapi\n");
        return -1;
    }

    PSDualShock4Controller ds4;

	printf("Attempting to open first DS4 Controller...\n");
	if (ds4.open())
	{
		printf("Successfully opened DS4 Controller!\n");

		const PSDualShock4ControllerState *ds4_state = nullptr;

		if (ds4.getIsBluetooth())
		{
			printf("Connected via Bluetooth\n");

			ds4.poll();
			ds4_state = static_cast<const PSDualShock4ControllerState *>(ds4.getState());

			while (ds4_state->PS != CommonControllerState::Button_DOWN)
			{
				ds4.poll();
				ds4_state = static_cast<const PSDualShock4ControllerState *>(ds4.getState());

				char *lastButtonName= "        ";
				if (ds4_state->L1 == CommonControllerState::Button_DOWN) 
					lastButtonName= "L1      ";
				if (ds4_state->L2 == CommonControllerState::Button_DOWN)
					lastButtonName= "L2      "; 
				if (ds4_state->L3 == CommonControllerState::Button_DOWN)
					lastButtonName= "L3      ";
				if (ds4_state->R1 == CommonControllerState::Button_DOWN)
					lastButtonName= "R1      ";
				if (ds4_state->R2 == CommonControllerState::Button_DOWN)
					lastButtonName= "R2      "; 
				if (ds4_state->R3 == CommonControllerState::Button_DOWN)
					lastButtonName= "R3      ";
				if (ds4_state->Circle == CommonControllerState::Button_DOWN)
					lastButtonName= "Circle  ";
				if (ds4_state->Cross == CommonControllerState::Button_DOWN)
					lastButtonName= "Cross   ";
				if (ds4_state->Square == CommonControllerState::Button_DOWN)
					lastButtonName= "Square  ";
				if (ds4_state->Triangle == CommonControllerState::Button_DOWN)
					lastButtonName= "Triangle";
				if (ds4_state->DPad_Up == CommonControllerState::Button_DOWN)
					lastButtonName= "Up      ";
				if (ds4_state->DPad_Down == CommonControllerState::Button_DOWN)
					lastButtonName= "Down    ";
				if (ds4_state->DPad_Left == CommonControllerState::Button_DOWN)
					lastButtonName= "Left    ";
				if (ds4_state->DPad_Right == CommonControllerState::Button_DOWN)
					lastButtonName= "Right   ";
				if (ds4_state->Share == CommonControllerState::Button_DOWN)
					lastButtonName= "Share   ";
				if (ds4_state->Options == CommonControllerState::Button_DOWN)
					lastButtonName= "Options ";
				if (ds4_state->PS == CommonControllerState::Button_DOWN)
					lastButtonName= "PS      ";
				if (ds4_state->TrackPadButton == CommonControllerState::Button_DOWN)
					lastButtonName= "TrackPad";				

				printf("\rButtons: LT=%.2f,LX=%.2f,LY=%.2f,RT=%.2f,RX=%.2f,RY=%.2f,Btns=%s", 
					ds4_state->LeftTrigger,
					ds4_state->LeftAnalogX,
					ds4_state->LeftAnalogY,
					ds4_state->RightTrigger,
					ds4_state->RightAnalogX,
					ds4_state->RightAnalogY,
					lastButtonName);
	#ifdef _WIN32
				_sleep(5); // 5 msec
	#else
				usleep(5000);
	#endif
			}
		}
		else
		{
			std::string hostBTAddress = ds4.getAssignedHostBluetoothAddress();

			printf("Connected via USB\n");
			printf("Assigned BT host address: %s\n", hostBTAddress.c_str());
		}		

		printf("Closing DS4 Controller...\n");
        ds4.close();
	}
	else
	{
		printf("Failed to open DS4 Controller...\n");
	}
    
    // Tear-down hid api
    hid_exit();

	log_dispose();
    
    return 0;
}
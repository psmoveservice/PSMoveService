#include <iostream>
#include <iomanip>
#include "PSMoveController.h"
#include "ServerLog.h"

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
        std::cerr << "Failed to initialize hidapi" << std::endl;
        return -1;
    }

    PSMoveController psmove;

	std::cout << "Opening PSMoveController..." << std::endl;
	if (psmove.open())
	{
        const PSMoveControllerState *psmstate= nullptr;

        psmove.poll();
        psmstate= static_cast<const PSMoveControllerState *>(psmove.getState());

		unsigned char r = 255;
		unsigned char g = 0;
		unsigned char b = 0;
        
        psmove.setRumbleIntensity(255);

		while (psmove.getIsBluetooth() && psmstate->Move != CommonControllerState::Button_DOWN)
		{
            psmove.poll();
            psmstate= static_cast<const PSMoveControllerState *>(psmove.getState());

			psmove.setRumbleIntensity(psmstate->TriggerValue);

			r = (r + 23) % 255;
			g = (g + 47) % 255;
			b = (b + 53) % 255;
			psmove.setLED(r, g, b);
			int mychar = 0x09;
			int myw = 4;
			std::cout <</* '\r' <<//*
				"# " << std::setw(myw) << std::left << psmstate->RawSequence <<
				" A(1): " <<
				std::setw(myw) << std::right << psmstate->RawAccel[0][0] << "," <<
				std::setw(myw) << std::right << psmstate->RawAccel[0][1] << "," <<
				std::setw(myw) << std::right << psmstate->RawAccel[0][2] <<
				"; A(2): " <<
				std::setw(myw) << std::right << psmstate->RawAccel[1][0] << "," <<
				std::setw(myw) << std::right << psmstate->RawAccel[1][1] << "," <<
				std::setw(myw) << std::right << psmstate->RawAccel[1][2] <<
				"; G(1): " <<
				std::setw(myw) << std::right << psmstate->RawGyro[0][0] << "," <<
				std::setw(myw) << std::right << psmstate->RawGyro[0][1] << "," <<
				std::setw(myw) << std::right << psmstate->RawGyro[0][2] <<
				"; G(2): " <<
				std::setw(myw) << std::right << psmstate->RawGyro[1][0] << "," <<
				std::setw(myw) << std::right << psmstate->RawGyro[1][1] << "," <<
				std::setw(myw) << std::right << psmstate->RawGyro[1][2] <<
				"; M: " <<
				std::setw(myw) << std::right << psmstate->RawMag[0] << "," <<
				std::setw(myw) << std::right << psmstate->RawMag[1] << "," <<
				std::setw(myw) << std::right << psmstate->RawMag[2] <<
				std::flush
				<<//*/
				"\nBatt: " << (psmstate->Battery)
				<< "; BattVal: " << int(psmstate->BatteryValue)
				<< "; Trig: " << int(psmstate->TriggerValue);

#ifdef _WIN32
			_sleep(50); // 5 msec
#else
            usleep(5000);
#endif
		}
        std::cout << std::endl;

        psmove.close();
	}
    
    // Tear-down hid api
    hid_exit();

	log_dispose();
    
    return 0;
}
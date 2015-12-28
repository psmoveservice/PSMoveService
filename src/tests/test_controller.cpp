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
        PSMoveControllerState psmstate;

        psmove.poll();
        psmove.getState(&psmstate);

		unsigned char r = 255;
		unsigned char g = 0;
		unsigned char b = 0;
        
        psmove.setRumbleIntensity(255);

		while (psmove.getIsBluetooth() && psmstate.Move != CommonControllerState::Button_DOWN)
		{
            psmove.poll();
            psmove.getState(&psmstate);

			psmove.setRumbleIntensity(psmstate.Trigger);

			r = (r + 23) % 255;
			g = (g + 47) % 255;
			b = (b + 53) % 255;
			psmove.setLED(r, g, b);

			int myw = 4;
			std::cout << '\r' <<
				"# " << std::setw(myw) << std::left << psmstate.Sequence <<
				" A(1): " <<
				std::setw(myw) << std::right << psmstate.Accel[0][0] << "," <<
				std::setw(myw) << std::right << psmstate.Accel[0][1] << "," <<
				std::setw(myw) << std::right << psmstate.Accel[0][2] <<
				"; A(2): " <<
				std::setw(myw) << std::right << psmstate.Accel[1][0] << "," <<
				std::setw(myw) << std::right << psmstate.Accel[1][1] << "," <<
				std::setw(myw) << std::right << psmstate.Accel[1][2] <<
				"; G(1): " <<
				std::setw(myw) << std::right << psmstate.Gyro[0][0] << "," <<
				std::setw(myw) << std::right << psmstate.Gyro[0][1] << "," <<
				std::setw(myw) << std::right << psmstate.Gyro[0][2] <<
				"; G(2): " <<
				std::setw(myw) << std::right << psmstate.Gyro[1][0] << "," <<
				std::setw(myw) << std::right << psmstate.Gyro[1][1] << "," <<
				std::setw(myw) << std::right << psmstate.Gyro[1][2] <<
				"; M: " <<
				std::setw(myw) << std::right << psmstate.Mag[0] << "," <<
				std::setw(myw) << std::right << psmstate.Mag[1] << "," <<
				std::setw(myw) << std::right << psmstate.Mag[2] <<
				std::flush;

#ifdef _WIN32
			_sleep(5); // 5 msec
#else
            usleep(5000);
#endif
		}
        std::cout << std::endl;

        psmove.close();
	}
    
    // Tear-down hid api
    hid_exit();
    
    return 0;
}
#include <iostream>
#include <iomanip>
#include "PSMoveController.h"

// For sleep
#ifdef __GNUC__
#include <unistd.h>
#endif

#ifdef _WIN32
#include <cstdlib>
#endif

int main()
{
	std::cout << "Creating new PSMoveController..." << std::endl;
    PSMoveController psmove;
	if (psmove.isOpen())
	{
		PSMoveState psmstate = psmove.getState();

		unsigned char r = 0;
		unsigned char g = 0;
		unsigned char b = 0;

		while (psmstate.Move == 0)
		{
			psmstate = psmove.getState();
			psmove.setRumbleIntensity(psmstate.Trigger);

			r = (r + 23) % 255;
			g = (g + 47) % 255;
			b = (b + 53) % 255;
			psmove.setLED(r, g, b);

			int myw = 4;
			std::cout << '\r' <<
				" A(1): " <<
				std::setw(myw) << std::right << psmstate.accel.oldFrame[0] << "," <<
				std::setw(myw) << std::right << psmstate.accel.oldFrame[1] << "," <<
				std::setw(myw) << std::right << psmstate.accel.oldFrame[2] <<
				"; A(2): " <<
				std::setw(myw) << std::right << psmstate.accel.newFrame[0] << "," <<
				std::setw(myw) << std::right << psmstate.accel.newFrame[1] << "," <<
				std::setw(myw) << std::right << psmstate.accel.newFrame[2] <<
				"; G(1): " <<
				std::setw(myw) << std::right << psmstate.gyro.oldFrame[0] << "," <<
				std::setw(myw) << std::right << psmstate.gyro.oldFrame[1] << "," <<
				std::setw(myw) << std::right << psmstate.gyro.oldFrame[2] <<
				"; G(2): " <<
				std::setw(myw) << std::right << psmstate.gyro.newFrame[0] << "," <<
				std::setw(myw) << std::right << psmstate.gyro.newFrame[1] << "," <<
				std::setw(myw) << std::right << psmstate.gyro.newFrame[2] <<
				"; M: " <<
				std::setw(myw) << std::right << psmstate.mag[0] << "," <<
				std::setw(myw) << std::right << psmstate.mag[1] << "," <<
				std::setw(myw) << std::right << psmstate.mag[2] <<
				std::flush;

#ifdef __GNUC__
			sleep(0.005); // 5 msec
#endif

#ifdef _WIN32
			_sleep(5); // 5 msec
#endif
		}
	}
    

    
    return 0;
}
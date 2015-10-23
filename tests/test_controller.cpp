#include <iostream>
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
        int rumble = 255;
		std::cout << "Attempting to setRumbleValue(" << rumble << ")..." << std::endl;
		psmove.setRumbleValue(rumble);
	}
    
#ifdef __GNUC__
    sleep(1); // one second
#endif
    
#ifdef _WIN32
    _sleep(1000); // one second
#endif
    
    return 0;
}
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
	int result = -1;

    log_init("info");

	if (hid_init() != -1)
	{
		PSMoveController psmove;

		SERVER_LOG_INFO("main") << "Opening first PSMoveController...";

		if (psmove.open())
		{
			SERVER_LOG_INFO("main") << "Checking for USB connection...";

			if (!psmove.getIsBluetooth())
			{
				SERVER_LOG_INFO("main") << "Enabling USB DFU mode...";

				if (psmove.enableDFUMode())
				{
					SERVER_LOG_INFO("main") << "Success!";
					SERVER_LOG_INFO("main") << "Unplug the controller to reset it";
				}
				else
				{
					SERVER_LOG_ERROR("main") << "Failed to enable DFU mode over USB.";
				}
			}
			else
			{
				SERVER_LOG_ERROR("main") << "Controller connected via bluetooth!";
				SERVER_LOG_ERROR("main") << "Turn all all controllers and only plug in one controller via USB.";
			}

			psmove.close();
		}
		else
		{
			SERVER_LOG_ERROR("main") << "Failed to open controller";
		}

		// Tear-down hid api
		hid_exit();
	}
	else
    {
		SERVER_LOG_ERROR("main") << "Failed to initialize hidapi";
    }

	log_dispose();
    
    return result;
}
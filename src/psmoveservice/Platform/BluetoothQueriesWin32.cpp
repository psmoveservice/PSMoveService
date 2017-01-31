#ifndef _WIN32
#error "Only include this file in windows builds!"
#endif // _WIN32

// -- includes -----
#include "BluetoothQueries.h"
#include "ServerLog.h"
#include "ServerUtility.h"

#include <windows.h>
#include <bthsdpdef.h>
#include <bluetoothapis.h>
#include <assert.h>
#include <sstream>
#include <iomanip>

static std::string bluetooth_address_to_string(const BLUETOOTH_ADDRESS* bt_address);
static bool find_first_bluetooth_radio(HANDLE *hRadio);

//-- Queries -----
bool bluetooth_get_host_address(std::string &out_address)
{
	static std::string x_cachedHostBluetoothAddress= "";
    bool bSuccess = true;

	if (x_cachedHostBluetoothAddress.length() == 0)
	{
		HANDLE hRadio;

		if (find_first_bluetooth_radio(&hRadio) && hRadio != INVALID_HANDLE_VALUE)
		{
			SERVER_LOG_INFO("bluetooth_get_host_address") << "Found a bluetooth radio";
		}
		else
		{
			SERVER_LOG_ERROR("bluetooth_get_host_address") << "Failed to find a bluetooth radio";
			bSuccess = false;
		}

		if (bSuccess)
		{
			BLUETOOTH_RADIO_INFO radioInfo;
			memset(&radioInfo, 0, sizeof(BLUETOOTH_RADIO_INFO));
			radioInfo.dwSize = sizeof(BLUETOOTH_RADIO_INFO);

			DWORD result = BluetoothGetRadioInfo(hRadio, &radioInfo);

			if (result == ERROR_SUCCESS)
			{
				SERVER_LOG_INFO("bluetooth_get_host_address") << "Retrieved radio info";
				out_address = bluetooth_address_to_string(&radioInfo.address);
				x_cachedHostBluetoothAddress= out_address;
			}
			else
			{
				SERVER_LOG_ERROR("bluetooth_get_host_address")
					<< "Failed to retrieve radio info (Error Code: "
					<< std::hex << std::setfill('0') << std::setw(8) << result;
				bSuccess = false;
			}
		}
	}
	else
	{
		SERVER_LOG_INFO("bluetooth_get_host_address") << "Using cached radio info";
		out_address= x_cachedHostBluetoothAddress;
		bSuccess= true;
	}

    return bSuccess;
}

// -- helper methods -----
static std::string
bluetooth_address_to_string(const BLUETOOTH_ADDRESS* bt_address)
{
    std::ostringstream stream;

    for (int buff_ind = 5; buff_ind >= 0; buff_ind--)
    {
        stream << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(bt_address->rgBytes[buff_ind]);

        if (buff_ind > 0)
        {
            stream << ":";
        }
    }

    return stream.str();
}

static bool
find_first_bluetooth_radio(HANDLE *hRadio)
{
    bool success= false;
    assert(hRadio != nullptr);

    BLUETOOTH_FIND_RADIO_PARAMS radio_params;
    radio_params.dwSize = sizeof(BLUETOOTH_FIND_RADIO_PARAMS);

    HBLUETOOTH_RADIO_FIND hFind = BluetoothFindFirstRadio(&radio_params, hRadio);
    if (hFind) 
    {
        BluetoothFindRadioClose(hFind);
        success= true;
    }

    return success;
}
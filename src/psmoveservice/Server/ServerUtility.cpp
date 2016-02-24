// -- includes -----
#include "ServerUtility.h"
#include <wchar.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <locale>

// -- public methods -----
namespace ServerUtility
{
    unsigned char int32_to_int8_verify(int value)
    {
        assert(value >= 0 && value <= 255);
        return static_cast<unsigned char>(value);
    }

    bool convert_wcs_to_mbs(const wchar_t *wc_string, char *out_mb_serial, const size_t mb_buffer_size)
    {
        bool success= false;

        if (wc_string != nullptr)
        {
    #ifdef _WIN32
            size_t countConverted;
            const wchar_t *wcsIndirectString = wc_string;
            mbstate_t mbstate;

            success= wcsrtombs_s(
                &countConverted,
                out_mb_serial,
                mb_buffer_size,
                &wcsIndirectString,
                _TRUNCATE,
                &mbstate) == 0;
    #else
            success=
                wcstombs(
                    out_mb_serial, 
                    wc_string, 
                    mb_buffer_size) != static_cast<size_t>(-1);
    #endif
        }

        return success;
    }

    bool normalize_bluetooth_address(
        const char *addr, bool bLowercase, char separator, 
        char *result, size_t result_max_size)
    {
        bool bSuccess= true;
        size_t count = strlen(addr);

        if (count == 17 && result_max_size >= 18)
        {
            for (int i = 0; bSuccess && i<17; i++) 
            {
                if (addr[i] >= 'A' && addr[i] <= 'F' && i % 3 != 2) 
                {
                    if (bLowercase) 
                    {
                        result[i] = tolower(addr[i]);
                    } 
                    else 
                    {
                        result[i] = addr[i];
                    }
                } 
                else if (addr[i] >= '0' && addr[i] <= '9' && i % 3 != 2) 
                {
                    result[i] = addr[i];
                } 
                else if (addr[i] >= 'a' && addr[i] <= 'f' && i % 3 != 2) 
                {
                    if (bLowercase) 
                    {
                        result[i] = addr[i];
                    }
                    else 
                    {
                        result[i] = toupper(addr[i]);
                    }
                }
                else if ((addr[i] == ':' || addr[i] == '-') && i % 3 == 2) 
                {
                    result[i] = separator;
                }
                else
                {
                    bSuccess= false;
                }
            }
        }
        else
        {
            bSuccess= false;
        }

        // Make sure the address is null terminated
        if (bSuccess)
        {
            result[count] = '\0';
        }

        return bSuccess;
    }
};
// -- includes -----
#include "ServerUtility.h"
#include <wchar.h>
#include <assert.h>

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
            std::wcstombs(
                out_mb_serial, 
                wc_string, 
                mb_buffer_size) != static_cast<std::size_t>(-1);
    #endif

        return success;
    }
};
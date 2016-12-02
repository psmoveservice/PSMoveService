// -- includes -----
#include "ServerUtility.h"
#include <wchar.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdarg.h>
#include <locale>
#include <iostream>
#include <sstream>
#include <iomanip>

#if defined WIN32 || defined _WIN32 || defined WINCE
    #include <windows.h>
    #include <algorithm>

    #ifdef _MSC_VER
        #pragma warning (disable: 4996) // 'This function or variable may be unsafe': vsnprintf
        #define vsnprintf _vsnprintf
    #endif
#else
	#include <sys/time.h>
	#include <time.h>
	#if defined __MACH__ && defined __APPLE__
		#include <mach/mach.h>
		#include <mach/mach_time.h>
	#endif
    #define MILLISECONDS_TO_NANOSECONDS 1000000
#endif

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

    int format_string(char *buffer, size_t buffer_size, const char *format, ...)
    {
        // Bake out the text string
        va_list args;
        va_start(args, format);
        int chars_written = vsnprintf(buffer, buffer_size, format, args);
        buffer[buffer_size - 1] = 0;
        va_end(args);

        return chars_written;
    }

    bool bluetooth_cstr_address_normalize(
        const char *addr, bool bLowercase, char separator, 
        char *result, size_t result_max_size)
    {
        bool bSuccess= true;
        size_t source_count = strlen(addr);
        size_t result_count= 0;

        if ((source_count == 17 || source_count == 12) && result_max_size >= 18)
        {
            int valid_octet_count= 0;

            for (size_t source_index = 0; 
                source_index<source_count && result_count<result_max_size;
                source_index++) 
            {
                char result_character = '\0';

                if (addr[source_index] >= 'A' && addr[source_index] <= 'F') 
                {
                    if (bLowercase) 
                    {
                        result_character = tolower(addr[source_index]);
                    } 
                    else 
                    {
                        result_character = addr[source_index];
                    }
                } 
                else if (addr[source_index] >= '0' && addr[source_index] <= '9') 
                {
                    result_character = addr[source_index];
                } 
                else if (addr[source_index] >= 'a' && addr[source_index] <= 'f') 
                {
                    if (bLowercase) 
                    {
                        result_character = addr[source_index];
                    }
                    else 
                    {
                        result_character = toupper(addr[source_index]);
                    }
                }

                if (result_character != '\0')
                {
                    result[result_count] = result_character;
                    ++result_count;

                    if (separator != '\0' && 
                        (result_count + 1) % 3 == 0)
                    {
                        if (valid_octet_count < 5)
                        {
                            result[result_count] = separator;
                        }
                        else
                        {
                            result[result_count] = '\0';
                        }

                        ++valid_octet_count;
                        ++result_count;
                    }
                }
            }

            // Make sure we parsed all all of the characters for a full 6 octet address
            bSuccess = (valid_octet_count == 6);
        }
        else
        {
            bSuccess= false;
        }

        return bSuccess;
    }

    std::string bluetooth_byte_addr_to_string(const unsigned char* addr_buff)
    {
        // http://stackoverflow.com/questions/11181251/saving-hex-values-to-a-c-string
        std::ostringstream stream;
        int buff_ind = 5;
        for (buff_ind = 5; buff_ind >= 0; buff_ind--)
        {
            stream << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(addr_buff[buff_ind]);
            if (buff_ind > 0)
            {
                stream << ":";
            }
        }
        return stream.str();
    }

    bool bluetooth_string_address_to_bytes(const std::string &addr, unsigned char *addr_buff, const int addr_buf_size)
    {
        bool success = false;

        if (addr.length() >= 17 && addr_buf_size >= 6)
        {
            const char *raw_string = addr.c_str();
            unsigned int octets[6];

            success =
                sscanf(raw_string, "%x:%x:%x:%x:%x:%x",
                &octets[5],
                &octets[4],
                &octets[3],
                &octets[2],
                &octets[1],
                &octets[0]) == 6;
            //TODO: Make safe (sscanf_s is not portable)

            if (success)
            {
                for (int i = 0; i < 6; ++i)
                {
                    addr_buff[i] = ServerUtility::int32_to_int8_verify(octets[i]);
                }
            }
        }

        return success;
    }
	
#if defined WIN32 || defined _WIN32 || defined WINCE
    const DWORD MS_VC_EXCEPTION = 0x406D1388;

    #pragma pack(push,8)
    typedef struct tagTHREADNAME_INFO
    {
        DWORD dwType;		// Must be 0x1000.
        LPCSTR szName;		// Pointer to name (in user addr space).
        DWORD dwThreadID;	// Thread ID (-1=caller thread).
        DWORD dwFlags;		// Reserved for future use, must be zero.
    } THREADNAME_INFO;
    #pragma pack(pop)

    void set_current_thread_name(const char* thread_name)
    {
        THREADNAME_INFO info;
        info.dwType = 0x1000;
        info.szName = thread_name;
        info.dwThreadID = GetCurrentThreadId();
        info.dwFlags = 0;

        __try
        {
            RaiseException(MS_VC_EXCEPTION, 0, sizeof(info) / sizeof(ULONG_PTR), (ULONG_PTR*)&info);
        }
        __except (EXCEPTION_EXECUTE_HANDLER)
        {
        }
    }
#else
    void set_current_thread_name(const char* threadName)
    {
        // Not sure how to implement this on linux/osx, so left empty...
    }
#endif

    void sleep_ms(int milliseconds)
    {
#ifdef _MSC_VER
        Sleep(milliseconds);
#else
        nanosleep((const struct timespec[]){ {0, milliseconds*MILLISECONDS_TO_NANOSECONDS} }, NULL);
#endif
    }	
};
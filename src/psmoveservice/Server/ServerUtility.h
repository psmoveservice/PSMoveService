#ifndef SERVER_UTILITY_H
#define SERVER_UTILITY_H

#include "stdlib.h" // size_t
#include <string>

namespace ServerUtility
{
    template <typename t_index>
    inline bool is_index_valid(const t_index index, const t_index count)
    {
        return index >= 0 && index < count;
    }

    unsigned char int32_to_int8_verify(int value);
    bool convert_wcs_to_mbs(const wchar_t *wc_string, char *out_mb_serial, const size_t mb_buffer_size);

    /// Formats a string into the given target buffer
    /// \param buffer The target buffer to write in to
    /// \param buffer_size The max number of bytes that can be written to the buffer
    /// \param format The formatting string that will be written to the buffer
    /// \return The number of characters successfully written
    int format_string(char *buffer, size_t buffer_size, const char *format, ...);

    /// Converts the given bluetooth string into standarized format. Ex) "XX-XX-XX-XX-XX-XX" to "xx:xx:xx:xx:xx:xx"
    /// \param addr The given bluetooth address we want to convers
    /// \param bLowercase true if we want the result to be in lower case or not
    /// \param separator The separator character to use (typically ':')
    /// \param result The buffer for the resulting string
    /// \param result_max_size The capacity of the result buffer. Must be at least 17 bytes.
    /// \return false if the result buffer is too small or there was a parsing error
    bool bluetooth_cstr_address_normalize(
        const char *addr, bool bLowercase, char separator, 
        char *result, size_t result_max_size);

    /// Converts the given 6-octet raw byte bluetooth address to a std::string, with ':' as the octet separator
    /// \param addr_buff An array whose first 6 bytes contain the 6 octets of a bluetooth address (reverse order)
    /// \return The bluetooth address as a string
    std::string bluetooth_byte_addr_to_string(const unsigned char* addr_buff);

    /// Converts the given string containing an address of the form "%x:%x:%x:%x:%x:%x" into a 6-octet byte array
    /// \param addr The source bluetooth string
    /// \param addr_buff The target object buffer
    /// \param addr_buf_size The size of the target buffer
    /// \return true of the string could be parse and the target array could hold the octets
    bool bluetooth_string_address_to_bytes(const std::string &addr, unsigned char *addr_buff, const int addr_buf_size);
};

#endif // SERVER_REQUEST_HANDLER_H
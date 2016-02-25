#ifndef SERVER_UTILITY_H
#define SERVER_UTILITY_H

#include "stdlib.h" // size_t

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

    /// Coverts the given bluetooth string into standarized format. Ex) "XX-XX-XX-XX-XX-XX" to "xx:xx:xx:xx:xx:xx"
    /// \param addr The given bluetooth address we want to convers
    /// \param bLowercase true if we want the result to be in lower case or not
    /// \param separator The separator character to use (typically ':')
    /// \param result The buffer for the resulting string
    /// \param result_max_size The capacity of the result buffer. Must be at least 17 bytes.
    /// \return false if the result buffer is too small or there was a parsing error
    bool normalize_bluetooth_address(
        const char *addr, bool bLowercase, char separator, 
        char *result, size_t result_max_size);
};

#endif // SERVER_REQUEST_HANDLER_H
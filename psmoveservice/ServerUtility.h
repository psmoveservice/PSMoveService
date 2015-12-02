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
};

#endif // SERVER_REQUEST_HANDLER_H
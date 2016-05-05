#ifndef LOGGER_H
#define LOGGER_H

#include <cstdio>

//-- macros -----
#define Log_INFO(section, format, ...) \
    fprintf(stdout, "INFO [" section "] " format "\n", ## __VA_ARGS__)

#define Log_ERROR(section, format, ...) \
    fprintf(stderr, "ERROR [" section "] " format "\n", ## __VA_ARGS__)

#endif // LOGGER_H
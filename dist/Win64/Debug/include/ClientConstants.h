#ifndef CLIENT_CONSTANTS_H
#define CLIENT_CONSTANTS_H

//-- includes -----
#include "SharedConstants.h"

/** 
\addtogroup PSMoveClient_CAPI 
@{ 
*/

//-- constants -----
#define PSM_DEFAULT_TIMEOUT 1000 // milliseconds

// The length of a controller serial string: "xx:xx:xx:xx:xx:xx\0"
#define PSMOVESERVICE_CONTROLLER_SERIAL_LEN  18

// The max length of the service version string
#define PSMOVESERVICE_MAX_VERSION_STRING_LEN 32

// Defines a standard _PAUSE function
#if __cplusplus >= 199711L  // if C++11
    #include <thread>
    #define _PAUSE(ms) (std::this_thread::sleep_for(std::chrono::milliseconds(ms)))
#elif defined(_WIN32)       // if windows system
    #include <windows.h>
    #define _PAUSE(ms) (Sleep(ms))
#else                       // assume this is Unix system
    #include <unistd.h>
    #define _PAUSE(ms) (usleep(1000 * ms))
#endif

//-- macros -----
#if defined(__cplusplus) && defined(HAS_PROTOCOL_ACCESS)
    #define GET_PSMOVEPROTOCOL_REQUEST(handle) \
    reinterpret_cast<const PSMoveProtocol::Request *>(handle)
    #define GET_PSMOVEPROTOCOL_RESPONSE(handle) \
    reinterpret_cast<const PSMoveProtocol::Response *>(handle)
    #define GET_PSMOVEPROTOCOL_EVENT(handle) \
    reinterpret_cast<const PSMoveProtocol::Response *>(handle) // events are a special case of responses
#endif // defined(__cplusplus) && defined(HAS_PROTOCOL_ACCESS)

/** 
@} 
*/ 

#endif // CLIENT_CONSTANTS_H

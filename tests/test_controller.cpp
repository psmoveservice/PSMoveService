#include <iostream>
#include "PSMoveController.h"

// For sleep
#ifdef __GNUC__
#include <unistd.h>
#endif

#ifdef _WIN32
#include <cstdlib>
#endif

int main()
{
    PSMoveController psmove;
    psmove.setRumbleValue(255);
    
#ifdef __GNUC__
    sleep(1); // one second
#endif
    
#ifdef _WIN32
    _sleep(1000); // one second
#endif
    
    return 0;
}
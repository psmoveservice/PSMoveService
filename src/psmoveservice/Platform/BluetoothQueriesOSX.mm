// -- includes -----
#include "BluetoothQueries.h"

#include <IOBluetooth/objc/IOBluetoothHostController.h>
#import <Foundation/NSAutoreleasePool.h>

// -- private prototypes -----
// Function declarations for IOBluetooth private API
extern "C"
{
    void IOBluetoothPreferenceSetControllerPowerState(int powered);
    int IOBluetoothPreferenceGetControllerPowerState();
};

bool macosx_bluetooth_set_powered(bool bPowered);
bool macosx_get_btaddr(const bool bEnsurePowered, char *result, size_t max_result_size);

// -- interface -----
bool bluetooth_get_host_address(std::string &out_address)
{
    bool bSuccess= false;
    
    NSAutoreleasePool *pool = [[NSAutoreleasePool alloc] init];
    
    char hostBTAddress[32];
    if (macosx_get_btaddr(false, hostBTAddress, sizeof(hostBTAddress)))
    {
        out_address= hostBTAddress;
        bSuccess= true;
    }
    
    [pool release];
    
    return bSuccess;
}

// -- private methods -----
bool
macosx_bluetooth_set_powered(bool bPowered)
{
    bool bSuccess= true;
    
    // Inspired by blueutil from Frederik Seiffert <ego@frederikseiffert.de>
    int desiredPowerState= bPowered ? 1 : 0;
    int currentPowerState = IOBluetoothPreferenceGetControllerPowerState();
    
    IOBluetoothPreferenceSetControllerPowerState(desiredPowerState);
    
    // Wait a bit for Bluetooth to be (de-)activated
    usleep(2000000);
    
    if (IOBluetoothPreferenceGetControllerPowerState() != desiredPowerState)
    {
        /*
         From http://dmaclach.blogspot.com/2010/10/its-dead-jim.html
         "There is no documented way of checking to see if Bluetooth is on or not without displaying a user interface.
         If you call the SPIs:
         
         int IOBluetoothPreferenceGetControllerPowerState(void);
         void IOBluetoothPreferenceSetControllerPowerState(int);
         
         You can control the state of bluetooth, but it may lie to you in that if you call:
         IOBluetoothPreferenceSetControllerPowerState(1)
         
         Immediately followed by:
         IOBluetoothPreferenceGetControllerPowerState()
         
         The get will return 1 even though if you attempt to set up a connection without waiting "a little while" it will fail.
         The little while was ~250 ms on my notebook, and closer to ~400 ms on my desktop."
         */
        bSuccess= false;
    }
    
    return bSuccess;
}

bool
macosx_get_btaddr(const bool bEnsurePowered, char *result, size_t max_result_size)
{
    bool bSuccess= true;
    
    if (bEnsurePowered && !macosx_bluetooth_set_powered(true))
    {
        bSuccess= false;
    }
    
    IOBluetoothHostController *controller = nullptr;
    if (bSuccess)
    {
        controller = [IOBluetoothHostController defaultController];
        
        if (controller == nullptr)
        {
            bSuccess= false;
        }
    }
    
    NSString *addressString = nullptr;
    if (bSuccess)
    {
        addressString = [controller addressAsString];
        
        if (addressString == nullptr)
        {
            bSuccess= false;
        }
    }
    
    const char *addressUTF8String= nullptr;
    if (bSuccess)
    {
        addressUTF8String= [addressString UTF8String];
        
        if (addressUTF8String == nullptr)
        {
            bSuccess= false;
        }
    }
    
    if (bSuccess)
    {
        size_t address_length= strlen(addressUTF8String);
        
        if (address_length + 1 < max_result_size)
        {
            strncpy(result, addressUTF8String, max_result_size);
        }
        else
        {
            bSuccess= false;
        }
    }
    
    return bSuccess;
}
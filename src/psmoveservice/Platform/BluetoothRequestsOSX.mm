// A large chunk of this file was adapted from PSMoveAPI.
// Reproducing the license here:
/*
The PS Move API library is licensed under the terms of the license below.
However, some optional third party libraries might have a different license.
Be sure to read the README file for details on third party licenses.

====

Copyright (c) 2011, 2012 Thomas Perl <m@thp.io>
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

// -- includes -----
#include "BluetoothRequests.h"
#include "../Device/Manager/DeviceManager.h"
#include "../Server/PSMoveService.h"
#include "../Device/View/ServerDeviceView.h"
#include "../Device/View/ServerControllerView.h"
#include "../Server/ServerNetworkManager.h"
#include "../Server/ServerLog.h"
#include "../Server/ServerUtility.h"

#include "PSMoveProtocol.pb.h"
#include "PSMoveProtocolInterface.h"

#include <sstream>
#include <iomanip>
#include <chrono>

#include <IOBluetooth/objc/IOBluetoothHostController.h>

#import <Foundation/NSAutoreleasePool.h>
#include <pthread.h>
#import <libkern/OSAtomic.h>

// -- constants ----
static const char *k_invalid_bluetooth_host= "00:00:00:00:00:00";

/* Location for the plist file that we want to modify */
#define OSX_BT_CONFIG_PATH "/Library/Preferences/com.apple.Bluetooth"

/* How often to poll the bluetooth daemon to see if it's closed */
#define BLUED_CLOSE_POLL_INTERVAL_USEC 1000000 // 1 second = 1MM usecs

/* The number of times we attempt to poll the bluetooth daemon to see if it's closed */
#define BLUED_CLOSE_MAX_POLL_ATTEMPTS 5

// -- definitions -----
class BluetoothDeviceOperationState
{
public:
    enum eOperation
    {
        addBluetoothDevice,
        removeBluetoothDevice
    };

    enum eStatus
    {
        start,
        fetchOSVersion,
        fetchDevicePairStatus,
        shutdownBluetooth,
        modifyBluetoothDeviceListings,
        startupBluetooth,

        k_total_steps,
        success= k_total_steps,
        failed,
    };

    pthread_t worker_thread_id;

    void initialize(const char *addr, eOperation op)
    {
        strncpy(controllerAddress, addr, sizeof(controllerAddress));
        operation= op;

        main_thread_id= pthread_self();
        worker_thread_id= pthread_self();
        
        subStatus_WorkerThread= start;
        isCanceled_WorkerThread= 0;

        subStatus_MainThread= start;
    }

    const char *getControllerAddress() const
    {
        return static_cast<const char *>(controllerAddress);
    }

    eOperation getOperationType() const
    {
        return operation;
    }

    void setSubStatus_WorkerThread(eStatus newSubStatus)
    {
        assert(pthread_equal(pthread_self(), worker_thread_id) != 0);
        
        // Atomically set the new status on subStatus_WorkerThread
        OSAtomicCompareAndSwapInt(subStatus_WorkerThread, static_cast<int32_t>(newSubStatus), &subStatus_WorkerThread);
    }

    bool getIsCanceled_WorkerThread()
    {
        assert(pthread_equal(pthread_self(), worker_thread_id) != 0);

        // Atomically fetch the canceled flag from the WorkerThread (adding 0, then fetching)
        int32_t isCanceled= OSAtomicAdd32(0, &isCanceled_WorkerThread);

        return isCanceled > 0;
    }

    bool pollSubStatus_MainThread(eStatus &outNewSubStatus)
    {
        assert(pthread_equal(pthread_self(), main_thread_id) != 0);
        bool subStatusChanged= false;

        // Atomically fetch the status from the WorkerThread (adding 0, then fetching)
        int32_t newSubStatus= OSAtomicAdd32(0, &subStatus_WorkerThread);

        if (newSubStatus != subStatus_MainThread)
        {
            subStatus_MainThread= newSubStatus;
            outNewSubStatus= static_cast<eStatus>(newSubStatus);
            subStatusChanged= true;
        }

        return subStatusChanged;
    }

    bool getIsCanceled_MainThread()
    {
        assert(pthread_equal(pthread_self(), main_thread_id) != 0);

        // Atomically fetch the canceled flag from the MainThread (adding 0, then fetching)
        int32_t isCanceled= OSAtomicAdd32(0, &isCanceled_WorkerThread);

        return isCanceled > 0;
    }

    void setIsCanceled_MainThread()
    {
        assert(pthread_self() == main_thread_id);
        
        // Atomically set the new status on subStatus_WorkerThread
        OSAtomicCompareAndSwapInt(isCanceled_WorkerThread, 1, &isCanceled_WorkerThread);
    }

private:
    char controllerAddress[32];
    eOperation operation;

    pthread_t main_thread_id;

    volatile int32_t subStatus_WorkerThread;
    volatile int32_t isCanceled_WorkerThread;

    int32_t subStatus_MainThread;
};

// -- prototypes -----
static void *async_bluetooth_device_operation_worker(void *thread_data);

// defined in BluetoothQueriesOSX.mm
bool macosx_get_btaddr(const bool bEnsurePowered, char *result, size_t max_result_size);
bool macosx_bluetooth_set_powered(bool bPowered);

static bool macosx_blued_running();
static void macosx_killall_blued();
static bool macosx_run_admin_command(const char *cmd);
static bool macosx_blued_is_paired(char *btaddr, std::vector<std::string> &other_paired_btaddrs);
static bool macosx_get_minor_version(int &outMinorVersion);
static bool macosx_register_bluetooth_address(const char *controllerBTAddr);
static void macosx_unregister_all_bluetooth_addresses();

static void send_unpair_completed_notification_to_client(int connectionID, PSMoveProtocol::Response_ResultCode resultCode);
static void send_pair_completed_notification_to_client(int connectionID, PSMoveProtocol::Response_ResultCode resultCode);
static void send_progress_notification_to_client(int connectionID, int controllerID, int stepsCompleted, int totalSteps);

// -- AsyncBluetoothUnpairDeviceRequest -----
AsyncBluetoothUnpairDeviceRequest::AsyncBluetoothUnpairDeviceRequest(
    int connectionId,
    ServerControllerViewPtr controllerView)
    : AsyncBluetoothRequest(connectionId)
    , m_controllerView(controllerView)
    , m_internal_state(nullptr)
{
    m_internal_state= new BluetoothDeviceOperationState();
}

AsyncBluetoothUnpairDeviceRequest::~AsyncBluetoothUnpairDeviceRequest()
{
    delete ((BluetoothDeviceOperationState *)m_internal_state);
}

bool 
AsyncBluetoothUnpairDeviceRequest::start()
{
    bool success= true;
    const int controller_id= m_controllerView->getDeviceID();
    const std::string bt_address_string= m_controllerView->getSerial();

    if (!m_controllerView->getIsOpen() || m_controllerView->getIsBluetooth())
    {
        SERVER_LOG_ERROR("AsyncBluetoothUnpairDeviceRequest") 
            << "Controller " << controller_id 
            << " isn't an open USB device";
        success= false;
    }

    if (success && !m_controllerView->setHostBluetoothAddress(k_invalid_bluetooth_host))
    {
        SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to clear host address on controller";
        success= false;
    }

    if (success)
    {
        BluetoothDeviceOperationState *state= reinterpret_cast<BluetoothDeviceOperationState *>(m_internal_state);

        state->initialize(bt_address_string.c_str(), BluetoothDeviceOperationState::removeBluetoothDevice);

        if (pthread_create(&state->worker_thread_id, NULL, async_bluetooth_device_operation_worker, m_internal_state) != 0)
        {
            SERVER_LOG_ERROR("AsyncBluetoothUnpairDeviceRequest") << "Failed to start worker thread!";
            success= false;
        }
    }

    m_status = success ? AsyncBluetoothRequest::running : AsyncBluetoothRequest::failed;

    return success;
}

void 
AsyncBluetoothUnpairDeviceRequest::update()
{
    BluetoothDeviceOperationState *state= ((BluetoothDeviceOperationState *)m_internal_state);

    // Check the worker thread to see if the sub-status changed
    BluetoothDeviceOperationState::eStatus subStatus;
    if (state->pollSubStatus_MainThread(subStatus))
    {
        // Tell the client about the sub status change
        send_progress_notification_to_client(
            m_connectionId, 
            m_controllerView->getDeviceID(),
            static_cast<int>(subStatus), 
            BluetoothDeviceOperationState::k_total_steps);

        // See if the worker thread has completed it's work
        if (subStatus == BluetoothDeviceOperationState::success ||
            subStatus == BluetoothDeviceOperationState::failed)
        {
            if (subStatus == BluetoothDeviceOperationState::success)
            {
                m_status= AsyncBluetoothRequest::succeeded;

                // Tell the client about the result
                send_unpair_completed_notification_to_client(m_connectionId, PSMoveProtocol::Response_ResultCode_RESULT_OK);
            }
            else if (subStatus == BluetoothDeviceOperationState::failed)
            {
                m_status= AsyncBluetoothRequest::failed;

                // Tell the client about the result
                send_unpair_completed_notification_to_client(
                    m_connectionId, 
                    state->getIsCanceled_MainThread() 
                    ? PSMoveProtocol::Response_ResultCode_RESULT_CANCELED
                    : PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
            }

            // Wait for the thread to exit (it should be done at this point)
            pthread_join(state->worker_thread_id, nullptr);
        }
    }
}

void 
AsyncBluetoothUnpairDeviceRequest::cancel(AsyncBluetoothRequest::eCancelReason reason)
{
    BluetoothDeviceOperationState *state= ((BluetoothDeviceOperationState *)m_internal_state);

    state->setIsCanceled_MainThread();
}

AsyncBluetoothRequest::eStatusCode 
AsyncBluetoothUnpairDeviceRequest::getStatusCode()
{
    return m_status;
}

std::string 
AsyncBluetoothUnpairDeviceRequest::getDescription()
{
    std::ostringstream description;

    description << "[Unpair] Serial: " << m_controllerView->getSerial() << " Conn: " << m_connectionId;

    return description.str();
}

// -- AsyncBluetoothPairDeviceRequest -----
AsyncBluetoothPairDeviceRequest::AsyncBluetoothPairDeviceRequest(
    int connectionId,
    ServerControllerViewPtr controllerView)
    : AsyncBluetoothRequest(connectionId)
    , m_controllerView(controllerView)
    , m_internal_state(nullptr)
{
    m_internal_state= new BluetoothDeviceOperationState();
}

AsyncBluetoothPairDeviceRequest::~AsyncBluetoothPairDeviceRequest()
{
    delete ((BluetoothDeviceOperationState *)m_internal_state);
}

bool 
AsyncBluetoothPairDeviceRequest::start()
{
    bool bSuccess= true;
    const int controller_id= m_controllerView->getDeviceID();
    const std::string controllerBTAddress= m_controllerView->getSerial();
    char hostBTAddress[32];

    if (bSuccess && m_controllerView->getIsOpen() && m_controllerView->getIsBluetooth())
    {
        SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") 
            << "Controller " << controller_id 
            << " isn't an open USB device";
        bSuccess= false;
    }

    if (bSuccess)
    {
        NSAutoreleasePool *pool = [[NSAutoreleasePool alloc] init];
        
        if (!macosx_get_btaddr(true, hostBTAddress, sizeof(hostBTAddress)))
        {
            SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Can't determine host bluetooth address";
            bSuccess= false;
        }

        [pool release];
    }
    
    char normalizedHostBTAddress[32];
    if (!ServerUtility::bluetooth_cstr_address_normalize(
            hostBTAddress, false, ':',
            normalizedHostBTAddress, sizeof(normalizedHostBTAddress)))
    {
        SERVER_MT_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Malformed controller bluetooth address: " << hostBTAddress;
        bSuccess= false;
    }


    if (bSuccess && !m_controllerView->setHostBluetoothAddress(normalizedHostBTAddress))
    {
        SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to set new host address on controller: " << normalizedHostBTAddress;
        bSuccess= false;
    }

    if (bSuccess)
    {
        BluetoothDeviceOperationState *state= reinterpret_cast<BluetoothDeviceOperationState *>(m_internal_state);

        state->initialize(controllerBTAddress.c_str(), BluetoothDeviceOperationState::addBluetoothDevice);

        if (pthread_create(&state->worker_thread_id, NULL, async_bluetooth_device_operation_worker, m_internal_state) != 0)
        {
            SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to start worker thread!";
            bSuccess= false;
        }
    }

    m_status = bSuccess ? AsyncBluetoothRequest::running : AsyncBluetoothRequest::failed;

    return bSuccess;
}

void 
AsyncBluetoothPairDeviceRequest::update()
{
    BluetoothDeviceOperationState *state= ((BluetoothDeviceOperationState *)m_internal_state);

    // Check the worker thread to see if the sub-status changed
    BluetoothDeviceOperationState::eStatus subStatus;
    if (state->pollSubStatus_MainThread(subStatus))
    {
        // Tell the client about the sub status change
        send_progress_notification_to_client(
            m_connectionId, 
            m_controllerView->getDeviceID(),
            static_cast<int>(subStatus),
            BluetoothDeviceOperationState::k_total_steps);

        // See if the worker thread has completed it's work
        if (subStatus == BluetoothDeviceOperationState::success ||
            subStatus == BluetoothDeviceOperationState::failed)
        {
            if (subStatus == BluetoothDeviceOperationState::success)
            {
                m_status= AsyncBluetoothRequest::succeeded;

                // Tell the client about the result
                send_pair_completed_notification_to_client(m_connectionId, PSMoveProtocol::Response_ResultCode_RESULT_OK);
            }
            else if (subStatus == BluetoothDeviceOperationState::failed)
            {
                m_status= AsyncBluetoothRequest::failed;

                // Tell the client about the result
                send_pair_completed_notification_to_client(
                    m_connectionId, 
                    state->getIsCanceled_MainThread() 
                    ? PSMoveProtocol::Response_ResultCode_RESULT_CANCELED
                    : PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
            }

            // Wait for the thread to exit (it should be done at this point)
            pthread_join(state->worker_thread_id, nullptr);
        }
    }
}

void 
AsyncBluetoothPairDeviceRequest::cancel(AsyncBluetoothRequest::eCancelReason reason)
{
    BluetoothDeviceOperationState *state= ((BluetoothDeviceOperationState *)m_internal_state);

    state->setIsCanceled_MainThread();
}

AsyncBluetoothRequest::eStatusCode 
AsyncBluetoothPairDeviceRequest::getStatusCode()
{
    return m_status;
}

std::string 
AsyncBluetoothPairDeviceRequest::getDescription()
{
    std::ostringstream description;

    description << "[Pair] ID: " << m_controllerView->getDeviceID() << " Conn: " << m_connectionId;

    return description.str();
}

//-- Worker Thread -----
static void *
async_bluetooth_device_operation_worker(void *thread_data)
{
    BluetoothDeviceOperationState *state= reinterpret_cast<BluetoothDeviceOperationState *>(thread_data);
    
    bool bFailure= false;
    bool bSkipToEnd= false;
    
    int minor_version = -1;

    char controllerBTAddr[32];
    std::vector<std::string> other_bt_addrs;

    NSAutoreleasePool *pool = [[NSAutoreleasePool alloc] init];
     
    if (!ServerUtility::bluetooth_cstr_address_normalize(
            state->getControllerAddress(), true, '-', controllerBTAddr, sizeof(controllerBTAddr)))
    {
        SERVER_MT_LOG_ERROR("async_bluetooth_device_operation_worker") << 
            "Malformed controller bluetooth address: " << state->getControllerAddress();
        bFailure= true;
    }

    if (!bFailure || !bSkipToEnd)
    {
        // Determine what version of OS X we're running first
        state->setSubStatus_WorkerThread(BluetoothDeviceOperationState::fetchOSVersion);

        if (!macosx_get_minor_version(minor_version)) 
        {
            SERVER_MT_LOG_ERROR("async_bluetooth_device_operation_worker") << "Cannot detect Mac OS X version.";
            bFailure= true;
        }
        else if (minor_version < 7)
        {
            SERVER_MT_LOG_INFO("async_bluetooth_device_operation_worker") << "No need to add entry for OS X before 10.7.";
            bSkipToEnd= true;
        }
        else 
        {
            SERVER_MT_LOG_INFO("async_bluetooth_device_operation_worker") << "Detected: Mac OS X 10." << minor_version;
        }
    }
    
    if (!bFailure || !bSkipToEnd)
    {
        // Determine if the bluetooth address of the controller is registered with the OS.
        // Also get a list of all other bluetooth address registered with the system.
        state->setSubStatus_WorkerThread(BluetoothDeviceOperationState::fetchDevicePairStatus);
        
        const bool bIsPaired= macosx_blued_is_paired(controllerBTAddr, other_bt_addrs);
        
        switch (state->getOperationType())
        {
            case BluetoothDeviceOperationState::addBluetoothDevice:
                if (bIsPaired)
                {
                    SERVER_MT_LOG_INFO("async_bluetooth_device_operation_worker") << "Entry for " << controllerBTAddr <<" already present.";
                    bSkipToEnd= true;
                }
                break;
            case BluetoothDeviceOperationState::removeBluetoothDevice:
                if (!bIsPaired)
                {
                    SERVER_MT_LOG_INFO("async_bluetooth_device_operation_worker") << "Entry for " << controllerBTAddr <<" isn't present.";
                    bSkipToEnd= true;
                }
                break;
            default:
                assert(0 && "Unreachable");
        }
    }

    if (!bFailure || !bSkipToEnd)
    {
        // Shutdown the host bluetooth adapter
        if (minor_version < 10)
        {
            state->setSubStatus_WorkerThread(BluetoothDeviceOperationState::shutdownBluetooth);

            if (!macosx_bluetooth_set_powered(false)) 
            {
                SERVER_MT_LOG_ERROR("async_bluetooth_device_operation_worker") << "Cannot shutdown Bluetooth (shut it down manually).";
                bFailure= true;
            }
            
            SERVER_MT_LOG_INFO("async_bluetooth_device_operation_worker") << "Waiting for blued shutdown (takes ca. 42s) ...";

            int attempt;
            for (attempt= 0; !bFailure && attempt < BLUED_CLOSE_MAX_POLL_ATTEMPTS; ++attempt)
            {
                if (state->getIsCanceled_WorkerThread())
                {
                    SERVER_MT_LOG_ERROR("async_bluetooth_device_operation_worker") << "Canceled from the main thread.";
                    bFailure= true;
                }

                if (!macosx_blued_running())
                {
                    SERVER_MT_LOG_INFO("async_bluetooth_device_operation_worker") << "blued successfully shutdown.";
                    break;
                }

                usleep(BLUED_CLOSE_POLL_INTERVAL_USEC);
            }
            
            if (!bFailure && attempt >= BLUED_CLOSE_MAX_POLL_ATTEMPTS)
            {
                SERVER_MT_LOG_INFO("async_bluetooth_device_operation_worker") << "blued still running. Attempting manual kill...";

                macosx_killall_blued();
            }

        }
        
        if (!bFailure)
        {
            // Registern (or unregister) the controller bluetooth address with the OS
            state->setSubStatus_WorkerThread(BluetoothDeviceOperationState::modifyBluetoothDeviceListings);

            switch (state->getOperationType())
            {
            case BluetoothDeviceOperationState::addBluetoothDevice:
                if (!macosx_register_bluetooth_address(controllerBTAddr))
                {
                    SERVER_MT_LOG_ERROR("async_bluetooth_device_operation_worker")
                        << "Could not add new bluetooth device entry: " << controllerBTAddr;
                    bFailure= true;
                }
                break;
            case BluetoothDeviceOperationState::removeBluetoothDevice:
                {
                    macosx_unregister_all_bluetooth_addresses();
                    
                    for (size_t entry_index= 0; !bFailure && entry_index < other_bt_addrs.size(); ++entry_index)
                    {
                        const std::string &bt_addr= other_bt_addrs[entry_index];
                        
                        if (!macosx_register_bluetooth_address(bt_addr.c_str()))
                        {
                            SERVER_MT_LOG_ERROR("async_bluetooth_device_operation_worker")
                                << "Could not add existing bluetooth device entry: " << bt_addr;
                            bFailure= true;
                        }
                    }
                }
                break;
            default:
                assert(0 && "Unreachable");
            }
        }
        
        // Startup the host bluetooth adapter
        if (!bFailure && minor_version < 10)
        {
            state->setSubStatus_WorkerThread(BluetoothDeviceOperationState::startupBluetooth);

            // FIXME: In OS X 10.7 this might not work - fork() and call set_powered(1)
            // from a fresh process (e.g. like "blueutil 1") to switch Bluetooth on
            if (!macosx_bluetooth_set_powered(true))
            {
                SERVER_MT_LOG_ERROR("async_bluetooth_device_operation_worker") << "Cannot startup Bluetooth (start it up manually).";
                bFailure= true;
            }
        }
    }
    
    [pool release];

    state->setSubStatus_WorkerThread(
        !bFailure
        ? BluetoothDeviceOperationState::success
        : BluetoothDeviceOperationState::failed);
    
    return 0;
}

static bool
macosx_blued_running()
{
    FILE *fp = popen("ps -axo comm", "r");
    char command[1024];
    bool running = false;
    
    while (fgets(command, sizeof(command), fp)) 
    {
        // Remove trailing newline
        command[strlen(command)-1] = '\0';
        
        if (strcmp(command, "/usr/sbin/blued") == 0) 
        {
            running = true;
        }
    }
    
    pclose(fp);
    
    return running;
}

static bool
macosx_run_admin_command(const char *cmd)
{
    const PSMoveService::ProgramSettings *settings= PSMoveService::getInstance()->getProgramSettings();
    char system_cmd[1024];
    int character_written;
    bool bSuccess= true;

    if (settings->admin_password.length() > 0)
    {
        character_written=
            snprintf(system_cmd, sizeof(system_cmd),
                 "osascript -e 'do shell script \"%s\" with administrator privileges password \"%s\"'", cmd, settings->admin_password.c_str());
    }
    else
    {
        character_written=
            snprintf(system_cmd, sizeof(system_cmd),
                 "osascript -e 'do shell script \"%s\" with administrator privileges'", cmd);
    }
    
    if (character_written > 0 && character_written < sizeof(system_cmd))
    {
        if (system(system_cmd) != 0)
        {
            SERVER_MT_LOG_ERROR("macosx_run_admin_command") << "Failed to run admin cmd: " << cmd;
            bSuccess= false;
        }
    }
    else
    {
        SERVER_MT_LOG_ERROR("macosx_run_admin_command") << "Admin cmd too long: " << cmd;
        bSuccess= false;
    }
    
    return bSuccess;
}

static void
macosx_killall_blued()
{
   
    if (!macosx_run_admin_command("killall blued"))
    {
        SERVER_MT_LOG_WARNING("macosx_killall_blued") << "Failed to kill blued";
    }
}

static bool
macosx_blued_is_paired(char *btaddr, std::vector<std::string> &other_paired_btaddrs)
{
    FILE *fp = popen("defaults read " OSX_BT_CONFIG_PATH " HIDDevices", "r");
    char line[1024];
    bool found = false;
 
    /* Example output that we need to parse:
     *
     * (
     *     "e0-ae-5e-00-00-00",
     *     "e0-ae-5e-aa-bb-cc",
     *     "00-06-f7-22-11-00",
     * )
     */
    
    while (fgets(line, sizeof(line), fp)) 
    {
        char *entry = strchr(line, '"');

        if (entry) 
        {
            entry++;
            char *delim = strchr(entry, '"');

            if (delim) 
            {
                *delim = '\0';

                if (strcmp(entry, btaddr) == 0) 
                {
                    found = true;
                }
                else
                {
                    std::string other_btaddr(btaddr);
                    
                    other_paired_btaddrs.push_back(other_btaddr);
                }
            }
        }
    }
    
    pclose(fp);
    return found;
}

static bool
macosx_get_minor_version(int &outMinorVersion)
{
    bool bSuccess= true;

    char versionString[1024];
    
    outMinorVersion= -1;

    FILE *fp= popen("sw_vers -productVersion", "r");

    if (bSuccess && fp == nullptr)
    {
        SERVER_MT_LOG_ERROR("macosx_get_minor_version") << "Failed to open sw_vers";
        bSuccess= false;
    }

    if (bSuccess && fgets(versionString, sizeof(versionString), fp) == nullptr)
    {
        SERVER_MT_LOG_ERROR("macosx_get_minor_version") << "Failed to read version string from sw_vers";
        bSuccess= false;
    }

    if (fp == nullptr)
    {
        pclose(fp);
    }
    
    if (bSuccess)
    {
        /* On Mac OS X 10.8.0, the command returns "10.8", so we allow parsing
         * only the first two numbers of the triplet, leaving the patch version
         * to the default (0) set above.
         *
         * See: https://github.com/thp/psmoveapi/issue/32
         */
        int major, minor, patch = 0;
        int assigned = sscanf(versionString, "%d.%d.%d", &major, &minor, &patch);

        if (assigned >= 2 || assigned <= 3)
        {
            outMinorVersion= minor;
        }
        else
        {
            SERVER_MT_LOG_ERROR("macosx_get_minor_version") << "Failed to parse version string: " << versionString;
            bSuccess= false;
        }
    }
   
    return bSuccess;
}

static bool
macosx_register_bluetooth_address(
    const char *controllerBTAddr)
{
    char cmd[1024];
    bool bSuccess= true;
    
    snprintf(cmd, sizeof(cmd), "defaults write " OSX_BT_CONFIG_PATH " HIDDevices -array-add \\\"%s\\\"", controllerBTAddr);
    
    SERVER_MT_LOG_INFO("async_bluetooth_device_operation_worker") << "Running: \'" << cmd << "\'";
    
    if (!macosx_run_admin_command(cmd))
    {
        SERVER_MT_LOG_ERROR("async_bluetooth_device_operation_worker") << "Could not run the command.";
        bSuccess= false;
    }
    
    return bSuccess;
}

static void
macosx_unregister_all_bluetooth_addresses()
{
    char cmd[1024];
    
    snprintf(cmd, sizeof(cmd), "defaults delete " OSX_BT_CONFIG_PATH " HIDDevices");
    
    SERVER_MT_LOG_INFO("async_bluetooth_device_operation_worker") << "Running: \'" << cmd << "\'";
    
    if (!macosx_run_admin_command(cmd))
    {
        SERVER_MT_LOG_WARNING("async_bluetooth_device_operation_worker") << "Could not delete the HIDDevices entry (already deleted?).";
    }
}

//-- network notification helper methods ----
static void
send_unpair_completed_notification_to_client(int connectionID, PSMoveProtocol::Response_ResultCode resultCode)
{
    ResponsePtr notification(new PSMoveProtocol::Response);

    notification->set_type(PSMoveProtocol::Response_ResponseType_UNPAIR_REQUEST_COMPLETED);
    notification->set_request_id(-1); // This is an notification, not a response
    notification->set_result_code(resultCode);

    ServerNetworkManager::get_instance()->send_notification(connectionID, notification);
}

static void
send_pair_completed_notification_to_client(int connectionID, PSMoveProtocol::Response_ResultCode resultCode)
{
    ResponsePtr notification(new PSMoveProtocol::Response);

    notification->set_type(PSMoveProtocol::Response_ResponseType_PAIR_REQUEST_COMPLETED);
    notification->set_request_id(-1); // This is an notification, not a response
    notification->set_result_code(resultCode);

    ServerNetworkManager::get_instance()->send_notification(connectionID, notification);
}

static void 
send_progress_notification_to_client(int connectionID, int controllerID, int stepsCompleted, int totalSteps)
{
    ResponsePtr notification(new PSMoveProtocol::Response);

    notification->set_type(PSMoveProtocol::Response_ResponseType_BLUETOOTH_REQUEST_PROGRESS);
    notification->set_request_id(-1); // This is an notification, not a response
    notification->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);

    PSMoveProtocol::Response_ResultBluetoothRequestProgress *progress=
        notification->mutable_result_bluetooth_request_progress();

    progress->set_controller_id(controllerID);
    progress->set_steps_completed(stepsCompleted);
    progress->set_total_steps(totalSteps);

    ServerNetworkManager::get_instance()->send_notification(connectionID, notification);
}
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
#include "BluetoothQueries.h"
#include "DeviceManager.h"
#include "ServerControllerView.h"
#include "ServerNetworkManager.h"
#include "ServerLog.h"
#include "ServerUtility.h"

#include "PSMoveProtocol.pb.h"
#include "PSMoveProtocolInterface.h"

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <sstream>
#include <iomanip>
#include <chrono>

// -- constants ----
/* time out indicator for a Bluetooth inquiry */
#define GET_BT_DEVICES_TIMEOUT_MULTIPLIER 1

// Sleep value between bt device scan
// Recommendation: Value should be higher than GET_BT_DEVICES_TIMEOUT_MULTIPLIER * 1.28 * 1000
#define SLEEP_BETWEEN_SCANS static_cast<long>(GET_BT_DEVICES_TIMEOUT_MULTIPLIER * 1.28 * 1000 * 1.1)

/* number of connection retries before removing the controller (in software)
 * and starting all over again
 */
#define CONN_RETRIES 80

/* the delay (in milliseconds) between connection retries
 */
#define CONN_DELAY 300

/* the number of successive checks that we require to be sure the Bluetooth
 * connection is indeed properly established
 */
#define CONN_CHECK_NUM_TRIES 5

/* the delay (in milliseconds) between consecutive checks for a properly
 * established Bluetooth connection
 */
#define CONN_CHECK_DELAY 300


// -- definitions -----
class BluetoothAsyncOperationState
{
public:

    virtual void initialize(const int id)
    {
        controller_id= id;
    }

    inline bool isMainThread() const
    {
        return false;
    }

    inline bool isWorkerThread() const
    {
        return false;
    }

    int getControllerID() const
    {
        return controller_id;
    }

    template <typename t_enum_status>
    void setSubStatus_WorkerThread(t_enum_status newSubStatus)
    {
        //assert(isWorkerThread());
        
        // Atomically set the new status on subStatus_WorkerThread
    }

    template <typename t_enum_status>
    t_enum_status getSubStatus_WorkerThread()
    {
        assert(isWorkerThread());
        
        // Atomically set the new status on subStatus_WorkerThread

        return static_cast<t_enum_status>(0);
    }

    bool getIsCanceled_WorkerThread()
    {
        assert(isWorkerThread());

        // Atomically fetch the canceled flag from the WorkerThread

        return false;
    }

    template <typename t_enum_status>
    bool pollSubStatus_MainThread(t_enum_status &outNewSubStatus)
    {
        assert(isMainThread());
        bool subStatusChanged= false;

        // Atomically fetch the status from the WorkerThread

        return subStatusChanged;
    }

    bool getIsCanceled_MainThread()
    {
        assert(isMainThread());

        // Atomically fetch the canceled flag from the MainThread

        return false;
    }

    void setIsCanceled_MainThread()
    {
        assert(isMainThread());
        // Atomically set the new status on subStatus_WorkerThread
    }

protected:
    int controller_id;

    int32_t subStatus_MainThread;
};

class BluetoothUnpairDeviceState : public BluetoothAsyncOperationState
{
public:
    enum eStatus
    {
        start,
        removingBluetoothDevice,

        k_total_steps,
        success= k_total_steps,
        failed,
    };

    virtual void initialize(const int id, const std::string &bt_address_string)
    {
        BluetoothAsyncOperationState::initialize(id);

        controllerAddress= bt_address_string;
    }

    const std::string &getControllerAddress() const
    {
        return controllerAddress;
    }

protected:
    std::string controllerAddress;
};

class BluetoothPairDeviceState : public BluetoothAsyncOperationState
{
public:
    enum eStatus
    {
        start,
        setupBluetoothRadio,
        deviceScan,
        attemptConnection,
        patchRegistry,
        verifyConnection,

        k_total_steps,
        success= k_total_steps,
        failed,
    };

    std::string controller_serial_string;
    std::string host_address_string;
    CommonDeviceState::eDeviceType controller_device_type;

    // Attempt Counters
    int scanCount;
    int connectionAttemptCount;
    int verifyConnectionCount;

    virtual void initialize(const int id) override
    {
        BluetoothAsyncOperationState::initialize(id);

        controller_serial_string.clear();
        host_address_string.clear();
        controller_device_type= CommonDeviceState::SUPPORTED_CONTROLLER_TYPE_COUNT;
        scanCount= 0;
        connectionAttemptCount= 0;
        verifyConnectionCount= 0;
    }

    void dispose()
    {
    }
};

// -- prototypes -----


// -- AsyncBluetoothUnpairDeviceRequest -----
AsyncBluetoothUnpairDeviceRequest::AsyncBluetoothUnpairDeviceRequest(
    int connectionId,
    ServerControllerViewPtr controllerView)
    : AsyncBluetoothRequest(connectionId)
    , m_controllerView(controllerView)
    , m_internal_state(nullptr)
{
    m_internal_state= new BluetoothUnpairDeviceState();
}

AsyncBluetoothUnpairDeviceRequest::~AsyncBluetoothUnpairDeviceRequest()
{
    delete ((BluetoothUnpairDeviceState *)m_internal_state);
}

bool 
AsyncBluetoothUnpairDeviceRequest::start()
{
    bool success= true;
    const int controller_id= m_controllerView->getDeviceID();
    const std::string bt_address_string= m_controllerView->getSerial();

    m_status = success ? AsyncBluetoothRequest::running : AsyncBluetoothRequest::failed;

    return success;
}

void 
AsyncBluetoothUnpairDeviceRequest::update()
{
    BluetoothUnpairDeviceState *state= ((BluetoothUnpairDeviceState *)m_internal_state);

    // Check the worker thread to see if the sub-status changed
    BluetoothUnpairDeviceState::eStatus subStatus;
}

void 
AsyncBluetoothUnpairDeviceRequest::cancel(AsyncBluetoothRequest::eCancelReason reason)
{
    BluetoothUnpairDeviceState *state= ((BluetoothUnpairDeviceState *)m_internal_state);

    // state->setIsCanceled_MainThread();
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

    description << "[Unpair] Controller ID: " << m_controllerView->getDeviceID() << " Conn: " << m_connectionId;

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
    m_internal_state= new BluetoothPairDeviceState();
}

AsyncBluetoothPairDeviceRequest::~AsyncBluetoothPairDeviceRequest()
{
    delete ((BluetoothPairDeviceState *)m_internal_state);
}

bool 
AsyncBluetoothPairDeviceRequest::start()
{
    bool success= true;
    const int controller_id= m_controllerView->getDeviceID();

    if (!success)
    {
        m_status = AsyncBluetoothRequest::failed;
    }

    return success;
}

void 
AsyncBluetoothPairDeviceRequest::update()
{
}

void 
AsyncBluetoothPairDeviceRequest::cancel(AsyncBluetoothRequest::eCancelReason reason)
{
    BluetoothPairDeviceState *state= ((BluetoothPairDeviceState *)m_internal_state);
    // state->setIsCanceled_MainThread();
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
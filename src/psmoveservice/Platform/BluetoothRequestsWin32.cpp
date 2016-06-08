#ifndef _WIN32
#error "Only include this file in windows builds!"
#endif // _WIN32

// -- includes -----
#include "BluetoothRequests.h"
#include "DeviceManager.h"
#include "ServerControllerView.h"
#include "ServerNetworkManager.h"
#include "ServerLog.h"
#include "ServerUtility.h"

#include "PSMoveProtocol.pb.h"
#include "PSMoveProtocolInterface.h"

#include <windows.h>
#include <bthsdpdef.h>
#include <bluetoothapis.h>
#include <tchar.h>
#include <StrSafe.h>
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
    HANDLE worker_thread_handle;
    DWORD worker_thread_id;

    virtual void initialize(const int id)
    {
        controller_id= id;

        main_thread_id= GetCurrentThreadId();
        worker_thread_id= main_thread_id;
        
        subStatus_WorkerThread= 0;
        isCanceled_WorkerThread= 0;

        subStatus_MainThread= 0;
    }

    inline bool isMainThread() const
    {
        return GetCurrentThreadId() == main_thread_id;
    }

    inline bool isWorkerThread() const
    {
        return GetCurrentThreadId() == worker_thread_id;
    }

    int getControllerID() const
    {
        return controller_id;
    }

    template <typename t_enum_status>
    void setSubStatus_WorkerThread(t_enum_status newSubStatus)
    {
        assert(isWorkerThread());
        
        // Atomically set the new status on subStatus_WorkerThread
        InterlockedCompareExchange(&subStatus_WorkerThread, static_cast<LONG>(newSubStatus), subStatus_WorkerThread);
    }

    template <typename t_enum_status>
    t_enum_status getSubStatus_WorkerThread()
    {
        assert(isWorkerThread());
        
        // Atomically set the new status on subStatus_WorkerThread
        LONG subStatus= 
            InterlockedCompareExchange(&subStatus_WorkerThread, subStatus_WorkerThread, subStatus_WorkerThread);

        return static_cast<t_enum_status>(subStatus);
    }

    bool getIsCanceled_WorkerThread()
    {
        assert(isWorkerThread());

        // Atomically fetch the canceled flag from the WorkerThread
        LONG isCanceled= InterlockedCompareExchange(&isCanceled_WorkerThread, isCanceled_WorkerThread, isCanceled_WorkerThread);

        return isCanceled > 0;
    }

    template <typename t_enum_status>
    bool pollSubStatus_MainThread(t_enum_status &outNewSubStatus)
    {
        assert(isMainThread());
        bool subStatusChanged= false;

        // Atomically fetch the status from the WorkerThread
        LONG newSubStatus= InterlockedCompareExchange(&subStatus_WorkerThread, subStatus_WorkerThread, subStatus_WorkerThread);

        if (newSubStatus != subStatus_MainThread)
        {
            subStatus_MainThread= newSubStatus;
            outNewSubStatus= static_cast<t_enum_status>(newSubStatus);
            subStatusChanged= true;
        }

        return subStatusChanged;
    }

    bool getIsCanceled_MainThread()
    {
        assert(isMainThread());

        // Atomically fetch the canceled flag from the MainThread
        LONG isCanceled= InterlockedCompareExchange(&isCanceled_WorkerThread, isCanceled_WorkerThread, isCanceled_WorkerThread);

        return isCanceled > 0;
    }

    void setIsCanceled_MainThread()
    {
        assert(isMainThread());
        
        // Atomically set the new status on subStatus_WorkerThread
        InterlockedCompareExchange(&isCanceled_WorkerThread, 1, isCanceled_WorkerThread);
    }

protected:
    int controller_id;

    LONG main_thread_id;

    volatile LONG subStatus_WorkerThread;
    volatile LONG isCanceled_WorkerThread;

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

    HANDLE hRadio;
    BLUETOOTH_RADIO_INFO radioInfo;
    BLUETOOTH_DEVICE_INFO deviceInfo;
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

        hRadio= INVALID_HANDLE_VALUE;
        memset(&radioInfo, 0, sizeof(BLUETOOTH_RADIO_INFO));
        radioInfo.dwSize = sizeof(BLUETOOTH_RADIO_INFO);
        memset(&deviceInfo, 0, sizeof(BLUETOOTH_DEVICE_INFO));
        deviceInfo.dwSize= sizeof(BLUETOOTH_DEVICE_INFO);
        controller_serial_string.clear();
        host_address_string.clear();
        controller_device_type= CommonDeviceState::SUPPORTED_CONTROLLER_TYPE_COUNT;
        scanCount= 0;
        connectionAttemptCount= 0;
        verifyConnectionCount= 0;
    }

    void dispose()
    {
        if (hRadio != INVALID_HANDLE_VALUE)
        {
            CloseHandle(hRadio);
            hRadio= INVALID_HANDLE_VALUE;
        }
    }
};

// -- prototypes -----
static DWORD WINAPI AsyncBluetoothUnpairDeviceThreadFunction(LPVOID lpParam);
static DWORD WINAPI AsyncBluetoothPairDeviceThreadFunction(LPVOID lpParam);

static bool AsyncBluetoothPairDeviceRequest__findBluetoothRadio(BluetoothPairDeviceState *state);
static bool AsyncBluetoothPairDeviceRequest__registerHostAddress(ServerControllerViewPtr &controllerView, BluetoothPairDeviceState *state);
static BluetoothPairDeviceState::eStatus AsyncBluetoothPairDeviceRequest__setupBluetoothRadio(BluetoothPairDeviceState *state);
static BluetoothPairDeviceState::eStatus AsyncBluetoothPairDeviceRequest__deviceScan(BluetoothPairDeviceState *state);
static BluetoothPairDeviceState::eStatus AsyncBluetoothPairDeviceRequest__attemptConnection(BluetoothPairDeviceState *state);
static BluetoothPairDeviceState::eStatus AsyncBluetoothPairDeviceRequest__patchRegistry(BluetoothPairDeviceState *state);
static BluetoothPairDeviceState::eStatus AsyncBluetoothPairDeviceRequest__verifyConnection(BluetoothPairDeviceState *state);

static bool string_to_bluetooth_address(const std::string bt_string, BLUETOOTH_ADDRESS * address);
static std::string bluetooth_address_to_string(const BLUETOOTH_ADDRESS* bt_address);
static bool find_first_bluetooth_radio(HANDLE *hRadio);
static bool get_bluetooth_device_info(const HANDLE hRadio, const BLUETOOTH_ADDRESS *addr, BLUETOOTH_DEVICE_INFO *device_info, BOOL inquire);
static bool is_matching_controller_type(const BLUETOOTH_DEVICE_INFO *device_info, const CommonDeviceState::eDeviceType controller_device_type);
static bool is_device_move_motion_controller(const BLUETOOTH_DEVICE_INFO *device_info);
static bool is_device_navigation_controller(const BLUETOOTH_DEVICE_INFO *device_info);
static bool is_hid_service_enabled(const HANDLE hRadio, BLUETOOTH_DEVICE_INFO *device_info);
static bool patch_registry(const BLUETOOTH_ADDRESS *move_addr, const BLUETOOTH_ADDRESS *radio_addr);

static void send_unpair_completed_notification_to_client(int connectionID, PSMoveProtocol::Response_ResultCode resultCode);
static void send_pair_completed_notification_to_client(int connectionID, PSMoveProtocol::Response_ResultCode resultCode);
static void send_progress_notification_to_client(int connectionID, int controllerID, int stepsCompleted, int totalSteps);

//-- Queries -----
bool bluetooth_get_host_address(std::string &out_address)
{
    bool bSuccess = true;
    HANDLE hRadio;

    if (find_first_bluetooth_radio(&hRadio) && hRadio != INVALID_HANDLE_VALUE)
    {
        SERVER_LOG_INFO("bluetooth_get_host_address") << "Found a bluetooth radio";
    }
    else
    {
        SERVER_LOG_ERROR("bluetooth_get_host_address") << "Failed to find a bluetooth radio";
        bSuccess = false;
    }

    if (bSuccess)
    {
        BLUETOOTH_RADIO_INFO radioInfo;
        memset(&radioInfo, 0, sizeof(BLUETOOTH_RADIO_INFO));
        radioInfo.dwSize = sizeof(BLUETOOTH_RADIO_INFO);

        DWORD result = BluetoothGetRadioInfo(hRadio, &radioInfo);

        if (result == ERROR_SUCCESS)
        {
            SERVER_LOG_INFO("bluetooth_get_host_address") << "Retrieved radio info";
            out_address = bluetooth_address_to_string(&radioInfo.address);
        }
        else
        {
            SERVER_LOG_ERROR("bluetooth_get_host_address")
                << "Failed to retrieve radio info (Error Code: "
                << std::hex << std::setfill('0') << std::setw(8) << result;
            bSuccess = false;
        }
    }

    return bSuccess;
}

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
    BLUETOOTH_ADDRESS bt_address;

    if (success && !string_to_bluetooth_address(bt_address_string, &bt_address))
    {
        SERVER_LOG_ERROR("AsyncBluetoothUnpairDeviceRequest") 
            << "Controller " << controller_id 
            << " doesn't have a valid BT address (" << bt_address_string
            << "). Already unpaired?";
        success= false;
    }

    if (success && (!m_controllerView->getIsOpen() || m_controllerView->getIsBluetooth()))
    {
        SERVER_LOG_ERROR("AsyncBluetoothUnpairDeviceRequest") 
            << "Controller " << controller_id 
            << " isn't an open USB device";
        success= false;
    }

    // Unregister the bluetooth host address with the controller
    if (success && !m_controllerView->setHostBluetoothAddress(std::string("00:00:00:00:00:00")))
    {
        SERVER_LOG_ERROR("AsyncBluetoothUnpairDeviceRequest") 
            << "Controller " << controller_id 
            << " can't unregister host radios BT address ";
        success= false;
    }

    // Close all controllers that use this serial number (bluetooth address)
    if (success)
    {
        const int controllerViewCount= DeviceManager::getInstance()->getControllerViewMaxCount();

        for (int index= 0; index < controllerViewCount; ++index)
        {
            ServerControllerViewPtr view= DeviceManager::getInstance()->getControllerViewPtr(index);

            if (view && view->getIsOpen() && view->getSerial() == bt_address_string)
            {
                view->close();
            }
        }
    }

    // Kick off the worker thread to do the rest of the work
    if (success)
    {
        BluetoothUnpairDeviceState *state= reinterpret_cast<BluetoothUnpairDeviceState *>(m_internal_state);

        state->initialize(controller_id, bt_address_string);
        state->worker_thread_handle=
            CreateThread( 
                NULL,                        // default security attributes
                0,                           // use default stack size  
                AsyncBluetoothUnpairDeviceThreadFunction,       // thread function pointer
                m_internal_state,            // argument to thread function 
                0,                           // use default creation flags 
                NULL);                       // returns the thread identifier

        if (state->worker_thread_handle == NULL)
        {
            SERVER_LOG_ERROR("AsyncBluetoothUnpairDeviceRequest") << "Failed to start worker thread!";
            success= false;
        }
    }

    m_status = success ? AsyncBluetoothRequest::running : AsyncBluetoothRequest::failed;

    return success;
}

static DWORD WINAPI 
AsyncBluetoothUnpairDeviceThreadFunction(LPVOID lpParam)
{
    BluetoothUnpairDeviceState *state= reinterpret_cast<BluetoothUnpairDeviceState *>(lpParam);

    state->worker_thread_id = GetCurrentThreadId();

    BLUETOOTH_ADDRESS bt_address;
    bool success= string_to_bluetooth_address(state->getControllerAddress(), &bt_address);

    if (!state->getIsCanceled_WorkerThread())
    {
        state->setSubStatus_WorkerThread(BluetoothUnpairDeviceState::removingBluetoothDevice);

        // Tell windows to remove the device
        if (success && BluetoothRemoveDevice(&bt_address) != ERROR_SUCCESS)
        {
            SERVER_MT_LOG_ERROR("AsyncBluetoothUnpairDeviceRequest") 
                << "Controller " << state->getControllerID() 
                << " failed to remove bluetooth device";
            success= false;
        }

        if (state->getIsCanceled_WorkerThread())
        {
            SERVER_MT_LOG_ERROR("AsyncBluetoothUnpairDeviceRequest") 
                << "Ignoring cancel unpair request for Controller " << state->getControllerID() 
                << ". Already removed.";
        }

        if (success)
        {
            state->setSubStatus_WorkerThread(BluetoothUnpairDeviceState::success);
        }
        else
        {
            state->setSubStatus_WorkerThread(BluetoothUnpairDeviceState::failed);
        }
    }
    else
    {
        SERVER_MT_LOG_ERROR("AsyncBluetoothUnpairDeviceRequest") << "Canceled from the main thread.";
        state->setSubStatus_WorkerThread(BluetoothUnpairDeviceState::failed);
    }

    return 0;
}

void 
AsyncBluetoothUnpairDeviceRequest::update()
{
    BluetoothUnpairDeviceState *state= ((BluetoothUnpairDeviceState *)m_internal_state);

    // Check the worker thread to see if the sub-status changed
    BluetoothUnpairDeviceState::eStatus subStatus;
    if (state->pollSubStatus_MainThread(subStatus))
    {
        // Tell the client about the sub status change
        send_progress_notification_to_client(
            m_connectionId, 
            m_controllerView->getDeviceID(), 
            static_cast<int>(subStatus), 
            BluetoothUnpairDeviceState::k_total_steps);

        // See if the worker thread has completed it's work
        if (subStatus == BluetoothUnpairDeviceState::success ||
            subStatus == BluetoothUnpairDeviceState::failed)
        {
            if (subStatus == BluetoothUnpairDeviceState::success)
            {
                m_status= AsyncBluetoothRequest::succeeded;

                // Tell the client about the result
                send_unpair_completed_notification_to_client(m_connectionId, PSMoveProtocol::Response_ResultCode_RESULT_OK);
            }
            else if (subStatus == BluetoothUnpairDeviceState::failed)
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
            WaitForSingleObject(state->worker_thread_handle, INFINITE);
        }
    }
}

void 
AsyncBluetoothUnpairDeviceRequest::cancel(AsyncBluetoothRequest::eCancelReason reason)
{
    BluetoothUnpairDeviceState *state= ((BluetoothUnpairDeviceState *)m_internal_state);

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

    // Reset the pairing device state
    BluetoothPairDeviceState *state= reinterpret_cast<BluetoothPairDeviceState *>(m_internal_state);
    state->initialize(controller_id);
    state->controller_serial_string= m_controllerView->getSerial();
    state->controller_device_type= m_controllerView->getControllerDeviceType();

    // Make sure the controller we're working with is a USB connection
    if (success && m_controllerView->getIsOpen() && m_controllerView->getIsBluetooth())
    {
        SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") 
            << "Controller " << controller_id 
            << " isn't an open USB device";
        success= false;
    }

    // Find the bluetooth address of the host adapter
    if (success)
    {
        success= AsyncBluetoothPairDeviceRequest__findBluetoothRadio(state);
    }

    // Make sure the controller isn't already paired with this host adapter
    if (m_controllerView->getAssignedHostBluetoothAddress() != state->host_address_string)
    {
        // Assign this host address on the controller.
        // NOTE: This needs to be done on the main thread since the controller view isn't thread safe.
        if (success)
        {
            success= AsyncBluetoothPairDeviceRequest__registerHostAddress(m_controllerView, state);
        }

        // Kick off the worker thread to do the rest of the work
        if (success)
        {
            state->worker_thread_handle=
                CreateThread( 
                    NULL,                        // default security attributes
                    0,                           // use default stack size  
                    AsyncBluetoothPairDeviceThreadFunction,       // thread function pointer
                    m_internal_state,            // argument to thread function 
                    0,                           // use default creation flags 
                    NULL);                       // returns the thread identifier

            if (state->worker_thread_handle == NULL)
            {
                SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to start worker thread!";
                success= false;
            }
        }

        if (success)
        {
            m_status = AsyncBluetoothRequest::running;
        }
    }
    else
    {
        SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Controller already paired";
        m_status= AsyncBluetoothRequest::succeeded;
    }

    if (!success)
    {
        m_status = AsyncBluetoothRequest::failed;
    }

    return success;
}

static DWORD WINAPI 
AsyncBluetoothPairDeviceThreadFunction(LPVOID lpParam)
{
    BluetoothPairDeviceState *state= reinterpret_cast<BluetoothPairDeviceState *>(lpParam);

    state->worker_thread_id = GetCurrentThreadId();

    bool isCompleted= false; 
    const int controller_id= state->getControllerID();

    state->setSubStatus_WorkerThread(BluetoothPairDeviceState::setupBluetoothRadio);

    while (!isCompleted)
    {
        const BluetoothPairDeviceState::eStatus subStatus= 
            state->getSubStatus_WorkerThread<BluetoothPairDeviceState::eStatus>();
        BluetoothPairDeviceState::eStatus nextSubStatus= subStatus;

        switch(subStatus)
        {
        case BluetoothPairDeviceState::setupBluetoothRadio:
            {
                nextSubStatus= AsyncBluetoothPairDeviceRequest__setupBluetoothRadio(state);
            } break;

        case BluetoothPairDeviceState::deviceScan:
            {
                nextSubStatus= AsyncBluetoothPairDeviceRequest__deviceScan(state);
            } break;

        case BluetoothPairDeviceState::attemptConnection:
            {
                nextSubStatus= AsyncBluetoothPairDeviceRequest__attemptConnection(state);
            } break;

        case BluetoothPairDeviceState::patchRegistry:
            {
                nextSubStatus= AsyncBluetoothPairDeviceRequest__patchRegistry(state);
            } break;

        case BluetoothPairDeviceState::verifyConnection:
            {
                nextSubStatus= AsyncBluetoothPairDeviceRequest__verifyConnection(state);
            } break;
        }

        if (nextSubStatus != subStatus)
        {
            state->setSubStatus_WorkerThread(nextSubStatus);
        }

        if (nextSubStatus == BluetoothPairDeviceState::success || 
            nextSubStatus == BluetoothPairDeviceState::failed)
        {
            isCompleted= true;
        }
    }

    return 0;
}

void 
AsyncBluetoothPairDeviceRequest::update()
{
    BluetoothPairDeviceState *state= ((BluetoothPairDeviceState *)m_internal_state);

    // Check the worker thread to see if the sub-status changed
    BluetoothPairDeviceState::eStatus subStatus;
    if (state->pollSubStatus_MainThread(subStatus))
    {
        // Tell the client about the sub status change
        send_progress_notification_to_client(
            m_connectionId, 
            m_controllerView->getDeviceID(), 
            static_cast<int>(subStatus), 
            BluetoothPairDeviceState::k_total_steps);

        // See if the worker thread has completed it's work
        if (subStatus == BluetoothPairDeviceState::success ||
            subStatus == BluetoothPairDeviceState::failed)
        {
            if (subStatus == BluetoothPairDeviceState::success)
            {
                m_status= AsyncBluetoothRequest::succeeded;

                // Tell the client about the result
                send_pair_completed_notification_to_client(m_connectionId, PSMoveProtocol::Response_ResultCode_RESULT_OK);
            }
            else if (subStatus == BluetoothPairDeviceState::failed)
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
            WaitForSingleObject(state->worker_thread_handle, INFINITE);
        }
    }
}

void 
AsyncBluetoothPairDeviceRequest::cancel(AsyncBluetoothRequest::eCancelReason reason)
{
    BluetoothUnpairDeviceState *state= ((BluetoothUnpairDeviceState *)m_internal_state);

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

//-- AsyncBluetoothPairDeviceRequest State Machine -----
static bool
AsyncBluetoothPairDeviceRequest__findBluetoothRadio(BluetoothPairDeviceState *state)
{
    assert(state->isMainThread());
    bool bSuccess= true;

    if (find_first_bluetooth_radio(&state->hRadio) && state->hRadio != INVALID_HANDLE_VALUE) 
    {
        SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Found a bluetooth radio";
    }
    else
    {
        SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to find a bluetooth radio";
        bSuccess= false;
    }

    if (bSuccess) 
    {
        DWORD result= BluetoothGetRadioInfo(state->hRadio, &state->radioInfo);
        if (result == ERROR_SUCCESS)
        {
            SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Retrieved radio info";
            state->host_address_string= bluetooth_address_to_string(&state->radioInfo.address);
        }
        else
        {
            SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") 
                << "Failed to retrieve radio info (Error Code: "
                << std::hex << std::setfill('0') << std::setw(8) << result;
            bSuccess= false;
        }
    }

    return bSuccess;
}

static bool
AsyncBluetoothPairDeviceRequest__registerHostAddress(
    ServerControllerViewPtr &controllerView, 
    BluetoothPairDeviceState *state)
{
    assert(state->isMainThread());
    const int controller_id= controllerView->getDeviceID();
    bool bSuccess= true;

    if (controllerView->setHostBluetoothAddress(state->host_address_string))
    {
        SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") 
            << "Assigned host address " << state->host_address_string
            << " to controller id " << controller_id;
    }
    else
    {
        SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") 
            << "Failed to set host address " << state->host_address_string
            << " on controller id " << controller_id;
        bSuccess= false;
    }

    return bSuccess;
}

static BluetoothPairDeviceState::eStatus
AsyncBluetoothPairDeviceRequest__setupBluetoothRadio(
    BluetoothPairDeviceState *state)
{
    assert(state->isWorkerThread());
    BluetoothPairDeviceState::eStatus nextSubStatus= 
        state->getSubStatus_WorkerThread<BluetoothPairDeviceState::eStatus>();

    /* NOTE: Order matters for the following two operations: The radio must
        *       allow incoming connections prior to being made discoverable.
        */
    if (!BluetoothIsConnectable(state->hRadio)) 
    {
        SERVER_MT_LOG_INFO("AsyncBluetoothPairDeviceRequest") 
            << "Making radio accept incoming connections";

        if (BluetoothEnableIncomingConnections(state->hRadio, TRUE) == FALSE) 
        {
            SERVER_MT_LOG_ERROR("AsyncBluetoothPairDeviceRequest") 
                << "Failed to enable incoming connections on radio " << state->host_address_string;
        }
    }

    if (!BluetoothIsDiscoverable(state->hRadio))                 
    {
        SERVER_MT_LOG_INFO("AsyncBluetoothPairDeviceRequest") 
            << "Making radio discoverable";

        if (BluetoothEnableDiscovery(state->hRadio, TRUE) == FALSE) 
        {
            SERVER_MT_LOG_ERROR("AsyncBluetoothPairDeviceRequest") 
                << "Failed to enable radio " << state->host_address_string << " discoverable";
        }
    }

    if (BluetoothIsConnectable(state->hRadio) != FALSE && BluetoothIsDiscoverable(state->hRadio) != FALSE)
    {
        nextSubStatus= BluetoothPairDeviceState::deviceScan;
    }
    else
    {
        nextSubStatus= BluetoothPairDeviceState::failed;
    }

    return nextSubStatus;
}

static BluetoothPairDeviceState::eStatus
AsyncBluetoothPairDeviceRequest__deviceScan(
    BluetoothPairDeviceState *state)
{
    assert(state->isWorkerThread());
    BluetoothPairDeviceState::eStatus nextSubStatus= 
        state->getSubStatus_WorkerThread<BluetoothPairDeviceState::eStatus>();
   
    std::string bt_address_string= state->controller_serial_string;
    BLUETOOTH_ADDRESS bt_address;
    bool success= string_to_bluetooth_address(bt_address_string, &bt_address);

    if (success) 
    {
        const bool inquire= (state->scanCount % 5) == 0;

        if (get_bluetooth_device_info(state->hRadio, &bt_address, &state->deviceInfo, inquire))
        {
            SERVER_MT_LOG_INFO("AsyncBluetoothPairDeviceRequest") 
                << "Bluetooth device found matching the given address: " << bt_address_string;
        }
        else
        {
            SERVER_MT_LOG_ERROR("AsyncBluetoothPairDeviceRequest") 
                << "No Bluetooth device found matching the given address: " << bt_address_string;
            success= false;
        }
    }

    if (success)
    {
        if (is_matching_controller_type(&state->deviceInfo, state->controller_device_type))
        {
            SERVER_MT_LOG_INFO("AsyncBluetoothPairDeviceRequest") 
                << "Bluetooth device matching the given address is the expected controller type";
        }
        else
        {
            char szDeviceName[256];
            ServerUtility::convert_wcs_to_mbs(state->deviceInfo.szName, szDeviceName, sizeof(szDeviceName));

            SERVER_MT_LOG_ERROR("AsyncBluetoothPairDeviceRequest") 
                << "Bluetooth device matching the given address is not an expected controller type: " << szDeviceName;
            success= false;
        }
    }

    // Keep track of the scan count attempts we have made
    ++state->scanCount;

    if (success)
    {
        // Reset the connection attempt count before starting the connection attempts
        state->connectionAttemptCount= 0;

        // Move onto attempting a connection
        nextSubStatus= BluetoothPairDeviceState::attemptConnection;
    }
    else
    {
        Sleep(SLEEP_BETWEEN_SCANS);
    }

    return nextSubStatus;
}

static BluetoothPairDeviceState::eStatus
AsyncBluetoothPairDeviceRequest__attemptConnection(
    BluetoothPairDeviceState *state)
{
    assert(state->isWorkerThread());
    BluetoothPairDeviceState::eStatus nextSubStatus= 
        state->getSubStatus_WorkerThread<BluetoothPairDeviceState::eStatus>();
    bool success= true;

    SERVER_MT_LOG_INFO("AsyncBluetoothPairDeviceRequest") 
        << "Connection attempt: " << state->connectionAttemptCount << "/" << CONN_RETRIES;

    if (BluetoothGetDeviceInfo(state->hRadio, &state->deviceInfo) != ERROR_SUCCESS) 
    {
        SERVER_MT_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to read device info";

        // Fail and go back to the device scan stage
        state->connectionAttemptCount= CONN_RETRIES;
        success= false;
    }

    if (success && !state->deviceInfo.fConnected)
    {
        SERVER_MT_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Device not connected";
        success= false;
    }

    // Keep track of how many connection attempts we have made
    ++state->connectionAttemptCount;

    if (state->connectionAttemptCount >= CONN_RETRIES)
    {
        // Fall back to scanning devices again
        nextSubStatus= BluetoothPairDeviceState::deviceScan;
    }
    else if (success)
    {
        // Move on to the patch registry state
        nextSubStatus= BluetoothPairDeviceState::patchRegistry;
    }
    else
    {
        Sleep(CONN_DELAY);
    }

    return nextSubStatus;
}

static BluetoothPairDeviceState::eStatus
AsyncBluetoothPairDeviceRequest__patchRegistry(
    BluetoothPairDeviceState *state)
{
    assert(state->isWorkerThread());
    BluetoothPairDeviceState::eStatus nextSubStatus= 
        state->getSubStatus_WorkerThread<BluetoothPairDeviceState::eStatus>();
    bool success= true;

    /* Windows 8 seems to require manual help with setting up the device
    * in the registry. Previous versions do this by themselves, but
    * doing it manually for them does not seem to harm them either. So we
    * do not single out Windows 8 but simply perform the necessary tweaks
    * for all versions of Windows.
    */
    SERVER_MT_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Patching the registry ...";
    patch_registry(&state->deviceInfo.Address, &state->radioInfo.address);

    // enable HID service only if necessary
    SERVER_MT_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Checking HID service";
    if(!is_hid_service_enabled(state->hRadio, &state->deviceInfo))
    {
        SERVER_MT_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "HID service not enabled, attempting to enable";
        GUID service = HumanInterfaceDeviceServiceClass_UUID;
        DWORD result = BluetoothSetServiceState(state->hRadio, &state->deviceInfo, &service, BLUETOOTH_SERVICE_ENABLE);
        
        if(result == ERROR_SUCCESS)
        {
            SERVER_MT_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Patching the registry ...";
            patch_registry(&state->deviceInfo.Address, &state->radioInfo.address);
        }
        else
        {
            SERVER_MT_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Failed to enable HID service. Error code: " << result;
            success= false;
        }
    }

    if (success)
    {
        // On Success, Move on to the verify connection state
        // Reset the connection verification count before entering that state
        state->verifyConnectionCount= 0;

        nextSubStatus= BluetoothPairDeviceState::verifyConnection;
    }
    else
    {
        // On Failure, fall back to the device scanning
        nextSubStatus= BluetoothPairDeviceState::deviceScan;
    }

    return nextSubStatus;
}

static BluetoothPairDeviceState::eStatus
AsyncBluetoothPairDeviceRequest__verifyConnection(
    BluetoothPairDeviceState *state)
{
    assert(state->isWorkerThread());
    BluetoothPairDeviceState::eStatus nextSubStatus= 
        state->getSubStatus_WorkerThread<BluetoothPairDeviceState::eStatus>();

    bool success= true;

    SERVER_MT_LOG_INFO("AsyncBluetoothPairDeviceRequest") 
        << "Verification attempt " << state->verifyConnectionCount
        << " / " << CONN_CHECK_NUM_TRIES;

    /* NOTE: Sometimes the Bluetooth connection appears to be established
        *       even though the Move decided that it is not really connected
        *       yet. That is why we cannot simply stop trying to connect after
        *       the first successful check. Instead, we require a minimum
        *       number of successive successful checks to be sure.
        */
    if (BluetoothGetDeviceInfo(state->hRadio, &state->deviceInfo) == ERROR_SUCCESS) 
    {
        if (state->deviceInfo.fConnected)
        {
            SERVER_MT_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Device Connected";
        }

        if (state->deviceInfo.fRemembered)
        {
            SERVER_MT_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Device Remembered";
        }

        if (is_hid_service_enabled(state->hRadio, &state->deviceInfo))
        {
            SERVER_MT_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "HID service enabled";
        }

        if (state->deviceInfo.fConnected && state->deviceInfo.fRemembered && 
            is_hid_service_enabled(state->hRadio, &state->deviceInfo))
        {
            SERVER_MT_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Connected, Remembered, and HID service enabled";
        }
        else
        {
            SERVER_MT_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "HID service not enabled";
            success= false;
        }
    }
    else
    {
        SERVER_MT_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Failed to read device info";
        success= false;
    }

    if (success)
    {
        ++state->verifyConnectionCount;

        if (state->verifyConnectionCount >= CONN_CHECK_NUM_TRIES)
        {
            SERVER_MT_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Verified connection!";
            nextSubStatus= BluetoothPairDeviceState::success;
        }
        else
        {
            nextSubStatus= BluetoothPairDeviceState::verifyConnection;
            Sleep(CONN_CHECK_DELAY);
        }
    }
    else
    {
        // Try and re-establish the connection
        SERVER_MT_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Verified failed. Re-establish connection";
        nextSubStatus= BluetoothPairDeviceState::attemptConnection;

        // Reset the connection attempt count before starting the connection attempts
        state->connectionAttemptCount= 0;
    }

    return nextSubStatus;
}

// -- helper methods -----
static bool 
string_to_bluetooth_address(const std::string bt_string, BLUETOOTH_ADDRESS *bt_address)
{
    bool success= true;

    memset(bt_address, 0, sizeof(BLUETOOTH_ADDRESS));

    // check input's length
    if (bt_string.length() == 17) 
    {
        const char *str= bt_string.c_str();
        const char *nptr = str;

        for (unsigned int i = 0; success && i < 6; i++) 
        {
            char *endptr = nullptr;

            bt_address->rgBytes[5-i] = (BYTE)strtol(nptr, &endptr, 16);

            /* we require blocks to be composed of exactly two hexadecimal
             * digits and to be separated by a double colon
             */
            if (((i < 5) && (*endptr != ':')) || (endptr - nptr != 2)) 
            {
                success= false;
            }

            /* continue with the character following the separator */
            nptr = endptr + 1;
        }
    }
    else
    {
        success= false;
    }

    return success;
}

static std::string
bluetooth_address_to_string(const BLUETOOTH_ADDRESS* bt_address)
{
    std::ostringstream stream;

    for (int buff_ind = 5; buff_ind >= 0; buff_ind--)
    {
        stream << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(bt_address->rgBytes[buff_ind]);

        if (buff_ind > 0)
        {
            stream << ":";
        }
    }

    return stream.str();
}

static bool
find_first_bluetooth_radio(HANDLE *hRadio)
{
    bool success= false;
    assert(hRadio != nullptr);

    BLUETOOTH_FIND_RADIO_PARAMS radio_params;
    radio_params.dwSize = sizeof(BLUETOOTH_FIND_RADIO_PARAMS);

    HBLUETOOTH_RADIO_FIND hFind = BluetoothFindFirstRadio(&radio_params, hRadio);
    if (hFind) 
    {
        BluetoothFindRadioClose(hFind);
        success= true;
    }

    return success;
}

static bool
get_bluetooth_device_info(
    const HANDLE hRadio, 
    const BLUETOOTH_ADDRESS *addr, 
    BLUETOOTH_DEVICE_INFO *device_info, 
    BOOL inquire)
{
    bool foundDeviceInfo= false;

    assert(addr != nullptr);
    assert(device_info != nullptr);

    BLUETOOTH_DEVICE_SEARCH_PARAMS search_params;
    search_params.dwSize               = sizeof(search_params);
    search_params.cTimeoutMultiplier   = GET_BT_DEVICES_TIMEOUT_MULTIPLIER;
    search_params.fIssueInquiry        = inquire;
    search_params.fReturnAuthenticated = TRUE;
    search_params.fReturnConnected     = TRUE;
    search_params.fReturnRemembered    = TRUE;
    search_params.fReturnUnknown       = TRUE;
    search_params.hRadio               = hRadio;

    device_info->dwSize = sizeof(*device_info);

    HBLUETOOTH_DEVICE_FIND hFind = BluetoothFindFirstDevice(&search_params, device_info);
    if (hFind) 
    {
        do 
        {
            // check if the device's Bluetooth address matches the one we are looking for
            if (device_info->Address.ullLong == addr->ullLong) 
            {
                foundDeviceInfo= true;
                break;
            }
        } while(BluetoothFindNextDevice(hFind, device_info));

        if (!BluetoothFindDeviceClose(hFind)) 
        {
            SERVER_MT_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to close bluetooth device enumeration handle";
        }
    }
    else
    {
        if (GetLastError() == ERROR_NO_MORE_ITEMS) 
        {
            SERVER_MT_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "No bluetooth devices connected.";
        }
        else
        {
            SERVER_MT_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to enumerate attached bluetooth devices";
        }
    }

    return foundDeviceInfo;
}

static bool
is_matching_controller_type(
    const BLUETOOTH_DEVICE_INFO *device_info,
    const CommonDeviceState::eDeviceType controller_device_type)
{
    bool matches= false;

    switch(controller_device_type)
    {
    case CommonDeviceState::PSMove:
        {
            matches= is_device_move_motion_controller(device_info);
        } break;
    case CommonDeviceState::PSNavi:
        {
            matches= is_device_navigation_controller(device_info);
        } break;
    default:
        assert(0 && "unreachable");
    }

    return matches;
}

static bool
is_device_move_motion_controller(const BLUETOOTH_DEVICE_INFO *device_info)
{
    return wcscmp(device_info->szName, L"Motion Controller") == 0;
}

static bool
is_device_navigation_controller(const BLUETOOTH_DEVICE_INFO *device_info)
{
    return wcscmp(device_info->szName, L"Navigation Controller") == 0;
}

static bool
is_hid_service_enabled(const HANDLE hRadio, BLUETOOTH_DEVICE_INFO *device_info)
{
    DWORD num_services = 0;
    GUID *service_list= nullptr;
    bool success = true;

    // retrieve number of installed services
    {
        DWORD result = BluetoothEnumerateInstalledServices(hRadio, device_info, &num_services, NULL);    

        if (result != ERROR_SUCCESS) 
        {
            /* NOTE: Sometimes we get ERROR_MORE_DATA, sometimes we do not.
             *       The number of services seems to be correct in any case, so
             *       we will just ignore this.
             */
            if (result != ERROR_MORE_DATA) 
            {
                SERVER_MT_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to count installed services";
                success= false;
            }
        }

        if (success && num_services == 0) 
        {
            success= false;
        }
    }

    // retrieve actual list of installed services 
    if (success)
    {
        service_list = (GUID *)calloc(num_services, sizeof(GUID));
        if (!service_list) 
        {
            success= false;
        }
    }

    if (success)
    {
        DWORD result = BluetoothEnumerateInstalledServices(hRadio, device_info, &num_services, service_list);

        if (result != ERROR_SUCCESS) 
        {
            SERVER_MT_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to enumerate installed services";
            return 0;
        }
    }

    // check if the HID service is part of that list
    if (success)
    {
        GUID service = HumanInterfaceDeviceServiceClass_UUID;

        success= false;
        for (unsigned int i = 0; i < num_services; i++) 
        {
            if (IsEqualGUID(service_list[i], service)) 
            {
                success = true;
                break;
            }
        }
    }

    if (service_list != nullptr)
    {
        free(service_list);
    }

    return success;
}

static bool
patch_registry(const BLUETOOTH_ADDRESS *move_addr, const BLUETOOTH_ADDRESS *radio_addr)
{
    bool success= true;

    TCHAR sub_key[1024];
    HRESULT res = StringCchPrintf(
        sub_key,
        1024,
        _T("SYSTEM\\CurrentControlSet\\Services\\HidBth\\Parameters\\Devices\\" \
           "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x"),
        radio_addr->rgBytes[5], radio_addr->rgBytes[4], radio_addr->rgBytes[3],
        radio_addr->rgBytes[2], radio_addr->rgBytes[1], radio_addr->rgBytes[0],
        move_addr->rgBytes[5], move_addr->rgBytes[4], move_addr->rgBytes[3],
        move_addr->rgBytes[2], move_addr->rgBytes[1], move_addr->rgBytes[0] );

    if (FAILED(res)) 
    {
        SERVER_MT_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to build registry subkey";
        success= false;
    }

    HKEY hKey;
    if (success)
    {
        LONG result = RegOpenKeyEx(HKEY_LOCAL_MACHINE, sub_key, 0, KEY_READ | KEY_QUERY_VALUE | KEY_WOW64_64KEY | KEY_ALL_ACCESS, &hKey);
        if (result != ERROR_SUCCESS) 
        {
            if (result == ERROR_FILE_NOT_FOUND) 
            {
                SERVER_MT_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to open registry key, it does not yet exist";
            }
            else
            {
                SERVER_MT_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to open registry key";
            }

            success= false;
        }
    }

    if (success)
    {
        {
            LONG result;

            do
            {
                DWORD pvData;
                DWORD dwData;
                DWORD pdwType;

                result = RegQueryValueEx(hKey, _T("VirtuallyCabled"), 0, &pdwType, (LPBYTE)&pvData, &dwData);

                if(result == ERROR_SUCCESS)
                {
                   SERVER_MT_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Get VirtuallyCabled: " << pvData;
                }
                else if( result != ERROR_MORE_DATA )
                {
                    SERVER_MT_LOG_WARNING("AsyncBluetoothPairDeviceRequest") << "Failed to get registry value. Error Code: " << result;
                    // Ignore and continue
                }
            }
            while(result == ERROR_MORE_DATA);
        }

        {
            DWORD data = 1;        
            LONG result = RegSetValueEx(hKey, _T("VirtuallyCabled"), 0, REG_DWORD, (const BYTE *)&data, sizeof(data));
            if (result != ERROR_SUCCESS) 
            {
                SERVER_MT_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to set 'VirtuallyCabled'";
                success= false;
            }
        }

        RegCloseKey(hKey);
    }

    return success;
}

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
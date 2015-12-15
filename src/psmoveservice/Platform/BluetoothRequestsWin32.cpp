#ifndef _WIN32
#error "Only include this file in windows builds!"
#endif // _WIN32

// -- includes -----
#include "BluetoothRequests.h"
#include "../ControllerManager.h"
#include "../ServerControllerView.h"
#include "../ServerNetworkManager.h"
#include "../ServerLog.h"
#include "../ServerUtility.h"

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
#define SLEEP_BETWEEN_SCANS (long long) GET_BT_DEVICES_TIMEOUT_MULTIPLIER * 1.28 * 1000 * 1.1

/* number of connection retries before removing the controller (in software)
 * and starting all over again
 */
#define CONN_RETRIES 80

/* the delay (in milliseconds) between connection retries
 */
#define CONN_DELAY 300

/* the delay (in milliseconds) between registry patch attempts
 */
#define REGISTRY_PATCH_DELAY 800

// The mac number of times we'll attempt to patch the registry
#define REGISTRY_PATCH_RETRIES 2

/* the number of successive checks that we require to be sure the Bluetooth
 * connection is indeed properly established
 */
#define CONN_CHECK_NUM_TRIES 5


// -- definitions -----
struct BluetoothPairDeviceState
{
    enum eStatus
    {
        findBluetoothRadio,
        registerHostAddress,
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
    std::string host_address_string;

    // Timers
    long long lastTimeDeviceScanned;
    long long lastTimeConnectionAttempted;
    long long lastTimeRegistryPatchAttempted;
    long long lastTimeVerifyConnection;
    
    // Attempt Counters
    int scanCount;
    int connectionAttemptCount;
    int registryPatchAttemptCount;
    int verifyConnectionCount;

    eStatus subStatus;

    void initialize()
    {
        lastTimeDeviceScanned= -1;
        lastTimeConnectionAttempted= -1;
        lastTimeRegistryPatchAttempted= -1;
        lastTimeVerifyConnection= -1;
        hRadio= INVALID_HANDLE_VALUE;
        memset(&radioInfo, 0, sizeof(BLUETOOTH_RADIO_INFO));
        radioInfo.dwSize = sizeof(BLUETOOTH_RADIO_INFO);
        memset(&deviceInfo, 0, sizeof(BLUETOOTH_DEVICE_INFO));
        deviceInfo.dwSize= sizeof(BLUETOOTH_DEVICE_INFO);
        host_address_string.clear();
        scanCount= 0;
        connectionAttemptCount= 0;
        registryPatchAttemptCount= 0;
        verifyConnectionCount= 0;
        subStatus= findBluetoothRadio;
    }

    void setSubStatus(eStatus newSubStatus)
    {
        // Handle leaving the previous sub status
        switch(subStatus)
        {
        case findBluetoothRadio:
            if (newSubStatus != findBluetoothRadio)
            {
                SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Leaving sub status: findBluetoothRadio"; 
            } break;
        case registerHostAddress:
            SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Leaving sub status: registerHostAddress"; 
            break;
        case setupBluetoothRadio:
            SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Leaving sub status: setupBluetoothRadio"; 
            break;
        case deviceScan:
            SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Leaving sub status: deviceScan"; 
            break;
        case attemptConnection:
            SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Leaving sub status: attemptConnection"; 
            break;
        case patchRegistry:
            SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Leaving sub status: patchRegistry"; 
            break;
        case verifyConnection:
            SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Leaving sub status: verifyConnection"; 
            break;
        }

        // Handle entering the new sub status
        switch(newSubStatus)
        {
        case findBluetoothRadio:
            {
                SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Entering sub status: findBluetoothRadio"; 
                initialize();
            } break;
        case registerHostAddress:
            {
                SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Entering sub status: registerHostAddress"; 
            } break;
        case setupBluetoothRadio:
            {
                SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Entering sub status: setupBluetoothRadio"; 
            } break;
        case deviceScan:
            {
                SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Entering sub status: deviceScan"; 
                if (subStatus == setupBluetoothRadio)
                {
                    lastTimeDeviceScanned= -1;
                    scanCount= 0;
                }
            } break;
        case attemptConnection:
            {
                SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Entering sub status: attemptConnection"; 
                if (subStatus == deviceScan)
                {
                    lastTimeConnectionAttempted= -1;
                    connectionAttemptCount= 0;
                }
            } break;
        case patchRegistry:
            {
                SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Entering sub status: patchRegistry"; 
                lastTimeRegistryPatchAttempted= -1;
                registryPatchAttemptCount= 0;
            } break;
        case verifyConnection:
            {
                SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Entering sub status: verifyConnection"; 
                lastTimeVerifyConnection= -1;
                verifyConnectionCount= 0;
            } break;
        case success:
            {
                SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Entering sub status: success"; 
                dispose();
            } break;
        case failed:
            {
                SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Entering sub status: failed"; 
                dispose();
            } break;
        default:
            assert(0 && "unreachable");
        }

        subStatus= newSubStatus;
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
static BluetoothPairDeviceState::eStatus AsyncBluetoothPairDeviceRequest__findBluetoothRadio(BluetoothPairDeviceState *state);
static BluetoothPairDeviceState::eStatus AsyncBluetoothPairDeviceRequest__registerHostAddress(ServerControllerViewPtr &controllerView, BluetoothPairDeviceState *state);
static BluetoothPairDeviceState::eStatus AsyncBluetoothPairDeviceRequest__setupBluetoothRadio(BluetoothPairDeviceState *state);
static BluetoothPairDeviceState::eStatus AsyncBluetoothPairDeviceRequest__deviceScan(ServerControllerViewPtr &controllerView, BluetoothPairDeviceState *state, bool &canDoMoreWork);
static BluetoothPairDeviceState::eStatus AsyncBluetoothPairDeviceRequest__attemptConnection(BluetoothPairDeviceState *state, bool &canDoMoreWork);
static BluetoothPairDeviceState::eStatus AsyncBluetoothPairDeviceRequest__patchRegistry(BluetoothPairDeviceState *state, bool &canDoMoreWork);
static BluetoothPairDeviceState::eStatus AsyncBluetoothPairDeviceRequest__verifyConnection(BluetoothPairDeviceState *state);

static bool string_to_bluetooth_address(const std::string bt_string, BLUETOOTH_ADDRESS * address);
static std::string bluetooth_address_to_string(const BLUETOOTH_ADDRESS* bt_address);
static bool find_first_bluetooth_radio(HANDLE *hRadio);
static bool get_bluetooth_device_info(const HANDLE hRadio, const BLUETOOTH_ADDRESS *addr, BLUETOOTH_DEVICE_INFO *device_info, BOOL inquire);
static bool is_matching_controller_type(const BLUETOOTH_DEVICE_INFO *device_info, const ServerControllerViewPtr &controllerView);
static bool is_device_move_motion_controller(const BLUETOOTH_DEVICE_INFO *device_info);
static bool is_device_navigation_controller(const BLUETOOTH_DEVICE_INFO *device_info);
static bool is_hid_service_enabled(const HANDLE hRadio, BLUETOOTH_DEVICE_INFO *device_info);
static bool patch_registry(const BLUETOOTH_ADDRESS *move_addr, const BLUETOOTH_ADDRESS *radio_addr);

static void send_pair_completed_notification_to_client(int connectionID, PSMoveProtocol::Response_ResultCode resultCode);
static void send_progress_notification_to_client(int connectionID, int controllerID, int stepsCompleted, int totalSteps);

// -- AsyncBluetoothUnpairDeviceRequest -----
bool 
AsyncBluetoothUnpairDeviceRequest::start()
{
    bool success= true;
    const int controller_id= m_controllerView->getControllerID();
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

    m_status = success ? AsyncBluetoothRequest::running : AsyncBluetoothRequest::failed;

    return success;
}

void 
AsyncBluetoothUnpairDeviceRequest::update()
{
    bool success= true;
    const int controller_id= m_controllerView->getControllerID();
    const std::string bt_address_string= m_controllerView->getSerial();
    BLUETOOTH_ADDRESS bt_address;

    success= string_to_bluetooth_address(bt_address_string, &bt_address);
    assert(success);

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
        const int controllerViewCount= ControllerManager::getInstance()->getControllerViewCount();

        for (int index= 0; index < controllerViewCount; ++index)
        {
            ServerControllerViewPtr view= ControllerManager::getInstance()->getControllerView(index);

            if (view->getIsOpen() && view->getSerial() == bt_address_string)
            {
                view->close();
            }
        }
    }

    // Tell windows to remove the device
    if (success)
    {
        if (BluetoothRemoveDevice(&bt_address) != ERROR_SUCCESS)
        {
            SERVER_LOG_ERROR("AsyncBluetoothUnpairDeviceRequest") 
                << "Controller " << controller_id 
                << " failed to remove bluetooth device";
            success= false;
        }
    }

    // Tell the client about the result
    {
        ResponsePtr notification(new PSMoveProtocol::Response);

        notification->set_type(PSMoveProtocol::Response_ResponseType_UNPAIR_REQUEST_COMPLETED);
        notification->set_request_id(-1); // This is an notification, not a response
        notification->set_result_code(
            success 
            ? PSMoveProtocol::Response_ResultCode_RESULT_OK 
            : PSMoveProtocol::Response_ResultCode_RESULT_ERROR); 

        ServerNetworkManager::get_instance()->send_notification(m_connectionId, notification);
    }

    m_status = success ? AsyncBluetoothRequest::succeeded : AsyncBluetoothRequest::failed;
}

void 
AsyncBluetoothUnpairDeviceRequest::cancel(AsyncBluetoothRequest::eCancelReason reason)
{
    if (reason != AsyncBluetoothRequest::connectionClosed)
    {
        ResponsePtr notification(new PSMoveProtocol::Response);

        notification->set_type(PSMoveProtocol::Response_ResponseType_UNPAIR_REQUEST_COMPLETED);
        notification->set_request_id(-1); // This is an notification, not a response
        notification->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_CANCELED); 

        ServerNetworkManager::get_instance()->send_notification(m_connectionId, notification);
    }
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
    const int controller_id= m_controllerView->getControllerID();

    if (success && m_controllerView->getIsOpen() && !m_controllerView->getIsBluetooth())
    {
        SERVER_LOG_ERROR("AsyncBluetoothUnpairDeviceRequest") 
            << "Controller " << controller_id 
            << " isn't an open USB device";
        success= false;
    }

    if (success)
    {
        ((BluetoothPairDeviceState *)m_internal_state)->setSubStatus(BluetoothPairDeviceState::findBluetoothRadio);
    }

    m_status = success ? AsyncBluetoothRequest::running : AsyncBluetoothRequest::failed;

    return success;
}

void 
AsyncBluetoothPairDeviceRequest::update()
{
    BluetoothPairDeviceState *state= ((BluetoothPairDeviceState *)m_internal_state);
    BluetoothPairDeviceState::eStatus nextSubStatus= state->subStatus;
    bool canDoMoreWork= 
        m_status != AsyncBluetoothRequest::succeeded && 
        m_status != AsyncBluetoothRequest::failed;
    const int controller_id= m_controllerView->getControllerID();

    while (canDoMoreWork)
    {
        switch(state->subStatus)
        {
        case BluetoothPairDeviceState::findBluetoothRadio:
            {
                nextSubStatus= AsyncBluetoothPairDeviceRequest__findBluetoothRadio(state);
                canDoMoreWork= true;
            } break;

        case BluetoothPairDeviceState::registerHostAddress:
            {
                nextSubStatus= AsyncBluetoothPairDeviceRequest__registerHostAddress(m_controllerView, state);
                canDoMoreWork= true;
            } break;

        case BluetoothPairDeviceState::setupBluetoothRadio:
            {
                nextSubStatus= AsyncBluetoothPairDeviceRequest__setupBluetoothRadio(state);
                canDoMoreWork= true;
            } break;

        case BluetoothPairDeviceState::deviceScan:
            {
                nextSubStatus= AsyncBluetoothPairDeviceRequest__deviceScan(m_controllerView, state, canDoMoreWork);
            } break;

        case BluetoothPairDeviceState::attemptConnection:
            {
                nextSubStatus= AsyncBluetoothPairDeviceRequest__attemptConnection(state, canDoMoreWork);
            } break;

        case BluetoothPairDeviceState::patchRegistry:
            {
                nextSubStatus= AsyncBluetoothPairDeviceRequest__patchRegistry(state, canDoMoreWork);
            } break;

        case BluetoothPairDeviceState::verifyConnection:
            {
                nextSubStatus= AsyncBluetoothPairDeviceRequest__verifyConnection(state);
                canDoMoreWork= false;
            } break;
        }

        if (nextSubStatus != state->subStatus)
        {
            state->setSubStatus(nextSubStatus);

            // Tell the client about the sub status change
            send_progress_notification_to_client(
                m_connectionId, 
                m_controllerView->getControllerID(), 
                static_cast<int>(nextSubStatus), 
                BluetoothPairDeviceState::k_total_steps);
        }

        if (nextSubStatus == BluetoothPairDeviceState::success)
        {
            canDoMoreWork= false;
            m_status= AsyncBluetoothRequest::succeeded;

            // Tell the client about the result
            send_pair_completed_notification_to_client(m_connectionId, PSMoveProtocol::Response_ResultCode_RESULT_OK);
        }
        else if (nextSubStatus == BluetoothPairDeviceState::failed)
        {
            canDoMoreWork= false;
            m_status= AsyncBluetoothRequest::failed;

            // Tell the client about the result
            send_pair_completed_notification_to_client(m_connectionId, PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
        }
        else
        {
            state->subStatus= nextSubStatus;
            m_status= AsyncBluetoothRequest::running;
        }
    }
}

void 
AsyncBluetoothPairDeviceRequest::cancel(AsyncBluetoothRequest::eCancelReason reason)
{
    BluetoothPairDeviceState *state= ((BluetoothPairDeviceState *)m_internal_state);

    state->setSubStatus(BluetoothPairDeviceState::failed);

    m_status= AsyncBluetoothRequest::failed;
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

    description << "[Pair] ID: " << m_controllerView->getControllerID() << " Conn: " << m_connectionId;

    return description.str();
}

//-- AsyncBluetoothPairDeviceRequest State Machine -----
static BluetoothPairDeviceState::eStatus
AsyncBluetoothPairDeviceRequest__findBluetoothRadio(BluetoothPairDeviceState *state)
{
    BluetoothPairDeviceState::eStatus nextSubStatus= state->subStatus;

    if (find_first_bluetooth_radio(&state->hRadio) && state->hRadio != INVALID_HANDLE_VALUE) 
    {
        SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Found a bluetooth radio";
    }
    else
    {
        SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to find a bluetooth radio";
        nextSubStatus= BluetoothPairDeviceState::failed;
    }

    if (nextSubStatus != BluetoothPairDeviceState::failed) 
    {
        if (BluetoothGetRadioInfo(&state->hRadio, &state->radioInfo) == ERROR_SUCCESS)
        {
            SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Retrieved radio info";
            state->host_address_string= bluetooth_address_to_string(&state->radioInfo.address);

            nextSubStatus = BluetoothPairDeviceState::registerHostAddress;
        }
        else
        {
            SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to retrieve radio info";
            nextSubStatus= BluetoothPairDeviceState::failed;
        }
    }

    return nextSubStatus;
}

static BluetoothPairDeviceState::eStatus
AsyncBluetoothPairDeviceRequest__registerHostAddress(
    ServerControllerViewPtr &controllerView, 
    BluetoothPairDeviceState *state)
{
    const int controller_id= controllerView->getControllerID();
    BluetoothPairDeviceState::eStatus nextSubStatus= state->subStatus;

    if (controllerView->getHostBluetoothAddress() == state->host_address_string)
    {
        SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Controller already paired";
        nextSubStatus= BluetoothPairDeviceState::success;
    }

    if (nextSubStatus != BluetoothPairDeviceState::failed) 
    {
        if (controllerView->setHostBluetoothAddress(state->host_address_string))
        {
            SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") 
                << "Assigned host address " << state->host_address_string
                << " to controller id " << controller_id;

            nextSubStatus = BluetoothPairDeviceState::setupBluetoothRadio;
        }
        else
        {
            SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") 
                << "Failed to set host address " << state->host_address_string
                << " on controller id " << controller_id;
            nextSubStatus= BluetoothPairDeviceState::failed;
        }
    }

    return nextSubStatus;
}

static BluetoothPairDeviceState::eStatus
AsyncBluetoothPairDeviceRequest__setupBluetoothRadio(
    BluetoothPairDeviceState *state)
{
    BluetoothPairDeviceState::eStatus nextSubStatus= state->subStatus;

    /* NOTE: Order matters for the following two operations: The radio must
        *       allow incoming connections prior to being made discoverable.
        */
    if (!BluetoothIsConnectable(state->hRadio)) 
    {
        SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") 
            << "Making radio accept incoming connections";

        if (BluetoothEnableIncomingConnections(state->hRadio, TRUE) == FALSE) 
        {
            SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") 
                << "Failed to enable incoming connections on radio " << state->host_address_string;

            nextSubStatus= BluetoothPairDeviceState::failed;
        }
    }

    if (nextSubStatus != BluetoothPairDeviceState::failed) 
    {
        if (!BluetoothIsDiscoverable(state->hRadio))                 
        {
            SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") 
                << "Making radio discoverable";

            if (BluetoothEnableDiscovery(state->hRadio, TRUE) == TRUE) 
            {
                nextSubStatus= BluetoothPairDeviceState::deviceScan;
            }
            else
            {
                SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") 
                    << "Failed to enable radio " << state->host_address_string << " discoverable";

                nextSubStatus= BluetoothPairDeviceState::failed;
            }
        }
    }

    return nextSubStatus;
}

static BluetoothPairDeviceState::eStatus
AsyncBluetoothPairDeviceRequest__deviceScan(
    ServerControllerViewPtr &controllerView, 
    BluetoothPairDeviceState *state,
    bool &canDoMoreWork)
{
    BluetoothPairDeviceState::eStatus nextSubStatus= state->subStatus;

    const long long now = 
        std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()).count();
    const long long diff= now - state->lastTimeDeviceScanned;
    
    // By default, assume we can't do more work this frame
    canDoMoreWork= false;

    if (diff >= SLEEP_BETWEEN_SCANS)
    {
        std::string bt_address_string= controllerView->getSerial();
        BLUETOOTH_ADDRESS bt_address;
        bool success= string_to_bluetooth_address(bt_address_string, &bt_address);

        if (success) 
        {
            const bool inquire= (state->scanCount % 5) == 0;

            if (get_bluetooth_device_info(state->hRadio, &bt_address, &state->deviceInfo, inquire))
            {
                SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") 
                    << "Bluetooth device found matching the given address: " << bt_address_string;
            }
            else
            {
                SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") 
                    << "No Bluetooth device found matching the given address: " << bt_address_string;
                success= false;
            }
        }

        if (success)
        {
            if (is_matching_controller_type(&state->deviceInfo, controllerView))
            {
                SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") 
                    << "Bluetooth device matching the given address is the expected controller type";
            }
            else
            {
                char szDeviceName[256];
                ServerUtility::convert_wcs_to_mbs(state->deviceInfo.szName, szDeviceName, sizeof(szDeviceName));

                SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") 
                    << "Bluetooth device matching the given address is not an expected controller type: " << szDeviceName;
                success= false;
            }
        }

        // Keep track of the scan count attempts we have made
        ++state->scanCount;

        // Remember the last time we scanned 
        state->lastTimeDeviceScanned= now;

        if (success)
        {
            // Move onto attempting a connection
            nextSubStatus= BluetoothPairDeviceState::attemptConnection;

            // We can run the next state this update
            canDoMoreWork= true;
        }
    }

    return nextSubStatus;
}

static BluetoothPairDeviceState::eStatus
AsyncBluetoothPairDeviceRequest__attemptConnection(
    BluetoothPairDeviceState *state,
    bool &canDoMoreWork)
{
    BluetoothPairDeviceState::eStatus nextSubStatus= state->subStatus;

    const long long now = 
        std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()).count();
    const long long diff= now - state->lastTimeConnectionAttempted;

    // by default, assume this is the only state we can run this frame
    canDoMoreWork= false;
    
    if (diff >= CONN_DELAY)
    {
        bool success= true;

        SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") 
            << "Connection attempt: " << state->connectionAttemptCount << "/" << CONN_RETRIES;

        if (success && BluetoothGetDeviceInfo(state->hRadio, &state->deviceInfo) != ERROR_SUCCESS) 
        {
            SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to read device info";

            // Fail and go back to the device scan stage
            state->connectionAttemptCount= CONN_RETRIES;
            success= false;
        }

        if (success)
        {
            SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Checking HID service ...";

            if (!is_hid_service_enabled(state->hRadio, &state->deviceInfo))
            {
                SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Enabling HID service ...";
                GUID service = HumanInterfaceDeviceServiceClass_UUID;
                DWORD result = BluetoothSetServiceState(state->hRadio, &state->deviceInfo, &service, BLUETOOTH_SERVICE_ENABLE);
                if (result != ERROR_SUCCESS) 
                {
                    SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to enable HID service";
                    success= false;
                }
            }
        }

        // Keep track of how many connection attempts we have made
        ++state->connectionAttemptCount;

        // Remember the last time we scanned 
        state->lastTimeConnectionAttempted= now;

        if (success)
        {
            // Move on to the patch registry state
            nextSubStatus= BluetoothPairDeviceState::patchRegistry;

            // Go ahead and move onto the patch registry state
            canDoMoreWork= true;
        }
        else
        {
            if (state->connectionAttemptCount >= CONN_RETRIES)
            {
                SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Device removed, starting all over again";
                BluetoothRemoveDevice(&state->deviceInfo.Address);

                nextSubStatus= BluetoothPairDeviceState::deviceScan;
            }
        }
    }

    return nextSubStatus;
}

static BluetoothPairDeviceState::eStatus
AsyncBluetoothPairDeviceRequest__patchRegistry(
    BluetoothPairDeviceState *state,
    bool &canDoMoreWork)
{
    BluetoothPairDeviceState::eStatus nextSubStatus= state->subStatus;

    const long long now = 
        std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()).count();
    const long long diff= now - state->lastTimeRegistryPatchAttempted;

    // By default, assume that this is the only state we can run this update
    canDoMoreWork= false;

    if (diff >= REGISTRY_PATCH_DELAY)
    {
        /* Windows 8 seems to require manual help with setting up the device
        * in the registry. Previous versions do this by themselves, but
        * doing it manually for them does not seem to harm them either. So we
        * do not single out Windows 8 but simply perform the necessary tweaks
        * for all versions of Windows.
        */
        SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Patching the registry ...";

        // Keep track of the number of patch attempts
        state->registryPatchAttemptCount++;

        // Remember the last time we attempted to patch the registry
        state->lastTimeRegistryPatchAttempted= now;

        /* open registry key for modifying a value */
        /* NOTE: At times, Windows seems a bit slow to generate the key we are looking for.
            *       We try more than once instead of exiting on the first failed attempt.
            */
        if (patch_registry(&state->deviceInfo.Address, &state->radioInfo.address)) 
        {
            // Move on to verification of the connection
            SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Successfully patched the registry";

            // Move on to the verify connection state
            nextSubStatus= BluetoothPairDeviceState::verifyConnection;

            // We can move on the next state this update
            canDoMoreWork= true;
        }
        else
        {
            SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to patch the registry";

            if (state->registryPatchAttemptCount > REGISTRY_PATCH_RETRIES)
            {
                // Try and re-establish the connection
                nextSubStatus= BluetoothPairDeviceState::attemptConnection;
            }
        }
    }

    return nextSubStatus;
}

static BluetoothPairDeviceState::eStatus
AsyncBluetoothPairDeviceRequest__verifyConnection(
    BluetoothPairDeviceState *state)
{
    BluetoothPairDeviceState::eStatus nextSubStatus= state->subStatus;

    const long long now = 
        std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()).count();
    const long long diff= now - state->lastTimeVerifyConnection;

    if (diff >= REGISTRY_PATCH_DELAY)
    {
        bool success= true;

        SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") 
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
            if (!state->deviceInfo.fConnected)
            {
                SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Device Not Connected";
                success= false;
            }

            if (!state->deviceInfo.fRemembered)
            {
                SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Device Not remembered";
                success= false;
            }

            if (!is_hid_service_enabled(state->hRadio, &state->deviceInfo))
            {
                SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "HID service not enabled";
                success= false;
            }
        }
        else
        {
            SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Failed to read device info";
            success= false;
        }

        if (success)
        {
            ++state->verifyConnectionCount;

            if (state->verifyConnectionCount >= CONN_CHECK_NUM_TRIES)
            {
                SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Verified connection!";
                nextSubStatus= BluetoothPairDeviceState::success;
            }
            else
            {
                nextSubStatus= BluetoothPairDeviceState::verifyConnection;
            }
        }
        else
        {
            // Try and re-establish the connection
            SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Verified failed. Re-establish connection";
            nextSubStatus= BluetoothPairDeviceState::attemptConnection;
        }
    }

    return nextSubStatus;
}

// -- helper methods -----
static bool 
string_to_bluetooth_address(const std::string bt_string, BLUETOOTH_ADDRESS *bt_address)
{
    bool success= true;

    // check input's length
    if (bt_string.length() != 17) 
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
            SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to close bluetooth device enumeration handle";
        }
    }
    else
    {
        if (GetLastError() == ERROR_NO_MORE_ITEMS) 
        {
            SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "No bluetooth devices connected.";
        }
        else
        {
            SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to enumerate attached bluetooth devices";
        }
    }

    return foundDeviceInfo;
}

static bool
is_matching_controller_type(
    const BLUETOOTH_DEVICE_INFO *device_info,
    const ServerControllerViewPtr &controllerView)
{
    bool matches= false;

    switch(controllerView->getControllerDeviceType())
    {
    case CommonControllerState::PSMove:
        {
            matches= is_device_move_motion_controller(device_info);
        } break;
    case CommonControllerState::PSNavi:
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
                SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to count installed services";
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
            SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to enumerate installed services";
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
        SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to build registry subkey";
        success= false;
    }

    HKEY hKey;
    if (success)
    {
        LONG result = RegOpenKeyEx(HKEY_LOCAL_MACHINE, sub_key, 0, KEY_SET_VALUE | KEY_WOW64_64KEY, &hKey);
        if (result != ERROR_SUCCESS) 
        {
            if (result == ERROR_FILE_NOT_FOUND) 
            {
                SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to open registry key, it does not yet exist";
            }
            else
            {
                SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to open registry key";
            }

            success= false;
        }
    }

    if (success)
    {
        DWORD data = 1;
        LONG result = RegSetValueEx(hKey, _T("VirtuallyCabled"), 0, REG_DWORD, (const BYTE *) &data, sizeof(data));

        if (result != ERROR_SUCCESS) 
        {
            SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to set 'VirtuallyCabled'";
            success= false;
        }

        RegCloseKey(hKey);
    }

    return success;
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
// -- includes -----
#include "BluetoothRequests.h"
#include "../ControllerManager.h"
#include "../ServerControllerView.h"
#include "../ServerNetworkManager.h"
#include "../ServerLog.h"
#include "../ServerUtility.h"

#include "PSMoveProtocol.pb.h"
#include "PSMoveProtocolInterface.h"

#include <sstream>
#include <iomanip>
#include <chrono>

/*
#include <IOBluetooth/objc/IOBluetoothHostController.h>

#import <Foundation/NSAutoreleasePool.h>

/// Location for the plist file that we want to modify
#define OSX_BT_CONFIG_PATH "/Library/Preferences/com.apple.Bluetooth"

/// Function declarations for IOBluetooth private API
void IOBluetoothPreferenceSetControllerPowerState(int);
int IOBluetoothPreferenceGetControllerPowerState();

#define OSXPAIR_DEBUG(msg, ...) \
psmove_PRINTF("PAIRING OSX", msg, ## __VA_ARGS__)

int
macosx_bluetooth_set_powered(int powered)
{
    // Inspired by blueutil from Frederik Seiffert <ego@frederikseiffert.de>
    int state = IOBluetoothPreferenceGetControllerPowerState();
    //OSXPAIR_DEBUG("Bluetooth was %s...\n", state ? "on":"off");
    //OSXPAIR_DEBUG("Switching Bluetooth %s...\n", powered?"on":"off");
    IOBluetoothPreferenceSetControllerPowerState(powered);
    
    // Wait a bit for Bluetooth to be (de-)activated
    usleep(2000000);
    
    if (IOBluetoothPreferenceGetControllerPowerState() != powered) {
        // Happened to me once while Bluetooth devices were connected
        return 0;
    }
    
    return 1;
}

char *
macosx_get_btaddr()
{
    NSAutoreleasePool *pool = [[NSAutoreleasePool alloc] init];
    char *result;
    
    macosx_bluetooth_set_powered(1);
    
    IOBluetoothHostController *controller =
    [IOBluetoothHostController defaultController];
    
    NSString *addr = [controller addressAsString];
    psmove_return_val_if_fail(addr != NULL, NULL);
    
    result = strdup([addr UTF8String]);
    psmove_return_val_if_fail(result != NULL, NULL);
    
    [pool release];
    return result;
}

int
macosx_blued_running()
{
    FILE *fp = popen("ps -axo comm", "r");
    char command[1024];
    int running = 0;
    
    while (fgets(command, sizeof(command), fp)) {
    // Remove trailing newline
        command[strlen(command)-1] = '\0';
        
        if (strcmp(command, "/usr/sbin/blued") == 0) {
            running = 1;
        }
    }
    
    pclose(fp);
    
    return running;
}

int
macosx_blued_is_paired(char *btaddr)
{
    FILE *fp = popen("defaults read " OSX_BT_CONFIG_PATH " HIDDevices", "r");
    char line[1024];
    int found = 0;
    
 //
     * Example output that we need to parse:
     *
     * (
     *     "e0-ae-5e-00-00-00",
     *     "e0-ae-5e-aa-bb-cc",
     *     "00-06-f7-22-11-00",
     * )
     *
 
    
    while (fgets(line, sizeof(line), fp)) {
        char *entry = strchr(line, '"');
        if (entry) {
            entry++;
            char *delim = strchr(entry, '"');
            if (delim) {
                *delim = '\0';
                if (strcmp(entry, btaddr) == 0) {
                    found = 1;
                }
            }
        }
    }
    
    pclose(fp);
    return found;
}

int
macosx_get_minor_version()
{
    char tmp[1024];
    int major, minor, patch = 0;
    FILE *fp;
    
    fp = popen("sw_vers -productVersion", "r");
    psmove_return_val_if_fail(fp != NULL, -1);
    psmove_return_val_if_fail(fgets(tmp, sizeof(tmp), fp) != NULL, -1);
    pclose(fp);
    
    int assigned = sscanf(tmp, "%d.%d.%d", &major, &minor, &patch);
    

     * On Mac OS X 10.8.0, the command returns "10.8", so we allow parsing
     * only the first two numbers of the triplet, leaving the patch version
     * to the default (0) set above.
     *
     * See: https://github.com/thp/psmoveapi/issue/32

    psmove_return_val_if_fail(assigned == 2 || assigned == 3, -1);
    
    return minor;
}

int
macosx_blued_register_psmove(char *addr)
{
    NSAutoreleasePool *pool = [[NSAutoreleasePool alloc] init];
    int result = 1;
    char cmd[1024];
    char *btaddr = _psmove_normalize_btaddr(addr, 1, '-');
    
    int minor_version = macosx_get_minor_version();
    if (minor_version == -1) {
        OSXPAIR_DEBUG("Cannot detect Mac OS X version.\n");
        result = 0;
        goto end;
    } else if (minor_version < 7) {
        OSXPAIR_DEBUG("No need to add entry for OS X before 10.7.\n");
        goto end;
    } else {
        OSXPAIR_DEBUG("Detected: Mac OS X 10.%d\n", minor_version);
    }
    
    if (macosx_blued_is_paired(btaddr)) {
        OSXPAIR_DEBUG("Entry for %s already present.\n", btaddr);
        goto end;
    }
    
    if (minor_version < 10)
    {
        if (!macosx_bluetooth_set_powered(0)) {
            OSXPAIR_DEBUG("Cannot shutdown Bluetooth (shut it down manually).\n");
        }
        
        int i = 0;
        OSXPAIR_DEBUG("Waiting for blued shutdown (takes ca. 42s) ...\n");
        while (macosx_blued_running()) {
            usleep(1000000);
            i++;
        }
        OSXPAIR_DEBUG("blued successfully shutdown.\n");
    }
    
    snprintf(cmd, sizeof(cmd), "osascript -e 'do shell script "
             "\"defaults write " OSX_BT_CONFIG_PATH
             " HIDDevices -array-add \\\"%s\\\"\""
             " with administrator privileges'", btaddr);
    OSXPAIR_DEBUG("Running: '%s'\n", cmd);
    if (system(cmd) != 0) {
        OSXPAIR_DEBUG("Could not run the command.");
    }
    
    if (minor_version < 10)
    {
        // FIXME: In OS X 10.7 this might not work - fork() and call set_powered(1)
        // from a fresh process (e.g. like "blueutil 1") to switch Bluetooth on
        macosx_bluetooth_set_powered(1);
    }
    
    free(btaddr);
    
end:
    [pool release];
    
    return result;
}
*/





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

/* the number of successive checks that we require to be sure the Bluetooth
 * connection is indeed properly established
 */
#define CONN_CHECK_NUM_TRIES 5

/* the delay (in milliseconds) between consecutive checks for a properly
 * established Bluetooth connection
 */
#define CONN_CHECK_DELAY 300


// -- definitions -----
struct BluetoothPairDeviceState
{
    enum eStatus
    {
        // psmove_init
        // psmove_connect_by_id
        // psmove_pair
        // -macosx_get_btaddr
        // -if (memcmp(current_host, btaddr, sizeof(PSMove_Data_BTAddr)) != 0)
        // -psmove_set_btaddr(move, &btaddr)
        // -char *addr = psmove_get_serial(move);
        // -macosx_blued_register_psmove(addr);
        findBluetoothRadio,
        registerHostAddress,
        setupBluetoothRadio,
        deviceScan,
        attemptConnection,

        k_total_steps,
        success= k_total_steps,
        failed,
    };

    std::string host_address_string;

    // Timers
    long long lastTimeDeviceScanned;
    long long lastTimeConnectionAttempted;
    long long lastTimeVerifyConnection;
    
    // Attempt Counters
    int scanCount;
    int connectionAttemptCount;
    int verifyConnectionCount;

    eStatus subStatus;

    void initialize()
    {
        lastTimeDeviceScanned= -1;
        lastTimeConnectionAttempted= -1;
        lastTimeVerifyConnection= -1;

        host_address_string.clear();
        scanCount= 0;
        connectionAttemptCount= 0;
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
                else
                {
                    //TODO: Remove device
                    SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Device removed, starting all over again";
                }
            } break;
        case attemptConnection:
            {
                SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Entering sub status: attemptConnection"; 
                connectionAttemptCount= 0;
                lastTimeConnectionAttempted= -1;
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
        //TODO: dispose device
    }
};

// -- prototypes -----
static BluetoothPairDeviceState::eStatus AsyncBluetoothPairDeviceRequest__findBluetoothRadio(BluetoothPairDeviceState *state);
static BluetoothPairDeviceState::eStatus AsyncBluetoothPairDeviceRequest__registerHostAddress(ServerControllerViewPtr &controllerView, BluetoothPairDeviceState *state);
static BluetoothPairDeviceState::eStatus AsyncBluetoothPairDeviceRequest__setupBluetoothRadio(BluetoothPairDeviceState *state);
static BluetoothPairDeviceState::eStatus AsyncBluetoothPairDeviceRequest__deviceScan(ServerControllerViewPtr &controllerView, BluetoothPairDeviceState *state, bool &canDoMoreWork);
static BluetoothPairDeviceState::eStatus AsyncBluetoothPairDeviceRequest__attemptConnection(BluetoothPairDeviceState *state, bool &canDoMoreWork);
static void send_pair_completed_notification_to_client(int connectionID, PSMoveProtocol::Response_ResultCode resultCode);
static void send_progress_notification_to_client(int connectionID, int controllerID, int stepsCompleted, int totalSteps);

// -- AsyncBluetoothUnpairDeviceRequest -----
bool 
AsyncBluetoothUnpairDeviceRequest::start()
{
    bool success= true;
    const int controller_id= m_controllerView->getControllerID();
    const std::string bt_address_string= m_controllerView->getSerial();
    
    // TODO: check if controller is already upaired.

    if (success && false)
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

    // TODO: Unregister the bluetooth host address with the controller
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

    // TODO: Tell OSX to remove the device
    if (success)
    {
        if (true)
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

    if (success && m_controllerView->getIsOpen() && m_controllerView->getIsBluetooth())
    {
        SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") 
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
    
    // TODO: Find first bluetooth radio

    if (false)
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
        //TODO: Retrieve radio info
        if (false)
        {
            SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") << "Retrieved radio info";
            nextSubStatus = BluetoothPairDeviceState::registerHostAddress;
        }
        else
        {
            SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") 
            << "Failed to retrieve radio info." << std::endl;
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
    
    //TODO: allow incoming connections and make discoverable

    if (true)
    {
        SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") 
            << "Making radio accept incoming connections";

        if (true)
        {
            SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") 
                << "Failed to enable incoming connections on radio " << state->host_address_string;
        }
    }

    if (false)
    {
        SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") 
            << "Making radio discoverable";

        if (true)
        {
            SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") 
                << "Failed to enable radio " << state->host_address_string << " discoverable";
        }
    }

    if (false)
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
        
        bool success= true;

        if (success) 
        {
            const bool inquire= (state->scanCount % 5) == 0;

            if (false)
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
            if (false)
            {
                SERVER_LOG_INFO("AsyncBluetoothPairDeviceRequest") 
                    << "Bluetooth device matching the given address is the expected controller type";
            }
            else
            {
                SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") 
                << "Bluetooth device matching the given address is not an expected controller type: ";// << szDeviceName;
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

        if (true)
        {
            SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Failed to read device info";

            // Fail and go back to the device scan stage
            state->connectionAttemptCount= CONN_RETRIES;
            success= false;
        }

        if (true)
        {
            SERVER_LOG_ERROR("AsyncBluetoothPairDeviceRequest") << "Device not connected";
            success= false;
        }

        // Keep track of how many connection attempts we have made
        ++state->connectionAttemptCount;

        // Remember the last time we scanned 
        state->lastTimeConnectionAttempted= now;

        if (state->connectionAttemptCount >= CONN_RETRIES)
        {
            // Fall back to scanning devices again
            nextSubStatus= BluetoothPairDeviceState::deviceScan;
        }
        else if (success)
        {
            // Go ahead and move onto the patch registry state
            canDoMoreWork= true;
        }
    }

    return nextSubStatus;
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
#ifndef _WIN32
#error "Only include this file in windows builds!"
#endif // _WIN32

// -- includes -----
#include "BluetoothRequests.h"
#include "../ControllerManager.h"
#include "../ServerControllerView.h"
#include "../ServerNetworkManager.h"
#include "../ServerLog.h"

#include "PSMoveProtocol.pb.h"
#include "PSMoveProtocolInterface.h"

#include <windows.h>
#include <bthsdpdef.h>
#include <bluetoothapis.h>
#include <sstream>

// -- prototypes -----
static bool string_to_bluetooth_address(const std::string bt_string, BLUETOOTH_ADDRESS * address);

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
        SERVER_LOG_INFO("AsyncBluetoothUnpairDeviceRequest") 
            << "Controller " << controller_id 
            << " doesn't have a valid BT address (" << bt_address_string
            << "). Already unpaired?";
        success= false;
    }

    if (success && m_controllerView->getIsOpen() && !m_controllerView->getIsBluetooth())
    {
        SERVER_LOG_INFO("AsyncBluetoothUnpairDeviceRequest") 
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
        SERVER_LOG_INFO("AsyncBluetoothUnpairDeviceRequest") 
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
            SERVER_LOG_INFO("AsyncBluetoothUnpairDeviceRequest") 
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
    return AsyncBluetoothRequest::succeeded;
}

std::string 
AsyncBluetoothUnpairDeviceRequest::getDescription()
{
    std::ostringstream description;

    description << "[Unpair] Serial: " << m_controllerView->getSerial() << " Conn: " << m_connectionId;

    return description.str();
}

// -- AsyncBluetoothPairDeviceRequest -----
bool 
AsyncBluetoothPairDeviceRequest::start()
{
    bool success= false;

    return success;
}

void 
AsyncBluetoothPairDeviceRequest::update()
{
}

void 
AsyncBluetoothPairDeviceRequest::cancel(AsyncBluetoothRequest::eCancelReason reason)
{
}

AsyncBluetoothRequest::eStatusCode 
AsyncBluetoothPairDeviceRequest::getStatusCode()
{
    return AsyncBluetoothRequest::succeeded;
}

std::string 
AsyncBluetoothPairDeviceRequest::getDescription()
{
    std::ostringstream description;

    description << "[Pair] ID: " << m_controllerView->getControllerID() << " Conn: " << m_connectionId;

    return description.str();
}

// -- helper methods -----
static bool string_to_bluetooth_address(const std::string bt_string, BLUETOOTH_ADDRESS *bt_address)
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

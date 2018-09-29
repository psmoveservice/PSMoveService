// -- includes -----
#include "BluetoothRequests.h"
#include "DeviceManager.h"

AsyncBluetoothRequest::AsyncBluetoothRequest(int connectionId)
    : m_connectionId(connectionId)
    , m_status(preflight)
{
	// Tell the platform layer to stop listening to bluetooth device connection changes
	// while bluetooth device pairing is in progress
	DeviceManager::getInstance()->handle_bluetooth_request_started();
}

AsyncBluetoothRequest::~AsyncBluetoothRequest()
{
	// Tell the platform layer to resume listening to bluetooth device connection changes
	DeviceManager::getInstance()->handle_bluetooth_request_finished();
}
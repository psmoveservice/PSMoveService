//-- includes -----
#include "ControllerManager.h"
#include "BluetoothQueries.h"
#include "ControllerDeviceEnumerator.h"
#include "ControllerGamepadEnumerator.h"
#include "OrientationFilter.h"
#include "PSMoveProtocol.pb.h"
#include "ServerLog.h"
#include "ServerControllerView.h"
#include "ServerDeviceView.h"
#include "ServerNetworkManager.h"
#include "ServerUtility.h"

#include "hidapi.h"
#include "gamepad/Gamepad.h"

//-- methods -----
ControllerManager::ControllerManager()
    : DeviceTypeManager(1000, 2)
{
}

bool
ControllerManager::startup()
{
    bool success = true;

    if (!DeviceTypeManager::startup())
    {
        success = false;
    }

    if (success)
    {
        // Initialize HIDAPI
        if (hid_init() == -1)
        {
            SERVER_LOG_ERROR("ControllerManager::startup") << "Failed to initialize HIDAPI";
            success = false;
        }
    }

	if (success && gamepad_api_enabled)
	{
		Gamepad_init();
	}

    if (success)
    {
        if (!bluetooth_get_host_address(m_bluetooth_host_address))
		{
			m_bluetooth_host_address = "00:00:00:00:00:00";
		}
	}

	return success;
}

void
ControllerManager::shutdown()
{
	DeviceTypeManager::shutdown();

	// Shutdown HIDAPI
	hid_exit();

	// Shutdown the gamepad api
	if (gamepad_api_enabled)
	{
		Gamepad_shutdown();
	}
}

void
ControllerManager::updateStateAndPredict(TrackerManager* tracker_manager)
{
	for (int device_id = 0; device_id < getMaxDevices(); ++device_id)
	{
		ServerControllerViewPtr controllerView = getControllerViewPtr(device_id);

		if (controllerView->getIsOpen() && controllerView->getIsBluetooth())
		{
			controllerView->updateOpticalPoseEstimation(tracker_manager);
			controllerView->updateStateAndPredict();
		}
	}
}

void ControllerManager::publish()
{
    DeviceTypeManager::publish();

    bool bWasSystemButtonPressed= false;
    for (int device_id = 0; device_id < getMaxDevices(); ++device_id)
	{
		ServerControllerViewPtr controllerView = getControllerViewPtr(device_id);

        if (controllerView->getIsOpen() && controllerView->getIsBluetooth())
        {
            bWasSystemButtonPressed= controllerView->getWasSystemButtonPressed();
        }
    }

    if (bWasSystemButtonPressed)
    {
        ResponsePtr response(new PSMoveProtocol::Response);
        response->set_type(PSMoveProtocol::Response_ResponseType_SYSTEM_BUTTON_PRESSED);
        response->set_request_id(-1);
        response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);

        ServerNetworkManager::get_instance()->send_notification_to_all_clients(response);
    }
}

void
ControllerManager::poll_devices()
{
	// Poll all gamepads before updating the individual controllers
	if (gamepad_api_enabled && can_poll_connected_devices())
	{
		Gamepad_processEvents();
	}

	DeviceTypeManager::poll_devices();
}

DeviceEnumerator *
ControllerManager::allocate_device_enumerator()
{
	return new ControllerDeviceEnumerator(ControllerDeviceEnumerator::CommunicationType_ALL);
}

void
ControllerManager::free_device_enumerator(DeviceEnumerator *enumerator)
{
	delete static_cast<ControllerDeviceEnumerator *>(enumerator);
}

ServerDeviceView *
ControllerManager::allocate_device_view(int device_id)
{
	return new ServerControllerView(device_id);
}

int ControllerManager::getListUpdatedResponseType()
{
	return PSMoveProtocol::Response_ResponseType_CONTROLLER_LIST_UPDATED;
}

void
ControllerManager::setControllerRumble(
    int controller_id, 
    float rumble_amount,
    CommonControllerState::RumbleChannel channel)
{
	ServerControllerViewPtr controllerView = getControllerViewPtr(controller_id);

    if (controllerView && controllerView->getIsOpen())
    {
        controllerView->setControllerRumble(rumble_amount, channel);
    }
}

ServerControllerViewPtr
ControllerManager::getControllerViewPtr(int device_id)
{
    assert(m_deviceViews != nullptr);

    return std::static_pointer_cast<ServerControllerView>(m_deviceViews[device_id]);
}
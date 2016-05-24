//-- includes -----
#include "ControllerManager.h"
#include "ControllerDeviceEnumerator.h"
#include "OrientationFilter.h"
#include "ServerLog.h"
#include "ServerControllerView.h"
#include "ServerDeviceView.h"
#include "ServerNetworkManager.h"
#include "ServerRequestHandler.h"
#include "ServerUtility.h"
#include "hidapi.h"

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

    return success;
}

void
ControllerManager::shutdown()
{
    DeviceTypeManager::shutdown();

    // Shutdown HIDAPI
    hid_exit();
}

void
ControllerManager::updateStateAndPredict(TrackerManager* tracker_manager)
{
    for (int device_id = 0; device_id < getMaxDevices(); ++device_id)
    {
        ServerControllerViewPtr controller = getControllerViewPtr(device_id);

		if (controller->getIsOpen())
		{
			controller->updatePositionEstimation(tracker_manager);
			controller->updateStateAndPredict();
		}
    }
}

bool
ControllerManager::can_update_connected_devices()
{
    return !ServerRequestHandler::get_instance()->any_active_bluetooth_requests();
}

DeviceEnumerator *
ControllerManager::allocate_device_enumerator()
{
    return new ControllerDeviceEnumerator;
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

bool
ControllerManager::setControllerRumble(int controller_id, int rumble_amount)
{
    bool result = false;

    if (ServerUtility::is_index_valid(controller_id, k_max_devices))
    {
        result = getControllerViewPtr(controller_id)->setControllerRumble(rumble_amount);
    }

    return result;
}

bool
ControllerManager::resetPose(int controller_id)
{
    bool bSuccess = false;
    ServerControllerViewPtr ControllerPtr = getControllerViewPtr(controller_id);

    if (ControllerPtr)
    {
        OrientationFilter *filter = ControllerPtr->getOrientationFilter();

        if (filter != nullptr)
        {
            filter->resetOrientation();
            bSuccess = true;
        }
    }

    return bSuccess;
}

ServerControllerViewPtr
ControllerManager::getControllerViewPtr(int device_id)
{
    assert(m_deviceViews != nullptr);

    return std::static_pointer_cast<ServerControllerView>(m_deviceViews[device_id]);
}
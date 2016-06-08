//-- includes -----
#include "ControllerManager.h"
#include "BluetoothQueries.h"
#include "ControllerDeviceEnumerator.h"
#include "OrientationFilter.h"
#include "ServerLog.h"
#include "ServerControllerView.h"
#include "ServerDeviceView.h"
#include "ServerNetworkManager.h"
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

        // Put all of the available tracking colors in the queue
        for (int color_index = 0; color_index < eCommonTrackingColorID::MAX_TRACKING_COLOR_TYPES; ++color_index)
        {
            m_available_controller_color_ids.push_back(static_cast<eCommonTrackingColorID>(color_index));
        }
    }

    if (success)
    {
        if (!bluetooth_get_host_address(m_bluetooth_host_address))
        {
            m_bluetooth_host_address= "00:00:00:00:00:00";
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
        ServerControllerViewPtr controllerView = getControllerViewPtr(device_id);

		if (controllerView->getIsOpen() && controllerView->getIsBluetooth())
		{
			controllerView->updatePositionEstimation(tracker_manager);
			controllerView->updateStateAndPredict();
		}
    }
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

eCommonTrackingColorID 
ControllerManager::allocateTrackingColorID()
{
    assert(m_available_controller_color_ids.size() > 0);
    eCommonTrackingColorID tracking_color = m_available_controller_color_ids.front();

    m_available_controller_color_ids.pop_front();

    return tracking_color;
}

void 
ControllerManager::claimTrackingColorID(eCommonTrackingColorID color_id)
{
    bool bColorWasInUse = false;

    // If any other controller has this tracking color, make them pick a new color
    for (int device_id = 0; device_id < getMaxDevices(); ++device_id)
    {
        ServerControllerViewPtr device = getControllerViewPtr(device_id);

        if (device->getIsOpen())
        {
            if (device->getTrackingColorID() == color_id)
            {
                device->setTrackingColorID(allocateTrackingColorID());
                bColorWasInUse = true;
                break;
            }
        }
    }

    // If the color was not in use, remove it from the color queue
    if (!bColorWasInUse)
    {
        for (auto iter = m_available_controller_color_ids.begin(); iter != m_available_controller_color_ids.end(); ++iter)
        {
            if (*iter == color_id)
            {
                m_available_controller_color_ids.erase(iter);
                break;
            }
        }
    }
}

void 
ControllerManager::freeTrackingColorID(eCommonTrackingColorID color_id)
{
    assert(std::find(m_available_controller_color_ids.begin(), m_available_controller_color_ids.end(), color_id) == m_available_controller_color_ids.end());
    m_available_controller_color_ids.push_back(color_id);
}

//-- includes -----
#include "HMDManager.h"
#include "HMDDeviceEnumerator.h"
#include "ServerLog.h"
#include "ServerHMDView.h"
#include "ServerDeviceView.h"

//-- methods -----
HMDManager::HMDManager()
    : DeviceTypeManager(1000, 2)
{
}

bool
HMDManager::startup()
{
    bool success = false;

    if (DeviceTypeManager::startup())
    {
        success = true;
    }

    return success;
}

void
HMDManager::shutdown()
{
    DeviceTypeManager::shutdown();
}

void
HMDManager::updateStateAndPredict(TrackerManager* tracker_manager)
{
	for (int device_id = 0; device_id < getMaxDevices(); ++device_id)
	{
		ServerHMDViewPtr hmdView = getHMDViewPtr(device_id);

		if (hmdView->getIsOpen())
		{
			hmdView->updateOpticalPoseEstimation(tracker_manager);
			hmdView->updateStateAndPredict();
		}
	}
}

ServerHMDViewPtr
HMDManager::getHMDViewPtr(int device_id)
{
    assert(m_deviceViews != nullptr);

    return std::static_pointer_cast<ServerHMDView>(m_deviceViews[device_id]);
}

bool
HMDManager::can_update_connected_devices()
{
    return true;
}

DeviceEnumerator *
HMDManager::allocate_device_enumerator()
{
    return new HMDDeviceEnumerator;
}

void
HMDManager::free_device_enumerator(DeviceEnumerator *enumerator)
{
    delete static_cast<HMDDeviceEnumerator *>(enumerator);
}

ServerDeviceView *
HMDManager::allocate_device_view(int device_id)
{
    return new ServerHMDView(device_id);
}

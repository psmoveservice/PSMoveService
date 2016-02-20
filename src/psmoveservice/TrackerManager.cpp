//-- includes -----
#include "TrackerManager.h"
#include "DeviceEnumerator.h"
#include "ServerLog.h"
#include "ServerTrackerView.h"
#include "ServerDeviceView.h"

//-- methods -----
TrackerManager::TrackerManager()
    : DeviceTypeManager(10000, 13)
{
}

bool
TrackerManager::can_update_connected_devices()
{
    return true;
}

DeviceEnumerator *
TrackerManager::allocate_device_enumerator()
{
    return new TrackerDeviceEnumerator;
}

void
TrackerManager::free_device_enumerator(DeviceEnumerator *enumerator)
{
    delete static_cast<TrackerDeviceEnumerator *>(enumerator);
}

ServerDeviceView *
TrackerManager::allocate_device_view(int device_id)
{
    return new ServerTrackerView(device_id);
}

ServerTrackerViewPtr
TrackerManager::getTrackerViewPtr(int device_id)
{
    assert(m_deviceViews != nullptr);

    return std::static_pointer_cast<ServerTrackerView>(m_deviceViews[device_id]);
}
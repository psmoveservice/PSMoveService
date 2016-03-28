//-- includes -----
#include "HMDManager.h"
#include "HMDDeviceEnumerator.h"
#include "ServerLog.h"
#include "ServerHMDView.h"
#include "ServerDeviceView.h"

#include "OVR_CAPI.h"

//-- methods -----
HMDManager::HMDManager()
    : DeviceTypeManager(1000, 2)
    , m_oculusapi_initialized(false)
{
}

bool
HMDManager::startup()
{
    bool success = false;

    if (DeviceTypeManager::startup())
    {
        if (ovr_Initialize(0) == ovrTrue)
        {
            m_oculusapi_initialized = true;
        }
        else
        {
            SERVER_LOG_WARNING("HMDManager::startup") << "Oculus API init failed (different SDK installed?)";
            success = false;
        }
    }

    return success;
}

void
HMDManager::shutdown()
{
    if (m_oculusapi_initialized)
    {
        ovr_Shutdown();
        m_oculusapi_initialized = false;
    }

    DeviceTypeManager::shutdown();
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

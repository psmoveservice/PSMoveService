//-- includes -----
#include "DeviceTypeManager.h"
#include "DeviceEnumerator.h"
#include "ServerLog.h"
#include "ServerDeviceView.h"
#include "ServerNetworkManager.h"
#include "ServerUtility.h"

//-- methods -----
/// Constructor and set intervals (ms) for reconnect and polling
DeviceTypeManager::DeviceTypeManager(const int recon_int, const int poll_int)
    : reconnect_interval(recon_int)
    , poll_interval(poll_int)
    , m_deviceViews(nullptr)
{
}

DeviceTypeManager::~DeviceTypeManager()
{
    assert(m_deviceViews == nullptr);
}

/// Override if the device type needs to initialize any services (e.g., hid_init)
bool
DeviceTypeManager::startup()
{
    assert(m_deviceViews == nullptr);

    const int maxDeviceCount = getMaxDevices();
    m_deviceViews = new ServerDeviceViewPtr[maxDeviceCount];

    // Allocate all of the device views
    for (int device_id = 0; device_id < maxDeviceCount; ++device_id)
    {
        ServerDeviceViewPtr deviceView = ServerDeviceViewPtr(allocate_device_view(device_id));

        m_deviceViews[device_id] = deviceView;
    }

    return true;
}

/// Override if the device type needs to teardown any services (e.g., hid_init)
void
DeviceTypeManager::shutdown()
{
    assert(m_deviceViews != nullptr);

    // Close any controllers that were opened
    for (int device_id = 0; device_id < getMaxDevices(); ++device_id)
    {
        ServerDeviceViewPtr device = m_deviceViews[device_id];

        if (device->getIsOpen())
        {
            device->close();
        }

        m_deviceViews[device_id] = ServerDeviceViewPtr();
    }

    // Free the device view pointer list
    delete[] m_deviceViews;
    m_deviceViews = nullptr;
}

/// Calls poll_devices and update_connected_devices if poll_interval and reconnect_interval has elapsed, respectively.
void
DeviceTypeManager::poll()
{
    std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();

    // See if it's time to poll controllers for data
    std::chrono::duration<double, std::milli> update_diff = now - m_last_poll_time;

    if (update_diff.count() >= poll_interval)
    {
        poll_devices();
        m_last_poll_time = now;
    }

    // See if it's time to try update the list of connected devices
    std::chrono::duration<double, std::milli> reconnect_diff = now - m_last_reconnect_time;
    if (reconnect_diff.count() >= reconnect_interval)
    {
        if (update_connected_devices())
        {
            m_last_reconnect_time = now;
        }
    }
}

bool
DeviceTypeManager::update_connected_devices()
{
    bool success = false;

    // Don't do any connection opening/closing until all pending bluetooth operations are finished
    if (can_update_connected_devices())
    {
        const int maxDeviceCount = getMaxDevices();
        ServerDeviceViewPtr *temp_device_list = new ServerDeviceViewPtr[maxDeviceCount];
        bool bSendControllerUpdatedNotification = false;

        // Step 1
        // See if any devices shuffled order OR if any new controllers were attached.
        // Migrate open devices to a new temp list in the order
        // that they appear in the device enumerator.
        {
            DeviceEnumerator *enumerator = allocate_device_enumerator();

            while (enumerator->is_valid())
            {
                // Find device index for the device with the matching device path
                int device_id = find_open_device_device_id(enumerator);

                // Existing device case (Most common)
                if (device_id != -1)
                {
                    // Fetch the device from it's existing device slot
                    ServerDeviceViewPtr existingDevice = getDeviceViewPtr(device_id);

                    // Move it to the same slot in the temp list
                    temp_device_list[device_id] = existingDevice;

                    // Remove it from the previous list
                    m_deviceViews[device_id] = ServerDeviceViewPtr();
                }
                // New controller connected case
                else
                {
                    int device_id = find_first_closed_device_device_id();

                    if (device_id != -1)
                    {
                        // Fetch the controller from it's existing controller slot
                        ServerDeviceViewPtr existingDevice = getDeviceViewPtr(device_id);

                        // Move it to the available slot
                        existingDevice->setDeviceID(static_cast<int>(device_id));
                        temp_device_list[device_id] = existingDevice;

                        // Remove it from the previous list
                        m_deviceViews[device_id] = ServerDeviceViewPtr();

                        // Attempt to open the device
                        if (existingDevice->open(enumerator))
                        {
                            const char *device_type_name =
                                CommonDeviceState::getDeviceTypeString(existingDevice->getDevice()->getDeviceType());

                            SERVER_LOG_INFO("DeviceTypeManager::update_connected_devices") <<
                                "Device device_id " << device_id << " (" << device_type_name << ") connected";
                            bSendControllerUpdatedNotification = true;
                        }
                    }
                    else
                    {
                        SERVER_LOG_ERROR("ControllerManager::reconnect_controllers") << "Can't connect any more new controllers. Too many open controllers";
                        break;
                    }
                }

                enumerator->next();
            }

            free_device_enumerator(enumerator);
        }

        // Step 2
        // Close any remaining open controllers not listed in the device enumerator.
        // Copy over any closed controllers to the temp.
        for (int existing_device_id = 0; existing_device_id < maxDeviceCount; ++existing_device_id)
        {
            ServerDeviceViewPtr existingDevice = getDeviceViewPtr(existing_device_id);

            if (existingDevice)
            {
                // Any "open" controllers remaining in the old list need to be closed
                // since they no longer appear in the connected device list.
                // This probably shouldn't happen very often (at all?) as polling should catch
                // disconnected controllers first.
                if (existingDevice->getIsOpen())
                {
                    const char *device_type_name =
                        CommonDeviceState::getDeviceTypeString(existingDevice->getDevice()->getDeviceType());

                    SERVER_LOG_WARNING("ControllerManager::reconnect_controllers") << "Closing device "
                        << existing_device_id << " (" << device_type_name << ") since it's no longer in the device list.";
                    existingDevice->close();
                    bSendControllerUpdatedNotification = true;
                }

                // Move it to the temp slot
                temp_device_list[existing_device_id] = existingDevice;

                // Remove it from the previous list
                m_deviceViews[existing_device_id] = ServerDeviceViewPtr();
            }
        }

        // Step 3
        // Copy the temp controller list back over top the original list
        for (int device_id = 0; device_id < maxDeviceCount; ++device_id)
        {
            m_deviceViews[device_id] = temp_device_list[device_id];
        }

        if (bSendControllerUpdatedNotification)
        {
            send_device_list_changed_notification();
        }

        delete[] temp_device_list;

        success = true;
    }

    return success;
}

void
DeviceTypeManager::updateStateAndPredict()
{
    // Recompute the state-space data about the device and make predictions about the future
    for (int device_id = 0; device_id < getMaxDevices(); ++device_id)
    {
        ServerDeviceViewPtr device = getDeviceViewPtr(device_id);

        device->updateStateAndPredict();
    }
}

void
DeviceTypeManager::publish()
{
    // Publish any new data to client connections
    for (int device_id = 0; device_id < getMaxDevices(); ++device_id)
    {
        ServerDeviceViewPtr device = getDeviceViewPtr(device_id);

        device->publish();
    }
}

void
DeviceTypeManager::send_device_list_changed_notification()
{
    ResponsePtr response(new PSMoveProtocol::Response);
    response->set_type(getListUpdatedResponseType());
    response->set_request_id(-1);
    response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);

    ServerNetworkManager::get_instance()->send_notification_to_all_clients(response);
}

void
DeviceTypeManager::poll_devices()
{
    bool bAllUpdatedOk = true;

    for (int device_id = 0; device_id < getMaxDevices(); ++device_id)
    {
        ServerDeviceViewPtr device = getDeviceViewPtr(device_id);
        bAllUpdatedOk &= device->poll();
    }

    if (!bAllUpdatedOk)
    {
        send_device_list_changed_notification();
    }
}

int
DeviceTypeManager::find_open_device_device_id(const DeviceEnumerator *enumerator)
{
    int result_device_id = -1;

    for (int device_id = 0; device_id < getMaxDevices(); ++device_id)
    {
        ServerDeviceViewPtr device = getDeviceViewPtr(device_id);

        if (device && device->matchesDeviceEnumerator(enumerator))
        {
            result_device_id = device_id;
            break;
        }
    }

    return result_device_id;
}

int
DeviceTypeManager::find_first_closed_device_device_id()
{
    int result_device_id = -1;
    for (int device_id = 0; device_id < getMaxDevices(); ++device_id)
    {
        ServerDeviceViewPtr device = getDeviceViewPtr(device_id);

        if (device && !device->getIsOpen())
        {
            result_device_id = device_id;
            break;
        }
    }
    return result_device_id;
}

ServerDeviceViewPtr
DeviceTypeManager::getDeviceViewPtr(int device_id)
{
    assert(m_deviceViews != nullptr);

    return m_deviceViews[device_id];
}
#ifndef CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_H

//-- includes -----
#include <memory>
#include "DeviceTypeManager.h"
#include "PSMoveProtocol.pb.h"
#include "TrackerManager.h"

//-- typedefs -----
class ServerControllerView;
typedef std::shared_ptr<ServerControllerView> ServerControllerViewPtr;

//-- definitions -----
class ControllerManager : public DeviceTypeManager
{
public:
    ControllerManager();

    /// Call hid_init()
    bool startup() override;

    /// Call hid_close()
    void shutdown() override;
    
    void updateStateAndPredict(TrackerManager* tracker_manager);

    static const int k_max_devices = 5;
    int getMaxDevices() const override
    {
        return ControllerManager::k_max_devices;
    }

    ServerControllerViewPtr getControllerViewPtr(int device_id);

    bool setControllerRumble(int controller_id, int rumble_amount);
    bool resetPose(int controller_id);

protected:
    bool can_update_connected_devices() override;
    class DeviceEnumerator *allocate_device_enumerator() override;
    void free_device_enumerator(class DeviceEnumerator *) override;
    ServerDeviceView *allocate_device_view(int device_id) override;

    const PSMoveProtocol::Response_ResponseType getListUpdatedResponseType() override
    {
        return ControllerManager::k_list_udpated_response_type;
    }

private:
    static const PSMoveProtocol::Response_ResponseType k_list_udpated_response_type = PSMoveProtocol::Response_ResponseType_CONTROLLER_LIST_UPDATED;
};

#endif // CONTROLLER_MANAGER_H
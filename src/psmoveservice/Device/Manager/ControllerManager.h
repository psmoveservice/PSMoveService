#ifndef CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_H

//-- includes -----
#include "DeviceInterface.h"
#include "DeviceTypeManager.h"
#include "PSMoveProtocol.pb.h"
#include "TrackerManager.h"
#include "MathEigen.h"

#include <memory>
#include <deque>

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

    inline std::string getCachedBluetoothHostAddress() const
    {
        return m_bluetooth_host_address;
    }

    ServerControllerViewPtr getControllerViewPtr(int device_id);

    void setControllerRumble(int controller_id, float rumble_amount, CommonControllerState::RumbleChannel channel);

    eCommonTrackingColorID allocateTrackingColorID();
    void claimTrackingColorID(const ServerControllerView *controller_view, eCommonTrackingColorID color_id);
    void freeTrackingColorID(eCommonTrackingColorID color_id);

protected:
	// Fetch latest controller state
	void poll_devices() override;

	// Controller enumerator methods
    class DeviceEnumerator *allocate_device_enumerator() override;
    void free_device_enumerator(class DeviceEnumerator *) override;
    ServerDeviceView *allocate_device_view(int device_id) override;
	int getListUpdatedResponseType() override;

private:
    static const PSMoveProtocol::Response_ResponseType k_list_udpated_response_type = PSMoveProtocol::Response_ResponseType_CONTROLLER_LIST_UPDATED;
    std::deque<eCommonTrackingColorID> m_available_controller_color_ids;
    std::string m_bluetooth_host_address;
};

#endif // CONTROLLER_MANAGER_H
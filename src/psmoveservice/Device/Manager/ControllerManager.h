#ifndef CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_H

//-- includes -----
#include "DeviceInterface.h"
#include "DeviceTypeManager.h"
#include "PSMoveProtocol.pb.h"
#include "TrackerManager.h"
#include "MathEigen.h"

#include <memory>

//-- typedefs -----
class ServerControllerView;
typedef std::shared_ptr<ServerControllerView> ServerControllerViewPtr;

//-- definitions -----
class ControllerManagerConfig : public PSMoveConfig
{
public:
    static const int CONFIG_VERSION;

    ControllerManagerConfig(const std::string &fnamebase = "ControllerManagerConfig");

    virtual const boost::property_tree::ptree config2ptree();
    virtual void ptree2config(const boost::property_tree::ptree &pt);

    int version;
    int virtual_controller_count;
};

class ControllerManager : public DeviceTypeManager
{
public:
    ControllerManager();

    /// Call hid_init()
    bool startup() override;

    /// Call hid_close()
    void shutdown() override;
    
    void updateStateAndPredict(TrackerManager* tracker_manager);
    void publish() override;

    inline const ControllerManagerConfig& getConfig() const
    {
        return cfg;
    }

    static const int k_max_devices = PSMOVESERVICE_MAX_CONTROLLER_COUNT;
    int getMaxDevices() const override
    {
        return ControllerManager::k_max_devices;
    }

    int getGamepadCount() const;

    inline std::string getCachedBluetoothHostAddress() const
    {
        return m_bluetooth_host_address;
    }

    ServerControllerViewPtr getControllerViewPtr(int device_id);

    void setControllerRumble(int controller_id, float rumble_amount, CommonControllerState::RumbleChannel channel);

protected:
	// Fetch latest controller state
	void poll_devices() override;

	// Controller enumerator methods
    class DeviceEnumerator *allocate_device_enumerator() override;
    void free_device_enumerator(class DeviceEnumerator *) override;
    ServerDeviceView *allocate_device_view(int device_id) override;
	int getListUpdatedResponseType() override;

public:
	bool gamepad_api_enabled;

private:
    static const PSMoveProtocol::Response_ResponseType k_list_udpated_response_type = PSMoveProtocol::Response_ResponseType_CONTROLLER_LIST_UPDATED;
    std::string m_bluetooth_host_address;
    ControllerManagerConfig cfg;
};

#endif // CONTROLLER_MANAGER_H
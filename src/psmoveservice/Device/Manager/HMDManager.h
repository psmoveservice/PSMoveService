#ifndef HMD_MANAGER_H
#define HMD_MANAGER_H

//-- includes -----
#include <memory>
#include "DeviceTypeManager.h"
#include "DeviceEnumerator.h"
#include "PSMoveProtocol.pb.h"

//-- typedefs -----

class ServerHMDView;
typedef std::shared_ptr<ServerHMDView> ServerHMDViewPtr;

//-- definitions -----

class HMDManager : public DeviceTypeManager
{
public:
    HMDManager();

    virtual bool startup() override;
    virtual void shutdown() override;

    inline bool getIsOculusAPIInitialized() const 
    {
        return m_oculusapi_initialized;
    }

    static const int k_max_devices = 1;
    int getMaxDevices() const override
    {
        return HMDManager::k_max_devices;
    }

    ServerHMDViewPtr getHMDViewPtr(int device_id);

protected:
    bool can_update_connected_devices() override;
    class DeviceEnumerator *allocate_device_enumerator() override;
    void free_device_enumerator(class DeviceEnumerator *) override;
    ServerDeviceView *allocate_device_view(int device_id) override;

    const PSMoveProtocol::Response_ResponseType getListUpdatedResponseType() override
    {
        return HMDManager::k_list_udpated_response_type;
    }

private:
    bool m_oculusapi_initialized;
    static const PSMoveProtocol::Response_ResponseType k_list_udpated_response_type = PSMoveProtocol::Response_ResponseType_HMD_LIST_UPDATED;
};

#endif // HMD_MANAGER_H
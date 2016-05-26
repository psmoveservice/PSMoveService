#ifndef TRACKER_MANAGER_H
#define TRACKER_MANAGER_H

//-- includes -----
#include <memory>
#include "DeviceTypeManager.h"
#include "DeviceEnumerator.h"
#include "DeviceInterface.h"
#include "PSMoveConfig.h"
#include "PSMoveProtocol.pb.h"

//-- typedefs -----

class ServerTrackerView;
typedef std::shared_ptr<ServerTrackerView> ServerTrackerViewPtr;

//-- definitions -----
struct TrackerProfile
{
    std::string profile_name;
    float exposure;
    float gain;
    CommonHSVColorRange color_presets[eCommonTrackColorType::MAX_TRACKING_COLOR_TYPES];

    void clear();
};

class TrackerManagerConfig : public PSMoveConfig
{
public:
    static const int CONFIG_VERSION;

    TrackerManagerConfig(const std::string &fnamebase = "TrackerManagerConfig")
        : PSMoveConfig(fnamebase)
        , version(CONFIG_VERSION)
    {
        magnetometer_ellipsoid.clear();
        magnetometer_identity = Eigen::Vector3f::Zero();
    };

    virtual const boost::property_tree::ptree config2ptree();
    virtual void ptree2config(const boost::property_tree::ptree &pt);

    long version;
};

class TrackerManager : public DeviceTypeManager
{
public:
    TrackerManager();

    static const int k_max_devices = 2;
    int getMaxDevices() const override
    {
        return TrackerManager::k_max_devices;
    }

    ServerTrackerViewPtr getTrackerViewPtr(int device_id);

    void saveDefaultTrackerProfile(float exposure, float gain, struct CommonHSVColorRange *colorPresets, int presetCount);

protected:
    bool can_update_connected_devices() override;
    DeviceEnumerator *allocate_device_enumerator() override;
    void free_device_enumerator(DeviceEnumerator *) override;
    ServerDeviceView *allocate_device_view(int device_id) override;

    const PSMoveProtocol::Response_ResponseType getListUpdatedResponseType() override
    {
        return TrackerManager::k_list_udpated_response_type;
    }

private:
    static const PSMoveProtocol::Response_ResponseType k_list_udpated_response_type = PSMoveProtocol::Response_ResponseType_TRACKER_LIST_UPDATED;

    TrackerManagerConfig cfg;
};

#endif // TRACKER_MANAGER_H
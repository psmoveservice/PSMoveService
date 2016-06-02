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

//-- constants -----
extern const CommonHSVColorRange *k_default_color_presets;

//-- definitions -----
struct TrackerProfile
{
    float exposure;
    float gain;
    CommonHSVColorRange color_presets[eCommonTrackingColorID::MAX_TRACKING_COLOR_TYPES];

    inline void clear()
    {
        exposure = 0.f;
        gain = 0;
        for (int preset_index = 0; preset_index < eCommonTrackingColorID::MAX_TRACKING_COLOR_TYPES; ++preset_index)
        {
            color_presets[preset_index].clear();
        }
    }
};

class TrackerManagerConfig : public PSMoveConfig
{
public:
    static const int CONFIG_VERSION;

    TrackerManagerConfig(const std::string &fnamebase = "TrackerManagerConfig");

    virtual const boost::property_tree::ptree config2ptree();
    virtual void ptree2config(const boost::property_tree::ptree &pt);

    long version;
    CommonDevicePose hmd_tracking_origin_pose;
    TrackerProfile default_tracker_profile;
};

class TrackerManager : public DeviceTypeManager
{
public:
    TrackerManager();

    bool startup() override;

    static const int k_max_devices = 2;
    int getMaxDevices() const override
    {
        return TrackerManager::k_max_devices;
    }

    ServerTrackerViewPtr getTrackerViewPtr(int device_id);

    inline void saveDefaultTrackerProfile(const TrackerProfile *profile)
    {
        cfg.default_tracker_profile = *profile;
        cfg.save();
    }

    inline const TrackerProfile *getDefaultTrackerProfile() const
    {
        return &cfg.default_tracker_profile; 
    }

    inline void setHmdTrackingOriginPose(const CommonDevicePose &pose)
    {
        cfg.hmd_tracking_origin_pose= pose;
        cfg.save();
    }

    inline CommonDevicePose getHmdTrackingOriginPose()
    {
        return cfg.hmd_tracking_origin_pose;
    }

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
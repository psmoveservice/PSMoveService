#ifndef TRACKER_MANAGER_H
#define TRACKER_MANAGER_H

//-- includes -----
#include <memory>
#include "DeviceTypeManager.h"
#include "DeviceEnumerator.h"
#include "DeviceInterface.h"
#include "PSMoveConfig.h"

//-- typedefs -----

class ServerTrackerView;
typedef std::shared_ptr<ServerTrackerView> ServerTrackerViewPtr;

//-- definitions -----
struct TrackerProfile
{
    float exposure;
    float gain;
    CommonHSVColorRangeTable color_preset_table;

    inline void clear()
    {
        exposure = 0.f;
        gain = 0;
		color_preset_table.table_name.clear();
        for (int preset_index = 0; preset_index < eCommonTrackingColorID::MAX_TRACKING_COLOR_TYPES; ++preset_index)
        {
            color_preset_table.color_presets[preset_index].clear();
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
    int optical_tracking_timeout;
	int tracker_sleep_ms;
	bool use_bgr_to_hsv_lookup_table;
	bool exclude_opposed_cameras;
	int min_valid_projection_area;
	bool disable_roi;
    TrackerProfile default_tracker_profile;
	float global_forward_degrees;

	CommonDeviceVector get_global_forward_axis() const;
	CommonDeviceVector get_global_backward_axis() const;
	CommonDeviceVector get_global_right_axis() const;
	CommonDeviceVector get_global_left_axis() const;
	CommonDeviceVector get_global_up_axis() const;
	CommonDeviceVector get_global_down_axis() const;
};

class TrackerManager : public DeviceTypeManager
{
public:
    TrackerManager();

    bool startup() override;

    void closeAllTrackers();

    static const int k_max_devices = 4;
    int getMaxDevices() const override
    {
        return TrackerManager::k_max_devices;
    }

    ServerTrackerViewPtr getTrackerViewPtr(int device_id) const;

    inline void saveDefaultTrackerProfile(const TrackerProfile *profile)
    {
        cfg.default_tracker_profile = *profile;
        cfg.save();
    }

    inline const TrackerProfile *getDefaultTrackerProfile() const
    {
        return &cfg.default_tracker_profile; 
    }

    inline const TrackerManagerConfig& getConfig() const
    {
        return cfg;
    }

protected:
    bool can_update_connected_devices() override;
    void mark_tracker_list_dirty();

    DeviceEnumerator *allocate_device_enumerator() override;
    void free_device_enumerator(DeviceEnumerator *) override;
    ServerDeviceView *allocate_device_view(int device_id) override;
	int getListUpdatedResponseType() override;

private:
    TrackerManagerConfig cfg;
    bool m_tracker_list_dirty;
};

#endif // TRACKER_MANAGER_H
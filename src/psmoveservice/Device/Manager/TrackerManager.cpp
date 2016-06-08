//-- includes -----
#include "TrackerManager.h"
#include "TrackerDeviceEnumerator.h"
#include "ServerLog.h"
#include "ServerTrackerView.h"
#include "ServerDeviceView.h"

//-- constants -----
// Format: {hue center, hue range}, {sat center, sat range}, {val center, val range}
// All hue angles are 60 degrees apart to maximize hue separation for 6 max tracked colors.
// Hue angle reference: http://i.imgur.com/PKjgfFXm.jpg 
// Hue angles divide by 2 for opencv which remaps hue range to [0,180]
const CommonHSVColorRange g_default_color_presets[] = {
    { { 300 / 2, 10 }, { 255, 32 }, { 255, 32 } }, // Magenta
    { { 180 / 2, 10 }, { 255, 32 }, { 255, 32 } }, // Cyan
    { { 60 / 2, 10 }, { 255, 32 }, { 255, 32 } }, // Yellow
    { { 0, 10 }, { 255, 32 }, { 255, 32 } }, // Red
    { { 120 / 2, 10 }, { 255, 32 }, { 255, 32 } }, // Green
    { { 240 / 2, 10 }, { 255, 32 }, { 255, 32 } }, // Blue
};
const CommonHSVColorRange *k_default_color_presets = g_default_color_presets;

//-- Tracker Manager Config -----
const int TrackerManagerConfig::CONFIG_VERSION = 2;

TrackerManagerConfig::TrackerManagerConfig(const std::string &fnamebase)
    : PSMoveConfig(fnamebase)
{
    default_tracker_profile.exposure = 32;
    default_tracker_profile.gain = 32;
    for (int preset_index = 0; preset_index < eCommonTrackingColorID::MAX_TRACKING_COLOR_TYPES; ++preset_index)
    {
        default_tracker_profile.color_presets[preset_index] = k_default_color_presets[preset_index];
    }

    hmd_tracking_origin_pose.clear();
};

const boost::property_tree::ptree
TrackerManagerConfig::config2ptree()
{
    boost::property_tree::ptree pt;

    pt.put("version", TrackerManagerConfig::CONFIG_VERSION);
    pt.put("default_tracker_profile.exposure", default_tracker_profile.exposure);
    pt.put("default_tracker_profile.gain", default_tracker_profile.gain);

    pt.put("hmd_tracking_space.orientation.w", hmd_tracking_origin_pose.Orientation.w);
    pt.put("hmd_tracking_space.orientation.x", hmd_tracking_origin_pose.Orientation.x);
    pt.put("hmd_tracking_space.orientation.y", hmd_tracking_origin_pose.Orientation.y);
    pt.put("hmd_tracking_space.orientation.z", hmd_tracking_origin_pose.Orientation.z);
    pt.put("hmd_tracking_space.position.x", hmd_tracking_origin_pose.Position.x);
    pt.put("hmd_tracking_space.position.y", hmd_tracking_origin_pose.Position.y);
    pt.put("hmd_tracking_space.position.z", hmd_tracking_origin_pose.Position.z);

    writeColorPreset(pt, "default_tracker_profile", "magenta", &default_tracker_profile.color_presets[eCommonTrackingColorID::Magenta]);
    writeColorPreset(pt, "default_tracker_profile", "cyan", &default_tracker_profile.color_presets[eCommonTrackingColorID::Cyan]);
    writeColorPreset(pt, "default_tracker_profile", "yellow", &default_tracker_profile.color_presets[eCommonTrackingColorID::Yellow]);
    writeColorPreset(pt, "default_tracker_profile", "red", &default_tracker_profile.color_presets[eCommonTrackingColorID::Red]);
    writeColorPreset(pt, "default_tracker_profile", "green", &default_tracker_profile.color_presets[eCommonTrackingColorID::Green]);
    writeColorPreset(pt, "default_tracker_profile", "blue", &default_tracker_profile.color_presets[eCommonTrackingColorID::Blue]);

    return pt;
}

void
TrackerManagerConfig::ptree2config(const boost::property_tree::ptree &pt)
{
    version = pt.get<int>("version", 0);

    if (version == TrackerManagerConfig::CONFIG_VERSION)
    {
        default_tracker_profile.exposure = pt.get<float>("default_tracker_profile.exposure", 32);
        default_tracker_profile.gain = pt.get<float>("default_tracker_profile.gain", 32);

        hmd_tracking_origin_pose.Orientation.w = pt.get<float>("hmd_tracking_space.orientation.w", 1.0);
        hmd_tracking_origin_pose.Orientation.x = pt.get<float>("hmd_tracking_space.orientation.x", 0.0);
        hmd_tracking_origin_pose.Orientation.y = pt.get<float>("hmd_tracking_space.orientation.y", 0.0);
        hmd_tracking_origin_pose.Orientation.z = pt.get<float>("hmd_tracking_space.orientation.z", 0.0);
        hmd_tracking_origin_pose.Position.x = pt.get<float>("hmd_tracking_space.position.x", 0.0);
        hmd_tracking_origin_pose.Position.y = pt.get<float>("hmd_tracking_space.position.y", 0.0);
        hmd_tracking_origin_pose.Position.z = pt.get<float>("hmd_tracking_space.position.z", 0.0);

        readColorPreset(pt, "default_tracker_profile", "magenta", &default_tracker_profile.color_presets[eCommonTrackingColorID::Magenta], &k_default_color_presets[eCommonTrackingColorID::Magenta]);
        readColorPreset(pt, "default_tracker_profile", "cyan", &default_tracker_profile.color_presets[eCommonTrackingColorID::Cyan], &k_default_color_presets[eCommonTrackingColorID::Cyan]);
        readColorPreset(pt, "default_tracker_profile", "yellow", &default_tracker_profile.color_presets[eCommonTrackingColorID::Yellow], &k_default_color_presets[eCommonTrackingColorID::Yellow]);
        readColorPreset(pt, "default_tracker_profile", "red", &default_tracker_profile.color_presets[eCommonTrackingColorID::Red], &k_default_color_presets[eCommonTrackingColorID::Red]);
        readColorPreset(pt, "default_tracker_profile", "green", &default_tracker_profile.color_presets[eCommonTrackingColorID::Green], &k_default_color_presets[eCommonTrackingColorID::Green]);
        readColorPreset(pt, "default_tracker_profile", "blue", &default_tracker_profile.color_presets[eCommonTrackingColorID::Blue], &k_default_color_presets[eCommonTrackingColorID::Blue]);
    }
    else
    {
        SERVER_LOG_WARNING("TrackerManagerConfig") <<
            "Config version " << version << " does not match expected version " <<
            TrackerManagerConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

//-- Tracker Manager -----
TrackerManager::TrackerManager()
    : DeviceTypeManager(10000, 13)
    , m_tracker_list_dirty(false)
{
}

bool 
TrackerManager::startup()
{
    bool bSuccess = DeviceTypeManager::startup();

    if (bSuccess)
    {
        if (!cfg.load())
        {
            // Save out the defaults if there is no config to load
            cfg.save();
        }

        // Refresh the tracker list
        mark_tracker_list_dirty();
    }

    return bSuccess;
}

void
TrackerManager::closeAllTrackers()
{
    for (int tracker_id = 0; tracker_id < k_max_devices; ++tracker_id)
    {
        ServerTrackerViewPtr tracker_view = getTrackerViewPtr(tracker_id);

        if (tracker_view->getIsOpen())
        {
            tracker_view->close();
        }
    }

    // Refresh the tracker list once we're allowed to
    mark_tracker_list_dirty();

    // Tell any clients that the tracker list changed
    send_device_list_changed_notification();
}

bool
TrackerManager::can_update_connected_devices()
{
    return m_tracker_list_dirty && DeviceTypeManager::can_update_connected_devices();
}

void 
TrackerManager::mark_tracker_list_dirty()
{
    m_tracker_list_dirty= true;
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


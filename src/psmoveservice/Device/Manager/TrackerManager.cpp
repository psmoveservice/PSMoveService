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
const int TrackerManagerConfig::CONFIG_VERSION = 1;

TrackerManagerConfig::TrackerManagerConfig(const std::string &fnamebase)
    : PSMoveConfig(fnamebase)
{
    default_tracker_profile.exposure = 32;
    default_tracker_profile.gain = 32;
    for (int preset_index = 0; preset_index < eCommonTrackColorType::MAX_TRACKING_COLOR_TYPES; ++preset_index)
    {
        default_tracker_profile.color_presets[preset_index] = k_default_color_presets[preset_index];
    }
};

const boost::property_tree::ptree
TrackerManagerConfig::config2ptree()
{
    boost::property_tree::ptree pt;

    pt.put("version", TrackerManagerConfig::CONFIG_VERSION);
    pt.put("default_tracker_profile.exposure", default_tracker_profile.exposure);
    pt.put("default_tracker_profile.gain", default_tracker_profile.gain);

    writeColorPreset(pt, "default_tracker_profile", "magenta", &default_tracker_profile.color_presets[eCommonTrackColorType::Magenta]);
    writeColorPreset(pt, "default_tracker_profile", "cyan", &default_tracker_profile.color_presets[eCommonTrackColorType::Cyan]);
    writeColorPreset(pt, "default_tracker_profile", "yellow", &default_tracker_profile.color_presets[eCommonTrackColorType::Yellow]);
    writeColorPreset(pt, "default_tracker_profile", "red", &default_tracker_profile.color_presets[eCommonTrackColorType::Red]);
    writeColorPreset(pt, "default_tracker_profile", "green", &default_tracker_profile.color_presets[eCommonTrackColorType::Green]);
    writeColorPreset(pt, "default_tracker_profile", "blue", &default_tracker_profile.color_presets[eCommonTrackColorType::Blue]);

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

        readColorPreset(pt, "default_tracker_profile", "magenta", &default_tracker_profile.color_presets[eCommonTrackColorType::Magenta], &k_default_color_presets[eCommonTrackColorType::Magenta]);
        readColorPreset(pt, "default_tracker_profile", "cyan", &default_tracker_profile.color_presets[eCommonTrackColorType::Cyan], &k_default_color_presets[eCommonTrackColorType::Cyan]);
        readColorPreset(pt, "default_tracker_profile", "yellow", &default_tracker_profile.color_presets[eCommonTrackColorType::Yellow], &k_default_color_presets[eCommonTrackColorType::Yellow]);
        readColorPreset(pt, "default_tracker_profile", "red", &default_tracker_profile.color_presets[eCommonTrackColorType::Red], &k_default_color_presets[eCommonTrackColorType::Red]);
        readColorPreset(pt, "default_tracker_profile", "green", &default_tracker_profile.color_presets[eCommonTrackColorType::Green], &k_default_color_presets[eCommonTrackColorType::Green]);
        readColorPreset(pt, "default_tracker_profile", "blue", &default_tracker_profile.color_presets[eCommonTrackColorType::Blue], &k_default_color_presets[eCommonTrackColorType::Blue]);
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
    }

    return bSuccess;
}

bool
TrackerManager::can_update_connected_devices()
{
    return true;
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


//-- includes -----
#include "TrackerManager.h"
#include "TrackerDeviceEnumerator.h"
#include "ControllerManager.h"
#include "DeviceManager.h"
#include "HMDManager.h"
#include "ServerLog.h"
#include "ServerControllerView.h"
#include "ServerHMDView.h"
#include "ServerTrackerView.h"
#include "ServerDeviceView.h"
#include "MathUtility.h"
#include "PSMoveProtocol.pb.h"

//-- constants -----

//-- Tracker Manager Config -----
const int TrackerManagerConfig::CONFIG_VERSION = 2;

TrackerManagerConfig::TrackerManagerConfig(const std::string &fnamebase)
    : PSMoveConfig(fnamebase)
{

	controller_position_smoothing = 0.f;
	ignore_pose_from_one_tracker = false;
    optical_tracking_timeout= 100;
	tracker_sleep_ms = 1;
	use_bgr_to_hsv_lookup_table = true;
	exclude_opposed_cameras = false;
	min_valid_projection_area= 16;
	disable_roi = false;
	default_tracker_profile.frame_width = 640;
	//default_tracker_profile.frame_height = 480;
	default_tracker_profile.frame_rate = 40;
    default_tracker_profile.exposure = 32;
    default_tracker_profile.gain = 32;
	default_tracker_profile.color_preset_table.table_name= "default_tracker_profile";
	global_forward_degrees = 270.f; // Down -Z by default
    for (int preset_index = 0; preset_index < eCommonTrackingColorID::MAX_TRACKING_COLOR_TYPES; ++preset_index)
    {
        default_tracker_profile.color_preset_table.color_presets[preset_index] = k_default_color_presets[preset_index];
    }
};

const boost::property_tree::ptree
TrackerManagerConfig::config2ptree()
{
    boost::property_tree::ptree pt;

    pt.put("version", TrackerManagerConfig::CONFIG_VERSION);

	pt.put("controller_position_smoothing", controller_position_smoothing);
	pt.put("ignore_pose_from_one_tracker", ignore_pose_from_one_tracker);
    pt.put("optical_tracking_timeout", optical_tracking_timeout);
	pt.put("use_bgr_to_hsv_lookup_table", use_bgr_to_hsv_lookup_table);
	pt.put("tracker_sleep_ms", tracker_sleep_ms);

	pt.put("excluded_opposed_cameras", exclude_opposed_cameras);	

	pt.put("min_valid_projection_area", min_valid_projection_area);	

	pt.put("disable_roi", disable_roi);

	pt.put("default_tracker_profile.frame_width", default_tracker_profile.frame_width);
	//pt.put("default_tracker_profile.frame_height", default_tracker_profile.frame_height);
	pt.put("default_tracker_profile.frame_rate", default_tracker_profile.frame_rate);
    pt.put("default_tracker_profile.exposure", default_tracker_profile.exposure);
    pt.put("default_tracker_profile.gain", default_tracker_profile.gain);

	pt.put("global_forward_degrees", global_forward_degrees);

	writeColorPropertyPresetTable(&default_tracker_profile.color_preset_table, pt);

    return pt;
}

void
TrackerManagerConfig::ptree2config(const boost::property_tree::ptree &pt)
{
    version = pt.get<int>("version", 0);

    if (version == TrackerManagerConfig::CONFIG_VERSION)
    {
		controller_position_smoothing = pt.get<float>("controller_position_smoothing", controller_position_smoothing);
		ignore_pose_from_one_tracker = pt.get<bool>("ignore_pose_from_one_tracker", ignore_pose_from_one_tracker);
        optical_tracking_timeout= pt.get<int>("optical_tracking_timeout", optical_tracking_timeout);
		use_bgr_to_hsv_lookup_table = pt.get<bool>("use_bgr_to_hsv_lookup_table", use_bgr_to_hsv_lookup_table);
		tracker_sleep_ms = pt.get<int>("tracker_sleep_ms", tracker_sleep_ms);
		exclude_opposed_cameras = pt.get<bool>("excluded_opposed_cameras", exclude_opposed_cameras);
		min_valid_projection_area = pt.get<float>("min_valid_projection_area", min_valid_projection_area);	
		disable_roi = pt.get<bool>("disable_roi", disable_roi);
		default_tracker_profile.frame_width = pt.get<float>("default_tracker_profile.frame_width", 640);
		//default_tracker_profile.frame_height = pt.get<float>("default_tracker_profile.frame_height", 480);
		default_tracker_profile.frame_rate = pt.get<float>("default_tracker_profile.frame_rate", 40);
        default_tracker_profile.exposure = pt.get<float>("default_tracker_profile.exposure", 32);
        default_tracker_profile.gain = pt.get<float>("default_tracker_profile.gain", 32);

		global_forward_degrees= pt.get<float>("global_forward_degrees", global_forward_degrees);

		readColorPropertyPresetTable(pt, &default_tracker_profile.color_preset_table);
    }
    else
    {
        SERVER_LOG_WARNING("TrackerManagerConfig") <<
            "Config version " << version << " does not match expected version " <<
            TrackerManagerConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

CommonDeviceVector 
TrackerManagerConfig::get_global_forward_axis() const
{
	return CommonDeviceVector::create(cosf(global_forward_degrees*k_degrees_to_radians), 0.f, sinf(global_forward_degrees*k_degrees_to_radians));
}

CommonDeviceVector 
TrackerManagerConfig::get_global_backward_axis() const
{
	return CommonDeviceVector::create(-cosf(global_forward_degrees*k_degrees_to_radians), 0.f, -sinf(global_forward_degrees*k_degrees_to_radians));
}

CommonDeviceVector
TrackerManagerConfig::get_global_right_axis() const
{
	return CommonDeviceVector::create(-sinf(global_forward_degrees*k_degrees_to_radians), 0.f, cosf(global_forward_degrees*k_degrees_to_radians));
}

CommonDeviceVector
TrackerManagerConfig::get_global_left_axis() const
{
	return CommonDeviceVector::create(sinf(global_forward_degrees*k_degrees_to_radians), 0.f, -cosf(global_forward_degrees*k_degrees_to_radians));
}

CommonDeviceVector 
TrackerManagerConfig::get_global_up_axis() const
{
	return CommonDeviceVector::create(0.f, 1.f, 0.f);
}

CommonDeviceVector 
TrackerManagerConfig::get_global_down_axis() const
{
	return CommonDeviceVector::create(0.f, -1.f, 0.f);
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
		// Load any config from disk
		cfg.load();

        // Save back out the config in case there were updated defaults
        cfg.save();

        // Refresh the tracker list
        mark_tracker_list_dirty();

        // Put all of the available tracking colors in the queue
        for (int color_index = 0; color_index < eCommonTrackingColorID::MAX_TRACKING_COLOR_TYPES; ++color_index)
        {
            m_available_color_ids.push_back(static_cast<eCommonTrackingColorID>(color_index));
        }
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

    // Tracker list is no longer dirty after we have iterated through the list of cameras
    m_tracker_list_dirty = false;
}

ServerDeviceView *
TrackerManager::allocate_device_view(int device_id)
{
    return new ServerTrackerView(device_id);
}

ServerTrackerViewPtr
TrackerManager::getTrackerViewPtr(int device_id) const
{
    assert(m_deviceViews != nullptr);

    return std::static_pointer_cast<ServerTrackerView>(m_deviceViews[device_id]);
}

int TrackerManager::getListUpdatedResponseType()
{
	return PSMoveProtocol::Response_ResponseType_TRACKER_LIST_UPDATED;
}

eCommonTrackingColorID 
TrackerManager::allocateTrackingColorID()
{
    assert(m_available_color_ids.size() > 0);
    eCommonTrackingColorID tracking_color = m_available_color_ids.front();

    m_available_color_ids.pop_front();

    return tracking_color;
}

bool 
TrackerManager::claimTrackingColorID(const ServerControllerView *claiming_controller_view, eCommonTrackingColorID color_id)
{
    bool bColorWasInUse = false;
    bool bSuccess= true;

    // If any other controller has this tracking color, make them pick a new color (if possible)
    HMDManager *hmdManager= DeviceManager::getInstance()->m_hmd_manager;
    for (int device_id = 0; device_id < hmdManager->getMaxDevices(); ++device_id)
    {
        ServerHMDViewPtr hmd_view = hmdManager->getHMDViewPtr(device_id);

        if (hmd_view->getIsOpen())
        {
            if (hmd_view->getTrackingColorID() == color_id)
            {
                eCommonTrackingColorID newTrackingColor= allocateTrackingColorID();

                if (!hmd_view->setTrackingColorID(newTrackingColor))
                {
                    freeTrackingColorID(newTrackingColor);
                    bSuccess= false;
                }

                bColorWasInUse = true;
                break;
            }
        }
    }

    // If any other controller has this tracking color, make them pick a new color
    if (!bColorWasInUse)
    {
        // If any other controller has this tracking color, make them pick a new color
        ControllerManager *controllerManager= DeviceManager::getInstance()->m_controller_manager;
        for (int device_id = 0; device_id < controllerManager->getMaxDevices(); ++device_id)
        {
            ServerControllerViewPtr controller_view = controllerManager->getControllerViewPtr(device_id);

            if (controller_view->getIsOpen() && controller_view.get() != claiming_controller_view)
            {
                if (controller_view->getTrackingColorID() == color_id)
                {
                    controller_view->setTrackingColorID(allocateTrackingColorID());
                    bColorWasInUse = true;
                    break;
                }
            }
        }
    }

    // If the color was not in use, remove it from the color queue
    if (!bColorWasInUse)
    {
        for (auto iter = m_available_color_ids.begin(); iter != m_available_color_ids.end(); ++iter)
        {
            if (*iter == color_id)
            {
                m_available_color_ids.erase(iter);
                break;
            }
        }
    }

    return bSuccess;
}

bool 
TrackerManager::claimTrackingColorID(const ServerHMDView *claiming_hmd_view, eCommonTrackingColorID color_id)
{
    bool bColorWasInUse = false;
    bool bSuccess= true;

    // If any other controller has this tracking color, make them pick a new color (if possible)
    HMDManager *hmdManager= DeviceManager::getInstance()->m_hmd_manager;
    for (int device_id = 0; device_id < hmdManager->getMaxDevices(); ++device_id)
    {
        ServerHMDViewPtr hmd_view = hmdManager->getHMDViewPtr(device_id);

        if (hmd_view->getIsOpen() && hmd_view.get() != claiming_hmd_view)
        {
            if (hmd_view->getTrackingColorID() == color_id)
            {
                eCommonTrackingColorID newTrackingColor= allocateTrackingColorID();

                if (!hmd_view->setTrackingColorID(newTrackingColor))
                {
                    freeTrackingColorID(newTrackingColor);
                    bSuccess= false;
                }

                bColorWasInUse = true;
                break;
            }
        }
    }

    // If any other controller has this tracking color, make them pick a new color
    if (!bColorWasInUse)
    {
        // If any other controller has this tracking color, make them pick a new color
        ControllerManager *controllerManager= DeviceManager::getInstance()->m_controller_manager;
        for (int device_id = 0; device_id < controllerManager->getMaxDevices(); ++device_id)
        {
            ServerControllerViewPtr controller_view = controllerManager->getControllerViewPtr(device_id);

            if (controller_view->getIsOpen())
            {
                if (controller_view->getTrackingColorID() == color_id)
                {
                    controller_view->setTrackingColorID(allocateTrackingColorID());
                    bColorWasInUse = true;
                    break;
                }
            }
        }
    }

    // If the color was not in use, remove it from the color queue
    if (!bColorWasInUse)
    {
        for (auto iter = m_available_color_ids.begin(); iter != m_available_color_ids.end(); ++iter)
        {
            if (*iter == color_id)
            {
                m_available_color_ids.erase(iter);
                break;
            }
        }
    }

    return bSuccess;
}

void 
TrackerManager::freeTrackingColorID(eCommonTrackingColorID color_id)
{
    assert(std::find(m_available_color_ids.begin(), m_available_color_ids.end(), color_id) == m_available_color_ids.end());
    m_available_color_ids.push_back(color_id);
}

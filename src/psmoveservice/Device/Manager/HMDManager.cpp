//-- includes -----
#include "HMDManager.h"
#include "HMDDeviceEnumerator.h"
#include "ServerLog.h"
#include "ServerHMDView.h"
#include "ServerDeviceView.h"
#include "PSMoveProtocol.pb.h"
#include <boost/foreach.hpp>
#include "VirtualHMDDeviceEnumerator.h"

//-- methods -----
//-- Tracker Manager Config -----
const int HMDManagerConfig::CONFIG_VERSION = 1;

HMDManagerConfig::HMDManagerConfig(const std::string &fnamebase)
    : PSMoveConfig(fnamebase)
    , virtual_hmd_count(0)
{

};

const boost::property_tree::ptree
HMDManagerConfig::config2ptree()
{
    boost::property_tree::ptree pt;

    pt.put("version", HMDManagerConfig::CONFIG_VERSION);
    pt.put("virtual_hmd_count", virtual_hmd_count);

    return pt;
}

void
HMDManagerConfig::ptree2config(const boost::property_tree::ptree &pt)
{
    version = pt.get<int>("version", 0);

    if (version == HMDManagerConfig::CONFIG_VERSION)
    {
        virtual_hmd_count = pt.get<int>("virtual_hmd_count", 0);
    }
    else
    {
        SERVER_LOG_WARNING("HMDManagerConfig") <<
            "Config version " << version << " does not match expected version " <<
            HMDManagerConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

//-- HMD Manager -----
HMDManager::HMDManager()
    : DeviceTypeManager(1000, 2)
{
}

bool
HMDManager::startup()
{
    bool success = false;

    if (DeviceTypeManager::startup())
    {
		// Load any config from disk
		cfg.load();

        // Save back out the config in case there were updated defaults
        cfg.save();

        // Copy the virtual controller count into the Virtual controller enumerator static variable.
        // This breaks the dependency between the Controller Manager and the enumerator.
        VirtualHMDDeviceEnumerator::virtual_hmd_count= cfg.virtual_hmd_count;

        success = true;
    }

    return success;
}

void
HMDManager::shutdown()
{
    DeviceTypeManager::shutdown();
}

void
HMDManager::updateStateAndPredict(TrackerManager* tracker_manager)
{
	for (int device_id = 0; device_id < getMaxDevices(); ++device_id)
	{
		ServerHMDViewPtr hmdView = getHMDViewPtr(device_id);

		if (hmdView->getIsOpen())
		{
			hmdView->updateOpticalPoseEstimation(tracker_manager);
			hmdView->updateStateAndPredict();
		}
	}
}

ServerHMDViewPtr
HMDManager::getHMDViewPtr(int device_id)
{
    assert(m_deviceViews != nullptr);

    return std::static_pointer_cast<ServerHMDView>(m_deviceViews[device_id]);
}

bool
HMDManager::can_update_connected_devices()
{
    return true;
}

DeviceEnumerator *
HMDManager::allocate_device_enumerator()
{
    return new HMDDeviceEnumerator(HMDDeviceEnumerator::CommunicationType_ALL);
}

void
HMDManager::free_device_enumerator(DeviceEnumerator *enumerator)
{
    delete static_cast<HMDDeviceEnumerator *>(enumerator);
}

ServerDeviceView *
HMDManager::allocate_device_view(int device_id)
{
    return new ServerHMDView(device_id);
}

int 
HMDManager::getListUpdatedResponseType()
{
    return (int)PSMoveProtocol::Response_ResponseType_HMD_LIST_UPDATED;
}
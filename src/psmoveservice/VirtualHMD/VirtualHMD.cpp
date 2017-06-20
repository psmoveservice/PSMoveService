//-- includes -----
#include "VirtualHMD.h"
#include "DeviceInterface.h"
#include "DeviceManager.h"
#include "HMDDeviceEnumerator.h"
#include "VirtualHMDDeviceEnumerator.h"
#include "MathUtility.h"
#include "ServerLog.h"
#include "ServerUtility.h"
#include <vector>
#include <cstdlib>
#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif
#include <math.h>

// -- constants -----
#define VIRTUAL_HMD_STATE_BUFFER_MAX 4

// -- private methods

// -- public interface
// -- Morpheus HMD Config
const int VirtualHMDConfig::CONFIG_VERSION = 1;

const boost::property_tree::ptree
VirtualHMDConfig::config2ptree()
{
    boost::property_tree::ptree pt;

    pt.put("is_valid", is_valid);
    pt.put("version", VirtualHMDConfig::CONFIG_VERSION);

    pt.put("Calibration.Position.VarianceExpFitA", position_variance_exp_fit_a);
    pt.put("Calibration.Position.VarianceExpFitB", position_variance_exp_fit_b);

    pt.put("Calibration.Time.MeanUpdateTime", mean_update_time_delta);

    pt.put("PositionFilter.FilterType", position_filter_type);
    pt.put("PositionFilter.MaxVelocity", max_velocity);

    pt.put("prediction_time", prediction_time);
    pt.put("bulb_radius", bulb_radius);

    writeTrackingColor(pt, tracking_color_id);

    return pt;
}

void
VirtualHMDConfig::ptree2config(const boost::property_tree::ptree &pt)
{
    version = pt.get<int>("version", 0);

    if (version == VirtualHMDConfig::CONFIG_VERSION)
    {
        is_valid = pt.get<bool>("is_valid", false);

        prediction_time = pt.get<float>("prediction_time", 0.f);

        position_variance_exp_fit_a = pt.get<float>("Calibration.Position.VarianceExpFitA", position_variance_exp_fit_a);
        position_variance_exp_fit_b = pt.get<float>("Calibration.Position.VarianceExpFitB", position_variance_exp_fit_b);

        mean_update_time_delta = pt.get<float>("Calibration.Time.MeanUpdateTime", mean_update_time_delta);

        position_filter_type = pt.get<std::string>("PositionFilter.FilterType", position_filter_type);
        max_velocity = pt.get<float>("PositionFilter.MaxVelocity", max_velocity);

        // Read the tracking color
        tracking_color_id = static_cast<eCommonTrackingColorID>(readTrackingColor(pt));
        bulb_radius = pt.get<float>("bulb_radius", bulb_radius);
    }
    else
    {
        SERVER_LOG_WARNING("VirtualHMDConfig") <<
            "Config version " << version << " does not match expected version " <<
            VirtualHMDConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

// -- Virtual HMD -----
VirtualHMD::VirtualHMD()
    : cfg()
    , NextPollSequenceNumber(0)
    , bIsOpen(false)
    , HMDStates()
    , bIsTracking(false)
{
    HMDStates.clear();
}

VirtualHMD::~VirtualHMD()
{
    if (getIsOpen())
    {
        SERVER_LOG_ERROR("~VirtualHMD") << "HMD deleted without calling close() first!";
    }
}

bool VirtualHMD::open()
{
    HMDDeviceEnumerator enumerator(HMDDeviceEnumerator::CommunicationType_VIRTUAL);
    bool success = false;

    if (enumerator.is_valid())
    {
        success = open(&enumerator);
    }

    return success;
}

bool VirtualHMD::open(
    const DeviceEnumerator *enumerator)
{
    const HMDDeviceEnumerator *pEnum = static_cast<const HMDDeviceEnumerator *>(enumerator);

    const char *cur_dev_path = pEnum->get_path();
    bool success = false;

    if (getIsOpen())
    {
        SERVER_LOG_WARNING("VirtualHMD::open") << "VirtualHMD(" << cur_dev_path << ") already open. Ignoring request.";
        success = true;
    }
    else
    {
        SERVER_LOG_INFO("VirtualHMD::open") << "Opening VirtualHMD(" << cur_dev_path << ").";

        device_identifier = cur_dev_path;
        bIsOpen= true;

        // Load the config file
        cfg = VirtualHMDConfig(pEnum->get_path());
        cfg.load();

        // Save it back out again in case any defaults changed
        cfg.save();

        // Reset the polling sequence counter
        NextPollSequenceNumber = 0;

        success = true;
    }

    return success;
}

void VirtualHMD::close()
{
    if (bIsOpen)
    {
        device_identifier= "";
        bIsOpen= true;
    }
    else
    {
        SERVER_LOG_INFO("VirtualHMD::close") << "MorpheusHMD already closed. Ignoring request.";
    }
}

// Getters
bool
VirtualHMD::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    // Down-cast the enumerator so we can use the correct get_path.
    const HMDDeviceEnumerator *pEnum = static_cast<const HMDDeviceEnumerator *>(enumerator);

    bool matches = false;

    if (pEnum->get_device_type() == getDeviceType())
    {
        const char *enumerator_path = pEnum->get_path();
        const char *dev_path = device_identifier.c_str();

#ifdef _WIN32
        matches = _stricmp(dev_path, enumerator_path) == 0;
#else
        matches = strcmp(dev_path, enumerator_path) == 0;
#endif
    }

    return matches;
}

bool
VirtualHMD::getIsReadyToPoll() const
{
    return (getIsOpen());
}

std::string
VirtualHMD::getUSBDevicePath() const
{
    return device_identifier;
}

bool
VirtualHMD::getIsOpen() const
{
    return bIsOpen;
}

IControllerInterface::ePollResult
VirtualHMD::poll()
{
    IHMDInterface::ePollResult result = IHMDInterface::_PollResultFailure;

    if (getIsOpen())
    {
        VirtualHMDState newState;

        // New data available. Keep iterating.
        result = IHMDInterface::_PollResultSuccessNewData;

        // Increment the sequence for every new polling packet
        newState.PollSequenceNumber = NextPollSequenceNumber;
        ++NextPollSequenceNumber;

        // Make room for new entry if at the max queue size
        if (HMDStates.size() >= VIRTUAL_HMD_STATE_BUFFER_MAX)
        {
            HMDStates.erase(HMDStates.begin(), HMDStates.begin() + HMDStates.size() - VIRTUAL_HMD_STATE_BUFFER_MAX);
        }

        HMDStates.push_back(newState);
    }

    return result;
}

void
VirtualHMD::getTrackingShape(CommonDeviceTrackingShape &outTrackingShape) const
{
    outTrackingShape.shape_type = eCommonTrackingShapeType::Sphere;
    outTrackingShape.shape.sphere.radius_cm= cfg.bulb_radius;
}


bool 
VirtualHMD::setTrackingColorID(const eCommonTrackingColorID tracking_color_id)
{
    bool bSuccess = false;

    if (getIsOpen())
    {
        cfg.tracking_color_id = tracking_color_id;
        cfg.save();
        bSuccess = true;
    }

    return bSuccess;
}

bool 
VirtualHMD::getTrackingColorID(eCommonTrackingColorID &out_tracking_color_id) const
{
    out_tracking_color_id = cfg.tracking_color_id;
    return true;
}

float 
VirtualHMD::getPredictionTime() const
{
    return getConfig()->prediction_time;
}

const CommonDeviceState *
VirtualHMD::getState(
    int lookBack) const
{
    const int queueSize = static_cast<int>(HMDStates.size());
    const CommonDeviceState * result =
        (lookBack < queueSize) ? &HMDStates.at(queueSize - lookBack - 1) : nullptr;

    return result;
}

long VirtualHMD::getMaxPollFailureCount() const
{
    return 1;
}

void VirtualHMD::setTrackingEnabled(bool bEnable)
{
    if (!bIsTracking && bEnable)
    {
        bIsTracking = true;
    }
    else if (bIsTracking && !bEnable)
    {
        bIsTracking = false;
    }
}
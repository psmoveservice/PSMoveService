//-- includes -----
#include "VirtualController.h"
#include "ControllerDeviceEnumerator.h"
#include "ServerLog.h"
#include "ServerUtility.h"
#include <vector>

//-- constants -----
#define VIRTUAL_CONTROLLER_STATE_BUFFER_MAX 16

// -- public methods

// -- Virtual Controller Config
// Bump this version when you are making a breaking config change.
// Simply adding or removing a field is ok and doesn't require a version bump.
const int VirtualControllerConfig::CONFIG_VERSION= 1;

const boost::property_tree::ptree
VirtualControllerConfig::config2ptree()
{
    boost::property_tree::ptree pt;

    pt.put("is_valid", is_valid);
    pt.put("version", VirtualControllerConfig::CONFIG_VERSION);

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
VirtualControllerConfig::ptree2config(const boost::property_tree::ptree &pt)
{
    version = pt.get<int>("version", 0);

    if (version == VirtualControllerConfig::CONFIG_VERSION)
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
        SERVER_LOG_WARNING("VirtualControllerConfig") << 
            "Config version " << version << " does not match expected version " << 
            VirtualControllerConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

// -- PSMove Controller -----
VirtualController::VirtualController()
    : cfg()
    , NextPollSequenceNumber(0)
    , bIsOpen(false)
    , ControllerStates()
    , bIsTracking(false)
{
    ControllerStates.clear();
}

VirtualController::~VirtualController()
{
    if (getIsOpen())
    {
        SERVER_LOG_ERROR("~VirtualController") << "Controller deleted without calling close() first!";
    }
}

bool VirtualController::open()
{
    ControllerDeviceEnumerator enumerator(ControllerDeviceEnumerator::CommunicationType_VIRTUAL, CommonControllerState::VirtualController);
    bool success= false;

    if (enumerator.is_valid())
    {
        success= open(&enumerator);
    }

    return success;
}

bool VirtualController::open(
    const DeviceEnumerator *enumerator)
{
    const ControllerDeviceEnumerator *pEnum = static_cast<const ControllerDeviceEnumerator *>(enumerator);
    
    const char *cur_dev_path= pEnum->get_path();
    bool success= false;

    if (getIsOpen())
    {
        SERVER_LOG_WARNING("VirtualController::open") << "VirtualController(" << cur_dev_path << ") already open. Ignoring request.";
        success= true;
    }
    else
    {
        SERVER_LOG_INFO("VirtualController::open") << "Opening VirtualController(" << cur_dev_path << ").";

        device_identifier = cur_dev_path;
        bIsOpen= true;

        // Load the config file
        cfg = VirtualControllerConfig(pEnum->get_path());
        cfg.load();

        // Save it back out again in case any defaults changed
        cfg.save();

        // Reset the polling sequence counter
        NextPollSequenceNumber = 0;

        success = true;
    }

    return success;
}

void VirtualController::close()
{
    if (bIsOpen)
    {
        device_identifier= "";
        bIsOpen= true;
    }
    else
    {
        SERVER_LOG_INFO("VirtualController::close") << "VirtualController already closed. Ignoring request.";
    }
}

bool 
VirtualController::setHostBluetoothAddress(const std::string &new_host_bt_addr)
{
    SERVER_LOG_WARNING("VirtualController::setHostBluetoothAddress") << "VirtualController(" << device_identifier << ") Can't have host bluetooth address assigned.";

    return false;
}

bool
VirtualController::setTrackingColorID(const eCommonTrackingColorID tracking_color_id)
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

// Getters
bool 
VirtualController::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    // Down-cast the enumerator so we can use the correct get_path.
    const ControllerDeviceEnumerator *pEnum = static_cast<const ControllerDeviceEnumerator *>(enumerator);
    
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
VirtualController::getIsBluetooth() const
{ 
    return false; 
}

bool
VirtualController::getIsReadyToPoll() const
{
    return getIsOpen();
}

std::string 
VirtualController::getUSBDevicePath() const
{
    return device_identifier;
}

int
VirtualController::getVendorID() const
{
	return 0x00;
}

int
VirtualController::getProductID() const
{
	return 0x00;
}

std::string 
VirtualController::getSerial() const
{
    return device_identifier;
}

std::string 
VirtualController::getAssignedHostBluetoothAddress() const
{
    return "00:00:00:00:00:00";
}

bool
VirtualController::getIsOpen() const
{
    return bIsOpen;
}

CommonDeviceState::eDeviceType
VirtualController::getDeviceType() const
{
    return CommonDeviceState::VirtualController;
}

IControllerInterface::ePollResult
VirtualController::poll()
{
    IControllerInterface::ePollResult result= IControllerInterface::_PollResultFailure;
      
    if (getIsOpen())
    {
        VirtualControllerState newState;

        // Device still in valid state
        result= IControllerInterface::_PollResultSuccessNewData;
                
        // Increment the sequence for every new polling packet
        newState.PollSequenceNumber= NextPollSequenceNumber;
        ++NextPollSequenceNumber;

        // Make room for new entry if at the max queue size
        if (ControllerStates.size() >= VIRTUAL_CONTROLLER_STATE_BUFFER_MAX)
        {
            ControllerStates.erase(ControllerStates.begin(),
                ControllerStates.begin() + ControllerStates.size() - VIRTUAL_CONTROLLER_STATE_BUFFER_MAX);
        }

        ControllerStates.push_back(newState);
    }

    return result;
}

const CommonDeviceState * 
VirtualController::getState(
    int lookBack) const
{
    const int queueSize= static_cast<int>(ControllerStates.size());
    const CommonDeviceState * result=
        (lookBack < queueSize) ? &ControllerStates.at(queueSize - lookBack - 1) : nullptr;

    return result;
}

const std::tuple<unsigned char, unsigned char, unsigned char> 
VirtualController::getColour() const
{
    return std::make_tuple(0, 0, 0);
}

void 
VirtualController::getTrackingShape(CommonDeviceTrackingShape &outTrackingShape) const
{
    outTrackingShape.shape_type= eCommonTrackingShapeType::Sphere;
    outTrackingShape.shape.sphere.radius_cm = getConfig()->bulb_radius;
}

bool
VirtualController::getTrackingColorID(eCommonTrackingColorID &out_tracking_color_id) const
{
	bool bSuccess = false;

	if (getIsOpen())
	{
		out_tracking_color_id = cfg.tracking_color_id;
		bSuccess = true;
	}

	return bSuccess;
}

float VirtualController::getIdentityForwardDegrees() const
{
	// Controller model points down the -Z axis when it has the identity orientation
	return 270.f;
}

float VirtualController::getPredictionTime() const
{
	return getConfig()->prediction_time;
}

bool VirtualController::getWasSystemButtonPressed() const
{
    return false;
}

long VirtualController::getMaxPollFailureCount() const
{
    return 1;
}
// -- includes -----
#include "PS3EyeTracker.h"
#include "ServerLog.h"
#include "ServerUtility.h"
#include "PSEyeVideoCapture.h"
#include "PSMoveProtocol.pb.h"
#include "TrackerDeviceEnumerator.h"
#include "TrackerManager.h"
#include "opencv2/opencv.hpp"

// -- constants -----
#define PS3EYE_STATE_BUFFER_MAX 16

static const char *OPTION_FOV_SETTING = "FOV Setting";
static const char *OPTION_FOV_RED_DOT = "Red Dot";
static const char *OPTION_FOV_BLUE_DOT = "Blue Dot";

// -- private definitions -----
class PSEyeCaptureData
{
public:
    PSEyeCaptureData()
        : frame()
    {

    }

    cv::Mat frame;
};

// -- public methods
// -- PS3EYE Controller Config
const int PS3EyeTrackerConfig::CONFIG_VERSION = 7;
const int PS3EyeTrackerConfig::LENS_CALIBRATION_VERSION= 1;

PS3EyeTrackerConfig::PS3EyeTrackerConfig(const std::string &fnamebase)
    : PSMoveConfig(fnamebase)
    , is_valid(false)
    , max_poll_failure_count(100)
	, frame_width(640)
	, frame_height(480)
	, frame_rate(40)
    , exposure(32)
    , gain(32)
    , focalLengthX(554.2563) // pixels
    , focalLengthY(554.2563) // pixels
    , principalX(320.0) // pixels
    , principalY(240.0) // pixels
    , hfov(60.0) // degrees
    , vfov(45.0) // degrees
    , zNear(10.0) // cm
    , zFar(200.0) // cm
    , distortionK1(-0.10771770030260086)
    , distortionK2(0.1213262677192688)
    , distortionK3(0.04875476285815239)
    , distortionP1(0.00091733073350042105)
    , distortionP2(0.00010589254816295579)
    , fovSetting(BlueDot)
{
    pose.clear();

	SharedColorPresets.table_name.clear();
    for (int preset_index = 0; preset_index < eCommonTrackingColorID::MAX_TRACKING_COLOR_TYPES; ++preset_index)
    {
        SharedColorPresets.color_presets[preset_index] = k_default_color_presets[preset_index];
    }
};

const boost::property_tree::ptree
PS3EyeTrackerConfig::config2ptree()
{
    boost::property_tree::ptree pt;

    pt.put("is_valid", is_valid);
    pt.put("version", PS3EyeTrackerConfig::CONFIG_VERSION);
	pt.put("lens_calibration_version", PS3EyeTrackerConfig::LENS_CALIBRATION_VERSION);
    pt.put("max_poll_failure_count", max_poll_failure_count);
	pt.put("frame_width", frame_width);
	pt.put("frame_height", frame_height);
	pt.put("frame_rate", frame_rate);
    pt.put("exposure", exposure);
	pt.put("gain", gain);
    pt.put("focalLengthX", focalLengthX);
    pt.put("focalLengthY", focalLengthY);
    pt.put("principalX", principalX);
    pt.put("principalY", principalY);
    pt.put("hfov", hfov);
    pt.put("vfov", vfov);
    pt.put("zNear", zNear);
    pt.put("zFar", zFar);
    pt.put("distortionK1", distortionK1);
    pt.put("distortionK2", distortionK2);
    pt.put("distortionK3", distortionK3);
    pt.put("distortionP1", distortionP1);
    pt.put("distortionP2", distortionP2);
    pt.put("fovSetting", static_cast<int>(fovSetting));

    pt.put("pose.orientation.w", pose.Orientation.w);
    pt.put("pose.orientation.x", pose.Orientation.x);
    pt.put("pose.orientation.y", pose.Orientation.y);
    pt.put("pose.orientation.z", pose.Orientation.z);
    pt.put("pose.position.x", pose.PositionCm.x);
    pt.put("pose.position.y", pose.PositionCm.y);
    pt.put("pose.position.z", pose.PositionCm.z);

	writeColorPropertyPresetTable(&SharedColorPresets, pt);

	for (auto &controller_preset_table : DeviceColorPresets)
	{
		writeColorPropertyPresetTable(&controller_preset_table, pt);
	}

    return pt;
}

void
PS3EyeTrackerConfig::ptree2config(const boost::property_tree::ptree &pt)
{
    int config_version = pt.get<int>("version", 0);
    if (config_version == PS3EyeTrackerConfig::CONFIG_VERSION)
    {
        is_valid = pt.get<bool>("is_valid", false);
        max_poll_failure_count = pt.get<long>("max_poll_failure_count", 100);
		frame_width = pt.get<double>("frame_width", 640);
		frame_height = pt.get<double>("frame_height", 480);
		frame_rate = pt.get<double>("frame_rate", 40);
        exposure = pt.get<double>("exposure", 32);
		gain = pt.get<double>("gain", 32);
        hfov = pt.get<double>("hfov", 60.0);
        vfov = pt.get<double>("vfov", 45.0);
        zNear = pt.get<double>("zNear", 10.0);
        zFar = pt.get<double>("zFar", 200.0);

		int lens_calibration_version = pt.get<int>("lens_calibration_version", 0);
		if (lens_calibration_version == PS3EyeTrackerConfig::LENS_CALIBRATION_VERSION)
		{
			focalLengthX = pt.get<double>("focalLengthX", focalLengthX);
			focalLengthY = pt.get<double>("focalLengthY", focalLengthY);
			principalX = pt.get<double>("principalX", principalX);
			principalY = pt.get<double>("principalY", principalY);
			distortionK1 = pt.get<double>("distortionK1", distortionK1);
			distortionK2 = pt.get<double>("distortionK2", distortionK2);
			distortionK3 = pt.get<double>("distortionK3", distortionK3);
			distortionP1 = pt.get<double>("distortionP1", distortionP1);
			distortionP2 = pt.get<double>("distortionP2", distortionP2);
		}
		else
		{
			SERVER_LOG_WARNING("PS3EyeTrackerConfig") <<
				"Config version " << lens_calibration_version << " does not match expected version " <<
				PS3EyeTrackerConfig::LENS_CALIBRATION_VERSION << ", Using defaults.";
		}

        fovSetting = 
            static_cast<PS3EyeTrackerConfig::eFOVSetting>(
                pt.get<int>("fovSetting", PS3EyeTrackerConfig::eFOVSetting::BlueDot));

        pose.Orientation.w = pt.get<float>("pose.orientation.w", 1.0);
        pose.Orientation.x = pt.get<float>("pose.orientation.x", 0.0);
        pose.Orientation.y = pt.get<float>("pose.orientation.y", 0.0);
        pose.Orientation.z = pt.get<float>("pose.orientation.z", 0.0);
        pose.PositionCm.x = pt.get<float>("pose.position.x", 0.0);
        pose.PositionCm.y = pt.get<float>("pose.position.y", 0.0);
        pose.PositionCm.z = pt.get<float>("pose.position.z", 0.0);

		// Read the default preset table
		readColorPropertyPresetTable(pt, &SharedColorPresets);

		// Read all of the controller preset tables
		const std::string controller_prefix("controller_");
		const std::string hmd_prefix("hmd_");
		for(auto iter = pt.begin(); iter != pt.end(); iter++)
		{
			const std::string &entry_name= iter->first;
			
			if (entry_name.compare(0, controller_prefix.length(), controller_prefix) == 0 ||
				entry_name.compare(0, hmd_prefix.length(), hmd_prefix) == 0)
			{
				CommonHSVColorRangeTable table;

				table.table_name= entry_name;
				for (int preset_index = 0; preset_index < eCommonTrackingColorID::MAX_TRACKING_COLOR_TYPES; ++preset_index)
				{
					table.color_presets[preset_index] = k_default_color_presets[preset_index];
				}

				readColorPropertyPresetTable(pt, &table);

				DeviceColorPresets.push_back(table);
			}
		}
    }
    else
    {
        SERVER_LOG_WARNING("PS3EyeTrackerConfig") <<
            "Config version " << config_version << " does not match expected version " <<
            PS3EyeTrackerConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

const CommonHSVColorRangeTable *
PS3EyeTrackerConfig::getColorRangeTable(const std::string &table_name) const
{
	const CommonHSVColorRangeTable *table= &SharedColorPresets;	

	if (table_name.length() > 0)
	{
		for (auto &entry : DeviceColorPresets)
		{
			if (entry.table_name == table_name)
			{
				table= &entry;
			}
		}
	}

	return table;
}

inline CommonHSVColorRangeTable *
PS3EyeTrackerConfig::getOrAddColorRangeTable(const std::string &table_name)
{
	CommonHSVColorRangeTable *table= nullptr;	

	if (table_name.length() > 0)
	{
		for (auto &entry : DeviceColorPresets)
		{
			if (entry.table_name == table_name)
			{
				table= &entry;
			}
		}

		if (table == nullptr)
		{
			CommonHSVColorRangeTable Table;

			Table.table_name= table_name;
			for (int preset_index = 0; preset_index < eCommonTrackingColorID::MAX_TRACKING_COLOR_TYPES; ++preset_index)
			{
				Table.color_presets[preset_index] = k_default_color_presets[preset_index];
			}

			DeviceColorPresets.push_back(Table);
			table= &DeviceColorPresets[DeviceColorPresets.size() - 1];
		}
	}
	else
	{
		table= &SharedColorPresets;
	}

	return table;
}

// -- PS3EYE Tracker
PS3EyeTracker::PS3EyeTracker()
    : cfg()
    , USBDevicePath()
    , VideoCapture(nullptr)
    , CaptureData(nullptr)
    , DriverType(PS3EyeTracker::Libusb)
    , NextPollSequenceNumber(0)
    , TrackerStates()
{
}

PS3EyeTracker::~PS3EyeTracker()
{
    if (getIsOpen())
    {
        SERVER_LOG_ERROR("~PS3EyeTracker") << "Tracker deleted without calling close() first!";
    }
}

// PSMoveTracker
bool PS3EyeTracker::open() // Opens the first HID device for the tracker
{
    TrackerDeviceEnumerator enumerator;
    bool success = false;

    // Skip over everything that isn't a PS3EYE
    while (enumerator.is_valid() && enumerator.get_device_type() != CommonDeviceState::PS3EYE)
    {
        enumerator.next();
    }

    if (enumerator.is_valid())
    {
        success = open(&enumerator);
    }

    return success;
}

// -- IDeviceInterface
bool PS3EyeTracker::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    // Down-cast the enumerator so we can use the correct get_path.
    const TrackerDeviceEnumerator *pEnum = static_cast<const TrackerDeviceEnumerator *>(enumerator);

    bool matches = false;

    if (pEnum->get_device_type() == CommonControllerState::PS3EYE)
    {
        std::string enumerator_path = pEnum->get_path();

        matches = (enumerator_path == USBDevicePath);
    }

    return matches;
}

bool PS3EyeTracker::open(const DeviceEnumerator *enumerator)
{
    const TrackerDeviceEnumerator *tracker_enumerator = static_cast<const TrackerDeviceEnumerator *>(enumerator);
    const char *cur_dev_path = tracker_enumerator->get_path();

    bool bSuccess = false;
    
    if (getIsOpen())
    {
        SERVER_LOG_WARNING("PS3EyeTracker::open") << "PS3EyeTracker(" << cur_dev_path << ") already open. Ignoring request.";
        bSuccess = true;
    }
    else
    {
        const int camera_index = tracker_enumerator->get_camera_index();

        SERVER_LOG_INFO("PS3EyeTracker::open") << "Opening PS3EyeTracker(" << cur_dev_path << ", camera_index=" << camera_index << ")";

        VideoCapture = new PSEyeVideoCapture(camera_index);

        if (VideoCapture->isOpened())
        {
            CaptureData = new PSEyeCaptureData;
            USBDevicePath = enumerator->get_path();
            bSuccess = true;
        }
        else
        {
            SERVER_LOG_ERROR("PS3EyeTracker::open") << "Failed to open PS3EyeTracker(" << cur_dev_path << ", camera_index=" << camera_index << ")";

            close();
        }
    }
    
    if (bSuccess)
    {
        std::string identifier = VideoCapture->getUniqueIndentifier();
        std::string config_name = "PS3EyeTrackerConfig_";
        config_name.append(identifier);

        cfg = PS3EyeTrackerConfig(config_name);

		// Load the ps3eye config
        cfg.load();
		// Save the config back out again in case defaults changed
		cfg.save();

		VideoCapture->set(cv::CAP_PROP_FRAME_WIDTH, cfg.frame_width);
		VideoCapture->set(cv::CAP_PROP_EXPOSURE, cfg.exposure);
		VideoCapture->set(cv::CAP_PROP_GAIN, cfg.gain);
		VideoCapture->set(cv::CAP_PROP_FPS, cfg.frame_rate);
    }

    return bSuccess;
}

bool PS3EyeTracker::getIsOpen() const
{
    return VideoCapture != nullptr;
}

bool PS3EyeTracker::getIsReadyToPoll() const
{
    return getIsOpen();
}

IDeviceInterface::ePollResult PS3EyeTracker::poll()
{
    IDeviceInterface::ePollResult result = IDeviceInterface::_PollResultFailure;

    if (getIsOpen())
    {
        if (!VideoCapture->grab() || 
            !VideoCapture->retrieve(CaptureData->frame, cv::CAP_OPENNI_BGR_IMAGE))
        {
            // Device still in valid state
            result = IControllerInterface::_PollResultSuccessNoData;
        }
        else
        {
            // New data available. Keep iterating.
            result = IControllerInterface::_PollResultSuccessNewData;
        }

        {
            PS3EyeTrackerState newState;

            // TODO: Process the frame and extract the blobs

            // Increment the sequence for every new polling packet
            newState.PollSequenceNumber = NextPollSequenceNumber;
            ++NextPollSequenceNumber;

            // Make room for new entry if at the max queue size
            //###bwalker $TODO Make this a fixed size circular buffer
            if (TrackerStates.size() >= PS3EYE_STATE_BUFFER_MAX)
            {
                TrackerStates.erase(TrackerStates.begin(), TrackerStates.begin() + TrackerStates.size() - PS3EYE_STATE_BUFFER_MAX);
            }

            TrackerStates.push_back(newState);
        }
    }

    return result;
}

void PS3EyeTracker::close()
{
    if (CaptureData != nullptr)
    {
        delete CaptureData;
        CaptureData = nullptr;
    }

    if (VideoCapture != nullptr)
    {
        delete VideoCapture;
        VideoCapture = nullptr;
    }
}

long PS3EyeTracker::getMaxPollFailureCount() const
{
    return cfg.max_poll_failure_count;
}

CommonDeviceState::eDeviceType PS3EyeTracker::getDeviceType() const
{
    return CommonDeviceState::PS3EYE;
}

const CommonDeviceState *PS3EyeTracker::getState(int lookBack) const
{
    const int queueSize = static_cast<int>(TrackerStates.size());
    const CommonDeviceState * result =
        (lookBack < queueSize) ? &TrackerStates.at(queueSize - lookBack - 1) : nullptr;

    return result;
}

ITrackerInterface::eDriverType PS3EyeTracker::getDriverType() const
{
    //###bwalker $TODO Get the driver type from VideoCapture
    return DriverType;
}

std::string PS3EyeTracker::getUSBDevicePath() const
{
    return USBDevicePath;
}

bool PS3EyeTracker::getVideoFrameDimensions(
    int *out_width,
    int *out_height,
    int *out_stride) const
{
    bool bSuccess = true;

    if (out_width != nullptr)
    {
        int width = static_cast<int>(VideoCapture->get(cv::CAP_PROP_FRAME_WIDTH));

        if (out_stride != nullptr)
        {
            int format = static_cast<int>(VideoCapture->get(cv::CAP_PROP_FORMAT));
            int bytes_per_pixel;

            if (format != -1)
            {
                switch (format)
                {
                case cv::CAP_MODE_BGR:
                case cv::CAP_MODE_RGB:
                    bytes_per_pixel = 3;
                    break;
                case cv::CAP_MODE_YUYV:
                    bytes_per_pixel = 2;
                    break;
                case cv::CAP_MODE_GRAY:
                    bytes_per_pixel = 1;
                    break;
                default:
                    assert(false && "Unknown video format?");
                    break;
                }
            }
            else
            {
                // Assume RGB?
                SERVER_LOG_ERROR("PS3EyeTracker::getVideoFrameDimensions") << "Unknown video format for camera" << USBDevicePath << ")";
                bytes_per_pixel = 3;
            }

            *out_stride = bytes_per_pixel * width;
        }

        *out_width = width;
    }

    if (out_height != nullptr)
    {
        int height = static_cast<int>(VideoCapture->get(cv::CAP_PROP_FRAME_HEIGHT));

        *out_height = height;
    }

    return bSuccess;
}

const unsigned char *PS3EyeTracker::getVideoFrameBuffer() const
{
    const unsigned char *result = nullptr;

    if (CaptureData != nullptr)
    {
        return static_cast<const unsigned char *>(CaptureData->frame.data);
    }

    return result;
}

void PS3EyeTracker::loadSettings()
{
	const double currentFrameWidth = VideoCapture->get(cv::CAP_PROP_FRAME_WIDTH);
	const double currentFrameRate = VideoCapture->get(cv::CAP_PROP_FPS);
    const double currentExposure= VideoCapture->get(cv::CAP_PROP_EXPOSURE);
    const double currentGain= VideoCapture->get(cv::CAP_PROP_GAIN);

    cfg.load();

	if (currentFrameWidth != cfg.frame_width)
	{
		VideoCapture->set(cv::CAP_PROP_FRAME_WIDTH, cfg.frame_width);
	}

    if (currentExposure != cfg.exposure)
    {
        VideoCapture->set(cv::CAP_PROP_EXPOSURE, cfg.exposure);
    }

    if (currentGain != cfg.gain)
    {
        VideoCapture->set(cv::CAP_PROP_GAIN, cfg.gain);
    }

	if (currentFrameRate != cfg.frame_rate)
	{
		VideoCapture->set(cv::CAP_PROP_FPS, cfg.frame_rate);
	}
}

void PS3EyeTracker::saveSettings()
{
    cfg.save();
}

void PS3EyeTracker::setFrameWidth(double value, bool bUpdateConfig)
{
	VideoCapture->set(cv::CAP_PROP_FRAME_WIDTH, value);

	if (bUpdateConfig)
	{
		cfg.frame_width = value;
	}
}

double PS3EyeTracker::getFrameWidth() const
{
	return VideoCapture->get(cv::CAP_PROP_FRAME_WIDTH);
}

void PS3EyeTracker::setFrameHeight(double value, bool bUpdateConfig)
{
	VideoCapture->set(cv::CAP_PROP_FRAME_HEIGHT, value);

	if (bUpdateConfig)
	{
		cfg.frame_height = value;
	}
}

double PS3EyeTracker::getFrameHeight() const
{
	return VideoCapture->get(cv::CAP_PROP_FRAME_HEIGHT);
}

void PS3EyeTracker::setFrameRate(double value, bool bUpdateConfig)
{
	VideoCapture->set(cv::CAP_PROP_FPS, value);

	if (bUpdateConfig)
	{
		cfg.frame_rate = value;
	}
}

double PS3EyeTracker::getFrameRate() const
{
	return VideoCapture->get(cv::CAP_PROP_FPS);
}

void PS3EyeTracker::setExposure(double value, bool bUpdateConfig)
{
    VideoCapture->set(cv::CAP_PROP_EXPOSURE, value);

	if (bUpdateConfig)
	{
		cfg.exposure = value;
	}
}

double PS3EyeTracker::getExposure() const
{
    return VideoCapture->get(cv::CAP_PROP_EXPOSURE);
}

void PS3EyeTracker::setGain(double value, bool bUpdateConfig)
{
	VideoCapture->set(cv::CAP_PROP_GAIN, value);

	if (bUpdateConfig)
	{
		cfg.gain = value;
	}
}

double PS3EyeTracker::getGain() const
{
	return VideoCapture->get(cv::CAP_PROP_GAIN);
}

void PS3EyeTracker::getCameraIntrinsics(
    float &outFocalLengthX, float &outFocalLengthY,
    float &outPrincipalX, float &outPrincipalY,
    float &outDistortionK1, float &outDistortionK2, float &outDistortionK3,
    float &outDistortionP1, float &outDistortionP2) const
{
    outFocalLengthX = static_cast<float>(cfg.focalLengthX);
    outFocalLengthY = static_cast<float>(cfg.focalLengthY);
    outPrincipalX = static_cast<float>(cfg.principalX);
    outPrincipalY = static_cast<float>(cfg.principalY);
    outDistortionK1 = static_cast<float>(cfg.distortionK1);
    outDistortionK2 = static_cast<float>(cfg.distortionK2);
    outDistortionK3 = static_cast<float>(cfg.distortionK3);
    outDistortionP1 = static_cast<float>(cfg.distortionP1);
    outDistortionP2 = static_cast<float>(cfg.distortionP2);
}

void PS3EyeTracker::setCameraIntrinsics(
    float focalLengthX, float focalLengthY,
    float principalX, float principalY,
    float distortionK1, float distortionK2, float distortionK3,
    float distortionP1, float distortionP2)
{
    cfg.focalLengthX = focalLengthX;
    cfg.focalLengthY = focalLengthY;
    cfg.principalX = principalX;
    cfg.principalY = principalY;
    cfg.distortionK1 = distortionK1;
    cfg.distortionK2 = distortionK2;
    cfg.distortionK3 = distortionK3;
    cfg.distortionP1 = distortionP1;
    cfg.distortionP2 = distortionP2;
}

CommonDevicePose PS3EyeTracker::getTrackerPose() const
{
    return cfg.pose;
}

void PS3EyeTracker::setTrackerPose(
    const struct CommonDevicePose *pose)
{
    cfg.pose = *pose;
    cfg.save();
}

void PS3EyeTracker::getFOV(float &outHFOV, float &outVFOV) const
{
    outHFOV = static_cast<float>(cfg.hfov);
    outVFOV = static_cast<float>(cfg.vfov);
}

void PS3EyeTracker::getZRange(float &outZNear, float &outZFar) const
{
    outZNear = static_cast<float>(cfg.zNear);
    outZFar = static_cast<float>(cfg.zFar);
}

void PS3EyeTracker::gatherTrackerOptions(
    PSMoveProtocol::Response_ResultTrackerSettings* settings) const
{
    PSMoveProtocol::OptionSet *optionSet = settings->add_option_sets();
    
    optionSet->set_option_name(OPTION_FOV_SETTING);
    optionSet->add_option_strings(OPTION_FOV_RED_DOT);
    optionSet->add_option_strings(OPTION_FOV_BLUE_DOT);
    optionSet->set_option_index(static_cast<int>(cfg.fovSetting));
}

bool PS3EyeTracker::setOptionIndex(
    const std::string &option_name,
    int option_index)
{
    bool bValidOption = false;

    if (option_name == OPTION_FOV_SETTING && 
        option_index >= 0 && 
        option_index < PS3EyeTrackerConfig::eFOVSetting::MAX_FOV_SETTINGS)
    {
        cfg.fovSetting = static_cast<PS3EyeTrackerConfig::eFOVSetting>(option_index);
        //###HipsterSloth $TODO Update the focal lengths?

        bValidOption = true;
    }

    return bValidOption;
}

bool PS3EyeTracker::getOptionIndex(
    const std::string &option_name, 
    int &out_option_index) const
{
    bool bValidOption = false;

    if (option_name == OPTION_FOV_SETTING)
    {
        out_option_index = static_cast<int>(cfg.fovSetting);
        bValidOption = true;
    }

    return bValidOption;
}

void PS3EyeTracker::gatherTrackingColorPresets(
	const std::string &controller_serial, 
    PSMoveProtocol::Response_ResultTrackerSettings* settings) const
{
	const CommonHSVColorRangeTable *table= cfg.getColorRangeTable(controller_serial);

    for (int list_index = 0; list_index < MAX_TRACKING_COLOR_TYPES; ++list_index)
    {
        const CommonHSVColorRange &hsvRange = table->color_presets[list_index];
        const eCommonTrackingColorID colorType = static_cast<eCommonTrackingColorID>(list_index);

        PSMoveProtocol::TrackingColorPreset *colorPreset= settings->add_color_presets();
        colorPreset->set_color_type(static_cast<PSMoveProtocol::TrackingColorType>(colorType));
        colorPreset->set_hue_center(hsvRange.hue_range.center);
        colorPreset->set_hue_range(hsvRange.hue_range.range);
        colorPreset->set_saturation_center(hsvRange.saturation_range.center);
        colorPreset->set_saturation_range(hsvRange.saturation_range.range);
        colorPreset->set_value_center(hsvRange.value_range.center);
        colorPreset->set_value_range(hsvRange.value_range.range);
    }
}

void PS3EyeTracker::setTrackingColorPreset(
	const std::string &controller_serial, 
    eCommonTrackingColorID color, 
    const CommonHSVColorRange *preset)
{
//    cfg.ColorPresets[color] = *preset; // from generic_camera conflict
	CommonHSVColorRangeTable *table= cfg.getOrAddColorRangeTable(controller_serial);

    table->color_presets[color] = *preset;
    cfg.save();
}

void PS3EyeTracker::getTrackingColorPreset(
	const std::string &controller_serial, 
    eCommonTrackingColorID color, 
    CommonHSVColorRange *out_preset) const
{
	const CommonHSVColorRangeTable *table= cfg.getColorRangeTable(controller_serial);

    *out_preset = table->color_presets[color];
}

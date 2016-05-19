// -- includes -----
#include "PS3EyeTracker.h"
#include "ServerLog.h"
#include "ServerUtility.h"
#include "PSEyeVideoCapture.h"
#include "TrackerDeviceEnumerator.h"
#include "opencv2/opencv.hpp"

// -- constants -----
#define PS3EYE_STATE_BUFFER_MAX 16

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
const int PS3EyeTrackerConfig::CONFIG_VERSION = 2;

const boost::property_tree::ptree
PS3EyeTrackerConfig::config2ptree()
{
    boost::property_tree::ptree pt;

    pt.put("is_valid", is_valid);
    pt.put("version", PS3EyeTrackerConfig::CONFIG_VERSION);
    pt.put("max_poll_failure_count", max_poll_failure_count);
    pt.put("exposure", exposure);
    pt.put("focalLengthX", focalLengthX);
    pt.put("focalLengthY", focalLengthY);
    pt.put("principalX", principalX);
    pt.put("principalY", principalY);

    return pt;
}

void
PS3EyeTrackerConfig::ptree2config(const boost::property_tree::ptree &pt)
{
    version = pt.get<int>("version", 0);

    if (version == PS3EyeTrackerConfig::CONFIG_VERSION)
    {
        is_valid = pt.get<bool>("is_valid", false);
        max_poll_failure_count = pt.get<long>("max_poll_failure_count", 100);
        exposure = pt.get<double>("exposure", 32);
        focalLengthX = pt.get<double>("focalLengthX", 640.f);
        focalLengthY = pt.get<double>("focalLengthY", 640.f);
        principalX = pt.get<double>("principalX", 320.f);
        principalY = pt.get<double>("principalY", 240.f);
    }
    else
    {
        SERVER_LOG_WARNING("PS3EyeTrackerConfig") <<
            "Config version " << version << " does not match expected version " <<
            PS3EyeTrackerConfig::CONFIG_VERSION << ", Using defaults.";
    }
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
    TrackerDeviceEnumerator enumerator(CommonControllerState::PS3EYE);
    bool success = false;

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
        cfg.load();
        setExposure(cfg.exposure);
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

void PS3EyeTracker::setExposure(double value)
{
    VideoCapture->set(cv::CAP_PROP_EXPOSURE, value);
    cfg.exposure = value;
    cfg.save();
}

double PS3EyeTracker::getExposure() const
{
    return VideoCapture->get(cv::CAP_PROP_EXPOSURE);
}

void PS3EyeTracker::getCameraIntrinsics(
    float &outFocalLengthX, float &outFocalLengthY,
    float &outPrincipalX, float &outPrincipalY) const
{
    outFocalLengthX = cfg.focalLengthX;
    outFocalLengthY = cfg.focalLengthY;
    outPrincipalX = cfg.principalX;
    outPrincipalY = cfg.principalY;
}

void PS3EyeTracker::setCameraIntrinsics(
    float focalLengthX, float focalLengthY,
    float principalX, float principalY)
{
    cfg.focalLengthX = focalLengthX;
    cfg.focalLengthY = focalLengthY;
    cfg.principalX = principalX;
    cfg.principalY = principalY;
    cfg.save();
}
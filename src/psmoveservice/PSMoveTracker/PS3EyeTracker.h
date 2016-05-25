#ifndef PS3EYE_TRACKER_H
#define PS3EYE_TRACKER_H

// -- includes -----
#include "PSMoveDataFrame.h"
#include "PSMoveConfig.h"
#include "DeviceEnumerator.h"
#include "DeviceInterface.h"
#include <string>
#include <vector>
#include <deque>

// -- pre-declarations -----
namespace PSMoveProtocol
{
    class Response_ResultTrackerSettings;
};

// -- definitions -----
class PS3EyeTrackerConfig : public PSMoveConfig
{
public:
    enum eFOVSetting
    {
        RedDot, // 56 degree FOV
        BlueDot, // 75 degree FOV
        
        MAX_FOV_SETTINGS
    };

    PS3EyeTrackerConfig(const std::string &fnamebase = "PS3EyeTrackerConfig")
    : PSMoveConfig(fnamebase)
    , is_valid(false)
    , max_poll_failure_count(100)
    , exposure(32)
	, gain(32)
    , focalLengthX(640.0) // pixels
    , focalLengthY(640.0) // pixels
    , principalX(320.0) // pixels
    , principalY(240.0) // pixels
    , hfov(60.0) // degrees
    , vfov(45.0) // degrees
    , zNear(10.0) // cm
    , zFar(200.0) // cm
    , fovSetting(BlueDot)
    {
        pose.clear();
        hmdRelativePose.clear();
    };
    
    virtual const boost::property_tree::ptree config2ptree();
    virtual void ptree2config(const boost::property_tree::ptree &pt);
    
    bool is_valid;
    long version;
    long max_poll_failure_count;
    double exposure;
	double gain;
    double focalLengthX;
    double focalLengthY;
    double principalX;
    double principalY;
    double hfov;
    double vfov;
    double zNear;
    double zFar;
    eFOVSetting fovSetting;
    CommonDevicePose pose;
    CommonDevicePose hmdRelativePose;

    static const int CONFIG_VERSION;
};

struct PS3EyeTrackerState : public CommonDeviceState
{   
    PS3EyeTrackerState()
    {
        clear();
    }
    
    void clear()
    {
        CommonDeviceState::clear();
        DeviceType = CommonDeviceState::PS3EYE;
    }
};

class PS3EyeTracker : public ITrackerInterface {
public:
    PS3EyeTracker();
    ~PS3EyeTracker();
        
    // PSMoveTracker
    bool open(); // Opens the first HID device for the controller
    
    // -- IDeviceInterface
    bool matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const override;
    bool open(const DeviceEnumerator *enumerator) override;
    bool getIsOpen() const override;
    bool getIsReadyToPoll() const override;
    IDeviceInterface::ePollResult poll() override;
    void close() override;
    long getMaxPollFailureCount() const override;
    static CommonDeviceState::eDeviceType getDeviceTypeStatic()
    { return CommonDeviceState::PS3EYE; }
    CommonDeviceState::eDeviceType getDeviceType() const override;
    const CommonDeviceState *getState(int lookBack = 0) const override;
    
    // -- ITrackerInterface
    ITrackerInterface::eDriverType getDriverType() const override;
    std::string getUSBDevicePath() const override;
    bool getVideoFrameDimensions(int *out_width, int *out_height, int *out_stride) const override;
    const unsigned char *getVideoFrameBuffer() const override;
    void setExposure(double value) override;
    double getExposure() const override;
	void setGain(double value) override;
	double getGain() const override;
    void getCameraIntrinsics(
        float &outFocalLengthX, float &outFocalLengthY,
        float &outPrincipalX, float &outPrincipalY) const override;
    void setCameraIntrinsics(
        float focalLengthX, float focalLengthY,
        float principalX, float principalY) override;
    void getTrackerPose(
        struct CommonDevicePose *outPose, 
        struct CommonDevicePose *outHmdRelativePose) const override;
    void setTrackerPose(
        const struct CommonDevicePose *pose, 
        const struct CommonDevicePose *hmdRelativePose) override;
    void getFOV(float &outHFOV, float &outVFOV) const override;
    void getZRange(float &outZNear, float &outZFar) const override;
    void gatherTrackerOptions(PSMoveProtocol::Response_ResultTrackerSettings* settings) const;
    bool setOptionIndex(const std::string &option_name, int option_index);
    bool getOptionIndex(const std::string &option_name, int &out_option_index) const;

    // -- Getters
    inline const PS3EyeTrackerConfig &getConfig() const
    { return cfg; }

private:
    PS3EyeTrackerConfig cfg;
    std::string USBDevicePath;
    class PSEyeVideoCapture *VideoCapture;
    class PSEyeCaptureData *CaptureData;
    ITrackerInterface::eDriverType DriverType;
    
    // Read Controller State
    int NextPollSequenceNumber;
    std::deque<PS3EyeTrackerState> TrackerStates;
};
#endif // PS3EYE_TRACKER_H

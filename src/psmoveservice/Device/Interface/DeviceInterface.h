#ifndef DEVICE_INTERFACE_H
#define DEVICE_INTERFACE_H

// -- includes -----
#include <string>
#include <tuple>

// -- pre-declarations ----
namespace PSMoveProtocol
{
    class Response_ResultTrackerSettings;
    class TrackingColorPreset;
};

// -- constants -----
enum eCommonTrackingColorID {
    INVALID_COLOR= -1,
    
    Magenta,
    Cyan,
    Yellow,
    Red,
    Green,
    Blue,

    MAX_TRACKING_COLOR_TYPES
};

enum eCommonTrackingShapeType {
    INVALID_SHAPE = -1,

    Sphere,
    LightBar,
	PointCloud,

    MAX_TRACKING_SHAPE_TYPES
};

enum eCommonTrackingProjectionType {
    INVALID_PROJECTION = -1,

    ProjectionType_Ellipse,
    ProjectionType_LightBar,
	ProjectionType_Points,

    MAX_TRACKING_PROJECTION_TYPES
};


// -- definitions -----
struct CommonDeviceRange
{
    float center, range;

    inline void clear()
    {
        center = range = 0.f;
    }
};

struct CommonHSVColorRange
{
    CommonDeviceRange hue_range;
    CommonDeviceRange saturation_range;
    CommonDeviceRange value_range;

    inline void clear()
    {
        hue_range.clear();
        saturation_range.clear();
        value_range.clear();
    }
};

struct CommonHSVColorRangeTable
{
	std::string table_name;
	CommonHSVColorRange color_presets[eCommonTrackingColorID::MAX_TRACKING_COLOR_TYPES];	
};

struct CommonDeviceVector
{
    float i, j, k;

	inline static CommonDeviceVector create(float _i, float _j, float _k)
	{
		CommonDeviceVector result;

		result.set(_i, _j, _k);
		return result;
	}

	inline void set(float _i, float _j, float _k)
	{
		i = _i;
		j = _j;
		k = _k;
	}

    inline void clear()
    {
        i = j = k = 0.f;
    }
};

struct CommonRawDeviceVector
{
	int i, j, k;

	inline void clear()
	{
		i = j = k = 0;
	}
};

struct CommonDevicePosition
{
    float x, y, z;

    inline void clear()
    {
        x = y = z = 0.f;
    }

    inline void set(float _x, float _y, float _z)
    {
        x= _x;
        y= _y;
        z= _z;
    }
};

/// A screen location in the space upper left:[0, 0] -> lower right[frameWidth-1, frameHeight-1]  
struct CommonDeviceScreenLocation
{
    float x, y;

    inline void clear()
    {
        x = y = 0.f;
    }

    inline void set(float _x, float _y)
    {
        x = _x;
        y = _y;
    }
};

struct CommonDeviceQuaternion
{
    float x, y, z, w;

    inline void clear()
    {
        x = y = z = 0.f;
        w = 1.f;
    }
};

struct CommonDevicePose
{
    CommonDevicePosition PositionCm;
    CommonDeviceQuaternion Orientation;

    void clear()
    {
        PositionCm.clear();
        Orientation.clear();
    }
};

struct CommonDevicePhysics
{
    CommonDeviceVector VelocityCmPerSec;
    CommonDeviceVector AccelerationCmPerSecSqr;
    CommonDeviceVector AngularVelocityRadPerSec;
    CommonDeviceVector AngularAccelerationRadPerSecSqr;

    void clear()
    {
        VelocityCmPerSec.clear();
        AccelerationCmPerSecSqr.clear();
        AngularVelocityRadPerSec.clear();
        AngularAccelerationRadPerSecSqr.clear();
    }
};

struct CommonDeviceState
{
    enum eDeviceClass
    {
        Controller = 0x00,
        TrackingCamera = 0x10,
        HeadMountedDisplay = 0x20
    };
    
    enum eDeviceType
    {
        PSMove = Controller + 0x00,
        PSNavi = Controller + 0x01,
        PSDualShock4 = Controller + 0x02,
        SUPPORTED_CONTROLLER_TYPE_COUNT = Controller + 0x03,
        
        PS3EYE = TrackingCamera + 0x00,
        SUPPORTED_CAMERA_TYPE_COUNT = TrackingCamera + 0x01,
        
        Morpheus = HeadMountedDisplay + 0x00,
        SUPPORTED_HMD_TYPE_COUNT = HeadMountedDisplay + 0x01        
    };
    
    eDeviceType DeviceType;
    int PollSequenceNumber;
    
    inline CommonDeviceState()
    {
        clear();
    }
    
    inline void clear()
    {
        DeviceType= SUPPORTED_CONTROLLER_TYPE_COUNT; // invalid
        PollSequenceNumber= 0;
    }

    static const char *getDeviceTypeString(eDeviceType device_type)
    {
        const char *result = nullptr;

        switch (device_type)
        {
        case PSMove:
            result= "PSMove";
            break;
        case PSNavi:
            result = "PSNavi";
            break;
        case PSDualShock4:
            result = "PSDualShock4";
            break;
        case PS3EYE:
            result = "PSEYE";
            break;
        case Morpheus:
            result = "Morpheus";
            break;        
        default:
            result = "UNKNOWN";
        }

        return result;
    }
};

struct CommonControllerState : CommonDeviceState
{
    enum ButtonState {
        Button_UP = 0x00,       // (00b) Not pressed
        Button_PRESSED = 0x01,  // (01b) Down for one frame only
        Button_DOWN = 0x03,     // (11b) Down for >1 frame
        Button_RELEASED = 0x02, // (10b) Up for one frame only
    };

    enum BatteryLevel {
        Batt_MIN = 0x00, /*!< Battery is almost empty (< 20%) */
        Batt_20Percent = 0x01, /*!< Battery has at least 20% remaining */
        Batt_40Percent = 0x02, /*!< Battery has at least 40% remaining */
        Batt_60Percent = 0x03, /*!< Battery has at least 60% remaining */
        Batt_80Percent = 0x04, /*!< Battery has at least 80% remaining */
        Batt_MAX = 0x05, /*!< Battery is fully charged (not on charger) */
        Batt_CHARGING = 0xEE, /*!< Battery is currently being charged */
        Batt_CHARGING_DONE = 0xEF, /*!< Battery is fully charged (on charger) */
    };

    enum RumbleChannel
    {
        ChannelAll,
        ChannelLeft,
        ChannelRight
    };

    enum BatteryLevel Battery;
    unsigned int AllButtons;                    // all-buttons, used to detect changes

    //TODO: high-precision timestamp. Need to do in hidapi?
    
    inline CommonControllerState()
    {
        clear();
    }

    inline void clear()
    {
        CommonDeviceState::clear();
        DeviceType= SUPPORTED_CONTROLLER_TYPE_COUNT; // invalid
        Battery= Batt_MAX;
        AllButtons= 0;
    }
};

struct CommonHMDState : CommonDeviceState
{
    CommonDevicePose Pose;

    inline CommonHMDState()
    {
        clear();
    }

    inline void clear()
    {
        CommonDeviceState::clear();

        Pose.clear();
    }
};

struct CommonDeviceTrackingShape
{
	enum eTrackingShapeConstants
	{
		TRIANGLE_POINT_COUNT = 3,
		QUAD_POINT_COUNT = 4,
		MAX_POINT_CLOUD_POINT_COUNT= 9
	};
	enum TriVertexEnum
	{
		TriVertexLowerRight= 0,
		TriVertexLowerLeft= 1,
		TriVertexUpperMiddle= 2,
	
		TriVertexCount
	};
	enum QuadVertexEnum
	{
		QuadVertexUpperRight= 0,
		QuadVertexUpperLeft= 1,
		QuadVertexLowerLeft= 2,
		QuadVertexLowerRight= 3,

		QuadVertexCount
	};

    union{
        struct {
            float radius_cm;
        } sphere;

        struct {
            CommonDevicePosition triangle[TRIANGLE_POINT_COUNT];
            CommonDevicePosition quad[QUAD_POINT_COUNT];
        } light_bar;

		struct {
			CommonDevicePosition point[MAX_POINT_CLOUD_POINT_COUNT];
			int point_count;
		} point_cloud;
    } shape;

    eCommonTrackingShapeType shape_type;
};

struct CommonDeviceTrackingProjection
{
	enum eTrackingShapeConstants
	{
		TRIANGLE_POINT_COUNT = 3,
		QUAD_POINT_COUNT = 4,
		MAX_POINT_CLOUD_POINT_COUNT = 6 // at most 6 points visible to a given camera
	};

    union{
        struct {
            CommonDeviceScreenLocation center;
            float half_x_extent;
            float half_y_extent;
            float angle;
        } ellipse;

        struct {
            CommonDeviceScreenLocation triangle[TRIANGLE_POINT_COUNT];
            CommonDeviceScreenLocation quad[QUAD_POINT_COUNT];
        } lightbar;

		struct {
			CommonDeviceScreenLocation point[MAX_POINT_CLOUD_POINT_COUNT];
			int point_count;
		} points;
    } shape;

    float screen_area; // area in pixels^2
    eCommonTrackingProjectionType shape_type;
    
    struct {
        CommonDeviceScreenLocation center_of_mass;
        CommonDeviceScreenLocation bounding_rect[4];
        float area;
    } basic;
};

/// Abstract base class for any device interface. Further defined in specific device abstractions.
class IDeviceInterface
{
public:
    enum ePollResult
    {
        _PollResultSuccessNoData,
        _PollResultSuccessNewData,
        _PollResultFailure,
    };
    
	virtual ~IDeviceInterface() {};

    // Return true if device path matches
    virtual bool matchesDeviceEnumerator(const class DeviceEnumerator *enumerator) const = 0;
    
    // Opens the HID device for the device at the given enumerator
    virtual bool open(const class DeviceEnumerator *enumerator) = 0;
    
    // Returns true if hidapi opened successfully
    virtual bool getIsOpen() const  = 0;
    
    virtual bool getIsReadyToPoll() const = 0;
    
    // Polls for new device data
    virtual ePollResult poll() = 0;
    
    // Closes the HID device for the device
    virtual void close() = 0;
    
    // Get the number of milliseconds we're willing to accept no data from the device before we disconnect it
    virtual long getMaxPollFailureCount() const = 0;
    
    // Returns what type of device
    virtual CommonDeviceState::eDeviceType getDeviceType() const = 0;
    
    // Fetch the device state at the given sample index.
    // A lookBack of 0 corresponds to the most recent data.
    virtual const CommonDeviceState * getState(int lookBack = 0) const = 0;   
};

/// Abstract class for controller interface. Implemented in PSMoveController.cpp
class IControllerInterface : public IDeviceInterface
{
public:
    // Sets the address of the bluetooth adapter on the host PC with the controller
    virtual bool setHostBluetoothAddress(const std::string &address) = 0;

	// Sets the tracking color enum of the controller
	virtual bool setTrackingColorID(const eCommonTrackingColorID tracking_color_id) = 0;

    // -- Getters
    // Returns true if the device is connected via Bluetooth, false if by USB
    virtual bool getIsBluetooth() const = 0;

    // Returns the full usb device path for the controller
    virtual std::string getUSBDevicePath() const = 0;

	// Returns the vendor ID of the controller
	virtual int getVendorID() const = 0;

	// Returns the product ID of the controller
	virtual int getProductID() const = 0;

    // Gets the bluetooth address of the adapter on the host PC that's registered with the controller
    virtual std::string getAssignedHostBluetoothAddress() const = 0;

    // Returns the serial number for the controller
    virtual std::string getSerial() const  = 0;

    // Get the tracking color of the controller
    virtual const std::tuple<unsigned char, unsigned char, unsigned char> getColour() const = 0;

    // Get the tracking shape use by the controller
    virtual void getTrackingShape(CommonDeviceTrackingShape &outTrackingShape) const = 0;

	// Get the tracking color enum of the controller
	virtual bool getTrackingColorID(eCommonTrackingColorID &out_tracking_color_id) const = 0;

	// Get the identity forward direction yaw direction relative to the global +X axis
	// * 0 degrees would mean that the controller model was pointing down the globals +X axis 
	//   when the controller had the identity pose 
	// * 90 degrees would mean that the controller model was pointing down the globals +Z axis 
	//   when the controller had the identity pose
	// ...
	virtual float getIdentityForwardDegrees() const = 0;

	// Get the state prediction time specified in the controller config
	virtual float getPredictionTime() const = 0;
};

/// Abstract class for Tracker interface. Implemented Tracker classes
class ITrackerInterface : public IDeviceInterface
{
public:
    enum eDriverType
    {
        Libusb,
        CL,
        CLMulti,
        Generic_Webcam,

        SUPPORTED_DRIVER_TYPE_COUNT,
    };

    // -- Getters
    // Returns the driver type being used by this camera
    virtual eDriverType getDriverType() const = 0;

    // Returns the full usb device path for the tracker
    virtual std::string getUSBDevicePath() const = 0;

    // Returns the video frame size (used to compute frame buffer size)
    virtual bool getVideoFrameDimensions(int *out_width, int *out_height, int *out_stride) const = 0;

    // Returns a pointer to the last video frame buffer captured
    virtual const unsigned char *getVideoFrameBuffer() const = 0;

    static const char *getDriverTypeString(eDriverType device_type)
    {
        const char *result = nullptr;

        switch (device_type)
        {
        case Libusb:
            result = "Libusb";
            break;
        case CL:
            result = "CL";
            break;
        case CLMulti:
            result = "CLMulti";
            break;
        case Generic_Webcam:
            result = "Generic_Webcam";
            break;
        default:
            result = "UNKNOWN";
        }

        return result;
    }
    
    virtual void loadSettings() = 0;
    virtual void saveSettings() = 0;

    virtual void setExposure(double value, bool bUpdateConfig) = 0;
    virtual double getExposure() const = 0;

	virtual void setGain(double value, bool bUpdateConfig) = 0;
	virtual double getGain() const = 0;

    virtual void getCameraIntrinsics(
        float &outFocalLengthX, float &outFocalLengthY,
        float &outPrincipalX, float &outPrincipalY,
        float &outDistortionK1, float &outDistortionK2, float &outDistortionK3,
        float &outDistortionP1, float &outDistortionP2) const = 0;
    virtual void setCameraIntrinsics(
        float focalLengthX, float focalLengthY,
        float principalX, float principalY,
        float distortionK1, float distortionK2, float distortionK3,
        float distortionP1, float distortionP2) = 0;

    virtual CommonDevicePose getTrackerPose() const = 0;
    virtual void setTrackerPose(const struct CommonDevicePose *pose) = 0;

    virtual void getFOV(float &outHFOV, float &outVFOV) const = 0;
    virtual void getZRange(float &outZNear, float &outZFar) const = 0;

    virtual void gatherTrackerOptions(PSMoveProtocol::Response_ResultTrackerSettings* settings) const = 0;
    virtual bool setOptionIndex(const std::string &option_name, int option_index) = 0;
    virtual bool getOptionIndex(const std::string &option_name, int &out_option_index) const = 0;

    virtual void gatherTrackingColorPresets(const std::string &controller_serial, PSMoveProtocol::Response_ResultTrackerSettings* settings) const = 0;
    virtual void setTrackingColorPreset(const std::string &controller_serial, eCommonTrackingColorID color, const CommonHSVColorRange *preset) = 0;
    virtual void getTrackingColorPreset(const std::string &controller_serial, eCommonTrackingColorID color, CommonHSVColorRange *out_preset) const = 0;
};

/// Abstract class for HMD interface. Implemented HMD classes
class IHMDInterface : public IDeviceInterface
{
public:
    // -- Getters
    // Returns the full usb device path for the HMD
    virtual std::string getUSBDevicePath() const = 0;

	// Get the tracking shape use by the controller
	virtual void getTrackingShape(CommonDeviceTrackingShape &outTrackingShape) const = 0;

	// Get the tracking color enum of the controller
	virtual bool getTrackingColorID(eCommonTrackingColorID &out_tracking_color_id) const = 0;

	// Get the state prediction time from the HMD config
	virtual float getPredictionTime() const = 0;
};

#endif // DEVICE_INTERFACE_H

//-- includes -----
#include "DeviceEnumerator.h"
#include "DeviceManager.h"
#include "ServerTrackerView.h"
#include "ServerControllerView.h"
#include "ServerHMDView.h"
#include "MathUtility.h"
#include "MathEigen.h"
#include "MathGLM.h"
#include "MathAlignment.h"
#include "PS3EyeTracker.h"
#include "PSMoveProtocol.pb.h"
#include "ServerUtility.h"
#include "ServerLog.h"
#include "ServerRequestHandler.h"
#include "SharedTrackerState.h"
#include "TrackerManager.h"

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <memory>

#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#define USE_OPEN_CV_ELLIPSE_FIT

//-- typedefs ----
typedef std::vector<cv::Point> t_opencv_contour;
typedef std::vector<t_opencv_contour> t_opencv_contour_list;

//-- private methods -----
class SharedVideoFrameReadWriteAccessor
{
public:
    SharedVideoFrameReadWriteAccessor()
        : m_shared_memory_object(nullptr)
        , m_region(nullptr)
    {}

    ~SharedVideoFrameReadWriteAccessor()
    {
        dispose();
    }

    bool initialize(const char *shared_memory_name, int width, int height, int stride)
    {
        bool bSuccess = false;

        try
        {
            SERVER_LOG_INFO("SharedMemory::initialize()") << "Allocating shared memory: " << shared_memory_name;

            // Remember the name of the shared memory
            m_shared_memory_name = shared_memory_name;

            // Make sure the shared memory block has been removed first
            boost::interprocess::shared_memory_object::remove(shared_memory_name);

            // Allow non admin-level processed to access the shared memory
            boost::interprocess::permissions permissions;
            permissions.set_unrestricted();

            // Create the shared memory object
            m_shared_memory_object =
                new boost::interprocess::shared_memory_object(
                    boost::interprocess::create_only,
                    shared_memory_name,
                    boost::interprocess::read_write,
                    permissions);

            // Resize the shared memory
            m_shared_memory_object->truncate(SharedVideoFrameHeader::computeTotalSize(stride, width));

            // Map all of the shared memory for read/write access
            m_region = new boost::interprocess::mapped_region(*m_shared_memory_object, boost::interprocess::read_write);

            // Initialize the shared memory (call constructor using placement new)
            // This make sure the mutex has the constructor called on it.
            SharedVideoFrameHeader *frameState = new (getFrameHeader()) SharedVideoFrameHeader();
            
            frameState->width = width;
            frameState->height = height;
            frameState->stride = stride;
            frameState->frame_index = 0;
            std::memset(
                frameState->getBufferMutable(),
                0,
                SharedVideoFrameHeader::computeVideoBufferSize(stride, height));

            bSuccess = true;
        }
        catch (boost::interprocess::interprocess_exception* e)
        {
            dispose();
            SERVER_LOG_ERROR("SharedMemory::initialize()") << "Failed to allocated shared memory: " << m_shared_memory_name
                << ", reason: " << e->what();
        }

        return bSuccess;
    }

    void dispose()
    {
        if (m_region != nullptr)
        {
            // Call the destructor manually on the frame header since it was constructed via placement new
            // This will make sure the mutex has the destructor called on it.
            getFrameHeader()->~SharedVideoFrameHeader();
            
            delete m_region;
            m_region = nullptr;
        }

        if (m_shared_memory_object != nullptr)
        {
            delete m_shared_memory_object;
            m_shared_memory_object = nullptr;
        }

        if (!boost::interprocess::shared_memory_object::remove(m_shared_memory_name))
        {
            SERVER_LOG_ERROR("SharedMemory::dispose") << "Failed to free shared memory: " << m_shared_memory_name;
        }
    }

    void writeVideoFrame(const unsigned char *buffer)
    {
        SharedVideoFrameHeader *sharedFrameState = getFrameHeader();
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(sharedFrameState->mutex);

        size_t buffer_size = 
            SharedVideoFrameHeader::computeVideoBufferSize(sharedFrameState->stride, sharedFrameState->height);
        size_t total_shared_mem_size =
            SharedVideoFrameHeader::computeTotalSize(sharedFrameState->stride, sharedFrameState->height);
        assert(m_region->get_size() >= total_shared_mem_size);

        ++sharedFrameState->frame_index;
        std::memcpy(sharedFrameState->getBufferMutable(), buffer, buffer_size);
    }

protected:
    SharedVideoFrameHeader *getFrameHeader()
    {
        return reinterpret_cast<SharedVideoFrameHeader *>(m_region->get_address());
    }

private:
    const char *m_shared_memory_name;
    boost::interprocess::shared_memory_object *m_shared_memory_object;
    boost::interprocess::mapped_region *m_region;
};

struct OpenCVPlane2D
{
    cv::Point2f origin;
    cv::Point3f coefficients; // coefficients a, b, c in 2d plane equation: a*x + b*y + c = 0

    OpenCVPlane2D() 
        : origin(0.f, 0.f)
        , coefficients(0.f, 0.f, 0.f)
    {
    }

    OpenCVPlane2D(
        const cv::Point2f &o,
        const cv::Point3f &c) 
        : origin(o)
        , coefficients(c)
    {
    }

    static OpenCVPlane2D createFromPoints(const cv::Point2f &a, const cv::Point2f &b, const cv::Point2f &inside)
    {
        const cv::Point2f tangent= b - a;
        const float tangent_length= static_cast<float>(cv::norm(tangent));
        cv::Point2f normal=
            !is_nearly_zero(tangent_length)
            ? cv::Point2f(-tangent.y/tangent_length, tangent.x/tangent_length) 
            : cv::Point2f(0.f, 0.f);

        // Make sure the "inside point" is on the positive side of the plane
        if ((inside - a).dot(normal) < 0)
        {
            normal= -normal;
        }

        const cv::Point2f center= (a + b) / 2.f;

        return OpenCVPlane2D::createFromPointAndNormal(center, normal);
    }

    static OpenCVPlane2D createFromPointAndNormal(const cv::Point2f &p, const cv::Point2f &n)
    {
        const float n_length= static_cast<float>(cv::norm(n));
        const cv::Point2f normal= !is_nearly_zero(n_length) ? (n / n_length) : cv::Point2f(0.f, 0.f);

        cv::Point3f coefficients;
        coefficients.x= normal.x;        // coefficient a
        coefficients.y= normal.y;        // coefficient b
        coefficients.z= -normal.dot(p);  // coefficient c

        return OpenCVPlane2D(p, coefficients); 
    }

    inline cv::Point2f getOrigin() const
    {
        return origin;
    }

    inline cv::Point2f getNormal() const
    {
        return cv::Point2f(coefficients.x, coefficients.y);
    }

    inline bool isValidPlane() const
    {
        return !is_nearly_zero(coefficients.x) || !is_nearly_zero(coefficients.y);
    }

    float signedDistance(const cv::Point2f &point) const
    {
        return point.x*coefficients.x + point.y*coefficients.y + coefficients.z;
    }

    float unsignedDistance(const cv::Point2f &point) const
    {
        return fabsf(signedDistance(point));
    }

    cv::Point2f computeTangent() const
    {
        return cv::Point2f(-coefficients.y, coefficients.x);
    }
};

class OpenCVBGRToHSVMapper
{
public:
	typedef cv::Point3_<uint8_t> ColorTuple;

	static OpenCVBGRToHSVMapper *allocate()
	{
		if (m_refCount == 0)
		{
			assert(m_instance == nullptr);
			m_instance = new OpenCVBGRToHSVMapper();
		}
		assert(m_instance != nullptr);

		++m_refCount;
		return m_instance;
	}

	static void dispose(OpenCVBGRToHSVMapper *instance)
	{
		assert(m_instance != nullptr);
		assert(m_instance == instance);
		assert(m_refCount > 0);

		--m_refCount;
		if (m_refCount <= 0)
		{
			delete m_instance;
			m_instance = nullptr;
		}
	}

	void cvtColor(const cv::Mat &bgrBuffer, cv::Mat &hsvBuffer)
	{
		hsvBuffer.forEach<ColorTuple>([&bgrBuffer, this](ColorTuple &hsvColor, const int position[]) -> void {
			const ColorTuple &bgrColor = bgrBuffer.at<ColorTuple>(position[0], position[1]);
			const int b = bgrColor.x;
			const int g = bgrColor.y;
			const int r = bgrColor.z;
			const int LUTIndex = OpenCVBGRToHSVMapper::getLUTIndex(r, g, b);

			hsvColor = bgr2hsv->at<ColorTuple>(LUTIndex, 0);
		});
	}

private:
	static OpenCVBGRToHSVMapper *m_instance;
	static int m_refCount;

	OpenCVBGRToHSVMapper()
	{
		bgr2hsv = new cv::Mat(256*256*256, 1, CV_8UC3);

		int LUTIndex = 0;
		for (int r = 0; r < 256; ++r)
		{
			for (int g = 0; g < 256; ++g)
			{
				for (int b = 0; b < 256; ++b)
				{
					bgr2hsv->at<ColorTuple>(LUTIndex, 0) = ColorTuple(b, g, r);
					++LUTIndex;
				}
			}
		}

		cv::cvtColor(*bgr2hsv, *bgr2hsv, cv::COLOR_BGR2HSV);
	}

	~OpenCVBGRToHSVMapper()
	{
		delete bgr2hsv;
	}

	static int getLUTIndex(int r, int g, int b)
	{
		return (256 * 256)*r + 256*g + b;
	}

	cv::Mat *bgr2hsv;
};
OpenCVBGRToHSVMapper *OpenCVBGRToHSVMapper::m_instance = nullptr;
int OpenCVBGRToHSVMapper::m_refCount= 0;

class OpenCVBufferState
{
public:
    OpenCVBufferState(int width, int height)
        : frameWidth(width)
        , frameHeight(height)
        , bgrBuffer(nullptr)
        , hsvBuffer(nullptr)
        , gsLowerBuffer(nullptr)
        , gsUpperBuffer(nullptr)
        , maskedBuffer(nullptr)
    {
		const TrackerManagerConfig &cfg= DeviceManager::getInstance()->m_tracker_manager->getConfig();

        bgrBuffer = new cv::Mat(height, width, CV_8UC3);
        hsvBuffer = new cv::Mat(height, width, CV_8UC3);
        gsLowerBuffer = new cv::Mat(height, width, CV_8UC1);
        gsUpperBuffer = new cv::Mat(height, width, CV_8UC1);
        maskedBuffer = new cv::Mat(height, width, CV_8UC3);
		
		if (cfg.use_bgr_to_hsv_lookup_table)
		{
			bgr2hsv = OpenCVBGRToHSVMapper::allocate();
		}
		else
		{
			bgr2hsv = nullptr;
		}
    }

    virtual ~OpenCVBufferState()
    {
        if (maskedBuffer != nullptr)
        {
            delete maskedBuffer;
        }

        if (gsLowerBuffer != nullptr)
        {
            delete gsLowerBuffer;
        }

        if (gsUpperBuffer != nullptr)
        {
            delete gsUpperBuffer;
        }

        if (hsvBuffer != nullptr)
        {
            delete hsvBuffer;
        }

        if (bgrBuffer != nullptr)
        {
            delete bgrBuffer;
        }

		if (bgr2hsv != nullptr)
		{
			OpenCVBGRToHSVMapper::dispose(bgr2hsv);
		}
    }

    void writeVideoFrame(const unsigned char *video_buffer)
    {
        const cv::Mat videoBufferMat(frameHeight, frameWidth, CV_8UC3, const_cast<unsigned char *>(video_buffer));

        // Copy and Flip image about the x-axis
        cv::flip(videoBufferMat, *bgrBuffer, 1);

        // Convert the video buffer to the HSV color space
		if (bgr2hsv != nullptr)
		{
			bgr2hsv->cvtColor(*bgrBuffer, *hsvBuffer);
		}
		else
		{
			cv::cvtColor(*bgrBuffer, *hsvBuffer, cv::COLOR_BGR2HSV);
		}
    }

    // Return points in raw image space:
    // i.e. [0, 0] at lower left  to [frameWidth-1, frameHeight-1] at lower right
    bool computeBiggestNContours(
        const CommonHSVColorRange &hsvColorRange,
		t_opencv_contour_list &out_biggest_N_contours,
		const int max_contour_count)
    {
        // Clamp the HSV image, taking into account wrapping the hue angle
        {
            const float hue_min = hsvColorRange.hue_range.center - hsvColorRange.hue_range.range;
            const float hue_max = hsvColorRange.hue_range.center + hsvColorRange.hue_range.range;
            const float saturation_min = clampf(hsvColorRange.saturation_range.center - hsvColorRange.saturation_range.range, 0, 255);
            const float saturation_max = clampf(hsvColorRange.saturation_range.center + hsvColorRange.saturation_range.range, 0, 255);
            const float value_min = clampf(hsvColorRange.value_range.center - hsvColorRange.value_range.range, 0, 255);
            const float value_max = clampf(hsvColorRange.value_range.center + hsvColorRange.value_range.range, 0, 255);

            if (hue_min < 0)
            {
                cv::inRange(
                    *hsvBuffer,
                    cv::Scalar(0, saturation_min, value_min),
                    cv::Scalar(clampf(hue_max, 0, 180), saturation_max, value_max),
                    *gsLowerBuffer);
                cv::inRange(
                    *hsvBuffer,
                    cv::Scalar(clampf(180 + hue_min, 0, 180), saturation_min, value_min),
                    cv::Scalar(180, saturation_max, value_max),
                    *gsUpperBuffer);
                cv::bitwise_or(*gsLowerBuffer, *gsUpperBuffer, *gsLowerBuffer);
            }
            else if (hue_max > 180)
            {
                cv::inRange(
                    *hsvBuffer,
                    cv::Scalar(0, saturation_min, value_min),
                    cv::Scalar(clampf(hue_max - 180, 0, 180), saturation_max, value_max),
                    *gsLowerBuffer);
                cv::inRange(
                    *hsvBuffer,
                    cv::Scalar(clampf(hue_min, 0, 180), saturation_min, value_min),
                    cv::Scalar(180, saturation_max, value_max),
                    *gsUpperBuffer);
                cv::bitwise_or(*gsLowerBuffer, *gsUpperBuffer, *gsLowerBuffer);
            }
            else
            {
                cv::inRange(
                    *hsvBuffer,
                    cv::Scalar(hue_min, saturation_min, value_min),
                    cv::Scalar(hue_max, saturation_max, value_max),
                    *gsLowerBuffer);
            }
        }

        // Find the largest convex blob in the filtered grayscale buffer
        {
			struct ContourInfo
			{
				int contour_index;
				double contour_area;
			};
			std::vector<ContourInfo> sorted_contour_list;

			// Find all counters in the image buffer
			t_opencv_contour_list contours;
            cv::findContours(*gsLowerBuffer, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

			// Compute the area of each contour
			int contour_index = 0;
            for (auto it = contours.begin(); it != contours.end(); ++it) 
            {
				const double contour_area = cv::contourArea(*it);
				const ContourInfo contour_info = { contour_index, contour_area };

				sorted_contour_list.push_back(contour_info);
				++contour_index;
            }
			
			// Sort the list of contours by area, largest to smallest
			if (sorted_contour_list.size() > 1)
			{
				std::sort(
					sorted_contour_list.begin(), sorted_contour_list.end(), 
					[](const ContourInfo &a, const ContourInfo &b) {
						return b.contour_area < a.contour_area;
				});
			}

			// Copy up to N valid contours
			for (auto it = sorted_contour_list.begin(); 
				it != sorted_contour_list.end() && static_cast<int>(out_biggest_N_contours.size()) < max_contour_count; 
				++it)
			{
				const ContourInfo &contour_info = *it;
				t_opencv_contour &contour = contours[contour_info.contour_index];

				if (contour.size() > 6)
				{
					// Remove any points in contour on edge of camera/ROI
					t_opencv_contour::iterator it = contour.begin();
					while (it != contour.end()) 
					{
						if (it->x == 0 || it->x == (frameWidth - 1) || it->y == 0 || it->y == (frameHeight - 1))
						{
							it = contour.erase(it);
						}
						else
						{
							++it;
						}
					}

					// Add cleaned up contour to the output list
					out_biggest_N_contours.push_back(contour);
				}
			}
        }

        return (out_biggest_N_contours.size() > 0);
    }

    int frameWidth;
    int frameHeight;
    cv::Mat *bgrBuffer; // source video frame
    cv::Mat *hsvBuffer; // source frame converted to HSV color space
    cv::Mat *gsLowerBuffer; // HSV image clamped by HSV range into grayscale mask
    cv::Mat *gsUpperBuffer; // HSV image clamped by HSV range into grayscale mask
    cv::Mat *maskedBuffer; // bgr image ANDed together with grayscale mask
	OpenCVBGRToHSVMapper *bgr2hsv; // Used to convert an rgb image to an hsv image
};

// -- Utility Methods -----
static glm::quat computeGLMCameraTransformQuaternion(const ITrackerInterface *tracker_device);
static glm::mat4 computeGLMCameraTransformMatrix(const ITrackerInterface *tracker_device);
static cv::Matx34f computeOpenCVCameraExtrinsicMatrix(const ITrackerInterface *tracker_device);
static cv::Matx33f computeOpenCVCameraIntrinsicMatrix(const ITrackerInterface *tracker_device);
static cv::Matx34f computeOpenCVCameraPinholeMatrix(const ITrackerInterface *tracker_device);
static bool computeTrackerRelativeLightBarContourPose(
    const ITrackerInterface *tracker_device,
    const CommonDeviceTrackingShape *tracking_shape,
    const t_opencv_contour &opencv_contour,
    const CommonDevicePose *tracker_relative_pose_guess,
    ControllerOpticalPoseEstimation *out_pose_estimate);
static bool computeTrackerRelativePointCloudContourPose(
	const ITrackerInterface *tracker_device,
	const CommonDeviceTrackingShape *tracking_shape,
	const t_opencv_contour_list &opencv_contours,
	const CommonDevicePose *tracker_relative_pose_guess,
	HMDOpticalPoseEstimation *out_pose_estimate);
static bool computeBestFitTriangleForContour(
    const t_opencv_contour &opencv_contour,
    cv::Point2f &out_triangle_top,
    cv::Point2f &out_triangle_bottom_left,
    cv::Point2f &out_triangle_bottom_right);
static bool computeBestFitQuadForContour(
    const t_opencv_contour &opencv_contour,
    const cv::Point2f &up_hint, 
    const cv::Point2f &right_hint,
    cv::Point2f &top_right,
    cv::Point2f &top_left,
    cv::Point2f &bottom_left,
    cv::Point2f &bottom_right);
static void commonDeviceOrientationToOpenCVRodrigues(
    const CommonDeviceQuaternion &orientation,
    cv::Mat &rvec);
static void openCVRodriguesToAngleAxis(
    const cv::Mat &rvec,
    float &axis_x, float &axis_y, float &axis_z, float &radians);
static void angleAxisVectorToEulerAngles(
    const float axis_x, const float axis_y, const float axis_z, const float radians,
    float &yaw, float &pitch, float &roll);
static void angleAxisVectorToCommonDeviceOrientation(
    const float axis_x, const float axis_y, const float axis_z, const float radians,
    CommonDeviceQuaternion &orientation);

//-- public implementation -----
ServerTrackerView::ServerTrackerView(const int device_id)
    : ServerDeviceView(device_id)
    , m_shared_memory_accesor(nullptr)
    , m_shared_memory_video_stream_count(0)
    , m_opencv_buffer_state(nullptr)
    , m_device(nullptr)
{
    ServerUtility::format_string(m_shared_memory_name, sizeof(m_shared_memory_name), "tracker_view_%d", device_id);
}

ServerTrackerView::~ServerTrackerView()
{
    if (m_shared_memory_accesor != nullptr)
    {
        delete m_shared_memory_accesor;
    }

    if (m_opencv_buffer_state != nullptr)
    {
        delete m_opencv_buffer_state;
    }

    if (m_device != nullptr)
    {
        delete m_device;
    }
}

CommonDeviceState::eDeviceType
ServerTrackerView::getTrackerDeviceType() const
{
    return m_device->getDeviceType();
}

ITrackerInterface::eDriverType 
ServerTrackerView::getTrackerDriverType() const
{
    return m_device->getDriverType();
}

std::string
ServerTrackerView::getUSBDevicePath() const
{
    return m_device->getUSBDevicePath();
}

std::string 
ServerTrackerView::getSharedMemoryStreamName() const
{
    return std::string(m_shared_memory_name);
}

bool ServerTrackerView::open(const class DeviceEnumerator *enumerator)
{
    bool bSuccess = ServerDeviceView::open(enumerator);

    if (bSuccess)
    {
        int width, height, stride;

        // Make sure the shared memory block has been removed first
        boost::interprocess::shared_memory_object::remove(m_shared_memory_name);

        // Query the video frame first so that we know how big to make the buffer
        if (m_device->getVideoFrameDimensions(&width, &height, &stride))
        {
            assert(m_shared_memory_accesor == nullptr);
            m_shared_memory_accesor = new SharedVideoFrameReadWriteAccessor();

            if (!m_shared_memory_accesor->initialize(m_shared_memory_name, width, height, stride))
            {
                delete m_shared_memory_accesor;
                m_shared_memory_accesor = nullptr;

                SERVER_LOG_ERROR("ServerTrackerView::open()") << "Failed to allocated shared memory: " << m_shared_memory_name;
            }

            // Allocate the OpenCV scratch buffers used for finding tracking blobs
            m_opencv_buffer_state = new OpenCVBufferState(width, height);
        }
        else
        {
            SERVER_LOG_ERROR("ServerTrackerView::open()") << "Failed to video frame dimensions";
        }
    }

    return bSuccess;
}

void ServerTrackerView::close()
{
    if (m_shared_memory_accesor != nullptr)
    {
        delete m_shared_memory_accesor;
        m_shared_memory_accesor = nullptr;
    }

    ServerDeviceView::close();
}

void ServerTrackerView::startSharedMemoryVideoStream()
{
    ++m_shared_memory_video_stream_count;
}

void ServerTrackerView::stopSharedMemoryVideoStream()
{
    assert(m_shared_memory_video_stream_count > 0);
    --m_shared_memory_video_stream_count;
}

bool ServerTrackerView::poll()
{
    bool bSuccess = ServerDeviceView::poll();

    if (bSuccess && m_device != nullptr)
    {
        const unsigned char *buffer = m_device->getVideoFrameBuffer();

        if (buffer != nullptr)
        {
            // Cache the raw video frame and convert it to an HSV buffer for filtering later
            if (m_opencv_buffer_state != nullptr)
            {
                m_opencv_buffer_state->writeVideoFrame(buffer);

                // Copy the video frame to shared memory (if requested)
                if (m_shared_memory_accesor != nullptr && m_shared_memory_video_stream_count > 0)
                {
                    m_shared_memory_accesor->writeVideoFrame(m_opencv_buffer_state->bgrBuffer->data);
                }
            }
        }
    }

    return bSuccess;
}

bool ServerTrackerView::allocate_device_interface(const class DeviceEnumerator *enumerator)
{
    switch (enumerator->get_device_type())
    {
    case CommonDeviceState::PS3EYE:
    {
        m_device = new PS3EyeTracker();
    } break;
    default:
        break;
    }

    return m_device != nullptr;
}

void ServerTrackerView::free_device_interface()
{
    if (m_device != nullptr)
    {
        delete m_device;  // Deleting abstract object should be OK because
        // this (ServerDeviceView) is abstract as well.
        // All non-abstract children will have non-abstract types
        // for m_device.
        m_device = nullptr;
    }
}

void ServerTrackerView::publish_device_data_frame()
{
    // Tell the server request handler we want to send out tracker updates.
    // This will call generate_tracker_data_frame_for_stream for each listening connection.
    ServerRequestHandler::get_instance()->publish_tracker_data_frame(
        this, &ServerTrackerView::generate_tracker_data_frame_for_stream);
}

void ServerTrackerView::generate_tracker_data_frame_for_stream(
    const ServerTrackerView *tracker_view,
    const struct TrackerStreamInfo *stream_info,
    DeviceOutputDataFramePtr &data_frame)
{
    PSMoveProtocol::DeviceOutputDataFrame_TrackerDataPacket *tracker_data_frame =
        data_frame->mutable_tracker_data_packet();

    tracker_data_frame->set_tracker_id(tracker_view->getDeviceID());
    tracker_data_frame->set_sequence_num(tracker_view->m_sequence_number);
    tracker_data_frame->set_isconnected(tracker_view->getIsOpen());

    switch (tracker_view->getTrackerDeviceType())
    {
    case CommonDeviceState::PS3EYE:
        {
            //TODO: PS3EYE tracker location
        } break;
    default:
        assert(0 && "Unhandled Tracker type");
    }

    data_frame->set_device_category(PSMoveProtocol::DeviceOutputDataFrame::TRACKER);
}

double ServerTrackerView::getExposure() const
{
    return m_device->getExposure();
}

void ServerTrackerView::setExposure(double value)
{
    m_device->setExposure(value);
}

double ServerTrackerView::getGain() const
{
	return m_device->getGain();
}

void ServerTrackerView::setGain(double value)
{
	m_device->setGain(value);
}

void ServerTrackerView::getCameraIntrinsics(
    float &outFocalLengthX, float &outFocalLengthY,
    float &outPrincipalX, float &outPrincipalY) const
{
    m_device->getCameraIntrinsics(
        outFocalLengthX, outFocalLengthY,
        outPrincipalX, outPrincipalY);
}

void ServerTrackerView::setCameraIntrinsics(
    float focalLengthX, float focalLengthY,
    float principalX, float principalY)
{
    m_device->setCameraIntrinsics(focalLengthX, focalLengthY, principalX, principalY);
}

CommonDevicePose ServerTrackerView::getTrackerPose() const
{
    return m_device->getTrackerPose();
}

void ServerTrackerView::setTrackerPose(
    const struct CommonDevicePose *pose)
{
    m_device->setTrackerPose(pose);
}

void ServerTrackerView::getPixelDimensions(float &outWidth, float &outHeight) const
{
    int pixelWidth, pixelHeight;

    m_device->getVideoFrameDimensions(&pixelWidth, &pixelHeight, nullptr);

    outWidth = static_cast<float>(pixelWidth);
    outHeight = static_cast<float>(pixelHeight);
}

void ServerTrackerView::getFOV(float &outHFOV, float &outVFOV) const
{
    m_device->getFOV(outHFOV, outVFOV);
}

void ServerTrackerView::getZRange(float &outZNear, float &outZFar) const
{
    m_device->getZRange(outZNear, outZFar);
}

void ServerTrackerView::gatherTrackerOptions(PSMoveProtocol::Response_ResultTrackerSettings* settings) const
{
    m_device->gatherTrackerOptions(settings);
}

bool ServerTrackerView::setOptionIndex(const std::string &option_name, int option_index)
{
    return m_device->setOptionIndex(option_name, option_index);
}

bool ServerTrackerView::getOptionIndex(const std::string &option_name, int &out_option_index) const
{
    return m_device->getOptionIndex(option_name, out_option_index);
}

void ServerTrackerView::gatherTrackingColorPresets(
	const class ServerControllerView *controller, 
	PSMoveProtocol::Response_ResultTrackerSettings* settings) const
{
	std::string controller_id= (controller != nullptr) ? controller->getConfigIdentifier() : "";

    return m_device->gatherTrackingColorPresets(controller_id, settings);
}

void ServerTrackerView::gatherTrackingColorPresets(
	const class ServerHMDView *hmd,
	PSMoveProtocol::Response_ResultTrackerSettings* settings) const
{
	std::string hmd_id = (hmd != nullptr) ? hmd->getConfigIdentifier() : "";

	return m_device->gatherTrackingColorPresets(hmd_id, settings);
}

void ServerTrackerView::setControllerTrackingColorPreset(
	const class ServerControllerView *controller,
	eCommonTrackingColorID color, 
	const CommonHSVColorRange *preset)
{
	std::string controller_id= (controller != nullptr) ? controller->getConfigIdentifier() : "";

    return m_device->setTrackingColorPreset(controller_id, color, preset);
}

void ServerTrackerView::getControllerTrackingColorPreset(
	const class ServerControllerView *controller,
	eCommonTrackingColorID color,
	CommonHSVColorRange *out_preset) const
{
	std::string controller_id= (controller != nullptr) ? controller->getConfigIdentifier() : "";

    return m_device->getTrackingColorPreset(controller_id, color, out_preset);
}

void ServerTrackerView::setHMDTrackingColorPreset(
	const class ServerHMDView *hmd,
	eCommonTrackingColorID color,
	const CommonHSVColorRange *preset)
{
	std::string hmd_id = (hmd != nullptr) ? hmd->getConfigIdentifier() : "";

	return m_device->setTrackingColorPreset(hmd_id, color, preset);
}

void ServerTrackerView::getHMDTrackingColorPreset(
	const class ServerHMDView *hmd,
	eCommonTrackingColorID color,
	CommonHSVColorRange *out_preset) const
{
	std::string hmd_id = (hmd != nullptr) ? hmd->getConfigIdentifier() : "";

	return m_device->getTrackingColorPreset(hmd_id, color, out_preset);
}

bool
ServerTrackerView::computePoseForController(
    const ServerControllerView* tracked_controller,
    const CommonDevicePose *tracker_pose_guess,
    ControllerOpticalPoseEstimation *out_pose_estimate)
{
    bool bSuccess = true;

    // Get the tracking shape used by the controller
    CommonDeviceTrackingShape tracking_shape;
    if (bSuccess)
    {
        bSuccess = tracked_controller->getTrackingShape(tracking_shape);
    }

    // Get the HSV filter used to find the tracking blob
    CommonHSVColorRange hsvColorRange;
    if (bSuccess)
    {
        eCommonTrackingColorID tracked_color_id = tracked_controller->getTrackingColorID();

        if (tracked_color_id != eCommonTrackingColorID::INVALID_COLOR)
        {
            getControllerTrackingColorPreset(tracked_controller, tracked_color_id, &hsvColorRange);
        }
        else
        {
            bSuccess = false;
        }
    }

    // Find the contour associated with the controller
	t_opencv_contour_list biggest_contours;
    if (bSuccess)
    {
        ///###HipsterSloth $TODO - ROI seed on last known position, clamp to frame edges. 
        bSuccess = m_opencv_buffer_state->computeBiggestNContours(hsvColorRange, biggest_contours, 1);
    }

    // Compute the tracker relative 3d position of the controller from the contour
    if (bSuccess)
    {
        float F_PX, F_PY;
        float PrincipalX, PrincipalY;
        m_device->getCameraIntrinsics(F_PX, F_PY, PrincipalX, PrincipalY);

        // TODO: cv::undistortPoints  http://docs.opencv.org/3.1.0/da/d54/group__imgproc__transform.html#ga55c716492470bfe86b0ee9bf3a1f0f7e&gsc.tab=0
        // Then replace F_PX with -1.

        switch (tracking_shape.shape_type)
        {
        case eCommonTrackingShapeType::Sphere:
            {                               
                float frameWidth, frameHeight;
                getPixelDimensions(frameWidth, frameHeight);

                // Compute the convex hull of the contour
				t_opencv_contour convex_contour;
                cv::convexHull(biggest_contours[0], convex_contour);

                // Convert opencv_contour in raw pixel space:
                // i.e. [0, 0]x[frameWidth-1, frameHeight-1]
                // eigen_contour in CommonDeviceScreenLocation space:
                // i.e. [-frameWidth/2, -frameHeight/2]x[frameWidth/2, frameHeight/2]   
                // TODO: Replace this with cv::undistortPoints
                //http://docs.opencv.org/3.1.0/da/d54/group__imgproc__transform.html#ga55c716492470bfe86b0ee9bf3a1f0f7e&gsc.tab=0
                std::vector<Eigen::Vector2f> eigen_contour;
                std::for_each(
                    convex_contour.begin(),
                    convex_contour.end(),
                    [frameWidth, frameHeight, &eigen_contour](cv::Point& p) {
                        eigen_contour.push_back(Eigen::Vector2f(p.x - (frameWidth / 2), (frameHeight / 2) - p.y));
                    });

                // Compute the sphere center AND the projected ellipse
                Eigen::Vector3f sphere_center;
                EigenFitEllipse ellipse_projection;
                eigen_alignment_fit_focal_cone_to_sphere(
                    eigen_contour.data(),
                    static_cast<int>(eigen_contour.size()),
                    tracking_shape.shape.sphere.radius,
                    F_PX,
                    &sphere_center,
                    &ellipse_projection);

                out_pose_estimate->position.set(sphere_center.x(), sphere_center.y(), sphere_center.z());
                out_pose_estimate->bCurrentlyTracking = true;

                // Not possible to get an orientation off of a sphere
                out_pose_estimate->orientation.clear();
                out_pose_estimate->bOrientationValid = false;

                // Save off the projection of the sphere (an ellipse)
                out_pose_estimate->projection.shape_type = eCommonTrackingProjectionType::ProjectionType_Ellipse;
                out_pose_estimate->projection.shape.ellipse.center.set(ellipse_projection.center.x(), ellipse_projection.center.y());
                out_pose_estimate->projection.shape.ellipse.half_x_extent = ellipse_projection.extents.x();
                out_pose_estimate->projection.shape.ellipse.half_y_extent = ellipse_projection.extents.y();
                out_pose_estimate->projection.shape.ellipse.angle = ellipse_projection.angle;
                out_pose_estimate->projection.screen_area= ellipse_projection.area;

                bSuccess = true;
            } break;
        case eCommonTrackingShapeType::LightBar:
            {
                bSuccess= 
                    computeTrackerRelativeLightBarContourPose(
                        m_device,
                        &tracking_shape,
                        biggest_contours[0],
                        tracker_pose_guess,
                        out_pose_estimate);
            } break;
        default:
            assert(0 && "Unreachable");
            break;
        }
    }

    return bSuccess;
}

bool ServerTrackerView::computePoseForHMD(
	const class ServerHMDView* tracked_hmd,
	const CommonDevicePose *tracker_pose_guess,
	struct HMDOpticalPoseEstimation *out_pose_estimate)
{
	bool bSuccess = true;

	// Get the tracking shape used by the controller
	CommonDeviceTrackingShape tracking_shape;
	if (bSuccess)
	{
		bSuccess = tracked_hmd->getTrackingShape(tracking_shape);
	}

	// Get the HSV filter used to find the tracking blob
	CommonHSVColorRange hsvColorRange;
	if (bSuccess)
	{
		eCommonTrackingColorID tracked_color_id = tracked_hmd->getTrackingColorID();

		if (tracked_color_id != eCommonTrackingColorID::INVALID_COLOR)
		{
			getHMDTrackingColorPreset(tracked_hmd, tracked_color_id, &hsvColorRange);
		}
		else
		{
			bSuccess = false;
		}
	}

	// Find the N best contours associated with the HMD
	t_opencv_contour_list biggest_contours;
	if (bSuccess)
	{
		///###HipsterSloth $TODO - ROI seed on last known position, clamp to frame edges.
		bSuccess = 
			m_opencv_buffer_state->computeBiggestNContours(
				hsvColorRange, biggest_contours, CommonDeviceTrackingProjection::MAX_POINT_CLOUD_POINT_COUNT);
	}

	// Compute the tracker relative 3d position of the controller from the contour
	if (bSuccess)
	{
		float F_PX, F_PY;
		float PrincipalX, PrincipalY;
		m_device->getCameraIntrinsics(F_PX, F_PY, PrincipalX, PrincipalY);

		// TODO: cv::undistortPoints  http://docs.opencv.org/3.1.0/da/d54/group__imgproc__transform.html#ga55c716492470bfe86b0ee9bf3a1f0f7e&gsc.tab=0
		// Then replace F_PX with -1.

		switch (tracking_shape.shape_type)
		{
		case eCommonTrackingShapeType::PointCloud:
		{
			bSuccess =
				computeTrackerRelativePointCloudContourPose(
					m_device,
					&tracking_shape,
					biggest_contours,
					tracker_pose_guess,
					out_pose_estimate);
		} break;
		default:
			assert(0 && "Unreachable");
			break;
		}
	}

	return bSuccess;
}

CommonDevicePosition
ServerTrackerView::computeWorldPosition(
    const CommonDevicePosition *tracker_relative_position)
{
    const glm::vec4 rel_pos(tracker_relative_position->x, tracker_relative_position->y, tracker_relative_position->z, 1.f);
    const glm::mat4 cameraTransform= computeGLMCameraTransformMatrix(m_device);
    const glm::vec4 world_pos = cameraTransform * rel_pos;
    
    CommonDevicePosition result;
    result.set(world_pos.x, world_pos.y, world_pos.z);

    return result;
}

CommonDeviceQuaternion
ServerTrackerView::computeWorldOrientation(
    const CommonDeviceQuaternion *tracker_relative_orientation)
{
    const glm::quat rel_orientation(
        tracker_relative_orientation->w,
        tracker_relative_orientation->x,
        tracker_relative_orientation->y,
        tracker_relative_orientation->z);    
    const glm::quat camera_quat= computeGLMCameraTransformQuaternion(m_device);
    // combined_rotation = second_rotation * first_rotation;
    const glm::quat world_quat = camera_quat * rel_orientation;
    
    CommonDeviceQuaternion result;
    result.w= world_quat.w;
    result.x= world_quat.x;
    result.y= world_quat.y;
    result.z= world_quat.z;

    return result;
}

CommonDevicePose
ServerTrackerView::triangulateWorldPose(
    const ServerTrackerView *tracker, 
    const CommonDeviceTrackingProjection *tracker_relative_projection,
    const ServerTrackerView *other_tracker,
    const CommonDeviceTrackingProjection *other_tracker_relative_projection)
{
    assert(tracker_relative_projection->shape_type == other_tracker_relative_projection->shape_type);
    CommonDevicePose pose;

    pose.clear();
    switch(tracker_relative_projection->shape_type)
    {
    case eCommonTrackingProjectionType::ProjectionType_Ellipse:
        {
			//###HipsterSloth $TODO
        } break;
    case eCommonTrackingProjectionType::ProjectionType_LightBar:
        {
			//###HipsterSloth $TODO
        } break;
	case eCommonTrackingProjectionType::ProjectionType_Points:
		{
			//###HipsterSloth $TODO
		} break;
    default:
        assert(0 && "unreachable");
    }

    return pose;
}

CommonDevicePosition
ServerTrackerView::triangulateWorldPosition(
    const ServerTrackerView *tracker, 
    const CommonDeviceScreenLocation *screen_location,
    const ServerTrackerView *other_tracker,
    const CommonDeviceScreenLocation *other_screen_location)
{
    // Convert the tracker screen locations in CommonDeviceScreenLocation space
    // i.e. [-frameWidth/2, -frameHeight/2]x[frameWidth/2, frameHeight/2] 
    // into OpenCV pixel space
    // i.e. [0, 0]x[frameWidth, frameHeight]
    float screenWidth, screenHeight;
    tracker->getPixelDimensions(screenWidth, screenHeight);

    float otherScreenWidth, otherScreenHeight;
    tracker->getPixelDimensions(otherScreenWidth, otherScreenHeight);

    cv::Mat projPoints1 = 
        cv::Mat(cv::Point2f(
            screen_location->x + (screenWidth / 2), 
            screen_location->y + (screenHeight / 2)));
    cv::Mat projPoints2 = 
        cv::Mat(cv::Point2f(
            other_screen_location->x + (otherScreenWidth / 2),
            other_screen_location->y + (otherScreenHeight / 2)));

    // Compute the pinhole camera matrix for each tracker that allows you to raycast
    // from the tracker center in world space through the screen location, into the world
    // See: http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    cv::Mat projMat1 = cv::Mat(computeOpenCVCameraPinholeMatrix(tracker->m_device));
    cv::Mat projMat2 = cv::Mat(computeOpenCVCameraPinholeMatrix(other_tracker->m_device));

    // Triangulate the world position from the two cameras
    cv::Mat point3D(1, 1, CV_32FC4);
    cv::triangulatePoints(projMat1, projMat2, projPoints1, projPoints2, point3D);

    // Return the world space position
    CommonDevicePosition result;
    const float w = point3D.at<float>(3, 0);
    result.x = point3D.at<float>(0, 0) / w;
    result.y = point3D.at<float>(1, 0) / w;
    result.z = point3D.at<float>(2, 0) / w;

    return result;
}

CommonDeviceScreenLocation
ServerTrackerView::projectTrackerRelativePosition(const CommonDevicePosition *trackerRelativePosition) const
{
    CommonDeviceScreenLocation screenLocation;

    // Assume no distortion
    // TODO: Probably should get the distortion coefficients out of the tracker
    cv::Mat cvDistCoeffs(4, 1, cv::DataType<float>::type);
    cvDistCoeffs.at<float>(0) = 0;
    cvDistCoeffs.at<float>(1) = 0;
    cvDistCoeffs.at<float>(2) = 0;
    cvDistCoeffs.at<float>(3) = 0;

    // Use the identity transform for tracker relative positions
    cv::Mat rvec(3, 1, cv::DataType<double>::type, double(0));
    cv::Mat tvec(3, 1, cv::DataType<double>::type, double(0));

    // Only one point to project
    std::vector<cv::Point3f> cvObjectPoints;
    cvObjectPoints.push_back(
        cv::Point3f(
        trackerRelativePosition->x,
        trackerRelativePosition->y,
        trackerRelativePosition->z));

    // Compute the camera intrinsic matrix in opencv format
    cv::Matx33f cvCameraMatrix = computeOpenCVCameraIntrinsicMatrix(m_device);

    // Projected point 
    std::vector<cv::Point2f> projectedPoints;
    cv::projectPoints(cvObjectPoints, rvec, tvec, cvCameraMatrix, cvDistCoeffs, projectedPoints);

    // cv::projectPoints() returns position in pixel coordinates where:
    //  (0, 0) is the lower left of the screen and +y is pointing up
    // Convert this to CommonDeviceScreenLocation space where:
    //  (0, 0) in the center of the screen with +y is pointing up
    {
        float screenWidth, screenHeight;
        getPixelDimensions(screenWidth, screenHeight);

        screenLocation.x = projectedPoints[0].x - (screenWidth / 2);
        screenLocation.y = projectedPoints[0].y - (screenHeight / 2);
    }

    return screenLocation;
}


// -- Tracker Utility Methods -----
static glm::quat computeGLMCameraTransformQuaternion(const ITrackerInterface *tracker_device)
{

    const CommonDevicePose pose = tracker_device->getTrackerPose();
    const CommonDeviceQuaternion &quat = pose.Orientation;

    const glm::quat glm_quat(quat.w, quat.x, quat.y, quat.z);

    return glm_quat;
}

static glm::mat4 computeGLMCameraTransformMatrix(const ITrackerInterface *tracker_device)
{

    const CommonDevicePose pose = tracker_device->getTrackerPose();
    const CommonDeviceQuaternion &quat = pose.Orientation;
    const CommonDevicePosition &pos = pose.Position;

    const glm::quat glm_quat(quat.w, quat.x, quat.y, quat.z);
    const glm::vec3 glm_pos(pos.x, pos.y, pos.z);
    const glm::mat4 glm_camera_xform = glm_mat4_from_pose(glm_quat, glm_pos);

    return glm_camera_xform;
}

static cv::Matx34f computeOpenCVCameraExtrinsicMatrix(const ITrackerInterface *tracker_device)
{
    cv::Matx34f out;

    // Extrinsic matrix is the inverse of the camera pose matrix
    const glm::mat4 glm_camera_xform = computeGLMCameraTransformMatrix(tracker_device);
    const glm::mat4 glm_mat = glm::inverse(glm_camera_xform);

    out(0, 0) = glm_mat[0][0]; out(0, 1) = glm_mat[1][0]; out(0, 2) = glm_mat[2][0]; out(0, 3) = glm_mat[3][0];
    out(1, 0) = glm_mat[0][1]; out(1, 1) = glm_mat[1][1]; out(1, 2) = glm_mat[2][1]; out(1, 3) = glm_mat[3][1];
    out(2, 0) = glm_mat[0][2]; out(2, 1) = glm_mat[1][2]; out(2, 2) = glm_mat[2][2]; out(2, 3) = glm_mat[3][2];

    return out;
}

static cv::Matx33f computeOpenCVCameraIntrinsicMatrix(const ITrackerInterface *tracker_device)
{
    cv::Matx33f out;

    float F_PX, F_PY;
    float PrincipalX, PrincipalY;
    tracker_device->getCameraIntrinsics(F_PX, F_PY, PrincipalX, PrincipalY);

    out(0, 0) = F_PX; out(0, 1) = 0.f; out(0, 2) = PrincipalX;
    out(1, 0) = 0.f; out(1, 1) = F_PY; out(1, 2) = PrincipalY;
    out(2, 0) = 0.f; out(2, 1) = 0.f; out(2, 2) = 1.f;

    return out;
}

static cv::Matx34f computeOpenCVCameraPinholeMatrix(const ITrackerInterface *tracker_device)
{
    cv::Matx34f extrinsic_matrix = computeOpenCVCameraExtrinsicMatrix(tracker_device);
    cv::Matx33f intrinsic_matrix = computeOpenCVCameraIntrinsicMatrix(tracker_device);
    cv::Matx34f pinhole_matrix = intrinsic_matrix * extrinsic_matrix;

    return pinhole_matrix;
}

static bool computeTrackerRelativeLightBarContourPose(
    const ITrackerInterface *tracker_device,
    const CommonDeviceTrackingShape *tracking_shape,
    const t_opencv_contour &opencv_contour,
    const CommonDevicePose *tracker_relative_pose_guess,
    ControllerOpticalPoseEstimation *out_pose_estimate)
{
    assert(tracking_shape->shape_type == eCommonTrackingShapeType::LightBar);

    // Get the pixel width and height of the tracker image
    int pixelWidth, pixelHeight;
    tracker_device->getVideoFrameDimensions(&pixelWidth, &pixelHeight, nullptr);

    bool bValidTrackerPose= true;
    float projectionArea= 0.f;
    std::vector<cv::Point2f> cvImagePoints;
    {
        cv::Point2f tri_top, tri_bottom_left, tri_bottom_right;
        cv::Point2f quad_top_right, quad_top_left, quad_bottom_left, quad_bottom_right;

        // Create a best fit triangle around the contour
        bValidTrackerPose= computeBestFitTriangleForContour(
            opencv_contour, 
            tri_top, tri_bottom_left, tri_bottom_right);

        // Also create a best fit quad around the contour
        // Use the best fit triangle to define the orientation
        if (bValidTrackerPose)
        {
            // Use the triangle to define an up and a right direction
            const cv::Point2f up_hint= tri_top - 0.5f*(tri_bottom_left + tri_bottom_right);
            const cv::Point2f right_hint= tri_bottom_right - tri_bottom_left;

            bValidTrackerPose= computeBestFitQuadForContour(
                opencv_contour, 
                up_hint, right_hint, 
                quad_top_right, quad_top_left, quad_bottom_left, quad_bottom_right);
        }

        if (bValidTrackerPose)
        {
            // In practice the best fit triangle top is a bit noisy.
            // Since it should be at the midpoint of the top of the quad we use that instead.
            tri_top= 0.5f*(quad_top_right + quad_top_left);

            // Put the image points in corresponding order with cvObjectPoints
            cvImagePoints.push_back(tri_bottom_right);
            cvImagePoints.push_back(tri_bottom_left);
            cvImagePoints.push_back(tri_top);
            cvImagePoints.push_back(quad_top_right);
            cvImagePoints.push_back(quad_top_left);
            cvImagePoints.push_back(quad_bottom_left);
            cvImagePoints.push_back(quad_bottom_right);

            // The projection area is the size of the best fit quad
            projectionArea= 
                static_cast<float>(
                    cv::norm(quad_bottom_right-quad_bottom_left)
                    *cv::norm(quad_bottom_left-quad_top_left));

            // Image pixel coordinates
            // intrinsic camera transform
            for (auto list_index = 0; list_index < cvImagePoints.size(); ++list_index)
            {
                cv::Point2f &cvPoint= cvImagePoints[list_index];

                cvPoint.y= pixelHeight - cvPoint.y;
            }                    
        }
    }

    // Solve the tracking position using solvePnP
    if (bValidTrackerPose)
    {
        // Copy the object/image point mappings into OpenCV format
        // Assumed vertex order is:
        // triangle - right, left, bottom
        // quad - top right, top left, bottom left, bottom right
        std::vector<cv::Point3f> cvObjectPoints;

        for (int corner_index= 0; corner_index < 3; ++corner_index)
        {        
            const CommonDevicePosition &corner = tracking_shape->shape.light_bar.triangle[corner_index];

            cvObjectPoints.push_back(cv::Point3f(corner.x, corner.y, corner.z));
        }

        for (int corner_index= 0; corner_index < 4; ++corner_index)
        {        
            const CommonDevicePosition &corner = tracking_shape->shape.light_bar.quad[corner_index];

            cvObjectPoints.push_back(cv::Point3f(corner.x, corner.y, corner.z));
        }

        // Assume no distortion
        // TODO: Probably should get the distortion coefficients out of the tracker
        cv::Mat cvDistCoeffs(4, 1, cv::DataType<float>::type);
        cvDistCoeffs.at<float>(0) = 0;
        cvDistCoeffs.at<float>(1) = 0;
        cvDistCoeffs.at<float>(2) = 0;
        cvDistCoeffs.at<float>(3) = 0;

        // Get the tracker "intrinsic" matrix that encodes the camera FOV
        cv::Matx33f cvCameraMatrix = computeOpenCVCameraIntrinsicMatrix(tracker_device);

        // Fill out the initial guess in OpenCV format for the contour pose
        // if a guess pose was provided
        cv::Mat rvec(3, 1, cv::DataType<double>::type);
        cv::Mat tvec(3, 1, cv::DataType<double>::type);

        bool bUseExtrinsicGuess= false;
        if (tracker_relative_pose_guess != nullptr)
        {
            const float k_max_valid_guess_distance= 300.f; // cm
            float guess_position_distance_sqrd= 
                tracker_relative_pose_guess->Position.x*tracker_relative_pose_guess->Position.x
                + tracker_relative_pose_guess->Position.y*tracker_relative_pose_guess->Position.y
                + tracker_relative_pose_guess->Position.z*tracker_relative_pose_guess->Position.z;

            if (guess_position_distance_sqrd < k_max_valid_guess_distance*k_max_valid_guess_distance)
            {
                // solvePnP expects a rotation as a Rodrigues (AngleAxis) vector
                commonDeviceOrientationToOpenCVRodrigues(tracker_relative_pose_guess->Orientation, rvec);

                tvec.at<double>(0)= tracker_relative_pose_guess->Position.x;
                tvec.at<double>(1)= tracker_relative_pose_guess->Position.y;
                tvec.at<double>(2)= tracker_relative_pose_guess->Position.z;

                bUseExtrinsicGuess= true;
            }
        }

        // Solve the Perspective-N-Point problem:
        // Given a set of 3D points and their corresponding 2D pixel projections,
        // solve for the object position and orientation that would allow
        // us to re-project the 3D points back onto the 2D pixel locations
        if (cv::solvePnP(
                cvObjectPoints, cvImagePoints, 
                cvCameraMatrix, cvDistCoeffs, 
                rvec, tvec, 
                bUseExtrinsicGuess, cv::SOLVEPNP_ITERATIVE))
        {
            float axis_x, axis_y, axis_z, axis_theta;
            float yaw, pitch, roll;

            // Extract the angle-axis components from the solution OpenCV Rodrigues vector
            openCVRodriguesToAngleAxis(rvec, axis_x, axis_y, axis_z, axis_theta);

            // Convert the angle-axis rotation into Euler angles (yaw-pitch-roll)
            angleAxisVectorToEulerAngles(axis_x, axis_y, axis_z, axis_theta, yaw, pitch, roll);
           
            //###HipsterSloth $TODO This should be a property of the lightbar tracking shape
            static const float k_max_valid_tracking_pitch= 30.f*k_degrees_to_radians;
            static const float k_max_valid_tracking_yaw= 30.f*k_degrees_to_radians;

            // Due to ambiguity of the off the yaw and pitch solution from solvePnP (two possible solutions)
            // we can't trust anything more than close to straightforward.
            // Any roll angle is fine though.
            if (fabsf(yaw) < k_max_valid_tracking_yaw && fabsf(pitch) < k_max_valid_tracking_pitch)
            {           
                // Convert the solution angle-axis into a CommonDeviceOrientation
                angleAxisVectorToCommonDeviceOrientation(axis_x, axis_y, axis_z, axis_theta, out_pose_estimate->orientation);
                out_pose_estimate->bOrientationValid= true;
            }
            else
            {
                out_pose_estimate->bOrientationValid= false;
            }

            // Return the position in the pose
            {
                CommonDevicePosition &position= out_pose_estimate->position;

                position.x = static_cast<float>(tvec.at<double>(0));
                position.y = static_cast<float>(tvec.at<double>(1));
                position.z = static_cast<float>(tvec.at<double>(2));
            }

            bValidTrackerPose= true;
        }
    }

    // Return the projection of the tracking shape
    if (bValidTrackerPose)
    {
        CommonDeviceTrackingProjection *out_projection= &out_pose_estimate->projection;

        out_projection->shape_type = eCommonTrackingProjectionType::ProjectionType_LightBar;

        for (int vertex_index = 0; vertex_index < 3; ++vertex_index)
        {
            const cv::Point2f &cvPoint = cvImagePoints[vertex_index];

            // Convert the tracker screen locations in OpenCV pixel space
            // i.e. [0, 0]x[frameWidth, frameHeight]
            // into PSMoveScreenLocation space
            // i.e. [-frameWidth/2, -frameHeight/2]x[frameWidth/2, frameHeight/2] 
            out_projection->shape.lightbar.triangle[vertex_index] = 
                { cvPoint.x - (pixelWidth / 2), cvPoint.y - (pixelHeight / 2) };
        }

        for (int vertex_index = 0; vertex_index < 4; ++vertex_index)
        {
            const cv::Point2f &cvPoint = cvImagePoints[vertex_index + 3];

            // Convert the tracker screen locations in OpenCV pixel space
            // i.e. [0, 0]x[frameWidth, frameHeight]
            // into PSMoveScreenLocation space
            // i.e. [-frameWidth/2, -frameHeight/2]x[frameWidth/2, frameHeight/2] 
            out_projection->shape.lightbar.quad[vertex_index] = 
                { cvPoint.x - (pixelWidth / 2), cvPoint.y - (pixelHeight / 2) };
        }

        out_projection->screen_area= projectionArea;
    }

    return bValidTrackerPose;
}

static bool computeTrackerRelativePointCloudContourPose(
	const ITrackerInterface *tracker_device,
	const CommonDeviceTrackingShape *tracking_shape,
	const t_opencv_contour_list &opencv_contours,
	const CommonDevicePose *tracker_relative_pose_guess,
	HMDOpticalPoseEstimation *out_pose_estimate)
{
	assert(tracking_shape->shape_type == eCommonTrackingShapeType::PointCloud);

	// Get the pixel width and height of the tracker image
	int pixelWidth, pixelHeight;
	tracker_device->getVideoFrameDimensions(&pixelWidth, &pixelHeight, nullptr);

	bool bValidTrackerPose = true;
	float projectionArea = 0.f;

	// Compute centers of mass for the contours
	std::vector<cv::Point2f> cvImagePoints;
	for (auto it = opencv_contours.begin(); it != opencv_contours.end(); ++it)
	{
		cv::Moments mu = cv::moments(*it);
		cv::Point2f massCenter =
			cv::Point2f(
				static_cast<float>(mu.m10 / mu.m00) - (pixelWidth / 2), 
				(pixelHeight / 2) - static_cast<float>(mu.m01 / mu.m00));

		cvImagePoints.push_back(massCenter);
	}

	if (cvImagePoints.size() >= 3)
	{
		//###HipsterSloth $TODO Solve the pose using SoftPOSIT
		out_pose_estimate->position.clear();
		out_pose_estimate->orientation.clear();
		out_pose_estimate->bOrientationValid = false;
		bValidTrackerPose = true;
	}

	// Return the projection of the tracking shape
	if (bValidTrackerPose)
	{
		CommonDeviceTrackingProjection *out_projection = &out_pose_estimate->projection;
		const int imagePointCount = static_cast<int>(cvImagePoints.size());

		out_projection->shape_type = eCommonTrackingProjectionType::ProjectionType_Points;

		for (int vertex_index = 0; vertex_index < imagePointCount; ++vertex_index)
		{
			const cv::Point2f &cvPoint = cvImagePoints[vertex_index];

			out_projection->shape.points.point[vertex_index] = {cvPoint.x, cvPoint.y};
		}

		out_projection->shape.points.point_count = imagePointCount;
		out_projection->screen_area = projectionArea;
	}

	return bValidTrackerPose;
}

static bool computeBestFitTriangleForContour(
    const std::vector<cv::Point> &opencv_contour,
    cv::Point2f &out_triangle_top,
    cv::Point2f &out_triangle_bottom_left,
    cv::Point2f &out_triangle_bottom_right)
{
    // Compute the tightest possible bounding triangle for the given contour
    std::vector<cv::Point2f> cv_min_triangle;
    cv::minEnclosingTriangle(opencv_contour, cv_min_triangle);

    if (cv_min_triangle.size() != 3)
    {
        return false;
    }

    cv::Point2f best_fit_origin_01 = (cv_min_triangle[0] + cv_min_triangle[1]) / 2.f;
    cv::Point2f best_fit_origin_12 = (cv_min_triangle[1] + cv_min_triangle[2]) / 2.f;
    cv::Point2f best_fit_origin_20 = (cv_min_triangle[2] + cv_min_triangle[0]) / 2.f;

    std::vector<cv::Point2f> cv_midpoint_triangle;
    cv_midpoint_triangle.push_back(best_fit_origin_01);
    cv_midpoint_triangle.push_back(best_fit_origin_12);
    cv_midpoint_triangle.push_back(best_fit_origin_20);

    // Find the corner closest to the center of mass.
    // This is the bottom of the triangle.
    int topCornerIndex = -1;
    {
        cv::Moments mu = cv::moments(opencv_contour);
        cv::Point2f massCenter = 
            cv::Point2f(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));

        double bestDistance = k_real_max;
        for (int cornerIndex = 0; cornerIndex < 3; ++cornerIndex)
        {
            const double testDistance = cv::norm(cv_midpoint_triangle[cornerIndex] - massCenter);

            if (testDistance < bestDistance)
            {
                topCornerIndex = cornerIndex;
                bestDistance = testDistance;
            }
        }
    }

    // Assign the left and right corner indices
    int leftCornerIndex = -1;
    int rightCornerIndex = -1;
    switch (topCornerIndex)
    {
    case 0:
        leftCornerIndex = 1;
        rightCornerIndex = 2;
        break;
    case 1:
        leftCornerIndex = 0;
        rightCornerIndex = 2;
        break;
    case 2:
        leftCornerIndex = 0;
        rightCornerIndex = 1;
        break;
    default:
        assert(0 && "unreachable");
    }

    // Make sure the left and right corners are actually 
    // on the left and right of the triangle
    out_triangle_top = cv_midpoint_triangle[topCornerIndex];
    out_triangle_bottom_left = cv_midpoint_triangle[leftCornerIndex];
    out_triangle_bottom_right = cv_midpoint_triangle[rightCornerIndex];

    const cv::Point2f topToLeft = out_triangle_bottom_left - out_triangle_top;
    const cv::Point2f topToRight = out_triangle_bottom_right - out_triangle_top;

    // Cross product should be positive if sides are correct
    // If not, then swap them.
    if (topToRight.cross(topToLeft) < 0)
    {
        std::swap(out_triangle_bottom_left, out_triangle_bottom_right);
    }

    return true;
}

static bool computeBestFitQuadForContour(
    const std::vector<cv::Point> &opencv_contour,
    const cv::Point2f &up_hint, 
    const cv::Point2f &right_hint,
    cv::Point2f &top_right,
    cv::Point2f &top_left,
    cv::Point2f &bottom_left,
    cv::Point2f &bottom_right)
{
    // Compute the tightest possible bounding triangle for the given contour
    cv::RotatedRect cv_min_box= cv::minAreaRect(opencv_contour);

    if (cv_min_box.size.width <= k_real_epsilon || cv_min_box.size.height <= k_real_epsilon)
    {
        return false;
    }

    float half_width, half_height;
    float radians;
    if (cv_min_box.size.width > cv_min_box.size.height)
    {
        half_width= cv_min_box.size.width / 2.f;
        half_height= cv_min_box.size.height / 2.f;
        radians= cv_min_box.angle*k_degrees_to_radians;
    }
    else
    {
        half_width= cv_min_box.size.height / 2.f;
        half_height= cv_min_box.size.width / 2.f;
        radians= (cv_min_box.angle + 90.f)*k_degrees_to_radians;
    }

    cv::Point2f quad_half_right, quad_half_up;
    {
        const float cos_angle= cosf(radians);
        const float sin_angle= sinf(radians);

        quad_half_right.x= half_width*cos_angle;
        quad_half_right.y= half_width*sin_angle;

        quad_half_up.x= -half_height*sin_angle;
        quad_half_up.y= half_height*cos_angle;
    }

    if (quad_half_up.dot(up_hint) < 0)
    {
        // up axis is flipped
        // flip the box vertically
        quad_half_up= -quad_half_up;
    }

    if (quad_half_right.dot(right_hint) < 0)
    {
        // right axis is flipped
        // flip the box horizontally
        quad_half_right= -quad_half_right;
    }

    top_right= cv_min_box.center + quad_half_up + quad_half_right;
    top_left= cv_min_box.center + quad_half_up - quad_half_right;
    bottom_right= cv_min_box.center - quad_half_up + quad_half_right;
    bottom_left= cv_min_box.center - quad_half_up - quad_half_right;

    return true;
}

// http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm
static void commonDeviceOrientationToOpenCVRodrigues(
    const CommonDeviceQuaternion &orientation,
    cv::Mat &rvec)
{
    double qw= clampf(orientation.w, -1.0, 1.0);
    double angle = 2.0 * acos(qw);
    double axis_normalizer = sqrt(1.0 - qw*qw);

    if (axis_normalizer > k_real_epsilon) 
    {
        rvec.at<double>(0) = angle * (orientation.x / axis_normalizer);
        rvec.at<double>(1) = angle * (orientation.y / axis_normalizer);
        rvec.at<double>(2) = angle * (orientation.z / axis_normalizer);
    }
    else
    {
        // Angle is either 0 or 360,
        // which is a rotation no-op so we are free to pick any axis we want
        rvec.at<double>(0) = angle; 
        rvec.at<double>(1) = 0.0;
        rvec.at<double>(2) = 0.0;
    }
}

static void openCVRodriguesToAngleAxis(
    const cv::Mat &rvec,
    float &axis_x, float &axis_y, float &axis_z, float &radians)
{
    const float r_x = static_cast<float>(rvec.at<double>(0));
    const float r_y = static_cast<float>(rvec.at<double>(1));
    const float r_z = static_cast<float>(rvec.at<double>(2));
    
    radians = sqrtf(r_x*r_x + r_y*r_y + r_z*r_z);

    axis_x= safe_divide_with_default(r_x, radians, 1.f);
    axis_y= safe_divide_with_default(r_y, radians, 0.f);
    axis_z= safe_divide_with_default(r_z, radians, 0.f);
}

// http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToEuler/index.htm
// NOTE: This code has the X and Z axis flipped from the code in the link
// because I consider rotation about the X-axis pitch and the Z-axis roll
// whereas the original code had the opposite.
// Also they refer to yaw as "heading", pitch as "attitude", and roll ""
static void angleAxisVectorToEulerAngles(
    const float axis_x, const float axis_y, const float axis_z, const float radians,
    float &yaw, float &pitch, float &roll)
{
    float s= sinf(radians);
    float c= cosf(radians);
    float t= 1.f-c;

    if ((axis_x*axis_y*t + axis_z*s) > 0.998) 
    {
        // north pole singularity detected
        yaw = 2*atan2f(axis_z*sinf(radians/2), cosf(radians/2));
        pitch = k_real_half_pi;
        roll = 0;
    }
    else if ((axis_x*axis_y*t + axis_z*s) < -0.998) 
    { 
        // south pole singularity detected
        yaw = -2*atan2(axis_z*sinf(radians/2), cosf(radians/2));
        pitch = -k_real_half_pi;
        roll = 0;
    }
    else
    {
        yaw = atan2f(axis_y*s - axis_x*axis_z*t, 1.f - (axis_y*axis_y + axis_z*axis_z)*t);
        pitch = asinf(axis_z*axis_y*t + axis_x*s) ;
        roll = atan2f(axis_z*s - axis_x*axis_y*t , 1.f - (axis_x*axis_x + axis_z*axis_z)*t);
    }
}

static void angleAxisVectorToCommonDeviceOrientation(
    const float axis_x, const float axis_y, const float axis_z, const float radians,
    CommonDeviceQuaternion &orientation)
{
    if (!is_nearly_zero(radians))
    {
        const float sin_theta_over_two = sinf(radians * 0.5f);

        orientation.w = cosf(radians * 0.5f);
        orientation.x = axis_x * sin_theta_over_two;
        orientation.y = axis_y * sin_theta_over_two;
        orientation.z = axis_z * sin_theta_over_two;
    }
    else
    {
        orientation.clear();
    }
}
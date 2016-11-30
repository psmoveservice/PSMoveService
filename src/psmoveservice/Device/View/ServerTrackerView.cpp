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
#include "PoseFilterInterface.h"

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
    OpenCVBufferState(ITrackerInterface *device)
        : bgrBuffer(nullptr)
        , hsvBuffer(nullptr)
        , gsLowerBuffer(nullptr)
        , gsUpperBuffer(nullptr)
        , maskedBuffer(nullptr)
    {
        device->getVideoFrameDimensions(&frameWidth, &frameHeight, nullptr);

        bgrBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC3);
        hsvBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC3);
        gsLowerBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC1);
        gsUpperBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC1);
        maskedBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC3);

        intrinsic_matrix = new cv::Mat(3, 3, CV_64FC1);
        distortion_coeffs = new cv::Mat(5, 1, CV_64FC1);
        
		rebuildDistortionParameters(device);

        const TrackerManagerConfig &cfg= DeviceManager::getInstance()->m_tracker_manager->getConfig();
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
               
        // TODO: If not nullptr wrapper
        delete intrinsic_matrix;
        delete distortion_coeffs;
        
        if (bgr2hsv != nullptr)
        {
            OpenCVBGRToHSVMapper::dispose(bgr2hsv);
        }
    }

    void rebuildDistortionParameters(ITrackerInterface *device)
    {
        float F_PX, F_PY;
        float PrincipalX, PrincipalY;
        float distortionK1, distortionK2, distortionK3;
        float distortionP1, distortionP2;

        device->getCameraIntrinsics(
            F_PX, F_PY, 
            PrincipalX, PrincipalY,
            distortionK1, distortionK2, distortionK3,
            distortionP1, distortionP2);

        // Fill in the intrinsic matrix
        intrinsic_matrix->at<double>(0, 0)= F_PX;
        intrinsic_matrix->at<double>(1, 0)= 0.0;
        intrinsic_matrix->at<double>(2, 0)= 0.0;

        intrinsic_matrix->at<double>(0, 1)= 0.0;
        intrinsic_matrix->at<double>(1, 1)= F_PY;
        intrinsic_matrix->at<double>(2, 1)= 0.0;

        intrinsic_matrix->at<double>(0, 2)= PrincipalX;
        intrinsic_matrix->at<double>(1, 2)= PrincipalY;
        intrinsic_matrix->at<double>(2, 2)= 1.0;

        // Fill in the distortion coefficients
        distortion_coeffs->at<double>(0, 0)= distortionK1; 
        distortion_coeffs->at<double>(1, 0)= distortionK2;
        distortion_coeffs->at<double>(2, 0)= distortionP1;
        distortion_coeffs->at<double>(3, 0)= distortionP2;
        distortion_coeffs->at<double>(4, 0)= distortionK3;
    }

    void writeVideoFrame(const unsigned char *video_buffer)
    {
        const cv::Mat videoBufferMat(frameHeight, frameWidth, CV_8UC3, const_cast<unsigned char *>(video_buffer));

		videoBufferMat.copyTo(*bgrBuffer);

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
        std::vector<double> &out_contour_areas,
		const int max_contour_count,
        const int min_points_in_contour = 6)
    {
        out_biggest_N_contours.clear();
        out_contour_areas.clear();
        
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
        
        //TODO: Why no blurring of the gsLowerBuffer?

        // Find the largest convex blob in the filtered grayscale buffer
        {
			struct ContourInfo
			{
				int contour_index;
				double contour_area;
			};
			std::vector<ContourInfo> sorted_contour_list;

			// Find all counters in the image buffer
            cv::Size size; cv::Point ofs;
            gsLowerBuffer->locateROI(size, ofs);
			t_opencv_contour_list contours;
            cv::findContours(*gsLowerBuffer,
                             contours,
                             CV_RETR_EXTERNAL,
                             CV_CHAIN_APPROX_SIMPLE,  //CV_CHAIN_APPROX_NONE?
                             ofs);

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

                if (contour.size() > min_points_in_contour)
				{
					// Remove any points in contour on edge of camera/ROI
                    // TODO: Contours touching image border will be clipped,
                    // so this might not be necessary.
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
                    // Add its area to the output list too.
                    out_contour_areas.push_back(contour_info.contour_area);
				}
			}
        }

        return (out_biggest_N_contours.size() > 0);
    }
    
    void
    draw_contour(const t_opencv_contour &contour)
    {
        // This is useful for debugging
        std::vector<t_opencv_contour> contours = {contour};
        cv::Moments mu(cv::moments(contour));
        cv::Point2f massCenter = cv::Point2f(static_cast<float>(mu.m10 / mu.m00),
                                             static_cast<float>(mu.m01 / mu.m00));
        cv::drawContours(*bgrBuffer, contours, 0, cv::Scalar(255, 255, 255));
        cv::rectangle(*bgrBuffer, cv::boundingRect(contour), cv::Scalar(255, 255, 255));
        cv::drawMarker(*bgrBuffer, massCenter, cv::Scalar(255, 255, 255));
//        float contourArea = mu.m00;
    }
    
    void
    draw_pose(const ControllerOpticalPoseEstimation &pose_estimate,
              const cv::Matx33f &intrinsic_matrix)
    {
        if (pose_estimate.projection.shape_type == eCommonTrackingProjectionType::ProjectionType_Ellipse)
        {
            
            //Create cv::ellipse from pose_estimate
            cv::Point ell_center(pose_estimate.projection.shape.ellipse.center.x,
                                 pose_estimate.projection.shape.ellipse.center.y);
            cv::Size ell_size(pose_estimate.projection.shape.ellipse.half_x_extent,
                              pose_estimate.projection.shape.ellipse.half_y_extent);
            
            //Undo offset created by undistort
            ell_center.x += intrinsic_matrix.val[2];
            ell_center.y += intrinsic_matrix.val[5];
            
            //Draw ellipse on bgrBuffer
            cv::ellipse(*bgrBuffer,
                        ell_center,
                        ell_size,
                        pose_estimate.projection.shape.ellipse.angle,
                        0, 360, cv::Scalar(0, 0, 255));
            cv::drawMarker(*bgrBuffer, ell_center, cv::Scalar(0, 0, 255));
        }
    }

    int frameWidth;
    int frameHeight;

    cv::Mat *bgrBuffer; // source video frame
    cv::Mat *hsvBuffer; // source frame converted to HSV color space
    cv::Mat *gsLowerBuffer; // HSV image clamped by HSV range into grayscale mask
    cv::Mat *gsUpperBuffer; // HSV image clamped by HSV range into grayscale mask
    cv::Mat *maskedBuffer; // bgr image ANDed together with grayscale mask
    cv::Mat *intrinsic_matrix;
    cv::Mat *distortion_coeffs;
	OpenCVBGRToHSVMapper *bgr2hsv; // Used to convert an rgb image to an hsv image
};

// -- Utility Methods -----
static glm::quat computeGLMCameraTransformQuaternion(const ITrackerInterface *tracker_device);
static glm::mat4 computeGLMCameraTransformMatrix(const ITrackerInterface *tracker_device);
static cv::Matx34f computeOpenCVCameraExtrinsicMatrix(const ITrackerInterface *tracker_device);
static cv::Matx33f computeOpenCVCameraIntrinsicMatrix(const ITrackerInterface *tracker_device);
static cv::Matx34f computeOpenCVCameraPinholeMatrix(const ITrackerInterface *tracker_device);
static bool computeTrackerRelativeLightBarProjection(
	const ITrackerInterface *tracker_device,
	const CommonDeviceTrackingShape *tracking_shape,
	const t_opencv_contour &opencv_contour,
	CommonDeviceTrackingProjection *out_projection);
static bool computeTrackerRelativeLightBarPose(
	const ITrackerInterface *tracker_device,
	const CommonDeviceTrackingShape *tracking_shape,
	const CommonDeviceTrackingProjection *projection,
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
            m_opencv_buffer_state = new OpenCVBufferState(m_device);
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
    // Copy the video frame to shared memory (if requested)
    if (m_shared_memory_accesor != nullptr && m_shared_memory_video_stream_count > 0)
    {
        m_shared_memory_accesor->writeVideoFrame(m_opencv_buffer_state->bgrBuffer->data);
    }
    
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

void ServerTrackerView::loadSettings()
{
    m_device->loadSettings();
}

void ServerTrackerView::saveSettings()
{
    m_device->saveSettings();
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
    float &outPrincipalX, float &outPrincipalY,
    float &outDistortionK1, float &outDistortionK2, float &outDistortionK3,
    float &outDistortionP1, float &outDistortionP2) const
{
    m_device->getCameraIntrinsics(
        outFocalLengthX, outFocalLengthY,
        outPrincipalX, outPrincipalY,
        outDistortionK1, outDistortionK2, outDistortionK3,
        outDistortionP1, outDistortionP2);
}

void ServerTrackerView::setCameraIntrinsics(
    float focalLengthX, float focalLengthY,
    float principalX, float principalY,
    float distortionK1, float distortionK2, float distortionK3,
    float distortionP1, float distortionP2)
{
    m_device->setCameraIntrinsics(
        focalLengthX, focalLengthY,
        principalX, principalY,
        distortionK1, distortionK2, distortionK3,
        distortionP1, distortionP2);

    m_opencv_buffer_state->rebuildDistortionParameters(m_device);
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
ServerTrackerView::computeProjectionForController(
    const ServerControllerView* tracked_controller,
	const CommonDeviceTrackingShape *tracking_shape,
    ControllerOpticalPoseEstimation *out_pose_estimate)
{
    bool bSuccess = true;

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
    
    // Calculate expected ROI
    // Default to full screen.
    float screenWidth, screenHeight;
    this->getPixelDimensions(screenWidth, screenHeight);
    cv::Rect2i ROI(0, 0, screenWidth, screenHeight);
    
    //Calculate a more refined ROI.
    //Based on the physical limits of the object's bounding box
    //projected onto the image.
    if (tracked_controller->getIsCurrentlyTracking())
    {
        // Get the (predicted) position in world space.
        const IPoseFilter* pose_filter = tracked_controller->getPoseFilter();
        Eigen::Vector3f position = pose_filter->getPosition(0.f);  //TODO: Replace 0.f with the tracker time.
        CommonDevicePosition world_position;
        world_position.set(position.x(), position.y(), position.z());
        
        // Get the (predicted) position in tracker-local space.
        CommonDevicePosition tracker_position = computeTrackerPosition(&world_position);
        
        // Project the position +/- object extents onto the image.
        CommonDevicePosition tl, br;
        switch (tracking_shape->shape_type)
        {
            case eCommonTrackingShapeType::Sphere:
            {
                // Simply center - radius, center + radius.
                tl.set(tracker_position.x-tracking_shape->shape.sphere.radius,
                       tracker_position.y+tracking_shape->shape.sphere.radius,
                       tracker_position.z);
                br.set(tracker_position.x+tracking_shape->shape.sphere.radius,
                       tracker_position.y-tracking_shape->shape.sphere.radius,
                       tracker_position.z);
                
                //TODO: Move the rest out of this case as it should be common for all shapes.
                //Assuming all shapes can be defined by their bounding box.
                std::vector<CommonDevicePosition> trps{tl, br};
                std::vector<CommonDeviceScreenLocation> screen_locs = projectTrackerRelativePositions(trps);
                cv::Size roi_size(screen_locs[1].x-screen_locs[0].x, screen_locs[1].y-screen_locs[0].y);
                ROI = cv::Rect2i(cv::Point2i(screen_locs[0].x - roi_size.width/2,
                                             screen_locs[0].y - roi_size.height/2),
                                 roi_size);
                ROI += roi_size;  // Double its size.
                
                //Draw it
                cv::rectangle(*m_opencv_buffer_state->bgrBuffer, ROI, cv::Scalar(255, 0, 0));
            } break;
                
            case eCommonTrackingShapeType::LightBar:
            {
                
            } break;
                
            case eCommonTrackingShapeType::PointCloud:
            {
                
            } break;
                
            default:
            {
                // TODO: ROI is whole image
            } break;
        }
    }
    

    // Find the contour associated with the controller
	t_opencv_contour_list biggest_contours;
    std::vector<double> contour_areas;
    if (bSuccess)
    {
//        cv::Mat roi(img, cv::Rect(10,10,100,100));
        ///###HipsterSloth $TODO - ROI seed on last known position, clamp to frame edges.
        //cv::findContours can pass in an offset when using ROIs.
        bSuccess = m_opencv_buffer_state->computeBiggestNContours(hsvColorRange, biggest_contours, contour_areas, 1);
    }
    
    // Process the contour for its 2D and 3D pose.
    if (bSuccess)
    {
        // Compute the tracker relative 3d position of the controller from the contour
        switch (tracking_shape->shape_type)
        {
		// For the sphere projection we can go ahead and compute the full pose estimation now
        case eCommonTrackingShapeType::Sphere:
            {
                
                // Compute the convex hull of the contour
                t_opencv_contour convex_contour;
                cv::convexHull(biggest_contours[0], convex_contour);
                m_opencv_buffer_state->draw_contour(convex_contour);
                
                // Convert integer to float
                std::vector<cv::Point2f> convex_contour_f;
                cv::Mat(convex_contour).convertTo(convex_contour_f, cv::Mat(convex_contour_f).type());
                
                // Get camera parameters.
                // Needed for undistortion.
                float F_PX, F_PY;
                float PrincipalX, PrincipalY;
                float distortionK1, distortionK2, distortionK3;
                float distortionP1, distortionP2;
                m_device->getCameraIntrinsics(F_PX, F_PY,
                                              PrincipalX, PrincipalY,
                                              distortionK1, distortionK2, distortionK3,
                                              distortionP1, distortionP2);
                cv::Matx33f camera_matrix(F_PX,     0.0,    PrincipalX,
                                          0.0,      -F_PY,  PrincipalY,  //Negate F_PY because +Y is down on the image.
                                          0.0,      0.0,    1.0 );
                
                // Undistort points
                std::vector<cv::Point2f> undistort_contour;
                cv::undistortPoints(convex_contour_f, undistort_contour,
                                    camera_matrix,
                                    std::vector<float> {distortionK1, distortionK2, distortionP1, distortionP2, distortionK3});//,
//                                    cv::noArray(),
//                                    camera_matrix);
                // Note: if we omit the last two arguments, then
                // undistort_contour points are in 'normalized' space.
                // i.e., they are relative to their F_PX,F_PY
                // We then have to change F_PX in eigen_alignment_fit_focal_cone_to_sphere
                // below to the focal length this normalized space (i.e., +1, -1?)
                
                // Compute the sphere center AND the projected ellipse
                Eigen::Vector3f sphere_center;
                EigenFitEllipse ellipse_projection;

                std::vector<Eigen::Vector2f> eigen_contour;
                std::for_each(undistort_contour.begin(),
                              undistort_contour.end(),
                              [&eigen_contour](cv::Point2f& p) {
                                  eigen_contour.push_back(Eigen::Vector2f(p.x, p.y));
                              });
                eigen_alignment_fit_focal_cone_to_sphere(eigen_contour.data(),
                                                         eigen_contour.size(),
                                                         tracking_shape->shape.sphere.radius,
                                                         1, //I was expecting to negate this...
                                                         &sphere_center,
                                                         &ellipse_projection);
                
                
                ellipse_projection.center.x() = ellipse_projection.center.x()*camera_matrix.val[0];// + camera_matrix.val[2];
                ellipse_projection.center.y() = ellipse_projection.center.y()*camera_matrix.val[4];// + camera_matrix.val[5];
                ellipse_projection.extents *= camera_matrix.val[0];

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
                
                //Draw results onto m_opencv_buffer_state
                m_opencv_buffer_state->draw_pose(*out_pose_estimate,
                                                 camera_matrix);

                bSuccess = true;
            } break;
		// For the LightBar projection we only want to compute the projection shape.
		// The pose estimation is deferred until we know if we can leverage triangulation or not.
        case eCommonTrackingShapeType::LightBar:
            {
                bSuccess= 
                    computeTrackerRelativeLightBarProjection(
                        m_device,
                        tracking_shape,
                        biggest_contours[0],
                        &out_pose_estimate->projection);
            } break;
        default:
            assert(0 && "Unreachable");
            break;
        }
    }

    return bSuccess;
}

bool 
ServerTrackerView::computePoseForProjection(
	const CommonDeviceTrackingProjection *projection,
	const CommonDeviceTrackingShape *tracking_shape,
	const CommonDevicePose *pose_guess,
	ControllerOpticalPoseEstimation *out_pose_estimate)
{
	bool bSuccess = false;

	switch (projection->shape_type)
	{
	case eCommonTrackingShapeType::Sphere:
		{
			// Nothing to do. The pose estimate is already computed in computeProjectionForController()
			bSuccess = true;
		} break;
	case eCommonTrackingShapeType::LightBar:
		{
			bSuccess =
				computeTrackerRelativeLightBarPose(
					m_device,
					tracking_shape,
					projection,
					pose_guess,
					out_pose_estimate);
		} break;
	default:
		assert(0 && "Unreachable");
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
    std::vector<double> contour_areas;
	if (bSuccess)
	{
		///###HipsterSloth $TODO - ROI seed on last known position, clamp to frame edges.
		bSuccess = 
			m_opencv_buffer_state->computeBiggestNContours(
				hsvColorRange, biggest_contours, contour_areas, CommonDeviceTrackingProjection::MAX_POINT_CLOUD_POINT_COUNT);
	}

	// Compute the tracker relative 3d position of the controller from the contour
	if (bSuccess)
	{
        float F_PX, F_PY;
        float PrincipalX, PrincipalY;
        float distortionK1, distortionK2, distortionK3;
        float distortionP1, distortionP2;
        m_device->getCameraIntrinsics(
                                      F_PX, F_PY,
                                      PrincipalX, PrincipalY,
                                      distortionK1, distortionK2, distortionK3,
                                      distortionP1, distortionP2);

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
	// Compute a rotations that rotates from +X to global "forward"
	const TrackerManagerConfig &cfg = DeviceManager::getInstance()->m_tracker_manager->getConfig();
	const float global_forward_yaw_radians = cfg.global_forward_degrees*k_degrees_to_radians;
	const glm::quat global_forward_quat= glm::quat(glm::vec3(0.f, global_forward_yaw_radians, 0.f));
	
	const glm::quat rel_orientation(
        tracker_relative_orientation->w,
        tracker_relative_orientation->x,
        tracker_relative_orientation->y,
        tracker_relative_orientation->z);    
    const glm::quat camera_quat= computeGLMCameraTransformQuaternion(m_device);
    const glm::quat world_quat = global_forward_quat * camera_quat * rel_orientation;
    
    CommonDeviceQuaternion result;
    result.w= world_quat.w;
    result.x= world_quat.x;
    result.y= world_quat.y;
    result.z= world_quat.z;

    return result;
}

CommonDevicePosition 
ServerTrackerView::computeTrackerPosition(
	const CommonDevicePosition *world_relative_position)
{
    const glm::vec4 world_pos(world_relative_position->x, world_relative_position->y, world_relative_position->z, 1.f);
    const glm::mat4 invCameraTransform= glm::inverse(computeGLMCameraTransformMatrix(m_device));
    const glm::vec4 rel_pos = invCameraTransform * world_pos;
    
    CommonDevicePosition result;
    result.set(rel_pos.x, rel_pos.y, rel_pos.z);

    return result;
}

CommonDeviceQuaternion 
ServerTrackerView::computeTrackerOrientation(
	const CommonDeviceQuaternion *world_relative_orientation)
{
    const glm::quat world_orientation(
        world_relative_orientation->w,
        world_relative_orientation->x,
        world_relative_orientation->y,
        world_relative_orientation->z);    
    const glm::quat camera_inv_quat= glm::conjugate(computeGLMCameraTransformQuaternion(m_device));
    // combined_rotation = second_rotation * first_rotation;
    const glm::quat rel_quat = camera_inv_quat * world_orientation;
    
    CommonDeviceQuaternion result;
    result.w= rel_quat.w;
    result.x= rel_quat.x;
    result.y= rel_quat.y;
    result.z= rel_quat.z;

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
			pose.Position =
				triangulateWorldPosition(
					tracker,
					&tracker_relative_projection->shape.ellipse.center,
					other_tracker,
					&other_tracker_relative_projection->shape.ellipse.center);
			pose.Orientation.clear();
        } break;
    case eCommonTrackingProjectionType::ProjectionType_LightBar:
        {
			// Copy the lightbar triangle and quad screen space points into flat arrays
			const int k_vertex_count= CommonDeviceTrackingShape::QuadVertexCount+CommonDeviceTrackingShape::TriVertexCount;
			CommonDeviceScreenLocation screen_locations[k_vertex_count];
			CommonDeviceScreenLocation other_screen_locations[k_vertex_count];			
			for (int quad_index = 0; quad_index < CommonDeviceTrackingShape::QuadVertexCount; ++quad_index)
			{
				screen_locations[quad_index]= tracker_relative_projection->shape.lightbar.quad[quad_index];
				other_screen_locations[quad_index]= other_tracker_relative_projection->shape.lightbar.quad[quad_index];
			}
			for (int tri_index = 0; tri_index < CommonDeviceTrackingShape::TriVertexCount; ++tri_index)
			{
				screen_locations[CommonDeviceTrackingShape::QuadVertexCount + tri_index]= 
					tracker_relative_projection->shape.lightbar.triangle[tri_index];
				other_screen_locations[CommonDeviceTrackingShape::QuadVertexCount + tri_index]=
					other_tracker_relative_projection->shape.lightbar.triangle[tri_index];
			}

			// Triangulate the 7 points on the lightbar
			Eigen::Vector3f lightbar_points[k_vertex_count];
			{
				CommonDevicePosition world_positions[k_vertex_count];
				ServerTrackerView::triangulateWorldPositions(
					tracker, 
					screen_locations,
					other_tracker,
					other_screen_locations,
					k_vertex_count,
					world_positions);

				for (int point_index = 0; point_index < k_vertex_count; ++point_index)
				{
					const CommonDevicePosition &p= world_positions[point_index];

					lightbar_points[point_index]= Eigen::Vector3f(p.x, p.y, p.z);
				}
			}

			// Compute best fit plane for the world space light bar points
			Eigen::Vector3f centroid, normal;
			if (eigen_alignment_fit_least_squares_plane(
					lightbar_points, k_vertex_count,
					&centroid, &normal))
			{
				// Assume that the normal for the projection should be facing the tracker.
				// Since the projection is planar and both trackers can see the projection
				// it doesn't matter which tracker we use for the facing test.
				{
					const CommonDevicePosition commonTrackerPosition= tracker->getTrackerPose().Position;
					const Eigen::Vector3f trackerPosition(commonTrackerPosition.x, commonTrackerPosition.y, commonTrackerPosition.z);
					const Eigen::Vector3f centroidToTracker= trackerPosition - centroid;

					if (centroidToTracker.dot(normal) < 0.f)
					{
						normal= -normal;
					}
				}

				// Project the lightbar 
				float projection_error= eigen_alignment_project_points_on_plane(centroid, normal, lightbar_points, k_vertex_count);

				// Compute the orientation of the lightbar
				// Forward is the normal vector
				// Up is defined by the orientation of the lightbar vertices
				{
					const Eigen::Vector3f &mid_left_vertex= 
						(lightbar_points[CommonDeviceTrackingShape::QuadVertexUpperLeft] 
						+ lightbar_points[CommonDeviceTrackingShape::QuadVertexLowerLeft]) / 2.f;
					const Eigen::Vector3f &mid_right_vertex =
						(lightbar_points[CommonDeviceTrackingShape::QuadVertexUpperRight]
						+ lightbar_points[CommonDeviceTrackingShape::QuadVertexLowerRight]) / 2.f;
					const Eigen::Vector3f right= mid_right_vertex - mid_left_vertex;

					// Get the global definition of tracking space "forward" and "right"
					const TrackerManagerConfig &cfg= DeviceManager::getInstance()->m_tracker_manager->getConfig();
					const CommonDeviceVector &global_forward = cfg.get_global_forward_axis();
					const CommonDeviceVector &global_right = cfg.get_global_right_axis();
					const Eigen::Vector3f eigen_global_forward(global_forward.i, global_forward.j, global_forward.k);
					const Eigen::Vector3f eigen_global_right(global_right.i, global_right.j, global_right.k);

					// Compute the rotation that would align the global forward and right 
					// with the normal and right vectors computed for the light bar
					const Eigen::Quaternionf align_normal_rotation= 
						Eigen::Quaternionf::FromTwoVectors(eigen_global_forward, normal);
					const Eigen::Vector3f x_axis_in_plane = 
						align_normal_rotation * eigen_global_right;
					const Eigen::Quaternionf align_right_rotation = 
						Eigen::Quaternionf::FromTwoVectors(x_axis_in_plane, right);
					const Eigen::Quaternionf q = align_right_rotation*align_normal_rotation;

					pose.Orientation.w= q.w();
					pose.Orientation.x= q.x();
					pose.Orientation.y= q.y();
					pose.Orientation.z= q.z();
				}

				// Use the centroid as the world pose location
				pose.Position.x= centroid.x();
				pose.Position.y= centroid.y();
				pose.Position.z= centroid.z();
			}
			else
			{
				pose.clear();
			}
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


    float F_PX, F_PY;
    float PrincipalX, PrincipalY;
    float distortionK1, distortionK2, distortionK3;
    float distortionP1, distortionP2;

    tracker->getCameraIntrinsics(
        F_PX, F_PY, 
        PrincipalX, PrincipalY,
        distortionK1, distortionK2, distortionK3,
        distortionP1, distortionP2);

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

void
ServerTrackerView::triangulateWorldPositions(
    const ServerTrackerView *tracker, 
    const CommonDeviceScreenLocation *screen_locations,
    const ServerTrackerView *other_tracker,
    const CommonDeviceScreenLocation *other_screen_locations,
	const int screen_location_count,
	CommonDevicePosition *out_result)
{
    // Convert the tracker screen locations in CommonDeviceScreenLocation space
    // i.e. [-frameWidth/2, -frameHeight/2]x[frameWidth/2, frameHeight/2] 
    // into OpenCV pixel space
    // i.e. [0, 0]x[frameWidth, frameHeight]
    float screenWidth, screenHeight;
    tracker->getPixelDimensions(screenWidth, screenHeight);

    float otherScreenWidth, otherScreenHeight;
    tracker->getPixelDimensions(otherScreenWidth, otherScreenHeight);

	std::vector<cv::Point2f> projPoints1;
	std::vector<cv::Point2f> projPoints2;
	for (int point_index = 0; point_index < screen_location_count; ++point_index)
	{
		const CommonDeviceScreenLocation &p1= screen_locations[point_index];
		const CommonDeviceScreenLocation &p2= other_screen_locations[point_index];

		projPoints1.push_back(cv::Point2f(p1.x + (screenWidth / 2), p1.y + (screenHeight / 2)));
		projPoints2.push_back(cv::Point2f(p2.x + (screenWidth / 2), p2.y + (screenHeight / 2)));
	}

    // Compute the pinhole camera matrix for each tracker that allows you to raycast
    // from the tracker center in world space through the screen location, into the world
    // See: http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    cv::Mat projMat1 = cv::Mat(computeOpenCVCameraPinholeMatrix(tracker->m_device));
    cv::Mat projMat2 = cv::Mat(computeOpenCVCameraPinholeMatrix(other_tracker->m_device));

    // Triangulate the world positions from the two cameras
    cv::Mat points3D(1, screen_location_count, CV_32FC4);
    cv::triangulatePoints(projMat1, projMat2, projPoints1, projPoints2, points3D);

    // Return the world space positions
	for (int point_index = 0; point_index < screen_location_count; ++point_index)
	{
		CommonDevicePosition &result= out_result[point_index];

		const float w = points3D.at<float>(3, point_index);
		result.x = points3D.at<float>(0, point_index) / w;
		result.y = points3D.at<float>(1, point_index) / w;
		result.z = points3D.at<float>(2, point_index) / w;
	}
}


std::vector<CommonDeviceScreenLocation>
ServerTrackerView::projectTrackerRelativePositions(const std::vector<CommonDevicePosition> &objectPositions) const
{
    float F_PX, F_PY;
    float PrincipalX, PrincipalY;
    float distortionK1, distortionK2, distortionK3;
    float distortionP1, distortionP2;
    m_device->getCameraIntrinsics(F_PX, F_PY,
                                  PrincipalX, PrincipalY,
                                  distortionK1, distortionK2, distortionK3,
                                  distortionP1, distortionP2);
    
    cv::Matx33f camera_matrix(F_PX,     0.0,    PrincipalX,
                              0.0,      -F_PY,  PrincipalY,  //Negate F_PY because +Y is down on the image.
                              0.0,      0.0,    1.0 );
    
    // Distortion coefficients into a matrix
    cv::Mat cvDistCoeffs(5, 1, cv::DataType<float>::type);
    cvDistCoeffs.at<float>(0) = distortionK1;
    cvDistCoeffs.at<float>(1) = distortionK2;
    cvDistCoeffs.at<float>(2) = distortionP1;
    cvDistCoeffs.at<float>(3) = distortionP2;
    cvDistCoeffs.at<float>(4) = distortionK3;
    
    // Use the identity transform for tracker relative positions
    cv::Mat rvec(3, 1, cv::DataType<double>::type, double(0));
    cv::Mat tvec(3, 1, cv::DataType<double>::type, double(0));
    
    std::vector<cv::Point3f> cvObjectPoints;
    size_t i;
    for (i=0; i<objectPositions.size(); ++i) {
        cvObjectPoints.push_back(cv::Point3f(objectPositions[i].x,
                                             objectPositions[i].y,
                                             objectPositions[i].z));
    }
    
    // Projected point
    std::vector<cv::Point2f> projectedPoints;
    cv::projectPoints(cvObjectPoints,
                      rvec,
                      tvec,
                      camera_matrix,
                      cvDistCoeffs,
                      projectedPoints);
    
    std::vector<CommonDeviceScreenLocation> screenLocations;
    for (i=0; i<projectedPoints.size(); ++i) {
        CommonDeviceScreenLocation thisloc;
        thisloc.set(projectedPoints[i].x, projectedPoints[i].y);
        screenLocations.push_back(thisloc);
    }
    
    return screenLocations;
}

CommonDeviceScreenLocation
ServerTrackerView::projectTrackerRelativePosition(const CommonDevicePosition *trackerRelativePosition) const
{
    std::vector<CommonDevicePosition> trp_vec {*trackerRelativePosition};
    CommonDeviceScreenLocation screenLocation = projectTrackerRelativePositions(trp_vec)[0];

    // cv::projectPoints() returns position in pixel coordinates where:
    //  (0, 0) is the lower left of the screen and +y is pointing up
    // Convert this to CommonDeviceScreenLocation space where:
    //  (0, 0) in the center of the screen with +y is pointing up
    {
        float screenWidth, screenHeight;
        getPixelDimensions(screenWidth, screenHeight);

        screenLocation.x = screenLocation.x - (screenWidth / 2);
        screenLocation.y = screenLocation.y - (screenHeight / 2);
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
    float distortionK1, distortionK2, distortionK3;
    float distortionP1, distortionP2;
    tracker_device->getCameraIntrinsics(
                                  F_PX, F_PY,
                                  PrincipalX, PrincipalY,
                                  distortionK1, distortionK2, distortionK3,
                                  distortionP1, distortionP2);

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

static bool computeTrackerRelativeLightBarProjection(
    const ITrackerInterface *tracker_device,
    const CommonDeviceTrackingShape *tracking_shape,
    const t_opencv_contour &opencv_contour,
	CommonDeviceTrackingProjection *out_projection)
{
    assert(tracking_shape->shape_type == eCommonTrackingShapeType::LightBar);

    // Get the pixel width and height of the tracker image
    int pixelWidth, pixelHeight;
    tracker_device->getVideoFrameDimensions(&pixelWidth, &pixelHeight, nullptr);

    bool bValidTrackerProjection= true;
    float projectionArea= 0.f;
    std::vector<cv::Point2f> cvImagePoints;
    {
        cv::Point2f tri_top, tri_bottom_left, tri_bottom_right;
        cv::Point2f quad_top_right, quad_top_left, quad_bottom_left, quad_bottom_right;

        // Create a best fit triangle around the contour
        bValidTrackerProjection= computeBestFitTriangleForContour(
            opencv_contour, 
            tri_top, tri_bottom_left, tri_bottom_right);

        // Also create a best fit quad around the contour
        // Use the best fit triangle to define the orientation
        if (bValidTrackerProjection)
        {
            // Use the triangle to define an up and a right direction
            const cv::Point2f up_hint= tri_top - 0.5f*(tri_bottom_left + tri_bottom_right);
            const cv::Point2f right_hint= tri_bottom_right - tri_bottom_left;

            bValidTrackerProjection= computeBestFitQuadForContour(
                opencv_contour, 
                up_hint, right_hint, 
                quad_top_right, quad_top_left, quad_bottom_left, quad_bottom_right);
        }

        if (bValidTrackerProjection)
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

    // Return the projection of the tracking shape
    if (bValidTrackerProjection)
    {
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

    return bValidTrackerProjection;
}

static bool computeTrackerRelativeLightBarPose(
    const ITrackerInterface *tracker_device,
	const CommonDeviceTrackingShape *tracking_shape,
	const CommonDeviceTrackingProjection *projection,
	const CommonDevicePose *tracker_relative_pose_guess,
	ControllerOpticalPoseEstimation *out_pose_estimate)
{
	assert(tracking_shape->shape_type == eCommonTrackingShapeType::LightBar);
    assert(projection->shape_type == eCommonTrackingProjectionType::ProjectionType_LightBar);

    // Get the pixel width and height of the tracker image
    int pixelWidth, pixelHeight;
    tracker_device->getVideoFrameDimensions(&pixelWidth, &pixelHeight, nullptr);

    bool bValidTrackerPose= true;
    std::vector<cv::Point2f> cvImagePoints;

	for (int vertex_index = 0; vertex_index < 3; ++vertex_index)
	{
		const CommonDeviceScreenLocation &screenLocation= projection->shape.lightbar.triangle[vertex_index];

		// Convert from PSMoveScreenLocation space
		// i.e. [-frameWidth/2, -frameHeight/2]x[frameWidth/2, frameHeight/2] 
		// into tracker screen locations in OpenCV pixel space
		// i.e. [0, 0]x[frameWidth, frameHeight]
		cvImagePoints.push_back(cv::Point2f(screenLocation.x + (pixelWidth / 2), screenLocation.y + (pixelHeight / 2)));
	}

	for (int vertex_index = 0; vertex_index < 4; ++vertex_index)
	{
		const CommonDeviceScreenLocation &screenLocation = projection->shape.lightbar.quad[vertex_index];

		// Convert from PSMoveScreenLocation space
		// i.e. [-frameWidth/2, -frameHeight/2]x[frameWidth/2, frameHeight/2] 
		// into tracker screen locations in OpenCV pixel space
		// i.e. [0, 0]x[frameWidth, frameHeight]
		cvImagePoints.push_back(cv::Point2f(screenLocation.x + (pixelWidth / 2), screenLocation.y + (pixelHeight / 2)));
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

//-- includes -----
#include "ServerTrackerView.h"
#include "ServerControllerView.h"
#include "DeviceEnumerator.h"
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

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <memory>

#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#define USE_OPEN_CV_ELLIPSE_FIT

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
        bgrBuffer = new cv::Mat(height, width, CV_8UC3);
        hsvBuffer = new cv::Mat(height, width, CV_8UC3);
        gsLowerBuffer = new cv::Mat(height, width, CV_8UC1);
        gsUpperBuffer = new cv::Mat(height, width, CV_8UC1);
        maskedBuffer = new cv::Mat(height, width, CV_8UC3);
    }

    virtual ~OpenCVBufferState()
    {
        if (maskedBuffer != nullptr)
        {
            delete maskedBuffer;
            maskedBuffer = nullptr;
        }

        if (gsLowerBuffer != nullptr)
        {
            delete gsLowerBuffer;
            gsLowerBuffer = nullptr;
        }

        if (gsUpperBuffer != nullptr)
        {
            delete gsUpperBuffer;
            gsUpperBuffer = nullptr;
        }

        if (hsvBuffer != nullptr)
        {
            delete hsvBuffer;
            hsvBuffer = nullptr;
        }

        if (bgrBuffer != nullptr)
        {
            delete bgrBuffer;
            bgrBuffer = nullptr;
        }
    }

    void writeVideoFrame(const unsigned char *video_buffer)
    {
        const cv::Mat videoBufferMat(frameHeight, frameWidth, CV_8UC3, const_cast<unsigned char *>(video_buffer));

        // Copy and Flip image about the x-axis
        cv::flip(videoBufferMat, *bgrBuffer, 1);

        // Convert the video buffer to the HSV color space
        cv::cvtColor(*bgrBuffer, *hsvBuffer, cv::COLOR_BGR2HSV);
    }

    // Return points in CommonDeviceScreenLocation space:
    // i.e. [-frameWidth/2, -frameHeight/2]x[frameWidth/2, frameHeight/2]    
    bool computeBiggestContour(
        const CommonHSVColorRange &hsvColorRange,
        std::vector<cv::Point> &out_biggest_contour)
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
            std::vector<std::vector<cv::Point> > contours;
            cv::findContours(*gsLowerBuffer, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

            if (contours.size() > 0)
            {
                double contArea = 0;
                double newArea = 0;
                for (auto it = contours.begin(); it != contours.end(); ++it) 
                {
                    newArea = cv::contourArea(*it);

                    if (newArea > contArea)
                    {
                        contArea = newArea;
                        out_biggest_contour = *it;
                    }
                }
            }

            //TODO: If our contour is suddenly much smaller than last frame,
            // but is next to an almost-as-big contour, then maybe these
            // 2 contours should be joined.
            // (i.e. if a finger is blocking the middle of the bulb)
            if (out_biggest_contour.size() > 6)
            {
                // Remove any points in contour on edge of camera/ROI
                std::vector<cv::Point>::iterator it = out_biggest_contour.begin();
                while (it != out_biggest_contour.end()) {
                    if (it->x == 0 || it->x == (frameWidth-1) || it->y == 0 || it->y == (frameHeight-1)) 
                    {
                        it = out_biggest_contour.erase(it);
                    }
                    else 
                    {
                        ++it;
                    }
                }
            }
        }

        return (out_biggest_contour.size() > 5);
    }

    int frameWidth;
    int frameHeight;
    cv::Mat *bgrBuffer; // source video frame
    cv::Mat *hsvBuffer; // source frame converted to HSV color space
    cv::Mat *gsLowerBuffer; // HSV image clamped by HSV range into grayscale mask
    cv::Mat *gsUpperBuffer; // HSV image clamped by HSV range into grayscale mask
    cv::Mat *maskedBuffer; // bgr image ANDed together with grayscale mask
};

// -- Utility Methods -----
static glm::mat4 computeGLMCameraTransformMatrix(const ITrackerInterface *tracker_device);
static cv::Matx34f computeOpenCVCameraExtrinsicMatrix(const ITrackerInterface *tracker_device);
static cv::Matx33f computeOpenCVCameraIntrinsicMatrix(const ITrackerInterface *tracker_device);
static cv::Matx34f computeOpenCVCameraPinholeMatrix(const ITrackerInterface *tracker_device);
static bool computeTrackerRelativeShapeContourPose(
    const ITrackerInterface *tracker_device,
    const CommonDeviceTrackingShape *tracking_shape,
    const std::vector<cv::Point> &opencv_contour,
    CommonDevicePose *out_tracker_relative_pose,
    CommonDeviceTrackingProjection *out_projection);
static bool computeBestFitQuadForContour(
    const std::vector<cv::Point> &opencv_contour,
    const cv::Point2f &up_hint, 
    const cv::Point2f &right_hint,
    std::vector<cv::Point2f> &out_best_fit_triangle);
static bool computeBestFitTriangleForContour(
    const std::vector<cv::Point> &opencv_contour,
    std::vector<cv::Point2f> &out_best_fit_triangle);

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
    //case CommonDeviceState::RiftDK2Sensor:
    //    {
    //        //TODO: RiftDK2Sensor tracker location
    //    } break;
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

void ServerTrackerView::gatherTrackingColorPresets(PSMoveProtocol::Response_ResultTrackerSettings* settings) const
{
    return m_device->gatherTrackingColorPresets(settings);
}

void ServerTrackerView::setTrackingColorPreset(eCommonTrackingColorID color, const CommonHSVColorRange *preset)
{
    return m_device->setTrackingColorPreset(color, preset);
}

void ServerTrackerView::getTrackingColorPreset(eCommonTrackingColorID color, CommonHSVColorRange *out_preset) const
{
    return m_device->getTrackingColorPreset(color, out_preset);
}

bool
ServerTrackerView::computePoseForController(
    ServerControllerView* tracked_controller,
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
            getTrackingColorPreset(tracked_color_id, &hsvColorRange);
        }
        else
        {
            bSuccess = false;
        }
    }

    // Find the contour associated with the controller
    std::vector<cv::Point> biggest_contour;
    if (bSuccess)
    {
        ///###HipsterSloth $TODO - ROI seed on last known position, clamp to frame edges.
        // NOTE: 
        // opencv_contour in OpenCV space:
        // i.e. [0, frameHeight]x[frameWidth, 0]    
        // eigen_contour in CommonDeviceScreenLocation space:
        // i.e. [-frameWidth/2, -frameHeight/2]x[frameWidth/2, frameHeight/2]    
        bSuccess = m_opencv_buffer_state->computeBiggestContour(hsvColorRange, biggest_contour);
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
                std::vector<cv::Point> convex_contour;
                cv::convexHull(biggest_contour, convex_contour);

                // Subtract midpoint from each point.
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

                bSuccess = true;
            } break;
        case eCommonTrackingShapeType::LightBar:
            {
                CommonDevicePose tracker_relative_pose;

                if (computeTrackerRelativeShapeContourPose(
                        m_device,
                        &tracking_shape,
                        biggest_contour,
                        &tracker_relative_pose,
                        &out_pose_estimate->projection))
                {
                    out_pose_estimate->orientation = tracker_relative_pose.Orientation;
                    out_pose_estimate->bOrientationValid = true;

                    out_pose_estimate->position = tracker_relative_pose.Position;
                    out_pose_estimate->bCurrentlyTracking = true;

                    bSuccess = true;
                }
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

static bool computeTrackerRelativeShapeContourPose(
    const ITrackerInterface *tracker_device,
    const CommonDeviceTrackingShape *tracking_shape,
    const std::vector<cv::Point> &opencv_contour,
    CommonDevicePose *out_tracker_relative_pose,
    CommonDeviceTrackingProjection *out_projection)
{
    assert(tracking_shape->shape_type == eCommonTrackingShapeType::LightBar);

    // Get the pixel width and height of the tracker image
    int pixelWidth, pixelHeight;
    tracker_device->getVideoFrameDimensions(&pixelWidth, &pixelHeight, nullptr);

    // Create a best fit triangle around the contour
    std::vector<cv::Point2f> cv_best_fit_shape;
    bool bValidTrackerPose= computeBestFitTriangleForContour(opencv_contour, cv_best_fit_shape);

    // Also create a best fit quad to provide a total of 7 points to correlate
    if (bValidTrackerPose)
    {
        // Standard triangle vertex index - right, left, bottom
        const cv::Point2f &tri_right= cv_best_fit_shape[0];
        const cv::Point2f &tri_left= cv_best_fit_shape[1];
        const cv::Point2f &tri_top= cv_best_fit_shape[2];

        // Use the triangle to define 
        const cv::Point2f up_hint= tri_top - 0.5f*(tri_left + tri_right);
        const cv::Point2f right_hint= tri_right - tri_left;

        bValidTrackerPose= computeBestFitQuadForContour(opencv_contour, up_hint, right_hint, cv_best_fit_shape);
    }

    // flip the y coordinates
    std::vector<cv::Point2f> cv_y_flipped_best_fit_shape;
    if (bValidTrackerPose)
    {     
        for (auto list_index = 0; list_index < cv_best_fit_shape.size(); ++list_index)
        {
            const cv::Point2f &cvPoint= cv_best_fit_shape[list_index];
            const cv::Point2f cvYFlippedPoint = { cvPoint.x, pixelHeight - cvPoint.y };

            cv_y_flipped_best_fit_shape.push_back(cvYFlippedPoint);
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

        // Solve the Project N-Point problem:
        // Given a set of 3D points and their corresponding 2D pixel projections,
        // solve for the cameras position and orientation that would allow
        // us to re-project the 3D points back onto the 2D pixel locations
        cv::Mat rvec(4, 1, cv::DataType<double>::type);
        cv::Mat tvec(4, 1, cv::DataType<double>::type);
        if (cv::solvePnP(
                cvObjectPoints, cv_y_flipped_best_fit_shape, 
                cvCameraMatrix, cvDistCoeffs, 
                rvec, tvec, 
                false, cv::SOLVEPNP_ITERATIVE))
        {
            // Return rvec (an angle-axis vector) as a quaternion in the pose
            {
                const float r_x = static_cast<float>(rvec.at<double>(0));
                const float r_y = static_cast<float>(rvec.at<double>(1));
                const float r_z = static_cast<float>(rvec.at<double>(2));
                const float theta = sqrtf(r_x*r_x + r_y*r_y + r_z*r_z);
            
                CommonDeviceQuaternion &orientation = out_tracker_relative_pose->Orientation;

                if (!is_nearly_zero(theta))
                {
                    const float sin_theta_over_two = sinf(theta * 0.5f);

                    orientation.w = cosf(theta * 0.5f);
                    orientation.x = (r_x / theta) * sin_theta_over_two;
                    orientation.y = (r_y / theta) * sin_theta_over_two;
                    orientation.z = (r_z / theta) * sin_theta_over_two;
                }
                else
                {
                    orientation.clear();
                }
            }

            // Return the position in the pose
            {
                CommonDevicePosition &position= out_tracker_relative_pose->Position;

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
        out_projection->shape_type = eCommonTrackingProjectionType::ProjectionType_LightBar;

        for (int vertex_index = 0; vertex_index < 3; ++vertex_index)
        {
            const cv::Point2f &cvPoint = cv_y_flipped_best_fit_shape[vertex_index];

            // Convert the tracker screen locations in OpenCV pixel space
            // i.e. [0, 0]x[frameWidth, frameHeight]
            // into PSMoveScreenLocation space
            // i.e. [-frameWidth/2, -frameHeight/2]x[frameWidth/2, frameHeight/2] 
            out_projection->shape.lightbar.triangle[vertex_index] = 
                { cvPoint.x - (pixelWidth / 2), cvPoint.y - (pixelHeight / 2) };
        }

        for (int vertex_index = 0; vertex_index < 4; ++vertex_index)
        {
            const cv::Point2f &cvPoint = cv_y_flipped_best_fit_shape[vertex_index + 3];

            // Convert the tracker screen locations in OpenCV pixel space
            // i.e. [0, 0]x[frameWidth, frameHeight]
            // into PSMoveScreenLocation space
            // i.e. [-frameWidth/2, -frameHeight/2]x[frameWidth/2, frameHeight/2] 
            out_projection->shape.lightbar.quad[vertex_index] = 
                { cvPoint.x - (pixelWidth / 2), cvPoint.y - (pixelHeight / 2) };
        }
    }

    return bValidTrackerPose;
}

static bool computeBestFitQuadForContour(
    const std::vector<cv::Point> &opencv_contour,
    const cv::Point2f &up_hint, 
    const cv::Point2f &right_hint,
    std::vector<cv::Point2f> &out_best_fit_quad)
{
    // Compute the tightest possible bounding triangle for the given contour
    cv::RotatedRect cv_min_box= cv::minAreaRect(opencv_contour);

    if (cv_min_box.size.width <= k_real_epsilon || cv_min_box.size.height <= k_real_epsilon)
    {
        return false;
    }

    cv::Mat cv_box_points(4, 2, cv::DataType<float>::type);
    cv::boxPoints(cv_min_box, cv_box_points);

    cv::Point2f top_right= cv_box_points.at<cv::Point2f>(3, 0);
    cv::Point2f top_left= cv_box_points.at<cv::Point2f>(0, 0);
    cv::Point2f bottom_left= cv_box_points.at<cv::Point2f>(1, 0);
    cv::Point2f bottom_right= cv_box_points.at<cv::Point2f>(2, 0);

    cv::Point2f box_up= top_left - bottom_left;
    if (box_up.dot(up_hint) < 0)
    {
        // up axis is flipped
        // flip the box vertically
        std::swap(top_right, bottom_right);
        std::swap(top_left, bottom_left);
    }

    cv::Point2f box_right= top_right - top_left;
    if (box_right.dot(right_hint) < 0)
    {
        // right axis is flipped
        // flip the box horizontally
        std::swap(top_right, top_left);
        std::swap(bottom_right, bottom_left);
    }

    // Emit the box in standard order: top_right, top_left, bottom_left, bottom_right
    out_best_fit_quad.push_back(top_right);
    out_best_fit_quad.push_back(top_left);
    out_best_fit_quad.push_back(bottom_left);
    out_best_fit_quad.push_back(bottom_right);

    return true;
}

static bool computeBestFitTriangleForContour(
    const std::vector<cv::Point> &opencv_contour,
    std::vector<cv::Point2f> &out_best_fit_triangle)
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
    {
        const cv::Point2f &top = cv_midpoint_triangle[topCornerIndex];
        const cv::Point2f &left = cv_midpoint_triangle[leftCornerIndex];
        const cv::Point2f &right = cv_midpoint_triangle[rightCornerIndex];
        const cv::Point2f topToLeft = left - top;
        const cv::Point2f topToRight = right - top;

        // Cross product should be positive if sides are correct
        // If not, then swap them.
        if (topToRight.cross(topToLeft) < 0)
        {
            std::swap(leftCornerIndex, rightCornerIndex);
        }
    }

    // Put the triangle corners in the same order as the 
    // world space tracking shape vertices (i.e right, left, top)
    // And flip the y coordinate
    int corner_list[3] = { rightCornerIndex , leftCornerIndex, topCornerIndex };
    for (int list_index = 0; list_index < 3; ++list_index)
    {
        int corner_index = corner_list[list_index];
        const cv::Point2f &cvPoint= cv_midpoint_triangle[corner_index];

        out_best_fit_triangle.push_back(cvPoint);
    }        

    return true;
}
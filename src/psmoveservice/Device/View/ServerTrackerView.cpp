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
    bool computeBiggestConvexContour(
        const CommonHSVColorRange &hsvColorRange, 
        std::vector<Eigen::Vector2f> &out_contour)
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

            std::vector<cv::Point> biggest_contour;
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
                        biggest_contour = *it;
                    }
                }
            }

            //TODO: If our contour is suddenly much smaller than last frame,
            // but is next to an almost-as-big contour, then maybe these
            // 2 contours should be joined.
            // (i.e. if a finger is blocking the middle of the bulb)
            if (biggest_contour.size() > 6)
            {
                // Remove any points in contour on edge of camera/ROI
                std::vector<cv::Point>::iterator it = biggest_contour.begin();
                while (it != biggest_contour.end()) {
                    if (it->x == 0 || it->x == (frameWidth-1) || it->y == 0 || it->y == (frameHeight-1)) 
                    {
                        it = biggest_contour.erase(it);
                    }
                    else 
                    {
                        ++it;
                    }
                }

                // Compute the convex hull of the contour
                std::vector<cv::Point> convex_hull;
                cv::convexHull(biggest_contour, convex_hull);

                // Subtract midpoint from each point.
                // TODO: Replace this with cv::undistortPoints
                //http://docs.opencv.org/3.1.0/da/d54/group__imgproc__transform.html#ga55c716492470bfe86b0ee9bf3a1f0f7e&gsc.tab=0
                std::for_each(
                    convex_hull.begin(),
                    convex_hull.end(),
                    [this, &out_contour](cv::Point& p) {
                        out_contour.push_back(
                            Eigen::Vector2f(p.x - (frameWidth / 2), (frameHeight / 2) - p.y));
                    });
            }
        }

        return (out_contour.size() > 5);
    }

    int frameWidth;
    int frameHeight;
    cv::Mat *bgrBuffer; // source video frame
    cv::Mat *hsvBuffer; // source frame converted to HSV color space
    cv::Mat *gsLowerBuffer; // HSV image clamped by HSV range into grayscale mask
    cv::Mat *gsUpperBuffer; // HSV image clamped by HSV range into grayscale mask
    cv::Mat *maskedBuffer; // bgr image ANDed together with grayscale mask
};

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
    DeviceDataFramePtr &data_frame)
{
    PSMoveProtocol::DeviceDataFrame_TrackerDataPacket *tracker_data_frame =
        data_frame->mutable_tracker_data_packet();

    tracker_data_frame->set_tracker_id(tracker_view->getDeviceID());
    tracker_data_frame->set_sequence_num(tracker_view->m_sequence_number);
    tracker_data_frame->set_isconnected(tracker_view->getDevice()->getIsOpen());

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

    data_frame->set_device_category(PSMoveProtocol::DeviceDataFrame::TRACKER);
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
ServerTrackerView::computePositionForController(
    class ServerControllerView* tracked_controller, 
    CommonDevicePosition* out_position,
    CommonDeviceTrackingProjection *out_projection_shape)
{
    bool bSuccess = m_bHasUnpublishedState;

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
    std::vector<Eigen::Vector2f> convex_contour;
    if (bSuccess)
    {
        ///###HipsterSloth $TODO - ROI seed on last known position, clamp to frame edges.
        // NOTE: convex_contour in CommonDeviceScreenLocation space:
        // i.e. [-frameWidth/2, -frameHeight/2]x[frameWidth/2, frameHeight/2]    
        bSuccess= m_opencv_buffer_state->computeBiggestConvexContour(hsvColorRange, convex_contour);              
    }

    // Compute the tracker relative 3d position of the controller from the contour
    if (bSuccess)
    {
        switch (tracking_shape.shape_type)
        {
        case eCommonTrackingShapeType::Sphere:
            {
                float F_PX, F_PY;
                float PrincipalX, PrincipalY;
                m_device->getCameraIntrinsics(F_PX, F_PY, PrincipalX, PrincipalY);
                
                // TODO: cv::undistortPoints  http://docs.opencv.org/3.1.0/da/d54/group__imgproc__transform.html#ga55c716492470bfe86b0ee9bf3a1f0f7e&gsc.tab=0
                // Then replace F_PX with -1.
                
                if (out_projection_shape != nullptr)
                {
                    // Compute the sphere center AND the projected ellipse
                    Eigen::Vector3f sphere_center;
                    EigenFitEllipse ellipse_projection;
                    eigen_alignment_fit_focal_cone_to_sphere(
                        convex_contour.data(),
                        static_cast<int>(convex_contour.size()),
                        tracking_shape.shape.sphere.radius,
                        F_PX,
                        &sphere_center,
                        &ellipse_projection);

                    out_position->set(sphere_center.x(), sphere_center.y(), sphere_center.z());
                    
                    out_projection_shape->shape_type = eCommonTrackingProjectionType::ProjectionType_Ellipse;
                    out_projection_shape->shape.ellipse.center.set(ellipse_projection.center.x(), ellipse_projection.center.y());
                    out_projection_shape->shape.ellipse.half_x_extent = ellipse_projection.extents.x();
                    out_projection_shape->shape.ellipse.half_y_extent = ellipse_projection.extents.y();
                    out_projection_shape->shape.ellipse.angle = ellipse_projection.angle;
                }
                else
                {
                    // Just compute the sphere center
                    Eigen::Vector3f sphere_center;
                    eigen_alignment_fit_focal_cone_to_sphere(
                        convex_contour.data(),
                        static_cast<int>(convex_contour.size()),
                        tracking_shape.shape.sphere.radius,
                        F_PX,
                        &sphere_center);

                    out_position->set(sphere_center.x(), sphere_center.y(), sphere_center.z());
                }

                bSuccess = true;
            } break;
        case eCommonTrackingShapeType::PlanarBlob:
            {
                //###HipsterSloth $TODO
                bSuccess= false;
            } break;
        default:
            assert(0 && "Unreachable");
            break;
        }
    }

    return bSuccess;
}

static glm::mat4 computeGLMCameraTransformMatrix(ITrackerInterface *tracker_device)
{

    const CommonDevicePose pose = tracker_device->getTrackerPose();
    const CommonDeviceQuaternion &quat = pose.Orientation;
    const CommonDevicePosition &pos = pose.Position;

    const glm::quat glm_quat(quat.w, quat.x, quat.y, quat.z);
    const glm::vec3 glm_pos(pos.x, pos.y, pos.z);
    const glm::mat4 glm_camera_xform = glm_mat4_from_pose(glm_quat, glm_pos);

    return glm_camera_xform;
}

static cv::Matx34f computeOpenCVCameraExtrinsicMatrix(ITrackerInterface *tracker_device)
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

static cv::Matx33f computeOpenCVCameraIntrinsicMatrix(ITrackerInterface *tracker_device)
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

static cv::Matx34f computeOpenCVCameraPinholeMatrix(ITrackerInterface *tracker_device)
{
    cv::Matx34f extrinsic_matrix = computeOpenCVCameraExtrinsicMatrix(tracker_device);
    cv::Matx33f intrinsic_matrix = computeOpenCVCameraIntrinsicMatrix(tracker_device);
    cv::Matx34f pinhole_matrix = intrinsic_matrix * extrinsic_matrix;

    return pinhole_matrix;
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
    cv::Matx33f cvCameraMatrix= computeOpenCVCameraIntrinsicMatrix(m_device);

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
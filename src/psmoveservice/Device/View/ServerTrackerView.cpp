//-- includes -----
#include "ServerTrackerView.h"
#include "DeviceEnumerator.h"
#include "PS3EyeTracker.h"
#include "ServerUtility.h"
#include "ServerLog.h"

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <memory>

//-- private methods -----
struct SharedVideoFrameState
{
    int width;
    int height;
    int stride;
    char buffer[0]; // tail array storing video frame data

    static size_t computeVideoBufferSize(int stride, int height)
    {
        return stride*height;
    }

    static size_t computeTotalSize(int stride, int height)
    {
        return sizeof(SharedVideoFrameState) + computeVideoBufferSize(stride, height);
    }
};

class SharedVideoFrameReadWriteAccessor
{
public:
    SharedVideoFrameReadWriteAccessor(boost::interprocess::shared_memory_object *object)
        : m_region(*object, boost::interprocess::read_write)
    {}

    void initialize(int width, int height, int stride)
    {
        SharedVideoFrameState *frameState = getFrameState();

        std::memset(frameState, 0, m_region.get_size());
        frameState->width = width;
        frameState->height = height;
        frameState->stride = stride;
    }

    void writeVideoFrame(const unsigned char *buffer)
    {
        SharedVideoFrameState *sharedFrameState = getFrameState();
        size_t buffer_size = SharedVideoFrameState::computeTotalSize(sharedFrameState->stride, sharedFrameState->height);

        std::memcpy(sharedFrameState->buffer, buffer, buffer_size);
    }

    SharedVideoFrameState *getFrameState()
    {
        return reinterpret_cast<SharedVideoFrameState *>(m_region.get_address());
    }

private:
    boost::interprocess::mapped_region m_region;
};

//-- public implementation -----
ServerTrackerView::ServerTrackerView(const int device_id)
    : ServerDeviceView(device_id)
    , m_device(nullptr)
    , m_shared_video_frame(nullptr)
    , m_shared_memory_video_stream_count(0)
{
    ServerUtility::format_string(m_shared_memory_name, sizeof(m_shared_memory_name), "tracker_view_%d", device_id);
}

ServerTrackerView::~ServerTrackerView()
{
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
            assert(m_shared_video_frame == nullptr);

            try
            {
                m_shared_video_frame =
                    new boost::interprocess::shared_memory_object(
                        boost::interprocess::create_only,
                        m_shared_memory_name,
                        boost::interprocess::read_write);
                SERVER_LOG_INFO("ServerTrackerView::open()") << "Allocated shared memory: " << m_shared_memory_name;

                m_shared_video_frame->truncate(SharedVideoFrameState::computeTotalSize(stride, width));

                // Initialize the shared memory buffer
                {
                    SharedVideoFrameReadWriteAccessor accessor(m_shared_video_frame);

                    accessor.initialize(width, height, stride);
                }
            }
            catch (boost::interprocess::interprocess_exception* e)
            {
                m_shared_video_frame = nullptr;
                SERVER_LOG_ERROR("ServerTrackerView::open()") << "Failed to allocated shared memory: " << m_shared_memory_name
                    << ", reason: " << e->what();
            }
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
    if (m_shared_video_frame != nullptr)
    {
        if (!boost::interprocess::shared_memory_object::remove(m_shared_memory_name))
        {
            SERVER_LOG_ERROR("ServerTrackerView::close") << "Failed to free shared memory: " << m_shared_memory_name;
        }

        delete m_shared_video_frame;
        m_shared_video_frame = nullptr;
    }
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

    if (bSuccess)
    {
        // Copy the video frame to shared memory (if requested)
        if (m_shared_video_frame != nullptr && m_shared_memory_video_stream_count > 0)
        {
            SharedVideoFrameReadWriteAccessor accessor(m_shared_video_frame);
            const unsigned char *buffer= m_device->getVideoFrameBuffer();

            if (buffer != nullptr)
            {
                accessor.writeVideoFrame(buffer);
            }
        }
    }

    return bSuccess;
}

void ServerTrackerView::updateStateAndPredict()
{
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
    //ServerRequestHandler::get_instance()->publish_tracker_data_frame(
    //    this, &ServerTrackerView::generate_tracker_data_frame_for_stream);
}
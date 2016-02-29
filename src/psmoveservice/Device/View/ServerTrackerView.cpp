//-- includes -----
#include "ServerTrackerView.h"
#include "DeviceEnumerator.h"
#include "PS3EyeTracker.h"
#include "ServerUtility.h"
#include "ServerLog.h"
#include "SharedTrackerState.h"

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <memory>

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

            // Initialize the shared memory
            SharedVideoFrameHeader *frameState = getFrameHeader();
            std::memset(frameState, 0, m_region->get_size());
            frameState->width = width;
            frameState->height = height;
            frameState->stride = stride;
            frameState->frame_index = 0;

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

//-- public implementation -----
ServerTrackerView::ServerTrackerView(const int device_id)
    : ServerDeviceView(device_id)
    , m_shared_memory_accesor(nullptr)
    , m_shared_memory_video_stream_count(0)
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
        if (m_shared_memory_accesor != nullptr && m_shared_memory_video_stream_count > 0)
        {
            const unsigned char *buffer= m_device->getVideoFrameBuffer();

            if (buffer != nullptr)
            {
                m_shared_memory_accesor->writeVideoFrame(buffer);
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
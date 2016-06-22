//-- includes -----
#include "ClientTrackerView.h"
#include "ClientLog.h"
#include "MathGLM.h"
#include "PSMoveProtocol.pb.h"
#include "SharedTrackerState.h"
#include <assert.h>
#include <chrono>
#include <sstream>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <memory>

//-- pre-declarations -----

//-- constants -----

//-- prototypes ----

//-- implementation -----

//-- ClientPSMoveView -----
class SharedVideoFrameReadOnlyAccessor
{
public:
    SharedVideoFrameReadOnlyAccessor()
        : m_shared_memory_object(nullptr)
        , m_region(nullptr)
        , m_bgr_frame_buffer(nullptr)
        , m_frame_width(0)
        , m_frame_height(0)
        , m_frame_stride(0)
        , m_last_frame_index(0)
    {}

    ~SharedVideoFrameReadOnlyAccessor()
    {
        dispose();
    }

    bool initialize(const char *shared_memory_name)
    {
        bool bSuccess = false;

        try
        {
            CLIENT_LOG_INFO("SharedMemory::initialize()") << "Opening shared memory: " << shared_memory_name;

            // Remember the name of the shared memory
            strncpy(m_shared_memory_name, shared_memory_name, sizeof(m_shared_memory_name)-1);
            m_shared_memory_name[sizeof(m_shared_memory_name) - 1] = '\0';

            // Create the shared memory object
            m_shared_memory_object =
                new boost::interprocess::shared_memory_object(
                boost::interprocess::open_only,
                shared_memory_name,
                boost::interprocess::read_write);

            // Map all of the shared memory for read/write access
            m_region = new boost::interprocess::mapped_region(*m_shared_memory_object, boost::interprocess::read_write);

            bSuccess = true;
        }
        catch (boost::interprocess::interprocess_exception &ex)
        {
            dispose();
            CLIENT_LOG_ERROR("SharedMemory::initialize()") << "Failed to allocated shared memory: " << m_shared_memory_name
                << ", reason: " << ex.what();
        }
        catch (std::exception &ex)
        {
            dispose();
            CLIENT_LOG_ERROR("SharedMemory::initialize()") << "Failed to allocated shared memory: " << m_shared_memory_name
                << ", reason: " << ex.what();
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

        if (m_bgr_frame_buffer != nullptr)
        {
            delete[] m_bgr_frame_buffer;
            m_bgr_frame_buffer = nullptr;
        }
    }

    bool readVideoFrame()
    {
        bool bNewFrame = false;
        SharedVideoFrameHeader *sharedFrameState = getFrameHeader();
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(sharedFrameState->mutex);

        // Make sure the target buffer is big enough to read the video frame into
        size_t buffer_size =
            SharedVideoFrameHeader::computeVideoBufferSize(sharedFrameState->stride, sharedFrameState->height);

        // Make sure the shared memory is the size we expect
        size_t total_shared_mem_size =
            SharedVideoFrameHeader::computeTotalSize(sharedFrameState->stride, sharedFrameState->height);
        assert(m_region->get_size() >= total_shared_mem_size);

        // Re-allocate the buffer if any of the video properties changed
        if (m_frame_width != sharedFrameState->width ||
            m_frame_height != sharedFrameState->height ||
            m_frame_stride != sharedFrameState->stride)
        {
            freeVideoBuffer();

            m_frame_width = sharedFrameState->width;
            m_frame_height = sharedFrameState->height;
            m_frame_stride = sharedFrameState->stride;

            allocateVideoBuffer();
        }

        // Copy over the video frame if the frame index changed
        if (m_last_frame_index != sharedFrameState->frame_index)
        {
            if (buffer_size > 0)
            {
                std::memcpy(m_bgr_frame_buffer, sharedFrameState->getBufferMutable(), buffer_size);
            }

            m_last_frame_index = sharedFrameState->frame_index;

            bNewFrame = true;
        }

        return bNewFrame;
    }

    void allocateVideoBuffer()
    {
        size_t buffer_size = SharedVideoFrameHeader::computeVideoBufferSize(m_frame_stride, m_frame_height);

        if (buffer_size > 0)
        {
            // Allocate the buffer to copy the video frame into
            m_bgr_frame_buffer = new unsigned char[buffer_size];
        }
    }

    void freeVideoBuffer()
    {
        // free the video frame buffer
        if (m_bgr_frame_buffer != nullptr)
        {
            delete[] m_bgr_frame_buffer;
            m_bgr_frame_buffer = 0;
        }
    }

    inline const unsigned char *getVideoFrameBuffer() const { return m_bgr_frame_buffer; }
    inline int getVideoFrameWidth() const { return m_frame_width; }
    inline int getVideoFrameHeight() const { return m_frame_height; }
    inline int getVideoFrameStride() const { return m_frame_stride; }
    inline int getLastVideoFrameIndex() const { return m_last_frame_index; }

protected:
    SharedVideoFrameHeader *getFrameHeader()
    {
        return reinterpret_cast<SharedVideoFrameHeader *>(m_region->get_address());
    }

private:
    char m_shared_memory_name[256];
    boost::interprocess::shared_memory_object *m_shared_memory_object;
    boost::interprocess::mapped_region *m_region;
    unsigned char *m_bgr_frame_buffer;
    int m_frame_width, m_frame_height, m_frame_stride;
    int m_last_frame_index;
};

// -- ClientTrackerView ------
ClientTrackerView::ClientTrackerView(const ClientTrackerInfo &trackerInfo)
    : m_tracker_info(trackerInfo)
    , m_shared_memory_accesor(nullptr)
    , m_listener_count(0)
    , m_is_connected(false)
{
    clearTrackerDataFrameState();
}

ClientTrackerView::~ClientTrackerView()
{
    closeVideoStream();
}

void ClientTrackerView::applyTrackerDataFrame(
    const PSMoveProtocol::DeviceOutputDataFrame_TrackerDataPacket *data_frame)
{
    assert(data_frame->tracker_id() == getTrackerId());

    // Compute the data frame receive window statistics if we have received enough samples
    {
        long long now =
            std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()).count();
        long long diff = now - m_data_frame_last_received_time;

        if (diff > 0)
        {
            float seconds = static_cast<float>(diff) / 1000.f;
            float fps = 1.f / seconds;

            data_frame_average_fps = (0.9f)*data_frame_average_fps + (0.1f)*fps;
        }

        m_data_frame_last_received_time = now;
    }

    if (data_frame->sequence_num() > this->m_sequence_num)
    {
        this->m_sequence_num = data_frame->sequence_num();
        this->m_is_connected = data_frame->isconnected();

        switch (data_frame->tracker_type())
        {
        case PSMoveProtocol::PS3EYE:
        {
            //this->ControllerViewType = PSMove;
            //this->ViewState.PSMoveView.ApplyControllerDataFrame(data_frame);
        } break;

        default:
            assert(0 && "Unhandled controller type");
        }
    }
}

void ClientTrackerView::clearTrackerDataFrameState()
{
    m_sequence_num= 0;
    m_is_connected= false;
    m_data_frame_last_received_time= -1;
    data_frame_average_fps= 0.f;
}

bool ClientTrackerView::openVideoStream()
{
    bool bSuccess = false;

    if (m_shared_memory_accesor == nullptr)
    {
        m_shared_memory_accesor = new SharedVideoFrameReadOnlyAccessor();

        if (m_shared_memory_accesor->initialize(m_tracker_info.shared_memory_name))
        {
            bSuccess = m_shared_memory_accesor->readVideoFrame();
        }
    }
    else
    {
        // Already open
        bSuccess = true;
    }

    if (!bSuccess)
    {
        closeVideoStream();
    }

    return bSuccess;
}

bool ClientTrackerView::pollVideoStream()
{
    bool bNewFrame = false;

    if (m_shared_memory_accesor != nullptr)
    {
        bNewFrame= m_shared_memory_accesor->readVideoFrame();
    }

    return bNewFrame;
}

void ClientTrackerView::closeVideoStream()
{
    if (m_shared_memory_accesor != nullptr)
    {
        delete m_shared_memory_accesor;
        m_shared_memory_accesor = nullptr;
    }
}

int ClientTrackerView::getVideoFrameWidth() const
{
    return (m_shared_memory_accesor != nullptr) ? m_shared_memory_accesor->getVideoFrameWidth() : 0;
}

int ClientTrackerView::getVideoFrameHeight() const
{
    return (m_shared_memory_accesor != nullptr) ? m_shared_memory_accesor->getVideoFrameHeight() : 0;
}

int ClientTrackerView::getVideoFrameStride() const
{
    return (m_shared_memory_accesor != nullptr) ? m_shared_memory_accesor->getVideoFrameStride() : 0;
}

const unsigned char *ClientTrackerView::getVideoFrameBuffer() const
{
    return (m_shared_memory_accesor != nullptr) ? m_shared_memory_accesor->getVideoFrameBuffer() : nullptr;
}

PSMoveFrustum ClientTrackerView::getTrackerFrustum() const
{
    PSMoveFrustum frustum;

    frustum.set_pose(m_tracker_info.tracker_pose);

    // Convert the FOV angles to radians for rendering purposes
    frustum.HFOV = m_tracker_info.tracker_hfov * k_degrees_to_radians;
    frustum.VFOV = m_tracker_info.tracker_vfov * k_degrees_to_radians;

    frustum.zNear = m_tracker_info.tracker_znear;
    frustum.zFar = m_tracker_info.tracker_zfar;

    return frustum;
}
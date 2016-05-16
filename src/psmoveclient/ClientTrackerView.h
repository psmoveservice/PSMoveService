#ifndef CLIENT_TRACKER_VIEW_H
#define CLIENT_TRACKER_VIEW_H

//-- includes -----
#include "ClientConfig.h"
#include "ClientGeometry.h"
#include <cassert>

//-- pre-declarations -----
namespace PSMoveProtocol
{
    class DeviceDataFrame;
    class DeviceDataFrame_TrackerDataPacket;
};

//-- constants -----
enum eTrackerType
{
    PS3Eye
};

enum eTrackerDriver
{
    LIBUSB,
    CL_EYE,
    CL_EYE_MULTICAM,
    GENERIC_WEBCAM
};

//-- declarations -----
struct CLIENTPSMOVEAPI ClientTrackerInfo
{
    int tracker_id;
    eTrackerType tracker_type;
    eTrackerDriver tracker_driver;
    char device_path[128];
    char shared_memory_name[64];
};

class CLIENTPSMOVEAPI ClientTrackerView
{
private:
    ClientTrackerInfo m_tracker_info;
    class SharedVideoFrameReadOnlyAccessor *m_shared_memory_accesor;

    int m_listener_count;

    bool m_is_connected;
    int m_sequence_num;
    long long m_data_frame_last_received_time;
    float data_frame_average_fps;

public:
    ClientTrackerView(const ClientTrackerInfo &trackerInfo);
    ~ClientTrackerView();

    // Apply a UDP data packet update to this tracker
    void applyTrackerDataFrame(const PSMoveProtocol::DeviceDataFrame_TrackerDataPacket *data_frame);
    void clearTrackerDataFrameState();

    // Open the shared memory buffer specified in the tracker info
    bool openVideoStream();

    // Extract the next frame from shared memory
    bool pollVideoStream();
    
    // Close the shared memory buffer
    void closeVideoStream();

    // Listener State
    inline void incListenerCount()
    {
        ++m_listener_count;
    }

    inline void decListenerCount()
    {
        assert(m_listener_count > 0);
        --m_listener_count;
    }

    inline int getListenerCount() const
    {
        return m_listener_count;
    }

    // Controller Data Accessors
    inline const ClientTrackerInfo &getTrackerInfo() const
    {
        return m_tracker_info;
    }

    inline int getTrackerId() const
    {
        return m_tracker_info.tracker_id;
    }

    inline int getSequenceNum() const
    {
        return isValid() ? m_sequence_num : -1;
    }

    inline eTrackerType getTrackerType() const
    {
        return m_tracker_info.tracker_type;
    }

    inline eTrackerDriver getTrackerDriver() const
    {
        return m_tracker_info.tracker_driver;
    }

    inline bool isValid() const
    {
        return m_tracker_info.tracker_id != -1;
    }

    inline bool getIsConnected() const
    {
        return (isValid() && m_is_connected);
    }

    int getVideoFrameWidth() const;
    int getVideoFrameHeight() const;
    int getVideoFrameStride() const;
    const unsigned char *getVideoFrameBuffer() const;

    // Statistics
    inline float GetDataFrameFPS() const
    {
        return data_frame_average_fps;
    }
};
#endif // CLIENT_TRACKER_VIEW_H
#ifndef CLIENT_TRACKER_VIEW_H
#define CLIENT_TRACKER_VIEW_H

//-- includes -----
#include "PSMoveClient_export.h"
#include "ClientGeometry.h"
#include <cassert>

//-- pre-declarations -----
namespace PSMoveProtocol
{
    class DeviceOutputDataFrame;
    class DeviceOutputDataFrame_TrackerDataPacket;
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
struct PSM_CPP_PUBLIC_CLASS ClientTrackerInfo
{
    // ID of the tracker in the service
    int tracker_id;

    // Tracker USB properties
    eTrackerType tracker_type;
    eTrackerDriver tracker_driver;
    char device_path[128];

    // Video stream properties
    char shared_memory_name[64];

    // Camera Intrinsic properties
    PSMoveFloatVector2 tracker_focal_lengths; // pixels
    PSMoveFloatVector2 tracker_principal_point; // pixels
    PSMoveFloatVector2 tracker_screen_dimensions; // pixels
    float tracker_hfov; // degrees
    float tracker_vfov; // degrees
    float tracker_znear; // cm
    float tracker_zfar; // cm

    // Camera Extrinsic properties
    PSMovePose tracker_pose;
};

class PSM_CPP_PUBLIC_CLASS ClientTrackerView
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
    void applyTrackerDataFrame(const PSMoveProtocol::DeviceOutputDataFrame_TrackerDataPacket *data_frame);
    void clearTrackerDataFrameState();

    // Used to apply tracker property changes after config tools run
    inline ClientTrackerInfo &getTrackerInfoMutable()
    {
        return m_tracker_info;
    }

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

    inline PSMoveFloatVector2 getTrackerPixelExtents() const
    {
        return m_tracker_info.tracker_screen_dimensions;
    }

    inline PSMoveMatrix3x3 getTrackerIntrinsicMatrix() const
    {
        return PSMoveMatrix3x3::create(
            PSMoveFloatVector3::create(m_tracker_info.tracker_focal_lengths.i, 0.f, m_tracker_info.tracker_principal_point.i),
            PSMoveFloatVector3::create(0.f, m_tracker_info.tracker_focal_lengths.j, m_tracker_info.tracker_principal_point.j),
            PSMoveFloatVector3::create(0.f, 0.f, 1.f));
    }

    inline PSMovePose getTrackerPose() const
    {
        return m_tracker_info.tracker_pose;
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

    PSMoveFrustum getTrackerFrustum() const;

    inline class SharedVideoFrameReadOnlyAccessor *getSharedMemoryAccessor() const 
    {
        return m_shared_memory_accesor;
    }

    // Statistics
    inline float GetDataFrameFPS() const
    {
        return data_frame_average_fps;
    }

    inline long long GetDataFrameLastReceivedTime() const
    {
        return m_data_frame_last_received_time;
    }
};
#endif // CLIENT_TRACKER_VIEW_H
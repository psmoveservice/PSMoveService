#ifndef SERVER_REQUEST_HANDLER_H
#define SERVER_REQUEST_HANDLER_H

// -- includes -----
#include "PSMoveProtocolInterface.h"

// -- pre-declarations -----
class DeviceManager;
namespace boost {
    namespace program_options {
        class variables_map;
}};

// -- definitions -----
struct ControllerStreamInfo
{
    bool include_raw_sensor_data;
    bool include_raw_tracker_data;

    inline void Clear()
    {
        include_raw_sensor_data= false;
        include_raw_tracker_data = false;
    }
};

struct TrackerStreamInfo
{
    bool streaming_video_data;

    inline void Clear()
    {
        streaming_video_data = false;
    }
};

class ServerRequestHandler 
{
public:
    ServerRequestHandler(DeviceManager *deviceManager);
    virtual ~ServerRequestHandler();

    static ServerRequestHandler *get_instance() { return m_instance; }

    bool any_active_bluetooth_requests() const;

    bool startup();
    void update();
    void shutdown();

    ResponsePtr handle_request(int connection_id, RequestPtr request);
    void handle_client_connection_stopped(int connection_id);

    /// When publishing controller data to all listening connections
    /// we need to provide a callback that will fill out a data frame given:
    /// * A \ref ServerControllerView we want to publish to all listening connections
    /// * A \ref ControllerStreamInfo that describes what info the connection wants
    /// This callback will be called for each listening connection
    typedef void (*t_generate_controller_data_frame_for_stream)(
            const class ServerControllerView *controller_view,
            const ControllerStreamInfo *stream_info,
            DeviceDataFramePtr &data_frame);
    void publish_controller_data_frame(
        class ServerControllerView *controller_view, t_generate_controller_data_frame_for_stream callback);

    /// When publishing tracker data to all listening connections
    /// we need to provide a callback that will fill out a data frame given:
    /// * A \ref ServerTrackerView we want to publish to all listening connections
    /// * A \ref TrackerStreamInfo that describes what info the connection wants
    /// This callback will be called for each listening connection
    typedef void(*t_generate_tracker_data_frame_for_stream)(
        const class ServerTrackerView *tracker_view,
        const TrackerStreamInfo *stream_info,
        DeviceDataFramePtr &data_frame);
    void publish_tracker_data_frame(
        class ServerTrackerView *tracker_view, t_generate_tracker_data_frame_for_stream callback);

private:
    // private implementation - same lifetime as the ServerRequestHandler
    class ServerRequestHandlerImpl *m_implementation_ptr;

    // Singleton instance of the class
    // Assigned in startup, cleared in teardown
    static ServerRequestHandler *m_instance;
};

#endif  // SERVER_REQUEST_HANDLER_H

//-- includes -----
#include "ServerTrackerView.h"

//-- private methods -----

//-- public implementation -----
ServerTrackerView::ServerTrackerView(const int device_id)
    : ServerDeviceView(device_id)
    , m_device(nullptr)
{
    //TODO: new PSMoveTracker();
    m_device = nullptr;
}

ServerTrackerView::~ServerTrackerView()
{
    if (m_device != nullptr)
    {
        delete m_device;
    }
}

void ServerTrackerView::publish_device_data_frame()
{
    // Tell the server request handler we want to send out tracker updates.
    // This will call generate_tracker_data_frame_for_stream for each listening connection.
    //ServerRequestHandler::get_instance()->publish_tracker_data_frame(
    //    this, &ServerTrackerView::generate_tracker_data_frame_for_stream);
}
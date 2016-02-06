//-- includes -----
#include "ServerTrackerView.h"

//-- private methods -----

//-- public implementation -----
ServerTrackerView::ServerTrackerView(const int device_id)
    : ServerDeviceView(device_id)
    , m_device(nullptr)
{
    //###bwalker $TODO The device should be allocated in open() based on the enumerator type
    m_device = nullptr;
}

ServerTrackerView::~ServerTrackerView()
{
    if (m_device != nullptr)
    {
        delete m_device;
    }
}

void ServerTrackerView::updateStateAndPredict()
{
}

bool ServerTrackerView::allocate_device_interface(const class DeviceEnumerator *enumerator)
{
    return false;
}

void ServerTrackerView::free_device_interface()
{
}

void ServerTrackerView::publish_device_data_frame()
{
    // Tell the server request handler we want to send out tracker updates.
    // This will call generate_tracker_data_frame_for_stream for each listening connection.
    //ServerRequestHandler::get_instance()->publish_tracker_data_frame(
    //    this, &ServerTrackerView::generate_tracker_data_frame_for_stream);
}
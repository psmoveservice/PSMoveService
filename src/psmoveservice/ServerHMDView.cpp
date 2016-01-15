//-- includes -----
#include "ServerHMDView.h"

//-- private methods -----

//-- public implementation -----
ServerHMDView::ServerHMDView(const int device_id)
    : ServerDeviceView(device_id)
    , m_device(nullptr)
{
    //###bwalker $TODO The device should be allocated in open() based on the enumerator type
    m_device = nullptr;
}

ServerHMDView::~ServerHMDView()
{
    if (m_device != nullptr)
    {
        delete m_device;
    }
}

void ServerHMDView::updateStateAndPredict()
{
}

void ServerHMDView::publish_device_data_frame()
{
    // Tell the server request handler we want to send out HMD updates.
    // This will call generate_hmd_data_frame_for_stream for each listening connection.
    //ServerRequestHandler::get_instance()->publish_hmd_data_frame(
    //    this, &ServerHMDView::generate_hmd_data_frame_for_stream);
}
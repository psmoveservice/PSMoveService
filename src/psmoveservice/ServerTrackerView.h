#ifndef SERVER_TRACKER_VIEW_H
#define SERVER_TRACKER_VIEW_H

//-- includes -----
#include "ServerDeviceView.h"

// -- declarations -----
class ServerTrackerView : public ServerDeviceView
{
public:
    ServerTrackerView(const int device_id);
    ~ServerTrackerView();
    
    IDeviceInterface* getDevice() const override {return m_device;}

protected:
    void publish_device_data_frame() override;

private:
    ITrackerInterface *m_device;
};

#endif // SERVER_TRACKER_VIEW_H
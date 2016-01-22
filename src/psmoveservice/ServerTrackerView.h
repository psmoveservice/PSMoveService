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

    // Get controller colors and update tracking blob positions/predictions
    void updateStateAndPredict() override;

    IDeviceInterface* getDevice() const override {return m_device;}

protected:
    bool allocate_device_interface(const class DeviceEnumerator *enumerator) override;
    void free_device_interface() override;
    void publish_device_data_frame() override;

private:
    ITrackerInterface *m_device;
};

#endif // SERVER_TRACKER_VIEW_H

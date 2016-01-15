#ifndef SERVER_HMD_VIEW_H
#define SERVER_HMD_VIEW_H

//-- includes -----
#include "ServerDeviceView.h"

// -- declarations -----
class ServerHMDView : public ServerDeviceView
{
public:
    ServerHMDView(const int device_id);
    ~ServerHMDView();

    // Get the pose + prediction for HMD
    void updateStateAndPredict() override;

    IDeviceInterface* getDevice() const override {return m_device;}

protected:
    void publish_device_data_frame() override;

private:
    IHMDInterface *m_device;
};

#endif // SERVER_HMD_VIEW_H
//-- includes -----
#include "ServerDeviceView.h"
#include "ServerLog.h"

#include <chrono>

//-- private methods -----

//-- public implementation -----
ServerDeviceView::ServerDeviceView(
    const int device_id)
    : m_bHasUnpublishedState(false)
    , m_last_updated_tick(0)
    , m_sequence_number(0)
    , m_deviceID(device_id)
{
}

ServerDeviceView::~ServerDeviceView()
{
}

bool
ServerDeviceView::open(const DeviceEnumerator *enumerator)
{
    bool bSuccess= getDevice()->open(enumerator);
    
    if (bSuccess)
    {
        // Consider a successful opening as an update
        m_last_updated_tick=
        std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }
    return bSuccess;
}

bool
ServerDeviceView::getIsOpen() const
{
    return getDevice()->getIsOpen();
}

bool ServerDeviceView::poll()
{
    bool bSuccessfullyUpdated= true;
    
    IDeviceInterface* device = getDevice();

    // Only poll data from open, bluetooth controllers
    if (device->getIsReadyToPoll())
    {
        switch (device->poll())
        {
            case IControllerInterface::_PollResultSuccessNoData:
            {
                long long now =
                std::chrono::duration_cast< std::chrono::milliseconds >(
                    std::chrono::system_clock::now().time_since_epoch()).count();
                long diff= static_cast<long>(now - m_last_updated_tick);
                long max_timeout= device->getDataTimeout();
                
                if (diff > max_timeout)
                {
                    SERVER_LOG_INFO("ServerControllerView::poll_open_controllers") <<
                    "Controller id " << getDeviceID() << " closing due to no data timeout (" << max_timeout << "ms)";
                    device->close();
                    
                    bSuccessfullyUpdated= false;
                }
            }
                break;
                
            case IControllerInterface::_PollResultSuccessNewData:
            {
                m_last_updated_tick=
                std::chrono::duration_cast< std::chrono::milliseconds >(
                    std::chrono::system_clock::now().time_since_epoch()).count();
                
                // If we got new sensor data, then we have new state to publish
                markStateAsUnpublished();

                bSuccessfullyUpdated= true;
            }
                break;
                
            case IControllerInterface::_PollResultFailure:
            {
                SERVER_LOG_INFO("ServerControllerView::poll_open_controllers") <<
                "Controller id " << getDeviceID() << " closing due to failed read";
                device->close();
                
                bSuccessfullyUpdated= false;
            }
                break;
        }
    }
    
    return bSuccessfullyUpdated;
}

void ServerDeviceView::publish()
{
    if (m_bHasUnpublishedState)
    {
        publish_device_data_frame();

        m_bHasUnpublishedState= false;
        m_sequence_number++;
    }
}

void
ServerDeviceView::close()
{
    getDevice()->close();
}

bool
ServerDeviceView::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    return getDevice()->matchesDeviceEnumerator(enumerator);
}
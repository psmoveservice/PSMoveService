//-- includes -----
#include "ServerDeviceView.h"
#include "ServerLog.h"

#include <chrono>

//-- private methods -----

//-- public implementation -----
ServerDeviceView::ServerDeviceView(
    const int device_id)
    : m_bHasUnpublishedState(false)
    , m_pollNoDataCount(0)
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
    // Attempt to allocate the device 
    bool bSuccess= allocate_device_interface(enumerator);
    
    // Attempt to open the device
    if (bSuccess)
    {
        bSuccess= getDevice()->open(enumerator);
    }
    
    if (bSuccess)
    {
        // Consider a successful opening as an update
        m_pollNoDataCount= 0;
    }

    return bSuccess;
}

bool
ServerDeviceView::getIsOpen() const
{
    IDeviceInterface* device= getDevice();

    return (device != nullptr) ? device->getIsOpen() : false;
}

bool ServerDeviceView::poll()
{
    bool bSuccessfullyUpdated= true;
    
    IDeviceInterface* device = getDevice();

    // Only poll data from open, bluetooth controllers
    if (device != nullptr && device->getIsReadyToPoll())
    {
        switch (device->poll())
        {
        case IDeviceInterface::_PollResultSuccessNoData:
            {
                long max_failure= device->getMaxPollFailureCount();
                
                ++m_pollNoDataCount;

                if (m_pollNoDataCount > max_failure)
                {
                    SERVER_LOG_INFO("ServerDeviceView::poll") <<
                        "Device id " << getDeviceID() << 
                        " closing due to no data (" << max_failure << 
                        " failed poll attempts)";
                    close();
                    
                    bSuccessfullyUpdated= false;
                }
            }
            break;
                
        case IDeviceInterface::_PollResultSuccessNewData:
            {
                m_pollNoDataCount= 0;
                m_lastNewDataTimestamp= std::chrono::high_resolution_clock::now();

                // If we got new sensor data, then we have new state to publish
                markStateAsUnpublished();

                bSuccessfullyUpdated= true;
            }
            break;
                
        case IDeviceInterface::_PollResultFailure:
            {
                SERVER_LOG_INFO("ServerDeviceView::poll") <<
                    "Device id " << getDeviceID() << " closing due to failed read";
                close();
                
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
    if (getIsOpen())
    {
        getDevice()->close();
        free_device_interface();
    }
}

bool
ServerDeviceView::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    return getIsOpen() && getDevice()->matchesDeviceEnumerator(enumerator);
}
#ifndef SERVER_DEVICE_VIEW_H
#define SERVER_DEVICE_VIEW_H

//-- includes -----
#include "DeviceInterface.h"
#include <assert.h>

// -- declarations -----
class ServerDeviceView
{
    
public:
    ServerDeviceView(const int device_id);
    virtual ~ServerDeviceView();
    
    bool open(const class DeviceEnumerator *enumerator);
    bool poll();
    void publish();
    void close();
    bool matchesDeviceEnumerator(const class DeviceEnumerator *enumerator) const;
    
    // getters
    inline int getDeviceID() const
    { return m_deviceID; }
    
    virtual IDeviceInterface* getDevice() const=0;
    
    // Used for when you have to get device specific data
    template <class t_controller_subclass>
    inline const t_controller_subclass *castCheckedConst() const
    { 
        IDeviceInterface* device= getDevice();
        assert(device->getDeviceType() == t_controller_subclass::getDeviceTypeStatic());
        const t_controller_subclass *controller= static_cast<const t_controller_subclass *>(device);

        return controller; 
    }
    template <class t_controller_subclass>
    inline t_controller_subclass *castChecked()
    {
        IDeviceInterface* device= getDevice();
        assert(device->getDeviceType() == t_controller_subclass::getDeviceTypeStatic());
        t_controller_subclass *controller= static_cast<t_controller_subclass *>(device);

        return controller; 
    }    

    // Returns true if device opened successfully
    bool getIsOpen() const;
    
    // setters
    inline void markStateAsUnpublished()
    { m_bHasUnpublishedState= true; }
    inline void setDeviceID(int id)
    { m_deviceID= id; }
    
protected:
    virtual void publish_device_data_frame() = 0;

    bool m_bHasUnpublishedState;
    long long m_last_updated_tick;
    int m_sequence_number;
    
private:
    int m_deviceID;
};
#endif // SERVER_DEVICE_VIEW_H

#ifndef SERVER_DEVICE_VIEW_H
#define SERVER_DEVICE_VIEW_H

//-- includes -----
#include "DeviceInterface.h"
#include "PSMoveDataFrame.h"
#include "PSMoveProtocolInterface.h"
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

class ServerControllerView : public ServerDeviceView
{
public:
    ServerControllerView(const int device_id);
    ~ServerControllerView();

    // Registers the address of the bluetooth adapter on the host PC with the controller
    bool setHostBluetoothAddress(const std::string &address);
    
    IDeviceInterface* getDevice() const override {return m_device;}

    // Estimate the given pose if the controller
    // Positive time values estimate into the future
    // Negative time values get pose values from the past
    psmovePosef getPose(int msec_time = 0) const;

    // Returns true if the device is connected via Bluetooth, false if by USB
    bool getIsBluetooth() const;

    // Returns the full usb device path for the controller
    std::string getUSBDevicePath() const;

    // Returns the serial number for the controller
    std::string getSerial() const;

    // Gets the host bluetooth address registered with the 
    std::string getHostBluetoothAddress() const;

    // Returns what type of controller this controller view represents
    CommonDeviceState::eDeviceType getControllerDeviceType() const;

    // Fetch the controller state at the given sample index.
    // A lookBack of 0 corresponds to the most recent data.
    void getState(struct CommonControllerState *out_state, int lookBack = 0) const;

    // Set the rumble value between 0-255
    bool setControllerRumble(int rumble_amount);

protected:
    void publish_device_data_frame() override;
    static void generate_controller_data_frame_for_stream(
        const ServerControllerView *controller_view,
        const struct ControllerStreamInfo *stream_info,
        ControllerDataFramePtr &data_frame);

private:
    IControllerInterface *m_device;
};

class ServerTrackerView : public ServerDeviceView
{
public:
    ServerTrackerView(const int device_id);
    ~ServerTrackerView();
    
    IDeviceInterface* getDevice() const override {return m_device;}

protected:
    void publish_device_data_frame() override;

private:
    //TODO: Make ITrackerInterface
    IControllerInterface *m_device;
};

class ServerHMDView : public ServerDeviceView
{
public:
    ServerHMDView(const int device_id);
    ~ServerHMDView();

    IDeviceInterface* getDevice() const override {return m_device;}

protected:
    void publish_device_data_frame() override;

private:
    //TODO: Make IHMDInterface
    IControllerInterface *m_device;
};
#endif // SERVER_DEVICE_VIEW_H

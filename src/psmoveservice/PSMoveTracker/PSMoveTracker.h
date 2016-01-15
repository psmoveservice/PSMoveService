#ifndef PSMOVE_CONTROLLER_H
#define PSMOVE_CONTROLLER_H

#include "PSMoveDataFrame.h"
#include "PSMoveConfig.h"
#include "../DeviceEnumerator.h"
#include "../DeviceInterface.h"
#include <string>
#include <vector>
#include <deque>

class PSMoveTrackerConfig : public PSMoveConfig
{
public:
    PSMoveTrackerConfig(const std::string &fnamebase = "PSMoveTrackerConfig")
    : PSMoveConfig(fnamebase)
    , is_valid(false)
    , data_timeout(1000) // ms
    {};
    
    virtual const boost::property_tree::ptree config2ptree();
    virtual void ptree2config(const boost::property_tree::ptree &pt);
    
    bool is_valid;
    long data_timeout;
};

// https://code.google.com/p/moveonpc/wiki/InputReport
struct PSMoveTrackerState : public CommonDeviceState
{
    // TODO - member variables containing current state.
    
    PSMoveTrackerState()
    {
        clear();
    }
    
    void clear()
    {
        CommonDeviceState::clear();
        DeviceType = Generic_Webcam;
    }
};

class PSMoveTracker : public IDeviceInterface {
public:
    PSMoveTracker();
    PSMoveTracker();
    
    // PSMoveTracker
    bool open(); // Opens the first HID device for the controller
    
    // -- IDeviceInterface
    virtual bool matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const override;
    virtual bool open(const DeviceEnumerator *enumerator) override;
    virtual bool getIsOpen() const override;
    virtual bool getIsReadyToPoll() const override;
    virtual IDeviceInterface::ePollResult poll() override;
    virtual void close() override;
    virtual long getDataTimeout() const override;
    virtual CommonDeviceState::eDeviceType getDeviceType() const override;
    virtual void getState(CommonDeviceState *out_state, int lookBack = 0) const override;
    
    // -- Getters
    inline const PSMoveTrackerConfig &getConfig() const
    { return cfg; }
    
    // -- Setters
    
private:
    PSMoveTrackerConfig cfg;
    
    // Read Controller State
    std::deque<PSMoveTrackerState> TrackerStates;
};
#endif // PSMOVE_CONTROLLER_H
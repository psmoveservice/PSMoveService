#pragma once

//-- included -----
#include <openvr_driver.h>
#include "ClientPSMoveAPI.h"
#include <string>
#include <vector>
#include <chrono>

//-- pre-declarations -----
class CPSMoveTrackedDeviceLatest;

//-- definitions -----
class CServerDriver_PSMoveService : public vr::IServerTrackedDeviceProvider
{
public:
    CServerDriver_PSMoveService();
    virtual ~CServerDriver_PSMoveService();

    // Inherited via IServerTrackedDeviceProvider
    virtual vr::EVRInitError Init( vr::IDriverLog * pDriverLog, vr::IServerDriverHost * pDriverHost, const char * pchUserDriverConfigDir, const char * pchDriverInstallDir ) override;
    virtual void Cleanup() override;
    virtual const char * const *GetInterfaceVersions() override;
    virtual uint32_t GetTrackedDeviceCount() override;
    virtual vr::ITrackedDeviceServerDriver * GetTrackedDeviceDriver( uint32_t unWhich) override;
    virtual vr::ITrackedDeviceServerDriver * FindTrackedDeviceDriver( const char * pchId) override;

    virtual void RunFrame() override;

    virtual bool ShouldBlockStandbyMode() override;
    virtual void EnterStandby() override;
    virtual void LeaveStandby() override;

    void LaunchPSMoveConfigTool();

	void SetHMDTrackingSpace(const PSMovePose &origin_pose);
    inline PSMovePose GetWorldFromDriverPose() const { return m_worldFromDriverPose; }
	bool FindFirstHMDPose(PSMovePose &out_origin_pose);

private:
    void AllocateUniquePSMoveController(int ControllerID);
    void AllocateUniquePSNaviController(int ControllerID);
    void AllocateUniqueDualShock4Controller(int ControllerID);
    void AllocateUniquePSMoveTracker(const ClientTrackerInfo &trackerInfo);
    bool ReconnectToPSMoveService();

    // Event Handling
    void HandleClientPSMoveEvent(const ClientPSMoveAPI::EventMessage *event);
    void HandleConnectedToPSMoveService();
    void HandleFailedToConnectToPSMoveService();
    void HandleDisconnectedFromPSMoveService();
    void HandleControllerListChanged();
    void HandleTrackerListChanged();

    // Response Handling
    void HandleClientPSMoveResponse(const ClientPSMoveAPI::ResponseMessage *response);
    void HandleControllerListReponse(const ClientPSMoveAPI::ResponsePayload_ControllerList *controller_list);
    void HandleTrackerListReponse(const ClientPSMoveAPI::ResponsePayload_TrackerList *tracker_list);
    void HandleHMDTrackingSpaceReponse(const ClientPSMoveAPI::ResponsePayload_HMDTrackingSpace *hmdTrackingSpace);
    
    void LaunchPSMoveConfigTool( const char * pchDriverInstallDir );

    vr::IServerDriverHost* m_pDriverHost;
    std::string m_strDriverInstallDir;

    bool m_bLaunchedPSMoveConfigTool;

    std::vector< CPSMoveTrackedDeviceLatest * > m_vecTrackedDevices;

    // HMD Tracking Space
    PSMovePose m_worldFromDriverPose;
};

class CClientDriver_PSMoveService : public vr::IClientTrackedDeviceProvider
{
public:
    CClientDriver_PSMoveService();
    virtual ~CClientDriver_PSMoveService();

    // Inherited via IClientTrackedDeviceProvider
    virtual vr::EVRInitError Init( vr::EClientDriverMode eDriverMode, vr::IDriverLog * pDriverLog, vr::IClientDriverHost * pDriverHost, const char * pchUserDriverConfigDir, const char * pchDriverInstallDir ) override;
    virtual void Cleanup() override;
    virtual bool BIsHmdPresent( const char * pchUserConfigDir ) override;
    virtual vr::EVRInitError SetDisplayId( const char * pchDisplayId ) override;
    virtual vr::HiddenAreaMesh_t GetHiddenAreaMesh( vr::EVREye eEye ) override;
    virtual uint32_t GetMCImage( uint32_t *pImgWidth, uint32_t *pImgHeight, uint32_t *pChannels, void *pDataBuffer, uint32_t unBufferLen ) override;

private:
    vr::IClientDriverHost* m_pDriverHost;
};

class CPSMoveTrackedDeviceLatest : public vr::ITrackedDeviceServerDriver
{
public:
    CPSMoveTrackedDeviceLatest(vr::IServerDriverHost * pDriverHost);
    virtual ~CPSMoveTrackedDeviceLatest();

    // Shared Implementation of vr::ITrackedDeviceServerDriver
    virtual vr::EVRInitError Activate(uint32_t unObjectId) override;
    virtual void Deactivate() override;
    virtual void PowerOff() override;
    virtual void *GetComponent(const char *pchComponentNameAndVersion) override;
    virtual void DebugRequest(const char * pchRequest, char * pchResponseBuffer, uint32_t unResponseBufferSize) override;
    virtual vr::DriverPose_t GetPose() override;
    virtual bool GetBoolTrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError * pError) override;
    virtual float GetFloatTrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError * pError) override;
    virtual int32_t GetInt32TrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError * pError) override;
    virtual uint64_t GetUint64TrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError * pError) override;
    virtual vr::HmdMatrix34_t GetMatrix34TrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError *pError) override;
    virtual uint32_t GetStringTrackedDeviceProperty(vr::ETrackedDeviceProperty prop, char * pchValue, uint32_t unBufferSize, vr::ETrackedPropertyError * pError) override;

    // CPSMoveTrackedDeviceLatest Interface
    virtual vr::ETrackedDeviceClass GetTrackedDeviceClass() const;
    virtual bool IsActivated() const;
    virtual void Update();
    virtual void RefreshWorldFromDriverPose();
    virtual const char *GetSerialNumber() const;

protected:
    // Handle for calling back into vrserver with events and updates
    vr::IServerDriverHost *m_pDriverHost;
    
    // Tracked device identification
    std::string m_strSerialNumber;

    // Assigned by vrserver upon Activate().  The same ID visible to clients
    uint32_t m_unSteamVRTrackedDeviceId;

    // Flag to denote we should re-publish the controller properties
    bool m_properties_dirty;

    // Cached for answering version queries from vrserver
    vr::DriverPose_t m_Pose;
    unsigned short m_firmware_revision;
    unsigned short m_hardware_revision;
};

class CPSMoveControllerLatest : public CPSMoveTrackedDeviceLatest, public vr::IVRControllerComponent
{
public:
    enum ePSButtonID
    {
        k_EPSButtonID_PS,
        k_EPSButtonID_Left,
        k_EPSButtonID_Up,
        k_EPSButtonID_Right,
        k_EPSButtonID_Down,
        k_EPSButtonID_Move,
        k_EPSButtonID_Trackpad,
        k_EPSButtonID_Trigger,
        k_EPSButtonID_Triangle,
        k_EPSButtonID_Square,
        k_EPSButtonID_Circle,
        k_EPSButtonID_Cross,
        k_EPSButtonID_Select,
        k_EPSButtonID_Share,
        k_EPSButtonID_Start,
        k_EPSButtonID_Options,
        k_EPSButtonID_L1,
        k_EPSButtonID_L2,
        k_EPSButtonID_L3,
        k_EPSButtonID_R1,
        k_EPSButtonID_R2,
        k_EPSButtonID_R3,

        k_EPSButtonID_Count
    };


	enum eVRTouchpadDirection
	{
		k_EVRTouchpadDirection_None,

		k_EVRTouchpadDirection_Left,
		k_EVRTouchpadDirection_Up,
		k_EVRTouchpadDirection_Right,
		k_EVRTouchpadDirection_Down,

		k_EVRTouchpadDirection_Count
	};


    CPSMoveControllerLatest( vr::IServerDriverHost * pDriverHost, int ControllerID );
    virtual ~CPSMoveControllerLatest();

    // Overridden Implementation of vr::ITrackedDeviceServerDriver
    virtual vr::EVRInitError Activate(uint32_t unObjectId) override;
    virtual void Deactivate() override;
    virtual void *GetComponent(const char *pchComponentNameAndVersion) override;
    virtual bool GetBoolTrackedDeviceProperty( vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError * pError ) override;
    virtual float GetFloatTrackedDeviceProperty( vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError * pError ) override;
    virtual int32_t GetInt32TrackedDeviceProperty( vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError * pError ) override;
    virtual uint64_t GetUint64TrackedDeviceProperty( vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError * pError ) override;
    virtual uint32_t GetStringTrackedDeviceProperty( vr::ETrackedDeviceProperty prop, char * pchValue, uint32_t unBufferSize, vr::ETrackedPropertyError * pError ) override;

    // Implementation of vr::IVRControllerComponent
    virtual vr::VRControllerState_t GetControllerState() override;
    virtual bool TriggerHapticPulse( uint32_t unAxisId, uint16_t usPulseDurationMicroseconds ) override;

    // Overridden Implementation of CPSMoveTrackedDeviceLatest
    virtual vr::ETrackedDeviceClass GetTrackedDeviceClass() const override { return vr::TrackedDeviceClass_Controller; }
    virtual void Update() override;

    bool HasControllerId(int ControllerID);

private:
    typedef void ( vr::IServerDriverHost::*ButtonUpdate )( uint32_t unWhichDevice, vr::EVRButtonId eButtonId, double eventTimeOffset );

    void SendButtonUpdates( ButtonUpdate ButtonEvent, uint64_t ulMask );
	void RealignHMDTrackingSpace();
    void UpdateControllerState();
	void UpdateControllerStateFromPsMoveButtonState(ePSButtonID buttonId, PSMoveButtonState buttonState, vr::VRControllerState_t* pControllerStateToUpdate);
    void UpdateTrackingState();
    void UpdateRumbleState();	

    // The last received state of a psmove controller from the service
    int m_nControllerId;
    ClientControllerView *m_controller_view;

    // Used to deduplicate state data from the sixense driver
    int m_nPoseSequenceNumber;

    // To main structures for passing state to vrserver
    vr::VRControllerState_t m_ControllerState;

    // Cached for answering version queries from vrserver
    bool m_bIsBatteryCharging;
    float m_fBatteryChargeFraction;

    // Rumble state
    uint16_t m_pendingHapticPulseDuration;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_lastTimeRumbleSent;
    bool m_lastTimeRumbleSentValid;

    // Button Remapping
    vr::EVRButtonId psButtonIDToVRButtonID[k_EPSButtonID_Count];
	eVRTouchpadDirection psButtonIDToVrTouchpadDirection[k_EPSButtonID_Count];
    void LoadButtonMapping(
        vr::IVRSettings *pSettings,
        const CPSMoveControllerLatest::ePSButtonID psButtonID,
        const vr::EVRButtonId defaultVRButtonID,
		const eVRTouchpadDirection defaultTouchpadDirection);

    // Callbacks
    static void start_controller_response_callback(const ClientPSMoveAPI::ResponseMessage *response, void *userdata);
};

class CPSMoveTrackerLatest : public CPSMoveTrackedDeviceLatest
{
public:
    CPSMoveTrackerLatest(vr::IServerDriverHost * pDriverHost, const ClientTrackerInfo &trackerInfo);
    virtual ~CPSMoveTrackerLatest();

    // Overridden Implementation of vr::ITrackedDeviceServerDriver
    virtual vr::EVRInitError Activate(uint32_t unObjectId) override;
    virtual void Deactivate() override;
    virtual float GetFloatTrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError * pError) override;
    virtual int32_t GetInt32TrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError * pError) override;
    virtual uint32_t GetStringTrackedDeviceProperty(vr::ETrackedDeviceProperty prop, char * pchValue, uint32_t unBufferSize, vr::ETrackedPropertyError * pError) override;

    // Overridden Implementation of CPSMoveTrackedDeviceLatest
    virtual vr::ETrackedDeviceClass GetTrackedDeviceClass() const override { return vr::TrackedDeviceClass_TrackingReference; }
    virtual void Update() override;

    bool HasTrackerId(int ControllerID);
    void SetClientTrackerInfo(const ClientTrackerInfo &trackerInfo);

private:
    // Which tracker
    int m_nTrackerId;

    // The static information about this tracker
    ClientTrackerInfo m_tracker_info;
};
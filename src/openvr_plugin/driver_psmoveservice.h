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

    void LaunchPSMoveMonitor();

	void SetHMDTrackingSpace(const PSMovePose &origin_pose);
    inline PSMovePose GetWorldFromDriverPose() const { return m_worldFromDriverPose; }

private:
    void AllocateUniquePSMoveController(int ControllerID, int ControllerListIndex, const ClientPSMoveAPI::t_response_handle response_handle);
    void AttachPSNaviToParentController(int ControllerID, int ControllerListIndex, const ClientPSMoveAPI::t_response_handle response_handle);
    void AllocateUniqueDualShock4Controller(int ControllerID, int ControllerListIndex, const ClientPSMoveAPI::t_response_handle response_handle);
    void AllocateUniquePSMoveTracker(const ClientTrackerInfo &trackerInfo);
    bool ReconnectToPSMoveService();

    // Event Handling
    void HandleClientPSMoveEvent(const ClientPSMoveAPI::EventMessage *event);
    void HandleConnectedToPSMoveService();
    void HandleFailedToConnectToPSMoveService();
    void HandleDisconnectedFromPSMoveService();
	static void HandleServiceVersionResponse(const ClientPSMoveAPI::ResponseMessage *response, void *userdata);
    void HandleControllerListChanged();
    void HandleTrackerListChanged();

    // Response Handling
    void HandleClientPSMoveResponse(const ClientPSMoveAPI::ResponseMessage *response);
    void HandleControllerListReponse(const ClientPSMoveAPI::ResponsePayload_ControllerList *controller_list, const ClientPSMoveAPI::t_response_handle response_handle);
    void HandleTrackerListReponse(const ClientPSMoveAPI::ResponsePayload_TrackerList *tracker_list);
    
    void LaunchPSMoveMonitor( const char * pchDriverInstallDir );

    vr::IServerDriverHost* m_pDriverHost;
    std::string m_strDriverInstallDir;
	std::string m_strPSMoveHMDSerialNo;

    bool m_bLaunchedPSMoveMonitor;
	bool m_bInitialized;

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
	virtual vr::HiddenAreaMesh_t GetHiddenAreaMesh( vr::EVREye eEye, vr::EHiddenAreaMeshType type ) override;
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
    virtual void EnterStandby() override;
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
    virtual const char *GetSteamVRIdentifier() const;

	typedef void(*t_hmd_request_callback)(const PSMovePose &hmd_pose_meters, void *userdata);
	void RequestLatestHMDPose(float maxPoseAgeMilliseconds = 0.f, t_hmd_request_callback callback = nullptr, void *userdata = nullptr);

protected:
    // Handle for calling back into vrserver with events and updates
    vr::IServerDriverHost *m_pDriverHost;
    
    // Tracked device identification
    std::string m_strSteamVRSerialNo;

    // Assigned by vrserver upon Activate().  The same ID visible to clients
    uint32_t m_unSteamVRTrackedDeviceId;

    // Flag to denote we should re-publish the controller properties
    bool m_properties_dirty;

    // Cached for answering version queries from vrserver
    vr::DriverPose_t m_Pose;
    unsigned short m_firmware_revision;
    unsigned short m_hardware_revision;

	// Cached HMD pose received from the monitor_psmove app
	PSMovePose m_lastHMDPoseInMeters;
	std::chrono::time_point<std::chrono::high_resolution_clock> m_lastHMDPoseTime;
	bool m_bIsLastHMDPoseValid;

	t_hmd_request_callback m_hmdResultCallback;
	void *m_hmdResultUserData;
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


    CPSMoveControllerLatest( vr::IServerDriverHost * pDriverHost, int ControllerID, ClientControllerView::eControllerType controllerType, const char *serialNo );
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
	virtual void RefreshWorldFromDriverPose() override;

	// CPSMoveControllerLatest Interface 
 	inline ClientControllerView::eControllerType getPSMControllerType() const { return m_PSMControllerType; }
    bool HasControllerId(int ControllerID);
	// CPSMoveControllerLatest Interface 
	bool AttachChildPSMController(int ChildControllerId, ClientControllerView::eControllerType controllerType, const std::string &ChildControllerSerialNo);
    inline bool HasPSMControllerId(int ControllerID) const { return ControllerID == m_nPSMControllerId; }
	inline const ClientControllerView * getPSMControllerView() const { return m_PSMControllerView; }
	inline std::string getPSMControllerSerialNo() const { return m_strPSMControllerSerialNo; }
	inline ClientControllerView::eControllerType getPSMControllerType() const { return m_PSMControllerType; }

private:
    typedef void ( vr::IServerDriverHost::*ButtonUpdate )( uint32_t unWhichDevice, vr::EVRButtonId eButtonId, double eventTimeOffset );

    void SendButtonUpdates( ButtonUpdate ButtonEvent, uint64_t ulMask );
	void StartRealignHMDTrackingSpace();
	static void FinishRealignHMDTrackingSpace(const PSMovePose &hmd_pose_meters, void *userdata);
    void UpdateControllerState();
	void UpdateControllerStateFromPsMoveButtonState(ePSButtonID buttonId, PSMoveButtonState buttonState, vr::VRControllerState_t* pControllerStateToUpdate);
	void GetMetersPosInRotSpace(PSMoveFloatVector3* pOutPosition, const PSMoveQuaternion& rRotation );
    void UpdateTrackingState();
    void UpdateRumbleState();	

    // Controller State
    int m_nPSMControllerId;
	ClientControllerView::eControllerType m_PSMControllerType;
    ClientControllerView *m_PSMControllerView;
	std::string m_strPSMControllerSerialNo;

    // Child Controller State
    int m_nPSMChildControllerId;
	ClientControllerView::eControllerType m_PSMChildControllerType;
    ClientControllerView *m_PSMChildControllerView;
	std::string m_strPSMChildControllerSerialNo;

	// Used to report the controllers calibration status
	vr::ETrackingResult m_trackingStatus;

    // Used to ignore old state from PSM Service
    int m_nPoseSequenceNumber;

    // To main structures for passing state to vrserver
    vr::VRControllerState_t m_ControllerState;

    // Cached for answering version queries from vrserver
    bool m_bIsBatteryCharging;
    float m_fBatteryChargeFraction;

    // Rumble state
	bool m_bRumbleSuppressed;
    uint16_t m_pendingHapticPulseDuration;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_lastTimeRumbleSent;
    bool m_lastTimeRumbleSentValid;

	//virtual extend controller in meters
	float m_fVirtuallExtendControllersYMeters;
	float m_fVirtuallExtendControllersZMeters;

	// delay in resetting touchpad position after touchpad press
	bool m_bDelayAfterTouchpadPress;

	// true while the touchpad is considered active (touched or pressed) 
	// after the initial touchpad delay, if any
	bool m_bTouchpadWasActive;

	std::chrono::time_point<std::chrono::high_resolution_clock> m_lastTouchpadPressTime;
	bool m_touchpadDirectionsUsed;

	std::chrono::time_point<std::chrono::high_resolution_clock> m_resetPoseButtonPressTime;
	bool m_bResetPoseRequestSent;

    // Button Remapping
    vr::EVRButtonId psButtonIDToVRButtonID[k_EPSButtonID_Count];
	eVRTouchpadDirection psButtonIDToVrTouchpadDirection[k_EPSButtonID_Count];
    void LoadButtonMapping(
        vr::IVRSettings *pSettings,
		const ClientControllerView::eControllerType controllerType,
        const CPSMoveControllerLatest::ePSButtonID psButtonID,
        const vr::EVRButtonId defaultVRButtonID,
		const eVRTouchpadDirection defaultTouchpadDirection);
	bool LoadBool(vr::IVRSettings *pSettings, const char *pchSection, const char *pchSettingsKey, const bool bDefaultValue);
	float LoadFloat(vr::IVRSettings *pSettings, const char *pchSection, const char *pchSettingsKey, const float fDefaultValue);

	// Settings values. Used to determine whether we'll map controller movement after touchpad
	// presses to touchpad axis values.
	bool m_bUseSpatialOffsetAfterTouchpadPressAsTouchpadAxis;
	float m_fMetersPerTouchpadAxisUnits;

	// Settings value: used to determine how many meters in front of the HMD the controller
	// is held when it's being calibrated.
	float m_fControllerMetersInFrontOfHmdAtCalibration;

	// The position of the controller in meters in driver space relative to its own rotation
	// at the time when the touchpad was most recently pressed (after being up).
	PSMoveFloatVector3 m_posMetersAtTouchpadPressTime;

	// The orientation of the controller in driver space at the time when
	// the touchpad was most recently pressed (after being up).
	PSMoveQuaternion m_driverSpaceRotationAtTouchpadPressTime;
	

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
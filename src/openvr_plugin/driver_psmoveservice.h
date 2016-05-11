#pragma once

//-- included -----
#include <openvr_driver.h>
#include "ClientPSMoveAPI.h"
#include <string>
#include <vector>

//-- pre-declarations -----
class CPSMoveControllerLatest;

//-- definitions -----
class CServerDriver_PSMoveService : public vr::IServerTrackedDeviceProvider
{
public:
	CServerDriver_PSMoveService();
	virtual ~CServerDriver_PSMoveService();

	// Inherited via IServerTrackedDeviceProvider
	virtual vr::EVRInitError Init( vr::IDriverLog * pDriverLog, vr::IServerDriverHost * pDriverHost, const char * pchUserDriverConfigDir, const char * pchDriverInstallDir ) override;
	virtual void Cleanup() override;
	virtual uint32_t GetTrackedDeviceCount() override;
	virtual vr::ITrackedDeviceServerDriver * GetTrackedDeviceDriver( uint32_t unWhich, const char *pchInterfaceVersion ) override;
	virtual vr::ITrackedDeviceServerDriver * FindTrackedDeviceDriver( const char * pchId, const char *pchInterfaceVersion ) override;
	virtual void RunFrame() override;

	virtual bool ShouldBlockStandbyMode() override;
	virtual void EnterStandby() override;
	virtual void LeaveStandby() override;

	void LaunchPSMoveConfigTool();

private:
    void AllocateUniquePSMoveController(int ControllerID, bool bNotifyServer);
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
    
	void CheckForChordedSystemButtons();

	void LaunchPSMoveConfigTool( const char * pchDriverInstallDir );

	vr::IServerDriverHost* m_pDriverHost;
	std::string m_strDriverInstallDir;

	bool m_bLaunchedPSMoveConfigTool;

	std::vector< CPSMoveControllerLatest * > m_vecPSMoveControllers;
};

class CClientDriver_PSMoveService : public vr::IClientTrackedDeviceProvider
{
public:
	CClientDriver_PSMoveService();
	virtual ~CClientDriver_PSMoveService();

	// Inherited via IClientTrackedDeviceProvider
	virtual vr::EVRInitError Init( vr::IDriverLog * pDriverLog, vr::IClientDriverHost * pDriverHost, const char * pchUserDriverConfigDir, const char * pchDriverInstallDir ) override;
	virtual void Cleanup() override;
	virtual bool BIsHmdPresent( const char * pchUserConfigDir ) override;
	virtual vr::EVRInitError SetDisplayId( const char * pchDisplayId ) override;
	virtual vr::HiddenAreaMesh_t GetHiddenAreaMesh( vr::EVREye eEye ) override;
	virtual uint32_t GetMCImage( uint32_t *pImgWidth, uint32_t *pImgHeight, uint32_t *pChannels, void *pDataBuffer, uint32_t unBufferLen ) override;

private:
	vr::IClientDriverHost* m_pDriverHost;

};

class CPSMoveControllerLatest : public vr::ITrackedDeviceServerDriver, public vr::IVRControllerComponent
{
public:
	CPSMoveControllerLatest( vr::IServerDriverHost * pDriverHost, int ControllerID );
	virtual ~CPSMoveControllerLatest();

	// Implementation of vr::ITrackedDeviceServerDriver
	virtual vr::EVRInitError Activate( uint32_t unObjectId ) override;
	virtual void Deactivate() override;
	virtual void PowerOff() override;
	void *GetComponent( const char *pchComponentNameAndVersion ) override;
	virtual void DebugRequest( const char * pchRequest, char * pchResponseBuffer, uint32_t unResponseBufferSize ) override;
	virtual vr::DriverPose_t GetPose() override;
	virtual bool GetBoolTrackedDeviceProperty( vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError * pError ) override;
	virtual float GetFloatTrackedDeviceProperty( vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError * pError ) override;
	virtual int32_t GetInt32TrackedDeviceProperty( vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError * pError ) override;
	virtual uint64_t GetUint64TrackedDeviceProperty( vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError * pError ) override;
	virtual vr::HmdMatrix34_t GetMatrix34TrackedDeviceProperty( vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError *pError ) override;
	virtual uint32_t GetStringTrackedDeviceProperty( vr::ETrackedDeviceProperty prop, char * pchValue, uint32_t unBufferSize, vr::ETrackedPropertyError * pError ) override;

	// Implementation of vr::IVRControllerComponent
	virtual vr::VRControllerState_t GetControllerState() override;
	virtual bool TriggerHapticPulse( uint32_t unAxisId, uint16_t usPulseDurationMicroseconds ) override;

    static const vr::EVRButtonId k_EButton_PS = vr::k_EButton_System;
    static const vr::EVRButtonId k_EButton_Move = vr::k_EButton_ApplicationMenu;
    static const vr::EVRButtonId k_EButton_Select = vr::k_EButton_Grip;
    static const vr::EVRButtonId k_EButton_Start = vr::k_EButton_Grip;
    static const vr::EVRButtonId k_EButton_Trigger = vr::k_EButton_Axis0;
	static const vr::EVRButtonId k_EButton_Triangle = ( vr::EVRButtonId ) 7;
    static const vr::EVRButtonId k_EButton_Circle = (vr::EVRButtonId) 8;
    static const vr::EVRButtonId k_EButton_Square = (vr::EVRButtonId) 9;
    static const vr::EVRButtonId k_EButton_Cross = (vr::EVRButtonId) 10;

	bool IsActivated() const;
	bool HasControllerId( int ControllerID );
	void Update();
	const char *GetSerialNumber();

private:
	static const float k_fScalePSMoveAPIToMeters;

	typedef void ( vr::IServerDriverHost::*ButtonUpdate )( uint32_t unWhichDevice, vr::EVRButtonId eButtonId, double eventTimeOffset );

	void SendButtonUpdates( ButtonUpdate ButtonEvent, uint64_t ulMask );
	void UpdateControllerState();
	void UpdateTrackingState();

	// Handle for calling back into vrserver with events and updates
	vr::IServerDriverHost *m_pDriverHost;

	// Which Hydra controller
	int m_nControllerId;
	std::string m_strSerialNumber;

    // The last received state of a psmove controller from the service
    ClientControllerView *m_controller_view;

	// Used to deduplicate state data from the sixense driver
	int m_nPoseSequenceNumber;

	// To main structures for passing state to vrserver
	vr::DriverPose_t m_Pose;
	vr::VRControllerState_t m_ControllerState;

	// Ancillary tracking state
    PSMoveFloatVector3 m_WorldFromDriverTranslation;
    PSMoveQuaternion m_WorldFromDriverRotation;
    bool m_bCalibrated;

	// Cached for answering version queries from vrserver
	unsigned short m_firmware_revision;
	unsigned short m_hardware_revision;

	// Assigned by vrserver upon Activate().  The same ID visible to clients
	uint32_t m_unSteamVRTrackedDeviceId;

	// The rendermodel used by the device. Check the contents of "c:\Program Files (x86)\Steam\steamapps\common\OpenVR\resources\rendermodels" for available models.
	std::string m_strRenderModel;
};
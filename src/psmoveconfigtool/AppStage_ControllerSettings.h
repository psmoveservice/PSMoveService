#ifndef APP_STAGE_CONTROLLER_SETTINGS_H
#define APP_STAGE_CONTROLLER_SETTINGS_H

//-- includes -----
#include "AppStage.h"
#include "ClientPSMoveAPI.h"

#include <vector>

//-- definitions -----
class AppStage_ControllerSettings : public AppStage
{
public:
    struct ControllerInfo
    {
        int ControllerID;
		int FirmwareVersion;
		int FirmwareRevision;
        ClientControllerView::eControllerType ControllerType;
        PSMoveTrackingColorType TrackingColorType;
        std::string DevicePath;
        std::string DeviceSerial;
        std::string AssignedHostSerial;
        bool PairedToHost;
		bool HasMagnetometer;
		int PositionFilterIndex;
		std::string PositionFilterName;
		int OrientationFilterIndex;
		std::string OrientationFilterName;
		int GyroGainIndex;
		std::string GyroGainSetting;
		float PredictionTime;
    };

    AppStage_ControllerSettings(class App *app);

    inline const ControllerInfo *getSelectedControllerInfo() const
    { 
        return 
            (m_selectedControllerIndex != -1) 
            ? &m_usableControllerInfos[m_selectedControllerIndex] 
            : nullptr; 
    }

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

protected:
    virtual bool onClientAPIEvent(
        ClientPSMoveAPI::eEventType event, 
        ClientPSMoveAPI::t_event_data_handle opaque_event_handle) override;

    void request_controller_list();
    static void handle_controller_list_response(
        const ClientPSMoveAPI::ResponseMessage *response_message,
        void *userdata);
	void request_set_orientation_filter(const int controller_id, const std::string &filter_name);
	void request_set_position_filter(const int controller_id, const std::string &filter_name);
	void request_set_gyroscope_gain_setting(const int controller_id, const std::string& gain_setting);
	void request_set_controller_prediction(const int controller_id, float prediction_time);

	void request_set_controller_tracking_color_id(
		int ControllerID,
		PSMoveTrackingColorType tracking_color_type);

private:
    enum eControllerMenuState
    {
        inactive,
        idle,

        pendingControllerListRequest,
        failedControllerListRequest,
    };
    eControllerMenuState m_menuState;

    std::vector<ControllerInfo> m_usableControllerInfos;
    std::vector<ControllerInfo> m_awaitingPairingControllerInfos;
    std::string m_hostSerial;

    int m_selectedControllerIndex;
};

#endif // APP_STAGE_SELECT_CONTROLLER_H
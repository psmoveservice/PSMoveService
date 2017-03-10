#ifndef APP_STAGE_CONTROLLER_SETTINGS_H
#define APP_STAGE_CONTROLLER_SETTINGS_H

//-- includes -----
#include "AppStage.h"
#include "PSMoveClient_CAPI.h"

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
		int AssignedParentControllerIndex;
		std::string AssignedParentControllerSerial;		
		std::vector<std::string> PotentialParentControllerSerials;
        PSMControllerType ControllerType;
        PSMTrackingColorType TrackingColorType;
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

		static bool ParentControllerComboItemGetter(void* userdata, int index, const char** out_string)
		{
			const ControllerInfo *this_ptr= reinterpret_cast<ControllerInfo *>(userdata);
			const int item_count= static_cast<int>(this_ptr->PotentialParentControllerSerials.size());
			
			if (index >= 0 && index < item_count)
			{
				*out_string= this_ptr->PotentialParentControllerSerials[index].c_str();
				return true;
			}
			else
			{
				*out_string= "<INVALID>";
				return false;
			}
		}
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
        PSMEventMessage::eEventType event, 
        PSMEventDataHandle opaque_event_handle) override;

    void request_controller_list();
    static void handle_controller_list_response(
        const PSMResponseMessage *response_message,
        void *userdata);
	void request_set_orientation_filter(const int controller_id, const std::string &filter_name);
	void request_set_position_filter(const int controller_id, const std::string &filter_name);
	void request_set_gyroscope_gain_setting(const int controller_id, const std::string& gain_setting);
	void request_set_controller_prediction(const int controller_id, float prediction_time);

	int find_controller_id_by_serial(std::string parent_controller_serial) const;

	void request_set_controller_tracking_color_id(
		int ControllerID,
		PSMTrackingColorType tracking_color_type);
	void request_set_parent_controller_id(
		int ControllerID,
		int ParentControllerID);

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
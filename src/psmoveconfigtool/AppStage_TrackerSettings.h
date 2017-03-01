#ifndef APP_STAGE_TRACKER_SETTINGS_H
#define APP_STAGE_TRACKER_SETTINGS_H

//-- includes -----
#include "AppStage.h"
#include "ClientTrackerView.h"

//-- definitions -----
class AppStage_TrackerSettings : public AppStage
{
public:
    struct ControllerInfo
    {
        int ControllerID;
		PSMoveTrackingColorType TrackingColorType;
        ClientControllerView::eControllerType ControllerType;
    };

	struct HMDInfo
	{
		int HmdID;
		PSMoveTrackingColorType TrackingColorType;
		ClientHMDView::eHMDViewType HmdType;
	};

    AppStage_TrackerSettings(class App *app);

    inline const ClientTrackerInfo *getSelectedTrackerInfo() const
    {
        return
            (m_selectedTrackerIndex != -1)
            ? &m_trackerInfos[m_selectedTrackerIndex]
            : nullptr;
    }
	
	inline void set_selectedTrackerIndex(int index) {
		m_selectedTrackerIndex = 
			(index != -1 && index < m_trackerInfos.size())
			? index
			: m_selectedTrackerIndex;
	}

	inline int get_tracker_count() const { return m_trackerInfos.size(); }
	inline int get_tracker_Index() const { return m_selectedTrackerIndex; }

	inline int get_controller_count() const { return m_controllerInfos.size(); }	
	inline const ControllerInfo * get_controller_info(int index) const { return &m_controllerInfos[index]; }
	const ControllerInfo *get_selected_controller();

	const HMDInfo *get_selected_hmd();


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

    void request_tracker_list();
    static void handle_tracker_list_response(
        const ClientPSMoveAPI::ResponseMessage *response,
        void *userdata);

    void request_controller_list();
    static void handle_controller_list_response(
        const ClientPSMoveAPI::ResponseMessage *response_message,
        void *userdata);

	void request_hmd_list();
	static void handle_hmd_list_response(
		const ClientPSMoveAPI::ResponseMessage *response_message,
		void *userdata);

    void request_search_for_new_trackers();
    static void handle_search_for_new_trackers_response(
        const ClientPSMoveAPI::ResponseMessage *response,
        void *userdata);

protected:
    enum eTrackerMenuState
    {
        inactive,
        idle,

        pendingTrackerListRequest,
        failedTrackerListRequest,
		pendingControllerListRequest,
		failedControllerListRequest,
		pendingHmdListRequest,
		failedHmdListRequest,
        pendingSearchForNewTrackersRequest,
    };
    eTrackerMenuState m_menuState;

    std::vector<ClientTrackerInfo> m_trackerInfos;
	std::vector<ControllerInfo> m_controllerInfos;
	std::vector<HMDInfo> m_hmdInfos;

    int m_selectedTrackerIndex;
	int m_selectedControllerIndex;
	int m_selectedHmdIndex;
};

#endif // APP_STAGE_TRACKER_SETTINGS_H
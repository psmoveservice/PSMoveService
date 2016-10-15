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

    AppStage_TrackerSettings(class App *app);

    inline const ClientTrackerInfo *getSelectedTrackerInfo() const
    {
        return
            (m_selectedTrackerIndex != -1)
            ? &m_trackerInfos[m_selectedTrackerIndex]
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

    void request_tracker_list();
    static void handle_tracker_list_response(
        const ClientPSMoveAPI::ResponseMessage *response,
        void *userdata);

    void request_controller_list();
    static void handle_controller_list_response(
        const ClientPSMoveAPI::ResponseMessage *response_message,
        void *userdata);

    void request_search_for_new_trackers();
    static void handle_search_for_new_trackers_response(
        const ClientPSMoveAPI::ResponseMessage *response,
        void *userdata);

	const ControllerInfo *get_selected_controller();

protected:
    enum eTrackerMenuState
    {
        inactive,
        idle,

        pendingTrackerListRequest,
        failedTrackerListRequest,
		pendingControllerListRequest,
		failedControllerListRequest,
        pendingSearchForNewTrackersRequest,
    };
    eTrackerMenuState m_menuState;

    std::vector<ClientTrackerInfo> m_trackerInfos;
	std::vector<ControllerInfo> m_controllerInfos;

    int m_selectedTrackerIndex;
	int m_selectedControllerIndex;
};

#endif // APP_STAGE_TRACKER_SETTINGS_H
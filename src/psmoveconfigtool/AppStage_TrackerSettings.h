#ifndef APP_STAGE_TRACKER_SETTINGS_H
#define APP_STAGE_TRACKER_SETTINGS_H

//-- includes -----
#include "AppStage.h"

//-- definitions -----
class AppStage_TrackerSettings : public AppStage
{
public:
    enum eTrackerType
    {
        PS3Eye
    };

    enum eTrackerDriver
    {
        LIBUSB,
        CL_EYE,
        CL_EYE_MULTICAM,
        GENERIC_WEBCAM
    };

    struct TrackerInfo
    {
        int TrackerID;
        eTrackerType TrackerType;
        eTrackerDriver TrackerDriver;
        std::string DevicePath;
        std::string SharedMemoryName;
    };

    AppStage_TrackerSettings(class App *app);

    inline const TrackerInfo *getSelectedTrackerInfo() const
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
        ClientPSMoveAPI::eClientPSMoveAPIEvent event,
        ClientPSMoveAPI::t_event_data_handle opaque_event_handle) override;

    void request_tracker_list();
    static void handle_tracker_list_response(
        ClientPSMoveAPI::eClientPSMoveResultCode ResultCode,
        const ClientPSMoveAPI::t_request_id request_id,
        ClientPSMoveAPI::t_response_handle response_handle,
        void *userdata);

protected:
    enum eTrackerMenuState
    {
        inactive,
        idle,

        pendingTrackerListRequest,
        failedTrackerListRequest,
    };
    eTrackerMenuState m_menuState;

    std::vector<TrackerInfo> m_trackerInfos;

    int m_selectedTrackerIndex;
};

#endif // APP_STAGE_TRACKER_SETTINGS_H
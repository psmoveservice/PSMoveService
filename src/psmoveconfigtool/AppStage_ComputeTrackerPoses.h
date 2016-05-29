#ifndef APP_STAGE_COMPUTE_TRACKER_POSES_H
#define APP_STAGE_COMPUTE_TRACKER_POSES_H

//-- includes -----
#include "AppStage.h"
#include "ClientPSMoveAPI.h"

#include <map>

//-- definitions -----
class AppStage_ComputeTrackerPoses : public AppStage
{
public:
    AppStage_ComputeTrackerPoses(class App *app);
    ~AppStage_ComputeTrackerPoses();

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

protected:
    enum eMenuState
    {
        inactive,

        pendingControllerListRequest,
        failedControllerListRequest,

        pendingControllerStartRequest,
        failedControllerStartRequest,

        pendingTrackerListRequest,
        failedTrackerListRequest,

        pendingTrackerStartRequest,
        failedTrackerStartRequest,

        verifyHMD,
        verifyTrackers,

        selectCalibrationType,

        calibrateWithHMD,
        calibrateWithMat,

        testTracking,
        calibrateStepFailed,
    };

    void setState(eMenuState newState);
    void onExitState(eMenuState newState);
    void onEnterState(eMenuState newState);

    void update_tracker_video();
    void render_tracker_video();
    void go_next_tracker();
    void go_previous_tracker();
    int get_tracker_count() const;
    int get_render_tracker_index() const;

    void request_controller_list();
    static void handle_controller_list_response(
        const ClientPSMoveAPI::ResponseMessage *response_message,
        void *userdata);

    void request_start_controller_stream(int ControllerID);
    static void handle_start_controller_response(
        const ClientPSMoveAPI::ResponseMessage *response_message,
        void *userdata);

    void request_tracker_list();
    static void handle_tracker_list_response(
        const ClientPSMoveAPI::ResponseMessage *response_message,
        void *userdata);

    void request_tracker_start_stream(const struct ClientTrackerInfo *TrackerInfo, int listIndex);
    static void handle_tracker_start_stream_response(
        const ClientPSMoveAPI::ResponseMessage *response,
        void *userdata);

    void request_set_tracker_pose(
        const struct PSMovePose *pose, 
        class ClientTrackerView *TrackerView);

    void request_set_hmd_tracking_space_origin(
        const struct PSMovePose *pose);

    void handle_all_devices_ready();

    void release_devices();
    void request_exit_to_app_stage(const char *app_stage_name);

private:
    eMenuState m_menuState;

    struct TrackerState
    {
        int listIndex;
        class ClientTrackerView *trackerView;
        class TextureAsset *textureAsset;
    };
    typedef std::map<int, TrackerState> t_tracker_state_map;
    typedef std::map<int, TrackerState>::iterator t_tracker_state_map_iterator;
    typedef std::pair<int, TrackerState> t_id_tracker_state_pair;
    
    class ClientHMDView *m_hmdView;
    class ClientControllerView *m_controllerView;
    t_tracker_state_map m_trackerViews;
    int m_pendingTrackerStartCount;

    int m_renderTrackerIndex;
    t_tracker_state_map_iterator m_renderTrackerIter;

    class AppSubStage_CalibrateWithHMD *m_pCalibrateWithHMD;
    friend class AppSubStage_CalibrateWithHMD;

    class AppSubStage_CalibrateWithMat *m_pCalibrateWithMat;
    friend class AppSubStage_CalibrateWithMat;
};

#endif // APP_STAGE_COMPUTE_TRACKER_POSES_H
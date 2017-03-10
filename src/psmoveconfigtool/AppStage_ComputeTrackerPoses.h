#ifndef APP_STAGE_COMPUTE_TRACKER_POSES_H
#define APP_STAGE_COMPUTE_TRACKER_POSES_H

//-- includes -----
#include "AppStage.h"
#include "PSMoveClient_CAPI.h"

#include <map>

//-- definitions -----
class AppStage_ComputeTrackerPoses : public AppStage
{
public:
    struct TrackerState
    {
        int listIndex;
        PSMTracker *trackerView;
        class TextureAsset *textureAsset;
    };
    typedef std::map<int, TrackerState> t_tracker_state_map;
    typedef std::map<int, TrackerState>::iterator t_tracker_state_map_iterator;
	typedef std::map<int, TrackerState>::const_iterator t_tracker_state_map_iterator_const;
    typedef std::pair<int, TrackerState> t_id_tracker_state_pair;

	struct ControllerState
	{
		int listIndex;
		PSMTrackingColorType trackingColorType;
		PSMController *controllerView;
	};
	typedef std::map<int, ControllerState> t_controller_state_map;
	typedef std::map<int, ControllerState>::iterator t_controller_state_map_iterator;
	typedef std::pair<int, ControllerState> t_id_controller_state_pair;

    AppStage_ComputeTrackerPoses(class App *app);
    ~AppStage_ComputeTrackerPoses();

    static void enterStageAndCalibrate(class App *app, int reqeusted_controller_id);
    static void enterStageAndSkipCalibration(class App *app, int reqeusted_controller_id);

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

        verifyTrackers,
		selectCalibrationMethod,
        calibrateWithMat,
		stereoCalibrate,

        testTracking,
		showTrackerVideo,
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
    PSMTracker *get_render_tracker_view() const;
	PSMController *get_calibration_controller_view() const;

    void request_controller_list();
    static void handle_controller_list_response(
        const PSMResponseMessage *response_message,
        void *userdata);

    void request_start_controller_stream(int ControllerID, int listIndex, PSMTrackingColorType trackingColorType);
    static void handle_start_controller_response(
        const PSMResponseMessage *response_message,
        void *userdata);

    void request_tracker_list();
    static void handle_tracker_list_response(
        const PSMResponseMessage *response_message,
        void *userdata);

    void request_tracker_start_stream(const PSMClientTrackerInfo *TrackerInfo, int listIndex);
    static void handle_tracker_start_stream_response(
        const PSMResponseMessage *response,
        void *userdata);

    void request_set_tracker_pose(
        const PSMPosef *pose, 
        PSMTracker *TrackerView);

    void handle_all_devices_ready();
	bool does_tracker_see_any_controller(const PSMTracker *trackerView);

    void release_devices();
    void request_exit_to_app_stage(const char *app_stage_name);

protected:
    eMenuState m_menuState;

    t_tracker_state_map m_trackerViews;
    int m_pendingTrackerStartCount;

	t_controller_state_map m_controllerViews;
	int m_pendingControllerStartCount;

    int m_renderTrackerIndex;
    t_tracker_state_map_iterator m_renderTrackerIter;

    class AppSubStage_CalibrateWithHMD *m_pCalibrateWithHMD;
    friend class AppSubStage_CalibrateWithHMD;

    class AppSubStage_CalibrateWithMat *m_pCalibrateWithMat;
    friend class AppSubStage_CalibrateWithMat;

    class AppSubStage_StereoCalibrate *m_pStereoCalibrate;
    friend class AppSubStage_StereoCalibrate;

    bool m_bSkipCalibration;
	int m_overrideControllerId;
};

#endif // APP_STAGE_COMPUTE_TRACKER_POSES_H
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

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

protected:
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

    void request_tracker_start_stream(const struct ClientTrackerInfo *TrackerInfo);
    static void handle_tracker_start_stream_response(
        const ClientPSMoveAPI::ResponseMessage *response,
        void *userdata);

    void request_exit_to_app_stage(const char *app_stage_name);

private:
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

        selectCalibrationType,

        // Calibration Mat Steps
        calibrationStepOriginPlacement,
        calibrationStepPlacePSMove,
        calibrationStepRecordPSMove,
        calibrationStepPlaceHMD,
        calibrationStepRecordHMD,

        // HMD Co-registration steps
        calibrationStepAttachPSMove,
        calibrationStepRecordHmdPSMove,

        calibrateStepComplete,
        calibrateStepFailed,
    };

    eMenuState m_menuState;

    struct TrackerState
    {
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
};

#endif // APP_STAGE_COMPUTE_TRACKER_POSES_H
#ifndef APP_STAGE_MAGNETOMETER_CALIBRATION_H
#define APP_STAGE_MAGNETOMETER_CALIBRATION_H

//-- includes -----
#include "AppStage.h"
#include "ClientGeometry.h"

#include <deque>

//-- definitions -----
class AppStage_MagnetometerCalibration : public AppStage
{
public:
    static const int k_max_magnetometer_samples= 500;


    AppStage_MagnetometerCalibration(class App *app);

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

protected:
    static void handle_acquire_controller(
        ClientPSMoveAPI::eClientPSMoveResultCode resultCode,
        const ClientPSMoveAPI::t_request_id request_id, 
        ClientPSMoveAPI::t_response_handle opaque_response_handle,
        void *userdata);
    void request_exit_to_app_stage(const char *app_stage_name);
    static void handle_release_controller(
        ClientPSMoveAPI::eClientPSMoveResultCode resultCode,
        const ClientPSMoveAPI::t_request_id request_id, 
        ClientPSMoveAPI::t_response_handle opaque_response_handle,
        void *userdata);

private:
    enum eCalibrationMenuState
    {
        inactive,
        
        waitingForStreamStartResponse,
        failedStreamStart,
        failedBadCalibration,
        measureBExtents,
        waitForGravityAlignment,
        measureBDirection,
        complete,
        pendingExit
    };
    eCalibrationMenuState m_menuState;
    const char *m_pendingAppStage;

    ClientControllerView *m_controllerView;
    bool m_isControllerStreamActive;
    int m_lastControllerSeqNum;

    PSMoveIntVector3 m_lastMagnetometer;
    PSMoveFloatVector3 m_lastAccelerometer;

    std::deque<PSMoveIntVector3> m_magnetometerIntSamples;
    PSMoveIntVector3 m_minSampleExtents;
    PSMoveIntVector3 m_maxSampleExtents;
    PSMoveFloatVector3 m_magnetometerScaledSamples[k_max_magnetometer_samples];
    PSMoveFloatVector3 m_magnetometerScaleRange;

    int m_led_color_r;
    int m_led_color_g;
    int m_led_color_b;
};

#endif // APP_STAGE_SELECT_CONTROLLER_H
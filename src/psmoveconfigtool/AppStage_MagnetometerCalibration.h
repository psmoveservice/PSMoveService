#ifndef APP_STAGE_MAGNETOMETER_CALIBRATION_H
#define APP_STAGE_MAGNETOMETER_CALIBRATION_H

//-- includes -----
#include "AppStage.h"
#include "ClientGeometry.h"

#include <deque>
#include <chrono>

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

    inline void setBypassCalibrationFlag(bool bFlag)
    { m_bBypassCalibration= bFlag; }

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
    static void handle_set_magnetometer_calibration(
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
        waitForSetCalibrationResponse,
        failedSetCalibration,
        complete,
        pendingExit
    };
    bool m_bBypassCalibration;
    eCalibrationMenuState m_menuState;
    const char *m_pendingAppStage;

    ClientControllerView *m_controllerView;
    bool m_isControllerStreamActive;
    int m_lastControllerSeqNum;

    PSMoveIntVector3 m_lastMagnetometer;
    PSMoveFloatVector3 m_lastAccelerometer;

    std::deque<PSMoveIntVector3> m_magnetometerIntSamples;
    PSMoveIntVector3 m_minSampleExtent;
    PSMoveIntVector3 m_maxSampleExtent;
    PSMoveFloatVector3 m_magnetometerNormalizedSamples[k_max_magnetometer_samples];
    PSMoveFloatVector3 m_minSampleExtentNormalized, m_maxSampleExtentNormalized;
    PSMoveFloatVector3 m_lastMagnetometerNormalized;

    int m_led_color_r;
    int m_led_color_g;
    int m_led_color_b;

    std::chrono::system_clock::time_point m_stableStartTime;
    bool m_bIsStable;

    PSMoveFloatVector3 m_identityPoseAverageMVector;
    int m_identityPoseSampleCount;
};

#endif // APP_STAGE_SELECT_CONTROLLER_H
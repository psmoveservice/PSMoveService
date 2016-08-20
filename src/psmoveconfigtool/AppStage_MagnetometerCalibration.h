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
    AppStage_MagnetometerCalibration(class App *app);
    virtual ~AppStage_MagnetometerCalibration();

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
        const ClientPSMoveAPI::ResponseMessage *response,
        void *userdata);
    void request_exit_to_app_stage(const char *app_stage_name);
    static void handle_release_controller(
        const ClientPSMoveAPI::ResponseMessage *response,
        void *userdata);
    static void handle_set_magnetometer_calibration(
        const ClientPSMoveAPI::ResponseMessage *response,
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

    PSMoveIntVector3 m_lastRawMagnetometer;
    PSMoveFloatVector3 m_lastCalibratedAccelerometer;

    struct MagnetometerBoundsStatistics *m_boundsStatistics;
	struct MagnetometerIdentityStatistics *m_identityStatistics;

    int m_led_color_r;
    int m_led_color_g;
    int m_led_color_b;

    std::chrono::time_point<std::chrono::high_resolution_clock> m_stableStartTime;
    bool m_bIsStable;
};

#endif // APP_STAGE_SELECT_CONTROLLER_H
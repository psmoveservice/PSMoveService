#ifndef APP_STAGE_MAGNETOMETER_CALIBRATION_H
#define APP_STAGE_MAGNETOMETER_CALIBRATION_H

//-- includes -----
#include "AppStage.h"

#include <vector>

//-- definitions -----
class AppStage_MagnetometerCalibration : public AppStage
{
public:
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
};

#endif // APP_STAGE_SELECT_CONTROLLER_H
#ifndef APP_STAGE_HMD_ACCELEROMETER_CALIBRATION_H
#define APP_STAGE_HMD_ACCELEROMETER_CALIBRATION_H

//-- includes -----
#include "AppStage.h"
#include "ClientGeometry_CAPI.h"
#include "PSMoveClient_CAPI.h"

#include <deque>
#include <chrono>

//-- definitions -----
class AppStage_HMDAccelerometerCalibration : public AppStage
{
public:
	AppStage_HMDAccelerometerCalibration(class App *app);
    virtual ~AppStage_HMDAccelerometerCalibration();

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

    inline void setBypassCalibrationFlag(bool bFlag)
    {
        m_bBypassCalibration = bFlag;
    }

protected:
    static void handle_acquire_hmd(
        const PSMResponseMessage *response,
        void *userdata);
    void request_exit_to_app_stage(const char *app_stage_name);

private:
    enum eCalibrationMenuState
    {
        inactive,

        waitingForStreamStartResponse,
        failedStreamStart,
        placeHMD,
        measureNoise,
        measureComplete,
        test
    };
    eCalibrationMenuState m_menuState;

    bool m_bBypassCalibration;

    PSMHeadMountedDisplay *m_hmdView;
    bool m_isHMDStreamActive;
    int m_lastHMDSeqNum;

    PSMVector3i m_lastRawAccelerometer;
    PSMVector3f m_lastCalibratedAccelerometer;

    struct HMDAccelerometerPoseSamples *m_noiseSamples;
};

#endif // APP_STAGE_HMD_ACCELEROMETER_CALIBRATION_H
#ifndef APP_STAGE_ACCELEROMETER_CALIBRATION_H
#define APP_STAGE_ACCELEROMETER_CALIBRATION_H

//-- includes -----
#include "AppStage.h"
#include "ClientGeometry.h"
#include "ClientPSMoveAPI.h"

#include <deque>
#include <chrono>

//-- pre-declarations -----
struct AccelerometerPoseSamples;
struct EigenFitEllipsoid;

//-- definitions -----
class AppStage_AccelerometerCalibration : public AppStage
{
public:
    AppStage_AccelerometerCalibration(class App *app);
    virtual ~AppStage_AccelerometerCalibration();

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

    enum eMeasurementPose
    {
        identity= -1,

        faceUp= 0,
        faceDown,
        leanLeft,
        leanRight,
        leanForward,
        leanBackward,

        k_measurement_pose_count
    };

protected:
    void request_set_accelerometer_calibration(const EigenFitEllipsoid *ellipsoid);
    static void handle_acquire_controller(
        const ClientPSMoveAPI::ResponseMessage *response,
        void *userdata);
    void request_exit_to_app_stage(const char *app_stage_name);

private:
    enum eCalibrationMenuState
    {
        inactive,

        waitingForStreamStartResponse,
        failedStreamStart,
        placeController,
        measureDirection,
        measureComplete,
        verifyCalibration,
        test
    };
    eCalibrationMenuState m_menuState;
    bool m_bBypassCalibration;

    class ClientControllerView *m_controllerView;
    bool m_isControllerStreamActive;
    int m_lastControllerSeqNum;

    PSMoveIntVector3 m_minSampleExtent;
    PSMoveIntVector3 m_maxSampleExtent;
    PSMoveIntVector3 m_lastRawAccelerometer;
    PSMoveFloatVector3 m_lastCalibratedAccelerometer;

    AccelerometerPoseSamples *m_poseSamples;

    eMeasurementPose m_currentPoseID;

    EigenFitEllipsoid *m_sampleFitEllipsoid;
};

#endif // APP_STAGE_ACCELEROMETER_CALIBRATION_H
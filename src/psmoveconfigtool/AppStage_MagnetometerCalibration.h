#ifndef APP_STAGE_MAGNETOMETER_CALIBRATION_H
#define APP_STAGE_MAGNETOMETER_CALIBRATION_H

//-- includes -----
#include "AppStage.h"
#include "ClientGeometry.h"
#include "MathEigen.h"
#include "MathAlignment.h"

#include <deque>
#include <chrono>

//-- constants -----
enum eEllipseFitMethod
{
    _ellipse_fit_method_box,
    _ellipse_fit_method_min_volume,
};

static const int k_max_magnetometer_samples = 500;

//-- definitions -----
struct MagnetometerAlignedSamples
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3f magnetometerEigenSamples[k_max_magnetometer_samples];
};

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

    PSMoveIntVector3 m_magnetometerIntSamples[k_max_magnetometer_samples];
    MagnetometerAlignedSamples *m_alignedSamples;
    int m_sampleCount;
    int m_samplePercentage;

    PSMoveIntVector3 m_minSampleExtent;
    PSMoveIntVector3 m_maxSampleExtent;

    EigenFitEllipsoid m_sampleFitEllipsoid;
    int m_ellipseFitMethod;

    int m_led_color_r;
    int m_led_color_g;
    int m_led_color_b;

    std::chrono::time_point<std::chrono::high_resolution_clock> m_stableStartTime;
    bool m_bIsStable;

	std::chrono::time_point<std::chrono::high_resolution_clock> m_resetPoseButtonPressTime;
	bool m_bResetPoseRequestSent;

    PSMoveIntVector3 m_identityPoseMVectorSum;
    int m_identityPoseSampleCount;   
};

#endif // APP_STAGE_SELECT_CONTROLLER_H
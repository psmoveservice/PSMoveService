#ifndef APP_STAGE_COREGISTER_WITH_MAT_H
#define APP_STAGE_COREGISTER_WITH_MAT_H

//-- includes -----

//-- definitions -----
class AppSubStage_CalibrateWithMat
{
public:
    AppSubStage_CalibrateWithMat(class AppStage_ComputeTrackerPoses *parentStage);

    void enter();
    void exit();
    void update();
    void render();

    void renderUI();

protected:
    enum eMenuState
    {
        inactive,

        calibrationStepOriginPlacement,
        calibrationStepPlacePSMove,
        calibrationStepRecordPSMove,
        calibrationStepPlaceHMD,
        calibrationStepRecordHMD,

        calibrateStepComplete,
        calibrateStepFailed,
    };

    void setState(eMenuState newState);
    void onExitState(eMenuState newState);
    void onEnterState(eMenuState newState);

private:
    class AppStage_ComputeTrackerPoses *m_parentStage;
    eMenuState m_menuState;
};

#endif // APP_STAGE_COREGISTER_WITH_MAT_H
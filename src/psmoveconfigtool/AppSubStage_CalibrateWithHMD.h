#ifndef APP_STAGE_COREGISTER_WITH_HMD_H
#define APP_STAGE_COREGISTER_WITH_HMD_H

//-- includes -----

//-- definitions -----
class AppSubStage_CalibrateWithHMD
{
public:
    AppSubStage_CalibrateWithHMD(class AppStage_ComputeTrackerPoses *parentStage);

    void enter();
    void exit();
    void update();
    void render();
    void renderUI();

protected:
    enum eMenuState
    {
        inactive,

        calibrationStepAttachPSMove,
        calibrationStepRecordHmdPSMove,

        calibrateStepComplete,
        calibrateStepFailed
    };

    void setState(eMenuState newState);
    void onExitState(eMenuState oldState);
    void onEnterState(eMenuState newState);

private:
    class AppStage_ComputeTrackerPoses *m_parentStage;
    eMenuState m_menuState;
};

#endif // APP_STAGE_COREGISTER_WITH_HMD_H
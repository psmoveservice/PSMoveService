//-- inludes -----
#include "AppSubStage_CalibrateWithHMD.h"
#include "AppStage_ComputeTrackerPoses.h"
#include "App.h"
#include "AssetManager.h"
#include "Camera.h"
#include "ClientHMDView.h"
#include "Logger.h"
#include "OpenVRContext.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "MathGLM.h"

#include "SDL_keycode.h"
#include "SDL_opengl.h"

//-- constants -----

//-- public methods -----
AppSubStage_CalibrateWithHMD::AppSubStage_CalibrateWithHMD(
    AppStage_ComputeTrackerPoses *parentStage)
    : m_parentStage(parentStage)
    , m_menuState(AppSubStage_CalibrateWithHMD::eMenuState::inactive)
{
}

void AppSubStage_CalibrateWithHMD::enter()
{
    setState(AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepAttachPSMove);
}

void AppSubStage_CalibrateWithHMD::exit()
{
    setState(AppSubStage_CalibrateWithHMD::eMenuState::inactive);
}

void AppSubStage_CalibrateWithHMD::update()
{
    switch (m_menuState)
    {
    case AppSubStage_CalibrateWithHMD::eMenuState::inactive:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepAttachPSMove:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepRecordHmdPSMove:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepComplete:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_CalibrateWithHMD::render()
{
    switch (m_menuState)
    {
    case AppSubStage_CalibrateWithHMD::eMenuState::inactive:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepAttachPSMove:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepRecordHmdPSMove:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepComplete:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_CalibrateWithHMD::renderUI()
{
    switch (m_menuState)
    {
    case AppSubStage_CalibrateWithHMD::eMenuState::inactive:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepAttachPSMove:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepRecordHmdPSMove:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepComplete:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}


//-- private methods -----
void AppSubStage_CalibrateWithHMD::setState(
    AppSubStage_CalibrateWithHMD::eMenuState newState)
{
    if (newState != m_menuState)
    {
        onExitState(m_menuState);
        onEnterState(newState);
        m_menuState = newState;
    }
}

void AppSubStage_CalibrateWithHMD::onExitState(
    AppSubStage_CalibrateWithHMD::eMenuState oldState)
{
    switch (oldState)
    {
    case AppSubStage_CalibrateWithHMD::eMenuState::inactive:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepAttachPSMove:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepRecordHmdPSMove:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepComplete:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_CalibrateWithHMD::onEnterState(
    AppSubStage_CalibrateWithHMD::eMenuState newState)
{
    switch (newState)
    {
    case AppSubStage_CalibrateWithHMD::eMenuState::inactive:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepAttachPSMove:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepRecordHmdPSMove:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepComplete:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}
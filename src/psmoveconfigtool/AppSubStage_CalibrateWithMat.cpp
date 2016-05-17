//-- inludes -----
#include "AppSubStage_CalibrateWithMat.h"
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
AppSubStage_CalibrateWithMat::AppSubStage_CalibrateWithMat(
    AppStage_ComputeTrackerPoses *parentStage)
    : m_parentStage(parentStage)
    , m_menuState(AppSubStage_CalibrateWithMat::eMenuState::inactive)
{
}

void AppSubStage_CalibrateWithMat::enter()
{
    setState(AppSubStage_CalibrateWithMat::eMenuState::calibrationStepOriginPlacement);
}

void AppSubStage_CalibrateWithMat::exit()
{
    setState(AppSubStage_CalibrateWithMat::eMenuState::inactive);
}

void AppSubStage_CalibrateWithMat::update()
{
    switch (m_menuState)
    {
    case AppSubStage_CalibrateWithMat::eMenuState::inactive:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepOriginPlacement:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordPSMove:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordHMD:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepComplete:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_CalibrateWithMat::render()
{
    switch (m_menuState)
    {
    case AppSubStage_CalibrateWithMat::eMenuState::inactive:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepOriginPlacement:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordPSMove:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordHMD:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepComplete:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_CalibrateWithMat::renderUI()
{
    switch (m_menuState)
    {
    case AppSubStage_CalibrateWithMat::eMenuState::inactive:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepOriginPlacement:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordPSMove:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordHMD:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepComplete:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}


//-- private methods -----
void AppSubStage_CalibrateWithMat::setState(
    AppSubStage_CalibrateWithMat::eMenuState newState)
{
    if (newState != m_menuState)
    {
        onExitState(m_menuState);
        onEnterState(newState);
        m_menuState = newState;
    }
}

void AppSubStage_CalibrateWithMat::onExitState(
    AppSubStage_CalibrateWithMat::eMenuState oldState)
{
    switch (oldState)
    {
    case AppSubStage_CalibrateWithMat::eMenuState::inactive:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepOriginPlacement:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordPSMove:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordHMD:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepComplete:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_CalibrateWithMat::onEnterState(
    AppSubStage_CalibrateWithMat::eMenuState newState)
{
    switch (newState)
    {
    case AppSubStage_CalibrateWithMat::eMenuState::inactive:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepOriginPlacement:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlacePSMove:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordPSMove:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepPlaceHMD:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrationStepRecordHMD:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepComplete:
        break;
    case AppSubStage_CalibrateWithMat::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}
//-- includes -----
#include "App.h"
#include "AppStage_AccelerometerCalibration.h"
#include "AppStage_ColorCalibration.h"
#include "AppStage_ComputeTrackerPoses.h"
#include "AppStage_ControllerSettings.h"
#include "AppStage_DistortionCalibration.h"
#include "AppStage_HMDAccelerometerCalibration.h"
#include "AppStage_HMDGyroscopeCalibration.h"
#include "AppStage_HMDModelCalibration.h"
#include "AppStage_HMDSettings.h"
#include "AppStage_GyroscopeCalibration.h"
#include "AppStage_MainMenu.h"
#include "AppStage_MagnetometerCalibration.h"
#include "AppStage_OpticalCalibration.h"
#include "AppStage_PairController.h"
#include "AppStage_ServiceSettings.h"
#include "AppStage_TrackerSettings.h"
#include "AppStage_TestButtons.h"
#include "AppStage_TestTracker.h"
#include "AppStage_TestRumble.h"

#ifdef _WIN32
#pragma comment (lib, "winmm.lib")     /* link with Windows MultiMedia lib */
#pragma comment (lib, "opengl32.lib")  /* link with Microsoft OpenGL lib */
#pragma comment (lib, "glu32.lib")     /* link with OpenGL Utility lib */

#pragma warning (disable:4244)	/* Disable bogus conversion warnings. */
#pragma warning (disable:4305)  /* VC++ 5.0 version of above warning. */
#endif

//-- entry point -----
extern "C" int main(int argc, char *argv[])
{
    App app;

    // Register all of the app stages
    app.registerAppStage<AppStage_AccelerometerCalibration>();
	app.registerAppStage<AppStage_ColorCalibration>();
    app.registerAppStage<AppStage_ComputeTrackerPoses>();
    app.registerAppStage<AppStage_ControllerSettings>();
    app.registerAppStage<AppStage_DistortionCalibration>();
    app.registerAppStage<AppStage_GyroscopeCalibration>();
	app.registerAppStage<AppStage_HMDAccelerometerCalibration>();
	app.registerAppStage<AppStage_HMDGyroscopeCalibration>();
	app.registerAppStage<AppStage_HMDModelCalibration>();
    app.registerAppStage<AppStage_HMDSettings>();
	app.registerAppStage<AppStage_MagnetometerCalibration>();
    app.registerAppStage<AppStage_MainMenu>();
	app.registerAppStage<AppStage_OpticalCalibration>();
    app.registerAppStage<AppStage_PairController>();
    app.registerAppStage<AppStage_ServiceSettings>();
    app.registerAppStage<AppStage_TestButtons>();
    app.registerAppStage<AppStage_TestRumble>();
    app.registerAppStage<AppStage_TestTracker>();
    app.registerAppStage<AppStage_TrackerSettings>();

    return app.exec(argc, argv, AppStage_MainMenu::APP_STAGE_NAME);
}

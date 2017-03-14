//-- inludes -----
#include "AppStage_ComputeTrackerPoses.h"
#include "AppStage_MainMenu.h"
#include "AppStage_TrackerSettings.h"
#include "AppSubStage_CalibrateWithMat.h"
#include "AppSubStage_StereoCalibrate.h"
#include "App.h"
#include "AssetManager.h"
#include "Camera.h"
#include "GeometryUtility.h"
#include "Logger.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include "SharedTrackerState.h"
#include "MathGLM.h"

#include "SDL_keycode.h"
#include "SDL_opengl.h"

#include <imgui.h>
#include <sstream>

//-- statics ----
const char *AppStage_ComputeTrackerPoses::APP_STAGE_NAME = "ComputeTrackerPoses";

//-- constants -----
static const glm::vec3 k_hmd_frustum_color = glm::vec3(1.f, 0.788f, 0.055f);
static const glm::vec3 k_psmove_frustum_color = glm::vec3(0.1f, 0.7f, 0.3f);
static const glm::vec3 k_psmove_frustum_color_no_track = glm::vec3(1.0f, 0.f, 0.f);

//-- private methods -----
static void drawController(const PSMController *controllerView, const glm::mat4 &transform, const PSMTrackingColorType trackingColorType);

//-- public methods -----
AppStage_ComputeTrackerPoses::AppStage_ComputeTrackerPoses(App *app)
    : AppStage(app)
    , m_menuState(AppStage_ComputeTrackerPoses::inactive)
    , m_pendingTrackerStartCount(0)
	, m_pendingControllerStartCount(0)
    , m_renderTrackerIndex(0)
    , m_pCalibrateWithMat(new AppSubStage_CalibrateWithMat(this))
	, m_pStereoCalibrate(new AppSubStage_StereoCalibrate(this))
    , m_bSkipCalibration(false)
	, m_overrideControllerId(-1)
{ 
    m_renderTrackerIter = m_trackerViews.end();
}

AppStage_ComputeTrackerPoses::~AppStage_ComputeTrackerPoses()
{
    delete m_pCalibrateWithMat;
}

void AppStage_ComputeTrackerPoses::enterStageAndCalibrate(App *app, int reqeusted_controller_id)
{
	AppStage_ComputeTrackerPoses *appStage= app->getAppStage<AppStage_ComputeTrackerPoses>();
	appStage->m_bSkipCalibration = false;
	appStage->m_overrideControllerId = reqeusted_controller_id;

	app->getAppStage<AppStage_ComputeTrackerPoses>()->m_overrideControllerId = reqeusted_controller_id;
    app->setAppStage(AppStage_ComputeTrackerPoses::APP_STAGE_NAME);
}

void AppStage_ComputeTrackerPoses::enterStageAndSkipCalibration(App *app, int reqeusted_controller_id)
{
	AppStage_ComputeTrackerPoses *appStage = app->getAppStage<AppStage_ComputeTrackerPoses>();
	appStage->m_bSkipCalibration = true;
	appStage->m_overrideControllerId = reqeusted_controller_id;

	app->getAppStage<AppStage_ComputeTrackerPoses>()->m_overrideControllerId = reqeusted_controller_id;
    app->setAppStage(AppStage_ComputeTrackerPoses::APP_STAGE_NAME);
}

void AppStage_ComputeTrackerPoses::enter()
{
    // Kick off this async request chain with a controller list request
    // -> controller start request
    // -> tracker list request
    // -> reacker start request
    request_controller_list();

    m_app->setCameraType(_cameraFixed);
}

void AppStage_ComputeTrackerPoses::exit()
{
    release_devices();

    setState(eMenuState::inactive);
}

void AppStage_ComputeTrackerPoses::update()
{
    switch (m_menuState)
    {
    case eMenuState::inactive:
        break;
    case eMenuState::pendingControllerListRequest:
    case eMenuState::pendingControllerStartRequest:
    case eMenuState::pendingTrackerListRequest:
    case eMenuState::pendingTrackerStartRequest:
        break;
    case eMenuState::failedControllerListRequest:
    case eMenuState::failedControllerStartRequest:
    case eMenuState::failedTrackerStartRequest:
        break;
    case eMenuState::verifyTrackers:
        update_tracker_video();
        break;
	case eMenuState::selectCalibrationMethod:
		break;
    case eMenuState::calibrateWithMat:
        {
            m_pCalibrateWithMat->update();

            if (m_pCalibrateWithMat->getMenuState() == AppSubStage_CalibrateWithMat::calibrateStepSuccess)
            {
                setState(AppStage_ComputeTrackerPoses::eMenuState::testTracking);
            }
            else if (m_pCalibrateWithMat->getMenuState() == AppSubStage_CalibrateWithMat::calibrateStepFailed)
            {
                setState(AppStage_ComputeTrackerPoses::eMenuState::calibrateStepFailed);
            }
        }
        break;
    case eMenuState::stereoCalibrate:
        {
            m_pStereoCalibrate->update();

            if (m_pStereoCalibrate->getMenuState() == AppSubStage_StereoCalibrate::calibrateStepSuccess)
            {
                setState(AppStage_ComputeTrackerPoses::eMenuState::testTracking);
            }
            else if (m_pStereoCalibrate->getMenuState() == AppSubStage_StereoCalibrate::calibrateStepFailed)
            {
                setState(AppStage_ComputeTrackerPoses::eMenuState::calibrateStepFailed);
            }
        }
        break;
    case eMenuState::testTracking:
        break;
	case eMenuState::showTrackerVideo:
		update_tracker_video();
		break;
    case eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ComputeTrackerPoses::render()
{
    switch (m_menuState)
    {
    case eMenuState::inactive:
        break;
    case eMenuState::pendingControllerListRequest:
    case eMenuState::pendingControllerStartRequest:
    case eMenuState::pendingTrackerListRequest:
    case eMenuState::pendingTrackerStartRequest:
        break;
    case eMenuState::failedControllerListRequest:
    case eMenuState::failedControllerStartRequest:
    case eMenuState::failedTrackerListRequest:
    case eMenuState::failedTrackerStartRequest:
        break;
    case eMenuState::verifyTrackers:
        {
            render_tracker_video();
        } break;
	case eMenuState::selectCalibrationMethod:
		break;
    case eMenuState::calibrateWithMat:
        m_pCalibrateWithMat->render();
        break;
    case eMenuState::stereoCalibrate:
        m_pStereoCalibrate->render();
        break;
    case eMenuState::testTracking:
        {
            // Draw the chaperone origin axes
            drawTransformedAxes(glm::mat4(1.0f), 100.f);

            // Draw the frustum for each tracking camera.
            // The frustums are defined in PSMove tracking space.
            // We need to transform them into chaperone space to display them along side the HMD.
            for (t_tracker_state_map_iterator tracker_iter = m_trackerViews.begin(); tracker_iter != m_trackerViews.end(); ++tracker_iter)
            {
                const PSMTracker *trackerView = tracker_iter->second.trackerView;
				const int tracker_id= trackerView->tracker_info.tracker_id;
                const PSMPosef trackerPose = trackerView->tracker_info.tracker_pose;
                const glm::mat4 trackerMat4 = psm_posef_to_glm_mat4(trackerPose);

                PSMFrustum frustum;
				PSM_GetTrackerFrustum(tracker_id, &frustum);

				// use color depending on tracking status
				glm::vec3 color= does_tracker_see_any_controller(trackerView) ? k_psmove_frustum_color : k_psmove_frustum_color_no_track;

				drawTextAtWorldPosition(glm::mat4(1.f), psm_vector3f_to_glm_vec3(trackerPose.Position), "#%d", tracker_id);
                drawTransformedFrustum(glm::mat4(1.f), &frustum, color);

                drawTransformedAxes(trackerMat4, 20.f);
            }

            // Draw the psmove model
			for (t_controller_state_map_iterator controller_iter = m_controllerViews.begin(); controller_iter != m_controllerViews.end(); ++controller_iter)
            {
				const PSMController *controllerView = controller_iter->second.controllerView;
				const PSMTrackingColorType trackingColorType= controller_iter->second.trackingColorType;

				PSMPosef controllerPose;
				PSMPhysicsData physicsData;
				switch (controllerView->ControllerType)
				{
				case PSMControllerType::PSMController_Move:
					controllerPose = controllerView->ControllerState.PSMoveState.Pose;
					physicsData= controllerView->ControllerState.PSMoveState.PhysicsData;
					break;
				case PSMControllerType::PSMController_DualShock4:
					controllerPose = controllerView->ControllerState.PSDS4State.Pose;
					physicsData= controllerView->ControllerState.PSDS4State.PhysicsData;
					break;
				}
                glm::mat4 controllerMat4 = psm_posef_to_glm_mat4(controllerPose);

				if (m_controllerViews.size() > 1)
				{
					drawTextAtWorldPosition(glm::mat4(1.f), psm_vector3f_to_glm_vec3(controllerPose.Position), "#%d", controllerView->ControllerID);
				}
                drawController(controllerView, controllerMat4, trackingColorType);
                drawTransformedAxes(controllerMat4, 10.f);

				// Draw the acceleration and velocity arrows
				{
					const glm::mat4 originMat4= glm::translate(glm::mat4(1.f), psm_vector3f_to_glm_vec3(controllerPose.Position));
					const glm::vec3 vel_endpoint = psm_vector3f_to_glm_vec3(physicsData.LinearVelocityCmPerSec);
					const glm::vec3 acc_endpoint = psm_vector3f_to_glm_vec3(physicsData.LinearAccelerationCmPerSecSqr)*PSM_CENTIMETERS_TO_METERS;
					
					const float vel= glm::length(vel_endpoint);
					if (vel > k_positional_epsilon)
					{
						drawArrow(originMat4, glm::vec3(0.f), vel_endpoint, 0.1f, glm::vec3(0.f, 1.f, 1.f));
						//drawTextAtWorldPosition(originMat4, vel_endpoint, "v=%.2fcm/s", vel);
					}

					const float acc = glm::length(acc_endpoint);
					if (acc > k_positional_epsilon)
					{
						drawArrow(originMat4, glm::vec3(0.f), acc_endpoint, 0.1f, glm::vec3(1.f, 1.f, 0.f));
						//drawTextAtWorldPosition(originMat4, acc_endpoint, "a=%.2fm/s^2", acc);
					}
				}
            }

        } break;
	case eMenuState::showTrackerVideo:
		{
			render_tracker_video();
		} break;
    case eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ComputeTrackerPoses::renderUI()
{
    const float k_panel_width = 300.f;
    const char *k_window_title = "Compute Tracker Poses";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case eMenuState::inactive:
        break;

    case eMenuState::pendingControllerListRequest:
    case eMenuState::pendingControllerStartRequest:
    case eMenuState::pendingTrackerListRequest:
    case eMenuState::pendingTrackerStartRequest:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 80));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Pending device initialization...");

            if (ImGui::Button("Return to Tracker Settings"))
            {
                request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;

    case eMenuState::failedControllerListRequest:
    case eMenuState::failedControllerStartRequest:
    case eMenuState::failedTrackerListRequest:
    case eMenuState::failedTrackerStartRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 180));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            switch (m_menuState)
            {
            case eMenuState::failedControllerListRequest:
                ImGui::Text("Failed controller list retrieval!");
                break;
            case eMenuState::failedControllerStartRequest:
                ImGui::Text("Failed controller stream start!");
                break;
            case eMenuState::failedTrackerListRequest:
                ImGui::Text("Failed tracker list retrieval!");
                break;
            case eMenuState::failedTrackerStartRequest:
                ImGui::Text("Failed tracker stream start!");
                break;
            }

            if (ImGui::Button("Ok"))
            {
                request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }

            if (ImGui::Button("Return to Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;

    case eMenuState::verifyTrackers:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - 500.f / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(500.f, (m_trackerViews.size() > 0) ? 150.f : 100.f));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Verify that your tracking cameras can see the tracking origin");
            ImGui::Separator();

            if (m_trackerViews.size() > 1)
            {
                ImGui::Text("Tracker #%d", m_renderTrackerIndex);

                if (ImGui::Button("Previous Tracker"))
                {
                    go_previous_tracker();
                }
                ImGui::SameLine();
                if (ImGui::Button("Next Tracker"))
                {
                    go_next_tracker();
                }
            }

            if (ImGui::Button("Looks Good!"))
            {
				if (m_trackerViews.size() == 2)
				{
					// only consider stereo camera calibration when there are two trackers
					setState(eMenuState::selectCalibrationMethod);
				}
				else
				{
					setState(eMenuState::calibrateWithMat);
				}
            }

            if (ImGui::Button("Hmm... Something is wrong."))
            {
                request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        }
		break;

	case eMenuState::selectCalibrationMethod:
		{
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - 500.f / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(500.f, 150.f));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Select a calibration method");
            ImGui::Separator();
			ImGui::TextWrapped("Use 'Stereo Camera' if you have two cameras rigidly aligned side by side a few centimeters apart, otherwise use 'Calibration Mat'.");
			ImGui::Separator();

            if (ImGui::Button("Calibration Mat"))
            {
                setState(eMenuState::calibrateWithMat);
            }
            if (ImGui::Button("Stereo Camera"))
            {
                setState(eMenuState::stereoCalibrate);
            }

            ImGui::End();
		} break;

    case eMenuState::calibrateWithMat:
        {
            m_pCalibrateWithMat->renderUI();
        } break;

    case eMenuState::stereoCalibrate:
        {
            m_pStereoCalibrate->renderUI();
        } break;

    case eMenuState::testTracking:
        {
            ImGui::SetNextWindowPos(ImVec2(20.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(250.f, 260.f));
            ImGui::Begin("Test Tracking", nullptr, window_flags);

			// display per tracker UI
			for (t_tracker_state_map_iterator iter = m_trackerViews.begin(); iter != m_trackerViews.end(); ++iter)
			{
				const PSMTracker *trackerView = iter->second.trackerView;

				ImGui::PushItemWidth(125.f);
				if (does_tracker_see_any_controller(trackerView))
				{
					ImGui::Text("Tracker #%d: OK", trackerView->tracker_info.tracker_id);
				}
				else 
				{
					ImGui::Text("Tracker #%d: FAIL", trackerView->tracker_info.tracker_id);
				}
				ImGui::PopItemWidth();

				ImGui::SameLine();

				ImGui::PushItemWidth(100.f);
				ImGui::PushID(trackerView->tracker_info.tracker_id);
				if (m_app->getIsLocalServer())
				{
					if (ImGui::Button("Tracker Video"))
					{
						m_renderTrackerIter = iter;
						setState(eMenuState::showTrackerVideo);
					}
				}
				else
				{
					ImGui::TextDisabled("Tracker Video");
				}
				ImGui::PopID();
				ImGui::PopItemWidth();
			}

			ImGui::Separator();

			if (!m_bSkipCalibration)
			{
				ImGui::Text("Calibration Complete");

				if (ImGui::Button("Redo Calibration"))
				{
					setState(eMenuState::verifyTrackers);
				}
			}

			if (ImGui::Button("Exit"))
			{
				m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
			}

            ImGui::End();
        }
        break;

	case eMenuState::showTrackerVideo:
		{
			ImGui::SetNextWindowPos(ImVec2(20.f, 20.f));
			ImGui::SetNextWindowSize(ImVec2(200, 100));
			ImGui::Begin("Tracker Video Feed", nullptr, window_flags);

			//ImGui::Text("Tracker ID: #%d", m_renderTrackerIter->second.trackerView->tracker_info.tracker_id);

			if (m_trackerViews.size() > 1)
			{
				if (ImGui::Button("<##Previous Tracker"))
				{
					go_previous_tracker();
				}
				ImGui::SameLine();
				ImGui::Text("Tracker ID: #%d", m_renderTrackerIter->second.trackerView->tracker_info.tracker_id);
				ImGui::SameLine();
				if (ImGui::Button(">##Next Tracker"))
				{
					go_next_tracker();
				}
			} 
			else {
				ImGui::Text("Tracker ID: 0");
			}

			if (ImGui::Button("Return"))
			{
				setState(eMenuState::testTracking);
			}

			ImGui::End();
		}
		break;

    case eMenuState::calibrateStepFailed:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Calibration Failed");

            if (ImGui::Button("Restart Calibration"))
            {
                setState(eMenuState::verifyTrackers);
            }

            if (ImGui::Button("Cancel"))
            {
                m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        }
        break;

    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ComputeTrackerPoses::setState(eMenuState newState)
{
    if (newState != m_menuState)
    {
        onExitState(m_menuState);
        onEnterState(newState);

        m_menuState = newState;
    }
}

void AppStage_ComputeTrackerPoses::onExitState(eMenuState newState)
{
    switch (m_menuState)
    {
    case eMenuState::inactive:
        break;
    case eMenuState::pendingControllerListRequest:
    case eMenuState::pendingControllerStartRequest:
    case eMenuState::pendingTrackerListRequest:
    case eMenuState::pendingTrackerStartRequest:
        break;
    case eMenuState::failedControllerListRequest:
    case eMenuState::failedControllerStartRequest:
    case eMenuState::failedTrackerListRequest:
    case eMenuState::failedTrackerStartRequest:
        break;
    case eMenuState::verifyTrackers:
        break;
	case eMenuState::selectCalibrationMethod:
        break;
    case eMenuState::calibrateWithMat:
        m_pCalibrateWithMat->exit();
        break;
	case eMenuState::stereoCalibrate:
		m_pStereoCalibrate->exit();
		break;
    case eMenuState::testTracking:
        m_app->setCameraType(_cameraFixed);
        break;
	case eMenuState::showTrackerVideo:
		break;
    case eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ComputeTrackerPoses::onEnterState(eMenuState newState)
{
    switch (newState)
    {
    case eMenuState::inactive:
        break;
    case eMenuState::pendingControllerListRequest:
		break;
    case eMenuState::pendingControllerStartRequest:
		m_controllerViews.clear();
		m_pendingControllerStartCount = 0;
		break;
    case eMenuState::pendingTrackerListRequest:
        break;
    case eMenuState::pendingTrackerStartRequest:
        m_trackerViews.clear();
        m_pendingTrackerStartCount = 0;
        break;
    case eMenuState::failedControllerListRequest:
    case eMenuState::failedControllerStartRequest:
    case eMenuState::failedTrackerListRequest:
    case eMenuState::failedTrackerStartRequest:
        break;
    case eMenuState::verifyTrackers:
        m_renderTrackerIter = m_trackerViews.begin();
        break;
	case eMenuState::selectCalibrationMethod:
        break;
    case eMenuState::calibrateWithMat:
        m_pCalibrateWithMat->enter();
        break;
    case eMenuState::stereoCalibrate:
        m_pStereoCalibrate->enter();
        break;
    case eMenuState::testTracking:
		{
			for (t_controller_state_map_iterator controller_iter = m_controllerViews.begin(); controller_iter != m_controllerViews.end(); ++controller_iter)
			{
				PSMController *controllerView = controller_iter->second.controllerView;

				switch (controllerView->ControllerType)
				{
				case PSMController_Move:
					controllerView->ControllerState.PSMoveState.bPoseResetButtonEnabled= true;
					break;
				case PSMController_DualShock4:
					controllerView->ControllerState.PSDS4State.bPoseResetButtonEnabled= true;
					break;
				}
			}

			m_app->setCameraType(_cameraOrbit);
		}
        break;
	case eMenuState::showTrackerVideo:
		break;
    case eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ComputeTrackerPoses::update_tracker_video()
{
    if (m_renderTrackerIter != m_trackerViews.end())
    {
		const int tracker_id= m_renderTrackerIter->second.trackerView->tracker_info.tracker_id;

        // Render the latest from the currently active tracker
        if (PSM_PollTrackerVideoStream(tracker_id) == PSMResult_Success)
        {
			const unsigned char *buffer= nullptr;
			if (PSM_GetTrackerVideoFrameBuffer(tracker_id, &buffer) == PSMResult_Success)
			{
				m_renderTrackerIter->second.textureAsset->copyBufferIntoTexture(buffer);
			}
        }
    }
}

void AppStage_ComputeTrackerPoses::render_tracker_video()
{
    if (m_renderTrackerIter != m_trackerViews.end() &&
        m_renderTrackerIter->second.textureAsset != nullptr)
    {
        drawFullscreenTexture(m_renderTrackerIter->second.textureAsset->texture_id);
    }
}

void AppStage_ComputeTrackerPoses::go_next_tracker()
{
    const int trackerCount = static_cast<int>(m_trackerViews.size());

    if (trackerCount > 1)
    {
        m_renderTrackerIndex = (m_renderTrackerIndex + 1) % trackerCount;

        // Find the tracker iterator that corresponds to the render index we want to show
        for (t_tracker_state_map_iterator iter = m_trackerViews.begin(); iter != m_trackerViews.end(); ++iter)
        {
            if (iter->second.listIndex == m_renderTrackerIndex)
            {
                m_renderTrackerIter = iter;
            }
        }
    }
}

void AppStage_ComputeTrackerPoses::go_previous_tracker()
{
    const int trackerCount = static_cast<int>(m_trackerViews.size());

    if (trackerCount > 1)
    {
        m_renderTrackerIndex = (m_renderTrackerIndex + trackerCount - 1) % trackerCount;

        // Find the tracker iterator that corresponds to the render index we want to show
        for (t_tracker_state_map_iterator iter = m_trackerViews.begin(); iter != m_trackerViews.end(); ++iter)
        {
            if (iter->second.listIndex == m_renderTrackerIndex)
            {
                m_renderTrackerIter = iter;
            }
        }
    }
}

int AppStage_ComputeTrackerPoses::get_tracker_count() const
{
    return static_cast<int>(m_trackerViews.size());
}

int AppStage_ComputeTrackerPoses::get_render_tracker_index() const
{
    return m_renderTrackerIndex;
}

PSMTracker *AppStage_ComputeTrackerPoses::get_render_tracker_view() const
{
    return (m_trackerViews.size() > 0) ? m_renderTrackerIter->second.trackerView : nullptr;
}

PSMController *AppStage_ComputeTrackerPoses::get_calibration_controller_view() const
{
	return (m_controllerViews.size() > 0) ? m_controllerViews.begin()->second.controllerView : nullptr;
}

void AppStage_ComputeTrackerPoses::release_devices()
{
    //###HipsterSloth $REVIEW Do we care about canceling in-flight requests?

	for (t_controller_state_map_iterator iter = m_controllerViews.begin(); iter != m_controllerViews.end(); ++iter)
	{
		ControllerState &controllerState = iter->second;

		if (controllerState.controllerView != nullptr)
		{
			PSM_StopControllerDataStreamAsync(controllerState.controllerView->ControllerID, nullptr);
			PSM_FreeControllerListener(controllerState.controllerView->ControllerID);
		}
	}

	m_controllerViews.clear();
	m_pendingControllerStartCount = 0;

    for (t_tracker_state_map_iterator iter = m_trackerViews.begin(); iter != m_trackerViews.end(); ++iter)
    {
        TrackerState &trackerState = iter->second;

        if (trackerState.textureAsset != nullptr)
        {
            delete trackerState.textureAsset;
        }

        if (trackerState.trackerView != nullptr)
        {
			const int tracker_id= trackerState.trackerView->tracker_info.tracker_id;

			PSM_CloseTrackerVideoStream(tracker_id);
			PSM_StopTrackerDataStreamAsync(tracker_id, nullptr);
			PSM_FreeTrackerListener(tracker_id);
        }
    }

    m_trackerViews.clear();
    m_pendingTrackerStartCount= 0;

    m_renderTrackerIndex= 0;
    m_renderTrackerIter = m_trackerViews.end();
}

void AppStage_ComputeTrackerPoses::request_exit_to_app_stage(const char *app_stage_name)
{
    release_devices();

    m_app->setAppStage(app_stage_name);
}

void AppStage_ComputeTrackerPoses::request_controller_list()
{
    if (m_menuState != AppStage_ComputeTrackerPoses::pendingControllerListRequest)
    {
        m_menuState = AppStage_ComputeTrackerPoses::pendingControllerListRequest;
        
        // Request a list of controllers back from the server
		PSMRequestID requestID;
		PSM_GetControllerListAsync(&requestID);
		PSM_RegisterCallback(requestID, AppStage_ComputeTrackerPoses::handle_controller_list_response, this);
    }
}

void AppStage_ComputeTrackerPoses::handle_controller_list_response(
    const PSMResponseMessage *response_message,
    void *userdata)
{
	const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_message->opaque_response_handle);
	const PSMoveProtocol::Request *request = GET_PSMOVEPROTOCOL_REQUEST(response_message->opaque_request_handle);
	
    AppStage_ComputeTrackerPoses *thisPtr = static_cast<AppStage_ComputeTrackerPoses *>(userdata);

    const PSMResult ResultCode = response_message->result_code;

    switch (ResultCode)
    {
    case PSMResult_Success:
        {
            assert(response_message->payload_type == PSMResponseMessage::_responsePayloadType_ControllerList);
            const PSMControllerList *controller_list = &response_message->payload.controller_list;

			if (thisPtr->m_overrideControllerId == -1)
			{
				bool bStartedAnyControllers = false;

				// Start all psmove and dual shock 4 controllers
				for (int list_index = 0; list_index < controller_list->count; ++list_index)
				{
					if (controller_list->controller_type[list_index] == PSMController_Move ||
						controller_list->controller_type[list_index] == PSMController_DualShock4)
					{
						int trackedControllerId = controller_list->controller_id[list_index];
						const auto &protocolControllerResponse = response->result_controller_list().controllers(list_index);
						const PSMTrackingColorType trackingColorType=
							static_cast<PSMTrackingColorType>(protocolControllerResponse.tracking_color_type());

						thisPtr->request_start_controller_stream(trackedControllerId, list_index, trackingColorType);
						bStartedAnyControllers = true;
					}
				}

				if (!bStartedAnyControllers)
				{
					thisPtr->setState(AppStage_ComputeTrackerPoses::failedControllerListRequest);
				}
			}
			else
			{
				int trackedControllerId = -1;
				int trackedControllerListIndex = -1;
				PSMTrackingColorType trackingColorType;

				// Start only the selected controller
				for (int list_index = 0; list_index < controller_list->count; ++list_index)
				{
					if (controller_list->controller_id[list_index] == thisPtr->m_overrideControllerId)
					{
						const auto &protocolControllerResponse = response->result_controller_list().controllers(list_index);

						trackingColorType = static_cast<PSMTrackingColorType>(protocolControllerResponse.tracking_color_type());
						trackedControllerId = controller_list->controller_id[list_index];
						trackedControllerListIndex = list_index;
						break;
					}
				}

				if (trackedControllerId != -1)
				{
					thisPtr->request_start_controller_stream(trackedControllerId, trackedControllerListIndex, trackingColorType);
				}
				else
				{
					thisPtr->setState(AppStage_ComputeTrackerPoses::failedControllerListRequest);
				}
			}
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
	case PSMResult_Timeout:
        {
            thisPtr->setState(AppStage_ComputeTrackerPoses::failedControllerListRequest);
        } break;
    }
}

void AppStage_ComputeTrackerPoses::request_start_controller_stream(
	int ControllerID,
	int listIndex,
	PSMTrackingColorType trackingColorType)
{
	ControllerState controllerState;

	setState(eMenuState::pendingControllerStartRequest);

	// Allocate a new controller view
	PSM_AllocateControllerListener(ControllerID);
	controllerState.listIndex = listIndex;
	controllerState.controllerView = PSM_GetController(ControllerID);
	controllerState.trackingColorType = trackingColorType;

	// Add the controller to the list of controllers we're monitoring
	assert(m_controllerViews.find(ControllerID) == m_controllerViews.end());
	m_controllerViews.insert(t_id_controller_state_pair(ControllerID, controllerState));

	// Increment the number of requests we're waiting to get back
	++m_pendingControllerStartCount;

	unsigned int flags =
		PSMStreamFlags_includePositionData |
		PSMStreamFlags_includeCalibratedSensorData |
		PSMStreamFlags_includeRawTrackerData |
		PSMStreamFlags_includePhysicsData;

	// If we are jumping straight to testing, we want the ROI optimization on
	if (!m_bSkipCalibration)
	{
		flags|= PSMStreamFlags_disableROI;
	}

    // Start receiving data from the controller
	PSMRequestID request_id;
	PSM_StartControllerDataStreamAsync(controllerState.controllerView->ControllerID, flags, &request_id);
	PSM_RegisterCallback(request_id, &AppStage_ComputeTrackerPoses::handle_start_controller_response, this);
}

void AppStage_ComputeTrackerPoses::handle_start_controller_response(
    const PSMResponseMessage *response_message,
    void *userdata)
{
    AppStage_ComputeTrackerPoses *thisPtr = static_cast<AppStage_ComputeTrackerPoses *>(userdata);

    const PSMResult ResultCode = response_message->result_code;

    switch (ResultCode)
    {
    case PSMResult_Success:
        {
			// See if this was the last controller we were waiting to get a response from
			--thisPtr->m_pendingControllerStartCount;
			if (thisPtr->m_pendingControllerStartCount <= 0)
			{
				// Move on to the controllers
				thisPtr->request_tracker_list();
			}
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
	case PSMResult_Timeout:
        {
            thisPtr->setState(AppStage_ComputeTrackerPoses::failedControllerStartRequest);
        } break;
    }
}

void AppStage_ComputeTrackerPoses::request_tracker_list()
{
    if (m_menuState != eMenuState::pendingTrackerListRequest)
    {
        setState(eMenuState::pendingTrackerListRequest);

        // Tell the psmove service that we we want a list of trackers connected to this machine
		PSMRequestID requestId;
		PSM_GetTrackerListAsync(&requestId);
		PSM_RegisterCallback(requestId, AppStage_ComputeTrackerPoses::handle_tracker_list_response, this);
    }
}

void AppStage_ComputeTrackerPoses::handle_tracker_list_response(
    const PSMResponseMessage *response_message,
    void *userdata)
{
    AppStage_ComputeTrackerPoses *thisPtr = static_cast<AppStage_ComputeTrackerPoses *>(userdata);

    switch (response_message->result_code)
    {
    case PSMResult_Success:
        {
            assert(response_message->payload_type == PSMResponseMessage::_responsePayloadType_TrackerList);
            const PSMTrackerList &tracker_list = response_message->payload.tracker_list;

            for (int tracker_index = 0; tracker_index < tracker_list.count; ++tracker_index)
            {
                thisPtr->request_tracker_start_stream(&tracker_list.trackers[tracker_index], tracker_index);
            }
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
	case PSMResult_Timeout:
        {
            thisPtr->setState(eMenuState::failedTrackerListRequest);
        } break;
    }
}

void request_tracker_start_stream(const PSMClientTrackerInfo *TrackerInfo, int listIndex);
void AppStage_ComputeTrackerPoses::request_tracker_start_stream(
    const PSMClientTrackerInfo *TrackerInfo,
    int listIndex)
{
    TrackerState trackerState;

    setState(eMenuState::pendingTrackerStartRequest);

    // Allocate a new tracker view
	const int tracker_id= TrackerInfo->tracker_id;
    trackerState.listIndex = listIndex;
	PSM_AllocateTrackerListener(tracker_id, TrackerInfo);
	trackerState.trackerView = PSM_GetTracker(tracker_id);
    trackerState.textureAsset = nullptr;

    // Add the tracker to the list of trackers we're monitoring
    assert(m_trackerViews.find(TrackerInfo->tracker_id) == m_trackerViews.end());
    m_trackerViews.insert(t_id_tracker_state_pair(TrackerInfo->tracker_id, trackerState));

    // Increment the number of requests we're waiting to get back
    ++m_pendingTrackerStartCount;

    // Request data to start streaming to the tracker
	PSMRequestID requestID;
	PSM_StartTrackerDataStreamAsync(
		TrackerInfo->tracker_id, 
		&requestID);
	PSM_RegisterCallback(requestID, AppStage_ComputeTrackerPoses::handle_tracker_start_stream_response, this);
}

void AppStage_ComputeTrackerPoses::handle_tracker_start_stream_response(
    const PSMResponseMessage *response_message,
    void *userdata)
{
    AppStage_ComputeTrackerPoses *thisPtr = static_cast<AppStage_ComputeTrackerPoses *>(userdata);

    switch (response_message->result_code)
    {
    case PSMResult_Success:
        {
            // Get the tracker ID this request was for
            const PSMoveProtocol::Request *request = GET_PSMOVEPROTOCOL_REQUEST(response_message->opaque_request_handle);
            const int tracker_id= request->request_start_tracker_data_stream().tracker_id();

            // Get the tracker state associated with the tracker id
            t_tracker_state_map_iterator trackerStateEntry = thisPtr->m_trackerViews.find(tracker_id);
            assert(trackerStateEntry != thisPtr->m_trackerViews.end());

            // The context holds everything a handler needs to evaluate a response
            TrackerState &trackerState = trackerStateEntry->second;
			PSMClientTrackerInfo &trackerInfo= trackerState.trackerView->tracker_info;

            // Open the shared memory that the video stream is being written to
            if (PSM_OpenTrackerVideoStream(trackerInfo.tracker_id) == PSMResult_Success)
            {
                // Create a texture to render the video frame to
                trackerState.textureAsset = new TextureAsset();
                trackerState.textureAsset->init(
                    static_cast<unsigned int>(trackerInfo.tracker_screen_dimensions.x),
                    static_cast<unsigned int>(trackerInfo.tracker_screen_dimensions.y),
                    GL_RGB, // texture format
                    GL_BGR, // buffer format
                    nullptr);
            }

            // See if this was the last tracker we were waiting to get a response from
            --thisPtr->m_pendingTrackerStartCount;
            if (thisPtr->m_pendingTrackerStartCount <= 0)
            {
                thisPtr->handle_all_devices_ready();
            }
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
	case PSMResult_Timeout:
        {
            thisPtr->setState(eMenuState::failedTrackerStartRequest);
        } break;
    }
}

static void copy_pose_to_request(
    const PSMPosef &pose,
    PSMoveProtocol::Pose *pose_request)
{
    {
        PSMoveProtocol::Orientation *orientation_request= pose_request->mutable_orientation();

        orientation_request->set_w(pose.Orientation.w);
        orientation_request->set_x(pose.Orientation.x);
        orientation_request->set_y(pose.Orientation.y);
        orientation_request->set_z(pose.Orientation.z);
    }

    {
        PSMoveProtocol::Position *position_request = pose_request->mutable_position();

        position_request->set_x(pose.Position.x);
        position_request->set_y(pose.Position.y);
        position_request->set_z(pose.Position.z);
    }
}

void AppStage_ComputeTrackerPoses::request_set_tracker_pose(
    const PSMPosef *pose,
    PSMTracker *TrackerView)
{
    // Set the pose on out local tracker view
    TrackerView->tracker_info.tracker_pose = *pose;

    // Update the pose on the service
    {
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_POSE);

        PSMoveProtocol::Request_RequestSetTrackerPose *set_pose_request =
            request->mutable_request_set_tracker_pose();

        set_pose_request->set_tracker_id(TrackerView->tracker_info.tracker_id);
        copy_pose_to_request(TrackerView->tracker_info.tracker_pose, set_pose_request->mutable_pose());

		PSM_SendOpaqueRequest(&request, nullptr);
    }
}

void AppStage_ComputeTrackerPoses::handle_all_devices_ready()
{
    if (!m_bSkipCalibration)
    {
        setState(eMenuState::verifyTrackers);
    }
    else
    {
        setState(eMenuState::testTracking);
    }
}

bool AppStage_ComputeTrackerPoses::does_tracker_see_any_controller(const PSMTracker *trackerView)
{
	bool bTrackerSeesAnyController = false;
	for (t_controller_state_map_iterator controller_iter = m_controllerViews.begin(); controller_iter != m_controllerViews.end(); ++controller_iter)
	{
		const PSMController *controllerView = controller_iter->second.controllerView;
		const int tracker_id= trackerView->tracker_info.tracker_id;

		PSMVector2f screenSample;
		glm::vec3 color;
		if (controllerView->ControllerType == PSMControllerType::PSMController_Move &&
			controllerView->ControllerState.PSMoveState.bIsCurrentlyTracking)
		{
			screenSample= controllerView->ControllerState.PSMoveState.RawTrackerData.ScreenLocations[tracker_id];
			bTrackerSeesAnyController = true;
			break;
		}
		else if (controllerView->ControllerType == PSMControllerType::PSMController_DualShock4 &&
				 controllerView->ControllerState.PSDS4State.bIsCurrentlyTracking)
		{
			screenSample= controllerView->ControllerState.PSDS4State.RawTrackerData.ScreenLocations[tracker_id];
			bTrackerSeesAnyController = true;
			break;
		}
	}

	return bTrackerSeesAnyController;
}

//-- private methods -----
static void drawController(
	const PSMController *controllerView, 
	const glm::mat4 &transform, 
	const PSMTrackingColorType trackingColorType)
{
	glm::vec3 bulb_color = glm::vec3(1.f, 1.f, 1.f);

	switch (trackingColorType)
	{
	case PSMTrackingColorType_Magenta:
		bulb_color = glm::vec3(1.f, 0.f, 1.f);
		break;
	case PSMTrackingColorType_Cyan:
		bulb_color = glm::vec3(0.f, 1.f, 1.f);
		break;
	case PSMTrackingColorType_Yellow:
		bulb_color = glm::vec3(1.f, 1.f, 0.f);
		break;
	case PSMTrackingColorType_Red:
		bulb_color = glm::vec3(1.f, 0.f, 0.f);
		break;
	case PSMTrackingColorType_Green:
		bulb_color = glm::vec3(0.f, 1.f, 0.f);
		break;
	case PSMTrackingColorType_Blue:
		bulb_color = glm::vec3(0.f, 0.f, 1.f);
		break;
	default:
		break;
	}

    switch(controllerView->ControllerType)
    {
    case PSMController_Move:
        drawPSMoveModel(transform, bulb_color);
        break;
    case PSMController_DualShock4:
        drawPSDualShock4Model(transform, bulb_color);
        break;
    }
}
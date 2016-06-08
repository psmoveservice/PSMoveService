//-- inludes -----
#include "AppStage_ComputeTrackerPoses.h"
#include "AppStage_MainMenu.h"
#include "AppStage_TrackerSettings.h"
#include "AppStage_HMDSettings.h"
#include "AppSubStage_CalibrateWithHMD.h"
#include "AppSubStage_CalibrateWithMat.h"
#include "App.h"
#include "AssetManager.h"
#include "Camera.h"
#include "ClientHMDView.h"
#include "GeometryUtility.h"
#include "Logger.h"
#include "OpenVRContext.h"
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

//-- public methods -----
AppStage_ComputeTrackerPoses::AppStage_ComputeTrackerPoses(App *app)
    : AppStage(app)
    , m_menuState(AppStage_ComputeTrackerPoses::inactive)
    , m_hmdView(nullptr)
    , m_controllerView(nullptr)
    , m_pendingTrackerStartCount(0)
    , m_renderTrackerIndex(0)
    , m_pCalibrateWithHMD(new AppSubStage_CalibrateWithHMD(this))
    , m_pCalibrateWithMat(new AppSubStage_CalibrateWithMat(this))
    , m_bSkipCalibration(false)
{ 
    m_renderTrackerIter = m_trackerViews.end();
}

AppStage_ComputeTrackerPoses::~AppStage_ComputeTrackerPoses()
{
    delete m_pCalibrateWithHMD;
    delete m_pCalibrateWithMat;
}

void AppStage_ComputeTrackerPoses::enterStageAndCalibrate(App *app)
{
    app->getAppStage<AppStage_ComputeTrackerPoses>()->m_bSkipCalibration = false;
    app->setAppStage(AppStage_ComputeTrackerPoses::APP_STAGE_NAME);
}

void AppStage_ComputeTrackerPoses::enterStageAndSkipCalibration(App *app)
{
    app->getAppStage<AppStage_ComputeTrackerPoses>()->m_bSkipCalibration = true;
    app->setAppStage(AppStage_ComputeTrackerPoses::APP_STAGE_NAME);
}

void AppStage_ComputeTrackerPoses::enter()
{
    // Allocate a new HMD stream
    assert(m_hmdView == nullptr);

    if (m_app->getOpenVRContext()->getIsInitialized())
    {
        m_hmdView = m_app->getOpenVRContext()->allocateHmdView();
    }

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
    case eMenuState::verifyHMD:
        break;
    case eMenuState::verifyTrackers:
        update_tracker_video();
        break;
    case eMenuState::selectCalibrationType:
        break;
    case eMenuState::calibrateWithHMD:
        {
            m_pCalibrateWithHMD->update();

            if (m_pCalibrateWithHMD->getMenuState() == AppSubStage_CalibrateWithHMD::calibrateStepSuccess)
            {
                setState(AppStage_ComputeTrackerPoses::eMenuState::testTracking);
            }
            else if (m_pCalibrateWithHMD->getMenuState() == AppSubStage_CalibrateWithHMD::calibrateStepFailed)
            {
                setState(AppStage_ComputeTrackerPoses::eMenuState::calibrateStepFailed);
            }
        }
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
    case eMenuState::testTracking:
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
    case eMenuState::verifyHMD:
        {
            if (m_hmdView != nullptr)
            {
                PSMovePose pose = m_hmdView->getDisplayHmdPose();
                glm::quat orientation(pose.Orientation.w, pose.Orientation.x, pose.Orientation.y, pose.Orientation.z);
                glm::vec3 position(pose.Position.x, pose.Position.y, pose.Position.z);

                glm::mat4 rot = glm::mat4_cast(orientation);
                glm::mat4 trans = glm::translate(glm::mat4(1.0f), position);
                glm::mat4 transform = trans * rot;

                drawDK2Model(transform);
                drawTransformedAxes(transform, 10.f);
            }

            {
                PSMoveVolume volume;

                if (m_app->getOpenVRContext()->getHMDTrackingVolume(volume))
                {
                    drawTransformedVolume(glm::mat4(1.f), &volume, glm::vec3(0.f, 1.f, 1.f));
                }
            }
        } break;
    case eMenuState::verifyTrackers:
        {
            render_tracker_video();
        } break;
    case eMenuState::selectCalibrationType:
        break;
    case eMenuState::calibrateWithHMD:
        m_pCalibrateWithHMD->render();
        break;
    case eMenuState::calibrateWithMat:
        m_pCalibrateWithMat->render();
        break;
    case eMenuState::testTracking:
        {
            // Draw the origin axes
            drawTransformedAxes(glm::mat4(1.0f), 100.f);

            // Draw the HMD and tracking volume
            if (m_hmdView != nullptr)
            {
                // Compute a transform that goes from HMD tracking space to PSMove tracking space
                PSMovePose hmd_pose_at_origin = m_app->getOpenVRContext()->getHMDPoseAtPSMoveTrackingSpaceOrigin();
                glm::mat4 tracking_space_transform = psmove_pose_to_glm_mat4(hmd_pose_at_origin);
                glm::mat4 tracking_space_inv_transform = glm::inverse(tracking_space_transform);

                // Put the HMD transform in PSMove tracking space
                PSMovePose hmd_pose = m_hmdView->getDisplayHmdPose();
                glm::mat4 hmd_transform = tracking_space_inv_transform * psmove_pose_to_glm_mat4(hmd_pose);

                drawDK2Model(hmd_transform);
                drawTransformedAxes(hmd_transform, 10.f);

                PSMoveVolume volume;
                if (m_app->getOpenVRContext()->getHMDTrackingVolume(volume))
                {
                    drawTransformedVolume(tracking_space_inv_transform, &volume, glm::vec3(0.f, 1.f, 1.f));
                }
            }

            // Draw the frustum for each tracking camera
            for (t_tracker_state_map_iterator iter = m_trackerViews.begin(); iter != m_trackerViews.end(); ++iter)
            {
                const ClientTrackerView *trackerView = iter->second.trackerView;

                {
                    PSMoveFrustum frustum = trackerView->getTrackerFrustum();

                    drawFrustum(&frustum, k_psmove_frustum_color);
                }

                {
                    PSMovePose pose = trackerView->getTrackerPose();
                    glm::mat4 cameraTransform = psmove_pose_to_glm_mat4(pose);

                    drawTransformedAxes(cameraTransform, 20.f);
                }
            }

            // Draw the psmove model
            {
                PSMovePose pose = m_controllerView->GetPSMoveView().GetPose();
                glm::mat4 worldTransform = psmove_pose_to_glm_mat4(pose);

                drawPSMoveModel(worldTransform, glm::vec3(1.f, 1.f, 1.f));
                drawTransformedAxes(worldTransform, 10.f);
            }

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

    case eMenuState::verifyHMD:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 180));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            if (m_hmdView != nullptr)
            {
                ImGui::Text("Verify that your HMD is tracking correctly");
                ImGui::Separator();

                if (ImGui::Button("Looks Good!"))
                {
                    setState(eMenuState::verifyTrackers);
                }

                if (ImGui::Button("Hmm... Something is wrong."))
                {
                    request_exit_to_app_stage(AppStage_HMDSettings::APP_STAGE_NAME);
                }
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
                ImGui::Text("Tracker #%d", m_renderTrackerIndex + 1);

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
                setState(eMenuState::selectCalibrationType);
            }

            if (ImGui::Button("Hmm... Something is wrong."))
            {
                request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;

    case eMenuState::selectCalibrationType:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Select a camera pose estimation method");

            if (m_hmdView != nullptr && ImGui::Button("Attach PSMove To HMD"))
            {
                setState(eMenuState::calibrateWithHMD);
            }

            if (ImGui::Button("Use Calibration Mat"))
            {
                setState(eMenuState::calibrateWithMat);
            }

            if (ImGui::Button("Cancel"))
            {
                m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;

    case eMenuState::calibrateWithHMD:
        {
            m_pCalibrateWithHMD->renderUI();
        } break;

    case eMenuState::calibrateWithMat:
        {
            m_pCalibrateWithMat->renderUI();
        } break;

    case eMenuState::testTracking:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            if (!m_bSkipCalibration)
            {
                ImGui::Text("Calibration Complete");

                if (ImGui::Button("Redo Calibration"))
                {
                    setState(eMenuState::selectCalibrationType);
                }
            }

            if (ImGui::Button("Exit"))
            {
                m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
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
                setState(eMenuState::selectCalibrationType);
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
    case eMenuState::verifyHMD:
        m_app->setCameraType(_cameraFixed);
        break;
    case eMenuState::verifyTrackers:
    case eMenuState::selectCalibrationType:
        break;
    case eMenuState::calibrateWithHMD:
        m_pCalibrateWithHMD->exit();
        break;
    case eMenuState::calibrateWithMat:
        m_pCalibrateWithMat->exit();
        break;
    case eMenuState::testTracking:
        m_app->setCameraType(_cameraFixed);
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
    case eMenuState::pendingControllerStartRequest:
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
    case eMenuState::verifyHMD:
        m_app->setCameraType(_cameraOrbit);
        break;
    case eMenuState::verifyTrackers:
        m_renderTrackerIter = m_trackerViews.begin();
        break;
    case eMenuState::selectCalibrationType:
        break;
    case eMenuState::calibrateWithHMD:
        m_pCalibrateWithHMD->enter();
        break;
    case eMenuState::calibrateWithMat:
        m_pCalibrateWithMat->enter();
        break;
    case eMenuState::testTracking:
        m_app->setCameraType(_cameraOrbit);
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
        // Render the latest from the currently active tracker
        if (m_renderTrackerIter->second.trackerView->pollVideoStream())
        {
            m_renderTrackerIter->second.textureAsset->copyBufferIntoTexture(
                m_renderTrackerIter->second.trackerView->getVideoFrameBuffer());
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

class ClientTrackerView *AppStage_ComputeTrackerPoses::get_render_tracker_view() const
{
    return (m_trackerViews.size() > 0) ? m_renderTrackerIter->second.trackerView : nullptr;
}

void AppStage_ComputeTrackerPoses::release_devices()
{
    //###HipsterSloth $REVIEW Do we care about canceling in-flight requests?

    if (m_hmdView != nullptr)
    {
        m_app->getOpenVRContext()->freeHmdView(m_hmdView);
        m_hmdView = nullptr;
    }

    if (m_controllerView != nullptr)
    {
        ClientPSMoveAPI::eat_response(ClientPSMoveAPI::stop_controller_data_stream(m_controllerView));
        ClientPSMoveAPI::free_controller_view(m_controllerView);
        m_controllerView = nullptr;
    }

    for (t_tracker_state_map_iterator iter = m_trackerViews.begin(); iter != m_trackerViews.end(); ++iter)
    {
        TrackerState &trackerState = iter->second;

        if (trackerState.textureAsset != nullptr)
        {
            delete trackerState.textureAsset;
        }

        if (trackerState.trackerView != nullptr)
        {
            trackerState.trackerView->closeVideoStream();

            ClientPSMoveAPI::eat_response(ClientPSMoveAPI::stop_tracker_data_stream(trackerState.trackerView));
            ClientPSMoveAPI::free_tracker_view(trackerState.trackerView);
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
        ClientPSMoveAPI::register_callback(
            ClientPSMoveAPI::get_controller_list(),
            AppStage_ComputeTrackerPoses::handle_controller_list_response, this);
    }
}

void AppStage_ComputeTrackerPoses::handle_controller_list_response(
    const ClientPSMoveAPI::ResponseMessage *response_message,
    void *userdata)
{
    AppStage_ComputeTrackerPoses *thisPtr = static_cast<AppStage_ComputeTrackerPoses *>(userdata);

    const ClientPSMoveAPI::eClientPSMoveResultCode ResultCode = response_message->result_code;
//    const ClientPSMoveAPI::t_request_id request_id = response_message->request_id;

    switch (ResultCode)
    {
    case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
        {
            assert(response_message->payload_type == ClientPSMoveAPI::_responsePayloadType_ControllerList);
            const ClientPSMoveAPI::ResponsePayload_ControllerList *controller_list = 
                &response_message->payload.controller_list;

            int PSMoveControllerId = -1;
            for (int list_index = 0; list_index < controller_list->count; ++list_index)
            {
                if (controller_list->controller_type[list_index] == ClientControllerView::PSMove)
                {
                    PSMoveControllerId = controller_list->controller_id[list_index];
                    break;
                }
            }

            if (PSMoveControllerId != -1)
            {
                thisPtr->request_start_controller_stream(PSMoveControllerId);
            }
            else
            {
                thisPtr->setState(AppStage_ComputeTrackerPoses::failedControllerListRequest);
            }
        } break;

    case ClientPSMoveAPI::_clientPSMoveResultCode_error:
    case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
        {
            thisPtr->setState(AppStage_ComputeTrackerPoses::failedControllerListRequest);
        } break;
    }
}

void AppStage_ComputeTrackerPoses::request_start_controller_stream(int ControllerID)
{
    // Allocate a controller view to track controller state
    assert(m_controllerView == nullptr);
    m_controllerView= ClientPSMoveAPI::allocate_controller_view(ControllerID);

    // Start receiving data from the controller
    setState(AppStage_ComputeTrackerPoses::pendingControllerStartRequest);
    ClientPSMoveAPI::register_callback(
        ClientPSMoveAPI::start_controller_data_stream(
            m_controllerView, 
            ClientPSMoveAPI::includePositionData | ClientPSMoveAPI::includeRawSensorData | ClientPSMoveAPI::includeRawTrackerData),
        AppStage_ComputeTrackerPoses::handle_start_controller_response, this);
}

void AppStage_ComputeTrackerPoses::handle_start_controller_response(
    const ClientPSMoveAPI::ResponseMessage *response_message,
    void *userdata)
{
    AppStage_ComputeTrackerPoses *thisPtr = static_cast<AppStage_ComputeTrackerPoses *>(userdata);

    const ClientPSMoveAPI::eClientPSMoveResultCode ResultCode = response_message->result_code;
//    const ClientPSMoveAPI::t_request_id request_id = response_message->request_id;

    switch (ResultCode)
    {
    case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
        {
            thisPtr->request_tracker_list();
        } break;

    case ClientPSMoveAPI::_clientPSMoveResultCode_error:
    case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
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
        ClientPSMoveAPI::register_callback(
            ClientPSMoveAPI::get_tracker_list(),
            AppStage_ComputeTrackerPoses::handle_tracker_list_response, this);
    }
}

void AppStage_ComputeTrackerPoses::handle_tracker_list_response(
    const ClientPSMoveAPI::ResponseMessage *response_message,
    void *userdata)
{
    AppStage_ComputeTrackerPoses *thisPtr = static_cast<AppStage_ComputeTrackerPoses *>(userdata);

    switch (response_message->result_code)
    {
    case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
        {
            assert(response_message->payload_type == ClientPSMoveAPI::_responsePayloadType_TrackerList);
            const ClientPSMoveAPI::ResponsePayload_TrackerList &tracker_list = response_message->payload.tracker_list;

            for (int tracker_index = 0; tracker_index < tracker_list.count; ++tracker_index)
            {
                const ClientTrackerInfo *TrackerInfo = &tracker_list.trackers[tracker_index];

                thisPtr->request_tracker_start_stream(TrackerInfo, tracker_index);
            }
        } break;

    case ClientPSMoveAPI::_clientPSMoveResultCode_error:
    case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
        {
            thisPtr->setState(eMenuState::failedTrackerListRequest);
        } break;
    }
}

void AppStage_ComputeTrackerPoses::request_tracker_start_stream(
    const ClientTrackerInfo *TrackerInfo,
    int listIndex)
{
    TrackerState trackerState;

    setState(eMenuState::pendingTrackerStartRequest);

    // Allocate a new tracker view
    trackerState.listIndex = listIndex;
    trackerState.trackerView = ClientPSMoveAPI::allocate_tracker_view(*TrackerInfo);
    trackerState.textureAsset = nullptr;

    // Add the tracker to the list of trackers we're monitoring
    assert(m_trackerViews.find(TrackerInfo->tracker_id) == m_trackerViews.end());
    m_trackerViews.insert(t_id_tracker_state_pair(TrackerInfo->tracker_id, trackerState));

    // Increment the number of requests we're waiting to get back
    ++m_pendingTrackerStartCount;

    // Request data to start streaming to the tracker
    ClientPSMoveAPI::register_callback(
        ClientPSMoveAPI::start_tracker_data_stream(trackerState.trackerView),
        AppStage_ComputeTrackerPoses::handle_tracker_start_stream_response, this);
}

void AppStage_ComputeTrackerPoses::handle_tracker_start_stream_response(
    const ClientPSMoveAPI::ResponseMessage *response_message,
    void *userdata)
{
    AppStage_ComputeTrackerPoses *thisPtr = static_cast<AppStage_ComputeTrackerPoses *>(userdata);

    switch (response_message->result_code)
    {
    case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
        {
            // Get the tracker ID this request was for
            const PSMoveProtocol::Request *request = GET_PSMOVEPROTOCOL_REQUEST(response_message->opaque_request_handle);
            const int tracker_id= request->request_start_tracker_data_stream().tracker_id();

            // Get the tracker state associated with the tracker id
            t_tracker_state_map_iterator trackerStateEntry = thisPtr->m_trackerViews.find(tracker_id);
            assert(trackerStateEntry != thisPtr->m_trackerViews.end());

            // The context holds everything a handler needs to evaluate a response
            TrackerState &trackerState = trackerStateEntry->second;

            // Open the shared memory that the video stream is being written to
            if (trackerState.trackerView->openVideoStream())
            {
                // Create a texture to render the video frame to
                trackerState.textureAsset = new TextureAsset();
                trackerState.textureAsset->init(
                    trackerState.trackerView->getVideoFrameWidth(),
                    trackerState.trackerView->getVideoFrameHeight(),
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

    case ClientPSMoveAPI::_clientPSMoveResultCode_error:
    case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
        {
            thisPtr->setState(eMenuState::failedTrackerStartRequest);
        } break;
    }
}

static void copy_pose_to_request(
    const PSMovePose &pose,
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
    const PSMovePose *pose,
    class ClientTrackerView *TrackerView)
{
    // Set the pose on out local tracker view
    {
        ClientTrackerInfo &trackerInfo= TrackerView->getTrackerInfoMutable();

        trackerInfo.tracker_pose = *pose;
    }

    // Update the pose on the service
    {
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_POSE);

        PSMoveProtocol::Request_RequestSetTrackerPose *set_pose_request =
            request->mutable_request_set_tracker_pose();

        set_pose_request->set_tracker_id(TrackerView->getTrackerId());
        copy_pose_to_request(TrackerView->getTrackerPose(), set_pose_request->mutable_pose());

        ClientPSMoveAPI::eat_response(ClientPSMoveAPI::send_opaque_request(&request));
    }
}

void AppStage_ComputeTrackerPoses::request_set_hmd_tracking_space_origin(
    const struct PSMovePose *pose)
{
    if (m_app->getOpenVRContext()->getIsInitialized())
    {
        m_app->getOpenVRContext()->setHMDTrackingSpaceOrigin(*pose);
    }

    // Update the pose on the service
    {
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_SET_HMD_TRACKING_SPACE_ORIGIN);

        PSMoveProtocol::Request_RequestSetHMDTrackingSpaceOrigin *set_origin_request =
            request->mutable_request_set_hmd_tracking_space_origin();

        copy_pose_to_request(*pose, set_origin_request->mutable_origin_pose());

        ClientPSMoveAPI::eat_response(ClientPSMoveAPI::send_opaque_request(&request));
    }
}

void AppStage_ComputeTrackerPoses::handle_all_devices_ready()
{
    if (!m_bSkipCalibration)
    {
        if (m_hmdView != nullptr)
        {
            setState(eMenuState::verifyHMD);
        }
        else
        {
            setState(eMenuState::verifyTrackers);
        }
    }
    else
    {
        setState(eMenuState::testTracking);
    }
}
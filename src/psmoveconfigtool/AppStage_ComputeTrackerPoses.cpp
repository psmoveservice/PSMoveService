//-- inludes -----
#include "AppStage_ComputeTrackerPoses.h"
#include "AppStage_MainMenu.h"
#include "AppStage_TrackerSettings.h"
#include "App.h"
#include "AssetManager.h"
#include "Camera.h"
#include "ClientHMDView.h"
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

//-- public methods -----
AppStage_ComputeTrackerPoses::AppStage_ComputeTrackerPoses(App *app)
    : AppStage(app)
    , m_menuState(AppStage_ComputeTrackerPoses::inactive)
    , m_hmdView(nullptr)
    , m_controllerView(nullptr)
{ }

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
    if (m_hmdView != nullptr)
    {
        m_app->getOpenVRContext()->freeHmdView(m_hmdView);
        m_hmdView = nullptr;
    }

    m_menuState = eMenuState::inactive;
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
    case eMenuState::selectCalibrationType:
        break;
    case eMenuState::calibrationStepOriginPlacement:
        break;
    case eMenuState::calibrationStepPlacePSMove:
        break;
    case eMenuState::calibrationStepRecordPSMove:
        break;
    case eMenuState::calibrationStepPlaceHMD:
        break;
    case eMenuState::calibrationStepRecordHMD:
        break;
    case eMenuState::calibrationStepAttachPSMove:
        break;
    case eMenuState::calibrationStepRecordHmdPSMove:
        break;
    case eMenuState::calibrateStepComplete:
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
        break;
    case eMenuState::failedControllerListRequest:
        break;
    case eMenuState::pendingControllerStartRequest:
        break;
    case eMenuState::failedControllerStartRequest:
        break;
    case eMenuState::pendingTrackerStartRequest:
        break;
    case eMenuState::failedTrackerStartRequest:
        break;
    case eMenuState::selectCalibrationType:
        break;
    case eMenuState::calibrationStepOriginPlacement:
        break;
    case eMenuState::calibrationStepPlacePSMove:
        break;
    case eMenuState::calibrationStepRecordPSMove:
        break;
    case eMenuState::calibrationStepPlaceHMD:
        break;
    case eMenuState::calibrationStepRecordHMD:
        break;
    case eMenuState::calibrationStepAttachPSMove:
        break;
    case eMenuState::calibrationStepRecordHmdPSMove:
        break;
    case eMenuState::calibrateStepComplete:
        break;
    case eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }

    //if (m_menuState == eMenuState::idle)
    //{
    //    if (m_hmdView != nullptr)
    //    {
    //        PSMovePose pose = m_hmdView->getHmdPose();
    //        glm::quat orientation(pose.Orientation.w, pose.Orientation.x, pose.Orientation.y, pose.Orientation.z);
    //        glm::vec3 position(pose.Position.x, pose.Position.y, pose.Position.z);

    //        glm::mat4 rot = glm::mat4_cast(orientation);
    //        glm::mat4 trans = glm::translate(glm::mat4(1.0f), position);
    //        glm::mat4 transform = trans * rot;

    //        drawDK2Model(transform);
    //        drawTransformedAxes(transform, 10.f);
    //    }
    //}
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
    case eMenuState::pendingTrackerStartRequest:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 50));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Pending controller setup...");

            if (ImGui::Button("Return to Tracker Settings"))
            {
                request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;

    case eMenuState::failedControllerListRequest:
    case eMenuState::failedControllerStartRequest:
    case eMenuState::failedTrackerStartRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Failed controller setup!");

            if (ImGui::Button("Ok"))
            {
                m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }

            if (ImGui::Button("Return to Main Menu"))
            {
                m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;

    case eMenuState::selectCalibrationType:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            if (m_hmdView != nullptr)
            {
                ImGui::Text("Select a camera pose estimation method");

                if (ImGui::Button("Attach PSMove To HMD"))
                {
                    m_menuState = eMenuState::calibrationStepAttachPSMove;
                }

                if (ImGui::Button("Use Calibration Mat"))
                {
                    m_menuState = eMenuState::calibrationStepOriginPlacement;
                }

                if (ImGui::Button("Cancel"))
                {
                    m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
                }
            }

            ImGui::End();
        } break;

    case eMenuState::calibrationStepOriginPlacement:
        break;
    case eMenuState::calibrationStepPlacePSMove:
        break;
    case eMenuState::calibrationStepRecordPSMove:
        break;
    case eMenuState::calibrationStepPlaceHMD:
        break;
    case eMenuState::calibrationStepRecordHMD:
        break;
    case eMenuState::calibrationStepAttachPSMove:
        break;
    case eMenuState::calibrationStepRecordHmdPSMove:
        break;
    case eMenuState::calibrateStepComplete:
        break;
    case eMenuState::calibrateStepFailed:
        break;

    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ComputeTrackerPoses::request_exit_to_app_stage(const char *app_stage_name)
{
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
    const ClientPSMoveAPI::t_request_id request_id = response_message->request_id;

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
                thisPtr->m_menuState = AppStage_ComputeTrackerPoses::failedControllerListRequest;
            }
        } break;

    case ClientPSMoveAPI::_clientPSMoveResultCode_error:
    case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
        {
            thisPtr->m_menuState = AppStage_ComputeTrackerPoses::failedControllerListRequest;
        } break;
    }
}

void AppStage_ComputeTrackerPoses::request_start_controller_stream(int ControllerID)
{
    // Allocate a controller view to track controller state
    assert(m_controllerView == nullptr);
    m_controllerView= ClientPSMoveAPI::allocate_controller_view(ControllerID);

    // Start receiving data from the controller
    m_menuState = AppStage_ComputeTrackerPoses::pendingControllerStartRequest;
    ClientPSMoveAPI::register_callback(
        ClientPSMoveAPI::start_controller_data_stream(m_controllerView, ClientPSMoveAPI::defaultStreamOptions),
        AppStage_ComputeTrackerPoses::handle_start_controller_response, this);
}

void AppStage_ComputeTrackerPoses::handle_start_controller_response(
    const ClientPSMoveAPI::ResponseMessage *response_message,
    void *userdata)
{
    AppStage_ComputeTrackerPoses *thisPtr = static_cast<AppStage_ComputeTrackerPoses *>(userdata);

    const ClientPSMoveAPI::eClientPSMoveResultCode ResultCode = response_message->result_code;
    const ClientPSMoveAPI::t_request_id request_id = response_message->request_id;

    switch (ResultCode)
    {
    case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
        {
            thisPtr->request_tracker_list();
        } break;

    case ClientPSMoveAPI::_clientPSMoveResultCode_error:
    case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
        {
            thisPtr->m_menuState = AppStage_ComputeTrackerPoses::failedControllerStartRequest;
        } break;
    }
}

void AppStage_ComputeTrackerPoses::request_tracker_list()
{
    if (m_menuState != eMenuState::pendingTrackerListRequest)
    {
        m_menuState = eMenuState::pendingTrackerListRequest;
        m_trackerViews.clear();
        m_pendingTrackerStartCount = 0;

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

                thisPtr->request_tracker_start_stream(TrackerInfo);
            }
        } break;

    case ClientPSMoveAPI::_clientPSMoveResultCode_error:
    case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
        {
            thisPtr->m_menuState = eMenuState::failedTrackerListRequest;
        } break;
    }
}

void AppStage_ComputeTrackerPoses::request_tracker_start_stream(
    const ClientTrackerInfo *TrackerInfo)
{
    m_menuState = eMenuState::pendingTrackerStartRequest;

    TrackerState trackerState;

    // Allocate a new tracker view
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
            const PSMoveProtocol::Request *response = GET_PSMOVEPROTOCOL_REQUEST(response_message->opaque_request_handle);
            const int tracker_id= response->request_start_tracker_data_stream().tracker_id();

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
                thisPtr->m_menuState = eMenuState::selectCalibrationType;
            }
        } break;

    case ClientPSMoveAPI::_clientPSMoveResultCode_error:
    case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
        {
            thisPtr->m_menuState = eMenuState::failedTrackerStartRequest;
        } break;
    }
}
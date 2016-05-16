//-- inludes -----
#include "AppStage_TrackerSettings.h"
#include "AppStage_TestTracker.h"
#include "AppStage_ComputeTrackerPoses.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "ClientPSMoveAPI.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"

#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>

//-- statics ----
const char *AppStage_TrackerSettings::APP_STAGE_NAME= "CameraSettings";

//-- constants -----

//-- public methods -----
AppStage_TrackerSettings::AppStage_TrackerSettings(App *app) 
    : AppStage(app)
    , m_menuState(AppStage_TrackerSettings::inactive)
    , m_selectedTrackerIndex(-1)
{ }

void AppStage_TrackerSettings::enter()
{
    m_app->setCameraType(_cameraFixed);
    m_selectedTrackerIndex = -1;

    request_tracker_list();
}

void AppStage_TrackerSettings::exit()
{
}

void AppStage_TrackerSettings::update()
{
}
    
void AppStage_TrackerSettings::render()
{
    switch (m_menuState)
    {
    case eTrackerMenuState::idle:
    {
        if (m_selectedTrackerIndex >= 0)
        {
            const ClientTrackerInfo &trackerInfo = m_trackerInfos[m_selectedTrackerIndex];

            switch (trackerInfo.tracker_type)
            {
            case PSMoveProtocol::PS3EYE:
                {
                    glm::mat4 scale3 = glm::scale(glm::mat4(1.f), glm::vec3(3.f, 3.f, 3.f));
                    drawPS3EyeModel(scale3);
                } break;
            default:
                assert(0 && "Unreachable");
            }
        }
    } break;

    case eTrackerMenuState::pendingTrackerListRequest:
    case eTrackerMenuState::failedTrackerListRequest:
    {
    } break;

    default:
        assert(0 && "unreachable");
    }
}

void AppStage_TrackerSettings::renderUI()
{
    const char *k_window_title = "Tracker Settings";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case eTrackerMenuState::idle:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(300, 400));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        if (m_trackerInfos.size() > 0)
        {
            const ClientTrackerInfo &trackerInfo = m_trackerInfos[m_selectedTrackerIndex];

            ImGui::Text("Tracker: %d", m_selectedTrackerIndex);
            ImGui::Text("  Tracker ID: %d", trackerInfo.tracker_id);

            switch (trackerInfo.tracker_type)
            {
            case eTrackerType::PS3Eye:
                {
                    ImGui::Text("  Controller Type: PS3 Eye");
                } break;
            default:
                assert(0 && "Unreachable");
            }

            switch (trackerInfo.tracker_driver)
            {
            case eTrackerDriver::LIBUSB:
                {
                    ImGui::Text("  Controller Type: LIBUSB");
                } break;
            case eTrackerDriver::CL_EYE:
                {
                    ImGui::Text("  Controller Type: CLEye");
                } break;
            case eTrackerDriver::CL_EYE_MULTICAM:
                {
                    ImGui::Text("  Controller Type: CLEye(Multicam SDK)");
                } break;
            case eTrackerDriver::GENERIC_WEBCAM:
                {
                    ImGui::Text("  Controller Type: Generic Webcam");
                } break;
            default:
                assert(0 && "Unreachable");
            }

            ImGui::Text("  Shared Mem Name: %s", trackerInfo.shared_memory_name);
            ImGui::TextWrapped("  Device Path: %s", trackerInfo.device_path);

            if (ImGui::Button("Compute Tracker Poses"))
            {
                m_app->setAppStage(AppStage_ComputeTrackerPoses::APP_STAGE_NAME);
            }

            if (m_selectedTrackerIndex > 0)
            {
                if (ImGui::Button("Previous Tracker"))
                {
                    --m_selectedTrackerIndex;
                }
            }

            if (m_selectedTrackerIndex + 1 < static_cast<int>(m_trackerInfos.size()))
            {
                if (ImGui::Button("Next Tracker"))
                {
                    ++m_selectedTrackerIndex;
                }
            }

            // TODO: Localhost only check
            if (ImGui::Button("Test Video Feed"))
            {
                m_app->setAppStage(AppStage_TestTracker::APP_STAGE_NAME);
            }
        }
        else
        {
            ImGui::Text("No trackers controllers");
        }

        if (ImGui::Button("Return to Main Menu"))
        {
            m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
        }

        ImGui::End();
    } break;
    case eTrackerMenuState::pendingTrackerListRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(300, 150));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Waiting for tracker list response...");

        ImGui::End();
    } break;
    case eTrackerMenuState::failedTrackerListRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(300, 150));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Failed to get tracker list!");

        if (ImGui::Button("Retry"))
        {
            request_tracker_list();
        }

        if (ImGui::Button("Return to Main Menu"))
        {
            m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
        }

        ImGui::End();
    } break;

    default:
        assert(0 && "unreachable");
    }
}

bool AppStage_TrackerSettings::onClientAPIEvent(
    ClientPSMoveAPI::eEventType event,
    ClientPSMoveAPI::t_event_data_handle opaque_event_handle)
{
    bool bHandled = false;

    switch (event)
    {
    case ClientPSMoveAPI::controllerListUpdated:
        {
            bHandled = true;
            request_tracker_list();
        } break;
    }

    return bHandled;
}

void AppStage_TrackerSettings::request_tracker_list()
{
    if (m_menuState != AppStage_TrackerSettings::pendingTrackerListRequest)
    {
        m_menuState = AppStage_TrackerSettings::pendingTrackerListRequest;
        m_selectedTrackerIndex = -1;
        m_trackerInfos.clear();

        // Tell the psmove service that we we want a list of trackers connected to this machine
        ClientPSMoveAPI::register_callback(
            ClientPSMoveAPI::get_tracker_list(), 
            AppStage_TrackerSettings::handle_tracker_list_response, this);
    }
}

void AppStage_TrackerSettings::handle_tracker_list_response(
    const ClientPSMoveAPI::ResponseMessage *response_message,
    void *userdata)
{
    AppStage_TrackerSettings *thisPtr = static_cast<AppStage_TrackerSettings *>(userdata);

    switch (response_message->result_code)
    {
    case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
        {
            assert(response_message->payload_type == ClientPSMoveAPI::_responsePayloadType_TrackerList);
            const ClientPSMoveAPI::ResponsePayload_TrackerList &tracker_list= response_message->payload.tracker_list;

            for (int tracker_index = 0; tracker_index < tracker_list.count; ++tracker_index)
            {
                const ClientTrackerInfo &TrackerInfo = tracker_list.trackers[tracker_index];

                thisPtr->m_trackerInfos.push_back(TrackerInfo);
            }

            thisPtr->m_selectedTrackerIndex = (thisPtr->m_trackerInfos.size() > 0) ? 0 : -1;
            thisPtr->m_menuState = AppStage_TrackerSettings::idle;
        } break;

    case ClientPSMoveAPI::_clientPSMoveResultCode_error:
    case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
        {
            thisPtr->m_menuState = AppStage_TrackerSettings::failedTrackerListRequest;
        } break;
    }
}
//-- inludes -----
#include "AppStage_TrackerSettings.h"
#include "AppStage_TestTracker.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
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
            const TrackerInfo &trackerInfo = m_trackerInfos[m_selectedTrackerIndex];

            switch (trackerInfo.TrackerType)
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
            const TrackerInfo &trackerInfo = m_trackerInfos[m_selectedTrackerIndex];

            ImGui::Text("Tracker: %d", m_selectedTrackerIndex);
            ImGui::Text("  Tracker ID: %d", trackerInfo.TrackerID);

            switch (trackerInfo.TrackerType)
            {
            case AppStage_TrackerSettings::PS3Eye:
            {
                ImGui::Text("  Controller Type: PS3 Eye");
            } break;
            default:
                assert(0 && "Unreachable");
            }

            switch (trackerInfo.TrackerDriver)
            {
            case AppStage_TrackerSettings::LIBUSB:
            {
                ImGui::Text("  Controller Type: LIBUSB");
            } break;
            case AppStage_TrackerSettings::CL_EYE:
            {
                ImGui::Text("  Controller Type: CLEye");
            } break;
            case AppStage_TrackerSettings::CL_EYE_MULTICAM:
            {
                ImGui::Text("  Controller Type: CLEye(Multicam SDK)");
            } break;
            case AppStage_TrackerSettings::GENERIC_WEBCAM:
            {
                ImGui::Text("  Controller Type: Generic Webcam");
            } break;
            default:
                assert(0 && "Unreachable");
            }

            ImGui::Text("  Shared Mem Name: %s", trackerInfo.SharedMemoryName.c_str());
            ImGui::TextWrapped("  Device Path: %s", trackerInfo.DevicePath.c_str());

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
    ClientPSMoveAPI::eClientPSMoveAPIEvent event,
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
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_GET_TRACKER_LIST);

        m_app->registerCallback(
            ClientPSMoveAPI::send_opaque_request(&request), 
            AppStage_TrackerSettings::handle_tracker_list_response, this);
    }
}

void AppStage_TrackerSettings::handle_tracker_list_response(
    ClientPSMoveAPI::eClientPSMoveResultCode ResultCode,
    const ClientPSMoveAPI::t_request_id request_id,
    ClientPSMoveAPI::t_response_handle response_handle,
    void *userdata)
{
    AppStage_TrackerSettings *thisPtr = static_cast<AppStage_TrackerSettings *>(userdata);

    switch (ResultCode)
    {
    case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
        {
            const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);

            for (int tracker_index = 0; tracker_index < response->result_tracker_list().trackers_size(); ++tracker_index)
            {
                const auto &TrackerResponse = response->result_tracker_list().trackers(tracker_index);

                AppStage_TrackerSettings::TrackerInfo TrackerInfo;

                TrackerInfo.TrackerID = TrackerResponse.tracker_id();

                switch (TrackerResponse.tracker_type())
                {
                case PSMoveProtocol::TrackerType::PS3EYE:
                    TrackerInfo.TrackerType = AppStage_TrackerSettings::PS3Eye;
                    break;
                default:
                    assert(0 && "unreachable");
                }

                switch (TrackerResponse.tracker_driver())
                {
                case PSMoveProtocol::TrackerDriver::LIBUSB:
                    TrackerInfo.TrackerDriver = AppStage_TrackerSettings::LIBUSB;
                    break;
                case PSMoveProtocol::TrackerDriver::CL_EYE:
                    TrackerInfo.TrackerDriver = AppStage_TrackerSettings::CL_EYE;
                    break;
                case PSMoveProtocol::TrackerDriver::CL_EYE_MULTICAM:
                    TrackerInfo.TrackerDriver = AppStage_TrackerSettings::CL_EYE_MULTICAM;
                    break;
                case PSMoveProtocol::TrackerDriver::GENERIC_WEBCAM:
                    TrackerInfo.TrackerDriver = AppStage_TrackerSettings::GENERIC_WEBCAM;
                    break;
                default:
                    assert(0 && "unreachable");
                }

                TrackerInfo.DevicePath = TrackerResponse.device_path();
                TrackerInfo.SharedMemoryName = TrackerResponse.shared_memory_name();

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
//-- inludes -----
#include "AppStage_TrackerSettings.h"
#include "AppStage_TestTracker.h"
#include "AppStage_ColorCalibration.h"
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
	, m_selectedControllerIndex(-1)
{ }

void AppStage_TrackerSettings::enter()
{
    m_app->setCameraType(_cameraFixed);

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

    case eTrackerMenuState::pendingSearchForNewTrackersRequest:
    case eTrackerMenuState::pendingTrackerListRequest:
    case eTrackerMenuState::failedTrackerListRequest:
	case eTrackerMenuState::pendingControllerListRequest:
	case eTrackerMenuState::failedControllerListRequest:
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

        //###HipsterSloth $TODO The tracker restart currently takes longer than it does
        // just to close and re-open the service.
        // For now let's just disable this until we can make this more performant.
        //if (ImGui::Button("Refresh Tracker List"))
        //{
        //    request_search_for_new_trackers();
        //}

        //ImGui::Separator();

        if (m_trackerInfos.size() > 0)
        {
            const ClientTrackerInfo &trackerInfo = m_trackerInfos[m_selectedTrackerIndex];

            if (m_selectedTrackerIndex > 0)
            {
                if (ImGui::Button("<##TrackerIndex"))
                {
                    --m_selectedTrackerIndex;
                }
                ImGui::SameLine();
            }
            ImGui::Text("Tracker: %d", m_selectedTrackerIndex);
            if (m_selectedTrackerIndex + 1 < static_cast<int>(m_trackerInfos.size()))
            {
                ImGui::SameLine();
                if (ImGui::Button(">##TrackerIndex"))
                {
                    ++m_selectedTrackerIndex;
                }
            }

            ImGui::BulletText("Tracker ID: %d", trackerInfo.tracker_id);

            switch (trackerInfo.tracker_type)
            {
            case eTrackerType::PS3Eye:
                {
                    ImGui::BulletText("Controller Type: PS3 Eye");
                } break;
            default:
                assert(0 && "Unreachable");
            }

            switch (trackerInfo.tracker_driver)
            {
            case eTrackerDriver::LIBUSB:
                {
                    ImGui::BulletText("Controller Type: LIBUSB");
                } break;
            case eTrackerDriver::CL_EYE:
                {
                    ImGui::BulletText("Controller Type: CLEye");
                } break;
            case eTrackerDriver::CL_EYE_MULTICAM:
                {
                    ImGui::BulletText("Controller Type: CLEye(Multicam SDK)");
                } break;
            case eTrackerDriver::GENERIC_WEBCAM:
                {
                    ImGui::BulletText("Controller Type: Generic Webcam");
                } break;
            default:
                assert(0 && "Unreachable");
            }

            ImGui::BulletText("Shared Mem Name: %s", trackerInfo.shared_memory_name);
            ImGui::BulletText("Device Path: ");
            ImGui::SameLine();
            ImGui::TextWrapped("%s", trackerInfo.device_path);

            //###HipsterSloth $TODO: Localhost only check
            if (ImGui::Button("Test Tracker Video Feed"))
            {
                m_app->setAppStage(AppStage_TestTracker::APP_STAGE_NAME);
            }
        }
        else
        {
            ImGui::Text("No trackers controllers");
        }

        ImGui::Separator();

        if (m_trackerInfos.size() > 0)
        {
			if (m_controllerInfos.size() > 0)
			{
				if (m_selectedControllerIndex >= 0)
				{
					if (ImGui::Button("<##Controller"))
					{
						--m_selectedControllerIndex;
					}
					ImGui::SameLine();
				}
				
				if (m_selectedControllerIndex != -1)
				{
					const AppStage_TrackerSettings::ControllerInfo &controllerInfo = 
						m_controllerInfos[m_selectedControllerIndex];

					if (controllerInfo.ControllerType == ClientControllerView::PSMove)
					{ 
						ImGui::Text("Controller: %d (PSMove)", m_selectedControllerIndex);
					}
					else
					{
						ImGui::Text("Controller: %d (DualShock4)", m_selectedControllerIndex);
					}
				}
				else
				{
					ImGui::Text("Controller: <ALL>");
				}

				if (m_selectedControllerIndex + 1 < static_cast<int>(m_controllerInfos.size()))
				{
					ImGui::SameLine();
					if (ImGui::Button(">##Controller"))
					{
						++m_selectedControllerIndex;
					}
				}
			}			

            //###HipsterSloth $TODO: Localhost only check
            if (ImGui::Button("Calibrate Tracking Colors"))
            {
				int controller_id= -1;
				if (m_selectedControllerIndex != -1)
				{ 
					const AppStage_TrackerSettings::ControllerInfo &controllerInfo = 
						m_controllerInfos[m_selectedControllerIndex];

					controller_id = controllerInfo.ControllerID;
				}

				m_app->getAppStage<AppStage_ColorCalibration>()->set_override_controller_id(controller_id);
                m_app->setAppStage(AppStage_ColorCalibration::APP_STAGE_NAME);
            }
        }


        ImGui::Separator();

        if (m_trackerInfos.size() > 0)
        {
            if (ImGui::Button("Compute Tracker Poses"))
            {
                AppStage_ComputeTrackerPoses::enterStageAndCalibrate(m_app);
            }

            if (ImGui::Button("Test Tracking"))
            {
                AppStage_ComputeTrackerPoses::enterStageAndSkipCalibration(m_app);
            }
        }

        if (ImGui::Button("Return to Main Menu"))
        {
            m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
        }

        ImGui::End();
    } break;
    case eTrackerMenuState::pendingSearchForNewTrackersRequest:
    case eTrackerMenuState::pendingTrackerListRequest:
	case eTrackerMenuState::pendingControllerListRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(300, 150));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Waiting for server response...");

        ImGui::End();
    } break;
    case eTrackerMenuState::failedTrackerListRequest:
	case eTrackerMenuState::failedControllerListRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(300, 150));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Failed to get server response!");

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
			int oldSelectedTrackerIndex= thisPtr->m_selectedTrackerIndex;

			thisPtr->m_selectedTrackerIndex = -1;
			thisPtr->m_trackerInfos.clear();

            for (int tracker_index = 0; tracker_index < tracker_list.count; ++tracker_index)
            {
                const ClientTrackerInfo &TrackerInfo = tracker_list.trackers[tracker_index];

                thisPtr->m_trackerInfos.push_back(TrackerInfo);
            }

			if (oldSelectedTrackerIndex != -1)
			{
				// Maintain the same position in the list if possible
				thisPtr->m_selectedTrackerIndex= 
					(oldSelectedTrackerIndex < thisPtr->m_trackerInfos.size()) 
					? oldSelectedTrackerIndex
					: 0;
			}
			else
			{
	            thisPtr->m_selectedTrackerIndex= (thisPtr->m_trackerInfos.size() > 0) ? 0 : -1;
			}

			// Request the list of controllers next
            thisPtr->request_controller_list();
        } break;

    case ClientPSMoveAPI::_clientPSMoveResultCode_error:
    case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
        {
            thisPtr->m_menuState = AppStage_TrackerSettings::failedTrackerListRequest;
        } break;
    }
}

void AppStage_TrackerSettings::request_controller_list()
{
    if (m_menuState != AppStage_TrackerSettings::pendingControllerListRequest)
    {
        m_menuState= AppStage_TrackerSettings::pendingControllerListRequest;

        // Tell the psmove service that we we want a list of controllers connected to this machine
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_GET_CONTROLLER_LIST);

        // Don't need the usb controllers
        request->mutable_request_get_controller_list()->set_include_usb_controllers(false);

        ClientPSMoveAPI::register_callback(
            ClientPSMoveAPI::send_opaque_request(&request), 
            AppStage_TrackerSettings::handle_controller_list_response, this);
    }
}

void AppStage_TrackerSettings::handle_controller_list_response(
    const ClientPSMoveAPI::ResponseMessage *response_message,
    void *userdata)
{
    AppStage_TrackerSettings *thisPtr= static_cast<AppStage_TrackerSettings *>(userdata);

    const ClientPSMoveAPI::eClientPSMoveResultCode ResultCode = response_message->result_code;
    const ClientPSMoveAPI::t_response_handle response_handle = response_message->opaque_response_handle;

    switch(ResultCode)
    {
        case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
        {
            const PSMoveProtocol::Response *response= GET_PSMOVEPROTOCOL_RESPONSE(response_handle);
			int oldSelectedControllerIndex= thisPtr->m_selectedControllerIndex;

			thisPtr->m_controllerInfos.clear();

            for (int controller_index= 0; controller_index < response->result_controller_list().controllers_size(); ++controller_index)
            {
                const auto &ControllerResponse= response->result_controller_list().controllers(controller_index);

                AppStage_TrackerSettings::ControllerInfo ControllerInfo;

                ControllerInfo.ControllerID= ControllerResponse.controller_id();

                switch(ControllerResponse.controller_type())
                {
                case PSMoveProtocol::PSMOVE:
                    ControllerInfo.ControllerType = ClientControllerView::eControllerType::PSMove;
					thisPtr->m_controllerInfos.push_back(ControllerInfo);
                    break;
                case PSMoveProtocol::PSNAVI:
                    ControllerInfo.ControllerType = ClientControllerView::eControllerType::PSNavi;
                    break;
                case PSMoveProtocol::PSDUALSHOCK4:
                    ControllerInfo.ControllerType = ClientControllerView::eControllerType::PSDualShock4;
					thisPtr->m_controllerInfos.push_back(ControllerInfo);
                    break;
                default:
                    assert(0 && "unreachable");
                }			                
            }

			if (oldSelectedControllerIndex != -1)
			{
				// Maintain the same position in the list if possible
				thisPtr->m_selectedControllerIndex= 
					(oldSelectedControllerIndex < thisPtr->m_controllerInfos.size()) 
					? oldSelectedControllerIndex
					: -1;
			}
			else
			{
	            thisPtr->m_selectedControllerIndex= (thisPtr->m_controllerInfos.size() > 0) ? 0 : -1;
			}

            thisPtr->m_menuState= AppStage_TrackerSettings::idle;
        } break;

        case ClientPSMoveAPI::_clientPSMoveResultCode_error:
        case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
        { 
            thisPtr->m_menuState= AppStage_TrackerSettings::failedControllerListRequest;
        } break;
    }
}

void AppStage_TrackerSettings::request_search_for_new_trackers()
{
    // Tell the psmove service that we want see if new trackers are connected.
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SEARCH_FOR_NEW_TRACKERS);

    m_menuState = AppStage_TrackerSettings::pendingSearchForNewTrackersRequest;
    m_selectedTrackerIndex = -1;
    m_trackerInfos.clear();

    ClientPSMoveAPI::register_callback(
        ClientPSMoveAPI::send_opaque_request(&request),
        AppStage_TrackerSettings::handle_search_for_new_trackers_response, this);
}

void AppStage_TrackerSettings::handle_search_for_new_trackers_response(
    const ClientPSMoveAPI::ResponseMessage *response,
    void *userdata)
{
    AppStage_TrackerSettings *thisPtr = static_cast<AppStage_TrackerSettings *>(userdata);

    thisPtr->request_tracker_list();
}
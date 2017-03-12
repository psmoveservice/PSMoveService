//-- inludes -----
#include "AppStage_TrackerSettings.h"
#include "AppStage_TestTracker.h"
#include "AppStage_ColorCalibration.h"
#include "AppStage_ComputeTrackerPoses.h"
#include "AppStage_DistortionCalibration.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "PSMoveClient_CAPI.h"
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
	, m_selectedHmdIndex(-1)
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
            const PSMClientTrackerInfo &trackerInfo = m_trackerInfos[m_selectedTrackerIndex];

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
	case eTrackerMenuState::pendingHmdListRequest:
	case eTrackerMenuState::failedHmdListRequest:
    {
    } break;

    default:
        assert(0 && "unreachable");
    }
}

const PSMClientTrackerInfo *AppStage_TrackerSettings::getSelectedTrackerInfo() const
{
    return
        (m_selectedTrackerIndex != -1)
        ? &m_trackerInfos[m_selectedTrackerIndex]
        : nullptr;
}
	
void AppStage_TrackerSettings::set_selectedTrackerIndex(int index)
{
	m_selectedTrackerIndex = 
		(index != -1 && index < m_trackerInfos.size())
		? index
		: m_selectedTrackerIndex;
}

int AppStage_TrackerSettings::get_tracker_count() const
{
	return static_cast<int>(m_trackerInfos.size()); 
}

int AppStage_TrackerSettings::get_tracker_Index() const
{
	return m_selectedTrackerIndex;
}

int AppStage_TrackerSettings::get_controller_count() const
{
	return static_cast<int>(m_controllerInfos.size());
}

const AppStage_TrackerSettings::ControllerInfo * AppStage_TrackerSettings::get_controller_info(int index) const
{
	return &m_controllerInfos[index];
}

const AppStage_TrackerSettings::ControllerInfo *AppStage_TrackerSettings::get_selected_controller() {
	const ControllerInfo *controller = NULL;

	if (m_selectedControllerIndex != -1)
	{
		const AppStage_TrackerSettings::ControllerInfo &controllerInfo =
			m_controllerInfos[m_selectedControllerIndex];

		controller = &controllerInfo;
	}

	return controller;
}

const AppStage_TrackerSettings::HMDInfo *AppStage_TrackerSettings::get_selected_hmd()
{
	const HMDInfo *hmd = NULL;

	if (m_selectedHmdIndex != -1)
	{
		const AppStage_TrackerSettings::HMDInfo &hmdinfo =
			m_hmdInfos[m_selectedHmdIndex];

		hmd = &hmdinfo;
	}

	return hmd;
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
            const PSMClientTrackerInfo &trackerInfo = m_trackerInfos[m_selectedTrackerIndex];

            if (m_selectedTrackerIndex > 0)
            {
                if (ImGui::Button("<##TrackerIndex"))
                {
                    --m_selectedTrackerIndex;
                }
            }
			else {
				if (ImGui::Button("<##TrackerIndex"))
				{
					m_selectedTrackerIndex = static_cast<int>(m_trackerInfos.size()) -1;
				}
			}
			ImGui::SameLine();
            ImGui::Text("Tracker: %d", m_selectedTrackerIndex);
			ImGui::SameLine();
            if (m_selectedTrackerIndex + 1 < static_cast<int>(m_trackerInfos.size()))
            {
                if (ImGui::Button(">##TrackerIndex"))
                {
                    ++m_selectedTrackerIndex;
                }
			}
			else {
				if (ImGui::Button(">##TrackerIndex"))
				{
					m_selectedTrackerIndex = 0;
				}
			}

            ImGui::BulletText("Tracker ID: %d", trackerInfo.tracker_id);

            switch (trackerInfo.tracker_type)
            {
            case PSMTracker_PS3Eye:
                {
                    ImGui::BulletText("Controller Type: PS3 Eye");
                } break;
            default:
                assert(0 && "Unreachable");
            }

            switch (trackerInfo.tracker_driver)
            {
            case PSMDriver_LIBUSB:
                {
                    ImGui::BulletText("Controller Type: LIBUSB");
                } break;
            case PSMDriver_CL_EYE:
                {
                    ImGui::BulletText("Controller Type: CLEye");
                } break;
            case PSMDriver_CL_EYE_MULTICAM:
                {
                    ImGui::BulletText("Controller Type: CLEye(Multicam SDK)");
                } break;
            case PSMDriver_GENERIC_WEBCAM:
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

			if (m_app->getIsLocalServer())
			{
				if (ImGui::Button("Test Tracker Video Feed"))
				{
					m_app->setAppStage(AppStage_TestTracker::APP_STAGE_NAME);
				}

				if (ImGui::Button("Calibrate Tracker Distortion"))
				{
					m_app->setAppStage(AppStage_DistortionCalibration::APP_STAGE_NAME);
				}
			}
			else
			{
				ImGui::TextDisabled("Test Tracker Video Feed");
				ImGui::TextDisabled("Calibrate Tracker Distortion");
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

					if (controllerInfo.ControllerType == PSMController_Move)
					{ 
						if (0 <= controllerInfo.TrackingColorType && controllerInfo.TrackingColorType < PSMTrackingColorType_MaxColorTypes) {
							const char *colors[] = { "Magenta","Cyan","Yellow","Red","Green","Blue" };

							ImGui::Text("Controller: %d (PSMove) - %s", 
								m_selectedControllerIndex,
								colors[controllerInfo.TrackingColorType]);
						}
						else {
							ImGui::Text("Controller: %d (PSMove)", m_selectedControllerIndex);
						}
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

				{
					int controllerID = (m_selectedControllerIndex != -1) ? m_controllerInfos[m_selectedControllerIndex].ControllerID : -1;

					if (m_app->getIsLocalServer())
					{
						if (ImGui::Button("Calibrate Controller Tracking Colors"))
						{
							const ControllerInfo *controller = get_selected_controller();
							if (controller != NULL) {
								m_app->getAppStage<AppStage_ColorCalibration>()->set_override_controller_id(controller->ControllerID);
								m_app->getAppStage<AppStage_ColorCalibration>()->set_override_tracking_color(controller->TrackingColorType);
							}
							m_app->setAppStage(AppStage_ColorCalibration::APP_STAGE_NAME);
						}

						if (ImGui::Button("Compute Tracker Poses"))
						{
							AppStage_ComputeTrackerPoses::enterStageAndCalibrate(m_app, controllerID);
						}

					}
					else
					{
						ImGui::TextDisabled("Calibrate Controller Tracking Colors");
						ImGui::TextDisabled("Compute Tracker Poses");
					}


					if (ImGui::Button("Test Tracking"))
					{
						AppStage_ComputeTrackerPoses::enterStageAndSkipCalibration(m_app, controllerID);
					}
				}
			}

			if (m_hmdInfos.size() > 0)
			{
				ImGui::Separator();

				if (m_selectedHmdIndex > 0)
				{
					if (ImGui::Button("<##HMD"))
					{
						--m_selectedHmdIndex;
					}
					ImGui::SameLine();
				}

				if (m_selectedHmdIndex != -1)
				{
					const AppStage_TrackerSettings::HMDInfo &hmdInfo = m_hmdInfos[m_selectedHmdIndex];

					if (hmdInfo.HmdType == PSMHmd_Morpheus)
					{
						if (0 <= hmdInfo.TrackingColorType && hmdInfo.TrackingColorType < PSMTrackingColorType_MaxColorTypes) 
						{
							const char *colors[] = { "Magenta","Cyan","Yellow","Red","Green","Blue" };

							ImGui::Text("HMD: %d (Morpheus) - %s",
								m_selectedHmdIndex,
								colors[hmdInfo.TrackingColorType]);
						}
						else
						{
							ImGui::Text("HMD: %d (Morpheus)", m_selectedHmdIndex);
						}
					}
				}

				if (m_selectedHmdIndex + 1 < static_cast<int>(m_hmdInfos.size()))
				{
					ImGui::SameLine();
					if (ImGui::Button(">##HMD"))
					{
						++m_selectedHmdIndex;
					}
				}

				if (m_app->getIsLocalServer())
				{
					if (ImGui::Button("Calibrate HMD Tracking Colors"))
					{
						const HMDInfo *hmd = get_selected_hmd();
						if (hmd != NULL) 
						{
							m_app->getAppStage<AppStage_ColorCalibration>()->set_override_hmd_id(hmd->HmdID);
							m_app->getAppStage<AppStage_ColorCalibration>()->set_override_tracking_color(hmd->TrackingColorType);
						}

						m_app->setAppStage(AppStage_ColorCalibration::APP_STAGE_NAME);
					}
				}
				else
				{
					ImGui::TextDisabled("Calibrate HMD Tracking Colors");
				}
			}
        }

        ImGui::Separator();

        if (ImGui::Button("Return to Main Menu"))
        {
            m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
        }

        ImGui::End();
    } break;
    case eTrackerMenuState::pendingSearchForNewTrackersRequest:
    case eTrackerMenuState::pendingTrackerListRequest:
	case eTrackerMenuState::pendingControllerListRequest:
	case eTrackerMenuState::pendingHmdListRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(300, 150));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Waiting for server response...");

        ImGui::End();
    } break;
    case eTrackerMenuState::failedTrackerListRequest:
	case eTrackerMenuState::failedControllerListRequest:
	case eTrackerMenuState::failedHmdListRequest:
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
    PSMEventMessage::eEventType event, 
    PSMEventDataHandle opaque_event_handle)
{
    bool bHandled = false;

    switch (event)
    {
    case  PSMEventMessage::PSMEvent_controllerListUpdated:
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
		PSMRequestID requestId;
		PSM_GetTrackerListAsync(&requestId);
		PSM_RegisterCallback(requestId, AppStage_TrackerSettings::handle_tracker_list_response, this);
    }
}

void AppStage_TrackerSettings::handle_tracker_list_response(
    const PSMResponseMessage *response_message,
    void *userdata)
{
    AppStage_TrackerSettings *thisPtr = static_cast<AppStage_TrackerSettings *>(userdata);

    switch (response_message->result_code)
    {
    case PSMResult_Success:
        {
            assert(response_message->payload_type == PSMResponseMessage::_responsePayloadType_TrackerList);
            const PSMTrackerList &tracker_list= response_message->payload.tracker_list;
			int oldSelectedTrackerIndex= thisPtr->m_selectedTrackerIndex;

			thisPtr->m_selectedTrackerIndex = -1;
			thisPtr->m_trackerInfos.clear();

            for (int tracker_index = 0; tracker_index < tracker_list.count; ++tracker_index)
            {
                const PSMClientTrackerInfo &TrackerInfo = tracker_list.trackers[tracker_index];

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

    case PSMResult_Error:
    case PSMResult_Canceled:
	case PSMResult_Timeout:
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

		PSMRequestID request_id;
		PSM_SendOpaqueRequest(&request, &request_id);
		PSM_RegisterCallback(request_id, AppStage_TrackerSettings::handle_controller_list_response, this);
    }
}

void AppStage_TrackerSettings::handle_controller_list_response(
    const PSMResponseMessage *response_message,
    void *userdata)
{
    AppStage_TrackerSettings *thisPtr= static_cast<AppStage_TrackerSettings *>(userdata);

    const PSMResult ResultCode = response_message->result_code;
    const PSMResponseHandle response_handle = response_message->opaque_response_handle;

    switch(ResultCode)
    {
        case PSMResult_Success:
        {
            const PSMoveProtocol::Response *response= GET_PSMOVEPROTOCOL_RESPONSE(response_handle);
			int oldSelectedControllerIndex= thisPtr->m_selectedControllerIndex;

			thisPtr->m_controllerInfos.clear();

            for (int controller_index= 0; controller_index < response->result_controller_list().controllers_size(); ++controller_index)
            {
                const auto &ControllerResponse= response->result_controller_list().controllers(controller_index);

                AppStage_TrackerSettings::ControllerInfo ControllerInfo;

                ControllerInfo.ControllerID= ControllerResponse.controller_id();
				ControllerInfo.TrackingColorType = (PSMTrackingColorType)ControllerResponse.tracking_color_type();

                switch(ControllerResponse.controller_type())
                {
                case PSMoveProtocol::PSMOVE:
                    ControllerInfo.ControllerType = PSMController_Move;
					thisPtr->m_controllerInfos.push_back(ControllerInfo);
                    break;
                case PSMoveProtocol::PSNAVI:
                    ControllerInfo.ControllerType = PSMController_Navi;
                    break;
                case PSMoveProtocol::PSDUALSHOCK4:
                    ControllerInfo.ControllerType = PSMController_DualShock4;
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

			// Request the list of HMDs next
			thisPtr->request_hmd_list();
        } break;

        case PSMResult_Error:
        case PSMResult_Canceled:
		case PSMResult_Timeout:
        { 
            thisPtr->m_menuState= AppStage_TrackerSettings::failedControllerListRequest;
        } break;
    }
}

void AppStage_TrackerSettings::request_hmd_list()
{
	if (m_menuState != AppStage_TrackerSettings::pendingHmdListRequest)
	{
		m_menuState = AppStage_TrackerSettings::pendingHmdListRequest;

		// Tell the psmove service that we we want a list of HMDs connected to this machine
		RequestPtr request(new PSMoveProtocol::Request());
		request->set_type(PSMoveProtocol::Request_RequestType_GET_HMD_LIST);

		PSMRequestID request_id;
		PSM_SendOpaqueRequest(&request, &request_id);
		PSM_RegisterCallback(request_id, AppStage_TrackerSettings::handle_hmd_list_response, this);
	}
}

void AppStage_TrackerSettings::handle_hmd_list_response(
	const PSMResponseMessage *response_message,
	void *userdata)
{
	AppStage_TrackerSettings *thisPtr = static_cast<AppStage_TrackerSettings *>(userdata);

	const PSMResult ResultCode = response_message->result_code;
	const PSMResponseHandle response_handle = response_message->opaque_response_handle;

	switch (ResultCode)
	{
	case PSMResult_Success:
	{
		const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);
		int oldSelectedHmdIndex = thisPtr->m_selectedHmdIndex;

		thisPtr->m_hmdInfos.clear();

		for (int hmd_index = 0; hmd_index < response->result_hmd_list().hmd_entries_size(); ++hmd_index)
		{
			const auto &HmdResponse = response->result_hmd_list().hmd_entries(hmd_index);

			AppStage_TrackerSettings::HMDInfo HmdInfo;

			HmdInfo.HmdID = HmdResponse.hmd_id();
			HmdInfo.TrackingColorType = (PSMTrackingColorType)HmdResponse.tracking_color_type();

			switch (HmdResponse.hmd_type())
			{
			case PSMoveProtocol::Morpheus:
				HmdInfo.HmdType = PSMHmd_Morpheus;
				thisPtr->m_hmdInfos.push_back(HmdInfo);
				break;
			default:
				assert(0 && "unreachable");
			}
		}

		if (oldSelectedHmdIndex != -1)
		{
			// Maintain the same position in the list if possible
			thisPtr->m_selectedHmdIndex =
				(oldSelectedHmdIndex < thisPtr->m_hmdInfos.size())
				? oldSelectedHmdIndex
				: -1;
		}
		else
		{
			thisPtr->m_selectedHmdIndex = (thisPtr->m_hmdInfos.size() > 0) ? 0 : -1;
		}

		thisPtr->m_menuState = AppStage_TrackerSettings::idle;
	} break;

	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
	{
		thisPtr->m_menuState = AppStage_TrackerSettings::failedControllerListRequest;
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

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_RegisterCallback(request_id, AppStage_TrackerSettings::handle_search_for_new_trackers_response, this);
}

void AppStage_TrackerSettings::handle_search_for_new_trackers_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    AppStage_TrackerSettings *thisPtr = static_cast<AppStage_TrackerSettings *>(userdata);

    thisPtr->request_tracker_list();
}

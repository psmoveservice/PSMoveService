//-- inludes -----
#include "AppStage_HMDSettings.h"
#include "AppStage_HMDAccelerometerCalibration.h"
#include "AppStage_HMDGyroscopeCalibration.h"
#include "AppStage_HMDModelCalibration.h"
#include "AppStage_MainMenu.h"
#include "AppStage_TestHMD.h"
#include "App.h"
#include "Camera.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"

#include "SDL_keycode.h"

#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>

//-- statics ----
const char *AppStage_HMDSettings::APP_STAGE_NAME= "HMDSettings";

//-- constants -----

//-- public methods -----
AppStage_HMDSettings::AppStage_HMDSettings(App *app) 
    : AppStage(app)
    , m_menuState(AppStage_HMDSettings::inactive)
    , m_selectedHmdIndex(-1)
{ }

void AppStage_HMDSettings::enter()
{
    m_app->setCameraType(_cameraFixed);
    m_selectedHmdIndex = -1;

    request_hmd_list();
}

void AppStage_HMDSettings::exit()
{
    m_menuState = AppStage_HMDSettings::inactive;
}

void AppStage_HMDSettings::update()
{
}
    
void AppStage_HMDSettings::render()
{
    switch (m_menuState)
    {
    case eHmdMenuState::idle:
    {
        if (m_selectedHmdIndex >= 0)
        {
            const HMDInfo &hmdInfo = m_hmdInfos[m_selectedHmdIndex];

            switch (hmdInfo.HmdType)
            {
            case PSMoveProtocol::Morpheus:
                {
                    glm::mat4 scale3 = glm::scale(glm::mat4(1.f), glm::vec3(2.f, 2.f, 2.f));
                    drawMorpheusModel(scale3);
                } break;
            default:
                assert(0 && "Unreachable");
            }
        }
    } break;

    case eHmdMenuState::pendingHmdListRequest:
    case eHmdMenuState::failedHmdListRequest:
        {
        } break;

    default:
        assert(0 && "unreachable");
    }
}

void AppStage_HMDSettings::renderUI()
{
    const char *k_window_title = "HMD Settings";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case eHmdMenuState::idle:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(300, 400));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        if (m_hmdInfos.size() > 0)
        {
            HMDInfo &hmdInfo = m_hmdInfos[m_selectedHmdIndex];

            ImGui::Text("HMD: %d", m_selectedHmdIndex);
            ImGui::Text("  HMD ID: %d", hmdInfo.HmdID);

            switch (hmdInfo.HmdType)
            {
            case AppStage_HMDSettings::Morpheus:
            {
                ImGui::Text("  HMD Type: Morpheus");
            } break;
            default:
                assert(0 && "Unreachable");
            }

            ImGui::TextWrapped("  Device Path: %s", hmdInfo.DevicePath.c_str());

            if (m_selectedHmdIndex > 0)
            {
                if (ImGui::Button("Previous HMD"))
                {
                    --m_selectedHmdIndex;
                }
            }

            if (m_selectedHmdIndex + 1 < static_cast<int>(m_hmdInfos.size()))
            {
                if (ImGui::Button("Next HMD"))
                {
                    ++m_selectedHmdIndex;
                }
            }

			if (hmdInfo.HmdType == AppStage_HMDSettings::eHMDType::Morpheus)
			{
				if (ImGui::Button("Calibrate Accelerometer"))
				{
					m_app->getAppStage<AppStage_HMDAccelerometerCalibration>()->setBypassCalibrationFlag(false);
					m_app->setAppStage(AppStage_HMDAccelerometerCalibration::APP_STAGE_NAME);
				}

				if (ImGui::Button("Test Accelerometer"))
				{
					m_app->getAppStage<AppStage_HMDAccelerometerCalibration>()->setBypassCalibrationFlag(true);
					m_app->setAppStage(AppStage_HMDAccelerometerCalibration::APP_STAGE_NAME);
				}
			}

			if (hmdInfo.HmdType == AppStage_HMDSettings::eHMDType::Morpheus)
			{
				if (ImGui::Button("Calibrate Gyroscope"))
				{
					m_app->getAppStage<AppStage_HMDGyroscopeCalibration>()->setBypassCalibrationFlag(false);
					m_app->setAppStage(AppStage_HMDGyroscopeCalibration::APP_STAGE_NAME);
				}

				if (ImGui::Button("Test Orientation"))
				{
					m_app->getAppStage<AppStage_HMDGyroscopeCalibration>()->setBypassCalibrationFlag(true);
					m_app->setAppStage(AppStage_HMDGyroscopeCalibration::APP_STAGE_NAME);
				}
			}

			if (hmdInfo.HmdType == AppStage_HMDSettings::eHMDType::Morpheus)
			{
				if (m_app->getIsLocalServer())
				{
					if (ImGui::Button("Calibrate LED Model"))
					{
						AppStage_HMDModelCalibration::enterStageAndCalibrate(m_app, m_selectedHmdIndex);
					}
				}
				else
				{
					ImGui::TextDisabled("Calibrate LED Model");
				}
			}

            if (ImGui::Button("Test HMD Tracking"))
            {
                m_app->setAppStage(AppStage_TestHMD::APP_STAGE_NAME);
            }

			{
				ImGui::PushItemWidth(195);
				//###HipterSloth $TODO Add filter settings
				//if (ImGui::Combo("position filter", &controllerInfo.PositionFilterIndex, k_position_filter_names, UI_ARRAYSIZE(k_position_filter_names)))
				//{
				//	controllerInfo.PositionFilterName = k_position_filter_names[controllerInfo.PositionFilterIndex];
				//	request_set_position_filter(controllerInfo.ControllerID, controllerInfo.PositionFilterName);
				//}
				//if (ImGui::Combo("orientation filter", &controllerInfo.OrientationFilterIndex, k_psmove_orientation_filter_names, UI_ARRAYSIZE(k_psmove_orientation_filter_names)))
				//{
				//	controllerInfo.OrientationFilterName = k_psmove_orientation_filter_names[controllerInfo.OrientationFilterIndex];
				//	request_set_orientation_filter(controllerInfo.ControllerID, controllerInfo.OrientationFilterName);
				//}
				if (ImGui::SliderFloat("Prediction Time", &hmdInfo.PredictionTime, 0.f, 1.f))
				{
					request_set_hmd_prediction(hmdInfo.HmdID, hmdInfo.PredictionTime);
				}
				//if (ImGui::Button("Reset Filter Defaults"))
				//{
				//	controllerInfo.PositionFilterIndex = k_default_position_filter_index;
				//	controllerInfo.OrientationFilterIndex = k_default_psmove_orientation_filter_index;
				//	controllerInfo.PositionFilterName = k_psmove_orientation_filter_names[k_default_position_filter_index];
				//	controllerInfo.OrientationFilterName = k_psmove_orientation_filter_names[k_default_psmove_orientation_filter_index];
				//	request_set_position_filter(controllerInfo.ControllerID, controllerInfo.PositionFilterName);
				//	request_set_orientation_filter(controllerInfo.ControllerID, controllerInfo.OrientationFilterName);
				//}
				ImGui::PopItemWidth();
			}
        }
        else
        {
            ImGui::Text("No HMDs");
        }

        if (ImGui::Button("Return to Main Menu"))
        {
            m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
        }

        ImGui::End();
    } break;
    case eHmdMenuState::pendingHmdListRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(300, 150));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Waiting for HMD list response...");

        ImGui::End();
    } break;
    case eHmdMenuState::failedHmdListRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(300, 150));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Failed to get tracker list!");

        if (ImGui::Button("Retry"))
        {
            request_hmd_list();
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

bool AppStage_HMDSettings::onClientAPIEvent(
    PSMEventMessage::eEventType event, 
    PSMEventDataHandle opaque_event_handle)
{
    bool bHandled = false;

    switch (event)
    {
    case PSMEventMessage::PSMEvent_hmdListUpdated:
        {
            bHandled = true;
            request_hmd_list();
        } break;
    }

    return bHandled;
}

void AppStage_HMDSettings::request_hmd_list()
{
    if (m_menuState != AppStage_HMDSettings::pendingHmdListRequest)
    {
        m_menuState = AppStage_HMDSettings::pendingHmdListRequest;
        m_selectedHmdIndex = -1;
        m_hmdInfos.clear();

        // Tell the psmove service that we we want a list of HMDs connected to this machine
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_GET_HMD_LIST);

		PSMRequestID request_id;
		PSM_SendOpaqueRequest(&request, &request_id);
		PSM_RegisterCallback(request_id, AppStage_HMDSettings::handle_hmd_list_response, this);
    }
}

void AppStage_HMDSettings::request_set_hmd_prediction(const int hmd_id, float prediction_time)
{
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_HMD_PREDICTION_TIME);

	PSMoveProtocol::Request_RequestSetHMDPredictionTime *calibration =
		request->mutable_request_set_hmd_prediction_time();

	calibration->set_hmd_id(hmd_id);
	calibration->set_prediction_time(prediction_time);

	PSM_SendOpaqueRequest(&request, nullptr);
}

void AppStage_HMDSettings::handle_hmd_list_response(
	const PSMResponseMessage *response,
	void *userdata)
{
	PSMResult ResultCode = response->result_code;
	PSMResponseHandle response_handle = response->opaque_response_handle;
    AppStage_HMDSettings *thisPtr = static_cast<AppStage_HMDSettings *>(userdata);

    switch (ResultCode)
    {
    case PSMResult_Success:
        {
            const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);

            for (int hmd_index = 0; hmd_index < response->result_hmd_list().hmd_entries_size(); ++hmd_index)
            {
                const auto &HmdResponse = response->result_hmd_list().hmd_entries(hmd_index);

                AppStage_HMDSettings::HMDInfo HmdInfo;

                HmdInfo.HmdID = HmdResponse.hmd_id();

                switch (HmdResponse.hmd_type())
                {
                case PSMoveProtocol::HMDType::Morpheus:
                    HmdInfo.HmdType = AppStage_HMDSettings::Morpheus;
                    break;
                default:
                    assert(0 && "unreachable");
                }

                HmdInfo.DevicePath = HmdResponse.device_path();
				HmdInfo.PredictionTime = HmdResponse.prediction_time();

                thisPtr->m_hmdInfos.push_back(HmdInfo);
            }

            thisPtr->m_selectedHmdIndex = (thisPtr->m_hmdInfos.size() > 0) ? 0 : -1;
            thisPtr->m_menuState = AppStage_HMDSettings::idle;
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
	case PSMResult_Timeout:
        {
            thisPtr->m_menuState = AppStage_HMDSettings::failedHmdListRequest;
        } break;
    }
}
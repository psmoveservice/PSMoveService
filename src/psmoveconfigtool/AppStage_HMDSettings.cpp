//-- inludes -----
#include "AppStage_HMDSettings.h"
#include "AppStage_HMDAccelerometerCalibration.h"
#include "AppStage_HMDGyroscopeCalibration.h"
#include "AppStage_HMDModelCalibration.h"
#include "AppStage_MainMenu.h"
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
const int k_default_hmd_position_filter_index = 3; // LowPassExponential
const int k_default_morpheus_position_filter_index = 5; // PositionKalman
const int k_default_morpheus_orientation_filter_index = 3; // OrientationKalman

const char* k_hmd_position_filter_names[] = { "PassThru", "LowPassOptical", "LowPassIMU", "LowPassExponential", "ComplimentaryOpticalIMU", "PositionKalman" };
const char* k_morpheus_orientation_filter_names[] = { "PassThru", "MadgwickARG", "ComplementaryOpticalARG", "OrientationKalman" };

const float k_max_hmd_prediction_time = 0.15f; // About 150ms seems to be about the point where you start to get really bad over-prediction 

inline int find_string_entry(const char *string_entry, const char* string_list[], size_t list_size)
{
    int found_index = -1;
    for (size_t test_index = 0; test_index < list_size; ++test_index)
    {
        if (strncmp(string_entry, string_list[test_index], 32) == 0)
        {
            found_index = static_cast<int>(test_index);
            break;
        }
    }

    return found_index;
}

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

            // Display the tracking color being used for the controller
            glm::vec3 bulb_color = glm::vec3(1.f, 1.f, 1.f);

            switch (hmdInfo.TrackingColorType)
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

            switch (hmdInfo.HmdType)
            {
            case PSMoveProtocol::Morpheus:
                {
                    glm::mat4 scale3 = glm::scale(glm::mat4(1.f), glm::vec3(2.f, 2.f, 2.f));
                    drawMorpheusModel(scale3);
                } break;
            case PSMoveProtocol::VirtualHMD:
                {
                    glm::mat4 scale3 = glm::scale(glm::mat4(1.f), glm::vec3(2.f, 2.f, 2.f));
                    drawVirtualHMDModel(scale3, bulb_color);
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
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case eHmdMenuState::idle:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(350, 400));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        if (m_hmdInfos.size() > 0)
        {
            HMDInfo &hmdInfo = m_hmdInfos[m_selectedHmdIndex];

            if (m_selectedHmdIndex > 0)
            {
                if (ImGui::Button("<##HMDIndex"))
                {
                    --m_selectedHmdIndex;
                }
                ImGui::SameLine();
            }
            ImGui::Text("HMD: %d", m_selectedHmdIndex);
            if (m_selectedHmdIndex + 1 < static_cast<int>(m_hmdInfos.size()))
            {
                ImGui::SameLine();
                if (ImGui::Button(">##HMDIndex"))
                {
                    ++m_selectedHmdIndex;
                }
            }
            
            // Combo box selection for hmd tracking color
            if (hmdInfo.HmdType == AppStage_HMDSettings::VirtualHMD)
            {
                int newTrackingColorType = hmdInfo.TrackingColorType;

                ImGui::PushItemWidth(195);
                if (ImGui::Combo("Tracking Color", &newTrackingColorType, "Magenta\0Cyan\0Yellow\0Red\0Green\0Blue\0\0"))
                {
                    hmdInfo.TrackingColorType = static_cast<PSMTrackingColorType>(newTrackingColorType);

                    request_set_hmd_tracking_color_id(hmdInfo.HmdID, hmdInfo.TrackingColorType);

                    // Re-request the controller list since the tracking colors could changed for other controllers
                    request_hmd_list();
                }
                ImGui::PopItemWidth();
            }
            else if (hmdInfo.HmdType == AppStage_HMDSettings::Morpheus)
            {
                switch (hmdInfo.TrackingColorType)
                {
                case PSMTrackingColorType_Magenta:
                    ImGui::BulletText("Tracking Color: Magenta");
                    break;
                case PSMTrackingColorType_Cyan:
                    ImGui::BulletText("Tracking Color: Cyan");
                    break;
                case PSMTrackingColorType_Yellow:
                    ImGui::BulletText("Tracking Color: Yellow");
                    break;
                case PSMTrackingColorType_Red:
                    ImGui::BulletText("Tracking Color: Red");
                    break;
                case PSMTrackingColorType_Green:
                    ImGui::BulletText("Tracking Color: Green");
                    break;
                case PSMTrackingColorType_Blue:
                    ImGui::BulletText("Tracking Color: Blue");
                    break;
                }
            }

            ImGui::BulletText("HMD ID: %d", hmdInfo.HmdID);

            switch (hmdInfo.HmdType)
            {
            case AppStage_HMDSettings::Morpheus:
                {
                    ImGui::BulletText("HMD Type: Morpheus");
                    ImGui::TextWrapped("Device Path: %s", hmdInfo.DevicePath.c_str());
                } break;
            case AppStage_HMDSettings::VirtualHMD:
                {
                    ImGui::BulletText("HMD Type: VirtualHMD");
                } break;
            default:
                assert(0 && "Unreachable");
            }

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

            if (hmdInfo.HmdType == AppStage_HMDSettings::eHMDType::Morpheus)
            {		
                ImGui::PushItemWidth(195);
                if (ImGui::Combo("Position Filter", &hmdInfo.PositionFilterIndex, k_hmd_position_filter_names, UI_ARRAYSIZE(k_hmd_position_filter_names)))
                {
                    hmdInfo.PositionFilterName = k_hmd_position_filter_names[hmdInfo.PositionFilterIndex];
                    request_set_position_filter(hmdInfo.HmdID, hmdInfo.PositionFilterName);
                }
                if (ImGui::Combo("Orientation Filter", &hmdInfo.OrientationFilterIndex, k_morpheus_orientation_filter_names, UI_ARRAYSIZE(k_morpheus_orientation_filter_names)))
                {
                    hmdInfo.OrientationFilterName = k_morpheus_orientation_filter_names[hmdInfo.OrientationFilterIndex];
                    request_set_orientation_filter(hmdInfo.HmdID, hmdInfo.OrientationFilterName);
                }
                if (ImGui::SliderFloat("Prediction Time", &hmdInfo.PredictionTime, 0.f, k_max_hmd_prediction_time))
                {
                    request_set_hmd_prediction(hmdInfo.HmdID, hmdInfo.PredictionTime);
                }
                if (ImGui::Button("Reset Filter Defaults"))
                {
                    hmdInfo.PositionFilterIndex = k_default_hmd_position_filter_index;
                    hmdInfo.OrientationFilterIndex = k_default_morpheus_position_filter_index;
                    hmdInfo.PositionFilterName = k_hmd_position_filter_names[k_default_hmd_position_filter_index];
                    hmdInfo.OrientationFilterName = k_morpheus_orientation_filter_names[k_default_morpheus_position_filter_index];
                    request_set_position_filter(hmdInfo.HmdID, hmdInfo.PositionFilterName);
                    request_set_orientation_filter(hmdInfo.HmdID, hmdInfo.OrientationFilterName);
                }
                ImGui::PopItemWidth();
            }				
            else if (hmdInfo.HmdType == AppStage_HMDSettings::eHMDType::VirtualHMD)
            {
                ImGui::PushItemWidth(195);
                if (ImGui::Combo("Position Filter", &hmdInfo.PositionFilterIndex, k_hmd_position_filter_names, UI_ARRAYSIZE(k_hmd_position_filter_names)))
                {
                    hmdInfo.PositionFilterName = k_hmd_position_filter_names[hmdInfo.PositionFilterIndex];
                    request_set_position_filter(hmdInfo.HmdID, hmdInfo.PositionFilterName);
                }
                if (ImGui::SliderFloat("Prediction Time", &hmdInfo.PredictionTime, 0.f, k_max_hmd_prediction_time))
                {
                    request_set_hmd_prediction(hmdInfo.HmdID, hmdInfo.PredictionTime);
                }
                if (ImGui::Button("Reset Filter Defaults"))
                {
                    hmdInfo.PositionFilterIndex = k_default_hmd_position_filter_index;
                    hmdInfo.PositionFilterName = k_hmd_position_filter_names[k_default_hmd_position_filter_index];
                    request_set_position_filter(hmdInfo.HmdID, hmdInfo.PositionFilterName);
                    request_set_orientation_filter(hmdInfo.HmdID, hmdInfo.OrientationFilterName);
                }
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

void AppStage_HMDSettings::request_set_orientation_filter(
    const int hmd_id,
    const std::string &filter_name)
{
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_HMD_ORIENTATION_FILTER);

    request->mutable_request_set_hmd_orientation_filter()->set_hmd_id(hmd_id);
    request->mutable_request_set_hmd_orientation_filter()->set_orientation_filter(filter_name);

    PSM_SendOpaqueRequest(&request, nullptr);
}

void AppStage_HMDSettings::request_set_position_filter(
    const int hmd_id,
    const std::string &filter_name)
{
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_HMD_POSITION_FILTER);

    request->mutable_request_set_hmd_position_filter()->set_hmd_id(hmd_id);
    request->mutable_request_set_hmd_position_filter()->set_position_filter(filter_name);

    PSM_SendOpaqueRequest(&request, nullptr);
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

void AppStage_HMDSettings::request_set_hmd_tracking_color_id(
	int HmdID,
	PSMTrackingColorType tracking_color_type)
{
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_HMD_LED_TRACKING_COLOR);
	request->mutable_set_hmd_led_tracking_color_request()->set_hmd_id(HmdID);
	request->mutable_set_hmd_led_tracking_color_request()->set_color_type(
		static_cast<PSMoveProtocol::TrackingColorType>(tracking_color_type));

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
                case PSMoveProtocol::HMDType::VirtualHMD:
                    HmdInfo.HmdType = AppStage_HMDSettings::VirtualHMD;
                    break;
                default:
                    assert(0 && "unreachable");
                }

                HmdInfo.TrackingColorType = static_cast<PSMTrackingColorType>(HmdResponse.tracking_color_type());
                HmdInfo.DevicePath = HmdResponse.device_path();
                HmdInfo.PredictionTime = HmdResponse.prediction_time();
                HmdInfo.OrientationFilterName= HmdResponse.orientation_filter();
                HmdInfo.PositionFilterName = HmdResponse.position_filter();

                if (HmdInfo.HmdType == AppStage_HMDSettings::Morpheus)
                {
                    HmdInfo.OrientationFilterIndex =
                        find_string_entry(
                            HmdInfo.OrientationFilterName.c_str(),
                            k_morpheus_orientation_filter_names,
                            UI_ARRAYSIZE(k_morpheus_orientation_filter_names));
                    if (HmdInfo.OrientationFilterIndex == -1)
                    {
                        HmdInfo.OrientationFilterName = k_morpheus_orientation_filter_names[0];
                        HmdInfo.OrientationFilterIndex = 0;
                    }
                }
                else
                {
                    HmdInfo.OrientationFilterName = "";
                    HmdInfo.OrientationFilterIndex = -1;
                }

                if (HmdInfo.HmdType == AppStage_HMDSettings::Morpheus ||
                    HmdInfo.HmdType == AppStage_HMDSettings::VirtualHMD)
                {
                    HmdInfo.PositionFilterIndex =
                        find_string_entry(
                            HmdInfo.PositionFilterName.c_str(),
                            k_hmd_position_filter_names,
                            UI_ARRAYSIZE(k_hmd_position_filter_names));
                    if (HmdInfo.PositionFilterIndex == -1)
                    {
                        HmdInfo.PositionFilterName = k_hmd_position_filter_names[0];
                        HmdInfo.PositionFilterIndex = 0;
                    }
                }
                else
                {
                    HmdInfo.PositionFilterName = "";
                    HmdInfo.PositionFilterIndex = -1;
                }

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
//-- inludes -----
#include "AppStage_ControllerSettings.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"

#include "SDL_keycode.h"

#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>

#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

//-- statics ----
const char *AppStage_ControllerSettings::APP_STAGE_NAME= "ControllerSettings";

//-- constants -----

//-- public methods -----
AppStage_ControllerSettings::AppStage_ControllerSettings(App *app) 
    : AppStage(app)
    , m_menuState(AppStage_ControllerSettings::inactive)
    , m_selectedControllerIndex(-1)
    , m_pendingBluetoothOpControllerIndex(-1)
    , m_pair_steps_completed(0)
    , m_pair_steps_total(0)
{ }

void AppStage_ControllerSettings::enter()
{
    m_app->setCameraType(_cameraFixed);
    m_selectedControllerIndex= -1;
    m_pendingBluetoothOpControllerIndex = -1;

    request_controller_list();
}

void AppStage_ControllerSettings::exit()
{
    m_menuState= AppStage_ControllerSettings::inactive;
}

void AppStage_ControllerSettings::update()
{
}
    
void AppStage_ControllerSettings::render()
{
    glm::mat4 scale2RotateX90= 
        glm::rotate(
            glm::scale(glm::mat4(1.f), glm::vec3(2.f, 2.f, 2.f)), 
            90.f, glm::vec3(1.f, 0.f, 0.f));    

    switch (m_menuState)
    {
    case eControllerMenuState::idle:
        {
            if (m_selectedControllerIndex >= 0)
            {
                const ControllerInfo &controllerInfo= m_pairedControllerInfos[m_selectedControllerIndex];

                switch(controllerInfo.ControllerType)
                {
                    case PSMoveProtocol::PSMOVE:
                        {
                            drawPSMoveModel(scale2RotateX90, glm::vec3(1.f, 1.f, 1.f));
                        } break;
                    case PSMoveProtocol::PSNAVI:
                        {
                            drawPSNaviModel(scale2RotateX90);
                        } break;
                    default:
                        assert(0 && "Unreachable");
                }        
            }
        } break;

    case eControllerMenuState::pendingControllerListRequest:
    case eControllerMenuState::failedControllerListRequest:
    case eControllerMenuState::pendingControllerUnpairRequest:
    case eControllerMenuState::failedControllerUnpairRequest:
    case eControllerMenuState::pendingControllerPairRequest:
    case eControllerMenuState::failedControllerPairRequest:
    case eControllerMenuState::pendingCancelBluetoothRequest:
    case eControllerMenuState::failedCancelBluetoothRequest:
        {
        } break;

    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ControllerSettings::renderUI()
{
    const char *k_window_title= "Controller Settings";
    const ImGuiWindowFlags window_flags = 
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize | 
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case eControllerMenuState::idle:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, ImVec2(300, 400), k_background_alpha, window_flags);

            if (m_pairedControllerInfos.size() > 0)
            {
                const ControllerInfo &controllerInfo= m_pairedControllerInfos[m_selectedControllerIndex];

                ImGui::Text("Controller: %d", m_selectedControllerIndex);
                ImGui::Text("  Controller ID: %d", controllerInfo.ControllerID);

                switch(controllerInfo.ControllerType)
                {
                    case AppStage_ControllerSettings::PSMove:
                        {
                            ImGui::Text("  Controller Type: PSMove");
                        } break;
                    case AppStage_ControllerSettings::PSNavi:
                        {
                            ImGui::Text("  Controller Type: PSNavi");
                        } break;
                    default:
                        assert(0 && "Unreachable");
                }

                ImGui::Text("  Controller Connection: %s", 
                    controllerInfo.ConnectionType == AppStage_ControllerSettings::Bluetooth
                    ? "Bluetooth" : "USB");
                ImGui::Text("  Device Serial: %s", controllerInfo.DeviceSerial.c_str());
                ImGui::Text("  Host Serial: %s", controllerInfo.HostSerial.c_str());
                ImGui::TextWrapped("  Device Path: %s", controllerInfo.DevicePath.c_str());

                if (m_selectedControllerIndex > 0)
                {
                    if (ImGui::Button("Previous Controller"))
                    {
                        --m_selectedControllerIndex;
                    }
                }

                if (m_selectedControllerIndex + 1 < static_cast<int>(m_pairedControllerInfos.size()))
                {
                    if (ImGui::Button("Next Controller"))
                    {
                        ++m_selectedControllerIndex;
                    }
                }

                // We can only unpair controllers connected via usb
                if (controllerInfo.ConnectionType == AppStage_ControllerSettings::USB)
                {
                    if (ImGui::Button("Unpair Controller"))
                    {
                        ControllerInfo &controllerInfo= m_pairedControllerInfos[m_selectedControllerIndex];

                        request_controller_unpair(controllerInfo.ControllerID);
                    }
                }

                // If there are any controllers waiting to be paired, 
                // just present the first one as an option
                if (m_unpairedControllerInfos.size() > 0)
                {
                    if (ImGui::Button("Pair New Controller"))
                    {
                        ControllerInfo &controllerInfo= m_unpairedControllerInfos[0];

                        request_controller_pair(controllerInfo.ControllerID);
                    }
                }
            }
            // If there are any controllers waiting to be paired, 
            // just present the first one as an option
            else if (m_unpairedControllerInfos.size() > 0)
            {
                if (ImGui::Button("Pair New Controller"))
                {
                    ControllerInfo &controllerInfo= m_unpairedControllerInfos[0];

                    request_controller_pair(controllerInfo.ControllerID);
                }
            }
            else
            {
                ImGui::Text("No connected controllers");
            }

            if (ImGui::Button("Return to Main Menu"))
            {
                m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eControllerMenuState::pendingControllerListRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, ImVec2(300, 150), k_background_alpha, window_flags);

            ImGui::Text("Waiting for controller list response...");

            ImGui::End();
        } break;
    case eControllerMenuState::failedControllerListRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, ImVec2(300, 150), k_background_alpha, window_flags);

            ImGui::Text("Failed to get controller list!");

            if (ImGui::Button("Retry"))
            {
                request_controller_list();
            }

            if (ImGui::Button("Return to Main Menu"))
            {
                m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eControllerMenuState::pendingControllerUnpairRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, ImVec2(300, 150), k_background_alpha, window_flags);

            ImGui::Text("Waiting for controller to unpair...");

            ImGui::End();
        } break;
    case eControllerMenuState::failedControllerUnpairRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, ImVec2(300, 150), k_background_alpha, window_flags);

            ImGui::Text("Failed to unpair controller!");

            if (ImGui::Button("Ok"))
            {
                request_controller_list();
            }

            if (ImGui::Button("Return to Main Menu"))
            {
                m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eControllerMenuState::pendingControllerPairRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, ImVec2(350, 300), k_background_alpha, window_flags);

            // Show progress
            if (m_pair_steps_total > 0)
            {
                char label_buffer[32];
                const float fraction= clampf01(static_cast<float>(m_pair_steps_completed) / static_cast<float>(m_pair_steps_total));
                _snprintf(label_buffer, sizeof(label_buffer), "Step %d/%d", m_pair_steps_completed, m_pair_steps_total);

                ImGui::TextWrapped(
                    "Unplug the controller.\n" \
                    "\n"
                    "Now press the controller's PS button.\n" \
                    "The red status LED will start blinking.\n" \
                    "Whenever it goes off, press the PS button again.\n" \
                    "Repeat this until the status LED finally remains lit.");

                ImGui::ProgressBar(fraction, ImVec2(250, 40), label_buffer);
            }
            else
            {
                ImGui::Text("Waiting for controller to pair...");
            }

            if (ImGui::Button("Cancel"))
            {
                assert(m_pendingBluetoothOpControllerIndex != -1);
                request_cancel_bluetooth_operation(m_pendingBluetoothOpControllerIndex);
            }

            ImGui::End();
        } break;
    case eControllerMenuState::failedControllerPairRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, ImVec2(300, 150), k_background_alpha, window_flags);

            ImGui::Text("Failed to pair controller!");

            if (ImGui::Button("Ok"))
            {
                request_controller_list();
            }

            if (ImGui::Button("Return to Main Menu"))
            {
                m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;

    case eControllerMenuState::pendingCancelBluetoothRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, ImVec2(300, 150), k_background_alpha, window_flags);
            ImGui::Text("Canceling bluetooth operation...");
            ImGui::End();
        } break;

    case eControllerMenuState::failedCancelBluetoothRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, ImVec2(300, 150), k_background_alpha, window_flags);

            ImGui::Text("Failed to cancel bluetooth operation (already completed?)");

            if (ImGui::Button("Ok"))
            {
                request_controller_list();
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

bool AppStage_ControllerSettings::onClientAPIEvent(
    ClientPSMoveAPI::eClientPSMoveAPIEvent event, 
    ClientPSMoveAPI::t_event_data_handle opaque_event_handle)
{
    bool bHandled= false;

    switch(event)
    {
    case ClientPSMoveAPI::disconnectedFromService:
        {
            m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
        } break;
    case ClientPSMoveAPI::controllerListUpdated:
        {
            // Ignore this if we have a pending bluetooth operation
            if (m_pendingBluetoothOpControllerIndex == -1)
            {
                request_controller_list();
            }
        } break;
    case ClientPSMoveAPI::opaqueServiceEvent:
        {
            const PSMoveProtocol::Response *event= GET_PSMOVEPROTOCOL_EVENT(opaque_event_handle);

            switch(event->type())
            {
            case PSMoveProtocol::Response_ResponseType_UNPAIR_REQUEST_COMPLETED:
                {
                    handle_controller_unpair_end_event(event);
                } break;
            case PSMoveProtocol::Response_ResponseType_PAIR_REQUEST_COMPLETED:
                {
                    handle_controller_pair_end_event(event);
                } break;
            case PSMoveProtocol::Response_ResponseType_BLUETOOTH_REQUEST_PROGRESS:
                {
                    handle_bluetooth_request_progress_event(event);
                } break;
            }
        } break;
    }

    return bHandled;
}

void AppStage_ControllerSettings::request_controller_list()
{
    if (m_menuState != AppStage_ControllerSettings::pendingControllerListRequest)
    {
        m_menuState= AppStage_ControllerSettings::pendingControllerListRequest;
        m_selectedControllerIndex= -1;
        m_pairedControllerInfos.clear();
        m_unpairedControllerInfos.clear();

        // Tell the psmove service that we we want a list of controllers connected to this machine
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_GET_CONTROLLER_LIST);

        ClientPSMoveAPI::send_opaque_request(&request, AppStage_ControllerSettings::handle_controller_list_response, this);
    }
}

void AppStage_ControllerSettings::handle_controller_list_response(
    ClientPSMoveAPI::eClientPSMoveResultCode ResultCode, 
    const ClientPSMoveAPI::t_request_id request_id, 
    ClientPSMoveAPI::t_response_handle response_handle, 
    void *userdata)
{
    AppStage_ControllerSettings *thisPtr= static_cast<AppStage_ControllerSettings *>(userdata);

    switch(ResultCode)
    {
        case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
        {
            const PSMoveProtocol::Response *response= GET_PSMOVEPROTOCOL_RESPONSE(response_handle);

            for (int controller_index= 0; controller_index < response->result_controller_list().controllers_size(); ++controller_index)
            {
                const auto &ControllerResponse= response->result_controller_list().controllers(controller_index);

                AppStage_ControllerSettings::ControllerInfo ControllerInfo;

                ControllerInfo.ControllerID= ControllerResponse.controller_id();

                switch(ControllerResponse.connection_type())
                {
                case PSMoveProtocol::Response_ResultControllerList_ControllerInfo_ConnectionType_BLUETOOTH:
                    ControllerInfo.ConnectionType= AppStage_ControllerSettings::Bluetooth;
                    break;
                case PSMoveProtocol::Response_ResultControllerList_ControllerInfo_ConnectionType_USB:
                    ControllerInfo.ConnectionType= AppStage_ControllerSettings::USB;
                    break;
                default:
                    assert(0 && "unreachable");
                }

                switch(ControllerResponse.controller_type())
                {
                case PSMoveProtocol::PSMOVE:
                    ControllerInfo.ControllerType= AppStage_ControllerSettings::PSMove;
                    break;
                case PSMoveProtocol::PSNAVI:
                    ControllerInfo.ControllerType= AppStage_ControllerSettings::PSNavi;
                    break;
                default:
                    assert(0 && "unreachable");
                }

                ControllerInfo.DevicePath= ControllerResponse.device_path();
                ControllerInfo.DeviceSerial= ControllerResponse.device_serial();
                ControllerInfo.HostSerial= ControllerResponse.host_serial();

                if (ControllerResponse.host_serial().length() > 0 && 
                    ControllerResponse.host_serial() != "00:00:00:00:00:00")
                {
                    thisPtr->m_pairedControllerInfos.push_back( ControllerInfo );
                }
                else
                {
                    thisPtr->m_unpairedControllerInfos.push_back( ControllerInfo );
                }
            }

            thisPtr->m_selectedControllerIndex= (thisPtr->m_pairedControllerInfos.size() > 0) ? 0 : -1;
            thisPtr->m_menuState= AppStage_ControllerSettings::idle;
        } break;

        case ClientPSMoveAPI::_clientPSMoveResultCode_error:
        case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
        { 
            thisPtr->m_menuState= AppStage_ControllerSettings::failedControllerListRequest;
        } break;
    }
}

void AppStage_ControllerSettings::request_controller_unpair(
    int controllerID)
{
    if (m_menuState != AppStage_ControllerSettings::pendingControllerUnpairRequest && 
        m_menuState != AppStage_ControllerSettings::pendingControllerPairRequest)
    {
        m_menuState= AppStage_ControllerSettings::pendingControllerUnpairRequest;
        m_pendingBluetoothOpControllerIndex= controllerID;

        // Tell the psmove service that we want to unpair the given controller
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_UNPAIR_CONTROLLER);
        request->mutable_unpair_controller()->set_controller_id(controllerID);

        ClientPSMoveAPI::send_opaque_request(&request, AppStage_ControllerSettings::handle_controller_unpair_start_response, this);
    }
}

void AppStage_ControllerSettings::handle_controller_unpair_start_response(
    ClientPSMoveAPI::eClientPSMoveResultCode ResultCode, 
    const ClientPSMoveAPI::t_request_id request_id, 
    ClientPSMoveAPI::t_response_handle response_handle, 
    void *userdata)
{
    AppStage_ControllerSettings *thisPtr= static_cast<AppStage_ControllerSettings *>(userdata);

    switch(ResultCode)
    {
        case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
        {
            thisPtr->m_menuState= AppStage_ControllerSettings::pendingControllerUnpairRequest;
        } break;

        case ClientPSMoveAPI::_clientPSMoveResultCode_error:
        case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
        { 
            thisPtr->m_menuState= AppStage_ControllerSettings::failedControllerUnpairRequest;
            thisPtr->m_pendingBluetoothOpControllerIndex= -1;
        } break;
    }
}

void AppStage_ControllerSettings::handle_controller_unpair_end_event(
    const PSMoveProtocol::Response *event)
{
    // No longer have a pending bluetooth operation
    m_pendingBluetoothOpControllerIndex= -1;

    switch(event->result_code())
    {
        case PSMoveProtocol::Response_ResultCode_RESULT_OK:
        {
            // Refresh the list of controllers now that we have confirmation the controller is unpaired
            request_controller_list();
        } break;

        case PSMoveProtocol::Response_ResultCode_RESULT_ERROR:
        case PSMoveProtocol::Response_ResultCode_RESULT_CANCELED:
        { 
            this->m_menuState= AppStage_ControllerSettings::failedControllerUnpairRequest;
        } break;
    }
}

void AppStage_ControllerSettings::request_controller_pair(
    int controllerID)
{
    if (m_menuState != AppStage_ControllerSettings::pendingControllerUnpairRequest && 
        m_menuState != AppStage_ControllerSettings::pendingControllerPairRequest)
    {
        m_menuState= AppStage_ControllerSettings::pendingControllerPairRequest;
        m_pendingBluetoothOpControllerIndex= controllerID;
        m_pair_steps_completed = 0;
        m_pair_steps_total = 0;

        // Tell the psmove service that we want to pair the given controller
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_PAIR_CONTROLLER);
        request->mutable_pair_controller()->set_controller_id(controllerID);

        ClientPSMoveAPI::send_opaque_request(&request, AppStage_ControllerSettings::handle_controller_pair_start_response, this);
    }
}

void AppStage_ControllerSettings::handle_controller_pair_start_response(
    ClientPSMoveAPI::eClientPSMoveResultCode ResultCode, 
    const ClientPSMoveAPI::t_request_id request_id, 
    ClientPSMoveAPI::t_response_handle response_handle, 
    void *userdata)
{
    AppStage_ControllerSettings *thisPtr= static_cast<AppStage_ControllerSettings *>(userdata);

    switch(ResultCode)
    {
        case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
        {
            thisPtr->m_menuState= AppStage_ControllerSettings::pendingControllerPairRequest;
        } break;

        case ClientPSMoveAPI::_clientPSMoveResultCode_error:
        case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
        { 
            thisPtr->m_menuState= AppStage_ControllerSettings::failedControllerPairRequest;
            thisPtr->m_pendingBluetoothOpControllerIndex= -1;
        } break;
    }
}

void AppStage_ControllerSettings::handle_controller_pair_end_event(
    const PSMoveProtocol::Response *event)
{
    // No longer have a pending bluetooth operation
    m_pendingBluetoothOpControllerIndex= -1;

    switch(event->result_code())
    {
        case PSMoveProtocol::Response_ResultCode_RESULT_OK:
        {
            // Refresh the list of controllers now that we have confirmation the controller is unpaired
            request_controller_list();
        } break;

        case PSMoveProtocol::Response_ResultCode_RESULT_ERROR:
        case PSMoveProtocol::Response_ResultCode_RESULT_CANCELED:
        { 
            this->m_menuState= AppStage_ControllerSettings::failedControllerPairRequest;
        } break;
    }
}

void AppStage_ControllerSettings::handle_bluetooth_request_progress_event(
    const PSMoveProtocol::Response *event)
{
    const PSMoveProtocol::Response_ResultBluetoothRequestProgress &progress= 
        event->result_bluetooth_request_progress();

    if (m_pendingBluetoothOpControllerIndex == progress.controller_id())
    {
        m_pair_steps_completed = progress.steps_completed();
        m_pair_steps_total = progress.total_steps();
    }
}

void AppStage_ControllerSettings::request_cancel_bluetooth_operation(
    int controllerID)
{
    if (m_pendingBluetoothOpControllerIndex != -1 &&
        (m_menuState == AppStage_ControllerSettings::pendingControllerUnpairRequest ||
         m_menuState == AppStage_ControllerSettings::pendingControllerPairRequest))
    {
        m_menuState= AppStage_ControllerSettings::pendingCancelBluetoothRequest;
        m_pendingBluetoothOpControllerIndex= controllerID;

        // Tell the psmove service that we want to cancel the pending bluetooth request
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_CANCEL_BLUETOOTH_REQUEST);
        request->mutable_cancel_bluetooth_request()->set_controller_id(controllerID);

        ClientPSMoveAPI::send_opaque_request(&request, AppStage_ControllerSettings::handle_cancel_bluetooth_operation_response, this);
    }
}

void AppStage_ControllerSettings::handle_cancel_bluetooth_operation_response(
    ClientPSMoveAPI::eClientPSMoveResultCode ResultCode, 
    const ClientPSMoveAPI::t_request_id request_id, 
    ClientPSMoveAPI::t_response_handle response_handle, 
    void *userdata)
{
    AppStage_ControllerSettings *thisPtr= static_cast<AppStage_ControllerSettings *>(userdata);

    if (thisPtr->m_menuState == AppStage_ControllerSettings::pendingCancelBluetoothRequest)
    {
        // No longer pending a bluetooth operation in any case
        thisPtr->m_pendingBluetoothOpControllerIndex= -1;

        switch(ResultCode)
        {
            case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
            case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
            {
                // Refresh the list of controllers now that we have confirmation the controller is unpaired
                thisPtr->request_controller_list();
            } break;

            case ClientPSMoveAPI::_clientPSMoveResultCode_error:
            { 
                thisPtr->m_menuState= AppStage_ControllerSettings::failedCancelBluetoothRequest;
            } break;
        }
    }
}
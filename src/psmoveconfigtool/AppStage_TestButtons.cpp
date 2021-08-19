//-- inludes -----
#include "AppStage_TestButtons.h"
#include "AppStage_ControllerSettings.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "PSMoveClient_CAPI.h"
#include "Logger.h"
#include "MathGLM.h"
#include "Renderer.h"
#include "UIConstants.h"

#include <imgui.h>

#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

//-- statics ----
const char *AppStage_TestButtons::APP_STAGE_NAME = "TestButtons";

//-- private methods -----
static void drawController(PSMController *controllerView, const glm::mat4 &transform, const glm::vec3 &bulb_color);

//-- public methods -----
AppStage_TestButtons::AppStage_TestButtons(App *app)
    : AppStage(app)
    , m_menuState(AppStage_TestButtons::inactive)
    , m_controllerView(nullptr)
{
}

AppStage_TestButtons::~AppStage_TestButtons()
{
}

void AppStage_TestButtons::enter()
{
    const AppStage_ControllerSettings *controllerSettings =
        m_app->getAppStage<AppStage_ControllerSettings>();
    const AppStage_ControllerSettings::ControllerInfo *controllerInfo =
        controllerSettings->getSelectedControllerInfo();

    m_app->setCameraType(_cameraFixed);
    m_app->getOrbitCamera()->resetOrientation();
    m_app->getOrbitCamera()->setCameraOrbitRadius(1000.f); // zoom out to see the magnetometer data at scale

    assert(controllerInfo->ControllerID != -1);
    assert(m_controllerView == nullptr);
	PSM_AllocateControllerListener(controllerInfo->ControllerID);
	m_controllerView= PSM_GetController(controllerInfo->ControllerID);


    m_menuState = eMenuState::waitingForStreamStartResponse;

	PSMRequestID request_id;
	PSM_StartControllerDataStreamAsync(m_controllerView->ControllerID, PSMStreamFlags_defaultStreamOptions, &request_id);
	PSM_RegisterCallback(request_id, &AppStage_TestButtons::handle_acquire_controller, this);
}

void AppStage_TestButtons::exit()
{
    assert(m_controllerView != nullptr);
    PSM_FreeControllerListener(m_controllerView->ControllerID);
    m_controllerView = nullptr;
    m_menuState = eMenuState::inactive;

    // Reset the orbit camera back to default orientation and scale
    m_app->getOrbitCamera()->reset();
}

void AppStage_TestButtons::update()
{
}

void AppStage_TestButtons::render()
{
    const float modelScale = 18.f;
    glm::mat4 scaleAndRotateModelX90 =
        glm::rotate(
        glm::scale(glm::mat4(1.f), glm::vec3(modelScale, modelScale, modelScale)),
        90.f, glm::vec3(1.f, 0.f, 0.f));

    switch (m_menuState)
    {
    case eMenuState::waitingForStreamStartResponse:
        {
        } break;
    case eMenuState::failedStreamStart:
        {
        } break;
    case eMenuState::idle:
        {
            // Draw the psmove model in the middle
			drawController(m_controllerView, scaleAndRotateModelX90, glm::vec3(0.f, 0.f, 0.f));
        } break;
    default:
        assert(0 && "unreachable");
    }
}

inline void showButtonState(const char *szButtonName, PSMButtonState buttonState)
{
    ImGui::Text(szButtonName);
    ImGui::SameLine();
    switch (buttonState)
    {
    case PSMButtonState_UP:
    case PSMButtonState_RELEASED:
        ImGui::Text(": Released");
        break;
    case PSMButtonState_PRESSED:
    case PSMButtonState_DOWN:
        ImGui::Text(": Pressed");
        break;
    }
}

inline void showFloatAxis(const char *szAxisName, float fAxisValue)
{
    ImGui::Text(szAxisName);
    ImGui::SameLine();
    ImGui::Text(": %.2f", fAxisValue);
}

inline void showSignedCharAxis(const char *szAxisName, unsigned char axisValue)
{
    const float fAxisValue= (static_cast<float>(axisValue) / 127.f) - 1.f;

    showFloatAxis(szAxisName, fAxisValue);
}

inline void showUnsignedCharAxis(const char *szAxisName, unsigned char axisValue)
{
    const float fAxisValue= static_cast<float>(axisValue) / 255.f;

    showFloatAxis(szAxisName, fAxisValue);
}

void AppStage_TestButtons::renderUI()
{
    const float k_panel_width = 500;
    const char *k_window_title = "Test Buttons";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case eMenuState::waitingForStreamStartResponse:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Waiting for controller stream to start...");

            ImGui::End();
        } break;
    case eMenuState::failedStreamStart:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Failed to start controller stream!");

            if (ImGui::Button("Ok"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::SameLine();

            if (ImGui::Button("Return to Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eMenuState::idle:
        {
            ImGui::SetNextWindowPosCenter();
            //ImGui::SetNextWindowSize(ImVec2(350, 490));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            switch (m_controllerView->ControllerType)
            {
            case PSMController_Move:
                {
                    const PSMPSMove &psmove= m_controllerView->ControllerState.PSMoveState;

                    ImGui::Text("PSMove (Controller %d)", m_controllerView->ControllerID);
                    ImGui::Separator();

                    ImGui::Text("[Buttons]");
                    showButtonState("Triangle", psmove.TriangleButton);
                    showButtonState("Circle", psmove.CircleButton);
                    showButtonState("Cross", psmove.CrossButton);
                    showButtonState("Square", psmove.SquareButton);
                    showButtonState("Select", psmove.SelectButton);
                    showButtonState("Start", psmove.StartButton);
                    showButtonState("PS", psmove.PSButton);
                    showButtonState("Move", psmove.MoveButton);
                    showButtonState("T", psmove.TriggerButton);

                    ImGui::Separator();
                    ImGui::Text("[Axes]");
                    showUnsignedCharAxis("Trigger", psmove.TriggerValue);
                }
                break;
            case PSMController_Navi:
                {
                    const PSMPSNavi &psnavi= m_controllerView->ControllerState.PSNaviState;

                    ImGui::Text("PSNavi (Controller %d)", m_controllerView->ControllerID);
                    ImGui::Separator();

                    ImGui::Text("[Buttons]");
                    showButtonState("L1", psnavi.L1Button);
                    showButtonState("L2", psnavi.L2Button);
                    showButtonState("L3", psnavi.L3Button);
                    showButtonState("Circle", psnavi.CircleButton);
                    showButtonState("Cross", psnavi.CrossButton);
                    showButtonState("PS", psnavi.PSButton);
                    showButtonState("DPad Up", psnavi.DPadUpButton);
                    showButtonState("DPad Right", psnavi.DPadRightButton);
                    showButtonState("DPad Down", psnavi.DPadDownButton);
                    showButtonState("DPad Left", psnavi.DPadLeftButton);

                    ImGui::Separator();
                    ImGui::Text("[Axes]");
                    showUnsignedCharAxis("Trigger", psnavi.TriggerValue);
                    showSignedCharAxis("Stick X", psnavi.Stick_XAxis);
                    showSignedCharAxis("Stick Y", psnavi.Stick_YAxis);
                }
                break;
            case PSMController_DualShock4:
                {
                    const PSMDualShock4 &ds4= m_controllerView->ControllerState.PSDS4State;

                    ImGui::Text("DualShock4 (Controller %d)", m_controllerView->ControllerID);
                    ImGui::Separator();

                    ImGui::Text("[Buttons]");
                    showButtonState("DPad Up", ds4.DPadUpButton);
                    showButtonState("DPad Down", ds4.DPadDownButton);
                    showButtonState("DPad Left", ds4.DPadLeftButton);
                    showButtonState("DPad Right", ds4.DPadRightButton);
                    showButtonState("Square", ds4.SquareButton);
                    showButtonState("Cross", ds4.CrossButton);
                    showButtonState("Circle", ds4.CircleButton);
                    showButtonState("Triangle", ds4.TriangleButton);
                    showButtonState("L1", ds4.L1Button);
                    showButtonState("R1", ds4.R1Button);
                    showButtonState("L2", ds4.L2Button);
                    showButtonState("R2", ds4.R2Button);
                    showButtonState("L3", ds4.L3Button);
                    showButtonState("R3", ds4.R3Button);
                    showButtonState("Share", ds4.ShareButton);
                    showButtonState("Options", ds4.OptionsButton);
                    showButtonState("PS", ds4.PSButton);
                    showButtonState("Trackpad", ds4.TrackPadButton);

                    ImGui::Separator();
                    ImGui::Text("[Axes]");
                    showFloatAxis("Left Stick X", ds4.LeftAnalogX);
                    showFloatAxis("Left Stick Y", ds4.LeftAnalogY);
                    showFloatAxis("Right Stick X", ds4.LeftAnalogX);
                    showFloatAxis("Right Stick Y", ds4.LeftAnalogY);
                    showFloatAxis("Left Trigger", ds4.LeftTriggerValue);
                    showFloatAxis("Right Trigger", ds4.RightTriggerValue);
                }
                break;
            case PSMController_Virtual:
                {
                    const PSMVirtualController &virtual_controller= m_controllerView->ControllerState.VirtualController;

                    ImGui::Text("Virtual Controller (Controller %d)", m_controllerView->ControllerID);
                    ImGui::Text("Vendor ID: 0x%04x", virtual_controller.vendorID);
                    ImGui::Text("Product ID: 0x%04x", virtual_controller.productID);

                    ImGui::Separator();
                    if (virtual_controller.numButtons > 0)
                    {
                        ImGui::Text("[Buttons]");
                        for (int buttonIndex = 0; buttonIndex < virtual_controller.numButtons; ++buttonIndex)
                        {
                            char szButtonName[32];
                            snprintf(szButtonName, sizeof(szButtonName), "Button%d", buttonIndex);
                            showButtonState(szButtonName, virtual_controller.buttonStates[buttonIndex]);
                        }
                    }
                    else
                    {
                        ImGui::Text("[No Buttons]");
                    }

                    ImGui::Separator();
                    if (virtual_controller.numAxes > 0)
                    {
                        ImGui::Text("[Axes]");
                        for (int axisIndex = 0; axisIndex < virtual_controller.numAxes; ++axisIndex)
                        {
                            char szAxisName[32];
                            snprintf(szAxisName, sizeof(szAxisName), "Axis%d", axisIndex);
                            showSignedCharAxis(szAxisName, virtual_controller.axisStates[axisIndex]);
                        }
                    }
                    else
                    {
                        ImGui::Text("[No Axes]");
                    }
                }
                break;
            }

            if (ImGui::Button("Controller Settings"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }
            ImGui::SameLine();
            if (ImGui::Button("Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;

    default:
        assert(0 && "unreachable");
    }
}

//-- private methods -----
void AppStage_TestButtons::handle_acquire_controller(
    const PSMResponseMessage *response,
    void *userdata)
{
    AppStage_TestButtons *thisPtr = reinterpret_cast<AppStage_TestButtons *>(userdata);

    if (response->result_code == PSMResult_Success)
    {
        thisPtr->m_menuState = AppStage_TestButtons::idle;
    }
    else
    {
        thisPtr->m_menuState = AppStage_TestButtons::failedStreamStart;
    }
}

void AppStage_TestButtons::request_exit_to_app_stage(const char *app_stage_name)
{
    PSMRequestID request_id;
	PSM_StopControllerDataStreamAsync(m_controllerView->ControllerID, &request_id);
    PSM_EatResponse(request_id);

    m_app->setAppStage(app_stage_name);
}

//-- private methods -----
static void drawController(PSMController *controllerView, const glm::mat4 &transform, const glm::vec3 &bulb_color)
{
    switch(controllerView->ControllerType)
    {
    case PSMController_Move:
        drawPSMoveModel(transform, bulb_color);
        break;
    case PSMController_Navi:
        drawPSNaviModel(transform);
        break;
    case PSMController_DualShock4:
        drawPSDualShock4Model(transform, bulb_color);
        break;
    case PSMController_Virtual:
        drawVirtualControllerModel(transform, bulb_color);
        break;
    }
}

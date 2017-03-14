//-- inludes -----
#include "AppStage_ColorCalibration.h"
#include "AppStage_TrackerSettings.h"
#include "AppStage_MainMenu.h"
#include "AssetManager.h"
#include "App.h"
#include "Camera.h"
#include "ClientLog.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include "SharedTrackerState.h"

#include "SDL_keycode.h"
#include "SDL_opengl.h"

#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <imgui.h>
#include <algorithm>
#include <chrono>
#include <thread>

#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

//-- statics ----
const char *AppStage_ColorCalibration::APP_STAGE_NAME = "ColorCalibration";

//-- constants -----
static const char *k_video_display_mode_names[] = {
    "BGR",
    "HSV",
    "Masked"
};

static const char *k_tracking_color_names[] = {
    "Magenta",
    "Cyan",
    "Yellow",
    "Red",
    "Green",
    "Blue"
};

//-- private definitions -----
class VideoBufferState
{
public:
    VideoBufferState(PSMTracker *trackerView)
        : videoTexture(nullptr)
        , bgrBuffer(nullptr)
        , hsvBuffer(nullptr)
        , gsLowerBuffer(nullptr)
        , gsUpperBuffer(nullptr)
        , maskedBuffer(nullptr)
    {
        const int frameWidth = static_cast<int>(trackerView->tracker_info.tracker_screen_dimensions.x);
        const int frameHeight = static_cast<int>(trackerView->tracker_info.tracker_screen_dimensions.y);

        // Create a texture to render the video frame to
        videoTexture = new TextureAsset();
        videoTexture->init(
            frameWidth,
            frameHeight,
            GL_RGB, // texture format
            GL_BGR, // buffer format
            nullptr);

        bgrBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC3);
        hsvBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC3);
        gsLowerBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC1);
        gsUpperBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC1);
        maskedBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC3);
    }

    virtual ~VideoBufferState()
    {
        if (maskedBuffer != nullptr)
        {
            delete maskedBuffer;
            maskedBuffer = nullptr;
        }

        if (gsLowerBuffer != nullptr)
        {
            delete gsLowerBuffer;
            gsLowerBuffer = nullptr;
        }

        if (gsUpperBuffer != nullptr)
        {
            delete gsUpperBuffer;
            gsUpperBuffer = nullptr;
        }

        if (hsvBuffer != nullptr)
        {
            delete hsvBuffer;
            hsvBuffer = nullptr;
        }

        if (bgrBuffer != nullptr)
        {
            delete bgrBuffer;
            bgrBuffer = nullptr;
        }

        if (videoTexture != nullptr)
        {
            delete videoTexture;
            videoTexture = nullptr;
        }
    }

    TextureAsset *videoTexture;
    cv::Mat *bgrBuffer; // source video frame
    cv::Mat *hsvBuffer; // source frame converted to HSV color space
    cv::Mat *gsLowerBuffer; // HSV image clamped by HSV range into grayscale mask
    cv::Mat *gsUpperBuffer; // HSV image clamped by HSV range into grayscale mask
    cv::Mat *maskedBuffer; // bgr image ANDed together with grayscale mask
};

//-- public methods -----
AppStage_ColorCalibration::AppStage_ColorCalibration(App *app)
    : AppStage(app)
	, m_overrideControllerId(-1)
    , m_masterControllerView(nullptr)
	, m_pendingControllerStartCount(0)
    , m_areAllControllerStreamsActive(false)
    , m_lastMasterControllerSeqNum(-1)
	, m_overrideHmdId(-1)
	, m_hmdView(nullptr)
	, m_isHmdStreamActive(false)
	, m_lastHmdSeqNum(-1)
    , m_trackerView(nullptr)
    , m_menuState(AppStage_ColorCalibration::inactive)
    , m_video_buffer_state(nullptr)
    , m_videoDisplayMode(AppStage_ColorCalibration::eVideoDisplayMode::mode_bgr)
	, m_trackerFramerate(0)
    , m_trackerExposure(0)
    , m_trackerGain(0)
	, m_bTurnOnAllControllers(false)
	, m_bAutoChangeController(false)
	, m_bAutoChangeColor(false)
	, m_bAutoChangeTracker(false)
	, m_bShowWindows(true)
    , m_masterTrackingColorType(PSMTrackingColorType_Magenta)
{ 
    memset(m_colorPresets, 0, sizeof(m_colorPresets));
}

void AppStage_ColorCalibration::enter()
{
    const AppStage_TrackerSettings *trackerSettings =
        m_app->getAppStage<AppStage_TrackerSettings>();
    const PSMClientTrackerInfo *trackerInfo = trackerSettings->getSelectedTrackerInfo();
    assert(trackerInfo->tracker_id != -1);

    m_app->setCameraType(_cameraFixed);

	tracker_count = trackerSettings->get_tracker_count();
	tracker_index = trackerSettings->get_tracker_Index();

    // Use the tracker selected from the tracker settings menu
    assert(m_trackerView == nullptr);
	PSM_AllocateTrackerListener(trackerInfo->tracker_id, trackerInfo);
	m_trackerView = PSM_GetTracker(trackerInfo->tracker_id);

	if (m_overrideHmdId != -1)
	{
		assert(m_hmdView == nullptr);
		PSM_AllocateHmdListener(m_overrideHmdId);
		m_hmdView = PSM_GetHmd(m_overrideHmdId);
		m_isHmdStreamActive = false;
		m_lastHmdSeqNum = -1;
	}
	else
	{
		// Assume that we can bind to controller 0 if no controller override is given
		const int masterControllerID = (m_overrideControllerId != -1) ? m_overrideControllerId : 0;

		m_controllerViews.clear();
		m_controllerTrackingColorTypes.clear();

		for (int list_index = 0; list_index < trackerSettings->get_controller_count(); ++list_index)
		{
			const AppStage_TrackerSettings::ControllerInfo *controller_info= trackerSettings->get_controller_info(list_index);
			PSM_AllocateControllerListener(controller_info->ControllerID);
			PSMController *controllerView= PSM_GetController(controller_info->ControllerID);

			if (masterControllerID == controller_info->ControllerID)
			{
				assert(m_masterControllerView == nullptr);
				m_masterControllerView= controllerView;
			}

			m_controllerViews.push_back(controllerView);
			m_controllerTrackingColorTypes.push_back(controller_info->TrackingColorType);
		}
		
		m_areAllControllerStreamsActive = false;
		m_lastMasterControllerSeqNum = -1;
		m_bTurnOnAllControllers= false;
		m_pendingControllerStartCount= false;

		m_bAutoChangeController = (m_bAutoChangeController) ? m_bAutoChangeController : false;
		m_bAutoChangeColor = (m_bAutoChangeColor) ? m_bAutoChangeColor : false;
		m_bAutoChangeTracker = (m_bAutoChangeTracker) ? m_bAutoChangeTracker : false;
	}

    // Request to start the tracker
    // Wait for the tracker response before requesting the controller
    assert(m_video_buffer_state == nullptr);
    request_tracker_start_stream();

    // In parallel, Get the settings for the selected tracker
    request_tracker_get_settings();
}

void AppStage_ColorCalibration::exit()
{
    setState(AppStage_ColorCalibration::inactive);

    release_devices();
}

void AppStage_ColorCalibration::update()
{
    bool bControllerDataUpdatedThisFrame= false;

    if (m_menuState == eMenuState::waitingForStreamStartResponse)
    {
        if (m_areAllControllerStreamsActive && m_masterControllerView->OutputSequenceNum != m_lastMasterControllerSeqNum)
        {
            request_set_controller_tracking_color(m_masterControllerView, m_masterTrackingColorType);
            setState(eMenuState::manualConfig);
        }
		else if (m_isHmdStreamActive && m_hmdView->OutputSequenceNum != m_lastHmdSeqNum)
		{
			setState(eMenuState::manualConfig);
		}
    }

    // Try and read the next video frame from shared memory
    if (m_video_buffer_state != nullptr)
    {
		const unsigned char *video_buffer= nullptr;
        if (PSM_PollTrackerVideoStream(m_trackerView->tracker_info.tracker_id) == PSMResult_Success &&
			PSM_GetTrackerVideoFrameBuffer(m_trackerView->tracker_info.tracker_id, &video_buffer) == PSMResult_Success)
        {
            const int frameWidth = static_cast<int>(m_trackerView->tracker_info.tracker_screen_dimensions.x);
            const int frameHeight = static_cast<int>(m_trackerView->tracker_info.tracker_screen_dimensions.y);
            const unsigned char *display_buffer = video_buffer;
            const TrackerColorPreset &preset = getColorPreset();

            // Copy the video frame buffer into the bgr opencv buffer
            {
                const cv::Mat videoBufferMat(frameHeight, frameWidth, CV_8UC3, const_cast<unsigned char *>(video_buffer));

                videoBufferMat.copyTo(*m_video_buffer_state->bgrBuffer);
            }

            // Convert the video buffer to the HSV color space
            cv::cvtColor(*m_video_buffer_state->bgrBuffer, *m_video_buffer_state->hsvBuffer, cv::COLOR_BGR2HSV);

            // Clamp the HSV image, taking into account wrapping the hue angle
            {
                const float hue_min = preset.hue_center - preset.hue_range;
                const float hue_max = preset.hue_center + preset.hue_range;
                const float saturation_min = clampf(preset.saturation_center - preset.saturation_range, 0, 255);
                const float saturation_max = clampf(preset.saturation_center + preset.saturation_range, 0, 255);
                const float value_min = clampf(preset.value_center - preset.value_range, 0, 255);
                const float value_max = clampf(preset.value_center + preset.value_range, 0, 255);

                if (hue_min < 0)
                {
                    cv::inRange(
                        *m_video_buffer_state->hsvBuffer,
                        cv::Scalar(0, saturation_min, value_min),
                        cv::Scalar(clampf(hue_max, 0, 180), saturation_max, value_max),
                        *m_video_buffer_state->gsLowerBuffer);
                    cv::inRange(
                        *m_video_buffer_state->hsvBuffer,
                        cv::Scalar(clampf(180 + hue_min, 0, 180), saturation_min, value_min),
                        cv::Scalar(180, saturation_max, value_max),
                        *m_video_buffer_state->gsUpperBuffer);
                    cv::bitwise_or(
                        *m_video_buffer_state->gsLowerBuffer, 
                        *m_video_buffer_state->gsUpperBuffer, 
                        *m_video_buffer_state->gsLowerBuffer);
                }
                else if (hue_max > 180)
                {
                    cv::inRange(
                        *m_video_buffer_state->hsvBuffer,
                        cv::Scalar(0, saturation_min, value_min),
                        cv::Scalar(clampf(hue_max - 180, 0, 180), saturation_max, value_max),
                        *m_video_buffer_state->gsLowerBuffer);
                    cv::inRange(
                        *m_video_buffer_state->hsvBuffer,
                        cv::Scalar(clampf(hue_min, 0, 180), saturation_min, value_min),
                        cv::Scalar(180, saturation_max, value_max),
                        *m_video_buffer_state->gsUpperBuffer);
                    cv::bitwise_or(
                        *m_video_buffer_state->gsLowerBuffer, 
                        *m_video_buffer_state->gsUpperBuffer, 
                        *m_video_buffer_state->gsLowerBuffer);
                }
                else
                {
                    cv::inRange(
                        *m_video_buffer_state->hsvBuffer,
                        cv::Scalar(hue_min, saturation_min, value_min),
                        cv::Scalar(hue_max, saturation_max, value_max),
                        *m_video_buffer_state->gsLowerBuffer);
                }
            }

            // Mask out the original video frame with the HSV filtered mask
            *m_video_buffer_state->maskedBuffer = cv::Scalar(0, 0, 0);
            cv::bitwise_and(
                *m_video_buffer_state->bgrBuffer, 
                *m_video_buffer_state->bgrBuffer, 
                *m_video_buffer_state->maskedBuffer, 
                *m_video_buffer_state->gsLowerBuffer);

            switch (m_videoDisplayMode)
            {
            case AppStage_ColorCalibration::mode_bgr:
                display_buffer = m_video_buffer_state->bgrBuffer->data;
                break;
            case AppStage_ColorCalibration::mode_hsv:
                display_buffer = m_video_buffer_state->hsvBuffer->data;
                break;
            case AppStage_ColorCalibration::mode_masked:
                display_buffer = m_video_buffer_state->maskedBuffer->data;
                break;
            default:
                assert(0 && "unreachable");
                break;
            }

            // Display the selected buffer
            m_video_buffer_state->videoTexture->copyBufferIntoTexture(display_buffer);
        }
    }
}

void AppStage_ColorCalibration::render()
{
    // If there is a video frame available to render, show it
    if (m_video_buffer_state != nullptr)
    {
        unsigned int texture_id = m_video_buffer_state->videoTexture->texture_id;

        if (texture_id != 0)
        {
            drawFullscreenTexture(texture_id);
        }
    }
}

void AppStage_ColorCalibration::renderUI()
{
    const float k_panel_width = 300.f;
    const char *k_window_title = "Color Calibration";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;
	int auto_calib_sleep = 150;

    switch (m_menuState)
    {
    case eMenuState::manualConfig:
    {
        // Video Control Panel
		if (m_bShowWindows)
        {
            ImGui::SetNextWindowPos(ImVec2(10.f, 10.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 260));
            ImGui::Begin(k_window_title, nullptr, window_flags);

			if (ImGui::Button("Return to Main Menu"))
			{
				request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
			}
            
            if (ImGui::Button("Return to Tracker Settings"))
            {
                request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }

            if (m_video_buffer_state != nullptr)
            {
                if (ImGui::Button("<##Filter"))
                {
                    m_videoDisplayMode =
                        static_cast<eVideoDisplayMode>(
                        (m_videoDisplayMode + eVideoDisplayMode::MAX_VIDEO_DISPLAY_MODES - 1)
                        % eVideoDisplayMode::MAX_VIDEO_DISPLAY_MODES);
                }
                ImGui::SameLine();
                if (ImGui::Button(">##Filter"))
                {
                    m_videoDisplayMode =
                        static_cast<eVideoDisplayMode>(
                        (m_videoDisplayMode + 1) % eVideoDisplayMode::MAX_VIDEO_DISPLAY_MODES);
                }
                ImGui::SameLine();
                ImGui::Text("Video [F]ilter Mode: %s", k_video_display_mode_names[m_videoDisplayMode]);
				
				int frame_rate_positive_change = 10;
				int frame_rate_negative_change = -10;
				
				double val = m_trackerFramerate;

				if (val == 2) { frame_rate_positive_change = 1; frame_rate_negative_change = 0; }
				else if (val == 3) { frame_rate_positive_change = 2; frame_rate_negative_change = -1; }
				else if (val == 5) { frame_rate_positive_change = 3; frame_rate_negative_change = -0; }
				else if (val == 8) { frame_rate_positive_change = 2; frame_rate_negative_change = -3; }
				else if (val == 10) { frame_rate_positive_change = 5; frame_rate_negative_change = -2; }
				else if (val == 15) { frame_rate_positive_change = 5; frame_rate_negative_change = -5; }
				else if (val == 20) { frame_rate_positive_change = 5; frame_rate_negative_change = -5; }
				else if (val == 25) { frame_rate_positive_change = 5; frame_rate_negative_change = -5; }
				else if (val == 30) { { frame_rate_negative_change = -5; } }
				else if (val == 60) { { frame_rate_positive_change = 15; } }
				else if (val == 75) { frame_rate_positive_change = 0; frame_rate_negative_change = -15; }
				else if (val == 83) { frame_rate_positive_change = 0; frame_rate_negative_change = -8; }

				if (ImGui::Button("-##Framerate"))
				{
					request_tracker_set_frame_rate(m_trackerFramerate + frame_rate_negative_change);
				}
				ImGui::SameLine();
				if (ImGui::Button("+##Framerate"))
				{
					request_tracker_set_frame_rate(m_trackerFramerate + frame_rate_positive_change);
				}
				ImGui::SameLine();
				ImGui::Text("Framerate: %.0f", m_trackerFramerate);

                if (ImGui::Button("-##Exposure"))
                {
                    request_tracker_set_exposure(m_trackerExposure - 8);
                }
                ImGui::SameLine();
                if (ImGui::Button("+##Exposure"))
                {
                    request_tracker_set_exposure(m_trackerExposure + 8);
                }
                ImGui::SameLine();
                ImGui::Text("Exposure: %.0f", m_trackerExposure);

                if (ImGui::Button("-##Gain"))
                {
                    request_tracker_set_gain(m_trackerGain - 8);
                }
                ImGui::SameLine();
                if (ImGui::Button("+##Gain"))
                {
                    request_tracker_set_gain(m_trackerGain + 8);
                }
                ImGui::SameLine();
                ImGui::Text("Gain: %.0f", m_trackerGain);

                // Render all of the option sets fetched from the settings query
                for (auto it = m_trackerOptions.begin(); it != m_trackerOptions.end(); ++it)
                {
                    TrackerOption &option = *it;
                    const int value_count = static_cast<int>(option.option_strings.size());

                    ImGui::PushID(option.option_name.c_str());
                    if (ImGui::Button("<"))
                    {
                        request_tracker_set_option(option, (option.option_index + value_count - 1) % value_count);
                    }
                    ImGui::SameLine();
                    if (ImGui::Button(">"))
                    {
                        request_tracker_set_option(option, (option.option_index + 1) % value_count);
                    }
                    ImGui::SameLine();
                    ImGui::Text("%s: %s", option.option_name.c_str(), option.option_strings[option.option_index].c_str());
                    ImGui::PopID();
                }

				if (m_masterControllerView != nullptr)
				{
					if (ImGui::Checkbox("Turn on all bulbs", &m_bTurnOnAllControllers))
					{
						request_turn_on_all_tracking_bulbs(m_bTurnOnAllControllers);
					}

					if (ImGui::Button("Save Default Profile"))
					{
						request_save_default_tracker_profile();
					}

					if (ImGui::Button("Apply Default Profile"))
					{
						request_apply_default_tracker_profile();
					}
				}
            }

            ImGui::End();
        }
        
        if (ImGui::IsMouseClicked(1) )
        {
			ImVec2 mousePos = ImGui::GetMousePos();
			ImVec2 dispSize = ImGui::GetIO().DisplaySize;
			int img_x = (static_cast<int>(mousePos.x) * m_video_buffer_state->hsvBuffer->cols) / static_cast<int>(dispSize.x);
			int img_y = (static_cast<int>(mousePos.y) * m_video_buffer_state->hsvBuffer->rows) / static_cast<int>(dispSize.y);
			cv::Vec< unsigned char, 3 > hsv_pixel = m_video_buffer_state->hsvBuffer->at<cv::Vec< unsigned char, 3 >>(cv::Point(img_x, img_y));

			TrackerColorPreset preset = getColorPreset();
			preset.hue_center = hsv_pixel[0];
			preset.saturation_center = hsv_pixel[1];
			preset.value_center = hsv_pixel[2];
			request_tracker_set_color_preset(m_masterTrackingColorType, preset);

			if (m_bAutoChangeColor) {
				setState(eMenuState::blank1);
				request_set_controller_tracking_color(m_masterControllerView, PSMTrackingColorType_Magenta);
				m_masterTrackingColorType = PSMTrackingColorType_Magenta;
				std::this_thread::sleep_for(std::chrono::milliseconds(auto_calib_sleep));
			}
			else if (m_bAutoChangeController) {
				setState(eMenuState::changeController);
			}
			else if (m_bAutoChangeTracker) {
				setState(eMenuState::changeTracker);
			}
		}

		// Keyboard shortcuts
		{
			// Hide setting windows: space bar
			if (ImGui::IsKeyReleased(32)) m_bShowWindows = !m_bShowWindows;
			// Change filter: F
			if (ImGui::IsKeyReleased(102)) {
				m_videoDisplayMode =
					static_cast<eVideoDisplayMode>(
					(m_videoDisplayMode + 1) % eVideoDisplayMode::MAX_VIDEO_DISPLAY_MODES);
			}
			// Change tracker: T
			if (ImGui::IsKeyReleased(116)) request_change_tracker(1);
			// Change controller: M
			if (ImGui::IsKeyReleased(109)) request_change_controller(1);
			// Change color: C
			if (ImGui::IsKeyReleased(99)) {
				PSMTrackingColorType new_color =
					static_cast<PSMTrackingColorType>(
					(m_masterTrackingColorType + 1) % PSMTrackingColorType_MaxColorTypes);
				request_set_controller_tracking_color(m_masterControllerView, new_color);
				m_masterTrackingColorType = new_color;
			}
		}

        // Color Control Panel
		if (m_bShowWindows)
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - k_panel_width - 10, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 280));
            ImGui::Begin("Controller Color", nullptr, window_flags);

			if (m_masterControllerView != nullptr)
			{
				if (ImGui::Button("<##Color"))
				{
					PSMTrackingColorType new_color =
						static_cast<PSMTrackingColorType>(
							(m_masterTrackingColorType + PSMTrackingColorType_MaxColorTypes - 1)
							% PSMTrackingColorType_MaxColorTypes);
					request_set_controller_tracking_color(m_masterControllerView, new_color);
				    m_masterTrackingColorType= new_color;
				}
				ImGui::SameLine();
				if (ImGui::Button(">##Color"))
				{
					PSMTrackingColorType new_color =
						static_cast<PSMTrackingColorType>(
							(m_masterTrackingColorType + 1) % PSMTrackingColorType_MaxColorTypes);
					request_set_controller_tracking_color(m_masterControllerView, new_color);
				    m_masterTrackingColorType= new_color;
				}
				ImGui::SameLine();
			}
            ImGui::Text("Tracking [C]olor: %s", k_tracking_color_names[m_masterTrackingColorType]);

            // -- Hue --
            if (ImGui::Button("-##HueCenter"))
            {
                TrackerColorPreset preset = getColorPreset();
                preset.hue_center = wrap_range(preset.hue_center - 5.f, 0.f, 180.f);
                request_tracker_set_color_preset(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            if (ImGui::Button("+##HueCenter"))
            {
                TrackerColorPreset preset = getColorPreset();
                preset.hue_center = wrap_range(preset.hue_center + 5.f, 0.f, 180.f);
                request_tracker_set_color_preset(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            ImGui::Text("Hue Angle: %f", getColorPreset().hue_center);

            if (ImGui::Button("-##HueRange"))
            {
                TrackerColorPreset preset = getColorPreset();
                preset.hue_range = clampf(preset.hue_range - 5.f, 0.f, 90.f);
                request_tracker_set_color_preset(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            if (ImGui::Button("+##HueRange"))
            {
                TrackerColorPreset preset = getColorPreset();
                preset.hue_range = clampf(preset.hue_range + 5.f, 0.f, 90.f);
                request_tracker_set_color_preset(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            ImGui::Text("Hue Range: %f", getColorPreset().hue_range);

            // -- Saturation --
            if (ImGui::Button("-##SaturationCenter"))
            {
                TrackerColorPreset preset = getColorPreset();
                preset.saturation_center = clampf(preset.saturation_center - 5.f, 0.f, 255.f);
                request_tracker_set_color_preset(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            if (ImGui::Button("+##SaturationCenter"))
            {
                TrackerColorPreset preset = getColorPreset();
                preset.saturation_center = clampf(preset.saturation_center + 5.f, 0.f, 255.f);
                request_tracker_set_color_preset(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            ImGui::Text("Saturation Center: %f", getColorPreset().saturation_center);

            if (ImGui::Button("-##SaturationRange"))
            {
                TrackerColorPreset preset = getColorPreset();
                preset.saturation_range = clampf(preset.saturation_range - 5.f, 0.f, 125.f);
                request_tracker_set_color_preset(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            if (ImGui::Button("+##SaturationRange"))
            {
                TrackerColorPreset preset = getColorPreset();
                preset.saturation_range = clampf(preset.saturation_range + 5.f, 0.f, 125.f);
                request_tracker_set_color_preset(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            ImGui::Text("Saturation Range: %f", getColorPreset().saturation_range);

            // -- Value --
            if (ImGui::Button("-##ValueCenter"))
            {
                TrackerColorPreset preset = getColorPreset();
                preset.value_center = clampf(preset.value_center - 5.f, 0.f, 255.f);
                request_tracker_set_color_preset(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            if (ImGui::Button("+##ValueCenter"))
            {
                TrackerColorPreset preset = getColorPreset();
                preset.value_center = clampf(preset.value_center + 5.f, 0.f, 255.f);
                request_tracker_set_color_preset(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            ImGui::Text("Value Center: %f", getColorPreset().value_center);

            if (ImGui::Button("-##ValueRange"))
            {
                TrackerColorPreset preset = getColorPreset();
                preset.value_range = clampf(preset.value_range - 5.f, 0.f, 125.f);
                request_tracker_set_color_preset(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            if (ImGui::Button("+##ValueRange"))
            {
                TrackerColorPreset preset = getColorPreset();
                preset.value_range = clampf(preset.value_range + 5.f, 0.f, 125.f);
                request_tracker_set_color_preset(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            ImGui::Text("Value Range: %f", getColorPreset().value_range);

			// -- Auto Calibration --
			ImGui::Text("Auto Change Setings:");
			ImGui::Checkbox("Color", &m_bAutoChangeColor);
			ImGui::SameLine();
			ImGui::Checkbox("Controller", &m_bAutoChangeController);
			ImGui::SameLine();
			ImGui::Checkbox("Tracker", &m_bAutoChangeTracker);

			// -- Change Controller --
			if (ImGui::Button("<##Controller"))
			{
				request_change_controller(-1);
			}
			ImGui::SameLine();
			if (ImGui::Button(">##Controller"))
			{
				request_change_controller(1);
			}
			ImGui::SameLine();
			ImGui::Text("PS[M]ove Controller ID: %d", m_overrideControllerId);

			// -- Change Tracker --
			if (ImGui::Button("<##Tracker"))
			{
				request_change_tracker(-1);
			}
			ImGui::SameLine();
			if (ImGui::Button(">##Tracker"))
			{
				request_change_tracker(1);
			}
			ImGui::SameLine();
			ImGui::Text("[T]racker ID: %d", tracker_index);
			
            ImGui::End();
        }
    } break;

    case eMenuState::autoConfig:
	{
		PSMTrackingColorType new_color =
			static_cast<PSMTrackingColorType>(
			(m_masterTrackingColorType + 1) % PSMTrackingColorType_MaxColorTypes);

		ImVec2 mousePos = ImGui::GetMousePos();
		ImVec2 dispSize = ImGui::GetIO().DisplaySize;
		int img_x = (static_cast<int>(mousePos.x) * m_video_buffer_state->hsvBuffer->cols) / static_cast<int>(dispSize.x);
		int img_y = (static_cast<int>(mousePos.y) * m_video_buffer_state->hsvBuffer->rows) / static_cast<int>(dispSize.y);
		cv::Vec< unsigned char, 3 > hsv_pixel = m_video_buffer_state->hsvBuffer->at<cv::Vec< unsigned char, 3 >>(cv::Point(img_x, img_y));

		TrackerColorPreset preset = getColorPreset();
		preset.hue_center = hsv_pixel[0];
		preset.saturation_center = hsv_pixel[1];
		preset.value_center = hsv_pixel[2];
		request_tracker_set_color_preset(m_masterTrackingColorType, preset);

		request_set_controller_tracking_color(m_masterControllerView, new_color);

		if (new_color == PSMTrackingColorType_Magenta) {
			if (m_bAutoChangeController) setState(eMenuState::changeController);
			else if (m_bAutoChangeTracker) setState(eMenuState::changeTracker);
			else setState(eMenuState::manualConfig);
		}
		else setState(eMenuState::blank1);

		m_masterTrackingColorType = new_color;

		std::this_thread::sleep_for(std::chrono::milliseconds(auto_calib_sleep));
	} break;

	case eMenuState::blank1:
		setState(eMenuState::blank3);
		std::this_thread::sleep_for(std::chrono::milliseconds(auto_calib_sleep));
		break;
	case eMenuState::blank2:
		setState(eMenuState::blank2);
		std::this_thread::sleep_for(std::chrono::milliseconds(auto_calib_sleep));
		break;
	case eMenuState::blank3:
		setState(eMenuState::autoConfig);
		std::this_thread::sleep_for(std::chrono::milliseconds(auto_calib_sleep));
		break;
	case eMenuState::changeController:
	{
		setState(eMenuState::manualConfig);
		request_change_controller(1);
	}
		break;
	case eMenuState::changeTracker:
		setState(eMenuState::manualConfig);
		request_change_tracker(1);
		break;
    case eMenuState::pendingTrackerStartStreamRequest:
    case eMenuState::pendingControllerStartRequest:
	case eMenuState::pendingHmdStartRequest:
    case eMenuState::waitingForStreamStartResponse:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 50));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Waiting for device stream to start...");

        ImGui::End();
    } break;

    case eMenuState::failedTrackerStartStreamRequest:
	case eMenuState::failedHmdStartRequest:
    case eMenuState::failedControllerStartRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        if (m_menuState == eMenuState::failedTrackerStartStreamRequest)
        {
            ImGui::Text("Failed to start tracker stream!");
        }
		else if (m_menuState == eMenuState::failedHmdStartRequest)
		{
			ImGui::Text("Failed to start controller stream!");
		}
        else
        {
            ImGui::Text("Failed to start controller stream!");
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

    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ColorCalibration::setState(
    AppStage_ColorCalibration::eMenuState newState)
{
    if (newState != m_menuState)
    {
        m_menuState = newState;
    }
}

void AppStage_ColorCalibration::request_start_controller_streams()
{
	for (PSMController *controllerView : m_controllerViews)
	{
		++m_pendingControllerStartCount;

		PSMRequestID request_id;
		PSM_StartControllerDataStreamAsync(controllerView->ControllerID, PSMStreamFlags_defaultStreamOptions, &request_id);
		PSM_RegisterCallback(request_id, &AppStage_ColorCalibration::handle_start_controller_response, this);
	}

    // Start receiving data from the controller
    setState(AppStage_ColorCalibration::pendingControllerStartRequest);
}

void AppStage_ColorCalibration::handle_start_controller_response(
    const PSMResponseMessage *response_message,
    void *userdata)
{
    AppStage_ColorCalibration *thisPtr = static_cast<AppStage_ColorCalibration *>(userdata);

    const PSMResult ResultCode = response_message->result_code;
//    const ClientPSMoveAPI::t_request_id request_id = response_message->request_id;

    switch (ResultCode)
    {
    case PSMResult_Success:
        {
			--thisPtr->m_pendingControllerStartCount;

			if (thisPtr->m_pendingControllerStartCount <= 0)
			{
				thisPtr->m_areAllControllerStreamsActive= true;
				thisPtr->setState(AppStage_ColorCalibration::waitingForStreamStartResponse);
			}
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
	case PSMResult_Timeout:
        {
            thisPtr->setState(AppStage_ColorCalibration::failedControllerStartRequest);
        } break;
    }
}

void AppStage_ColorCalibration::request_set_controller_tracking_color(
	PSMController *controllerView,
    PSMTrackingColorType tracking_color)
{
    unsigned char r, g, b;

    switch (tracking_color)
    {
    case PSMoveProtocol::Magenta:
        r = 0xFF; g = 0x00; b = 0xFF;
        break;
    case PSMoveProtocol::Cyan:
        r = 0x00; g = 0xFF; b = 0xFF;
        break;
    case PSMoveProtocol::Yellow:
        r = 0xFF; g = 0xFF; b = 0x00;
        break;
    case PSMoveProtocol::Red:
        r = 0xFF; g = 0x00; b = 0x00;
        break;
    case PSMoveProtocol::Green:
        r = 0x00; g = 0xFF; b = 0x00;
        break;
    case PSMoveProtocol::Blue:
        r = 0x00; g = 0x00; b = 0xFF;
        break;
    default:
        assert(0 && "unreachable");
    }

	PSM_SetControllerLEDOverrideColor(controllerView->ControllerID, r, g, b);
}

void AppStage_ColorCalibration::request_start_hmd_stream()
{
	// Start receiving data from the controller
	setState(AppStage_ColorCalibration::pendingHmdStartRequest);

	PSMRequestID requestId;
	PSM_StartHmdDataStreamAsync(m_hmdView->HmdID, PSMStreamFlags_includePositionData, &requestId); // turns on tracking lights
	PSM_RegisterCallback(requestId, AppStage_ColorCalibration::handle_start_hmd_response, this);
}

void AppStage_ColorCalibration::handle_start_hmd_response(
	const PSMResponseMessage *response_message,
	void *userdata)
{
	AppStage_ColorCalibration *thisPtr = static_cast<AppStage_ColorCalibration *>(userdata);
	const PSMResult ResultCode = response_message->result_code;

	switch (ResultCode)
	{
	case PSMResult_Success:
		{
			thisPtr->m_isHmdStreamActive = true;
			thisPtr->setState(AppStage_ColorCalibration::waitingForStreamStartResponse);
		} break;

	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
		{
			thisPtr->setState(AppStage_ColorCalibration::failedControllerStartRequest);
		} break;
	}
}

void AppStage_ColorCalibration::request_tracker_start_stream()
{
    if (m_menuState != AppStage_ColorCalibration::pendingTrackerStartStreamRequest)
    {
        setState(AppStage_ColorCalibration::pendingTrackerStartStreamRequest);

        // Tell the psmove service that we want to start streaming data from the tracker
		PSMRequestID requestID;
		PSM_StartTrackerDataStreamAsync(m_trackerView->tracker_info.tracker_id, &requestID);
		PSM_RegisterCallback(requestID, AppStage_ColorCalibration::handle_tracker_start_stream_response, this);
    }
}

void AppStage_ColorCalibration::handle_tracker_start_stream_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    AppStage_ColorCalibration *thisPtr = static_cast<AppStage_ColorCalibration *>(userdata);

    switch (response->result_code)
    {
    case PSMResult_Success:
        {
            PSMTracker *trackerView = thisPtr->m_trackerView;

            // Open the shared memory that the video stream is being written to
            if (PSM_OpenTrackerVideoStream(trackerView->tracker_info.tracker_id) == PSMResult_Success)
            {
                thisPtr->allocate_video_buffers();
            }

            // Now that the tracker stream is started, start the controller stream
			if (thisPtr->m_hmdView != nullptr)
			{
				thisPtr->request_start_hmd_stream();
			}
			else
			{
				thisPtr->request_start_controller_streams();
			}
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
	case PSMResult_Timeout:
        {
            thisPtr->setState(AppStage_ColorCalibration::failedTrackerStartStreamRequest);
        } break;
    }
}

void AppStage_ColorCalibration::allocate_video_buffers()
{
    m_video_buffer_state = new VideoBufferState(m_trackerView);
}

void AppStage_ColorCalibration::release_video_buffers()
{
    delete m_video_buffer_state;
    m_video_buffer_state = nullptr;
}

void AppStage_ColorCalibration::request_tracker_set_frame_rate(double value)
{
	// Tell the psmove service that we want to change frame rate.
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_FRAMERATE);
	request->mutable_request_set_tracker_frame_rate()->set_tracker_id(m_trackerView->tracker_info.tracker_id);
	request->mutable_request_set_tracker_frame_rate()->set_value(static_cast<float>(value));
	request->mutable_request_set_tracker_frame_rate()->set_save_setting(true);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_RegisterCallback(request_id, AppStage_ColorCalibration::handle_tracker_set_frame_rate_response, this);
}

void AppStage_ColorCalibration::handle_tracker_set_frame_rate_response(
	const PSMResponseMessage *response,
	void *userdata)
{
	PSMResult ResultCode = response->result_code;
	PSMResponseHandle response_handle = response->opaque_response_handle;
	AppStage_ColorCalibration *thisPtr = static_cast<AppStage_ColorCalibration *>(userdata);

	switch (ResultCode)
	{
	case PSMResult_Success:
		{
			const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);
			thisPtr->m_trackerFramerate = response->result_set_tracker_frame_rate().new_frame_rate();
		} break;
	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
		{
			//###HipsterSloth $TODO - Replace with C_API style log
			//CLIENT_LOG_INFO("AppStage_ColorCalibration") << "Failed to set the tracker frame rate!";
		} break;
	}
}

void AppStage_ColorCalibration::request_tracker_set_exposure(double value)
{
    // Tell the psmove service that we want to change exposure.
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_EXPOSURE);
    request->mutable_request_set_tracker_exposure()->set_tracker_id(m_trackerView->tracker_info.tracker_id);
    request->mutable_request_set_tracker_exposure()->set_value(static_cast<float>(value));
    request->mutable_request_set_tracker_exposure()->set_save_setting(true);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_RegisterCallback(request_id, AppStage_ColorCalibration::handle_tracker_set_exposure_response, this);
}

void AppStage_ColorCalibration::handle_tracker_set_exposure_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    PSMResult ResultCode = response->result_code;
    PSMResponseHandle response_handle = response->opaque_response_handle;
    AppStage_ColorCalibration *thisPtr = static_cast<AppStage_ColorCalibration *>(userdata);

    switch (ResultCode)
    {
    case PSMResult_Success:
        {
            const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);
            thisPtr->m_trackerExposure = response->result_set_tracker_exposure().new_exposure();
        } break;
    case PSMResult_Error:
    case PSMResult_Canceled:
	case PSMResult_Timeout:
        {
			//###HipsterSloth $TODO - Replace with C_API style log
            //CLIENT_LOG_INFO("AppStage_ColorCalibration") << "Failed to set the tracker exposure!";
        } break;
    }
}

void AppStage_ColorCalibration::request_tracker_set_gain(double value)
{
    // Tell the psmove service that we want to change gain.
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_GAIN);
    request->mutable_request_set_tracker_gain()->set_tracker_id(m_trackerView->tracker_info.tracker_id);
    request->mutable_request_set_tracker_gain()->set_value(static_cast<float>(value));
    request->mutable_request_set_tracker_gain()->set_save_setting(true);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_RegisterCallback(request_id, AppStage_ColorCalibration::handle_tracker_set_gain_response, this);
}

void AppStage_ColorCalibration::handle_tracker_set_gain_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    PSMResult ResultCode = response->result_code;
    PSMResponseHandle response_handle = response->opaque_response_handle;
    AppStage_ColorCalibration *thisPtr = static_cast<AppStage_ColorCalibration *>(userdata);

    switch (ResultCode)
    {
    case PSMResult_Success:
        {
            const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);
            thisPtr->m_trackerGain = response->result_set_tracker_gain().new_gain();
        } break;
    case PSMResult_Error:
    case PSMResult_Canceled:
	case PSMResult_Timeout:
        {
			//###HipsterSloth $TODO - Replace with C_API style log
            //CLIENT_LOG_INFO("AppStage_ColorCalibration") << "Failed to set the tracker gain!";
        } break;
    }
}

void AppStage_ColorCalibration::request_tracker_set_option(
    TrackerOption &option, 
    int new_option_index)
{
    // Tell the psmove service that we want to change gain.
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_OPTION);
    request->mutable_request_set_tracker_option()->set_tracker_id(m_trackerView->tracker_info.tracker_id);
    request->mutable_request_set_tracker_option()->set_option_name(option.option_name);
    request->mutable_request_set_tracker_option()->set_option_index(new_option_index);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_RegisterCallback(request_id, AppStage_ColorCalibration::handle_tracker_set_option_response, this);
}

void AppStage_ColorCalibration::handle_tracker_set_option_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    PSMResult ResultCode = response->result_code;
    PSMResponseHandle response_handle = response->opaque_response_handle;
    AppStage_ColorCalibration *thisPtr = static_cast<AppStage_ColorCalibration *>(userdata);

    switch (ResultCode)
    {
    case PSMResult_Success:
        {
            const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);
            int result_option_index = response->result_set_tracker_option().new_option_index();
            std::string option_name = response->result_set_tracker_option().option_name();

            // Find the option with the matching option_name
            auto it = std::find_if(
                thisPtr->m_trackerOptions.begin(),
                thisPtr->m_trackerOptions.end(),
                [&option_name](const TrackerOption &option) { 
                    return option.option_name == option_name; 
                });

            if (it != thisPtr->m_trackerOptions.end())
            {
                it->option_index = result_option_index;
            }
        } break;
    case PSMResult_Error:
    case PSMResult_Canceled:
	case PSMResult_Timeout:
        {
			//###HipsterSloth $TODO - Replace with C_API style log
            //CLIENT_LOG_INFO("AppStage_ColorCalibration") << "Failed to set the tracker gain!";
        } break;
    }
}

void AppStage_ColorCalibration::request_tracker_set_color_preset(
    PSMTrackingColorType color_type,
    TrackerColorPreset &color_preset)
{
    // Tell the psmove service that we want to change gain.
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_COLOR_PRESET);
    request->mutable_request_set_tracker_color_preset()->set_tracker_id(m_trackerView->tracker_info.tracker_id);

	if (m_hmdView != nullptr)
	{
		request->mutable_request_set_tracker_color_preset()->set_device_id(m_overrideHmdId);
		request->mutable_request_set_tracker_color_preset()->set_device_category(
			PSMoveProtocol::Request_RequestSetTrackerColorPreset_DeviceCategory_HMD);
	}
	else
	{
		request->mutable_request_set_tracker_color_preset()->set_device_id(m_overrideControllerId);
		request->mutable_request_set_tracker_color_preset()->set_device_category(
			PSMoveProtocol::Request_RequestSetTrackerColorPreset_DeviceCategory_CONTROLLER);
	}

    {
        PSMoveProtocol::TrackingColorPreset* tracking_color_preset =
            request->mutable_request_set_tracker_color_preset()->mutable_color_preset();

        tracking_color_preset->set_color_type(static_cast<PSMoveProtocol::TrackingColorType>(color_type));
        tracking_color_preset->set_hue_center(color_preset.hue_center);
        tracking_color_preset->set_hue_range(color_preset.hue_range);
        tracking_color_preset->set_saturation_center(color_preset.saturation_center);
        tracking_color_preset->set_saturation_range(color_preset.saturation_range);
        tracking_color_preset->set_value_center(color_preset.value_center);
        tracking_color_preset->set_value_range(color_preset.value_range);
    }

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_RegisterCallback(request_id, AppStage_ColorCalibration::handle_tracker_set_color_preset_response, this);
}

void AppStage_ColorCalibration::handle_tracker_set_color_preset_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    switch (response->result_code)
    {
    case PSMResult_Success:
        {
            const PSMResponseHandle response_handle = response->opaque_response_handle;
            const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);
            const PSMoveProtocol::TrackingColorPreset &srcPreset= response->result_set_tracker_color_preset().new_color_preset();
            const PSMTrackingColorType color_type = static_cast<PSMTrackingColorType>(srcPreset.color_type());

            AppStage_ColorCalibration *thisPtr = static_cast<AppStage_ColorCalibration *>(userdata);
            AppStage_ColorCalibration::TrackerColorPreset &targetPreset= thisPtr->m_colorPresets[color_type];

            targetPreset.hue_center= srcPreset.hue_center();
            targetPreset.hue_range= srcPreset.hue_range();
            targetPreset.saturation_center= srcPreset.saturation_center();
            targetPreset.saturation_range= srcPreset.saturation_range();
            targetPreset.value_center= srcPreset.value_center();
            targetPreset.value_range= srcPreset.value_range();
        } break;
    case PSMResult_Error:
    case PSMResult_Canceled:
	case PSMResult_Timeout:
        {
			//###HipsterSloth $TODO - Replace with C_API style log
            //CLIENT_LOG_INFO("AppStage_ColorCalibration") << "Failed to set the tracker presets!";
        } break;
    }
}

void AppStage_ColorCalibration::request_tracker_get_settings()
{
    // Tell the psmove service that we want to change exposure.
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_GET_TRACKER_SETTINGS);
    request->mutable_request_get_tracker_settings()->set_tracker_id(m_trackerView->tracker_info.tracker_id);

	if (m_overrideHmdId != -1)
	{
		request->mutable_request_get_tracker_settings()->set_device_id(m_overrideHmdId);
		request->mutable_request_get_tracker_settings()->set_device_category(PSMoveProtocol::Request_RequestGetTrackerSettings_DeviceCategory_HMD);
	}
	else
	{
		request->mutable_request_get_tracker_settings()->set_device_id(m_overrideControllerId);
		request->mutable_request_get_tracker_settings()->set_device_category(PSMoveProtocol::Request_RequestGetTrackerSettings_DeviceCategory_CONTROLLER);
	}

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_RegisterCallback(request_id, AppStage_ColorCalibration::handle_tracker_get_settings_response, this);
}

void AppStage_ColorCalibration::handle_tracker_get_settings_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    PSMResult ResultCode = response->result_code;
    PSMResponseHandle response_handle = response->opaque_response_handle;
    AppStage_ColorCalibration *thisPtr = static_cast<AppStage_ColorCalibration *>(userdata);

    switch (ResultCode)
    {
    case PSMResult_Success:
        {
            const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);
			thisPtr->m_trackerFramerate = response->result_tracker_settings().frame_rate();
            thisPtr->m_trackerExposure = response->result_tracker_settings().exposure();
            thisPtr->m_trackerGain = response->result_tracker_settings().gain();

            thisPtr->m_trackerOptions.clear();
            for (auto it = response->result_tracker_settings().option_sets().begin();
                it != response->result_tracker_settings().option_sets().end();
                ++it)
            {
                const PSMoveProtocol::OptionSet &srcOption = *it;
                AppStage_ColorCalibration::TrackerOption destOption;

                destOption.option_index = srcOption.option_index();
                destOption.option_name = srcOption.option_name();
                
                // Copy the option strings into the destOption
                std::for_each(
                    srcOption.option_strings().begin(), 
                    srcOption.option_strings().end(), 
                    [&destOption](const std::string &option_string) { 
                        destOption.option_strings.push_back(option_string); 
                    });

                thisPtr->m_trackerOptions.push_back(destOption);
            }

            for (auto it = response->result_tracker_settings().color_presets().begin();
                it != response->result_tracker_settings().color_presets().end();
                ++it)
            {
                const PSMoveProtocol::TrackingColorPreset &srcPreset = *it;
                const PSMTrackingColorType client_color= 
                    static_cast<PSMTrackingColorType>(srcPreset.color_type());

                AppStage_ColorCalibration::TrackerColorPreset &destPreset = thisPtr->m_colorPresets[client_color];
                destPreset.hue_center= srcPreset.hue_center();
                destPreset.hue_range= srcPreset.hue_range();
                destPreset.saturation_center = srcPreset.saturation_center();
                destPreset.saturation_range = srcPreset.saturation_range();
                destPreset.value_center = srcPreset.value_center();
                destPreset.value_range = srcPreset.value_range();
            }
        } break;
    case PSMResult_Error:
    case PSMResult_Canceled:
	case PSMResult_Timeout:
        {
			//###HipsterSloth $TODO - Replace with C_API style log
            //CLIENT_LOG_INFO("AppStage_ColorCalibration") << "Failed to get the tracker settings!";
        } break;
    }
}

void AppStage_ColorCalibration::request_save_default_tracker_profile()
{
    // Tell the psmove service that we want to save the current trackers profile.
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SAVE_TRACKER_PROFILE);
    request->mutable_request_save_tracker_profile()->set_tracker_id(m_trackerView->tracker_info.tracker_id);
	request->mutable_request_save_tracker_profile()->set_controller_id(m_overrideControllerId);

	PSM_SendOpaqueRequest(&request, nullptr);
}

void AppStage_ColorCalibration::request_apply_default_tracker_profile()
{
    // Tell the psmove service that we want to apply the saved default profile to the current tracker.
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_APPLY_TRACKER_PROFILE);
    request->mutable_request_save_tracker_profile()->set_tracker_id(m_trackerView->tracker_info.tracker_id);
	request->mutable_request_save_tracker_profile()->set_controller_id(m_overrideControllerId);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_RegisterCallback(request_id, AppStage_ColorCalibration::handle_tracker_get_settings_response, this);
}

void AppStage_ColorCalibration::release_devices()
{
    //###HipsterSloth $REVIEW Do we care about canceling in-flight requests?

    release_video_buffers();

    for (PSMController *controllerView : m_controllerViews)
    {
		PSM_SetControllerLEDOverrideColor(controllerView->ControllerID, 0, 0, 0);

        if (m_areAllControllerStreamsActive)
        {
			PSM_StopControllerDataStreamAsync(controllerView->ControllerID, nullptr);
        }

        PSM_FreeControllerListener(controllerView->ControllerID);
    }
	m_controllerViews.clear();

    m_masterControllerView = nullptr;
    m_areAllControllerStreamsActive= false;
    m_lastMasterControllerSeqNum= -1;


	if (m_hmdView != nullptr)
	{
		if (m_isHmdStreamActive)
		{
			PSM_StopHmdDataStreamAsync(m_hmdView->HmdID, nullptr);
		}

		PSM_FreeHmdListener(m_hmdView->HmdID);
		m_hmdView = nullptr;
		m_isHmdStreamActive = false;
		m_lastHmdSeqNum = -1;
	}

    if (m_trackerView != nullptr)
    {
		PSM_CloseTrackerVideoStream(m_trackerView->tracker_info.tracker_id);
        PSM_StopTrackerDataStreamAsync(m_trackerView->tracker_info.tracker_id, nullptr);
		PSM_FreeTrackerListener(m_trackerView->tracker_info.tracker_id);
        m_trackerView = nullptr;
    }
}

void AppStage_ColorCalibration::request_exit_to_app_stage(const char *app_stage_name)
{
    release_devices();

    m_app->setAppStage(app_stage_name);
}

void AppStage_ColorCalibration::request_turn_on_all_tracking_bulbs(bool bEnabled)
{
	assert(m_controllerViews.size() == m_controllerTrackingColorTypes.size());
	for (int list_index= 0; list_index < m_controllerViews.size(); ++list_index)
	{
		PSMController *controllerView= m_controllerViews[list_index];

		if (controllerView == m_masterControllerView)
			continue;

		if (bEnabled)
		{
			request_set_controller_tracking_color(controllerView, m_controllerTrackingColorTypes[list_index]);
		}
		else
		{
			PSM_SetControllerLEDOverrideColor(controllerView->ControllerID, 0, 0, 0);
		}
	}
}

void AppStage_ColorCalibration::request_change_controller(int step)
{
	assert(m_controllerViews.size() == m_controllerTrackingColorTypes.size());
	//for (int list_index = 0; list_index < m_controllerViews.size(); ++list_index)
	{
		PSMController *controllerView = m_controllerViews[m_overrideControllerId];

		if (controllerView == m_masterControllerView) {
			PSM_SetControllerLEDOverrideColor(m_masterControllerView->ControllerID, 0, 0, 0);
			if (m_overrideControllerId + step < static_cast<int>(m_controllerViews.size()) && m_overrideControllerId + step >= 0) {
				m_overrideControllerId = m_overrideControllerId + step;
				m_masterControllerView = m_controllerViews[m_overrideControllerId];
				request_set_controller_tracking_color(m_masterControllerView, m_masterTrackingColorType);
				//setState(eMenuState::manualConfig);
			}
			else if (step > 0) {
				m_overrideControllerId = 0;
				m_masterControllerView = m_controllerViews[0];
				request_set_controller_tracking_color(m_masterControllerView, m_masterTrackingColorType);
				if (m_bAutoChangeTracker) setState(eMenuState::changeTracker);
				//else setState(eMenuState::manualConfig);
			}
			else {
				m_overrideControllerId = static_cast<int>(m_controllerViews.size()) -1;
				m_masterControllerView = m_controllerViews[m_overrideControllerId];
				request_set_controller_tracking_color(m_masterControllerView, m_masterTrackingColorType);
				//if (m_bAutoChangeTracker) setState(eMenuState::changeTracker);
				//else 
				//	setState(eMenuState::manualConfig);
			}
			//break;
		}
	}
}

void AppStage_ColorCalibration::request_change_tracker(int step)
{
	m_app->getAppStage<AppStage_ColorCalibration>()->
	set_autoConfig(m_bAutoChangeColor, m_bAutoChangeController, m_bAutoChangeTracker);
	//int TrackerId = m_trackerView->tracker_info.tracker_id;
	if (tracker_index + step < tracker_count && tracker_index + step >= 0)
	{
		m_app->getAppStage<AppStage_TrackerSettings>()->set_selectedTrackerIndex(tracker_index + step);
		request_exit_to_app_stage(AppStage_ColorCalibration::APP_STAGE_NAME);
	}
	else if (step > 0)
	{
		m_app->getAppStage<AppStage_TrackerSettings>()->set_selectedTrackerIndex(0);
		request_exit_to_app_stage(AppStage_ColorCalibration::APP_STAGE_NAME);
	}
	else
	{
		m_app->getAppStage<AppStage_TrackerSettings>()->set_selectedTrackerIndex(tracker_count -1);
		request_exit_to_app_stage(AppStage_ColorCalibration::APP_STAGE_NAME);
	}
}
/**
 * Large chunks of this calibration tool came from tracker_camera_calibration.c in psmoveapi.
 * Reproducing the license from that file here:
 *
 * PS Move API - An interface for the PS Move Motion Controller
 * Copyright (c) 2012 Thomas Perl <m@thp.io>
 * Copyright (c) 2012 Benjamin Venditt <benjamin.venditti@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **/

//-- includes -----
#include "AppStage_DistortionCalibration.h"
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

#include <imgui.h>

#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

//-- statics ----
const char *AppStage_DistortionCalibration::APP_STAGE_NAME = "DistortionCalibration";

//-- constants -----
static const char *k_video_display_mode_names[] = {
    "BGR",
    "Grayscale",
    "Undistorted"
};

#define PATTERN_W 9 // Internal corners
#define PATTERN_H 6
#define N_BOARDS 10

//-- private definitions -----
class OpenCVBufferState
{
public:
    OpenCVBufferState(int width, int height)
        : frameWidth(width)
        , frameHeight(height)
        , bgrSourceBuffer(nullptr)
        , gsBuffer(nullptr)
        , bgrUndistortBuffer(nullptr)
    {
        bgrSourceBuffer = new cv::Mat(height, width, CV_8UC3);
        gsBuffer = new cv::Mat(height, width, CV_8UC1);
        gsBGRBuffer = new cv::Mat(height, width, CV_8UC3);
        bgrUndistortBuffer = new cv::Mat(height, width, CV_8UC3);

        image_points = new cv::Mat(N_BOARDS * PATTERN_W * PATTERN_H, 2, CV_32FC1);
        object_points = new cv::Mat(N_BOARDS * PATTERN_W * PATTERN_H, 3, CV_32FC1);
        point_counts = new cv::Mat(N_BOARDS, 1, CV_32SC1);
        intrinsic_matrix = new cv::Mat(3, 3, CV_32FC1);
        distortion_coeffs = new cv::Mat(5, 1, CV_32FC1);
    }

    virtual ~OpenCVBufferState()
    {
        delete bgrSourceBuffer;
        delete gsBuffer;
        delete gsBGRBuffer;
        delete bgrUndistortBuffer;

        delete image_points;
        delete object_points;
        delete point_counts;
        delete intrinsic_matrix;
        delete distortion_coeffs;
    }

    void applyVideoFrame(const unsigned char *video_buffer)
    {
        const cv::Mat videoBufferMat(frameHeight, frameWidth, CV_8UC3, const_cast<unsigned char *>(video_buffer));

        // Copy and Flip image about the x-axis
        videoBufferMat.copyTo(*bgrSourceBuffer);

        // Convert the video buffer to a grayscale image
        cv::cvtColor(*bgrSourceBuffer, *gsBuffer, cv::COLOR_BGR2GRAY);
        cv::cvtColor(*gsBuffer, *gsBGRBuffer, cv::COLOR_GRAY2BGR);
    }

    int frameWidth;
    int frameHeight;

    // Video frame buffers
    cv::Mat *bgrSourceBuffer;
    cv::Mat *gsBuffer;
    cv::Mat *gsBGRBuffer;
    cv::Mat *bgrUndistortBuffer;

    // Chess board computed state
    cv::Mat *image_points;
    cv::Mat *object_points;
    cv::Mat *point_counts;
    cv::Mat *intrinsic_matrix;
    cv::Mat *distortion_coeffs;
};

//-- public methods -----
AppStage_DistortionCalibration::AppStage_DistortionCalibration(App *app)
    : AppStage(app)
    , m_menuState(AppStage_DistortionCalibration::inactive)
    , m_bStreamIsActive(false)
    , m_tracker_view(nullptr)
    , m_video_texture(nullptr)
    , m_opencv_state(nullptr)
    , m_videoDisplayMode(AppStage_DistortionCalibration::eVideoDisplayMode::mode_bgr)
{ }

void AppStage_DistortionCalibration::enter()
{
    const AppStage_TrackerSettings *trackerSettings =
        m_app->getAppStage<AppStage_TrackerSettings>();
    const ClientTrackerInfo *trackerInfo = trackerSettings->getSelectedTrackerInfo();
    assert(trackerInfo->tracker_id != -1);

    m_app->setCameraType(_cameraFixed);

    assert(m_tracker_view == nullptr);
    m_tracker_view= ClientPSMoveAPI::allocate_tracker_view(*trackerInfo);

    assert(!m_bStreamIsActive);
    request_tracker_start_stream();
}

void AppStage_DistortionCalibration::exit()
{
    m_menuState = AppStage_DistortionCalibration::inactive;

    if (m_opencv_state != nullptr)
    {
        delete m_opencv_state;
        m_opencv_state= nullptr;
    }

    ClientPSMoveAPI::free_tracker_view(m_tracker_view);
    m_tracker_view = nullptr;
}

void AppStage_DistortionCalibration::update()
{
    // Try and read the next video frame from shared memory
    if (m_video_texture != nullptr)
    {
        if (m_tracker_view->pollVideoStream())
        {
            const unsigned char *video_frame_buffer= m_tracker_view->getVideoFrameBuffer();

            m_opencv_state->applyVideoFrame(video_frame_buffer);

            switch (m_videoDisplayMode)
            {
            case AppStage_DistortionCalibration::mode_bgr:
                m_video_texture->copyBufferIntoTexture(m_opencv_state->bgrSourceBuffer->data);
                break;
            case AppStage_DistortionCalibration::mode_grayscale:
                m_video_texture->copyBufferIntoTexture(m_opencv_state->gsBGRBuffer->data);
                break;
            case AppStage_DistortionCalibration::mode_undistored:
                m_video_texture->copyBufferIntoTexture(m_opencv_state->bgrUndistortBuffer->data);
                break;
            default:
                assert(0 && "unreachable");
                break;
            }            
        }
    }
}

void AppStage_DistortionCalibration::render()
{
    // If there is a video frame available to render, show it
    if (m_video_texture != nullptr)
    {
        unsigned int texture_id = m_video_texture->texture_id;

        if (texture_id != 0)
        {
            drawFullscreenTexture(texture_id);
        }
    }
}

void AppStage_DistortionCalibration::renderUI()
{
    const float k_panel_width = 300.f;
    const char *k_window_title = "Tracker Test";
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
        ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 200));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        if (m_opencv_state != nullptr)
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
            ImGui::Text("Video Filter Mode: %s", k_video_display_mode_names[m_videoDisplayMode]);
        }

        if (ImGui::Button("Return to Tracker Settings"))
        {
            if (m_bStreamIsActive)
            {
                const AppStage_TrackerSettings *trackerSettings =
                    m_app->getAppStage<AppStage_TrackerSettings>();
                const ClientTrackerInfo *trackerInfo = trackerSettings->getSelectedTrackerInfo();

                request_tracker_stop_stream();
            }
            else
            {
                m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }
        }              

        ImGui::End();
    } break;

    case eTrackerMenuState::pendingTrackerStartStreamRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 50));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Waiting for tracker stream to start...");

        ImGui::End();
    } break;

    case eTrackerMenuState::failedTrackerStartStreamRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Failed to start tracker stream!");

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

    case eTrackerMenuState::pendingTrackerStopStreamRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 50));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Waiting for tracker stream to stop...");

        ImGui::End();
    } break;

    case eTrackerMenuState::failedTrackerStopStreamRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Failed to stop tracker stream!");

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

    default:
        assert(0 && "unreachable");
    }
}

void AppStage_DistortionCalibration::request_tracker_start_stream()
{
    if (m_menuState != AppStage_DistortionCalibration::pendingTrackerStartStreamRequest)
    {
        m_menuState = AppStage_DistortionCalibration::pendingTrackerStartStreamRequest;

        // Tell the psmove service that we want to start streaming data from the tracker
        ClientPSMoveAPI::register_callback(
            ClientPSMoveAPI::start_tracker_data_stream(m_tracker_view),
            AppStage_DistortionCalibration::handle_tracker_start_stream_response, this);
    }
}

void AppStage_DistortionCalibration::handle_tracker_start_stream_response(
    const ClientPSMoveAPI::ResponseMessage *response,
    void *userdata)
{
    AppStage_DistortionCalibration *thisPtr = static_cast<AppStage_DistortionCalibration *>(userdata);

    switch (response->result_code)
    {
    case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
        {
            ClientTrackerView *trackerView= thisPtr->m_tracker_view;

            thisPtr->m_bStreamIsActive = true;
            thisPtr->m_menuState = AppStage_DistortionCalibration::idle;

            // Open the shared memory that the vidoe stream is being written to
            if (trackerView->openVideoStream())
            {
                int width= trackerView->getVideoFrameWidth();
                int height= trackerView->getVideoFrameHeight();

                // Create a texture to render the video frame to
                thisPtr->m_video_texture = new TextureAsset();
                thisPtr->m_video_texture->init(
                    width,
                    height,
                    GL_RGB, // texture format
                    GL_BGR, // buffer format
                    nullptr);

                // Allocate an opencv buffer 
                thisPtr->m_opencv_state = new OpenCVBufferState(width, height);
            }
        } break;

    case ClientPSMoveAPI::_clientPSMoveResultCode_error:
    case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
        {
            thisPtr->m_menuState = AppStage_DistortionCalibration::failedTrackerStartStreamRequest;
        } break;
    }
}

void AppStage_DistortionCalibration::request_tracker_stop_stream()
{
    if (m_bStreamIsActive && m_menuState != AppStage_DistortionCalibration::pendingTrackerStopStreamRequest)
    {
        m_menuState = AppStage_DistortionCalibration::pendingTrackerStopStreamRequest;

        // Tell the psmove service that we want to stop streaming data from the tracker        
        ClientPSMoveAPI::register_callback(
            ClientPSMoveAPI::stop_tracker_data_stream(m_tracker_view), 
            AppStage_DistortionCalibration::handle_tracker_stop_stream_response, this);
    }
}

void AppStage_DistortionCalibration::handle_tracker_stop_stream_response(
    const ClientPSMoveAPI::ResponseMessage *response,
    void *userdata)
{
    AppStage_DistortionCalibration *thisPtr = static_cast<AppStage_DistortionCalibration *>(userdata);

    // In either case consider the stream as now inactive
    thisPtr->m_bStreamIsActive = false;

    switch (response->result_code)
    {
    case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
        {
            thisPtr->m_menuState = AppStage_DistortionCalibration::inactive;

            // Close the shared memory buffer
            thisPtr->m_tracker_view->closeVideoStream();

            // Free the texture we were rendering to
            if (thisPtr->m_video_texture != nullptr)
            {
                delete thisPtr->m_video_texture;
                thisPtr->m_video_texture = nullptr;
            }

            // After closing the stream, we should go back to the tracker settings
            thisPtr->m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
        } break;

    case ClientPSMoveAPI::_clientPSMoveResultCode_error:
    case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
        {
            thisPtr->m_menuState = AppStage_DistortionCalibration::failedTrackerStopStreamRequest;
        } break;
    }
}
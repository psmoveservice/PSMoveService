//-- inludes -----
#include "AppStage_TestTracker.h"
#include "AppStage_TrackerSettings.h"
#include "AppStage_MainMenu.h"
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
#include <sstream>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <memory>

#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

//-- statics ----
const char *AppStage_TestTracker::APP_STAGE_NAME = "TestTracker";

//-- constants -----

//-- private methods -----
class SharedVideoFrameReadOnlyAccessor
{
public:
    SharedVideoFrameReadOnlyAccessor()
        : m_shared_memory_object(nullptr)
        , m_region(nullptr)
        , m_bgr_frame_buffer(nullptr)
        , m_frame_width(0)
        , m_frame_height(0)
        , m_frame_stride(0)
        , m_last_frame_index(0)
        , m_video_frame_texture_id(0)
    {}

    ~SharedVideoFrameReadOnlyAccessor()
    {
        dispose();
    }

    bool initialize(const char *shared_memory_name)
    {
        bool bSuccess = false;

        try
        {
            CLIENT_LOG_INFO("SharedMemory::initialize()") << "Opening shared memory: " << shared_memory_name;

            // Remember the name of the shared memory
            m_shared_memory_name = shared_memory_name;

            // Create the shared memory object
            m_shared_memory_object =
                new boost::interprocess::shared_memory_object(
                boost::interprocess::open_only,
                shared_memory_name,
                boost::interprocess::read_write);

            // Map all of the shared memory for read/write access
            m_region = new boost::interprocess::mapped_region(*m_shared_memory_object, boost::interprocess::read_write);

            bSuccess = true;
        }
        catch (boost::interprocess::interprocess_exception &ex)
        {
            dispose();
            CLIENT_LOG_ERROR("SharedMemory::initialize()") << "Failed to allocated shared memory: " << m_shared_memory_name
                << ", reason: " << ex.what();
        }
        catch (std::exception &ex)
        {
            dispose();
            CLIENT_LOG_ERROR("SharedMemory::initialize()") << "Failed to allocated shared memory: " << m_shared_memory_name
                << ", reason: " << ex.what();
        }

        return bSuccess;
    }

    void dispose()
    {
        if (m_region != nullptr)
        {
            delete m_region;
            m_region = nullptr;
        }

        if (m_shared_memory_object != nullptr)
        {
            delete m_shared_memory_object;
            m_shared_memory_object = nullptr;
        }

        if (m_bgr_frame_buffer != nullptr)
        {
            delete[] m_bgr_frame_buffer;
            m_bgr_frame_buffer = nullptr;
        }
    }

    void readVideoFrame()
    {
        SharedVideoFrameHeader *sharedFrameState = getFrameHeader();
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(sharedFrameState->mutex);

        // Make sure the target buffer is big enough to read the video frame into
        size_t buffer_size =
            SharedVideoFrameHeader::computeVideoBufferSize(sharedFrameState->stride, sharedFrameState->height);

        // Make sure the shared memory is the size we expect
        size_t total_shared_mem_size =
            SharedVideoFrameHeader::computeTotalSize(sharedFrameState->stride, sharedFrameState->height);
        assert(m_region->get_size() >= total_shared_mem_size);

        // Re-allocate the buffer if any of the video properties changed
        if (m_frame_width != sharedFrameState->width ||
            m_frame_height != sharedFrameState->height ||
            m_frame_stride != sharedFrameState->stride)
        {
            free_video_buffer();

            m_frame_width = sharedFrameState->width;
            m_frame_height = sharedFrameState->height;
            m_frame_stride = sharedFrameState->stride;

            allocate_video_buffer();
        }

        // Copy over the video frame if the frame index changed
        if (m_last_frame_index != sharedFrameState->frame_index)
        {
            if (buffer_size > 0)
            {
                std::memcpy(m_bgr_frame_buffer, sharedFrameState->getBufferMutable(), buffer_size);

                glPixelStorei(GL_UNPACK_SWAP_BYTES, GL_FALSE);
                glPixelStorei(GL_UNPACK_LSB_FIRST, GL_TRUE);
                glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
                glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
                glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
                glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

                glBindTexture(GL_TEXTURE_2D, m_video_frame_texture_id);
                glTexSubImage2D(
                    GL_TEXTURE_2D,
                    0,
                    0,
                    0,
                    m_frame_width,
                    m_frame_height,
                    GL_BGR,
                    GL_UNSIGNED_BYTE,
                    m_bgr_frame_buffer);
                glBindTexture(GL_TEXTURE_2D, 0);
            }

            m_last_frame_index = sharedFrameState->frame_index;
        }
    }

    void allocate_video_buffer()
    {
        size_t buffer_size = SharedVideoFrameHeader::computeVideoBufferSize(m_frame_stride, m_frame_height);

        if (buffer_size > 0)
        {
            // Allocate the buffer to copy the video frame into
            m_bgr_frame_buffer = new unsigned char[buffer_size];

            // Setup the open gl texture to render the video frame into
            glGenTextures(1, &m_video_frame_texture_id);
            glBindTexture(GL_TEXTURE_2D, m_video_frame_texture_id);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexImage2D(
                GL_TEXTURE_2D,
                0,
                GL_RGB,
                m_frame_width,
                m_frame_height,
                0,
                GL_BGR,
                GL_UNSIGNED_BYTE,
                NULL);
            glBindTexture(GL_TEXTURE_2D, 0);
        }
    }

    void free_video_buffer()
    {
        // Free the open gl video texture
        if (m_video_frame_texture_id != 0)
        {
            glDeleteTextures(1, &m_video_frame_texture_id);
            m_video_frame_texture_id = 0;
        }

        // free the video frame buffer
        if (m_bgr_frame_buffer != nullptr)
        {
            delete[] m_bgr_frame_buffer;
            m_bgr_frame_buffer = 0;
        }
    }

    inline unsigned int getVideoFrameTextureID()
    {
        return m_video_frame_texture_id;
    }

protected:
    SharedVideoFrameHeader *getFrameHeader()
    {
        return reinterpret_cast<SharedVideoFrameHeader *>(m_region->get_address());
    }

private:
    const char *m_shared_memory_name;
    boost::interprocess::shared_memory_object *m_shared_memory_object;
    boost::interprocess::mapped_region *m_region;
    unsigned char *m_bgr_frame_buffer;
    int m_frame_width, m_frame_height, m_frame_stride;
    int m_last_frame_index;
    unsigned int m_video_frame_texture_id;
};

//-- public methods -----
AppStage_TestTracker::AppStage_TestTracker(App *app)
    : AppStage(app)
    , m_menuState(AppStage_TestTracker::inactive)
    , m_bStreamIsActive(false)
    , m_shared_memory_accesor(nullptr)
{ }

void AppStage_TestTracker::enter()
{
    const AppStage_TrackerSettings *trackerSettings =
        m_app->getAppStage<AppStage_TrackerSettings>();
    const AppStage_TrackerSettings::TrackerInfo *trackerInfo =
        trackerSettings->getSelectedTrackerInfo();
    assert(trackerInfo->TrackerID != -1);

    m_app->setCameraType(_cameraFixed);

    assert(!m_bStreamIsActive);
    request_tracker_start_stream(trackerInfo->TrackerID);
}

void AppStage_TestTracker::exit()
{
    m_menuState = AppStage_TestTracker::inactive;

    if (m_shared_memory_accesor != nullptr)
    {
        delete[] m_shared_memory_accesor;
        m_shared_memory_accesor = nullptr;
    }
}

void AppStage_TestTracker::update()
{
    // Try and read the next video frame from shared memory
    if (m_shared_memory_accesor != nullptr)
    {
        m_shared_memory_accesor->readVideoFrame();
    }
}

void AppStage_TestTracker::render()
{
    // If there is a video frame available to render, show it
    if (m_shared_memory_accesor != nullptr)
    {
        unsigned int texture_id = m_shared_memory_accesor->getVideoFrameTextureID();

        if (texture_id != 0)
        {
            drawFullscreenTexture(texture_id);
        }
    }
}

void AppStage_TestTracker::renderUI()
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
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 50));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        if (ImGui::Button("Return to Tracker Settings"))
        {
            if (m_bStreamIsActive)
            {
                const AppStage_TrackerSettings *trackerSettings =
                    m_app->getAppStage<AppStage_TrackerSettings>();
                const AppStage_TrackerSettings::TrackerInfo *trackerInfo =
                    trackerSettings->getSelectedTrackerInfo();

                request_tracker_stop_stream(trackerInfo->TrackerID);
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

void AppStage_TestTracker::request_tracker_start_stream(
    int trackerID)
{
    if (m_menuState != AppStage_TestTracker::pendingTrackerStartStreamRequest)
    {
        m_menuState = AppStage_TestTracker::pendingTrackerStartStreamRequest;

        // Tell the psmove service that we want to start streaming data from the tracker
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_START_TRACKER_DATA_STREAM);
        request->mutable_request_start_tracker_data_stream()->set_tracker_id(trackerID);

        m_app->registerCallback(
            ClientPSMoveAPI::send_opaque_request(&request), 
            AppStage_TestTracker::handle_tracker_start_stream_response, this);
    }
}

void AppStage_TestTracker::handle_tracker_start_stream_response(
    ClientPSMoveAPI::eClientPSMoveResultCode ResultCode,
    const ClientPSMoveAPI::t_request_id request_id,
    ClientPSMoveAPI::t_response_handle response_handle,
    void *userdata)
{
    AppStage_TestTracker *thisPtr = static_cast<AppStage_TestTracker *>(userdata);

    switch (ResultCode)
    {
    case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
        {
            thisPtr->m_bStreamIsActive = true;
            thisPtr->m_menuState = AppStage_TestTracker::idle;
            thisPtr->open_shared_memory_stream();
        } break;

    case ClientPSMoveAPI::_clientPSMoveResultCode_error:
    case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
        {
            thisPtr->m_menuState = AppStage_TestTracker::failedTrackerStartStreamRequest;
        } break;
    }
}

void AppStage_TestTracker::open_shared_memory_stream()
{
    const AppStage_TrackerSettings *trackerSettings = m_app->getAppStage<AppStage_TrackerSettings>();
    const AppStage_TrackerSettings::TrackerInfo *trackerInfo = trackerSettings->getSelectedTrackerInfo();

    assert(m_shared_memory_accesor == nullptr);
    m_shared_memory_accesor = new SharedVideoFrameReadOnlyAccessor();

    if (!m_shared_memory_accesor->initialize(trackerInfo->SharedMemoryName.c_str()))
    {
        delete m_shared_memory_accesor;
        m_shared_memory_accesor = nullptr;
    }
}

void AppStage_TestTracker::request_tracker_stop_stream(
    int trackerID)
{
    if (m_bStreamIsActive && m_menuState != AppStage_TestTracker::pendingTrackerStopStreamRequest)
    {
        m_menuState = AppStage_TestTracker::pendingTrackerStopStreamRequest;

        // Tell the psmove service that we want to stop streaming data from the tracker
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_STOP_TRACKER_DATA_STREAM);
        request->mutable_request_stop_tracker_data_stream()->set_tracker_id(trackerID);

        m_app->registerCallback(
            ClientPSMoveAPI::send_opaque_request(&request), 
            AppStage_TestTracker::handle_tracker_stop_stream_response, this);
    }
}

void AppStage_TestTracker::handle_tracker_stop_stream_response(
    ClientPSMoveAPI::eClientPSMoveResultCode ResultCode,
    const ClientPSMoveAPI::t_request_id request_id,
    ClientPSMoveAPI::t_response_handle response_handle,
    void *userdata)
{
    AppStage_TestTracker *thisPtr = static_cast<AppStage_TestTracker *>(userdata);

    // In either case consider the stream as now inactive
    thisPtr->m_bStreamIsActive = false;

    switch (ResultCode)
    {
    case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
        {
            thisPtr->m_menuState = AppStage_TestTracker::inactive;
            thisPtr->close_shared_memory_stream();

            // After closing the stream, we should go back to the tracker settings
            thisPtr->m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
        } break;

    case ClientPSMoveAPI::_clientPSMoveResultCode_error:
    case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
        {
            thisPtr->m_menuState = AppStage_TestTracker::failedTrackerStopStreamRequest;
        } break;
    }
}

void AppStage_TestTracker::close_shared_memory_stream()
{
    if (m_shared_memory_accesor != nullptr)
    {
        delete m_shared_memory_accesor;
        m_shared_memory_accesor = nullptr;
    }
}
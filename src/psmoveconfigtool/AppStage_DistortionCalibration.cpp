// Derived From example 11-1 of "Learning OpenCV: Computer Vision with the OpenCV Library" by Gary Bradski

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

#include <vector>

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
#define CORNER_COUNT (PATTERN_W*PATTERN_H)
#define DEFAULT_SQUARE_LEN_MM 24
#define DESIRED_CAPTURE_BOARD_COUNT 12

#define BOARD_MOVED_PIXEL_DIST 5
#define BOARD_MOVED_ERROR_SUM BOARD_MOVED_PIXEL_DIST*CORNER_COUNT

#define BOARD_NEW_LOCATION_PIXEL_DIST 100 
#define BOARD_NEW_LOCATION_ERROR_SUM BOARD_NEW_LOCATION_PIXEL_DIST*CORNER_COUNT

#define STRAIGHT_LINE_TOLERANCE 5 // error tolerance in pixels

//-- private definitions -----
class OpenCVBufferState
{
public:
    OpenCVBufferState(const PSMClientTrackerInfo &_trackerInfo)
        : trackerInfo(_trackerInfo)
        , frameWidth(static_cast<int>(_trackerInfo.tracker_screen_dimensions.x))
        , frameHeight(static_cast<int>(_trackerInfo.tracker_screen_dimensions.y))
        , capturedBoardCount(0)
    {
        // Video Frame data
        bgrSourceBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC3);
        gsBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC1);
        gsBGRBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC3);
        bgrUndistortBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC3);

        // Chessboard state
        intrinsic_matrix = new cv::Mat(3, 3, CV_64FC1);
        distortion_coeffs = new cv::Mat(5, 1, CV_64FC1);

        // Distortion state
        distortionMapX = new cv::Mat(cv::Size(frameWidth, frameHeight), CV_32FC1);
        distortionMapY = new cv::Mat(cv::Size(frameWidth, frameHeight), CV_32FC1);

        resetCaptureState();
        resetCalibrationState();
    }

    virtual ~OpenCVBufferState()
    {
        // Video Frame data
        delete bgrSourceBuffer;
        delete gsBuffer;
        delete gsBGRBuffer;
        delete bgrUndistortBuffer;

        // Chessboard state
        delete intrinsic_matrix;
        delete distortion_coeffs;

        // Distortion state
        delete distortionMapX;
        delete distortionMapY;
    }

    void resetCaptureState()
    {
        capturedBoardCount= 0;
        bCurrentImagePointsValid= false;
        currentImagePoints.clear();
        lastValidImagePoints.clear();
        quadList.clear();
        imagePointsList.clear();
    }

    void resetCalibrationState()
    {
        reprojectionError= 0.f;

        // Fill in the intrinsic matrix
        intrinsic_matrix->at<double>(0, 0)= trackerInfo.tracker_focal_lengths.x;
        intrinsic_matrix->at<double>(1, 0)= 0.0;
        intrinsic_matrix->at<double>(2, 0)= 0.0;

        intrinsic_matrix->at<double>(0, 1)= 0.0;
        intrinsic_matrix->at<double>(1, 1)= trackerInfo.tracker_focal_lengths.y;
        intrinsic_matrix->at<double>(2, 1)= 0.0;

        intrinsic_matrix->at<double>(0, 2)= trackerInfo.tracker_principal_point.x;
        intrinsic_matrix->at<double>(1, 2)= trackerInfo.tracker_principal_point.y;
        intrinsic_matrix->at<double>(2, 2)= 1.0;

        // Fill in the distortion coefficients
        distortion_coeffs->at<double>(0, 0)= trackerInfo.tracker_k1; 
        distortion_coeffs->at<double>(1, 0)= trackerInfo.tracker_k2;
        distortion_coeffs->at<double>(2, 0)= trackerInfo.tracker_p1;
        distortion_coeffs->at<double>(3, 0)= trackerInfo.tracker_p2;
        distortion_coeffs->at<double>(4, 0)= trackerInfo.tracker_k3;

        // Generate the distortion map that corresponds to the tracker's camera settings
        rebuildDistortionMap();
    }

    void applyVideoFrame(const unsigned char *video_buffer)
    {
        const cv::Mat videoBufferMat(frameHeight, frameWidth, CV_8UC3, const_cast<unsigned char *>(video_buffer));

        // Copy and Flip image about the x-axis
        videoBufferMat.copyTo(*bgrSourceBuffer);

        // Convert the video buffer to a grayscale image
        cv::cvtColor(*bgrSourceBuffer, *gsBuffer, cv::COLOR_BGR2GRAY);
        cv::cvtColor(*gsBuffer, *gsBGRBuffer, cv::COLOR_GRAY2BGR);

        // Apply the distortion map
        cv::remap(
            *bgrSourceBuffer, *bgrUndistortBuffer, 
            *distortionMapX, *distortionMapY, 
            cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    }

    void findAndAppendNewChessBoard(bool appWantsAppend)
    {
        
        if (capturedBoardCount < DESIRED_CAPTURE_BOARD_COUNT)
        {
            std::vector<cv::Point2f> new_image_points;

            // Find chessboard corners:
            if (cv::findChessboardCorners(
                    *gsBuffer, 
                    cv::Size(PATTERN_W, PATTERN_H), 
                    new_image_points, // output corners
                    cv::CALIB_CB_ADAPTIVE_THRESH 
                    + cv::CALIB_CB_FILTER_QUADS 
                    // + cv::CALIB_CB_NORMALIZE_IMAGE is suuuper slow
                    + cv::CALIB_CB_FAST_CHECK))
            {
                // Get subpixel accuracy on those corners
                cv::cornerSubPix(
                    *gsBuffer, 
                    new_image_points, // corners to refine
                    cv::Size(11, 11), // winSize- Half of the side length of the search window
                    cv::Size(-1, -1), // zeroZone- (-1,-1) means no dead zone in search
                    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

                // Append the new chessboard corner pixels into the image_points matrix
                // Append the corresponding 3d chessboard corners into the object_points matrix
                if (new_image_points.size() == CORNER_COUNT) 
                {
                    bCurrentImagePointsValid= false;
                    // See if the board is stationary (didn't move much since last frame)
                    if (currentImagePoints.size() > 0)
                    {
                        float error_sum= 0.f;

                        for (int corner_index= 0; corner_index < CORNER_COUNT; ++corner_index)
                        {
                            float squared_error= static_cast<float>(cv::norm(new_image_points[corner_index] - currentImagePoints[corner_index]));

                            error_sum+= squared_error;
                        }

                        bCurrentImagePointsValid= error_sum <= BOARD_MOVED_ERROR_SUM;
                    }
                    else
                    {
                        // We don't have previous capture.
                        bCurrentImagePointsValid= true;
                    }

                    // See if the board moved far enough from the last valid location
                    if (bCurrentImagePointsValid)
                    {
                        if (lastValidImagePoints.size() > 0)
                        {
                            float error_sum= 0.f;

                            for (int corner_index= 0; corner_index < CORNER_COUNT; ++corner_index)
                            {
                                float squared_error= static_cast<float>(cv::norm(new_image_points[corner_index] - lastValidImagePoints[corner_index]));

                                error_sum+= squared_error;
                            }

                            bCurrentImagePointsValid= error_sum >= BOARD_NEW_LOCATION_ERROR_SUM;
                        }
                    }

                    if (bCurrentImagePointsValid)
                    {
                        bCurrentImagePointsValid= areGridLinesStraight(new_image_points);
                    }

                    // If it's a valid new location, append it to the board list
                    if (bCurrentImagePointsValid && appWantsAppend)
                    {
                        // Keep track of the corners of all of the chessboards we sample
                        quadList.push_back(new_image_points[0]);
                        quadList.push_back(new_image_points[PATTERN_W - 1]);
                        quadList.push_back(new_image_points[CORNER_COUNT-1]);
                        quadList.push_back(new_image_points[CORNER_COUNT-PATTERN_W]);                        

                        // Append the new images points and object points
                        imagePointsList.push_back(new_image_points);

                        // Remember the last valid captured points
                        lastValidImagePoints= currentImagePoints;

                        // Keep track of how many boards have been captured so far
                        capturedBoardCount++;
                    }

                    // Remember the last set of valid corners
                    currentImagePoints= new_image_points;
                }
            }
        }
    }

    static bool areGridLinesStraight(const std::vector<cv::Point2f> &corners)
    {
        assert(corners.size() == CORNER_COUNT);
        bool bAllLinesStraight= true;

        for (int line_index= 0; bAllLinesStraight && line_index < PATTERN_H; ++line_index)
        {
            int start_index= line_index*PATTERN_W;
            int end_index= start_index + PATTERN_W - 1;

            cv::Point2f line_start= corners[start_index];
            cv::Point2f line_end= corners[end_index];

            for (int point_index= start_index + 1; bAllLinesStraight && point_index < end_index; ++point_index)
            {
                cv::Point2f point= corners[point_index];

                if (distanceToLine(line_start, line_end, point) > STRAIGHT_LINE_TOLERANCE)
                {
                    bAllLinesStraight= false;
                }
            }
        }

        return bAllLinesStraight;
    }

    static float distanceToLine(cv::Point2f line_start, cv::Point2f line_end, cv::Point2f point)
    {
        const auto start_to_end= line_end - line_start;
        const auto start_to_point= point - line_start;

        float area = static_cast<float>(start_to_point.cross(start_to_end));
        float line_length= static_cast<float>(cv::norm(start_to_end));
        return fabsf(safe_divide_with_default(area, line_length, 0.f));
    }

    bool computeCameraCalibration(const float square_length_mm)
    {
        bool bSuccess= false;

        if (capturedBoardCount >= DESIRED_CAPTURE_BOARD_COUNT)
        {
            // Only need to calculate objectPointsList once,
            // then resize for each set of image points.
            std::vector<std::vector<cv::Point3f> > objectPointsList(1);
            calcBoardCornerPositions(square_length_mm, objectPointsList[0]);
            objectPointsList.resize(imagePointsList.size(), objectPointsList[0]);
            
            // Compute the camera intrinsic matrix and distortion parameters
            reprojectionError= 
                cv::calibrateCamera(
                    objectPointsList, imagePointsList,
                    cv::Size(frameWidth, frameHeight), 
                    *intrinsic_matrix, *distortion_coeffs, // Output we care about
                    cv::noArray(), cv::noArray(), // best fit board poses as rvec/tvec pairs
                    cv::CALIB_FIX_ASPECT_RATIO,
                    cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON));
            
            // Regenerate the distortion map now for the new calibration
            rebuildDistortionMap();

            bSuccess= true;
        }

        return bSuccess;
    }

    void rebuildDistortionMap()
    {
        cv::initUndistortRectifyMap(
            *intrinsic_matrix, *distortion_coeffs, 
            cv::noArray(), // unneeded rectification transformation computed by stereoRectify()
                                // newCameraMatrix - can be computed by getOptimalNewCameraMatrix(), but
            *intrinsic_matrix, // "In case of a monocular camera, newCameraMatrix is usually equal to cameraMatrix"
            cv::Size(frameWidth, frameHeight),
            CV_32FC1, // Distortion map type
            *distortionMapX, *distortionMapY);
    }
    
    void calcBoardCornerPositions(const float square_length_mm, std::vector<cv::Point3f>& corners)
    {
        corners.clear();
        
        for( int i = 0; i < PATTERN_H; ++i )
        {
            for( int j = 0; j < PATTERN_W; ++j )
            {
                corners.push_back(cv::Point3f(float(j*square_length_mm), float(i*square_length_mm), 0.f));
            }
        }
    }

    const PSMClientTrackerInfo &trackerInfo;
    int frameWidth;
    int frameHeight;

    // Video frame buffers
    cv::Mat *bgrSourceBuffer;
    cv::Mat *gsBuffer;
    cv::Mat *gsBGRBuffer;
    cv::Mat *bgrUndistortBuffer;

    // Chess board computed state
    int capturedBoardCount;
    std::vector<cv::Point2f> lastValidImagePoints;
    std::vector<cv::Point2f> currentImagePoints;
    bool bCurrentImagePointsValid;
    std::vector<cv::Point2f> quadList;
    std::vector<std::vector<cv::Point2f>> imagePointsList;

    // Calibration state
    double reprojectionError;
    cv::Mat *intrinsic_matrix;
    cv::Mat *distortion_coeffs;

    // Distortion preview
    cv::Mat *distortionMapX;
    cv::Mat *distortionMapY;
};

//-- public methods -----
AppStage_DistortionCalibration::AppStage_DistortionCalibration(App *app)
    : AppStage(app)
    , m_menuState(AppStage_DistortionCalibration::inactive)
    , m_videoDisplayMode(AppStage_DistortionCalibration::eVideoDisplayMode::mode_bgr)
	, m_square_length_mm(DEFAULT_SQUARE_LEN_MM)
    , m_trackerExposure(0.0)
    , m_trackerGain(0.0)
    , m_bStreamIsActive(false)
    , m_tracker_view(nullptr)
    , m_video_texture(nullptr)
    , m_opencv_state(nullptr)
{ }

void AppStage_DistortionCalibration::enter()
{
    const AppStage_TrackerSettings *trackerSettings =
        m_app->getAppStage<AppStage_TrackerSettings>();
    const PSMClientTrackerInfo *trackerInfo = trackerSettings->getSelectedTrackerInfo();
    assert(trackerInfo->tracker_id != -1);

    m_app->setCameraType(_cameraFixed);

    assert(m_tracker_view == nullptr);
	PSM_AllocateTrackerListener(trackerInfo->tracker_id, trackerInfo);
	m_tracker_view = PSM_GetTracker(trackerInfo->tracker_id);

	m_square_length_mm = DEFAULT_SQUARE_LEN_MM;

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

    // Revert unsaved modifications to the tracker settings
    request_tracker_reload_settings();

    PSM_FreeTrackerListener(m_tracker_view->tracker_info.tracker_id);
    m_tracker_view = nullptr;
    m_bStreamIsActive= false;
}

void AppStage_DistortionCalibration::update()
{
    if (m_menuState == AppStage_DistortionCalibration::capture ||
        m_menuState == AppStage_DistortionCalibration::complete)
    {
        assert(m_video_texture != nullptr);

        // Try and read the next video frame from shared memory
        if (PSM_PollTrackerVideoStream(m_tracker_view->tracker_info.tracker_id) == PSMResult_Success)
        {
            const unsigned char *video_frame_buffer= nullptr;
			if (PSM_GetTrackerVideoFrameBuffer(m_tracker_view->tracker_info.tracker_id, &video_frame_buffer) == PSMResult_Success)
			{
				// Update the video frame buffers
				m_opencv_state->applyVideoFrame(video_frame_buffer);

				// Update the video frame display texture
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

            if (m_menuState == AppStage_DistortionCalibration::capture)
            {
                
                // Update the chess board capture state
                ImGuiIO io_state = ImGui::GetIO();
                m_opencv_state->findAndAppendNewChessBoard(io_state.KeysDown[32]);

                if (m_opencv_state->capturedBoardCount >= DESIRED_CAPTURE_BOARD_COUNT)
                {
                    
                    m_opencv_state->computeCameraCalibration(m_square_length_mm); //Will update intrinsic_matrix and distortion_coeffs
                    cv::Mat *intrinsic_matrix= m_opencv_state->intrinsic_matrix;
                    cv::Mat *distortion_coeffs= m_opencv_state->distortion_coeffs;
                    
                    
                    float frameWidth= static_cast<float>(m_opencv_state->frameWidth);
                    float frameHeight= static_cast<float>(m_opencv_state->frameHeight);
                    
//                    double apertureWidthmm = 3.984;
//                    double apertureHeightmm = 2.952;
//                    double fovX;
//                    double fovY;
//                    double focalLength;
//                    double aspectRatio;
//                    cv::Point2d principalPoint;
//                    cv::calibrationMatrixValues(*intrinsic_matrix,
//                                                cv::Size(frameWidth, frameHeight),
//                                                apertureWidthmm,
//                                                apertureHeightmm,
//                                                fovX,
//                                                fovY,
//                                                focalLength,
//                                                principalPoint,
//                                                aspectRatio);
//                    std::cout << "fovX: " << fovX << "; fovY: " << fovY;
//                    std::cout << "; focalLength: " << focalLength;
//                    std::cout << "; aspectRatio: " << aspectRatio;
//                    std::cout << "; principalPoint: " << principalPoint.x << ", " << principalPoint.y;
//                    std::cout << std::endl;
                    
                    const float f_x= static_cast<float>(intrinsic_matrix->at<double>(0, 0));
                    const float f_y= static_cast<float>(intrinsic_matrix->at<double>(1, 1));
                    const float p_x= static_cast<float>(intrinsic_matrix->at<double>(0, 2));
                    const float p_y= static_cast<float>(intrinsic_matrix->at<double>(1, 2));

                    const float k_1= static_cast<float>(distortion_coeffs->at<double>(0, 0));
                    const float k_2= static_cast<float>(distortion_coeffs->at<double>(1, 0));
                    const float p_1= static_cast<float>(distortion_coeffs->at<double>(2, 0));
                    const float p_2= static_cast<float>(distortion_coeffs->at<double>(3, 0));
                    const float k_3= static_cast<float>(distortion_coeffs->at<double>(4, 0));
                    
                    double fovx = 2 * atan(frameWidth / (2 * f_x)) * 180.0 / CV_PI;
                    double fovy = 2 * atan(frameHeight / (2 * f_y)) * 180.0 / CV_PI;
                    std::cout << "Manual fov x: " << fovx << "; y: " << fovy << std::endl;

                    // Update the camera intrinsics for this camera
                    request_tracker_set_intrinsic(
                        f_x, f_y,
                        p_x, p_y,
                        k_1, k_2, k_3,
                        p_1, p_2);

                    m_videoDisplayMode= AppStage_DistortionCalibration::mode_undistored;
                    m_menuState= AppStage_DistortionCalibration::complete;
                }
            }
        }
    }
}

void AppStage_DistortionCalibration::render()
{
    if (m_menuState == AppStage_DistortionCalibration::capture ||
        m_menuState == AppStage_DistortionCalibration::complete)
    {
        assert(m_video_texture != nullptr);
        unsigned int texture_id = m_video_texture->texture_id;

        if (texture_id != 0)
        {
            drawFullscreenTexture(texture_id);
        }

        if (m_menuState == AppStage_DistortionCalibration::capture)
        {
            float frameWidth= static_cast<float>(m_opencv_state->frameWidth);
            float frameHeight= static_cast<float>(m_opencv_state->frameHeight);

            // Draw the last valid capture chessboard
            if (m_opencv_state->lastValidImagePoints.size() > 0)
            {
                drawOpenCVChessBoard(
                    frameWidth, frameHeight, 
                    reinterpret_cast<float *>(m_opencv_state->lastValidImagePoints.data()), // cv::point2f is just two floats 
                    static_cast<int>(m_opencv_state->lastValidImagePoints.size()),
                    true);
            }            

            // Draw the most recently capture chessboard
            if (m_opencv_state->currentImagePoints.size() > 0)
            {
                drawOpenCVChessBoard(
                    frameWidth, frameHeight, 
                    reinterpret_cast<float *>(m_opencv_state->currentImagePoints.data()), // cv::point2f is just two floats 
                    static_cast<int>(m_opencv_state->currentImagePoints.size()),
                    m_opencv_state->bCurrentImagePointsValid);
            }

            // Draw the outlines of all of the chess boards 
            if (m_opencv_state->quadList.size() > 0)
            {
                drawQuadList2d(
                    frameWidth, frameHeight, 
                    glm::vec3(1.f, 1.f, 0.f), 
                        reinterpret_cast<float *>(m_opencv_state->quadList.data()), // cv::point2f is just two floats 
                        static_cast<int>(m_opencv_state->quadList.size()));
            }
        }
    }
}

void AppStage_DistortionCalibration::renderUI()
{
    const float k_panel_width = 200.f;
    const char *k_window_title = "Distortion Calibration";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
	case eMenuState::showWarning:
		{
			const float k_wide_panel_width = 350.f;
			ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_wide_panel_width / 2.f, 20.f));
			ImGui::SetNextWindowSize(ImVec2(k_wide_panel_width, 130));

			ImGui::Begin("WARNING", nullptr, window_flags);

			ImGui::TextWrapped(
				"The tracker you want to calibrate already has pre-computed distortion and focal lengths." \
				"If you proceed you will be overriding these defaults.");

			ImGui::Spacing();

			if (ImGui::Button("Continue"))
			{
				m_menuState = eMenuState::enterBoardSettings;
			}
			ImGui::SameLine();
			if (ImGui::Button("Cancel"))
			{
				request_exit();
			}

			ImGui::End();
		} break;
	case eMenuState::enterBoardSettings:
		{
			const float k_wide_panel_width = 350.f;
			ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_wide_panel_width / 2.f, 20.f));
			ImGui::SetNextWindowSize(ImVec2(k_wide_panel_width, 100));

			ImGui::Begin("Enter Calibration Settings", nullptr, window_flags);

			ImGui::PushItemWidth(100.f);
			if (ImGui::InputFloat("Square Length (mm)", &m_square_length_mm, 0.5f, 1.f, 1))
			{
				if (m_square_length_mm < 1.f)
				{
					m_square_length_mm = 1.f;
				}

				if (m_square_length_mm > 100.f)
				{
					m_square_length_mm = 100.f;
				}
			}
			ImGui::PopItemWidth();

			ImGui::Spacing();

			if (ImGui::Button("Ok"))
			{
				// Crank up the exposure and gain so that we can see the chessboard
				// These overrides will get rolled back once tracker gets closed
				request_tracker_set_temp_exposure(128.f);
				request_tracker_set_temp_gain(128.f);

				m_menuState = eMenuState::capture;
			}
			ImGui::SameLine();
			if (ImGui::Button("Cancel"))
			{
				request_exit();
			}

			ImGui::End();
		} break;
    case eMenuState::capture:
        {
            assert (m_opencv_state != nullptr);

            {
                ImGui::SetNextWindowPos(ImVec2(10.f, 10.f));
                ImGui::SetNextWindowSize(ImVec2(275, 150));
                ImGui::Begin("Video Controls", nullptr, window_flags);

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

                if (ImGui::Button("-##Exposure"))
                {
                    request_tracker_set_temp_exposure(m_trackerExposure - 8);
                }
                ImGui::SameLine();
                if (ImGui::Button("+##Exposure"))
                {
                    request_tracker_set_temp_exposure(m_trackerExposure + 8);
                }
                ImGui::SameLine();
                ImGui::Text("Exposure: %f", m_trackerExposure);

                if (ImGui::Button("-##Gain"))
                {
                    request_tracker_set_temp_gain(m_trackerGain - 8);
                }
                ImGui::SameLine();
                if (ImGui::Button("+##Gain"))
                {
                    request_tracker_set_temp_gain(m_trackerGain + 8);
                }
                ImGui::SameLine();
                ImGui::Text("Gain: %f", m_trackerGain);

                ImGui::End();
            }

            {
                ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
                ImGui::SetNextWindowSize(ImVec2(k_panel_width, 110));
                ImGui::Begin(k_window_title, nullptr, window_flags);

                const float samplePercentage= 
                    static_cast<float>(m_opencv_state->capturedBoardCount) / static_cast<float>(DESIRED_CAPTURE_BOARD_COUNT);
                ImGui::ProgressBar(samplePercentage, ImVec2(k_panel_width - 20, 20));

                if (ImGui::Button("Restart"))
                {
                    m_opencv_state->resetCaptureState();
                    m_opencv_state->resetCalibrationState();
                }
                ImGui::SameLine();
                if (ImGui::Button("Cancel"))
                {
                    request_exit();
                }
                if (m_opencv_state->bCurrentImagePointsValid)
                {
                    ImGui::Text("Press spacebar to capture");
                }

                ImGui::End();
            }
        } break;

    case eMenuState::complete:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 10.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 110));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Calibration complete!");
            ImGui::Text("Error: %f", m_opencv_state->reprojectionError);

            if (ImGui::Button("Ok"))
            {
                request_exit();
            }

            if (ImGui::Button("Redo Calibration"))
            {
                m_opencv_state->resetCaptureState();
                m_opencv_state->resetCalibrationState();
                m_videoDisplayMode= AppStage_DistortionCalibration::mode_bgr;
                m_menuState= eMenuState::capture;
            }

            ImGui::End();
        } break;

    case eMenuState::pendingTrackerStartStreamRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 50));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Waiting for tracker stream to start...");

            ImGui::End();
        } break;

    case eMenuState::failedTrackerStartStreamRequest:
    case eMenuState::failedTrackerOpenStreamRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            if (m_menuState == eMenuState::failedTrackerStartStreamRequest)
                ImGui::Text("Failed to start tracker stream!");
            else
                ImGui::Text("Failed to open tracker stream!");

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

    case eMenuState::pendingTrackerStopStreamRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 50));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Waiting for tracker stream to stop...");

            ImGui::End();
        } break;

    case eMenuState::failedTrackerStopStreamRequest:
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
		PSMRequestID requestID;
		PSM_StartTrackerDataStreamAsync(
			m_tracker_view->tracker_info.tracker_id, 
			&requestID);
		PSM_RegisterCallback(requestID, AppStage_DistortionCalibration::handle_tracker_start_stream_response, this);
    }
}

void AppStage_DistortionCalibration::handle_tracker_start_stream_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    AppStage_DistortionCalibration *thisPtr = static_cast<AppStage_DistortionCalibration *>(userdata);

    switch (response->result_code)
    {
    case PSMResult_Success:
        {
            PSMTracker *trackerView= thisPtr->m_tracker_view;

            thisPtr->m_bStreamIsActive = true;

            // Open the shared memory that the video stream is being written to
            if (PSM_OpenTrackerVideoStream(trackerView->tracker_info.tracker_id) == PSMResult_Success)
            {
                const PSMClientTrackerInfo &trackerInfo= trackerView->tracker_info;
                const int width= static_cast<int>(trackerInfo.tracker_screen_dimensions.x);
                const int height= static_cast<int>(trackerInfo.tracker_screen_dimensions.y);

                // Create a texture to render the video frame to
                thisPtr->m_video_texture = new TextureAsset();
                thisPtr->m_video_texture->init(
                    width, 
                    height,
                    GL_RGB, // texture format
                    GL_BGR, // buffer format
                    nullptr);

                // Allocate an opencv buffer 
                thisPtr->m_opencv_state = new OpenCVBufferState(trackerInfo);

				// Warn the user if they are about to change the distortion calibration settings for the PS3EYE
				if (trackerInfo.tracker_type == PSMTrackerType::PSMTracker_PS3Eye)
				{
					thisPtr->m_menuState = AppStage_DistortionCalibration::showWarning;
				}
				else
				{
					// Start capturing chess boards
					thisPtr->m_menuState = AppStage_DistortionCalibration::enterBoardSettings;
				}
            }
            else
            {
                thisPtr->m_menuState = AppStage_DistortionCalibration::failedTrackerOpenStreamRequest;
            }
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
	case PSMResult_Timeout:
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
		PSMRequestID request_id;
		PSM_StopTrackerDataStreamAsync(m_tracker_view->tracker_info.tracker_id, &request_id);
		PSM_RegisterCallback(request_id, AppStage_DistortionCalibration::handle_tracker_stop_stream_response, this);
    }
}

void AppStage_DistortionCalibration::handle_tracker_stop_stream_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    AppStage_DistortionCalibration *thisPtr = static_cast<AppStage_DistortionCalibration *>(userdata);

    // In either case consider the stream as now inactive
    thisPtr->m_bStreamIsActive = false;

    switch (response->result_code)
    {
    case PSMResult_Success:
        {
            thisPtr->m_menuState = AppStage_DistortionCalibration::inactive;

            // Close the shared memory buffer
			PSM_CloseTrackerVideoStream(thisPtr->m_tracker_view->tracker_info.tracker_id);

            // Free the texture we were rendering to
            if (thisPtr->m_video_texture != nullptr)
            {
                delete thisPtr->m_video_texture;
                thisPtr->m_video_texture = nullptr;
            }

            // After closing the stream, we should go back to the tracker settings
            thisPtr->m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
	case PSMResult_Timeout:
        {
            thisPtr->m_menuState = AppStage_DistortionCalibration::failedTrackerStopStreamRequest;
        } break;
    }
}

void AppStage_DistortionCalibration::request_tracker_set_temp_gain(float gain)
{
    m_trackerGain= gain;

    // Tell the psmove service that we want to change gain, but not save the change
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_GAIN);
    request->mutable_request_set_tracker_gain()->set_tracker_id(m_tracker_view->tracker_info.tracker_id);
    request->mutable_request_set_tracker_gain()->set_value(gain);
    request->mutable_request_set_tracker_gain()->set_save_setting(false);

    PSM_SendOpaqueRequest(&request, nullptr);
}

void AppStage_DistortionCalibration::request_tracker_set_temp_exposure(float exposure)
{
    m_trackerExposure= exposure;

    // Tell the psmove service that we want to change exposure, but not save the change.
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_EXPOSURE);
    request->mutable_request_set_tracker_exposure()->set_tracker_id(m_tracker_view->tracker_info.tracker_id);
    request->mutable_request_set_tracker_exposure()->set_value(exposure);
    request->mutable_request_set_tracker_exposure()->set_save_setting(false);

    PSM_SendOpaqueRequest(&request, nullptr);
}

void AppStage_DistortionCalibration::request_tracker_set_intrinsic(
    float focalLengthX, float focalLengthY,
    float principalX, float principalY,
    float distortionK1, float distortionK2, float distortionK3,
    float distortionP1, float distortionP2)
{
    // Update the intrinsic state on the tracker info
    // so that this becomes the new reset point.
    PSMClientTrackerInfo trackerInfo= m_tracker_view->tracker_info;
    trackerInfo.tracker_focal_lengths.x= focalLengthX;
    trackerInfo.tracker_focal_lengths.y= focalLengthY;
    trackerInfo.tracker_principal_point.x= principalX;
    trackerInfo.tracker_principal_point.y= principalY;
    trackerInfo.tracker_k1= distortionK1;
    trackerInfo.tracker_k2= distortionK2;
    trackerInfo.tracker_k3= distortionK3;
    trackerInfo.tracker_p1= distortionP1;
    trackerInfo.tracker_p2= distortionP2;

    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_INTRINSICS);
    request->mutable_request_set_tracker_intrinsics()->set_tracker_id(m_tracker_view->tracker_info.tracker_id);

    PSMoveProtocol::Pixel *focal_lengths= request->mutable_request_set_tracker_intrinsics()->mutable_tracker_focal_lengths();
    focal_lengths->set_x(focalLengthX);
    focal_lengths->set_y(focalLengthY);

    PSMoveProtocol::Pixel *principal_point= request->mutable_request_set_tracker_intrinsics()->mutable_tracker_principal_point();
    principal_point->set_x(principalX);
    principal_point->set_y(principalY);

    request->mutable_request_set_tracker_intrinsics()->set_tracker_k1(distortionK1);
    request->mutable_request_set_tracker_intrinsics()->set_tracker_k2(distortionK2);
    request->mutable_request_set_tracker_intrinsics()->set_tracker_k3(distortionK3);
    request->mutable_request_set_tracker_intrinsics()->set_tracker_p1(distortionP1);
    request->mutable_request_set_tracker_intrinsics()->set_tracker_p2(distortionP2);    

    PSM_SendOpaqueRequest(&request, nullptr);
}

void AppStage_DistortionCalibration::request_tracker_reload_settings()
{
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_RELOAD_TRACKER_SETTINGS);
    request->mutable_request_reload_tracker_settings()->set_tracker_id(m_tracker_view->tracker_info.tracker_id);

    PSM_SendOpaqueRequest(&request, nullptr);
}

void AppStage_DistortionCalibration::request_exit()
{
    if (m_bStreamIsActive)
    {
        const AppStage_TrackerSettings *trackerSettings =
            m_app->getAppStage<AppStage_TrackerSettings>();
        const PSMClientTrackerInfo *trackerInfo = trackerSettings->getSelectedTrackerInfo();

        request_tracker_stop_stream();
    }
    else
    {
        m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
    }
}

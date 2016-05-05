//-- inludes -----
#include "AppStage_MagnetometerCalibration.h"
#include "AppStage_ControllerSettings.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "ClientPSMoveAPI.h"
#include "ClientControllerView.h"
#include "GeometryUtility.h"
#include "Logger.h"
#include "MathAlignment.h"
#include "MathGLM.h"
#include "MathUtility.h"
#include "PSMoveProtocolInterface.h"
#include "Renderer.h"
#include "UIConstants.h"

#include "SDL_keycode.h"

#include <imgui.h>

#include <algorithm>

//-- statics ----
const char *AppStage_MagnetometerCalibration::APP_STAGE_NAME= "MagnetometerCalibration";

//-- constants -----
const int k_sample_count_target = 200;
const int k_sample_range_target= 320;
const double k_stabilize_wait_time_ms= 1000.f;
const int k_desired_magnetometer_sample_count= 100;
const int k_min_sample_distance= 20;
const int k_min_sample_distance_sq= k_min_sample_distance*k_min_sample_distance;

//-- private methods -----
static void expandMagnetometerBounds(
    const PSMoveIntVector3 &sample, PSMoveIntVector3 &minSampleExtents, PSMoveIntVector3 &maxSampleExtents);
static int computeMagnetometerCalibrationMinRange(
    const PSMoveIntVector3 &minSampleExtents, const PSMoveIntVector3 &maxSampleExtents);
static int computeMagnetometerCalibrationMaxRange(
    const PSMoveIntVector3 &minSampleExtents, const PSMoveIntVector3 &maxSampleExtents);
static PSMoveFloatVector3 computeNormalizedMagnetometerVector(
    const PSMoveIntVector3 &sample,
    const PSMoveIntVector3 &minSampleExtents,
    const PSMoveIntVector3 &maxSampleExtents);
static bool isMoveStableAndAlignedWithGravity(ClientControllerView *m_controllerView);
static void write_calibration_parameter(const Eigen::Vector3f &in_vector, PSMoveProtocol::FloatVector *out_vector);

//-- public methods -----
AppStage_MagnetometerCalibration::AppStage_MagnetometerCalibration(App *app) 
    : AppStage(app)
    , m_bBypassCalibration(false)
    , m_menuState(AppStage_MagnetometerCalibration::inactive)
    , m_pendingAppStage(nullptr)
    , m_controllerView(nullptr)
    , m_isControllerStreamActive(false)
    , m_lastControllerSeqNum(-1)
    , m_lastMagnetometer()
    , m_lastAccelerometer()
    , m_alignedSamples(new MagnetometerAlignedSamples)
    , m_sampleCount(0)
    , m_samplePercentage(0)
    , m_minSampleExtent()
    , m_maxSampleExtent()
    , m_ellipseFitMethod(_ellipse_fit_method_box)
    , m_led_color_r(0)
    , m_led_color_g(0)
    , m_led_color_b(0)
    , m_stableStartTime()
    , m_bIsStable(false)
    , m_identityPoseMVectorSum()
    , m_identityPoseSampleCount(0)
{ 
}

AppStage_MagnetometerCalibration::~AppStage_MagnetometerCalibration()
{
    delete m_alignedSamples;
    m_alignedSamples = nullptr;
}

void AppStage_MagnetometerCalibration::enter()
{
    const AppStage_ControllerSettings *controllerSettings= 
        m_app->getAppStage<AppStage_ControllerSettings>();
    const AppStage_ControllerSettings::ControllerInfo *controllerInfo=
        controllerSettings->getSelectedControllerInfo();

    m_app->setCameraType(_cameraOrbit);
    m_app->getOrbitCamera()->resetOrientation();
    m_app->getOrbitCamera()->setCameraOrbitRadius(1000.f); // zoom out to see the magnetometer data at scale

    assert(controllerInfo->ControllerID != -1);
    assert(m_controllerView == nullptr);
    m_controllerView= ClientPSMoveAPI::allocate_controller_view(controllerInfo->ControllerID);

    m_lastMagnetometer= *k_psmove_int_vector3_zero;
    m_lastAccelerometer= *k_psmove_float_vector3_zero;

    m_sampleCount = 0;
    m_samplePercentage = 0;

    m_minSampleExtent= *k_psmove_int_vector3_zero;
    m_maxSampleExtent= *k_psmove_int_vector3_zero;

    m_led_color_r= 0;
    m_led_color_g= 0;
    m_led_color_b= 0;

	m_stableStartTime = std::chrono::time_point<std::chrono::high_resolution_clock>();
    m_bIsStable= false;

    m_identityPoseMVectorSum= *k_psmove_int_vector3_zero;
    m_identityPoseSampleCount= 0;

    m_menuState= eCalibrationMenuState::waitingForStreamStartResponse;
    assert(!m_isControllerStreamActive);
    m_lastControllerSeqNum= -1;

    m_app->registerCallback(
        ClientPSMoveAPI::start_controller_data_stream(m_controllerView, ClientPSMoveAPI::includeRawSensorData),
        &AppStage_MagnetometerCalibration::handle_acquire_controller, this);
}

void AppStage_MagnetometerCalibration::exit()
{
    assert(m_controllerView != nullptr);
    ClientPSMoveAPI::free_controller_view(m_controllerView);
    m_controllerView= nullptr;
    m_menuState= eCalibrationMenuState::inactive;

    // Reset the orbit camera back to default orientation and scale
    m_app->getOrbitCamera()->reset();
}

void AppStage_MagnetometerCalibration::update()
{
    bool bControllerDataUpdatedThisFrame= false;

    if (m_isControllerStreamActive && m_controllerView->GetSequenceNum() != m_lastControllerSeqNum)
    {
        const PSMoveRawSensorData &sensorData= m_controllerView->GetPSMoveView().GetRawSensorData();

        m_lastMagnetometer= sensorData.Magnetometer;
        m_lastAccelerometer= sensorData.Accelerometer;
        m_lastControllerSeqNum= m_controllerView->GetSequenceNum();
        bControllerDataUpdatedThisFrame= true;
    }

    switch (m_menuState)
    {
    case eCalibrationMenuState::waitingForStreamStartResponse:
        {
            if (bControllerDataUpdatedThisFrame)
            {
                if (m_controllerView->GetPSMoveView().GetHasValidHardwareCalibration())
                {
                    m_sampleCount = 0;
                    m_samplePercentage = 0;

                    m_minSampleExtent= *k_psmove_int_vector3_zero;
                    m_maxSampleExtent= *k_psmove_int_vector3_zero;
                    
                    m_led_color_r= 255; m_led_color_g= 0; m_led_color_b= 0;

                    if (m_bBypassCalibration)
                    {
                        m_app->getOrbitCamera()->resetOrientation();
                        m_menuState= AppStage_MagnetometerCalibration::complete;
                    }
                    else
                    {
                        m_menuState= AppStage_MagnetometerCalibration::measureBExtents;
                    }
                }
                else
                {
                    m_menuState= AppStage_MagnetometerCalibration::failedBadCalibration;
                }
            }
        } break;
    case eCalibrationMenuState::failedStreamStart:
    case eCalibrationMenuState::failedBadCalibration:
        {
        } break;
    case eCalibrationMenuState::measureBExtents:
        {
            if (bControllerDataUpdatedThisFrame && m_sampleCount < k_max_magnetometer_samples)
            {
                // Grow the measurement extents bounding box
                expandMagnetometerBounds(m_lastMagnetometer, m_minSampleExtent, m_maxSampleExtent);

                // Make sure this sample isn't too close to another sample
                bool bTooClose= false;
                for (int sampleIndex= m_sampleCount-1; sampleIndex >= 0; --sampleIndex)
                {
                    const PSMoveIntVector3 diff= m_lastMagnetometer - m_magnetometerIntSamples[sampleIndex];
                    const int distanceSquared= diff.lengthSquared();

                    if (distanceSquared < k_min_sample_distance_sq)
                    {
                        bTooClose= true;
                        break;
                    }
                }

                // Display the last N samples
                if (!bTooClose)
                {
                    // Store the new sample
                    m_magnetometerIntSamples[m_sampleCount]= m_lastMagnetometer;
                    m_alignedSamples->magnetometerEigenSamples[m_sampleCount] = psmove_int_vector3_to_eigen_vector3(m_lastMagnetometer);
                    ++m_sampleCount;

                    // Compute a best fit ellipsoid for the sample points
                    switch (m_ellipseFitMethod)
                    {
                    case _ellipse_fit_method_box:
                        eigen_alignment_fit_bounding_box_ellipsoid(
                            m_alignedSamples->magnetometerEigenSamples, m_sampleCount, m_sampleFitEllipsoid);
                        break;
                    case _ellipse_fit_method_min_volume:
                        eigen_alignment_fit_min_volume_ellipsoid(
                            m_alignedSamples->magnetometerEigenSamples, m_sampleCount, 0.0001f, m_sampleFitEllipsoid);
                        break;
                    }

                    // Update the extents progress based on min extent size
                    int minRange = computeMagnetometerCalibrationMinRange(m_minSampleExtent, m_maxSampleExtent);
                    if (minRange > 0)
                    {
                        m_samplePercentage = 
                            std::min(
                                std::min((100 * m_sampleCount) / k_sample_count_target, 100),
                                std::min((100 * minRange) / k_sample_range_target, 100));

                        int led_color_r = (255 * (100 - m_samplePercentage)) / 100;
                        int led_color_g = (255 * m_samplePercentage) / 100;
                        int led_color_b = 0;

                        // Send request to change led color, don't care about callback
                        if (led_color_r != m_led_color_r || led_color_g != m_led_color_g || led_color_b != m_led_color_b)
                        {
                            m_led_color_r = led_color_r;
                            m_led_color_g = led_color_g;
                            m_led_color_b = led_color_b;
                            ClientPSMoveAPI::set_led_color(m_controllerView, m_led_color_r, m_led_color_g, m_led_color_b);
                        }
                    }
                }
            }
        } break;
    case eCalibrationMenuState::waitForGravityAlignment:
        {
            if (isMoveStableAndAlignedWithGravity(m_controllerView))
            {
                std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();

                if (m_bIsStable)
                {
                    std::chrono::duration<double, std::milli> stableDuration = now - m_stableStartTime;
    
                    if (stableDuration.count() >= k_stabilize_wait_time_ms)
                    {
                        m_identityPoseMVectorSum= *k_psmove_int_vector3_zero;
                        m_identityPoseSampleCount = 0;
                        m_menuState= AppStage_MagnetometerCalibration::measureBDirection;
                    }
                }
                else
                {
                    m_bIsStable= true;
                    m_stableStartTime= now;
                }
            }
            else
            {
                if (m_bIsStable)
                {
                    m_bIsStable= false;
                }
            }
        } break;
    case eCalibrationMenuState::measureBDirection:
        {
            if (isMoveStableAndAlignedWithGravity(m_controllerView))
            {
                if (bControllerDataUpdatedThisFrame)
                {
                    m_identityPoseMVectorSum = m_identityPoseMVectorSum + m_lastMagnetometer;
                    ++m_identityPoseSampleCount;

                    if (m_identityPoseSampleCount > k_desired_magnetometer_sample_count)
                    {
                        float N= static_cast<float>(m_identityPoseSampleCount);

                        // The average magnetometer direction was recorded while the controller
                        // was in the cradle pose
                        PSMoveFloatVector3 identityPoseAvg= m_identityPoseMVectorSum.castToFloatVector3().unsafe_divide(N);

                        // Project the averaged magnetometer sample into the space of the ellipse
                        // And then normalize it (any deviation from unit length is error)
                        Eigen::Vector3f projected_sample =
                            eigen_alignment_project_point_on_ellipsoid_basis(
                                psmove_float_vector3_to_eigen_vector3(identityPoseAvg),
                                m_sampleFitEllipsoid);
                        eigen_vector3f_normalize_with_default(projected_sample, Eigen::Vector3f(0.f, 1.f, 0.f));

                        // Tell the psmove service about the new magnetometer settings
                        {                            
                            RequestPtr request(new PSMoveProtocol::Request());
                            request->set_type(PSMoveProtocol::Request_RequestType_SET_MAGNETOMETER_CALIBRATION);

                            PSMoveProtocol::Request_RequestSetMagnetometerCalibration *calibration=
                                request->mutable_set_magnetometer_calibration_request();

                            calibration->set_controller_id(m_controllerView->GetControllerID());

                            write_calibration_parameter(m_sampleFitEllipsoid.center, calibration->mutable_ellipse_center());
                            write_calibration_parameter(m_sampleFitEllipsoid.extents, calibration->mutable_ellipse_extents());
                            write_calibration_parameter(m_sampleFitEllipsoid.basis.col(0), calibration->mutable_ellipse_basis_x());
                            write_calibration_parameter(m_sampleFitEllipsoid.basis.col(1), calibration->mutable_ellipse_basis_y());
                            write_calibration_parameter(m_sampleFitEllipsoid.basis.col(2), calibration->mutable_ellipse_basis_z());
                            calibration->set_ellipse_fit_error(m_sampleFitEllipsoid.error);

                            write_calibration_parameter(projected_sample, calibration->mutable_magnetometer_identity());

                            m_app->registerCallback(
                                ClientPSMoveAPI::send_opaque_request(&request), 
                                AppStage_MagnetometerCalibration::handle_set_magnetometer_calibration, this);
                        }

                        // Wait for the response
                        m_menuState= AppStage_MagnetometerCalibration::waitForSetCalibrationResponse;
                    }
                }
            }
            else
            {
                m_bIsStable= false;
                m_menuState= AppStage_MagnetometerCalibration::waitForGravityAlignment;
            }
        } break;
    case eCalibrationMenuState::waitForSetCalibrationResponse:
        {
        } break;
    case eCalibrationMenuState::failedSetCalibration:
        {
        } break;
    case eCalibrationMenuState::complete:
        {
        } break;
    case eCalibrationMenuState::pendingExit:
        {
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_MagnetometerCalibration::render()
{
    const float modelScale = 18.f;
    glm::mat4 scaleAndRotateModelX90= 
        glm::rotate(
            glm::scale(glm::mat4(1.f), glm::vec3(modelScale, modelScale, modelScale)),
            90.f, glm::vec3(1.f, 0.f, 0.f));  
    
    PSMoveIntVector3 rawSampleExtents = (m_maxSampleExtent - m_minSampleExtent).unsafe_divide(2);

    glm::vec3 boxMin = psmove_float_vector3_to_glm_vec3(m_minSampleExtent.castToFloatVector3());
    glm::vec3 boxMax = psmove_float_vector3_to_glm_vec3(m_maxSampleExtent.castToFloatVector3());
    glm::vec3 boxCenter = (boxMax + boxMin) * 0.5f;
    glm::vec3 boxExtents = (boxMax - boxMin) * 0.5f;

    glm::mat4 recenterMatrix = 
        glm::translate(glm::mat4(1.f), -eigen_vector3f_to_glm_vec3(m_sampleFitEllipsoid.center));

    switch (m_menuState)
    {
    case eCalibrationMenuState::waitingForStreamStartResponse:
        {
        } break;
    case eCalibrationMenuState::failedStreamStart:
    case eCalibrationMenuState::failedBadCalibration:
        {
        } break;
    case eCalibrationMenuState::measureBExtents:
        {

            float r= clampf01(static_cast<float>(m_led_color_r) / 255.f);
            float g= clampf01(static_cast<float>(m_led_color_g) / 255.f);
            float basis= clampf01(static_cast<float>(m_led_color_b) / 255.f);

            // Draw the psmove model in the middle
            drawPSMoveModel(scaleAndRotateModelX90, glm::vec3(r, g, basis));

            // Draw the sample point cloud around the origin
            drawPointCloud(
                recenterMatrix,
                glm::vec3(1.f, 1.f, 1.f),
                reinterpret_cast<float *>(&m_alignedSamples->magnetometerEigenSamples[0]),
                m_sampleCount);

            // Draw the sample bounding box
            // Label the min and max corners with the min and max magnetometer readings
            drawTransformedBox(recenterMatrix, boxMin, boxMax, glm::vec3(1.f, 1.f, 1.f));
            drawTextAtWorldPosition(recenterMatrix, boxMin, "%d,%d,%d",
                                    m_minSampleExtent.i, m_minSampleExtent.j, m_minSampleExtent.k);
            drawTextAtWorldPosition(recenterMatrix, boxMax, "%d,%d,%d",
                                    m_maxSampleExtent.i, m_maxSampleExtent.j, m_maxSampleExtent.k);

            // Draw and label the extent axes
            drawTransformedAxes(glm::mat4(1.f), boxExtents.x, boxExtents.y, boxExtents.z);
            drawTextAtWorldPosition(glm::mat4(1.f), glm::vec3(boxExtents.x, 0.f, 0.f), "%d", rawSampleExtents.i);
            drawTextAtWorldPosition(glm::mat4(1.f), glm::vec3(0.f, boxExtents.y, 0.f), "%d", rawSampleExtents.j);
            drawTextAtWorldPosition(glm::mat4(1.f), glm::vec3(0.f, 0.f, boxExtents.z), "%d", rawSampleExtents.k);

            // Draw the best fit ellipsoid
            {
                glm::mat3 basis = eigen_matrix3f_to_glm_mat3(m_sampleFitEllipsoid.basis);
                glm::vec3 center = eigen_vector3f_to_glm_vec3(m_sampleFitEllipsoid.center);
                glm::vec3 extents = eigen_vector3f_to_glm_vec3(m_sampleFitEllipsoid.extents);

                drawEllipsoid(
                    recenterMatrix,
                    glm::vec3(0.f, 0.4f, 1.f),
                    basis, center, extents);
                drawTextAtWorldPosition(
                    recenterMatrix,
                    center - basis[0]*extents.x,
                    "E:%.1f", m_sampleFitEllipsoid.error);
            }

            // Draw the current magnetometer direction
            {
                glm::vec3 m_start= boxCenter;
                glm::vec3 m_end= psmove_float_vector3_to_glm_vec3(m_lastMagnetometer.castToFloatVector3());

                drawArrow(recenterMatrix, m_start, m_end, 0.1f, glm::vec3(1.f, 0.f, 0.f));
                drawTextAtWorldPosition(recenterMatrix, m_end, "M");
            }
        } break;
    case eCalibrationMenuState::waitForGravityAlignment:
        {
            drawPSMoveModel(scaleAndRotateModelX90, glm::vec3(1.f, 1.f, 1.f));

            // Draw the current direction of gravity
            {
                const float renderScale = 200.f;
                glm::mat4 renderScaleMatrix = 
                    glm::scale(glm::mat4(1.f), glm::vec3(renderScale, renderScale, renderScale));
                glm::vec3 g= psmove_float_vector3_to_glm_vec3(m_lastAccelerometer);

                drawArrow(
                    renderScaleMatrix,
                    glm::vec3(), g, 
                    0.1f, 
                    glm::vec3(0.f, 1.f, 0.f));
                drawTextAtWorldPosition(renderScaleMatrix, g, "G");
            }
        } break;
    case eCalibrationMenuState::measureBDirection:
        {
            drawPSMoveModel(scaleAndRotateModelX90, glm::vec3(1.f, 1.f, 1.f));

            // Draw the current magnetometer direction
            {
                glm::vec3 m_start = boxCenter;
                glm::vec3 m_end = psmove_float_vector3_to_glm_vec3(m_lastMagnetometer.castToFloatVector3());

                drawArrow(recenterMatrix, m_start, m_end, 0.1f, glm::vec3(1.f, 0.f, 0.f));
                drawTextAtWorldPosition(recenterMatrix, m_end, "M");
            }

        } break;
    case eCalibrationMenuState::waitForSetCalibrationResponse:
        {
        } break;
    case eCalibrationMenuState::failedSetCalibration:
        {
        } break;
    case eCalibrationMenuState::complete:
        {
            // Get the orientation of the controller in world space (OpenGL Coordinate System)            
            glm::quat q= psmove_quaternion_to_glm_quat(m_controllerView->GetPSMoveView().GetOrientation());
            glm::mat4 worldSpaceOrientation= glm::mat4_cast(q);
            glm::mat4 worldTransform = glm::scale(worldSpaceOrientation, glm::vec3(modelScale, modelScale, modelScale));

            drawPSMoveModel(worldTransform, glm::vec3(1.f, 1.f, 1.f));
            drawTransformedAxes(glm::mat4(1.f), 200.f);
        } break;
    case eCalibrationMenuState::pendingExit:
        {
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_MagnetometerCalibration::renderUI()
{
    const float k_panel_width= 500;
    const char *k_window_title= "Controller Settings";
    const ImGuiWindowFlags window_flags = 
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize | 
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case eCalibrationMenuState::waitingForStreamStartResponse:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Waiting for controller stream to start...");

            ImGui::End();
        } break;
    case eCalibrationMenuState::failedStreamStart:
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
    case eCalibrationMenuState::failedBadCalibration:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::TextWrapped(
                "Bad controller hardware calibration!\n" \
                "Try un-pairing and re-pairing the controller.");

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
    case eCalibrationMenuState::measureBExtents:
        {
            {
                ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
                ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
                ImGui::Begin(k_window_title, nullptr, window_flags);

                if (m_sampleCount < k_max_magnetometer_samples)
                {
                    ImGui::TextWrapped(
                        "Calibrating Controller ID #%d\n" \
                        "[Step 1 of 2: Measuring extents of the magnetometer]\n" \
                        "Rotate the controller in all directions.", m_controllerView->GetControllerID());
                }
                else
                {
                    ImGui::TextWrapped(
                        "Calibrating Controller ID #%d\n" \
                        "[Step 1 of 2: Measuring extents of the magnetometer - Complete!]\n" \
                        "Press OK to continue", m_controllerView->GetControllerID());


                }

                if (m_samplePercentage < 100)
                {
                    ImGui::ProgressBar(static_cast<float>(m_samplePercentage) / 100.f, ImVec2(250, 20));
                }
                else
                {
                    if (ImGui::Button("Ok"))
                    {
                        ClientPSMoveAPI::set_led_color(m_controllerView, 0, 0, 0);
                        m_menuState = waitForGravityAlignment;
                    }
                    ImGui::SameLine();
                }

                if (ImGui::Button("Cancel"))
                {
                    request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
                }

                ImGui::End();
            }

            {
                ImGui::SetNextWindowPos(ImVec2(10.f, 450.f));
                ImGui::SetNextWindowSize(ImVec2(170.f, 80.f));
                ImGui::Begin("Ellipse Fitting Mode", nullptr, window_flags);

                if (ImGui::RadioButton("Bounds Fitting", &m_ellipseFitMethod, _ellipse_fit_method_box))
                {
                    // Refit to a box
                    eigen_alignment_fit_bounding_box_ellipsoid(
                        m_alignedSamples->magnetometerEigenSamples, m_sampleCount, m_sampleFitEllipsoid);
                }

                if (ImGui::RadioButton("Min Volume Fitting", &m_ellipseFitMethod, _ellipse_fit_method_min_volume))
                {
                    // Re-fit using min bounds
                    eigen_alignment_fit_min_volume_ellipsoid(
                        m_alignedSamples->magnetometerEigenSamples, m_sampleCount, 0.0001f, m_sampleFitEllipsoid);
                }

                ImGui::End();
            }
        } break;
    case eCalibrationMenuState::waitForGravityAlignment:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x/2.f - k_panel_width/2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 200));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::TextWrapped(
                "[Step 2 of 2: Measuring reference magnetic field direction]\n" \
                "Stand the controller on a level surface with the Move button facing you.\n" \
                "This will be the default orientation of the move controller.\n" \
                "Measurement will start once the controller is aligned with gravity and stable.");

            if (m_bIsStable)
            {
                std::chrono::time_point<std::chrono::high_resolution_clock> now= std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::milli> stableDuration = now - m_stableStartTime;
                float fraction = static_cast<float>(stableDuration.count() / k_stabilize_wait_time_ms);

                ImGui::ProgressBar(fraction, ImVec2(250, 20));
                ImGui::Spacing();
            }
            else
            {
                ImGui::Text("Move Destabilized! Waiting for stabilization..");
            }

            if (ImGui::Button("Cancel"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::measureBDirection:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x/2.f - k_panel_width/2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 200));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::TextWrapped(
                "[Step 2 of 2: Measuring reference magnetic field direction]\n" \
                "Stand the controller on a level surface with the Move button facing you.\n"
                "This will be the default orientation of the move controller.\n"
                "Measurement will start once the controller is aligned with gravity and stable.");

            ImGui::ProgressBar(
                static_cast<float>(m_identityPoseSampleCount) / static_cast<float>(k_desired_magnetometer_sample_count), 
                ImVec2(250, 20));
            ImGui::Spacing();

            if (ImGui::Button("Cancel"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::waitForSetCalibrationResponse:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Sending final calibration to server...");

            ImGui::End();
        } break;
    case eCalibrationMenuState::failedSetCalibration:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Failed to set calibration!");

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
    case eCalibrationMenuState::complete:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 80));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            if (m_bBypassCalibration)
            {
                ImGui::Text("Testing Calibration of Controller ID #%d", m_controllerView->GetControllerID());
            }
            else
            {
                ImGui::Text("Calibration of Controller ID #%d complete!", m_controllerView->GetControllerID());
            }

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
    case eCalibrationMenuState::pendingExit:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Waiting for controller stream to stop...");

            ImGui::End();
        } break;
    default:
        assert(0 && "unreachable");
    }
}

//-- private methods -----
void AppStage_MagnetometerCalibration::handle_acquire_controller(
    ClientPSMoveAPI::eClientPSMoveResultCode resultCode,
    const ClientPSMoveAPI::t_request_id request_id, 
    ClientPSMoveAPI::t_response_handle opaque_response_handle,
    void *userdata)
{
    AppStage_MagnetometerCalibration *thisPtr= reinterpret_cast<AppStage_MagnetometerCalibration *>(userdata);

    if (resultCode == ClientPSMoveAPI::_clientPSMoveResultCode_ok)
    {
        thisPtr->m_isControllerStreamActive= true;
        thisPtr->m_lastControllerSeqNum= -1;
        // Wait for the first controller packet to show up...
    }
    else
    {
        thisPtr->m_menuState= AppStage_MagnetometerCalibration::failedStreamStart;
    }
}

void AppStage_MagnetometerCalibration::request_exit_to_app_stage(const char *app_stage_name)
{
    if (m_pendingAppStage == nullptr)
    {
        if (m_isControllerStreamActive)
        {
            m_pendingAppStage= app_stage_name;
            ClientPSMoveAPI::set_led_color(m_controllerView, 0, 0, 0);

            m_app->registerCallback(
                ClientPSMoveAPI::stop_controller_data_stream(m_controllerView), 
                &AppStage_MagnetometerCalibration::handle_release_controller, this);
        }
        else
        {
            m_app->setAppStage(app_stage_name);
        }
    }
}

void AppStage_MagnetometerCalibration::handle_release_controller(
    ClientPSMoveAPI::eClientPSMoveResultCode resultCode,
    const ClientPSMoveAPI::t_request_id request_id, 
    ClientPSMoveAPI::t_response_handle opaque_response_handle,
    void *userdata)
{
    AppStage_MagnetometerCalibration *thisPtr= reinterpret_cast<AppStage_MagnetometerCalibration *>(userdata);

    if (resultCode != ClientPSMoveAPI::_clientPSMoveResultCode_ok)
    {
        Log_ERROR("AppStage_MagnetometerCalibration", "Failed to release controller on server!");
    }

    thisPtr->m_isControllerStreamActive= false;
    thisPtr->m_pendingAppStage = nullptr;
    thisPtr->m_app->setAppStage(AppStage_ControllerSettings::APP_STAGE_NAME);
}

void AppStage_MagnetometerCalibration::handle_set_magnetometer_calibration(
    ClientPSMoveAPI::eClientPSMoveResultCode resultCode,
    const ClientPSMoveAPI::t_request_id request_id, 
    ClientPSMoveAPI::t_response_handle opaque_response_handle,
    void *userdata)
{
    AppStage_MagnetometerCalibration *thisPtr= reinterpret_cast<AppStage_MagnetometerCalibration *>(userdata);

    if (resultCode == ClientPSMoveAPI::_clientPSMoveResultCode_ok)
    {
        thisPtr->m_app->getOrbitCamera()->resetOrientation();
        thisPtr->m_menuState= AppStage_MagnetometerCalibration::complete;
    }
    else
    {
        thisPtr->m_menuState= AppStage_MagnetometerCalibration::failedSetCalibration;
    }
}

//-- private methods -----
static void expandMagnetometerBounds(
    const PSMoveIntVector3 &sample,
    PSMoveIntVector3 &minSampleExtents,
    PSMoveIntVector3 &maxSampleExtents)
{
    minSampleExtents= PSMoveIntVector3::min(minSampleExtents, sample);
    maxSampleExtents= PSMoveIntVector3::max(maxSampleExtents, sample);
}

static int computeMagnetometerCalibrationMinRange(
    const PSMoveIntVector3 &minSampleExtents,
    const PSMoveIntVector3 &maxSampleExtents)
{
    PSMoveIntVector3 extents= maxSampleExtents - minSampleExtents;

    return extents.minValue();
}

static int computeMagnetometerCalibrationMaxRange(
    const PSMoveIntVector3 &minSampleExtents,
    const PSMoveIntVector3 &maxSampleExtents)
{
    PSMoveIntVector3 extents= maxSampleExtents - minSampleExtents;

    return extents.maxValue();
}

PSMoveFloatVector3 computeNormalizedMagnetometerVector(
    const PSMoveIntVector3 &sample,
    const PSMoveIntVector3 &minSampleExtents,
    const PSMoveIntVector3 &maxSampleExtents)
{
    PSMoveFloatVector3 range = (maxSampleExtents - minSampleExtents).castToFloatVector3();
    PSMoveFloatVector3 offset = (sample - minSampleExtents).castToFloatVector3();

    // 2*(raw-move->magnetometer_min)/(move->magnetometer_max - move->magnetometer_min) - <1,1,1>
    PSMoveFloatVector3 result= offset.safe_divide(range, *k_psmove_float_vector3_zero)*2.f - *k_psmove_float_vector3_one;

    // The magnetometer y-axis is flipped compared to the accelerometer and gyro.
    // Flip it back around to get it into the same space.
    result.j = -result.j;

    result.normalize_with_default(*k_psmove_float_vector3_zero);

    return result;
}

static bool
isMoveStableAndAlignedWithGravity(
    ClientControllerView *m_controllerView)
{
    const float k_cosine_10_degrees = 0.984808f;

    // Get the direction the gravity vector should be pointing 
    // while the controller is in cradle pose.
    const PSMoveFloatVector3 &gravity= m_controllerView->GetPSMoveView().GetIdentityGravityCalibrationDirection();
    glm::vec3 k_identity_gravity_vector= psmove_float_vector3_to_glm_vec3(gravity);

    // Get the current acceleration reading from the controller
    const PSMoveFloatVector3 &acceleration= m_controllerView->GetPSMoveView().GetRawSensorData().Accelerometer;
    glm::vec3 acceleration_direction= psmove_float_vector3_to_glm_vec3(acceleration);
    const float acceleration_magnitude= glm_vec3_normalize_with_default(acceleration_direction, glm::vec3());

    const bool isOk =
        is_nearly_equal(1.f, acceleration_magnitude, 0.1f) &&
        glm::dot(k_identity_gravity_vector, acceleration_direction) >= k_cosine_10_degrees;

    return isOk;
}

static void
write_calibration_parameter(
    const Eigen::Vector3f &in_vector,
    PSMoveProtocol::FloatVector *out_vector)
{
    out_vector->set_i(in_vector.x());
    out_vector->set_j(in_vector.y());
    out_vector->set_k(in_vector.z());
}

//-- inludes -----
#include "AppStage_AccelerometerCalibration.h"
#include "AppStage_ControllerSettings.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "ClientControllerView.h"
#include "GeometryUtility.h"
#include "Logger.h"
#include "MathAlignment.h"
#include "MathGLM.h"
#include "MathEigen.h"
#include "MathUtility.h"

#include "PSMoveProtocolInterface.h"
#include "Renderer.h"
#include "UIConstants.h"

#include "SDL_keycode.h"

#include <imgui.h>

#include <algorithm>

//-- statics ----
const char *AppStage_AccelerometerCalibration::APP_STAGE_NAME = "AcceleromterCalibration";

//-- constants -----
static const double k_stabilize_wait_time_ms = 1000.f;
static const int k_max_accelerometer_samples = 500;

static const float k_min_sample_distance = 1000.f;
static const float k_min_sample_distance_sq = k_min_sample_distance*k_min_sample_distance;

//-- definitions -----
struct AccelerometerPoseSamples
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PSMoveFloatVector3 psmove_accelerometer_samples[k_max_accelerometer_samples];
    Eigen::Vector3f eigen_accelerometer_samples[k_max_accelerometer_samples];
    Eigen::Vector3f avg_accelerometer_sample;
    int sample_count;

    void clear()
    {
        avg_accelerometer_sample = Eigen::Vector3f::Zero();
        sample_count= 0;
    }
};

//-- private methods -----
static void request_set_accelerometer_calibration(
    const int controller_id,
    const EigenFitEllipsoid *ellipsoid);
static void expandAccelerometerBounds(
    const PSMoveIntVector3 &sample, PSMoveIntVector3 &minSampleExtents, PSMoveIntVector3 &maxSampleExtents);
static void write_calibration_parameter(const Eigen::Vector3f &in_vector, PSMoveProtocol::FloatVector *out_vector);
static glm::mat4 computeControllerPoseTransform(AppStage_AccelerometerCalibration::eMeasurementPose PoseID);

//-- public methods -----
AppStage_AccelerometerCalibration::AppStage_AccelerometerCalibration(App *app)
    : AppStage(app)
    , m_menuState(AppStage_AccelerometerCalibration::inactive)
    , m_bBypassCalibration(false)
    , m_controllerView(nullptr)
    , m_isControllerStreamActive(false)
    , m_lastControllerSeqNum(-1)
    , m_minSampleExtent()
    , m_maxSampleExtent()
    , m_lastRawAccelerometer()
    , m_poseSamples(new AccelerometerPoseSamples[AppStage_AccelerometerCalibration::k_measurement_pose_count])
    , m_gravitySamples(new AccelerometerPoseSamples)
    , m_sphereSamples(new AccelerometerPoseSamples)
    , m_currentPoseID(AppStage_AccelerometerCalibration::eMeasurementPose::faceUp)
    , m_sampleFitEllipsoid(new EigenFitEllipsoid)
{
}

AppStage_AccelerometerCalibration::~AppStage_AccelerometerCalibration()
{
    delete[] m_poseSamples;
    delete m_gravitySamples;
    delete m_sphereSamples;
    delete m_sampleFitEllipsoid;
}

void AppStage_AccelerometerCalibration::enter()
{
    const AppStage_ControllerSettings *controllerSettings =
        m_app->getAppStage<AppStage_ControllerSettings>();
    const AppStage_ControllerSettings::ControllerInfo *controllerInfo =
        controllerSettings->getSelectedControllerInfo();

    // Reset the menu state
    m_app->setCameraType(_cameraOrbit);
    m_app->getOrbitCamera()->resetOrientation();
    m_app->getOrbitCamera()->setCameraOrbitRadius(1000.f); // zoom out to see the magnetometer data at scale

    m_menuState = eCalibrationMenuState::waitingForStreamStartResponse;

    // Reset all of the sampling state
    for (int pose_index = 0; pose_index < k_measurement_pose_count; ++pose_index)
    {
        m_poseSamples[pose_index].clear();
    }
    m_currentPoseID = eMeasurementPose::faceUp;

    m_gravitySamples->clear();
    m_sphereSamples->clear();

    m_calibrationMethod= eCalibrationMethod::boundingBox;

    // Initialize the controller state
    assert(controllerInfo->ControllerID != -1);
    assert(m_controllerView == nullptr);
    m_controllerView = ClientPSMoveAPI::allocate_controller_view(controllerInfo->ControllerID);

    m_minSampleExtent = *k_psmove_int_vector3_zero;
    m_maxSampleExtent = *k_psmove_int_vector3_zero;
    m_lastRawAccelerometer = *k_psmove_int_vector3_zero;
    m_lastControllerSeqNum = -1;

    // Start streaming in controller data
    assert(!m_isControllerStreamActive);
    ClientPSMoveAPI::register_callback(
        ClientPSMoveAPI::start_controller_data_stream(
            m_controllerView, 
            ClientPSMoveAPI::includeRawSensorData | ClientPSMoveAPI::includeCalibratedSensorData),
        &AppStage_AccelerometerCalibration::handle_acquire_controller, this);
}

void AppStage_AccelerometerCalibration::exit()
{
    assert(m_controllerView != nullptr);
    ClientPSMoveAPI::free_controller_view(m_controllerView);
    m_controllerView = nullptr;
    m_menuState = eCalibrationMenuState::inactive;

    // Reset the orbit camera back to default orientation and scale
    m_app->getOrbitCamera()->reset();
}

void AppStage_AccelerometerCalibration::update()
{
    bool bControllerDataUpdatedThisFrame = false;

    if (m_isControllerStreamActive && m_controllerView->GetOutputSequenceNum() != m_lastControllerSeqNum)
    {
        const PSDualShock4RawSensorData &rawSensorData =
            m_controllerView->GetPSDualShock4View().GetRawSensorData();
        const PSDualShock4CalibratedSensorData &calibratedSensorData =
            m_controllerView->GetPSDualShock4View().GetCalibratedSensorData();

        m_lastRawAccelerometer = rawSensorData.Accelerometer;
        m_lastCalibratedAccelerometer = calibratedSensorData.Accelerometer;
        m_lastControllerSeqNum = m_controllerView->GetOutputSequenceNum();
        bControllerDataUpdatedThisFrame = true;
    }

    switch (m_menuState)
    {
    case eCalibrationMenuState::waitingForStreamStartResponse:
        {
            if (bControllerDataUpdatedThisFrame)
            {
                if (m_bBypassCalibration)
                {
                    m_app->getOrbitCamera()->resetOrientation();
                    m_menuState = AppStage_AccelerometerCalibration::test;
                }
                else
                {
                    m_menuState = AppStage_AccelerometerCalibration::selectMethod;
                }
            }
        } break;
    case eCalibrationMenuState::failedStreamStart:
    case eCalibrationMenuState::selectMethod:
    case eCalibrationMenuState::placeController:
    case eCalibrationMenuState::holdAndSpinController:
        {
        } break;
    case eCalibrationMenuState::measureDirection:
        {
            AccelerometerPoseSamples &poseSamples = m_poseSamples[m_currentPoseID];

            if (bControllerDataUpdatedThisFrame && poseSamples.sample_count < k_max_accelerometer_samples)
            {
                // Get the last accelerometer sample we've received for the controller
                PSMoveFloatVector3 psmove_sample = m_lastRawAccelerometer.castToFloatVector3();
                Eigen::Vector3f eigen_sample = psmove_float_vector3_to_eigen_vector3(psmove_sample);

                // Grow the measurement extents bounding box
                expandAccelerometerBounds(m_lastRawAccelerometer, m_minSampleExtent, m_maxSampleExtent);

                // Store the new sample
                poseSamples.psmove_accelerometer_samples[poseSamples.sample_count] = psmove_sample;
                poseSamples.eigen_accelerometer_samples[poseSamples.sample_count] = eigen_sample;
                poseSamples.avg_accelerometer_sample += eigen_sample;
                ++poseSamples.sample_count;

                // See if we filled all of the samples for this pose
                if (poseSamples.sample_count >= k_max_accelerometer_samples)
                {
                    // Compute the average gravity value in this pose.
                    // This assumes that the acceleration noise has a Gaussian distribution.
                    poseSamples.avg_accelerometer_sample /= static_cast<float>(k_max_accelerometer_samples);

                    // See if we completed the last pose
                    if (m_currentPoseID >= eMeasurementPose::leanBackward)
                    {
                        Eigen::Vector3f avg_pose_samples[k_measurement_pose_count];

                        Eigen::Vector3f boxMin = psmove_float_vector3_to_eigen_vector3(m_minSampleExtent.castToFloatVector3());
                        Eigen::Vector3f boxMax = psmove_float_vector3_to_eigen_vector3(m_maxSampleExtent.castToFloatVector3());
                        Eigen::Vector3f boxCenter = (boxMax + boxMin) * 0.5f;

                        PSMoveIntVector3 rawSampleExtents = (m_maxSampleExtent - m_minSampleExtent).unsafe_divide(2);
                        const float maxSampleExtent = static_cast<float>(rawSampleExtents.maxValue());

                        // Copy all of the average pose samples into one array
                        // re-centered and scaled down to keep the optimization stable
                        for (int pose_index = 0; pose_index < k_measurement_pose_count; ++pose_index)
                        {
                            avg_pose_samples[pose_index] = 
                                (m_poseSamples[pose_index].avg_accelerometer_sample - boxCenter) / maxSampleExtent;
                        }

                        // Compute an ellipsoid that best fits the 
                        eigen_alignment_fit_min_volume_ellipsoid(
                            avg_pose_samples, k_measurement_pose_count, 0.0001f, *m_sampleFitEllipsoid);

                        // Transform the fit ellipsoid back into the space of the original data
                        m_sampleFitEllipsoid->center += boxCenter;
                        m_sampleFitEllipsoid->extents.x() = m_sampleFitEllipsoid->extents.x() * maxSampleExtent;
                        m_sampleFitEllipsoid->extents.y() = m_sampleFitEllipsoid->extents.y() * maxSampleExtent;
                        m_sampleFitEllipsoid->extents.z() = m_sampleFitEllipsoid->extents.z() * maxSampleExtent;
                        m_sampleFitEllipsoid->error *= maxSampleExtent;

                        // Tell the service what the new calibration constraints are
                        request_set_accelerometer_calibration(
                            m_controllerView->GetControllerID(),
                            m_sampleFitEllipsoid);
                    }

                    m_menuState = AppStage_AccelerometerCalibration::measureComplete;
                }
            }
        } break;
    case eCalibrationMenuState::measureSphere:
        {
            if (bControllerDataUpdatedThisFrame && m_sphereSamples->sample_count < k_max_accelerometer_samples)
            {
                // Get the last accelerometer sample we've received for the controller
                PSMoveFloatVector3 psmove_sample = m_lastRawAccelerometer.castToFloatVector3();
                Eigen::Vector3f eigen_sample = psmove_float_vector3_to_eigen_vector3(psmove_sample);

                // Make sure this sample isn't too close to another sample
                bool bTooClose= false;
                for (int sampleIndex= m_sphereSamples->sample_count-1; sampleIndex >= 0; --sampleIndex)
                {
                    const Eigen::Vector3f diff= eigen_sample - m_sphereSamples->eigen_accelerometer_samples[sampleIndex];
                    const float distanceSquared= diff.squaredNorm();

                    if (distanceSquared < k_min_sample_distance_sq)
                    {
                        bTooClose= true;
                        break;
                    }
                }

                if (!bTooClose)
                {
                    // Grow the measurement extents bounding box
                    expandAccelerometerBounds(m_lastRawAccelerometer, m_minSampleExtent, m_maxSampleExtent);

                    // Store the new sample
                    m_sphereSamples->psmove_accelerometer_samples[m_sphereSamples->sample_count] = psmove_sample;
                    m_sphereSamples->eigen_accelerometer_samples[m_sphereSamples->sample_count] = eigen_sample;
                    m_sphereSamples->avg_accelerometer_sample += eigen_sample;
                    ++m_sphereSamples->sample_count;

                    // See if we filled all of the samples for this pose
                    if (m_sphereSamples->sample_count >= k_max_accelerometer_samples)
                    {
                        // Compute the average gravity value in this pose.
                        // This assumes that the acceleration noise has a Gaussian distribution.
                        m_sphereSamples->avg_accelerometer_sample /= static_cast<float>(k_max_accelerometer_samples);

                        // Compute a best fit ellipsoid based on the bounding box 
                        eigen_alignment_fit_bounding_box_ellipsoid(
                            m_sphereSamples->eigen_accelerometer_samples, 
                            m_sphereSamples->sample_count, 
                            *m_sampleFitEllipsoid);

                        // Instruct the use to set the controller down to measure gravity
                        m_menuState = AppStage_AccelerometerCalibration::placeController;
                    }
                }
            }
        } break;
    case eCalibrationMenuState::measureGravity:
        {
            if (bControllerDataUpdatedThisFrame && m_gravitySamples->sample_count < k_max_accelerometer_samples)
            {
                // Get the last accelerometer sample we've received for the controller
                PSMoveFloatVector3 psmove_sample = m_lastRawAccelerometer.castToFloatVector3();
                Eigen::Vector3f eigen_sample = psmove_float_vector3_to_eigen_vector3(psmove_sample);

                // Store the new sample
                m_gravitySamples->psmove_accelerometer_samples[m_gravitySamples->sample_count] = psmove_sample;
                m_gravitySamples->eigen_accelerometer_samples[m_gravitySamples->sample_count] = eigen_sample;
                m_gravitySamples->avg_accelerometer_sample += eigen_sample;
                ++m_gravitySamples->sample_count;

                // See if we filled all of the samples for this pose
                if (m_gravitySamples->sample_count >= k_max_accelerometer_samples)
                {
                    // Compute the average gravity value in this pose.
                    // This assumes that the acceleration noise has a Gaussian distribution.
                    m_gravitySamples->avg_accelerometer_sample /= static_cast<float>(k_max_accelerometer_samples);

                    // The average accelerometer reading in the face up pose becomes the "identity" gravity direction
                    Eigen::Vector3f eigen_indentity_gravity = 
                        m_gravitySamples->avg_accelerometer_sample - m_sampleFitEllipsoid->center;
                    eigen_vector3f_normalize_with_default(eigen_indentity_gravity, Eigen::Vector3f(0.f, 1.f, 0.f));                        

                    // Tell the service what the new calibration constraints are
                    //request_set_accelerometer_calibration(
                    //    m_controllerView->GetControllerID(),
                    //    m_sampleFitEllipsoid);

                    m_menuState = AppStage_AccelerometerCalibration::measureComplete;
                }
            }
        } break;
    case eCalibrationMenuState::measureComplete:
    case eCalibrationMenuState::verifyCalibration:
    case eCalibrationMenuState::test:
        {
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_AccelerometerCalibration::render()
{
    PSMoveIntVector3 rawSampleExtents = (m_maxSampleExtent - m_minSampleExtent).unsafe_divide(2);
    const float maxSampleExtent= static_cast<float>(rawSampleExtents.maxValue());

    glm::vec3 boxMin = psmove_float_vector3_to_glm_vec3(m_minSampleExtent.castToFloatVector3());
    glm::vec3 boxMax = psmove_float_vector3_to_glm_vec3(m_maxSampleExtent.castToFloatVector3());
    glm::vec3 boxCenter = (boxMax + boxMin) * 0.5f;
    glm::vec3 boxExtents = (boxMax - boxMin) * 0.5f;

    const float desiredMaxSampleExtent = 200.f;
    const float sampleScale = safe_divide_with_default(desiredMaxSampleExtent, maxSampleExtent, 1.f);
    glm::mat4 recenterMatrix =         
        glm::translate(
            glm::scale(glm::mat4(1.f), glm::vec3(sampleScale, sampleScale, sampleScale)),
            -boxCenter);

    switch (m_menuState)
    {
    case eCalibrationMenuState::waitingForStreamStartResponse:
    case eCalibrationMenuState::failedStreamStart:
    case eCalibrationMenuState::selectMethod:
        {
        } break;
    case eCalibrationMenuState::holdAndSpinController:
    case eCalibrationMenuState::placeController:
        {
            // Draw the controller model in the pose we want the user place it in
            glm::mat4 controllerTransform = computeControllerPoseTransform(m_currentPoseID);
            drawPSDualShock4Model(controllerTransform, glm::vec3(1.f, 1.f, 1.f));
        } break;
    case eCalibrationMenuState::measureDirection:
    case eCalibrationMenuState::measureGravity:
    case eCalibrationMenuState::measureSphere:
    case eCalibrationMenuState::measureComplete:
        {
            // Draw the controller in the middle
            glm::mat4 controllerTransform = computeControllerPoseTransform(eMeasurementPose::identity);
            drawPSDualShock4Model(controllerTransform, glm::vec3(1.f, 1.f, 1.f));

            // Get the sample list to draw based on the calibration method
            float *poseSamples= nullptr;
            int sampleCount= 0;
            switch(m_calibrationMethod)
            {
            case eCalibrationMethod::boundingBox:
                if (m_menuState == eCalibrationMenuState::measureSphere)
                {
                    poseSamples= reinterpret_cast<float *>(&m_sphereSamples->psmove_accelerometer_samples[0]);
                    sampleCount= m_sphereSamples->sample_count;
                }
                else if (m_menuState == eCalibrationMenuState::measureGravity)
                {
                    poseSamples= reinterpret_cast<float *>(&m_gravitySamples->psmove_accelerometer_samples[0]);
                    sampleCount= m_gravitySamples->sample_count;
                }
                break;
            case eCalibrationMethod::minVolumeFit:
                poseSamples= reinterpret_cast<float *>(&m_poseSamples[m_currentPoseID].psmove_accelerometer_samples[0]);
                sampleCount= m_poseSamples[m_currentPoseID].sample_count;
                break;
            }

            // Draw the sample point cloud around the origin
            drawPointCloud(recenterMatrix, glm::vec3(1.f, 1.f, 1.f), poseSamples, sampleCount);

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

            // Draw the current raw accelerometer direction
            {
                glm::vec3 m_start = boxCenter;
                glm::vec3 m_end = psmove_float_vector3_to_glm_vec3(m_lastRawAccelerometer.castToFloatVector3());

                drawArrow(recenterMatrix, m_start, m_end, 0.1f, glm::vec3(1.f, 0.f, 0.f));
                drawTextAtWorldPosition(recenterMatrix, m_end, "A");
            }
        } break;
    case eCalibrationMenuState::verifyCalibration:
        {
            // Draw the controller in the middle
            glm::mat4 controllerTransform= computeControllerPoseTransform(eMeasurementPose::identity);
            drawPSDualShock4Model(controllerTransform, glm::vec3(1.f, 1.f, 1.f));

            // Draw all of the recorded sample point clouds
            for (int pose_id = eMeasurementPose::faceUp; pose_id < eMeasurementPose::k_measurement_pose_count; ++pose_id)
            {
                drawPointCloud(
                    recenterMatrix,
                    glm::vec3(1.f, 1.f, 1.f),
                    reinterpret_cast<float *>(&m_poseSamples[pose_id].psmove_accelerometer_samples[0]),
                    m_poseSamples[pose_id].sample_count);
            }

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

            // Draw the current raw accelerometer direction
            {
                glm::vec3 m_start = boxCenter;
                glm::vec3 m_end = psmove_float_vector3_to_glm_vec3(m_lastRawAccelerometer.castToFloatVector3());

                drawArrow(recenterMatrix, m_start, m_end, 0.1f, glm::vec3(1.f, 0.f, 0.f));
                drawTextAtWorldPosition(recenterMatrix, m_end, "A");
            }

            // Draw the best fit ellipsoid, once available
            {
                glm::mat3 basis = eigen_matrix3f_to_glm_mat3(m_sampleFitEllipsoid->basis);
                glm::vec3 center = eigen_vector3f_to_glm_vec3(m_sampleFitEllipsoid->center);
                glm::vec3 extents = eigen_vector3f_to_glm_vec3(m_sampleFitEllipsoid->extents);

                drawEllipsoid(
                    recenterMatrix,
                    glm::vec3(0.f, 0.4f, 1.f),
                    basis, center, extents);
                drawTextAtWorldPosition(
                    recenterMatrix,
                    center - basis[0] * extents.x,
                    "E:%.1f", m_sampleFitEllipsoid->error);
            }
        } break;
    case eCalibrationMenuState::test:
        {
            // Draw the ps dualshock4 model in the middle
            glm::mat4 controllerTransform = computeControllerPoseTransform(AppStage_AccelerometerCalibration::identity);

            drawPSDualShock4Model(controllerTransform, glm::vec3(1.f, 1.f, 1.f));
            drawTransformedAxes(controllerTransform, 200.f);

            // Draw the current calibrated accelerometer direction
            {
                const float acceleration = m_lastCalibratedAccelerometer.length();
                const float render_scale= 5.f;
                glm::vec3 m_start = glm::vec3(0.f);
                glm::vec3 m_end = psmove_float_vector3_to_glm_vec3(m_lastCalibratedAccelerometer*render_scale);

                drawArrow(controllerTransform, m_start, m_end, 0.1f, glm::vec3(1.f, 0.f, 0.f));
                drawTextAtWorldPosition(controllerTransform, m_end, "A(%.1fg)", acceleration);
            }
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_AccelerometerCalibration::renderUI()
{
    const float k_panel_width = 500;
    const char *k_window_title = "Controller Settings";
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
    case eCalibrationMenuState::selectMethod:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Select a calibration method (recommend bounding box)!");

            if (ImGui::Button("Bounding Box Fit"))
            {
                m_calibrationMethod= eCalibrationMethod::boundingBox;
                m_menuState = eCalibrationMenuState::holdAndSpinController;
            }

            if (ImGui::Button("Min Volume Fit"))
            {
                m_calibrationMethod= eCalibrationMethod::minVolumeFit;
                m_menuState = eCalibrationMenuState::placeController;
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::holdAndSpinController:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::TextWrapped(
                "Hold the controller in your hand\n" \
                "Slowly Rotate the controller around.\n"
                "Try to sweep out a sphere.\n");

            if (ImGui::Button("Start Sampling"))
            {
                m_menuState = eCalibrationMenuState::measureSphere;
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::placeController:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            if (m_calibrationMethod == eCalibrationMethod::minVolumeFit)
            {
                switch (m_currentPoseID)
                {
                case AppStage_AccelerometerCalibration::faceUp:
                    ImGui::Text("Lay the controller flat on the table face up");
                    break;
                case AppStage_AccelerometerCalibration::faceDown:
                    ImGui::Text("Lay the controller flat on the table face down");
                    break;
                case AppStage_AccelerometerCalibration::leanLeft:
                    ImGui::Text("Hold the controller on it's left side");
                    break;
                case AppStage_AccelerometerCalibration::leanRight:
                    ImGui::Text("Hold the controller on it's right side");
                    break;
                case AppStage_AccelerometerCalibration::leanForward:
                    ImGui::Text("Balance the controller on trigger button face");
                    break;
                case AppStage_AccelerometerCalibration::leanBackward:
                    ImGui::Text("Balance the controller on the thumbsticks");
                    break;
                }

                if (ImGui::Button("Start Sampling"))
                {
                    m_menuState = eCalibrationMenuState::measureDirection;
                }
                ImGui::SameLine();
            }
            else
            {
                ImGui::Text("Lay the controller flat on the table face up");

                if (ImGui::Button("Start Sampling"))
                {
                    m_menuState = eCalibrationMenuState::measureGravity;
                }
                ImGui::SameLine();
            }

            if (ImGui::Button("Cancel"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::measureGravity:
    case eCalibrationMenuState::measureSphere:
    case eCalibrationMenuState::measureDirection:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            float sampleFraction = 0;

            switch (m_menuState)
            {
            case eCalibrationMenuState::measureGravity:
                sampleFraction=
                    static_cast<float>(m_gravitySamples->sample_count)
                    / static_cast<float>(k_max_accelerometer_samples);
                break;
            case eCalibrationMenuState::measureSphere:
                sampleFraction=
                    static_cast<float>(m_sphereSamples->sample_count)
                    / static_cast<float>(k_max_accelerometer_samples);
                break;
            case eCalibrationMenuState::measureDirection:
                sampleFraction=
                    static_cast<float>(m_poseSamples[m_currentPoseID].sample_count)
                    / static_cast<float>(k_max_accelerometer_samples);
                break;
            }

            ImGui::Text("Sampling accelerometer.");
            ImGui::ProgressBar(sampleFraction, ImVec2(250, 20));

            ImGui::End();
        } break;
    case eCalibrationMenuState::measureComplete:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::TextWrapped(
                "Sampling complete.\n" \
                "Press OK to continue or Redo to resample this pose");

            if (ImGui::Button("Ok"))
            {
                m_controllerView->GetPSDualShock4ViewMutable().SetLEDOverride(0, 0, 0);

                if (m_calibrationMethod == eCalibrationMethod::minVolumeFit)
                {
                    if (m_currentPoseID >= eMeasurementPose::leanBackward)
                    {
                        m_menuState = eCalibrationMenuState::verifyCalibration;
                    }
                    else
                    {
                        // Reset the sample info for the next pose
                        AccelerometerPoseSamples &nextPoseSamples = m_poseSamples[m_currentPoseID + 1];
                        nextPoseSamples.sample_count = 0;
                        nextPoseSamples.avg_accelerometer_sample = Eigen::Vector3f::Zero();

                        // Move onto the next pose
                        m_currentPoseID = static_cast<eMeasurementPose>(m_currentPoseID + 1);

                        m_menuState = eCalibrationMenuState::placeController;
                    }
                }
                else
                {
                    m_menuState = eCalibrationMenuState::verifyCalibration;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Redo"))
            {
                // Reset the sample info for the current pose
                m_poseSamples[m_currentPoseID].sample_count = 0;
                m_poseSamples[m_currentPoseID].avg_accelerometer_sample = Eigen::Vector3f::Zero();

                m_menuState = eCalibrationMenuState::placeController;
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::verifyCalibration:
    case eCalibrationMenuState::test:
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
    default:
        assert(0 && "unreachable");
    }
}

//-- private methods -----
static void request_set_accelerometer_calibration(
    const int controller_id,
    const EigenFitEllipsoid *ellipsoid)
{
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_ACCELEROMETER_CALIBRATION);

    PSMoveProtocol::Request_RequestSetAccelerometerCalibration *calibration =
        request->mutable_set_accelerometer_calibration_request();

    calibration->set_controller_id(controller_id);

    // The gain for each axis is 1 / ellipsoid extent on that axis
    const float gain_x = safe_divide_with_default(1.f, ellipsoid->extents.x(), 1.f);
    const float gain_y = safe_divide_with_default(1.f, ellipsoid->extents.y(), 1.f);
    const float gain_z = safe_divide_with_default(1.f, ellipsoid->extents.z(), 1.f);

    calibration->mutable_gain()->set_i(gain_x);
    calibration->mutable_gain()->set_j(gain_y);
    calibration->mutable_gain()->set_k(gain_z);

    // If the calibrated acceleration is: a=raw_a*gain + bias,
    // the the bias need to be -ellipsoid.center*gain
    calibration->mutable_bias()->set_i(-ellipsoid->center.x() * gain_x);
    calibration->mutable_bias()->set_j(-ellipsoid->center.y() * gain_y);
    calibration->mutable_bias()->set_k(-ellipsoid->center.z() * gain_z);

    calibration->set_ellipse_fit_error(ellipsoid->error);

    ClientPSMoveAPI::eat_response(ClientPSMoveAPI::send_opaque_request(&request));
}

void AppStage_AccelerometerCalibration::handle_acquire_controller(
    const ClientPSMoveAPI::ResponseMessage *response,
    void *userdata)
{
    AppStage_AccelerometerCalibration *thisPtr = reinterpret_cast<AppStage_AccelerometerCalibration *>(userdata);

    if (response->result_code == ClientPSMoveAPI::_clientPSMoveResultCode_ok)
    {
        thisPtr->m_isControllerStreamActive = true;
        thisPtr->m_lastControllerSeqNum = -1;
        // Wait for the first controller packet to show up...
    }
    else
    {
        thisPtr->m_menuState = AppStage_AccelerometerCalibration::failedStreamStart;
    }
}

void AppStage_AccelerometerCalibration::request_exit_to_app_stage(const char *app_stage_name)
{
    ClientPSMoveAPI::eat_response(ClientPSMoveAPI::stop_controller_data_stream(m_controllerView));
    m_isControllerStreamActive= false;
    m_app->setAppStage(app_stage_name);
}

//-- private methods -----
static void 
expandAccelerometerBounds(
    const PSMoveIntVector3 &sample,
    PSMoveIntVector3 &minSampleExtents,
    PSMoveIntVector3 &maxSampleExtents)
{
    minSampleExtents = PSMoveIntVector3::min(minSampleExtents, sample);
    maxSampleExtents = PSMoveIntVector3::max(maxSampleExtents, sample);
}

static glm::mat4 
computeControllerPoseTransform(AppStage_AccelerometerCalibration::eMeasurementPose PoseID)
{
    const float modelScale = 18.f;
    glm::mat4 transform;

    switch (PoseID)
    {
    case AppStage_AccelerometerCalibration::identity:
        transform = glm::scale(glm::mat4(1.f), glm::vec3(modelScale, modelScale, modelScale));
        break;
    case AppStage_AccelerometerCalibration::faceUp:
        transform =
            glm::rotate(
            glm::scale(glm::mat4(1.f), glm::vec3(modelScale, modelScale, modelScale)),
            -15.f, glm::vec3(1.f, 0.f, 0.f));
        break;
    case AppStage_AccelerometerCalibration::faceDown:
        transform =
            glm::rotate(
            glm::scale(glm::mat4(1.f), glm::vec3(modelScale, modelScale, modelScale)),
            190.f, glm::vec3(1.f, 0.f, 0.f));
        break;
    case AppStage_AccelerometerCalibration::leanLeft:
        transform =
            glm::rotate(
            glm::scale(glm::mat4(1.f), glm::vec3(modelScale, modelScale, modelScale)),
            80.f, glm::vec3(0.f, 0.f, 1.f));
        break;
    case AppStage_AccelerometerCalibration::leanRight:
        transform =
            glm::rotate(
            glm::scale(glm::mat4(1.f), glm::vec3(modelScale, modelScale, modelScale)),
            -80.f, glm::vec3(0.f, 0.f, 1.f));
        break;
    case AppStage_AccelerometerCalibration::leanForward:
        transform =
            glm::rotate(
            glm::scale(glm::mat4(1.f), glm::vec3(modelScale, modelScale, modelScale)),
            -90.f, glm::vec3(1.f, 0.f, 0.f));
        break;
    case AppStage_AccelerometerCalibration::leanBackward:
        transform =
            glm::rotate(
            glm::scale(glm::mat4(1.f), glm::vec3(modelScale, modelScale, modelScale)),
            135.f, glm::vec3(1.f, 0.f, 0.f));
        break;
    }

    return transform;
}
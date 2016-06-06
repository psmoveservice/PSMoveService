//-- inludes -----
#include "AppSubStage_CalibrateWithHMD.h"
#include "AppStage_ComputeTrackerPoses.h"
#include "App.h"
#include "AssetManager.h"
#include "Camera.h"
#include "ClientHMDView.h"
#include "GeometryUtility.h"
#include "Logger.h"
#include "OpenVRContext.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "MathGLM.h"
#include "MathEigen.h"

#include "SDL_keycode.h"
#include "SDL_opengl.h"

#include <imgui.h>

//-- constants -----
static const glm::vec3 k_hmd_frustum_color = glm::vec3(1.f, 0.788f, 0.055f);
static const glm::vec3 k_psmove_frustum_color = glm::vec3(0.1f, 0.7f, 0.3f);

//-- private methods ----
static void drawFrustumBounds(const FrustumBounds &frustum, const glm::vec3 &color);
static bool computeCameraPoseTransform(TrackerCoregistrationData &trackerCoregData);

//-- public methods -----
AppSubStage_CalibrateWithHMD::AppSubStage_CalibrateWithHMD(
    AppStage_ComputeTrackerPoses *parentStage)
    : m_parentStage(parentStage)
    , m_menuState(AppSubStage_CalibrateWithHMD::eMenuState::invalid)
{
    for (int tracker_index = 0; tracker_index < PSMOVESERVICE_MAX_TRACKER_COUNT; ++tracker_index)
    {
        m_trackerCoreg[tracker_index].clear();
    }
}

void AppSubStage_CalibrateWithHMD::enter()
{
    setState(AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepAttachPSMove);
}

void AppSubStage_CalibrateWithHMD::exit()
{
    setState(AppSubStage_CalibrateWithHMD::eMenuState::invalid);
}

void AppSubStage_CalibrateWithHMD::update()
{
    const ClientControllerView *ControllerView = m_parentStage->m_controllerView;
    const ClientPSMoveView &PSMoveView = ControllerView->GetPSMoveView();
    const ClientHMDView *HMDView = m_parentStage->m_hmdView;

    switch (m_menuState)
    {
    case AppSubStage_CalibrateWithHMD::eMenuState::invalid:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepAttachPSMove:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepRecordHmdPSMove:
        {
            PSMovePose dk2pose = HMDView->getRawHmdPose();
            PSMovePosition dk2position = dk2pose.Position;
            bool bAllTrackersComplete = true;

            for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                iter != m_parentStage->m_trackerViews.end();
                ++iter)
            {
                const int trackerIndex = iter->second.listIndex;
                const ClientTrackerView *trackerView = iter->second.trackerView;

                TrackerCoregistrationData &trackerCoregData = m_trackerCoreg[trackerIndex];
                PSMovePosition positionOnTracker;
                if (PSMoveView.GetIsCurrentlyTracking() &&
                    PSMoveView.GetRawTrackerData().GetPositionOnTrackerId(trackerView->getTrackerId(), positionOnTracker) &&
                    trackerCoregData.poseCount < NPOSES)
                {
                    trackerCoregData.hmd_poses[trackerCoregData.poseCount] = dk2pose;
                    trackerCoregData.psmoveposes[trackerCoregData.poseCount] = positionOnTracker;
                    trackerCoregData.poseCount++;
                }

                if (trackerCoregData.poseCount < NPOSES)
                {
                    bAllTrackersComplete = false;
                }
            }

            if (bAllTrackersComplete)
            {
                setState(AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepComputeTrackerPoses);
            }
        } break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepComputeTrackerPoses:
        {
            bool bSuccess = true;

            // Compute and the pose transform for each tracker
            for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                bSuccess && iter != m_parentStage->m_trackerViews.end();
                ++iter)
            {
                const int trackerIndex = iter->second.listIndex;
                const ClientTrackerView *trackerView = iter->second.trackerView;
                TrackerCoregistrationData &trackerCoregData = m_trackerCoreg[trackerIndex];

                bSuccess= computeCameraPoseTransform(trackerCoregData);
            }

            // Update the poses on each local tracker view and notify the service of the new pose
            if (bSuccess)
            {
                for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                    bSuccess && iter != m_parentStage->m_trackerViews.end();
                    ++iter)
                {
                    const int trackerIndex = iter->second.listIndex;
                    TrackerCoregistrationData &trackerCoregData = m_trackerCoreg[trackerIndex];

                    // In this calibration mode,
                    // the psmove calibration space origin is the hmd tracking camera.
                    // Therefore trackerPose = hmdRelativeTrackerPose
                    const PSMovePose trackerPose = trackerCoregData.trackerPose;

                    ClientTrackerView *trackerView = iter->second.trackerView;

                    m_parentStage->request_set_tracker_pose(&trackerPose, trackerView);
                }
            }

            if (bSuccess)
            {
                setState(AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepSuccess);
            }
            else
            {
                setState(AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepFailed);
            }
        } break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepSuccess:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_CalibrateWithHMD::render()
{
    switch (m_menuState)
    {
    case AppSubStage_CalibrateWithHMD::eMenuState::invalid:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepAttachPSMove:
        {
            glm::mat4 rotateX90 = glm::rotate(glm::mat4(1.f), 90.f, glm::vec3(1.f, 0.f, 0.f));

            // Offset the models (in cm) just enough so that they look like they are attached
            drawDK2Model(glm::translate(glm::mat4(1.f), glm::vec3(-9.f, 0.f, 0.f)));
            drawPSMoveModel(glm::translate(glm::mat4(1.f), glm::vec3(2.3f, 0.f, 0.f)) * rotateX90, glm::vec3(1.f, 1.f, 1.f));
        } break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepRecordHmdPSMove:
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepComputeTrackerPoses:
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepSuccess:
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepFailed:
        {
            const ClientControllerView *ControllerView = m_parentStage->m_controllerView;
            const ClientPSMoveView &PSMoveView = ControllerView->GetPSMoveView();
            const ClientHMDView *HMDView = m_parentStage->m_hmdView;

            // Draw the origin axes
            drawTransformedAxes(glm::mat4(1.0f), 100.f);

            for (AppStage_ComputeTrackerPoses::t_tracker_state_map_iterator iter = m_parentStage->m_trackerViews.begin();
                iter != m_parentStage->m_trackerViews.end();
                ++iter)
            {
                const int trackerIndex = iter->second.listIndex;
                const ClientTrackerView *trackerView = iter->second.trackerView;
                TrackerCoregistrationData &trackerCoregData = m_trackerCoreg[trackerIndex];

                // Draw a line strip connecting all of the dk2 positions collected so far
                if (trackerCoregData.poseCount > 0)
                {
                    drawPoseArrayStrip(trackerCoregData.hmd_poses, trackerCoregData.poseCount, glm::vec3(1.f, 1.f, 0.f));
                }
            }

            // Render the HMD tracking volume
            {
                PSMoveVolume volume;

                if (m_parentStage->m_app->getOpenVRContext()->getHMDTrackingVolume(volume))
                {
                    drawTransformedVolume(glm::mat4(1.f), &volume, glm::vec3(0.f, 1.f, 1.f));
                }
            }

            // Draw the DK2 model
            {
                glm::mat4 transform = psmove_pose_to_glm_mat4(HMDView->getDisplayHmdPose());

                drawDK2Model(transform);
                drawTransformedAxes(transform, 10.f);
            }
        }
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_CalibrateWithHMD::renderUI()
{
    const float k_panel_width = 300.f;
    const char *k_window_title = "Coregister Tracker with HMD";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case AppSubStage_CalibrateWithHMD::eMenuState::invalid:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepAttachPSMove:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 100));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Attach the PSMove to the side of the HMD");
            ImGui::Text("Press Begin when ready to sample poses");

            if (ImGui::Button("Cancel"))
            {
                m_parentStage->setState(AppStage_ComputeTrackerPoses::eMenuState::selectCalibrationType);
            }
            ImGui::SameLine();
            if (ImGui::Button("Begin"))
            {
                setState(AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepRecordHmdPSMove);
            }

            ImGui::End();
        } break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepRecordHmdPSMove:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 100));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Sweep HMD+PSMove around the frustum.");
            ImGui::Text("Try to cover as much area as possible.");

            if (ImGui::Button("Cancel"))
            {
                m_parentStage->setState(AppStage_ComputeTrackerPoses::eMenuState::selectCalibrationType);
            }
            ImGui::SameLine();
            if (ImGui::Button("Restart"))
            {
                setState(AppSubStage_CalibrateWithHMD::eMenuState::invalid);
            }

            ImGui::End();
        } break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepComputeTrackerPoses:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepSuccess:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}


//-- private methods -----
void AppSubStage_CalibrateWithHMD::setState(
    AppSubStage_CalibrateWithHMD::eMenuState newState)
{
    if (newState != m_menuState)
    {
        onExitState(m_menuState);
        onEnterState(newState);
        m_menuState = newState;
    }
}

void AppSubStage_CalibrateWithHMD::onExitState(
    AppSubStage_CalibrateWithHMD::eMenuState oldState)
{
    switch (oldState)
    {
    case AppSubStage_CalibrateWithHMD::eMenuState::invalid:
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepAttachPSMove:
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepRecordHmdPSMove:
        m_parentStage->m_app->setCameraType(_cameraFixed);
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepComputeTrackerPoses:
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepSuccess:
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_CalibrateWithHMD::onEnterState(
    AppSubStage_CalibrateWithHMD::eMenuState newState)
{
    switch (newState)
    {
    case AppSubStage_CalibrateWithHMD::eMenuState::invalid:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepAttachPSMove:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepRecordHmdPSMove:
        {
            for (int trackerIndex = 0; trackerIndex < PSMOVESERVICE_MAX_TRACKER_COUNT; ++trackerIndex)
            {
                m_trackerCoreg[trackerIndex].clear();
            }

            m_parentStage->m_app->setCameraType(_cameraOrbit);
            m_parentStage->m_app->getOrbitCamera()->setCameraOrbitLocation(45.f, 30.f, 700.f);
        } break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepComputeTrackerPoses:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepSuccess:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

//-- private methods -----
void FrustumBounds::clear()
{
    origin = *k_psmove_position_origin;
    forward = *k_psmove_float_vector3_k;
    left = *k_psmove_float_vector3_i;
    up = *k_psmove_float_vector3_j;
    hAngleMin = hAngleMax = 0.f;
    vAngleMin = vAngleMax = 0.f;
    zNear = zFar = 0.f;
}

void FrustumBounds::init(const PSMoveFrustum &frustum)
{
    origin = frustum.origin;
    forward = frustum.forward;
    left = frustum.left;
    up = frustum.up;
    hAngleMin = k_real_pi;
    hAngleMax = -k_real_pi;
    vAngleMin = k_real_pi;
    vAngleMax = -k_real_pi;
    zNear = k_real_max;
    zFar = -k_real_max;
}

void FrustumBounds::enclosePoint(const PSMovePosition &point)
{
    PSMoveFloatVector3 v = point - origin;
    float vAngle = atan2f(PSMoveFloatVector3::dot(v, up), PSMoveFloatVector3::dot(v, forward));
    float hAngle = atan2f(PSMoveFloatVector3::dot(v, left), PSMoveFloatVector3::dot(v, forward));
    float z = PSMoveFloatVector3::dot(v, forward);

    if (z > k_real_epsilon)
    {
        hAngleMin = fmin(hAngle, hAngleMin);
        hAngleMax = fmax(hAngle, hAngleMax);
        vAngleMin = fmin(vAngle, vAngleMin);
        vAngleMax = fmax(vAngle, vAngleMax);
        zNear = fmin(z, zNear);
        zFar = fmax(z, zFar);
    }
}

static void drawFrustumBounds(const FrustumBounds &frustum, const glm::vec3 &color)
{
    assert(Renderer::getIsRenderingStage());

    if (frustum.hAngleMax > frustum.hAngleMin &&
        frustum.vAngleMax > frustum.vAngleMin &&
        frustum.zFar > frustum.zNear)
    {
        const float HMinRatio = tanf(frustum.hAngleMin);
        const float HMaxRatio = tanf(frustum.hAngleMax);
        const float VMinRatio = tanf(frustum.vAngleMin);
        const float VMaxRatio = tanf(frustum.vAngleMax);

        glm::vec3 origin = psmove_position_to_glm_vec3(frustum.origin);

        glm::vec3 nearXMin = psmove_float_vector3_to_glm_vec3(frustum.left*frustum.zNear*HMinRatio);
        glm::vec3 farXMin = psmove_float_vector3_to_glm_vec3(frustum.left*frustum.zFar*HMinRatio);
        glm::vec3 nearXMax = psmove_float_vector3_to_glm_vec3(frustum.left*frustum.zNear*HMaxRatio);
        glm::vec3 farXMax = psmove_float_vector3_to_glm_vec3(frustum.left*frustum.zFar*HMaxRatio);

        glm::vec3 nearYMin = psmove_float_vector3_to_glm_vec3(frustum.up*frustum.zNear*VMinRatio);
        glm::vec3 farYMin = psmove_float_vector3_to_glm_vec3(frustum.up*frustum.zFar*VMinRatio);
        glm::vec3 nearYMax = psmove_float_vector3_to_glm_vec3(frustum.up*frustum.zNear*VMaxRatio);
        glm::vec3 farYMax = psmove_float_vector3_to_glm_vec3(frustum.up*frustum.zFar*VMaxRatio);

        glm::vec3 nearZ = psmove_float_vector3_to_glm_vec3(frustum.forward*frustum.zNear);
        glm::vec3 farZ = psmove_float_vector3_to_glm_vec3(frustum.forward*frustum.zFar);

        glm::vec3 nearCenter = origin + nearZ;
        glm::vec3 near0 = origin + nearXMax + nearYMax + nearZ;
        glm::vec3 near1 = origin + nearXMin + nearYMax + nearZ;
        glm::vec3 near2 = origin + nearXMin + nearYMin + nearZ;
        glm::vec3 near3 = origin + nearXMax + nearYMin + nearZ;

        glm::vec3 far0 = origin + farXMax + farYMax + farZ;
        glm::vec3 far1 = origin + farXMin + farYMax + farZ;
        glm::vec3 far2 = origin + farXMin + farYMin + farZ;
        glm::vec3 far3 = origin + farXMax + farYMin + farZ;

        glBegin(GL_LINES);

        glColor3fv(glm::value_ptr(color));

        glVertex3fv(glm::value_ptr(near0)); glVertex3fv(glm::value_ptr(near1));
        glVertex3fv(glm::value_ptr(near1)); glVertex3fv(glm::value_ptr(near2));
        glVertex3fv(glm::value_ptr(near2)); glVertex3fv(glm::value_ptr(near3));
        glVertex3fv(glm::value_ptr(near3)); glVertex3fv(glm::value_ptr(near0));

        glVertex3fv(glm::value_ptr(far0)); glVertex3fv(glm::value_ptr(far1));
        glVertex3fv(glm::value_ptr(far1)); glVertex3fv(glm::value_ptr(far2));
        glVertex3fv(glm::value_ptr(far2)); glVertex3fv(glm::value_ptr(far3));
        glVertex3fv(glm::value_ptr(far3)); glVertex3fv(glm::value_ptr(far0));

        glVertex3fv(glm::value_ptr(near0)); glVertex3fv(glm::value_ptr(far0));
        glVertex3fv(glm::value_ptr(near1)); glVertex3fv(glm::value_ptr(far1));
        glVertex3fv(glm::value_ptr(near2)); glVertex3fv(glm::value_ptr(far2));
        glVertex3fv(glm::value_ptr(near3)); glVertex3fv(glm::value_ptr(far3));

        glEnd();
    }
}

static bool computeCameraPoseTransform(
    TrackerCoregistrationData &trackerCoregData)
{
    const PSMovePose *hmd_poses = trackerCoregData.hmd_poses;
    const PSMovePosition *psmoveposes = trackerCoregData.psmoveposes;
    const int poseCount = trackerCoregData.poseCount;

    bool bSuccess = false;

    Eigen::MatrixXf A(poseCount * 3, 15);  // X = A/b
    Eigen::VectorXf b(poseCount * 3);

    // Build the A matrix and the b column vector from the given poses
    {
        Eigen::Matrix4f eigenHmdPose;             // HMD pose in Eigen 4x4 mat
        Eigen::Matrix3f RMi;                // Transpose of inner 3x3 of DK2 pose

        for (int poseIndex = 0; poseIndex < poseCount; ++poseIndex)
        {
            const PSMovePosition &psmove_pos = psmoveposes[poseIndex];

            //###HipsterSloth $TODO Go from PSMovePose directly to Eigen::Matrix4f
            PSMovePose hmdPose = hmd_poses[poseIndex];
            glm::mat4 glmHmdPose = psmove_pose_to_glm_mat4(hmdPose);
            eigenHmdPose= glm_mat4_to_eigen_matrix4f(glmHmdPose);
            RMi = eigenHmdPose.topLeftCorner(3, 3).transpose();           // inner 33 transposed

            A.block<3, 3>(poseIndex * 3, 0) = RMi * psmove_pos.x;
            A.block<3, 3>(poseIndex * 3, 3) = RMi * psmove_pos.y;
            A.block<3, 3>(poseIndex * 3, 6) = RMi * psmove_pos.z;
            A.block<3, 3>(poseIndex * 3, 9) = RMi;
            A.block<3, 3>(poseIndex * 3, 12) = -Eigen::Matrix3f::Identity();
            b.segment(poseIndex * 3, 3) = RMi * eigenHmdPose.block<3, 1>(0, 3);
        }
    }

    // Compute the co-registration transform and save to disk
    {
        glm::mat4 glmResultTransform = glm::mat4(1.f);

        Eigen::VectorXf x(15);
        x = A.colPivHouseholderQr().solve(b);
        Log_INFO("Coreg", "\nglobalxfm:\n%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f\n",
            x(0), x(3), x(6), x(9),
            x(1), x(4), x(7), x(10),
            x(2), x(5), x(8), x(11));
        Log_INFO("Coreg", "\nlocalxfm:\n%f,%f,%f\n", x(12), x(13), x(14));

        //###HipsterSloth $TODO Go from Eigen::Matrix4f directly to PSMovePose
        glmResultTransform = glm::mat4(1.f);
        for (int i = 0; i < 12; i++)
        {
            int row_ix = i % 3;
            int col_ix = i / 3;
            glmResultTransform[col_ix][row_ix] = x(i);
        }

        trackerCoregData.trackerPose = glm_mat4_to_psmove_pose(glmResultTransform);
        trackerCoregData.bComputedCoregTransform = true;

        //###HipsterSloth $TODO Actually detect the failure cases for solving this linear system
        bSuccess = true;
    }

    return bSuccess;
}
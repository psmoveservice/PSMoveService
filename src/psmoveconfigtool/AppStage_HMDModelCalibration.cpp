//-- inludes -----
#include "AppStage_HMDModelCalibration.h"
#include "AppStage_HMDSettings.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "AssetManager.h"
#include "Camera.h"
#include "GeometryUtility.h"
#include "Logger.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include "SharedTrackerState.h"
#include "MathGLM.h"

#include "SDL_keycode.h"
#include "SDL_opengl.h"

#include "ClientPSMoveAPI.h"

#include <imgui.h>
#include <sstream>

//-- statics ----
const char *AppStage_HMDModelCalibration::APP_STAGE_NAME = "HMDModelCalibration";

//-- constants -----
static const glm::vec3 k_psmove_frustum_color = glm::vec3(0.1f, 0.7f, 0.3f);
static const glm::vec3 k_psmove_frustum_color_no_track = glm::vec3(1.0f, 0.f, 0.f);

//-- private methods -----
static void drawHMD(ClientHMDView *hmdView, const glm::mat4 &transform);

//-- public methods -----
AppStage_HMDModelCalibration::AppStage_HMDModelCalibration(App *app)
	: AppStage(app)
	, m_menuState(AppStage_HMDModelCalibration::inactive)
	, m_hmdView(nullptr)
	, m_pendingTrackerStartCount(0)
	, m_renderTrackerIndex(0)
	, m_bBypassCalibration(false)
{
	m_renderTrackerIter = m_trackerViews.end();
}

AppStage_HMDModelCalibration::~AppStage_HMDModelCalibration()
{
}

void AppStage_HMDModelCalibration::enterStageAndCalibrate(App *app, int requested_hmd_id)
{
	app->getAppStage<AppStage_HMDModelCalibration>()->m_bBypassCalibration = false;
	app->getAppStage<AppStage_HMDModelCalibration>()->m_overrideHmdId = requested_hmd_id;
	app->setAppStage(AppStage_HMDModelCalibration::APP_STAGE_NAME);
}

void AppStage_HMDModelCalibration::enterStageAndSkipCalibration(App *app, int requested_hmd_id)
{
	app->getAppStage<AppStage_HMDModelCalibration>()->m_bBypassCalibration = true;
	app->getAppStage<AppStage_HMDModelCalibration>()->m_overrideHmdId = requested_hmd_id;
	app->setAppStage(AppStage_HMDModelCalibration::APP_STAGE_NAME);
}

void AppStage_HMDModelCalibration::enter()
{
	// Kick off this async request chain with an hmd list request
	// -> hmd start request
	// -> tracker list request
	// -> tracker start request
	request_hmd_list();

	m_app->setCameraType(_cameraFixed);
}

void AppStage_HMDModelCalibration::exit()
{
	release_devices();

	setState(eMenuState::inactive);
}

void AppStage_HMDModelCalibration::update()
{
	switch (m_menuState)
	{
	case eMenuState::inactive:
		break;
	case eMenuState::pendingHmdListRequest:
	case eMenuState::pendingHmdStartRequest:
	case eMenuState::pendingTrackerListRequest:
	case eMenuState::pendingTrackerStartRequest:
		break;
	case eMenuState::failedHmdListRequest:
	case eMenuState::failedHmdStartRequest:
	case eMenuState::failedTrackerListRequest:
	case eMenuState::failedTrackerStartRequest:
		break;
	case eMenuState::verifyTrackers:
		update_tracker_video();
		break;
	case eMenuState::calibrate:
	{
		update_tracker_video();
	}
	break;
	case eMenuState::test:
		//TODO
		break;
	default:
		assert(0 && "unreachable");
	}
}

void AppStage_HMDModelCalibration::render()
{
	switch (m_menuState)
	{
	case eMenuState::inactive:
		break;
	case eMenuState::pendingHmdListRequest:
	case eMenuState::pendingHmdStartRequest:
	case eMenuState::pendingTrackerListRequest:
	case eMenuState::pendingTrackerStartRequest:
		break;
	case eMenuState::failedHmdListRequest:
	case eMenuState::failedHmdStartRequest:
	case eMenuState::failedTrackerListRequest:
	case eMenuState::failedTrackerStartRequest:
		break;
	case eMenuState::verifyTrackers:
		{
			render_tracker_video();
		} break;
	case eMenuState::calibrate:
		{
			// Draw the video from the PoV of the current tracker
			render_tracker_video();

			// Draw the projection shape of the controller in the pov of the current tracker being rendered
			{
				const ClientTrackerView *TrackerView = get_render_tracker_view();
				const MorpheusRawTrackerData &RawTrackerData = m_hmdView->GetRawTrackerData();
				const int TrackerID = TrackerView->getTrackerId();

				PSMoveTrackingProjection trackingProjection;
				PSMoveScreenLocation centerProjection;

				if (m_hmdView->GetIsCurrentlyTracking() &&
					RawTrackerData.GetPixelLocationOnTrackerId(TrackerID, centerProjection) &&
					RawTrackerData.GetProjectionOnTrackerId(TrackerID, trackingProjection))
				{
					const PSMoveFloatVector2 screenSize = TrackerView->getTrackerInfo().tracker_screen_dimensions;

					drawTrackingProjection(
						&centerProjection,
						&trackingProjection,
						screenSize.i, screenSize.j);
				}
			}
		}
		break;
	case eMenuState::test:
		{
			// Draw the chaperone origin axes
			drawTransformedAxes(glm::mat4(1.0f), 100.f);

			// Draw the frustum for each tracking camera.
			// The frustums are defined in PSMove tracking space.
			// We need to transform them into chaperone space to display them along side the HMD.
			for (t_tracker_state_map_iterator iter = m_trackerViews.begin(); iter != m_trackerViews.end(); ++iter)
			{
				const ClientTrackerView *trackerView = iter->second.trackerView;
				const PSMovePose psmove_space_pose = trackerView->getTrackerPose();
				const glm::mat4 chaperoneSpaceTransform = psmove_pose_to_glm_mat4(psmove_space_pose);

				{
					PSMoveFrustum frustum = trackerView->getTrackerFrustum();

					// use color depending on tracking status
					PSMoveScreenLocation screenSample;
					glm::vec3 color;
					if (m_hmdView->GetIsCurrentlyTracking() &&
						m_hmdView->GetRawTrackerData().GetPixelLocationOnTrackerId(trackerView->getTrackerId(), screenSample))
					{
						color = k_psmove_frustum_color;
					}
					else
					{
						color = k_psmove_frustum_color_no_track;
					}

					drawTransformedFrustum(glm::mat4(1.f), &frustum, color);
				}

				drawTransformedAxes(chaperoneSpaceTransform, 20.f);
			}


			// Draw the Morpheus model
			{
				PSMovePose hmd_pose = m_hmdView->GetPose();
				glm::mat4 hmd_transform = psmove_pose_to_glm_mat4(hmd_pose);

				drawHMD(m_hmdView, hmd_transform);
				drawTransformedAxes(hmd_transform, 10.f);
			}

		} break;
	default:
		assert(0 && "unreachable");
	}
}

void AppStage_HMDModelCalibration::renderUI()
{
	const float k_panel_width = 300.f;
	const char *k_window_title = "Compute HMD Model";
	const ImGuiWindowFlags window_flags =
		ImGuiWindowFlags_ShowBorders |
		ImGuiWindowFlags_NoResize |
		ImGuiWindowFlags_NoMove |
		ImGuiWindowFlags_NoScrollbar |
		ImGuiWindowFlags_NoCollapse;

	switch (m_menuState)
	{
	case eMenuState::inactive:
		break;

	case eMenuState::pendingHmdListRequest:
	case eMenuState::pendingHmdStartRequest:
	case eMenuState::pendingTrackerListRequest:
	case eMenuState::pendingTrackerStartRequest:
	{
		ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
		ImGui::SetNextWindowSize(ImVec2(k_panel_width, 80));
		ImGui::Begin(k_window_title, nullptr, window_flags);

		ImGui::Text("Pending device initialization...");

		if (ImGui::Button("Return to HMD Settings"))
		{
			request_exit_to_app_stage(AppStage_HMDModelCalibration::APP_STAGE_NAME);
		}

		ImGui::End();
	} break;

	case eMenuState::failedHmdListRequest:
	case eMenuState::failedHmdStartRequest:
	case eMenuState::failedTrackerListRequest:
	case eMenuState::failedTrackerStartRequest:
	{
		ImGui::SetNextWindowPosCenter();
		ImGui::SetNextWindowSize(ImVec2(k_panel_width, 180));
		ImGui::Begin(k_window_title, nullptr, window_flags);

		switch (m_menuState)
		{
		case eMenuState::failedHmdListRequest:
			ImGui::Text("Failed hmd list retrieval!");
			break;
		case eMenuState::failedHmdStartRequest:
			ImGui::Text("Failed hmd stream start!");
			break;
		case eMenuState::failedTrackerListRequest:
			ImGui::Text("Failed tracker list retrieval!");
			break;
		case eMenuState::failedTrackerStartRequest:
			ImGui::Text("Failed tracker stream start!");
			break;
		}

		if (ImGui::Button("Ok"))
		{
			request_exit_to_app_stage(AppStage_HMDSettings::APP_STAGE_NAME);
		}

		if (ImGui::Button("Return to Main Menu"))
		{
			request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
		}

		ImGui::End();
	} break;

	case eMenuState::verifyTrackers:
	{
		ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - 500.f / 2.f, 20.f));
		ImGui::SetNextWindowSize(ImVec2(500.f, (m_trackerViews.size() > 0) ? 150.f : 100.f));
		ImGui::Begin(k_window_title, nullptr, window_flags);

		ImGui::Text("Verify that your tracking cameras can see your HMD");
		ImGui::Separator();

		if (m_trackerViews.size() > 1)
		{
			ImGui::Text("Tracker #%d", m_renderTrackerIndex + 1);

			if (ImGui::Button("Previous Tracker"))
			{
				go_previous_tracker();
			}
			ImGui::SameLine();
			if (ImGui::Button("Next Tracker"))
			{
				go_next_tracker();
			}
		}

		if (ImGui::Button("Looks Good!"))
		{
			setState(eMenuState::calibrate);
		}

		if (ImGui::Button("Hmm... Something is wrong."))
		{
			request_exit_to_app_stage(AppStage_HMDSettings::APP_STAGE_NAME);
		}

		ImGui::End();
	} break;

	case eMenuState::calibrate:
	{
		ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
		ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
		ImGui::Begin(k_window_title, nullptr, window_flags);

		if (m_trackerViews.size() > 1)
		{
			ImGui::Text("Tracker #%d", m_renderTrackerIndex + 1);

			if (ImGui::Button("Previous Tracker"))
			{
				go_previous_tracker();
			}
			ImGui::SameLine();
			if (ImGui::Button("Next Tracker"))
			{
				go_next_tracker();
			}
		}

		ImGui::Separator();

		// TODO: Show calibration progress

		// display tracking quality
		for (t_tracker_state_map_iterator iter = m_trackerViews.begin(); iter != m_trackerViews.end(); ++iter)
		{
			PSMoveScreenLocation screenSample;

			const ClientTrackerView *trackerView = iter->second.trackerView;
			if (m_hmdView->GetIsCurrentlyTracking() &&
				m_hmdView->GetRawTrackerData().GetPixelLocationOnTrackerId(trackerView->getTrackerId(), screenSample))
			{
				ImGui::Text("Tracking %d: OK", trackerView->getTrackerId() + 1);
			}
			else {
				ImGui::Text("Tracking %d: FAIL", trackerView->getTrackerId() + 1);
			}
		}

		ImGui::Separator();

		if (ImGui::Button("Restart Calibration"))
		{
			setState(eMenuState::verifyTrackers);
		}
		ImGui::SameLine();
		if (ImGui::Button("Exit"))
		{
			m_app->setAppStage(AppStage_HMDSettings::APP_STAGE_NAME);
		}

		ImGui::End();
	} break;

	case eMenuState::test:
	{
		ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
		ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
		ImGui::Begin(k_window_title, nullptr, window_flags);

		if (!m_bBypassCalibration)
		{
			ImGui::Text("Calibration Complete");

			if (ImGui::Button("Redo Calibration"))
			{
				setState(eMenuState::verifyTrackers);
			}
		}

		if (ImGui::Button("Exit"))
		{
			m_app->setAppStage(AppStage_HMDModelCalibration::APP_STAGE_NAME);
		}

		// display tracking quality
		for (t_tracker_state_map_iterator iter = m_trackerViews.begin(); iter != m_trackerViews.end(); ++iter)
		{
			PSMoveScreenLocation screenSample;

			const ClientTrackerView *trackerView = iter->second.trackerView;
			if (m_hmdView->GetIsCurrentlyTracking() &&
				m_hmdView->GetRawTrackerData().GetPixelLocationOnTrackerId(trackerView->getTrackerId(), screenSample))
			{
				ImGui::Text("Tracking %d: OK", trackerView->getTrackerId() + 1);
			}
			else {
				ImGui::Text("Tracking %d: FAIL", trackerView->getTrackerId() + 1);
			}
		}
		ImGui::Text("");

		ImGui::End();
	}
	break;

	default:
		assert(0 && "unreachable");
	}
}

void AppStage_HMDModelCalibration::setState(AppStage_HMDModelCalibration::eMenuState newState)
{
	if (newState != m_menuState)
	{
		onExitState(m_menuState);
		onEnterState(newState);

		m_menuState = newState;
	}
}

void AppStage_HMDModelCalibration::onExitState(AppStage_HMDModelCalibration::eMenuState newState)
{
	switch (m_menuState)
	{
	case eMenuState::inactive:
		break;
	case eMenuState::pendingHmdListRequest:
	case eMenuState::pendingHmdStartRequest:
	case eMenuState::pendingTrackerListRequest:
	case eMenuState::pendingTrackerStartRequest:
		break;
	case eMenuState::failedHmdListRequest:
	case eMenuState::failedHmdStartRequest:
	case eMenuState::failedTrackerListRequest:
	case eMenuState::failedTrackerStartRequest:
		break;
	case eMenuState::verifyTrackers:
		break;
	case eMenuState::calibrate:
		break;
	case eMenuState::test:
		m_app->setCameraType(_cameraFixed);
		break;
	default:
		assert(0 && "unreachable");
	}
}

void AppStage_HMDModelCalibration::onEnterState(AppStage_HMDModelCalibration::eMenuState newState)
{
	switch (newState)
	{
	case eMenuState::inactive:
		break;
	case eMenuState::pendingHmdListRequest:
	case eMenuState::pendingHmdStartRequest:
	case eMenuState::pendingTrackerListRequest:
		break;
	case eMenuState::pendingTrackerStartRequest:
		m_trackerViews.clear();
		m_pendingTrackerStartCount = 0;
		break;
	case eMenuState::failedHmdListRequest:
	case eMenuState::failedHmdStartRequest:
	case eMenuState::failedTrackerListRequest:
	case eMenuState::failedTrackerStartRequest:
		break;
	case eMenuState::verifyTrackers:
		m_renderTrackerIter = m_trackerViews.begin();
		break;
	case eMenuState::calibrate:
		// TODO
		break;
	case eMenuState::test:
		m_app->setCameraType(_cameraOrbit);
		break;
	default:
		assert(0 && "unreachable");
	}
}

void AppStage_HMDModelCalibration::update_tracker_video()
{
	if (m_renderTrackerIter != m_trackerViews.end())
	{
		// Render the latest from the currently active tracker
		if (m_renderTrackerIter->second.trackerView->pollVideoStream())
		{
			m_renderTrackerIter->second.textureAsset->copyBufferIntoTexture(
				m_renderTrackerIter->second.trackerView->getVideoFrameBuffer());
		}
	}
}

void AppStage_HMDModelCalibration::render_tracker_video()
{
	if (m_renderTrackerIter != m_trackerViews.end() &&
		m_renderTrackerIter->second.textureAsset != nullptr)
	{
		drawFullscreenTexture(m_renderTrackerIter->second.textureAsset->texture_id);
	}
}

void AppStage_HMDModelCalibration::go_next_tracker()
{
	const int trackerCount = static_cast<int>(m_trackerViews.size());

	if (trackerCount > 1)
	{
		m_renderTrackerIndex = (m_renderTrackerIndex + 1) % trackerCount;

		// Find the tracker iterator that corresponds to the render index we want to show
		for (t_tracker_state_map_iterator iter = m_trackerViews.begin(); iter != m_trackerViews.end(); ++iter)
		{
			if (iter->second.listIndex == m_renderTrackerIndex)
			{
				m_renderTrackerIter = iter;
			}
		}
	}
}

void AppStage_HMDModelCalibration::go_previous_tracker()
{
	const int trackerCount = static_cast<int>(m_trackerViews.size());

	if (trackerCount > 1)
	{
		m_renderTrackerIndex = (m_renderTrackerIndex + trackerCount - 1) % trackerCount;

		// Find the tracker iterator that corresponds to the render index we want to show
		for (t_tracker_state_map_iterator iter = m_trackerViews.begin(); iter != m_trackerViews.end(); ++iter)
		{
			if (iter->second.listIndex == m_renderTrackerIndex)
			{
				m_renderTrackerIter = iter;
			}
		}
	}
}

int AppStage_HMDModelCalibration::get_tracker_count() const
{
	return static_cast<int>(m_trackerViews.size());
}

int AppStage_HMDModelCalibration::get_render_tracker_index() const
{
	return m_renderTrackerIndex;
}

class ClientTrackerView *AppStage_HMDModelCalibration::get_render_tracker_view() const
{
	return (m_trackerViews.size() > 0) ? m_renderTrackerIter->second.trackerView : nullptr;
}

void AppStage_HMDModelCalibration::release_devices()
{
	//###HipsterSloth $REVIEW Do we care about canceling in-flight requests?

	if (m_hmdView != nullptr)
	{
		ClientPSMoveAPI::eat_response(ClientPSMoveAPI::stop_hmd_data_stream(m_hmdView));
		ClientPSMoveAPI::free_hmd_view(m_hmdView);
		m_hmdView = nullptr;
	}

	for (t_tracker_state_map_iterator iter = m_trackerViews.begin(); iter != m_trackerViews.end(); ++iter)
	{
		TrackerState &trackerState = iter->second;

		if (trackerState.textureAsset != nullptr)
		{
			delete trackerState.textureAsset;
		}

		if (trackerState.trackerView != nullptr)
		{
			trackerState.trackerView->closeVideoStream();

			ClientPSMoveAPI::eat_response(ClientPSMoveAPI::stop_tracker_data_stream(trackerState.trackerView));
			ClientPSMoveAPI::free_tracker_view(trackerState.trackerView);
		}
	}

	m_trackerViews.clear();
	m_pendingTrackerStartCount = 0;

	m_renderTrackerIndex = 0;
	m_renderTrackerIter = m_trackerViews.end();
}

void AppStage_HMDModelCalibration::request_exit_to_app_stage(const char *app_stage_name)
{
	release_devices();

	m_app->setAppStage(app_stage_name);
}

void AppStage_HMDModelCalibration::request_hmd_list()
{
	if (m_menuState != AppStage_HMDModelCalibration::pendingHmdListRequest)
	{
		m_menuState = AppStage_HMDModelCalibration::pendingHmdListRequest;

		// Request a list of controllers back from the server
		ClientPSMoveAPI::register_callback(
			ClientPSMoveAPI::get_hmd_list(),
			AppStage_HMDModelCalibration::handle_hmd_list_response, this);
	}
}

void AppStage_HMDModelCalibration::handle_hmd_list_response(
	const ClientPSMoveAPI::ResponseMessage *response_message,
	void *userdata)
{
	const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_message->opaque_response_handle);
	const PSMoveProtocol::Request *request = GET_PSMOVEPROTOCOL_REQUEST(response_message->opaque_request_handle);

	AppStage_HMDModelCalibration *thisPtr = static_cast<AppStage_HMDModelCalibration *>(userdata);

	const ClientPSMoveAPI::eClientPSMoveResultCode ResultCode = response_message->result_code;


	switch (ResultCode)
	{
	case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
	{
		assert(response_message->payload_type == ClientPSMoveAPI::_responsePayloadType_HMDList);
		const ClientPSMoveAPI::ResponsePayload_HMDList *hmd_list =
			&response_message->payload.hmd_list;

		int trackedHmdId = thisPtr->m_overrideHmdId;

		if (trackedHmdId == -1)
		{
			for (int list_index = 0; list_index < hmd_list->count; ++list_index)
			{
				if (hmd_list->hmd_type[list_index] == ClientHMDView::Morpheus)
				{
					trackedHmdId = hmd_list->hmd_id[list_index];
					break;
				}
			}
		}

		if (trackedHmdId != -1)
		{
			thisPtr->request_start_hmd_stream(trackedHmdId);
		}
		else
		{
			thisPtr->setState(AppStage_HMDModelCalibration::failedHmdListRequest);
		}
	} break;

	case ClientPSMoveAPI::_clientPSMoveResultCode_error:
	case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
		{
			thisPtr->setState(AppStage_HMDModelCalibration::failedHmdListRequest);
		} break;
	}
}

void AppStage_HMDModelCalibration::request_start_hmd_stream(int HmdID)
{
	// Allocate a hmd view to track HMD state
	assert(m_hmdView == nullptr);
	m_hmdView = ClientPSMoveAPI::allocate_hmd_view(HmdID);

	// Start receiving data from the controller
	setState(AppStage_HMDModelCalibration::pendingHmdStartRequest);
	ClientPSMoveAPI::register_callback(
		ClientPSMoveAPI::start_hmd_data_stream(
			m_hmdView,
			ClientPSMoveAPI::includePositionData | ClientPSMoveAPI::includeCalibratedSensorData | ClientPSMoveAPI::includeRawTrackerData),
		AppStage_HMDModelCalibration::handle_start_hmd_response, this);
}

void AppStage_HMDModelCalibration::handle_start_hmd_response(
	const ClientPSMoveAPI::ResponseMessage *response_message,
	void *userdata)
{
	AppStage_HMDModelCalibration *thisPtr = static_cast<AppStage_HMDModelCalibration *>(userdata);

	const ClientPSMoveAPI::eClientPSMoveResultCode ResultCode = response_message->result_code;

	switch (ResultCode)
	{
	case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
	{
		thisPtr->request_tracker_list();
	} break;

	case ClientPSMoveAPI::_clientPSMoveResultCode_error:
	case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
	{
		thisPtr->setState(AppStage_HMDModelCalibration::failedHmdStartRequest);
	} break;
	}
}

void AppStage_HMDModelCalibration::request_tracker_list()
{
	if (m_menuState != eMenuState::pendingTrackerListRequest)
	{
		setState(eMenuState::pendingTrackerListRequest);

		// Tell the psmove service that we we want a list of trackers connected to this machine
		ClientPSMoveAPI::register_callback(
			ClientPSMoveAPI::get_tracker_list(),
			AppStage_HMDModelCalibration::handle_tracker_list_response, this);
	}
}

void AppStage_HMDModelCalibration::handle_tracker_list_response(
	const ClientPSMoveAPI::ResponseMessage *response_message,
	void *userdata)
{
	AppStage_HMDModelCalibration *thisPtr = static_cast<AppStage_HMDModelCalibration *>(userdata);

	switch (response_message->result_code)
	{
	case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
	{
		assert(response_message->payload_type == ClientPSMoveAPI::_responsePayloadType_TrackerList);
		const ClientPSMoveAPI::ResponsePayload_TrackerList &tracker_list = response_message->payload.tracker_list;

		for (int tracker_index = 0; tracker_index < tracker_list.count; ++tracker_index)
		{
			const ClientTrackerInfo *TrackerInfo = &tracker_list.trackers[tracker_index];

			thisPtr->request_tracker_start_stream(TrackerInfo, tracker_index);
		}
	} break;

	case ClientPSMoveAPI::_clientPSMoveResultCode_error:
	case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
	{
		thisPtr->setState(eMenuState::failedTrackerListRequest);
	} break;
	}
}

void AppStage_HMDModelCalibration::request_tracker_start_stream(
	const ClientTrackerInfo *TrackerInfo,
	int listIndex)
{
	TrackerState trackerState;

	setState(eMenuState::pendingTrackerStartRequest);

	// Allocate a new tracker view
	trackerState.listIndex = listIndex;
	trackerState.trackerView = ClientPSMoveAPI::allocate_tracker_view(*TrackerInfo);
	trackerState.textureAsset = nullptr;

	// Add the tracker to the list of trackers we're monitoring
	assert(m_trackerViews.find(TrackerInfo->tracker_id) == m_trackerViews.end());
	m_trackerViews.insert(t_id_tracker_state_pair(TrackerInfo->tracker_id, trackerState));

	// Increment the number of requests we're waiting to get back
	++m_pendingTrackerStartCount;

	// Request data to start streaming to the tracker
	ClientPSMoveAPI::register_callback(
		ClientPSMoveAPI::start_tracker_data_stream(trackerState.trackerView),
		AppStage_HMDModelCalibration::handle_tracker_start_stream_response, this);
}

void AppStage_HMDModelCalibration::handle_tracker_start_stream_response(
	const ClientPSMoveAPI::ResponseMessage *response_message,
	void *userdata)
{
	AppStage_HMDModelCalibration *thisPtr = static_cast<AppStage_HMDModelCalibration *>(userdata);

	switch (response_message->result_code)
	{
	case ClientPSMoveAPI::_clientPSMoveResultCode_ok:
	{
		// Get the tracker ID this request was for
		const PSMoveProtocol::Request *request = GET_PSMOVEPROTOCOL_REQUEST(response_message->opaque_request_handle);
		const int tracker_id = request->request_start_tracker_data_stream().tracker_id();

		// Get the tracker state associated with the tracker id
		t_tracker_state_map_iterator trackerStateEntry = thisPtr->m_trackerViews.find(tracker_id);
		assert(trackerStateEntry != thisPtr->m_trackerViews.end());

		// The context holds everything a handler needs to evaluate a response
		TrackerState &trackerState = trackerStateEntry->second;

		// Open the shared memory that the video stream is being written to
		if (trackerState.trackerView->openVideoStream())
		{
			// Create a texture to render the video frame to
			trackerState.textureAsset = new TextureAsset();
			trackerState.textureAsset->init(
				trackerState.trackerView->getVideoFrameWidth(),
				trackerState.trackerView->getVideoFrameHeight(),
				GL_RGB, // texture format
				GL_BGR, // buffer format
				nullptr);
		}

		// See if this was the last tracker we were waiting to get a response from
		--thisPtr->m_pendingTrackerStartCount;
		if (thisPtr->m_pendingTrackerStartCount <= 0)
		{
			thisPtr->handle_all_devices_ready();
		}
	} break;

	case ClientPSMoveAPI::_clientPSMoveResultCode_error:
	case ClientPSMoveAPI::_clientPSMoveResultCode_canceled:
	{
		thisPtr->setState(eMenuState::failedTrackerStartRequest);
	} break;
	}
}

void AppStage_HMDModelCalibration::handle_all_devices_ready()
{
	if (!m_bBypassCalibration)
	{
		setState(eMenuState::verifyTrackers);
	}
	else
	{
		setState(eMenuState::test);
	}
}

//-- private methods -----
static void drawHMD(ClientHMDView *hmdView, const glm::mat4 &transform)
{
	switch (hmdView->GetHmdViewType())
	{
	case ClientHMDView::Morpheus:
		drawMorpheusModel(transform);
		break;
	}
}
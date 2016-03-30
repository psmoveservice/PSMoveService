//-- includes -----
#include "ServerRequestHandler.h"

#include "BluetoothRequests.h"
#include "ControllerManager.h"
#include "DeviceManager.h"
#include "DeviceEnumerator.h"
#include "HMDManager.h"
#include "MathEigen.h"
#include "OrientationFilter.h"
#include "PS3EyeTracker.h"
#include "PSMoveController.h"
#include "PSMoveProtocol.pb.h"
#include "ServerControllerView.h"
#include "ServerDeviceView.h"
#include "ServerNetworkManager.h"
#include "ServerTrackerView.h"
#include "ServerHMDView.h"
#include "ServerLog.h"
#include "ServerUtility.h"
#include "TrackerManager.h"

#include <cassert>
#include <bitset>
#include <map>
#include <boost/shared_ptr.hpp>

//-- pre-declarations -----
class ServerRequestHandlerImpl;
typedef boost::shared_ptr<ServerRequestHandlerImpl> ServerRequestHandlerImplPtr;

//-- definitions -----
struct RequestConnectionState
{
    int connection_id;
    std::bitset<ControllerManager::k_max_devices> active_controller_streams;
    std::bitset<TrackerManager::k_max_devices> active_tracker_streams;
    std::bitset<HMDManager::k_max_devices> active_hmd_streams;
    AsyncBluetoothRequest *pending_bluetooth_request;
    ControllerStreamInfo active_controller_stream_info[ControllerManager::k_max_devices];
    TrackerStreamInfo active_tracker_stream_info[TrackerManager::k_max_devices];
    HMDStreamInfo active_hmd_stream_info[HMDManager::k_max_devices];

    RequestConnectionState()
        : connection_id(-1)
        , active_controller_streams()
        , active_tracker_streams()
        , active_hmd_streams()
        , pending_bluetooth_request(nullptr)
    {
        for (int index = 0; index < ControllerManager::k_max_devices; ++index)
        {
            active_controller_stream_info[index].Clear();
        }

        for (int index = 0; index < TrackerManager::k_max_devices; ++index)
        {
            active_tracker_stream_info->Clear();
        }

        for (int index = 0; index < HMDManager::k_max_devices; ++index)
        {
            active_hmd_stream_info->Clear();
        }
    }
};
typedef boost::shared_ptr<RequestConnectionState> RequestConnectionStatePtr;
typedef std::map<int, RequestConnectionStatePtr> t_connection_state_map;
typedef std::map<int, RequestConnectionStatePtr>::iterator t_connection_state_iter;
typedef std::map<int, RequestConnectionStatePtr>::const_iterator t_connection_state_const_iter;
typedef std::pair<int, RequestConnectionStatePtr> t_id_connection_state_pair;

struct RequestContext
{
    RequestConnectionStatePtr connection_state;
    RequestPtr request;
};

//-- private implementation -----
class ServerRequestHandlerImpl
{
public:
    ServerRequestHandlerImpl(DeviceManager &deviceManager)
        : m_device_manager(deviceManager)
        , m_connection_state_map()
    {
    }

    virtual ~ServerRequestHandlerImpl()
    {
        // Without this we get a warning for deletion:
        // "Delete called on 'class ServerRequestHandlerImpl' that has virtual functions but non-virtual destructor"
    }

    bool any_active_bluetooth_requests() const
    {
        bool any_active= false;

        for (t_connection_state_const_iter iter= m_connection_state_map.begin(); iter != m_connection_state_map.end(); ++iter)
        {
            RequestConnectionStatePtr connection_state= iter->second;

            if (connection_state->pending_bluetooth_request != nullptr)
            {
                any_active= true;
                break;
            }
        }

        return any_active;
    }

    void update()
    {
        for (t_connection_state_iter iter= m_connection_state_map.begin(); iter != m_connection_state_map.end(); ++iter)
        {
            int connection_id= iter->first;
            RequestConnectionStatePtr connection_state= iter->second;

            // Update any asynchronous bluetooth requests
            if (connection_state->pending_bluetooth_request != nullptr)
            {
                bool delete_request;

                connection_state->pending_bluetooth_request->update();

                switch(connection_state->pending_bluetooth_request->getStatusCode())
                {
                case AsyncBluetoothRequest::running:
                    {
                        // Don't delete. Still have work to do
                        delete_request= false;
                    } break;
                case AsyncBluetoothRequest::succeeded:
                    {
                        SERVER_LOG_INFO("ServerRequestHandler") 
                            << "Async bluetooth request(" 
                            << connection_state->pending_bluetooth_request->getDescription() 
                            << ") completed.";
                        delete_request= true;
                    } break;
                case AsyncBluetoothRequest::failed:
                    {
                        SERVER_LOG_ERROR("ServerRequestHandler") 
                            << "Async bluetooth request(" 
                            << connection_state->pending_bluetooth_request->getDescription() 
                            << ") failed!";
                        delete_request= true;
                    } break;
                default:
                    assert(0 && "unreachable");
                }

                if (delete_request)
                {
                    delete connection_state->pending_bluetooth_request;
                    connection_state->pending_bluetooth_request= nullptr;
                }
            }
        }
    }

    ResponsePtr handle_request(int connection_id, RequestPtr request)
    {
        // The context holds everything a handler needs to evaluate a request
        RequestContext context;
        context.request= request;
        context.connection_state= FindOrCreateConnectionState(connection_id);

        // All responses track which request they came from
        PSMoveProtocol::Response *response= new PSMoveProtocol::Response;
        response->set_request_id(request->request_id());

        switch (request->type())
        {
            // Controller Requests
            case PSMoveProtocol::Request_RequestType_GET_CONTROLLER_LIST:
                handle_request__get_controller_list(context, response);
                break;
            case PSMoveProtocol::Request_RequestType_START_CONTROLLER_DATA_STREAM:
                handle_request__start_controller_data_stream(context, response);
                break;
            case PSMoveProtocol::Request_RequestType_STOP_CONTROLLER_DATA_STREAM:
                handle_request__stop_controller_data_stream(context, response);
                break;
            case PSMoveProtocol::Request_RequestType_SET_RUMBLE:
                handle_request__set_rumble(context, response);
                break;
            case PSMoveProtocol::Request_RequestType_RESET_POSE:
                handle_request__reset_pose(context, response);
                break;
            case PSMoveProtocol::Request_RequestType_UNPAIR_CONTROLLER:
                handle_request__unpair_controller(context, response);
                break;
            case PSMoveProtocol::Request_RequestType_PAIR_CONTROLLER:
                handle_request__pair_controller(context, response);
                break;
            case PSMoveProtocol::Request_RequestType_CANCEL_BLUETOOTH_REQUEST:
                handle_request__cancel_bluetooth_request(context, response);
                break;
            case PSMoveProtocol::Request_RequestType_SET_LED_COLOR:
                handle_request__set_led_color(context, response);
                break;
            case PSMoveProtocol::Request_RequestType_SET_MAGNETOMETER_CALIBRATION:
                handle_request__set_magnetometer_calibration(context, response);
                break;

            // Tracker Requests
            case PSMoveProtocol::Request_RequestType_GET_TRACKER_LIST:
                handle_request__get_tracker_list(context, response);
                break;
            case PSMoveProtocol::Request_RequestType_START_TRACKER_DATA_STREAM:
                handle_request__start_tracker_data_stream(context, response);
                break;
            case PSMoveProtocol::Request_RequestType_STOP_TRACKER_DATA_STREAM:
                handle_request__stop_tracker_data_stream(context, response);
                break;

            // HMD Requests
            case PSMoveProtocol::Request_RequestType_GET_HMD_LIST:
                handle_request__get_hmd_list(context, response);
                break;
            case PSMoveProtocol::Request_RequestType_START_HMD_DATA_STREAM:
                handle_request__start_hmd_data_stream(context, response);
                break;
            case PSMoveProtocol::Request_RequestType_STOP_HMD_DATA_STREAM:
                handle_request__stop_hmd_data_stream(context, response);
                break;

            default:
                assert(0 && "Whoops, bad request!");
        }

        return ResponsePtr(response);
    }

    void handle_client_connection_stopped(int connection_id)
    {
        t_connection_state_iter iter= m_connection_state_map.find(connection_id);

        if (iter != m_connection_state_map.end())
        {
            int connection_id= iter->first;
            RequestConnectionStatePtr connection_state= iter->second;

            // Cancel any pending asynchronous bluetooth requests
            if (connection_state->pending_bluetooth_request != nullptr)
            {
                assert(connection_state->pending_bluetooth_request->getStatusCode() == AsyncBluetoothRequest::running);

                connection_state->pending_bluetooth_request->cancel(AsyncBluetoothRequest::connectionClosed);

                delete connection_state->pending_bluetooth_request;
                connection_state->pending_bluetooth_request= nullptr;
            }

            // Halt any shared memory streams this connection has going
            for (int tracker_id = 0; tracker_id < TrackerManager::k_max_devices; ++tracker_id)
            {
                if (connection_state->active_tracker_stream_info[tracker_id].streaming_video_data)
                {
                    m_device_manager.getTrackerViewPtr(tracker_id)->stopSharedMemoryVideoStream();
                }
            }

            // Remove the connection state from the state map
            m_connection_state_map.erase(iter);
        }
    }

    void publish_controller_data_frame(
         ServerControllerView *controller_view, 
         ServerRequestHandler::t_generate_controller_data_frame_for_stream callback)
    {
        int controller_id= controller_view->getDeviceID();

        // Notify any connections that care about the controller update
        for (t_connection_state_iter iter= m_connection_state_map.begin(); iter != m_connection_state_map.end(); ++iter)
        {
            int connection_id= iter->first;
            RequestConnectionStatePtr connection_state= iter->second;

            if (connection_state->active_controller_streams.test(controller_id))
            {
                const ControllerStreamInfo &streamInfo=
                    connection_state->active_controller_stream_info[controller_id];

                // Fill out a data frame specific to this stream using the given callback
                DeviceDataFramePtr data_frame(new PSMoveProtocol::DeviceDataFrame);
                callback(controller_view, &streamInfo, data_frame);

                // Send the controller data frame over the network
                ServerNetworkManager::get_instance()->send_device_data_frame(connection_id, data_frame);
            }
        }
    }

    void publish_tracker_data_frame(
        class ServerTrackerView *tracker_view,
            ServerRequestHandler::t_generate_tracker_data_frame_for_stream callback)
    {
        int tracker_id = tracker_view->getDeviceID();

        // Notify any connections that care about the tracker update
        for (t_connection_state_iter iter = m_connection_state_map.begin(); iter != m_connection_state_map.end(); ++iter)
        {
            int connection_id = iter->first;
            RequestConnectionStatePtr connection_state = iter->second;

            if (connection_state->active_tracker_streams.test(tracker_id))
            {
                const TrackerStreamInfo &streamInfo =
                    connection_state->active_tracker_stream_info[tracker_id];

                // Fill out a data frame specific to this stream using the given callback
                DeviceDataFramePtr data_frame(new PSMoveProtocol::DeviceDataFrame);
                callback(tracker_view, &streamInfo, data_frame);

                // Send the tracker data frame over the network
                ServerNetworkManager::get_instance()->send_device_data_frame(connection_id, data_frame);
            }
        }
    }

    void publish_hmd_data_frame(
        class ServerHMDView *hmd_view,
        ServerRequestHandler::t_generate_hmd_data_frame_for_stream callback)
    {
        int hmd_id = hmd_view->getDeviceID();

        // Notify any connections that care about the tracker update
        for (t_connection_state_iter iter = m_connection_state_map.begin(); iter != m_connection_state_map.end(); ++iter)
        {
            int connection_id = iter->first;
            RequestConnectionStatePtr connection_state = iter->second;

            if (connection_state->active_hmd_streams.test(hmd_id))
            {
                const HMDStreamInfo &streamInfo =
                    connection_state->active_hmd_stream_info[hmd_id];

                // Fill out a data frame specific to this stream using the given callback
                DeviceDataFramePtr data_frame(new PSMoveProtocol::DeviceDataFrame);
                callback(hmd_view, &streamInfo, data_frame);

                // Send the hmd data frame over the network
                ServerNetworkManager::get_instance()->send_device_data_frame(connection_id, data_frame);
            }
        }
    }

protected:
    RequestConnectionStatePtr FindOrCreateConnectionState(int connection_id)
    {
        t_connection_state_iter iter= m_connection_state_map.find(connection_id);
        RequestConnectionStatePtr connection_state;

        if (iter == m_connection_state_map.end())
        {
            connection_state= RequestConnectionStatePtr(new RequestConnectionState());
            connection_state->connection_id= connection_id;

            m_connection_state_map.insert(t_id_connection_state_pair(connection_id, connection_state));
        }
        else
        {
            connection_state= iter->second;
        }

        return connection_state;
    }

    // -- Controller Requests -----
    void handle_request__get_controller_list(
        const RequestContext &context, 
        PSMoveProtocol::Response *response)
    {
        PSMoveProtocol::Response_ResultControllerList* list= response->mutable_result_controller_list();

        for (int controller_id= 0; controller_id < m_device_manager.getControllerViewMaxCount(); ++controller_id)
        {
            ServerControllerViewPtr controller_view= m_device_manager.getControllerViewPtr(controller_id);

            if (controller_view->getIsOpen())
            {
                PSMoveProtocol::Response_ResultControllerList_ControllerInfo *controller_info= list->add_controllers();

                switch(controller_view->getControllerDeviceType())
                {
                case CommonControllerState::PSMove:
                    controller_info->set_controller_type(PSMoveProtocol::PSMOVE);
                    break;
                case CommonControllerState::PSNavi:
                    controller_info->set_controller_type(PSMoveProtocol::PSNAVI);
                    break;
                default:
                    assert(0 && "Unhandled controller type");
                }

                controller_info->set_controller_id(controller_id);
                controller_info->set_connection_type(
                    controller_view->getIsBluetooth()
                    ? PSMoveProtocol::Response_ResultControllerList_ControllerInfo_ConnectionType_BLUETOOTH
                    : PSMoveProtocol::Response_ResultControllerList_ControllerInfo_ConnectionType_USB);            
                controller_info->set_device_path(controller_view->getUSBDevicePath());
                controller_info->set_device_serial(controller_view->getSerial());
                controller_info->set_host_serial(controller_view->getHostBluetoothAddress());
            }
        }

        response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);
    }

    void handle_request__start_controller_data_stream(
        const RequestContext &context, 
        PSMoveProtocol::Response *response)
    {
        const PSMoveProtocol::Request_RequestStartPSMoveDataStream& request=
            context.request->request_start_psmove_data_stream();
        int controller_id= request.controller_id();

        if (ServerUtility::is_index_valid(controller_id, m_device_manager.getControllerViewMaxCount()))
        {
            ControllerStreamInfo &streamInfo=
                context.connection_state->active_controller_stream_info[controller_id];

            // The controller manager will always publish updates regardless of who is listening.
            // All we have to do is keep track of which connections care about the updates.
            context.connection_state->active_controller_streams.set(controller_id, true);

            // Set control flags for the stream
            streamInfo.Clear();
            streamInfo.include_raw_sensor_data= request.include_raw_sensor_data();

            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);
        }
        else
        {
            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
        }
    }

    void handle_request__stop_controller_data_stream(
        const RequestContext &context,
        PSMoveProtocol::Response *response)
    {
        int controller_id= context.request->request_stop_psmove_data_stream().controller_id();

        if (ServerUtility::is_index_valid(controller_id, m_device_manager.getControllerViewMaxCount()))
        {
            context.connection_state->active_controller_streams.set(controller_id, false);
            context.connection_state->active_controller_stream_info[controller_id].Clear();

            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);
        }
        else
        {
            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
        }
    }

    void handle_request__set_rumble(
        const RequestContext &context,
        PSMoveProtocol::Response *response)
    {
        const int controller_id= context.request->request_rumble().controller_id();
        const int rumble_amount= context.request->request_rumble().rumble();

        if (m_device_manager.m_controller_manager->setControllerRumble(controller_id, rumble_amount))
        {
            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);
        }
        else
        {
            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
        }
    }

    void handle_request__reset_pose(
        const RequestContext &context, 
        PSMoveProtocol::Response *response)
    {
        const int controller_id= context.request->reset_pose().controller_id();

        if (m_device_manager.m_controller_manager->resetPose(controller_id))
        {
            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);
        }
        else
        {
            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
        }
    }

    void handle_request__unpair_controller(
        const RequestContext &context, 
        PSMoveProtocol::Response *response)
    {
        const int connection_id= context.connection_state->connection_id;
        const int controller_id= context.request->unpair_controller().controller_id();        

        if (context.connection_state->pending_bluetooth_request == nullptr)
        {
            ServerControllerViewPtr controllerView= m_device_manager.getControllerViewPtr(controller_id);

            context.connection_state->pending_bluetooth_request =
                new AsyncBluetoothUnpairDeviceRequest(connection_id, controllerView);

            std::string description = context.connection_state->pending_bluetooth_request->getDescription();

            if (context.connection_state->pending_bluetooth_request->start())
            {
                SERVER_LOG_INFO("ServerRequestHandler") << "Async bluetooth request(" << description << ") started.";

                response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);
            }
            else
            {
                SERVER_LOG_ERROR("ServerRequestHandler") << "Async bluetooth request(" << description << ") failed to start!";

                delete context.connection_state->pending_bluetooth_request;
                context.connection_state->pending_bluetooth_request = nullptr;

                response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
            }
        }
        else
        {
            SERVER_LOG_ERROR("ServerRequestHandler") 
                << "Can't start unpair request due to existing request: " 
                << context.connection_state->pending_bluetooth_request->getDescription();

            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
        }
    }

    void handle_request__pair_controller(
        const RequestContext &context, 
        PSMoveProtocol::Response *response)
    {
        const int connection_id= context.connection_state->connection_id;
        const int controller_id= context.request->pair_controller().controller_id();        

        if (context.connection_state->pending_bluetooth_request == nullptr)
        {
            ServerControllerViewPtr controllerView= m_device_manager.getControllerViewPtr(controller_id);

            context.connection_state->pending_bluetooth_request = 
                new AsyncBluetoothPairDeviceRequest(connection_id, controllerView);

            if (context.connection_state->pending_bluetooth_request->start())
            {
                SERVER_LOG_INFO("ServerRequestHandler") 
                    << "Async bluetooth request(" 
                    << context.connection_state->pending_bluetooth_request->getDescription() 
                    << ") started.";

                response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);
            }
            else
            {
                SERVER_LOG_ERROR("ServerRequestHandler") 
                    << "Async bluetooth request(" 
                    << context.connection_state->pending_bluetooth_request->getDescription() 
                    << ") failed to start!";

                delete context.connection_state->pending_bluetooth_request;
                context.connection_state->pending_bluetooth_request= nullptr;

                response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
            }
        }
        else
        {
            SERVER_LOG_ERROR("ServerRequestHandler") 
                << "Can't start pair request due to existing request: " 
                << context.connection_state->pending_bluetooth_request->getDescription();

            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
        }
    }

    void handle_request__cancel_bluetooth_request(
        const RequestContext &context, 
        PSMoveProtocol::Response *response)
    {
        const int connection_id= context.connection_state->connection_id;
        const int controller_id= context.request->cancel_bluetooth_request().controller_id();        

        if (context.connection_state->pending_bluetooth_request != nullptr)
        {
            ServerControllerViewPtr controllerView= m_device_manager.getControllerViewPtr(controller_id);

            SERVER_LOG_INFO("ServerRequestHandler") 
                << "Async bluetooth request(" 
                << context.connection_state->pending_bluetooth_request->getDescription() 
                << ") Canceled.";

            context.connection_state->pending_bluetooth_request->cancel(AsyncBluetoothRequest::userRequested);
            
            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);
        }
        else
        {
            SERVER_LOG_ERROR("ServerRequestHandler") << "No active bluetooth operation active";

            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
        }
    }

    void handle_request__set_led_color(
        const RequestContext &context, 
        PSMoveProtocol::Response *response)
    {
        const int connection_id= context.connection_state->connection_id;
        const int controller_id= context.request->set_led_color_request().controller_id();
        const unsigned char r= ServerUtility::int32_to_int8_verify(context.request->set_led_color_request().r());
        const unsigned char g= ServerUtility::int32_to_int8_verify(context.request->set_led_color_request().g());
        const unsigned char b= ServerUtility::int32_to_int8_verify(context.request->set_led_color_request().b());

        ServerControllerViewPtr ControllerView= m_device_manager.getControllerViewPtr(controller_id);

        if (ControllerView && ControllerView->getControllerDeviceType() == CommonDeviceState::PSMove)
        {
            PSMoveController *controller= ControllerView->castChecked<PSMoveController>();

            if (controller->setLED(r, g, b))
            {
                response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);
            }
            else
            {
                response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
            }
        }
        else
        {
            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
        }
    }

    inline void set_magnetometer_config_vector(
        const PSMoveProtocol::FloatVector &source_vector,
        Eigen::Vector3f &target_vector)
    {
        target_vector = Eigen::Vector3f(source_vector.i(), source_vector.j(), source_vector.k());
    }

    void handle_request__set_magnetometer_calibration(
        const RequestContext &context, 
        PSMoveProtocol::Response *response)
    {
        const int controller_id= context.request->set_magnetometer_calibration_request().controller_id();

        ServerControllerViewPtr ControllerView= m_device_manager.getControllerViewPtr(controller_id);

        if (ControllerView && ControllerView->getControllerDeviceType() == CommonDeviceState::PSMove)
        {
            PSMoveController *controller= ControllerView->castChecked<PSMoveController>();
            PSMoveControllerConfig *config= controller->getConfigMutable();

            const PSMoveProtocol::Request_RequestSetMagnetometerCalibration &request= 
                context.request->set_magnetometer_calibration_request();

            set_magnetometer_config_vector(request.ellipse_center(), config->magnetometer_ellipsoid.center);
            set_magnetometer_config_vector(request.ellipse_extents(), config->magnetometer_ellipsoid.extents);
            set_magnetometer_config_vector(request.magnetometer_identity(), config->magnetometer_identity);
            config->magnetometer_ellipsoid.error = request.ellipse_fit_error();

            {
                Eigen::Vector3f basis_x, basis_y, basis_z;

                set_magnetometer_config_vector(request.ellipse_basis_x(), basis_x);
                set_magnetometer_config_vector(request.ellipse_basis_y(), basis_y);
                set_magnetometer_config_vector(request.ellipse_basis_z(), basis_z);

                config->magnetometer_ellipsoid.basis.col(0) = basis_x;
                config->magnetometer_ellipsoid.basis.col(1) = basis_y;
                config->magnetometer_ellipsoid.basis.col(2) = basis_z;
            }

            config->save();

            // Reset the orientation filter state the calibration changed
            ControllerView->getOrientationFilter()->resetFilterState();

            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);
        }
        else
        {
            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
        }
    }

    // -- tracker requests -----
    void handle_request__get_tracker_list(
        const RequestContext &context,
        PSMoveProtocol::Response *response)
    {
        PSMoveProtocol::Response_ResultTrackerList* list = response->mutable_result_tracker_list();

        for (int tracker_id = 0; tracker_id < m_device_manager.getTrackerViewMaxCount(); ++tracker_id)
        {
            ServerTrackerViewPtr tracker_view = m_device_manager.getTrackerViewPtr(tracker_id);

            if (tracker_view->getIsOpen())
            {
                PSMoveProtocol::Response_ResultTrackerList_TrackerInfo *tracker_info = list->add_trackers();

                switch (tracker_view->getTrackerDeviceType())
                {
                case CommonControllerState::PS3EYE:
                    tracker_info->set_tracker_type(PSMoveProtocol::PS3EYE);
                    break;
                default:
                    assert(0 && "Unhandled tracker type");
                }

                switch (tracker_view->getTrackerDriverType())
                {
                case ITrackerInterface::Libusb:
                    tracker_info->set_tracker_driver(PSMoveProtocol::LIBUSB);
                    break;
                case ITrackerInterface::CL:
                    tracker_info->set_tracker_driver(PSMoveProtocol::CL_EYE);
                    break;
                case ITrackerInterface::CLMulti:
                    tracker_info->set_tracker_driver(PSMoveProtocol::CL_EYE_MULTICAM);
                    break;
                    // PSMoveProtocol::ISIGHT?
                case ITrackerInterface::Generic_Webcam:
                    tracker_info->set_tracker_driver(PSMoveProtocol::GENERIC_WEBCAM);
                    break;
                default:
                    assert(0 && "Unhandled tracker type");
                }

                tracker_info->set_tracker_id(tracker_id);
                tracker_info->set_device_path(tracker_view->getUSBDevicePath());
                tracker_info->set_shared_memory_name(tracker_view->getSharedMemoryStreamName());
            }
        }

        response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);
    }

    void handle_request__start_tracker_data_stream(
        const RequestContext &context,
        PSMoveProtocol::Response *response)
    {
        const PSMoveProtocol::Request_RequestStartTrackerDataStream& request =
            context.request->request_start_tracker_data_stream();
        int tracker_id = request.tracker_id();

        if (ServerUtility::is_index_valid(tracker_id, m_device_manager.getTrackerViewMaxCount()))
        {
            ServerTrackerViewPtr tracker_view = m_device_manager.getTrackerViewPtr(tracker_id);

            if (tracker_view->getIsOpen())
            {
                TrackerStreamInfo &streamInfo =
                    context.connection_state->active_tracker_stream_info[tracker_id];

                // The tracker manager will always publish updates regardless of who is listening.
                // All we have to do is keep track of which connections care about the updates.
                context.connection_state->active_tracker_streams.set(tracker_id, true);

                // Set control flags for the stream
                streamInfo.streaming_video_data = true;

                // Increment the number of stream listeners
                tracker_view->startSharedMemoryVideoStream();

                // Return the name of the shared memory block the video frames will be written to
                response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);
            }
            else
            {
                // Device not opened
                response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
            }
        }
        else
        {
            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
        }
    }

    void handle_request__stop_tracker_data_stream(
        const RequestContext &context,
        PSMoveProtocol::Response *response)
    {
        int tracker_id = context.request->request_stop_tracker_data_stream().tracker_id();

        if (ServerUtility::is_index_valid(tracker_id, m_device_manager.getTrackerViewMaxCount()))
        {
            ServerTrackerViewPtr tracker_view = m_device_manager.getTrackerViewPtr(tracker_id);

            if (tracker_view->getIsOpen())
            {
                context.connection_state->active_tracker_streams.set(tracker_id, false);
                context.connection_state->active_tracker_stream_info[tracker_id].Clear();

                // Decrement the number of stream listeners
                tracker_view->stopSharedMemoryVideoStream();

                response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);
            }
            else
            {
                // Device not opened
                response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
            }
        }
        else
        {
            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
        }
    }

    // -- hmd requests -----
    void handle_request__get_hmd_list(
        const RequestContext &context,
        PSMoveProtocol::Response *response)
    {
        PSMoveProtocol::Response_ResultHMDList* list = response->mutable_result_hmd_list();

        for (int hmd_id = 0; hmd_id < m_device_manager.getHMDViewMaxCount(); ++hmd_id)
        {
            ServerHMDViewPtr hmd_view = m_device_manager.getHMDViewPtr(hmd_id);

            if (hmd_view->getIsOpen())
            {
                PSMoveProtocol::Response_ResultHMDList_HMDInfo *hmd_info = list->add_hmd_entries();

                switch (hmd_view->getHMDDeviceType())
                {
                case CommonHMDState::OculusDK2:
                    hmd_info->set_hmd_type(PSMoveProtocol::OculusDK2);
                    break;
                default:
                    assert(0 && "Unhandled tracker type");
                }

                hmd_info->set_hmd_id(hmd_id);
                hmd_info->set_device_path(hmd_view->getUSBDevicePath());
            }
        }

        response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);
    }

    void handle_request__start_hmd_data_stream(
        const RequestContext &context,
        PSMoveProtocol::Response *response)
    {
        const PSMoveProtocol::Request_RequestStartHmdDataStream& request =
            context.request->request_start_hmd_data_stream();
        int hmd_id = request.hmd_id();

        if (ServerUtility::is_index_valid(hmd_id, m_device_manager.getHMDViewMaxCount()))
        {
            ServerHMDViewPtr hmd_view = m_device_manager.getHMDViewPtr(hmd_id);

            if (hmd_view->getIsOpen())
            {
                HMDStreamInfo &streamInfo =
                    context.connection_state->active_hmd_stream_info[hmd_id];

                // The hmd manager will always publish updates regardless of who is listening.
                // All we have to do is keep track of which connections care about the updates.
                context.connection_state->active_hmd_streams.set(hmd_id, true);

                // Set control flags for the stream
                streamInfo.Clear();
                streamInfo.include_raw_sensor_data = request.include_raw_sensor_data();

                // Return the name of the shared memory block the video frames will be written to
                response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);
            }
            else
            {
                // Device not opened
                response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
            }
        }
        else
        {
            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
        }
    }

    void handle_request__stop_hmd_data_stream(
        const RequestContext &context,
        PSMoveProtocol::Response *response)
    {
        int hmd_id = context.request->request_stop_hmd_data_stream().hmd_id();

        if (ServerUtility::is_index_valid(hmd_id, m_device_manager.getHMDViewMaxCount()))
        {
            ServerHMDViewPtr hmd_view = m_device_manager.getHMDViewPtr(hmd_id);

            if (hmd_view->getIsOpen())
            {
                context.connection_state->active_tracker_streams.set(hmd_id, false);
                context.connection_state->active_tracker_stream_info[hmd_id].Clear();

                response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);
            }
            else
            {
                // Device not opened
                response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
            }
        }
        else
        {
            response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_ERROR);
        }
    }

private:
    DeviceManager &m_device_manager;
    t_connection_state_map m_connection_state_map;
};

//-- public interface -----
ServerRequestHandler *ServerRequestHandler::m_instance = NULL;

ServerRequestHandler::ServerRequestHandler(DeviceManager *deviceManager)
    : m_implementation_ptr(new ServerRequestHandlerImpl(*deviceManager))
{

}

ServerRequestHandler::~ServerRequestHandler()
{
    if (m_instance != NULL)
    {
        SERVER_LOG_ERROR("~ServerRequestHandler") << "Request handler deleted without calling shutdown first!";
    }

    delete m_implementation_ptr;
}

bool ServerRequestHandler::any_active_bluetooth_requests() const
{
    return m_implementation_ptr->any_active_bluetooth_requests();
}

bool ServerRequestHandler::startup()
{
    m_instance= this;
    return true;
}

void ServerRequestHandler::update()
{
    return m_implementation_ptr->update();
}

void ServerRequestHandler::shutdown()
{
    m_instance= NULL;
}

ResponsePtr ServerRequestHandler::handle_request(int connection_id, RequestPtr request)
{
    return m_implementation_ptr->handle_request(connection_id, request);
}

void ServerRequestHandler::handle_client_connection_stopped(int connection_id)
{
    return m_implementation_ptr->handle_client_connection_stopped(connection_id);
}

void ServerRequestHandler::publish_controller_data_frame(
    ServerControllerView *controller_view, 
    t_generate_controller_data_frame_for_stream callback)
{
    return m_implementation_ptr->publish_controller_data_frame(controller_view, callback);
}

void ServerRequestHandler::publish_tracker_data_frame(
    class ServerTrackerView *tracker_view,
    t_generate_tracker_data_frame_for_stream callback)
{
    return m_implementation_ptr->publish_tracker_data_frame(tracker_view, callback);
}

void ServerRequestHandler::publish_hmd_data_frame(
    class ServerHMDView *hmd_view,
    t_generate_hmd_data_frame_for_stream callback)
{
    return m_implementation_ptr->publish_hmd_data_frame(hmd_view, callback);
}

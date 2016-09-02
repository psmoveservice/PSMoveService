#include "ClientPSMoveAPI.h"
#include "ClientControllerView.h"
#include <chrono>

#if defined(__linux) || defined (__APPLE__)
#include <unistd.h>
#endif

#ifdef _WIN32
#include <windows.h>
#endif

#define FPS_REPORT_DURATION 500 // ms
#define CONTROLLER_OUTPUT_FILE "controller_stream.csv"

class ControllerOutputStream
{
public:
	ControllerOutputStream(ClientControllerView *controller, const char *filename)
	{
		m_fp = fopen(filename, "wt");
		m_startTime = std::chrono::time_point<std::chrono::high_resolution_clock>();

		if (m_fp != nullptr)
		{
			switch (controller->GetControllerViewType())
			{
			case ClientControllerView::PSMove:
				fprintf(m_fp, "psmove");
				break;
			case ClientControllerView::PSDualShock4:
				fprintf(m_fp, "dualshock4");
				break;
			}

			fprintf(m_fp, "TIME,POS_X,POS_Y,POS_Z,POS_QUAL,ORI_W,ORI_X,ORI_Y,ORI_Z,ORI_QUAL,ACC_X,ACC_Y,ACC_Z,MAG_X,MAG_Y,MAG_Z,GYRO_X,GYRO_Y,GYRO_Z");
		}
	}

	~ControllerOutputStream()
	{
		if (m_fp != nullptr)
		{
			fclose(m_fp);
		}
	}

	void writeControllerState(ClientControllerView *controller)
	{
		if (m_fp != nullptr)
		{
			const PSMoveRawTrackerData trackerData= controller->GetRawTrackerData();

			PSMovePosition pos;
			PSMoveQuaternion quat;
			PSMoveFloatVector3 acc;
			PSMoveFloatVector3 gyro;
			PSMoveFloatVector3 mag;
			float position_quality = 1.f; // TODO
			float orientation_quality = 1.f; // TODO

			const std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
			const std::chrono::duration<float, std::milli> time_delta = now - m_startTime;
			const float time = time_delta.count() / 1000.f;

			if (!trackerData.GetPositionOnTrackerId(0, pos))
			{
				pos = PSMovePosition::create(0.f, 0.f, 0.f);
			}

			if (!trackerData.GetOrientationOnTrackerId(0, quat))
			{
				quat = PSMoveQuaternion::create(1.f, 0.f, 0.f, 0.f);
			}

			switch (controller->GetControllerViewType())
			{
			case ClientControllerView::PSDualShock4:
				{
					const PSDualShock4CalibratedSensorData &sensorData = controller->GetPSDualShock4View().GetCalibratedSensorData();
					acc = sensorData.Accelerometer;
					mag = PSMoveFloatVector3::create(0.f, 0.f, 0.f);
					gyro = sensorData.Gyroscope;
				} break;
			case ClientControllerView::PSMove:
				{
					const PSMoveCalibratedSensorData &sensorData = controller->GetPSMoveView().GetCalibratedSensorData();
					acc = sensorData.Accelerometer;
					mag = sensorData.Magnetometer;
					gyro = sensorData.Gyroscope;
				} break;
			}

			fprintf(m_fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f",
				time,
				pos.x, pos.y, pos.z, position_quality,
				quat.w, quat.x, quat.y, quat.z, orientation_quality,
				acc.i, acc.j, acc.k,
				mag.i, mag.j, mag.k,
				gyro.i, gyro.j, gyro.k);
		}
	}

private:
	std::chrono::time_point<std::chrono::high_resolution_clock> m_startTime;
	FILE* m_fp;
};

class PSMoveConsoleClient
{
public:
    PSMoveConsoleClient() 
        : m_keepRunning(true)
        , controller_view(nullptr)
		, output_stream(nullptr)
        , start_stream_request_id(-1)
    {
    }

    int run()
    {
        // Attempt to start and run the client
        try 
        {
            if (startup())
            {
                while (m_keepRunning)
                {
                    update();

                    sleep_millisecond(1);
                }
            }
            else
            {
                std::cerr << "Failed to startup the PSMove Client" << std::endl;
            }
        }
        catch (std::exception& e) 
        {
            std::cerr << e.what() << std::endl;
        }

        // Attempt to shutdown the client
        try 
        {
           shutdown();
        }
        catch (std::exception& e) 
        {
            std::cerr << e.what() << std::endl;
        }
      
        return 0;
   }

private:
    void sleep_millisecond(int sleepMs)
    {
    #if defined(__linux) || defined (__APPLE__)
        usleep(sleepMs * 1000);
    #endif
    #ifdef WINDOWS
        Sleep(sleepMs);
    #endif
    }

    // ClientPSMoveAPI
    void handle_client_psmove_event(ClientPSMoveAPI::eEventType event_type)
    {
        switch (event_type)
        {
        case ClientPSMoveAPI::connectedToService:
            std::cout << "PSMoveConsoleClient - Connected to service" << std::endl;

            // Once created, updates will automatically get pushed into this view
            controller_view= ClientPSMoveAPI::allocate_controller_view(0);

            // Kick off request to start streaming data from the first controller
            start_stream_request_id= 
                ClientPSMoveAPI::start_controller_data_stream(
                    controller_view, 
					ClientPSMoveAPI::includePositionData | ClientPSMoveAPI::includeRawTrackerData | ClientPSMoveAPI::includeCalibratedSensorData);
            break;
        case ClientPSMoveAPI::failedToConnectToService:
            std::cout << "PSMoveConsoleClient - Failed to connect to service" << std::endl;
            m_keepRunning= false;
            break;
        case ClientPSMoveAPI::disconnectedFromService:
            std::cout << "PSMoveConsoleClient - Disconnected from service" << std::endl;
            m_keepRunning= false;
            break;
        case ClientPSMoveAPI::opaqueServiceEvent:
            std::cout << "PSMoveConsoleClient - Opaque service event(%d)" << static_cast<int>(event_type) << std::endl;
            m_keepRunning= false;
            break;
        default:
            assert(0 && "Unhandled event type");
            break;
        }
    }

    void handle_acquire_controller(ClientPSMoveAPI::eClientPSMoveResultCode resultCode)
    {
        if (resultCode == ClientPSMoveAPI::_clientPSMoveResultCode_ok)
        {
            std::cout << "PSMoveConsoleClient - Acquired controller " 
                << controller_view->GetControllerID() << std::endl;

            // Updates will now automatically get pushed into the controller view
			if (controller_view->GetControllerViewType() != ClientControllerView::PSNavi)
			{
				output_stream = new ControllerOutputStream(controller_view, CONTROLLER_OUTPUT_FILE);
			}

            if (controller_view->GetIsCurrentlyTracking())
            {
                PSMovePosition controller_position = controller_view->GetPosition();

                std::cout << "Controller State: " << std::endl;
                std::cout << "  Position (" << controller_position.x << ", " << controller_position.y << ", " << controller_position.z << ")" << std::endl;
            }
        }
        else
        {
            std::cout << "PSMoveConsoleClient - failed to acquire controller " << std::endl;
            m_keepRunning= false;
        }
    }

    // PSMoveConsoleClient
    bool startup()
    {
        bool success= true;

        // Attempt to connect to the server
        if (success)
        {
            if (!ClientPSMoveAPI::startup("localhost", "9512"))
            {
                std::cout << "PSMoveConsoleClient - Failed to initialize the client network manager" << std::endl;
                success= false;
            }
        }

        if (success)
        {
            last_report_fps_timestamp= 
                std::chrono::duration_cast< std::chrono::milliseconds >( 
                    std::chrono::system_clock::now().time_since_epoch() );
        }

        return success;
    }

    void update()
    {
        // Process incoming/outgoing networking requests
        ClientPSMoveAPI::update();

        // Poll events queued up by the call to ClientPSMoveAPI::update()
        ClientPSMoveAPI::Message message;
        while (ClientPSMoveAPI::poll_next_message(&message, sizeof(message)))
        {
            switch (message.payload_type)
            {
            case ClientPSMoveAPI::_messagePayloadType_Response:
                if (start_stream_request_id != -1 &&
                    message.response_data.request_id == start_stream_request_id)
                {
                    handle_acquire_controller(message.response_data.result_code);
                    start_stream_request_id= -1;
                }
                break;
            case ClientPSMoveAPI::_messagePayloadType_Event:
                handle_client_psmove_event(message.event_data.event_type);
                break;
            }
        }

        if (controller_view)
        {
            std::chrono::milliseconds now= 
                std::chrono::duration_cast< std::chrono::milliseconds >( 
                    std::chrono::system_clock::now().time_since_epoch() );
            std::chrono::milliseconds diff= now - last_report_fps_timestamp;

			output_stream->writeControllerState(controller_view);

            if (diff.count() > FPS_REPORT_DURATION && controller_view->GetDataFrameFPS() > 0)
            {
                std::cout << "PSMoveConsoleClient - DataFrame Update FPS: " << controller_view->GetDataFrameFPS() << "FPS" << std::endl;
                last_report_fps_timestamp= now;
            }
        }
    }

    void shutdown()
    {
		if (output_stream != nullptr)
		{
			delete output_stream;
			output_stream = nullptr;
		}

        // Free any allocated controller views
        if (controller_view)
        {
            ClientPSMoveAPI::free_controller_view(controller_view);
            controller_view= nullptr;
        }

        // Close all active network connections
        ClientPSMoveAPI::shutdown();
    }

private:
    bool m_keepRunning;
    ClientControllerView *controller_view;
	ControllerOutputStream *output_stream;
    std::chrono::milliseconds last_report_fps_timestamp;
    ClientPSMoveAPI::t_request_id start_stream_request_id;
};

int main(int argc, char *argv[])
{   
    PSMoveConsoleClient app;

    // app instantiation
    return app.run();
}


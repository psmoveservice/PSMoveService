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

class PSMoveConsoleClient
{
public:
    PSMoveConsoleClient() 
        : m_keepRunning(true)
        , controller_view(nullptr)
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
                    controller_view, ClientPSMoveAPI::includePositionData);
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

            if (controller_view->GetControllerViewType() == ClientControllerView::PSMove)
            {
                const ClientPSMoveView &PSMoveView= controller_view->GetPSMoveView();
                
                if (PSMoveView.GetIsCurrentlyTracking())
                {
                    PSMovePosition controller_position= PSMoveView.GetPosition();

                    std::cout << "Controller State: " << std::endl;
                    std::cout << "  Position (" << controller_position.x << ", " << controller_position.y << ", " << controller_position.z << ")" << std::endl;
                }
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

            if (diff.count() > FPS_REPORT_DURATION && controller_view->GetDataFrameFPS() > 0)
            {
                std::cout << "PSMoveConsoleClient - DataFrame Update FPS: " << controller_view->GetDataFrameFPS() << "FPS" << std::endl;
                last_report_fps_timestamp= now;
            }
        }
    }

    void shutdown()
    {
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
    std::chrono::milliseconds last_report_fps_timestamp;
    ClientPSMoveAPI::t_request_id start_stream_request_id;
};

int main(int argc, char *argv[])
{   
    PSMoveConsoleClient app;

    // app instantiation
    return app.run();
}


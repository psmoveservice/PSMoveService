#include "ClientPSMoveAPI.h"
#include "ClientControllerView.h"

#ifdef __linux
#include <unistd.h>
#endif
#ifdef _WIN32
#include <windows.h>
#endif

class PSMoveConsoleClient
{
public:
    PSMoveConsoleClient() 
        : m_keepRunning(true)
        , controller_view()
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
    #ifdef LINUX
        usleep(sleepMs * 1000);
    #endif
    #ifdef WINDOWS
        Sleep(sleepMs);
    #endif
    }

    // ClientPSMoveAPI
    void handle_client_psmove_event(ClientPSMoveAPI::eClientPSMoveAPIEvent event_type)
    {
        switch (event_type)
        {
        case ClientPSMoveAPI::connectedToService:
            std::cout << "PSMoveConsoleClient - Connected to service" << std::endl;

            // Once created, updates will automatically get pushed into this view
            controller_view= ClientPSMoveAPI::allocate_controller_view(0);

            // Kick off request to start streaming data from the first controller
            ClientPSMoveAPI::start_controller_data_stream(
                controller_view, 
                std::bind(&PSMoveConsoleClient::handle_acquire_controller, this, std::placeholders::_1));
            break;
        case ClientPSMoveAPI::failedToConnectToService:
            std::cout << "PSMoveConsoleClient - Failed to connect to service" << std::endl;
            m_keepRunning= false;
            break;
        case ClientPSMoveAPI::disconnectedFromService:
            std::cout << "PSMoveConsoleClient - Disconnected from service" << std::endl;
            m_keepRunning= false;
            break;
        default:
            break;
        }
    }

    void handle_acquire_controller(ClientPSMoveAPI::eClientPSMoveResultCode resultCode)
    {
        if (resultCode == ClientPSMoveAPI::_clientPSMoveResultCode_ok)
        {
            std::cout << "PSMoveConsoleClient - Acquired controller " 
                << controller_view->GetPSMoveID() << std::endl;

            // Updates will now automatically get pushed into the controller view

            if (controller_view->GetIsCurrentlyTracking())
            {
                PSMoveVector3 controller_position= controller_view->GetPosition();

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
            if (!ClientPSMoveAPI::startup(
                    "localhost", "9512", 
                    std::bind(&PSMoveConsoleClient::handle_client_psmove_event, this, std::placeholders::_1)))
            {
                std::cout << "PSMoveConsoleClient - Failed to initialize the client network manager" << std::endl;
                success= false;
            }
        }

        return success;
    }

    void update()
    {
        // Process incoming/outgoing networking requests
        ClientPSMoveAPI::update();
    }

    void shutdown()
    {
        // Free any allocated controller views
        if (controller_view)
        {
            ClientPSMoveAPI::free_controller_view(controller_view);
            controller_view.reset();
        }

        // Close all active network connections
        ClientPSMoveAPI::shutdown();
    }

private:
    bool m_keepRunning;
    ClientControllerViewPtr controller_view;
};

int main(int argc, char *argv[])
{   
    PSMoveConsoleClient app;

    // app instantiation
    return app.run();
}


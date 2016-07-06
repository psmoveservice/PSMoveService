#include "PSMoveClient_CAPI.h"
#include <iostream>
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

    // PSMoveConsoleClient
    bool startup()
    {
        bool success= true;

        // Attempt to connect to the server
        if (success)
        {
            if (PSM_Initialize("localhost", "9512") != PSMResult_Success)
            {
                std::cout << "PSMoveConsoleClient::startup() - Failed to initialize the client network manager" << std::endl;
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
        std::chrono::milliseconds now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        std::chrono::milliseconds diff= now - last_report_fps_timestamp;
        if (diff.count() > 10000)
        {
            m_keepRunning = false;
        }
        
            

            /*
            if (diff.count() > FPS_REPORT_DURATION && controller_view->GetDataFrameFPS() > 0)
            {
                std::cout << "PSMoveConsoleClient - DataFrame Update FPS: " << controller_view->GetDataFrameFPS() << "FPS" << std::endl;
                last_report_fps_timestamp= now;
            }
             */
    }

    void shutdown()
    {
        PSM_Shutdown();
    }

private:
    bool m_keepRunning;
    std::chrono::milliseconds last_report_fps_timestamp;
};

int main(int argc, char *argv[])
{   
    PSMoveConsoleClient app;

    // app instantiation
    return app.run();
}


#include "PSMoveClient_CAPI.h"
#include "ClientConstants.h"
#include <iostream>
#include <iomanip>
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
        memset(&controllerList, 0, sizeof(PSMControllerList));
        memset(&trackerList, 0, sizeof(PSMTrackerList));
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

                    _PAUSE(1);
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
    // PSMoveConsoleClient
    bool startup()
    {
        bool success= true;

        // Attempt to connect to the server
        if (success)
        {
            if (PSM_Initialize("localhost", "9512") == PSMResult_Success)
            {
                std::cout << "PSMoveConsoleClient::startup() - Initialized client version - " << PSM_GetVersionString() << std::endl;
            }
            else
            {
                std::cout << "PSMoveConsoleClient::startup() - Failed to initialize the client network manager" << std::endl;
                success= false;
            }

            if (success)
            {
                PSM_GetControllerList(&controllerList);
                std::cout << "Found " << controllerList.count << " controllers." << std::endl;
                for (int trkr_ix=0; trkr_ix<controllerList.count; ++trkr_ix) 
                {
                    const char *controller_type= "NONE";

                    switch (controllerList.controller_type[trkr_ix])
                    {
                    case PSMController_Move:
                        controller_type= "PSMove";
                        break;
                    case PSMController_Navi:
                        controller_type= "PSNavi";
                        break;
                    }

                    std::cout << "  Controller ID: " << controllerList.controller_id[trkr_ix] << " is a " << controller_type << std::endl;
                }


                PSM_GetTrackerList(&trackerList);
                std::cout << "Found " << trackerList.count << " trackers." << std::endl;
                for (int trkr_ix=0; trkr_ix<trackerList.count; ++trkr_ix) 
                {
                    const char *tracker_type= "NONE";

                    switch (trackerList.trackers[trkr_ix].tracker_type)
                    {
                    case PSMTracker_PS3Eye:
                        tracker_type= "PS3Eye";
                        break;
                    }

                    std::cout << "  Tracker ID: " << trackerList.trackers[trkr_ix].tracker_id << " is a " << tracker_type << std::endl;
                }
            
                // Register as listener and start stream for each controller
                unsigned int data_stream_flags = PSMControllerDataStreamFlags::PSMStreamFlags_includePositionData |
                PSMControllerDataStreamFlags::PSMStreamFlags_includePhysicsData | PSMControllerDataStreamFlags::PSMStreamFlags_includeRawSensorData |
                PSMControllerDataStreamFlags::PSMStreamFlags_includeRawTrackerData;
                PSMResult result;
                for (int trkr_ix=0; trkr_ix<controllerList.count; ++trkr_ix) {
                    result = PSM_AllocateControllerListener(controllerList.controller_id[trkr_ix]);
                    result = PSM_StartControllerDataStream(controllerList.controller_id[trkr_ix], data_stream_flags);
                }
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
        
        // Polls events and updates controller state
        PSMResult result = PSM_Update();

        PSMController *controller0= PSM_GetController(controllerList.controller_id[0]);
        PSMRawSensorData rawsens = controller0->ControllerState.PSMoveState.RawSensorData;
        
        std::cout << "Controller 0 (AGMXYZ):  ";
        
        std::cout << std::setw(12) << std::right << std::setprecision(6) << rawsens.Accelerometer.x;
        std::cout << std::setw(12) << std::right << std::setprecision(6) << rawsens.Accelerometer.y;
        std::cout << std::setw(12) << std::right << std::setprecision(6) << rawsens.Accelerometer.z;
        
        std::cout << std::setw(12) << std::right << std::setprecision(6) << rawsens.Gyroscope.x;
        std::cout << std::setw(12) << std::right << std::setprecision(6) << rawsens.Gyroscope.y;
        std::cout << std::setw(12) << std::right << std::setprecision(6) << rawsens.Gyroscope.z;
        
        std::cout << std::setw(5) << std::right << rawsens.Magnetometer.x;
        std::cout << std::setw(5) << std::right << rawsens.Magnetometer.y;
        std::cout << std::setw(5) << std::right << rawsens.Magnetometer.z;
        
        PSMVector3f position = controller0->ControllerState.PSMoveState.RawTrackerData.RelativePositions[0];
        std::cout << std::setw(12) << std::right << std::setprecision(6) << position.x;
        std::cout << std::setw(12) << std::right << std::setprecision(6) << position.y;
        std::cout << std::setw(12) << std::right << std::setprecision(6) << position.z;
        
        std::cout << std::endl;
        
        if (controller0->ControllerState.PSMoveState.CrossButton != PSMButtonState_UP)
        {
            m_keepRunning = false;
        }
    }

    void shutdown()
    {
        for (int ctrl_ix=0; ctrl_ix < controllerList.count; ++ctrl_ix)
        {
            PSM_StopControllerDataStream(controllerList.controller_id[ctrl_ix]);
            PSM_FreeControllerListener(controllerList.controller_id[ctrl_ix]);
        }
        PSM_Shutdown();
    }

private:
    bool m_keepRunning;
    std::chrono::milliseconds last_report_fps_timestamp;
    PSMControllerList controllerList;
    PSMTrackerList trackerList;
};

int main(int argc, char *argv[])
{   
    PSMoveConsoleClient app;

    // app instantiation
    return app.run();
}


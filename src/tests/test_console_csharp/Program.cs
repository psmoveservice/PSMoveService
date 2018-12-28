using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using PSMoveService;

namespace test_console_csharp
{
    class PSMoveConsoleClient
    {
        private DateTime last_report_fps_timestamp;
        private bool keepRunning;
        public PSMControllerList controllerList;
        public PSMTrackerList trackerList;
        public PSMHmdList hmdList;

        public PSMoveConsoleClient()
        {
            keepRunning = true;
            controllerList = new PSMControllerList();
            trackerList = new PSMTrackerList();
            hmdList = new PSMHmdList();
        }

        public void run()
        {
            // Attempt to start and run the client
            try
            {
                if (startup())
                {
                    while (keepRunning)
                    {
                        update();

                        System.Threading.Thread.Sleep(1);
                    }
                }
                else
                {
                    System.Console.WriteLine("Failed to startup the PSMove Client");
                }
            }
            catch (System.Exception e)
            {
                System.Console.WriteLine(e.ToString());
            }

            // Attempt to shutdown the client
            try
            {
                shutdown();
            }
            catch (System.Exception e)
            {
                System.Console.WriteLine(e.ToString());
            }
        }

        private bool startup()
        {
            bool success = true;

            // Attempt to connect to the server
            if (success) 
            {
                if (PSMoveClient.PSM_Initialize(
                        PSMoveClient.PSMOVESERVICE_DEFAULT_ADDRESS,
                        PSMoveClient.PSMOVESERVICE_DEFAULT_PORT,
                        PSMoveClient.PSM_DEFAULT_TIMEOUT) == PSMResult.PSMResult_Success) 
                {
                    System.Console.WriteLine(
                        string.Format("PSMoveConsoleClient::startup() - Initialized client version - {0}", 
                        PSMoveClient.PSM_GetClientVersionString()));
                }
                else 
                {
                    System.Console.WriteLine("PSMoveConsoleClient::startup() - Failed to initialize the client network manager");
                    success = false;
                }

                if (success) 
                {
                    rebuildControllerList();
                    rebuildTrackerList();
                    rebuildHmdList();

                    // Register as listener and start stream for each controller
                    uint data_stream_flags =
                        (uint)PSMControllerDataStreamFlags.PSMStreamFlags_includePositionData |
                        (uint)PSMControllerDataStreamFlags.PSMStreamFlags_includePhysicsData |
                        (uint)PSMControllerDataStreamFlags.PSMStreamFlags_includeCalibratedSensorData |
                        (uint)PSMControllerDataStreamFlags.PSMStreamFlags_includeRawTrackerData;

                    if (controllerList.count > 0) 
                    {
                        if (PSMoveClient.PSM_AllocateControllerListener(controllerList.controllers[0].controller_id) != PSMResult.PSMResult_Success)
                        {
                            success = false;
                        }
                        if (success && PSMoveClient.PSM_StartControllerDataStream(controllerList.controllers[0].controller_id, data_stream_flags, PSMoveClient.PSM_DEFAULT_TIMEOUT) != PSMResult.PSMResult_Success)
                        {
                            success = false;
                        }
                    }
                    else
                    {
                        System.Console.WriteLine("PSMoveConsoleClient::startup() - No controllers found.");
                        success = false;
                    }
                }
            }

            if (success)
            {
                last_report_fps_timestamp = DateTime.Now;
            }

            return success;
        }

        void update()
        {
            PSMClientControllerInfo[] controllers = controllerList.controllers;

            DateTime now = DateTime.Now;
            TimeSpan diff = now - last_report_fps_timestamp;

            // Polls events and updates controller state
            if (PSMoveClient.PSM_Update() != PSMResult.PSMResult_Success)
            {
                keepRunning = false;
            }

            // See if we need to rebuild the controller list
            if (keepRunning && PSMoveClient.PSM_HasControllerListChanged()) 
            {
                uint data_stream_flags =
                    (uint)PSMControllerDataStreamFlags.PSMStreamFlags_includePositionData |
                    (uint)PSMControllerDataStreamFlags.PSMStreamFlags_includePhysicsData |
                    (uint)PSMControllerDataStreamFlags.PSMStreamFlags_includeCalibratedSensorData |
                    (uint)PSMControllerDataStreamFlags.PSMStreamFlags_includeRawTrackerData;

                // Stop all controller streams
                PSMoveClient.PSM_StopControllerDataStream(controllers[0].controller_id, PSMoveClient.PSM_DEFAULT_TIMEOUT);

                // Get the current controller list
                rebuildControllerList();

                // Restart the controller streams
                if (controllerList.count > 0)
                {
                    if (PSMoveClient.PSM_StartControllerDataStream(controllers[0].controller_id, data_stream_flags, PSMoveClient.PSM_DEFAULT_TIMEOUT) != PSMResult.PSMResult_Success)
                    {
                        keepRunning = false;
                    }
                }
                else
                {
                    System.Console.WriteLine("PSMoveConsoleClient::startup() - No controllers found.");
                    keepRunning = false;
                }
            }

            // See if we need to rebuild the tracker list
            if (keepRunning && PSMoveClient.PSM_HasTrackerListChanged())
            {
                rebuildTrackerList();
            }

            // See if we need to rebuild the hmd list
            if (keepRunning && PSMoveClient.PSM_HasHMDListChanged())
            {
                rebuildTrackerList();
            }

            // Get the controller data for the first controller
            if (keepRunning)
            {
                PSMController controller0 = PSMoveClient.PSM_GetController(controllers[0].controller_id);
                PSMPSMoveCalibratedSensorData calibsens = controller0.ControllerState.PSMoveState.CalibratedSensorData;
                PSMVector3f position = controller0.ControllerState.PSMoveState.RawTrackerData.RelativePositionCm;

                System.Console.WriteLine(
                    string.Format("Controller 0 (AGMXYZ): {0,-12:F6}{0,-12:F6}{0,-12:F6}{0,-12:F6}{0,-12:F6}{0,-12:F6}{0,-12:F6}{0,-12:F6}{0,-12:F6}{0,-12:F6}{0,-12:F6}{0,-12:F6}",
                        calibsens.Accelerometer.x, calibsens.Accelerometer.y, calibsens.Accelerometer.z,
                        calibsens.Gyroscope.x, calibsens.Gyroscope.y, calibsens.Gyroscope.z,
                        calibsens.Magnetometer.x, calibsens.Magnetometer.y, calibsens.Magnetometer.z,
                        position.x, position.y, position.z));

                if (controller0.ControllerState.PSMoveState.CrossButton != PSMButtonState.PSMButtonState_UP) {
                    keepRunning = false;
                }
            }
        }

        void shutdown()
        {
            PSMClientControllerInfo[] controllers = controllerList.controllers;

            if (controllers.Length > 0)
            {
                PSMoveClient.PSM_StopControllerDataStream(controllers[0].controller_id, PSMoveClient.PSM_DEFAULT_TIMEOUT);
                PSMoveClient.PSM_FreeControllerListener(controllers[0].controller_id);
            }
            // No tracker data streams started
            // No HMD data streams started

            PSMoveClient.PSM_Shutdown();
        }

        private void rebuildControllerList()
        {
            PSMoveClient.PSM_GetControllerList(controllerList, PSMoveClient.PSM_DEFAULT_TIMEOUT);
            PSMClientControllerInfo[] controllers = controllerList.controllers;

            System.Console.WriteLine(string.Format("Found {0} controllers.", controllerList.count));

            for (int cntlr_ix = 0; cntlr_ix < controllers.Length; ++cntlr_ix) {
                string controller_type = "NONE";

                switch (controllers[cntlr_ix].controller_type) {
                    case PSMControllerType.PSMController_Move:
                        controller_type = "PSMove";
                        break;
                    case PSMControllerType.PSMController_Navi:
                        controller_type = "PSNavi";
                        break;
                    case PSMControllerType.PSMController_DualShock4:
                        controller_type = "DualShock4";
                        break;
                    case PSMControllerType.PSMController_Virtual:
                        controller_type = "Virtual";
                        break;
                }

                System.Console.WriteLine(
                    string.Format("  Controller ID: {0} is a {1}",
                        controllerList.controllers[cntlr_ix].controller_id, 
                        controller_type));
            }
        }

        private void rebuildTrackerList()
        {
            PSMoveClient.PSM_GetTrackerList(trackerList, PSMoveClient.PSM_DEFAULT_TIMEOUT);
            PSMClientTrackerInfo[] trackers = trackerList.trackers;

            System.Console.WriteLine(string.Format("Found {0} trackers.", controllerList.count));

            for (int tracker_ix = 0; tracker_ix < trackers.Length; ++tracker_ix) {
                string tracker_type = "NONE";

                switch (trackers[tracker_ix].tracker_type) {
                    case PSMTrackerType.PSMTracker_PS3Eye:
                        tracker_type = "PS3Eye";
                        break;
                }

                System.Console.WriteLine(
                    string.Format("  Tracker ID: {0} is a {1}",
                        trackers[tracker_ix].tracker_id,
                        tracker_type));
            }
        }

        private void rebuildHmdList()
        {
            PSMoveClient.PSM_GetHmdList(hmdList, PSMoveClient.PSM_DEFAULT_TIMEOUT);
            PSMClientHMDInfo[] HMDs = hmdList.hmds;

            System.Console.WriteLine(string.Format("Found {0} HMDs.", controllerList.count));

            for (int hmd_ix = 0; hmd_ix < HMDs.Length; ++hmd_ix) {
                string hmd_type = "NONE";

                switch (hmdList.hmds[hmd_ix].hmd_type) {
                    case PSMHmdType.PSMHmd_Morpheus:
                        hmd_type = "Morpheus";
                        break;
                    case PSMHmdType.PSMHmd_Virtual:
                        hmd_type = "Virtual";
                        break;
                }

                System.Console.WriteLine(
                    string.Format("  v ID: {0} is a {1}",
                        HMDs[hmd_ix].hmd_id,
                        hmd_type));
            }
        }
    }

    class Program
    {
        static void Main(string[] args)
        {
            PSMoveConsoleClient app = new PSMoveConsoleClient();

            app.run();
        }
    }
}

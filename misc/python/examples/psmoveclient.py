import sys, os
sys.path.insert(0, os.path.abspath('..'))

import time
from pypsmove._psmoveclient import ffi, lib
#TODO: Use python csv writer into ../../

# Then try out the functions
result = lib.PSM_Initialize(b"localhost", b"9512", 1000)
if result == 0:
    controllerList = ffi.new("PSMControllerList *")
    result = lib.PSM_GetControllerList(controllerList, 1000)
    if controllerList.count > 0:
        myController = lib.PSM_GetController(controllerList.controller_id[0])
        result = lib.PSM_AllocateControllerListener(myController.ControllerID)
        if result == 0:
            flags = lib.PSMStreamFlags_includePositionData |\
                lib.PSMStreamFlags_includePhysicsData | lib.PSMStreamFlags_includePositionData | \
                lib.PSMStreamFlags_includeCalibratedSensorData | lib.PSMStreamFlags_includeRawTrackerData
            result = lib.PSM_StartControllerDataStream(myController.ControllerID, flags, 1000)
            if result == 0:
                start = time.time()
                old_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                while time.time() - start < 5:
                    now = time.time()
                    result = lib.PSM_Update()
                    a = myController.ControllerState.PSMoveState.CalibratedSensorData.Accelerometer
                    g = myController.ControllerState.PSMoveState.CalibratedSensorData.Gyroscope
                    m = myController.ControllerState.PSMoveState.CalibratedSensorData.Magnetometer
                    sens = [a.x, a.y, a.z, g.x, g.y, g.z, m.x, m.y, m.z]
                    pos = myController.ControllerState.PSMoveState.RawTrackerData.RelativePositions[0]
                    pos = [pos.x, pos.y, pos.z]
                    data = sens+pos
                    if any([data[ix] != old_data[ix] for ix in range(len(data))]):
                        orient = myController.ControllerState.PSMoveState.Pose.Orientation
                        quat = [orient.w, orient.x, orient.y, orient.z]
                        data += quat+[now-start]
                        print('{0:8.3f},{1:8.3f},{2:8.3f},{3:8.3f},{4:8.3f},{5:8.3f},{6:8.3f},{7:8.3f},{8:8.3f},{9:9.3f},{10:9.3f},{11:9.3f},{12:9.3f},{13:9.3f},{14:9.3f},{15:9.3f},{16}'.format(*data))
                        old_data = data
                result = lib.PSM_StopControllerDataStream(0, 1000)
                result = lib.PSM_FreeControllerListener(0)
    result = lib.PSM_Shutdown()
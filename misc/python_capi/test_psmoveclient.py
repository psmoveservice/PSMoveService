import time
from _psmoveclient import ffi, lib

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
                lib.PSMStreamFlags_includePhysicsData | lib.PSMStreamFlags_includePositionData |\
                lib.PSMStreamFlags_includeRawSensorData | lib.PSMStreamFlags_includeRawTrackerData
            result = lib.PSM_StartControllerDataStream(myController.ControllerID, flags, 1000)
            if result == 0:
                start = time.time()
                old_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0.0, 0.0, 0.0]
                while time.time() - start < 5:
                    result = lib.PSM_Update()
                    a = myController.ControllerState.PSMoveState.RawSensorData.Accelerometer
                    g = myController.ControllerState.PSMoveState.RawSensorData.Gyroscope
                    m = myController.ControllerState.PSMoveState.RawSensorData.Magnetometer
                    sens = [a.x, a.y, a.z, g.x, g.y, g.z, m.x, m.y, m.z]
                    pos = myController.ControllerState.PSMoveState.RawTrackerData.RelativePositions[0]
                    pos = [pos.x, pos.y, pos.z]
                    data = sens+pos
                    if any([data[ix] != old_data[ix] for ix in range(len(data))]):
                        print('{0:8.3f} {1:8.3f} {2:8.3f} {3:8.3f} {4:8.3f} {5:8.3f} {6:5d} {7:5d} {8:5d} {9:9.3f} {10:9.3f} {11:9.3f}'.format(*data))
                        old_data = data
                result = lib.PSM_StopControllerDataStream(0, 1000)
                result = lib.PSM_FreeControllerListener(0)
    result = lib.PSM_Shutdown()
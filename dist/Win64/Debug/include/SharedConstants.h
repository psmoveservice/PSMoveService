#ifndef SHARED_CONSTANTS_H
#define SHARED_CONSTANTS_H

#define PSMOVESERVICE_DEFAULT_ADDRESS   "localhost"
#define PSMOVESERVICE_DEFAULT_PORT      "9512"

#define MAX_OUTPUT_DATA_FRAME_MESSAGE_SIZE 500
#define MAX_INPUT_DATA_FRAME_MESSAGE_SIZE 64

// See ControllerManager.h in PSMoveService
#define PSMOVESERVICE_MAX_CONTROLLER_COUNT  5

// See TrackerManager.h in PSMoveService
#define PSMOVESERVICE_MAX_TRACKER_COUNT  8

// See HMDManager.h in PSMoveService
#define PSMOVESERVICE_MAX_HMD_COUNT  4

// The max number of axes allowed on a virtual controller
#define PSM_MAX_VIRTUAL_CONTROLLER_AXES  32

// The max number of buttons allowed on a virtual controller
#define PSM_MAX_VIRTUAL_CONTROLLER_BUTTONS  32

 
#endif  // SHARED_CONSTANTS_H

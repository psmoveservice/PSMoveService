/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// This library is part of CL-Eye SDK
// It allows the use of multiple CL-Eye cameras in your own applications
//
// For updates and file downloads go to: http://codelaboratories.com
//
// Copyright 2008-2012 (c) Code Laboratories, Inc. All rights reserved.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once
#include <windows.h>
#define IMPORT(type) extern "C" __declspec(dllimport)## type __cdecl

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CLEyeMulticam Camera API
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// camera instance type
typedef void *CLEyeCameraInstance;

// camera modes
typedef enum
{ 
	CLEYE_MONO_PROCESSED,
	CLEYE_COLOR_PROCESSED,
	CLEYE_MONO_RAW,
	CLEYE_COLOR_RAW,
	CLEYE_BAYER_RAW
}CLEyeCameraColorMode;

// camera resolution
typedef enum
{ 
	CLEYE_QVGA,
	CLEYE_VGA
}CLEyeCameraResolution;

// camera parameters
typedef enum
{
	// camera sensor parameters
	CLEYE_AUTO_GAIN,			// [false, true]
	CLEYE_GAIN,					// [0, 79]
	CLEYE_AUTO_EXPOSURE,		// [false, true]
	CLEYE_EXPOSURE,				// [0, 511]
	CLEYE_AUTO_WHITEBALANCE,	// [false, true]
	CLEYE_WHITEBALANCE_RED,		// [0, 255]
	CLEYE_WHITEBALANCE_GREEN,	// [0, 255]
	CLEYE_WHITEBALANCE_BLUE,	// [0, 255]
	// camera linear transform parameters (valid for CLEYE_MONO_PROCESSED, CLEYE_COLOR_PROCESSED modes)
	CLEYE_HFLIP,				// [false, true]
	CLEYE_VFLIP,				// [false, true]
	CLEYE_HKEYSTONE,			// [-500, 500]
	CLEYE_VKEYSTONE,			// [-500, 500]
	CLEYE_XOFFSET,				// [-500, 500]
	CLEYE_YOFFSET,				// [-500, 500]
	CLEYE_ROTATION,				// [-500, 500]
	CLEYE_ZOOM,					// [-500, 500]
	// camera non-linear transform parameters (valid for CLEYE_MONO_PROCESSED, CLEYE_COLOR_PROCESSED modes)
	CLEYE_LENSCORRECTION1,		// [-500, 500]
	CLEYE_LENSCORRECTION2,		// [-500, 500]
	CLEYE_LENSCORRECTION3,		// [-500, 500]
	CLEYE_LENSBRIGHTNESS		// [-500, 500]
}CLEyeCameraParameter;

// Camera information
IMPORT(int) CLEyeGetCameraCount();
IMPORT(GUID) CLEyeGetCameraUUID(int camId);

// Library initialization
IMPORT(CLEyeCameraInstance) CLEyeCreateCamera(GUID camUUID, CLEyeCameraColorMode mode, 
												CLEyeCameraResolution res, float frameRate);
IMPORT(bool) CLEyeDestroyCamera(CLEyeCameraInstance cam);

// Camera capture control
IMPORT(bool) CLEyeCameraStart(CLEyeCameraInstance cam);
IMPORT(bool) CLEyeCameraStop(CLEyeCameraInstance cam);

// Camera LED control
IMPORT(bool) CLEyeCameraLED(CLEyeCameraInstance cam, bool on);

// Camera parameters control
IMPORT(bool) CLEyeSetCameraParameter(CLEyeCameraInstance cam, CLEyeCameraParameter param, int value);
IMPORT(int) CLEyeGetCameraParameter(CLEyeCameraInstance cam, CLEyeCameraParameter param);

// Camera video frame image data retrieval
IMPORT(bool) CLEyeCameraGetFrameDimensions(CLEyeCameraInstance cam, int &width, int &height);
IMPORT(bool) CLEyeCameraGetFrame(CLEyeCameraInstance cam, PBYTE pData, int waitTimeout = 2000);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

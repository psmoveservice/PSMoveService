# PSMoveService
A background service that manages multiple PSMove Controllers and PS3 Eye Cameras. Clients connect to the service and stream PSMove Controller state (position, orientation and button presses). Also included are a visual client front-end for controller/tracker configuration as well as a plugin for SteamVR. The [FAQ](https://github.com/cboulay/PSMoveService/wiki/Frequently-Asked-Questions) is a good starting point for any specific questions you may have about the project. 

**NOTE** This is alpha software still heavily in development. If you are downloading this project to play games on SteamVR please be aware that this tool may not work for the game you want to play so buyer beware. That said, if you are feeling brave and want to test this we appriciate the feedback about what works and what doesn't.

# Prebuilt Releases
You can download prebuilt releases (Windows only at the moment) from the [Releases](https://github.com/cboulay/PSMoveService/releases) page. Then follow the initial setup instructions found in the [wiki](https://github.com/cboulay/PSMoveService/wiki#initial-setup). If you run into problems take a look at the [Troubleshooting Page](https://github.com/cboulay/PSMoveService/wiki/Troubleshooting-%28Windows%29) first. If that doesn't help you fix your issue, look at the [issues](https://github.com/cboulay/PSMoveService/issues) page to see if anyone else has the same problem. If not, feel free to post a new issue.

# Building from source
If you want to make modifications to the service or want to debug it, you can build the project from source by following the  [Building-from-source](https://github.com/cboulay/PSMoveService/wiki/Building-from-source) instructions. Currently supported build platforms are Win10 and OS X with Linux support hopefully coming in the near future.

#Getting Help
Please start with the wiki. If you can't find help with your problem then please search through the issues (especially the closed ones) to see if your problem has been addressed already. If you still find no previous mention of your problem then you have one of two options:

A) Join us in the the [PSMoveService Google Group](https://groups.google.com/forum/#!forum/psmoveservice) to ask your question. There are several people there who have experience debugging problems with the PSMoveService.

B) If your problem actually is a new bug and you have files to attach (logs, pictures, etc) then go ahead and create a new issue. That said, it's probably best to start with the Google Group first anyway since we can help add context before posting an issue.

The OpenVR driver is technically unsupported. For issues related to OpenVR, calibration, third party applications, specific games, etc., please post in the Google Group first.

# Near Term Roadmap
 * Ongoing Stabilization of service and SteamVR plugin
 * DualShock4 Controller Support
 * PSNavi Controller Support
 * PS4 Camera Support
 * Support for other webcams (need camera calibration tool)
 * Position smoothing and prediction (via Unscented Kalman Filter)
 * C99 client interface to make interfacing with other languages (Python, C#) easier

# Long Term Roadmap
 * Integration with PSMove-UE4
 * Integration with PSMove-Unity5
 * Linux support
 * Integration with OSVR

#Attribution and Thanks
Special thanks to the following people who helped make this project possible:
* Thomas Perl and his [psmoveapi](https://github.com/thp/psmoveapi) project which laid the groundwork for this project.
* Alexander Nitch and his work on [psmove-pair-win](https://github.com/nitsch/psmove-pair-win) that enabled psmove controller pairing over Bluetooth on Windows. Alex also did nearly all of the investigation into the PSMove communication protocol.
* Eugene Zatepyakin and his [PS3EYEDriver](https://github.com/inspirit/PS3EYEDriver) project which enabled access to the PS3Eye camera.
* Ritesh Oedayrajsingh Varma and his work on [PS3EYEDriver](https://github.com/rovarma/PS3EYEDriver) enabling improved camera polling performance (consistent 60fps)
* Frédéric Lopez and his work on [PS-Move-Setup](https://github.com/Fredz66/PS-Move-Setup) that enabled co registration of  and HMD with the PSMove.
* Greg New - Improvements to the SteamVR plugin and config tool
* YossiMH - Improvements to touch pad mappings and help with the HMD/Controller alignment tool
* William (zelmon64) - Beta testing and troubleshooting hero
* Antonio Jose Ramos Marquez - Work on PS4EyeDriver and PSX hardware reverse engineering

# PSMoveService
A background service that manages multiple PSMove Controllers and PS3 Eye Cameras. Clients connect to the service and stream PSMove Controller state (position, orientation and button presses). Also included are a visual client front-end for controller/tracker configuration as well as a plugin for SteamVR. The [FAQ](https://github.com/cboulay/PSMoveService/wiki/Frequently-Asked-Questions) is a good starting point for any specific questions you may have about the project. 

**NOTE** This is alpha software still heavily in development. If you are downloading this project to play games on SteamVR please be aware that this tool may not work for the game you want to play so buyer beware. That said, if you are feeling brave and want to test this we appriciate the feedback about what works and what doesn't.

# Prebuilt Releases
You can download prebuilt releases (Windows only at the moment) from the [Releases](https://github.com/cboulay/PSMoveService/releases) page. Then follow the initial setup instructions found in the [wiki](https://github.com/cboulay/PSMoveService/wiki#initial-setup). If you run into problems take a look at the [Troubleshooting Page](https://github.com/cboulay/PSMoveService/wiki/Troubleshooting-%28Windows%29) first. If that doesn't help you fix your issue, look at the [issues](https://github.com/cboulay/PSMoveService/issues) page to see if anyone else has the same problem. If not, feel free to post a new issue.

# Building from source
If you want to make modifications to the service or want to debug it, you can build the project from source by following the  [Building-from-source](https://github.com/cboulay/PSMoveService/wiki/Building-from-source) instructions. Currently supported build platforms are Win10 and OS X with Linux support hopefully coming in the near future.

# Getting Help
Please start with the [wiki](https://github.com/cboulay/PSMoveService/wiki). If you can't find help with your problem then please search through the issues (especially the closed ones) to see if your problem has been addressed already. If you still find no previous mention of your issue then post a new issue.

If you are having trouble with a game while trying to use PSMoveService, please be clear about how the problem you are having with the game relates to PSMoveService. For example, saying "I can't throw the ball in X because there appears to be no physics" is a good bug. Saying "I can't get through this game" doesn't help us debug it. Chances are that we don't have the game you are trying to play and have no context for how it's suppose to work. Our time is also stretched thin just trying to get this service working in the first place so please help us by trying to debug the issue as much as you can yourself and posting your findings. Also understand that making a particular game work for you is a low priority for us until we finish getting some major features in place.

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

#Attribution
Special thanks to the following people who helped make this project possible:
* Thomas Perl and his [psmoveapi](https://github.com/thp/psmoveapi) project which laid the groundwork for this project.
* Alexander Nitch and his work on [psmove-pair-win](https://github.com/nitsch/psmove-pair-win) that enabled psmove controller pairing over Bluetooth on Windows. Alex also did nearly all of the investigation into the PSMove communication protocol.
* Eugene Zatepyakin and his [PS3EYEDriver](https://github.com/inspirit/PS3EYEDriver) project which enabled access to the PS3Eye camera.
* Ritesh Oedayrajsingh Varma and his work on [PS3EYEDriver](https://github.com/rovarma/PS3EYEDriver) enabling improved camera polling performance (consistent 60fps)
* Frédéric Lopez and his work on [PS-Move-Setup](https://github.com/Fredz66/PS-Move-Setup) that enabled co registration of  and HMD with the PSMove.

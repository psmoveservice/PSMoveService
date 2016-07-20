# PSMoveService
A background service that manages multiple PSMove Controllers and PS3 Eye Cameras. Clients connect to the service and stream PSMove Controller state (position, orientation and button presses). Also included are a visual client front-end for controller/tracker configuration as well as a plugin for SteamVR. The [FAQ](https://github.com/cboulay/PSMoveService/wiki/Frequently-Asked-Questions) is a good starting point for any specific questions you may have about the project. 

**NOTE** This is alpha software still heavily in development. If you are downloading this project to play games on SteamVR please be aware that this tool may not work for the game you want to play so buyer beware. That said, if you are feeling brave and want to test this we appriciate the feedback about what works and what doesn't.

# Prebuilt Releases
You can download prebuilt releases (Windows only at the moment) from the [Releases](https://github.com/cboulay/PSMoveService/releases) page. Then follow the initial setup instructions found in the [wiki](https://github.com/cboulay/PSMoveService/wiki#initial-setup). If you run into problems take a look at the [Troubleshooting Page](https://github.com/cboulay/PSMoveService/wiki/Troubleshooting-%28Windows%29) first. If that doesn't help you fix your issue, look at the [issues](https://github.com/cboulay/PSMoveService/issues) page to see if anyone else has the same problem. If not, feel free to post a new issue.

# Building from source
If you want to make modifications to the service or want to debug it, you can build the project from source by following the  [Building-from-source](https://github.com/cboulay/PSMoveService/wiki/Building-from-source) instructions. Currently supported build platforms are Win10 and OS X with Linux support hopefully coming in the near future.

# Setup

YouTuber [VirtuallyChris](https://www.youtube.com/channel/UCeHc0xNYWSSoS1aBZ_xmDbw) has put together this really excellent [setup video](https://www.youtube.com/watch?v=XiLYa1EZL04). All of the steps in the video are also in the [setup wiki](https://github.com/cboulay/PSMoveService/wiki#initial-setup), but it's probably best to watch the video first to get a sense of what's involved.

**PLEASE DO NOT REPOST THIS VIDEO**. I know some of you love to re-post this stuff on /r/oculus or other sites and I appriciate your enthusiasm. But this video is intended to help people who have discovered this project get started, **NOT** to advertise this project. PSMoveService still has several major features in development and is **NOT** ready for broad adoption. By widely broadcasting this service in its current state we get overwealmed with support requests which make it more difficult for us complete the work we have left. We've held back on posting this setup video for a while for this reason, but enough people were having difficulty with the setup that I felt it was doing more harm than good to not make it available to people trying to use this service now. Thank you for your patience and support.

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

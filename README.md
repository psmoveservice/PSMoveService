# PSMoveService
A background service that communicates with the psmove and stores pose and button data.

# Download

`git clone --recursive https://github.com/cboulay/PSMoveService.git`
`cd PSMoveService`

# Build Dependencies

1. OpenCV
    * Windows
        * Follow steps 1-4 found [here](https://github.com/MicrocontrollersAndMore/OpenCV_3_Windows_10_Installation_Tutorial/blob/master/Installation%20Cheat%20Sheet%201%20-%20OpenCV%203%20and%20C%2B%2B.pdf)
        * TODO: cmake option to specify OpenCV search directory
    * Mac
        * Install [homebrew](http://brew.sh/)
        * `brew tap homebrew/science`
        * `brew install opencv`
1. Optional: libusb
    * Only necessary for PS3EYEDriver (required on Mac and Windows 64-bit)
    * Windows:
        * Open psmoveapi\external\libusb-1.0\msvc\libusb_2013.sln
        * Change the target to Release x64 (at the top of the Visual Studio window).
        * Right-click on libusb-1.0 (static) and select Properties.
        * In the properties Window, make sure the Platform is set to Active or All Platforms.
        * In the properties Window, navigate to Configuration Properties > C/C++ > Code Generation
        * Change "Runtime Library" to Multi-threaded DLL (/MD)
        * Click OK
        * Right-click on libusb-1.0 (static) and Build.
    * Mac:
        * `cd thirdparty/libusb`
        * `./autogen.sh`
        * `./configure`
        * `./configure` (yes, a second time)
        * `make`

# Build PSMoveService

1. `mkdir build`
1. `cd build`
1. Run cmake
    * Windows: `cmake .. -G "Visual Studio 12 Win64"`
    * Mac: `cmake ..`
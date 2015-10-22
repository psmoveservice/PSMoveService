# PSMoveService
A background service that communicates with the psmove and stores pose and button data.

# Download

`git clone --recursive https://github.com/cboulay/PSMoveService.git`

`cd PSMoveService`

# Build Dependencies

1. Compiler
    * Windows
        * Visual Studio 2013 is required by OpenCV 3.0.0 pre-compiled binaries.
    * Mac
        * Tested with XCode/clang. gcc may work.
1. OpenCV
    * I am opting for a system install of opencv instead of project-specific.
    * Windows
        * Follow steps 1-3 found [here](https://github.com/MicrocontrollersAndMore/OpenCV_3_Windows_10_Installation_Tutorial/blob/master/Installation%20Cheat%20Sheet%201%20-%20OpenCV%203%20and%20C%2B%2B.pdf)
    * Mac
        * Install [homebrew](http://brew.sh/)
        * `brew tap homebrew/science`
        * `brew install opencv`
1. Optional: libusb
    * Only necessary for PS3EYEDriver (required on Mac and Windows 64-bit)
    * Windows:
        * Open PSMoveService\thirdparty\libusb\msvc\libusb_2013.sln
        * For each combination of Release/Debug * Win32/x64, right-click on libusb-1.0 (static) and Build.
        * Close this Visual Studio Solution.
    * Mac:
        * `cd thirdparty/libusb`
        * `./autogen.sh`
        * `./configure`
        * `./configure` (yes, a second time)
        * `make`
1. Optional: [CL Eye Driver](https://codelaboratories.com/products/eye/driver/)
    * Only necessary for Windows 32-bit if not using PS3EYEDriver
    * Currently $2.99 USD (paypal or credit card)
    * Platform SDK not necessary

# Make PSMoveService

1. `mkdir build`
1. `cd build`
1. Run cmake
    * Windows: `cmake .. -G "Visual Studio 12" -DOpenCV_DIR=C:\OpenCV-3.0.0\build`
    * Mac: `cmake .. - G Xcode`

# Build PSMoveService

### Windows

1. Open <path_to_repo>\build\PSMoveService.sln
1. Change to "Release" configuration
1. Remove _DEBUG preprocessor definition
    * TODO: Why is this happening?
	* Rt-click on the project name, Open 'Properties'
	* 'Configuration Properties' > 'C/C++' > Preprocessor
	* Edit "Preprocessor Definitions" and delete _DEBUG
1. Rt-click on the project and build
# PSMoveService
A background service that communicates with the psmove and serves its pose and button states. It also include a visual client front-end for configuring and managing controllers. The [FAQ](https://github.com/cboulay/PSMoveService/wiki/Frequently-Asked-Questions) is a good starting point for any questions you may have about the project.

# Download

`git clone --recursive https://github.com/cboulay/PSMoveService.git`

`cd PSMoveService`

# Build Dependencies (Windows)
1. Compiler
    * Visual Studio 2013 is required by OpenCV 3.0.0 pre-compiled binaries.
1. OpenCV
    * I am opting for a system install of opencv instead of project-specific.
    * Download [OpenCV 3.0](http://sourceforge.net/projects/opencvlibrary/files/opencv-win/3.0.0/opencv-3.0.0.exe/download)
    * The CMake scripts assume you install to the default directory (C:\OpenCV-3.0.0).
    If not, you will have to add a `-DOpenCV_DIR=<install dir>\build` flag to your cmake command.
1. Boost
    * From [here](https://sourceforge.net/projects/boost/files/boost-binaries/1.59.0_b1/), get boost_1_59_0-msvc-12.0-32.exe and/or -64.exe.
    * Install to `C:\` (e.g., resulting in `C:\boost_1_59_0\`)
    * This path will be referred to BOOST_ROOT later
1. Optional: [CL Eye Driver](https://codelaboratories.com/products/eye/driver/)
    * Only necessary for Windows 32-bit if not using PS3EYEDriver
    * Currently $2.99 USD (paypal or credit card)
    * Platform SDK not necessary
1. InitialSetup.bat Batch Script
    * The `InitialSetup.bat` in the root folder will automatically configure and build the dependencies in the thirdparty folder.
    * It will ask you for the root install folders of Boost and OpenCV.
    * After the initial setup phase, if you add source files or other CMake changes you can run `GenerateProjectFiles.bat` to regenerate the PSMoveService solution.
    * If you would rather do initial setup by hand, then proceed to the next step
1. protobuf (Manual Steps)
    * cd to thirdparty\protobuf
    * mkdir vsprojects & cd vsprojects
    * cmake -G "Visual Studio 12 2013" -Dprotobuf_DEBUG_POSTFIX="" -Dprotobuf_BUILD_TESTS=OFF ../cmake
    * Open protobuf.sln
    * Select Release|Win32 and Build > Rebuild Solution
    * Select Debug|Win32 and Build > Rebuild Solution
1. Optional: libusb (Manual Steps)
    * Only necessary for PS3EYEDriver (required on Mac and Windows 64-bit)
    * Open PSMoveService\thirdparty\libusb\msvc\libusb_2013.sln
    * For each combination of Release/Debug * Win32/x64, right-click on libusb-1.0 (static) and Build.
    * Close this Visual Studio Solution.
1. Optional: SDL2 (Manual Steps)
    * Optional - Only required if you are building the the configuration client.
    * `cd third_party/SDL2`
    * `mkdir build & cd build`
    * `cmake .. -G "Visual Studio 12 2013" -DDIRECTX=OFF -DDIRECTX=OFF -DSDL_STATIC=ON -DFORCE_STATIC_VCRT=ON -DEXTRA_CFLAGS="-MT -Z7 -DSDL_MAIN_HANDLED -DWIN32 -DNDEBUG -D_CRT_SECURE_NO_WARNINGS -DHAVE_LIBC -D_USE_MATH_DEFINES`
    * Open the solution (psmoveapi\external\SDL2\build\SDL2.sln)
    * Change the target to Release (at the top of the VS window).
    * Build the SLD2-static and SDL2main projects (Not the SDL2 project) 
    
# Build Dependencies (OS X)

1. Compiler
    * Tested with XCode/clang. gcc may work.
1. Homebrew
    * I am opting for a system install of dependent libraries where possible, instead of project-specific.
    * Install [homebrew](http://brew.sh/) if you do not already have it.
1. OpenCV
    * `brew update`
    * `brew tap homebrew/science`
    * `brew install opencv3`
1. Boost
    * `brew install boost`
1. protobuf
    * `brew install protobuf --devel`
    * It will likely be necessary to drop the --devel flag after >= 3.0 becomes stable.
1. Optional: libusb
    * Only necessary for PS3EYEDriver (required on Mac and Windows 64-bit)
    * `cd thirdparty/libusb`
    * `./autogen.sh`
    * `./configure`
    * `./configure` (yes, a second time)
    * `make`
1. Optional: SDL2
    * Optional - Only required if you are building the the configuration client.
    * `brew install SDL2`
   
# Generate PSMoveService project files

1. `mkdir build && cd build`
1. Run cmake
    * Windows: `cmake .. -G "Visual Studio 12 2013"` OR `GenerateProjectFiles.bat`
    * Mac: `cmake ..` OR `cmake -G "Xcode" ..`

# Build PSMoveService

### Windows

1. Open <path_to_repo>\build\PSMoveService.sln
1. Change to "Release" configuration
1. Rt-click on the project and build

### OS X (Xcode)

1. Launch Xcode
1. Open <path_to_repo>\build\PSMoveService.xcodeproj
1. Go to "Product" > "Build" or hit <Command>-B

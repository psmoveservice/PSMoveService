# PSMoveService
A background service that communicates with the psmove and stores pose and button data.

# Download

`git clone --recursive https://github.com/cboulay/PSMoveService.git`
`cd PSMoveService`

# Build Dependencies

1. OpenCV
    1. `cd thirdparty`
    1. `cd opencv`
    1. `mkdir build`
    1. `cd build`
    1. Run cmake
        * Windows 32: `cmake .. -G "Visual Studio 12 Win64" -DBUILD_SHARED_LIBS=OFF -DBUILD_WITH_STATIC_CRT=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DBUILD_DOCS=OFF -DBUILD_opencv_apps=OFF -DBUILD_opencv_calib3d=ON -DBUILD_opencv_flann=ON -DBUILD_opencv_features2d=ON -DBUILD_opencv_objdetect=OFF -DBUILD_opencv_photo=OFF -DBUILD_opencv_ts=OFF -DBUILD_opencv_ml=OFF -DBUILD_opencv_video=OFF -DBUILD_opencv_java=OFF -DWITH_OPENEXR=OFF -DWITH_FFMPEG=OFF -DWITH_JASPER=OFF -DWITH_TIFF=OFF`
        * Windows 64: `cmake .. -G "Visual Studio 12" -DBUILD_SHARED_LIBS=OFF -DBUILD_WITH_STATIC_CRT=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DBUILD_DOCS=OFF -DBUILD_opencv_apps=OFF -DBUILD_opencv_calib3d=ON -DBUILD_opencv_flann=ON -DBUILD_opencv_features2d=ON -DBUILD_opencv_objdetect=OFF -DBUILD_opencv_photo=OFF -DBUILD_opencv_ts=OFF -DBUILD_opencv_ml=OFF -DBUILD_opencv_video=OFF -DBUILD_opencv_java=OFF -DWITH_OPENEXR=OFF -DWITH_FFMPEG=OFF -DWITH_JASPER=OFF -DWITH_TIFF=OFF`
        * Mac: `cmake .. -DBUILD_SHARED_LIBS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DBUILD_DOCS=OFF -DBUILD_FAT_JAVA_LIB=OFF -DBUILD_PACKAGE=OFF -DBUILD_opencv_apps=OFF -DBUILD_opencv_calib3d=ON -DBUILD_opencv_flann=ON -DBUILD_opencv_features2d=OFF -DBUILD_opencv_objdetect=OFF -DBUILD_opencv_photo=OFF -DBUILD_opencv_shape=OFF -DBUILD_opencv_stitching=OFF -DBUILD_opencv_superres=OFF -DBUILD_opencv_ts=OFF -DBUILD_opencv_ml=ON -DBUILD_opencv_video=OFF -DBUILD_opencv_videostab=OFF -DBUILD_opencv_world=OFF -DBUILD_opencv_java=OFF -DWITH_OPENEXR=OFF -DWITH_FFMPEG=OFF -DWITH_JASPER=OFF -DWITH_TIFF=OFF`
    1. Build
        * Windows:
            * Open the solution in psmoveapi\external\opencv\build\OpenCV.sln
            * Change the target to Release x64 (at the top of the Visual Studio window).
            * Build the solution (Press F7).
        * Mac: `make`
    1. `cd ..`
    1. `cd ..`
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
Temporary build notes:

* Download vcpkg
* >vcpkg install sdl2:x64-windows libusb:x64-windows opencv:x64-windows boost:x64-windows eigen3:x64-windows protobuf:x64-windows glm:x64-windows imgui:x64-windows
* Build with VS2017 integrated cmake
    * Open PSMoveService folder in Visual Studio 2017. Let cmake scan and attempt to configure.
    See [here](https://docs.microsoft.com/en-us/cpp/ide/cmake-tools-for-visual-cpp?view=vs-2017#ide-integration).
    * Using the dropdown menu, make sure your configuration is set to x64-Release (or Debug if so desired).
    * Set your CMake configuration settings from the cmake menu
        ```
        ,
            "variables": [{
              "name": "CMAKE_TOOLCHAIN_FILE",
              "value": "D:\\Tools\\Misc\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake"
            }]
        ```
* Alternatively, Config and generate with command line
    * cmake .. -G "Visual Studio 15 2017 Win64" -DCMAKE_TOOLCHAIN_FILE=D:\Tools\Misc\vcpkg\scripts\buildsystems\vcpkg.cmake
        * Optional addons:  -DVCPKG_TARGET_TRIPLET=x64-windows -DCMAKE_BUILD_TYPE=Release


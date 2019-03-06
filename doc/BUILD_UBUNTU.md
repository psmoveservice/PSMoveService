## <a name="Linux">Linux</a>

Tested with Ubuntu 18.04 AMD64. Not quite ready.

### Prerequisites

The following should be run from inside the cloned PSMoveService directory.

1. `mkdir build_deps && cd build_deps`
1. Build tools
    * `sudo apt-get install build-essential git checkinstall pkg-config autoconf automake libtool`
    * Ubuntu ships with an old version of cmake. We need to manually install a more recent version:
        * `wget https://github.com/Kitware/CMake/releases/download/v3.13.4/cmake-3.13.4-Linux-x86_64.sh`
        * `sudo sh cmake-3.13.4-Linux-x86_64.sh --prefix=/usr/local --exclude-subdir`
1. Third party libraries from apt-get
    * `sudo apt-get install libopencv-dev libsdl2-dev libeigen3-dev libbluetooth-dev libhidapi-dev libusb-1.0 libboost-all-dev libprotobuf-dev protobuf-compiler libglm-dev`
1. You will probably have to close your terminal window and open a new one.
1. ImGui - Unlike Windows with vcpkg, there's no package-manager version of ImGui, so we download and build.
    * `git clone https://github.com/ocornut/imgui.git`


### Generate project files

1. `mkdir build && cd build`
1. `cmake ..`
1. `make .`

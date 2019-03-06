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
    * `sudo apt-get install libopencv-dev libsdl2-dev libeigen3-dev libbluetooth-dev libhidapi-dev libusb-1.0 libprotobuf-dev protobuf-compiler libglm-dev`
1. You will probably have to close your terminal window and open a new one.
1. ImGui - Unlike Windows with vcpkg, there's no package-manager version of ImGui, so we download it. I don't think we have to build.
    * `git clone https://github.com/ocornut/imgui.git`
1. We need a newish version of boost, one that has boost.asio.io_context instead of .io_service
    * `wget -O boost.tar.bz2 "https://dl.bintray.com/boostorg/release/1.69.0/source/boost_1_69_0.tar.bz2"`
    * `tar xvfj boost.tar.bz2`
    * `cd boost_1_69_0`
    * `sudo ./bootstrap.sh`
        * TODO: Add `--with-libraries=library-name-list` to shorten compile time.
    * `sudo ./b2 install`

### Generate project files

1. Assuming you are still in the build_deps folder, do a `cd ..`
1. `mkdir build && cd build`
1. `cmake ..`
1. `make .`

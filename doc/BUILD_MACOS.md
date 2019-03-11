Temporary build notes:

* install homebrew
    * `/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"` # Install homebrew
* `brew update`
* `brew install cmake git sdl2 libusb eigen opencv@3 boost protobuf glm`
* cd into root PSMoveService folder
* `mkdir build_deps && cd build_deps`
    * From root PSMoveService folder
* `git clone https://github.com/ocornut/imgui.git`
* `cd ..`
* `mkdir build && cd build`
* `cmake ..`

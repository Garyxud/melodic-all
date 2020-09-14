Roboception Dynamics API
========================

The rc_dynamics_api provides an API for easy handling of the dynamic-state data
streams provided by Roboception's [rc_visard](http://rc-visard.com) stereo
camera with self-localization.

Dynamic-state estimates of the rc_visard relate to its self-localization and
ego-motion estimation. These states refer to rc_visard's current pose,
velocity, or acceleration and are published on demand via several data streams.
For a complete list and descriptions of these dynamics states and the
respective data streams please refer to rc_visard's user manual.

Compiling and Installing
------------------------

This package is based on some open source projects that are listed below. All
dependencies are provided as submodules that are optionally used if the
packages cannot be found on the system:

- **[C++ Requests](https://github.com/whoshuu/cpr) (version 1.3.0):**
Requesting and deleting data streams is done via rc_visard's REST-API. This
library provides an easy-to-use interface for doing REST-API calls.

- **[Google Protocol Buffers:](https://developers.google.com/protocol-buffers/)**
The data sent via rc_visard's data streams is serialized via Google protocol
message definitions (/roboception/msgs). After receiving the data, the
rc_dynamics_api needs these definitions in order to de-serialized it. This
project requires both the `protobuf-compiler` for compiling the protocol buffer
definition files and the `libprotobuf` C++ library.

After cloning the git repository, the packages described above are cloned as
submodules by:

    git submodule update --init --recursive

Additionally this packages uses the single header file of
- **[JSON for Modern C++](https://github.com/nlohmann/json) (version v2.0.0):**
A simple and modern C++ JSON parsing library.

### Linux

Building follows the standard cmake build flow.

    cd <main-directory>
    mkdir build
    cd build
    cmake -DCMAKE_INSTALL_PREFIX=<install-directory> ..
    make
    make install

### Windows and Visual Studio

Building is based on cmake. Therefore, cmake must be downloaded and installed
according to the operating system from https://cmake.org/download/ After
starting the cmake-gui, the path to the main directory as well as the build
directory must be specified. It is common to choose a sub-directory of the
main directory and name it 'build' for the the temporary files that are created
during the build process. After setting both paths, the 'Configure' button must
be pressed. In the up-coming dialog, it can be chosen for which version of
Visual Studio and which platform (e.g. Win64) the project files should be
generated. The dialog is closed by pressing 'Finish'.

After configuration, the value of the key with the name 'CMAKE_INSTALL_PREFIX'
may be changed to an install directory. By default, the install directory is
set to a path like 'C:/Program Files/...'. The 'Generate' button leads to
creating the project file. Visual Studio can be opened with this project by
pressing the 'Open Project' button.

By default, a 'Debug' version will be compiled. This can be changed to 'Release'
for compiling an optimized version. The package can then be created, e.g. by
pressing 'F7'. For installing the compiled package, the 'INSTALL' target can be
*created* in the project explorer.

After installation, the install directory will contain three sub-directories.
The 'bin' directory contains the tools and DLLs. The 'include' sub-directory
contains the sub-directories 'rc_dynamics_api' and 'roboception'. The former
contains the main headers of the library to build own applications. The latter
contains the protobuf headers of the rc_dynamics interface. There may be other
include directories that may be created by the submodules. They can be ignored
or deleted. Finally, the 'lib' sub-directories contains the rc_dynamics_api
link library that is required for building own applications. Other link
libraries may be installed by the submodules. They can be ignored or deleted.

Tools
-----

Currently, the rc_dynamics_api comes with the following tool which is also
meant as an example on how to use this API:

- **rcdynamics_stream**

    Connect with an rc_visard device and request a specific data stream. The
    received data messages containing aspects of rc_visard's dynamic state can
    be simply printed to std::out or saved as .csv-file.

    Simplest example:

    Connect to rc_visard with specified IP address and request the 'imu' stream.
    A certain amount of messages are received and simply print to std::out

        ./tools/rcdynamics_stream -v 10.0.2.99 -s imu

    Different use case with more command line parameters:

    Connect to rc_visard with specified IP address and request the 'pose_rt'
    stream for 10 seconds. The client host's network interface to be used to
    receive the data is specified as 'eth0'. Messages are stored in the
    specified file as .csv-lines.

        ./tools/rcdynamics_stream -v 10.0.2.99 -s pose_rt -i eth0 -a -t10 -o poses.csv

Links
-----

- http://www.roboception.com

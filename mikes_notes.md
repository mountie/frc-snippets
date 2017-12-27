The whole papps thing is Aidan's fiddle space, cloned from his github at https://github.com/mountie/frc-snippets


# Things I needed to install (debian 8):
(note: this list may differ a bit based on what distro of linux you're using, and what you already have installed. If you're missing something, you'll know when you go to do the cmake step.)

apt-get install cmake libeigen3-dev libflann-dev freeglut3-dev libboost-all-dev

# Things I installed while fumbling around (may or may not have been needed):
apt-get install


# Instructions I followed:
http://pointclouds.org/documentation/tutorials/compiling_pcl_posix.php

To standardize our CMake files across machines, let's all install PCL to /opt/pointclouds:

    cmake -DCMAKE_INSTALL_PREFIX=/opt/pointclouds ..
    make && sudo make install

*Note*: build re-started from 7% at 19:50.


As a starting point, clone the sample PCL apps that Aidan has on his personal github:

    git clone https://github.com/mountie/frc-snippets.git

then build them:






# Problems I ran into and solved:

## PCLConfig.cmake

**SOLUTION**: the pre-build binaries are only for MacOS ("darwin"). You need to build from source!

I copied the pre-built PLC 1.8.1 binaries from their github page and stuck them into /opt/pointclouds/share/pcl-1.8 where CMakeLists.txt was pointing, but I still get this error:

    CMake Error at CMakeLists.txt:7 (find_package):
    By not providing "FindPCL.cmake" in CMAKE_MODULE_PATH this project has
    asked CMake to find a package configuration file provided by "PCL", but
    CMake did not find one.

    Could not find a package configuration file provided by "PCL" (requested
    version 1.2) with any of the following names:

    PCLConfig.cmake
    pcl-config.cmake

    Add the installation prefix of "PCL" to CMAKE_PREFIX_PATH or set "PCL_DIR"
    to a directory containing one of the above files.  If "PCL" provides a
    separate development package or SDK, be sure it has been installed.


My linux / CMake foo isn't strong enough for this, so I'll just install the libpcl1.7 that's available through the debian apt-get (not sure yet if this is a -dev package, or only a binary one)

FIXED!! Those files are in pcl-1.8.1-darwin.tar.bz2/share/pcl-1.8, so the solution was to `sudo ln -s ~/pcl-1.8.1-darwin /opt/pointclouds`.

## Boost components missing

**SOLUTION**: `apt-get install libboost-all-dev`

Error:

    -- Could NOT find eigen (missing:  EIGEN_INCLUDE_DIRS)

This is part of Boost, so the theory is that I don't have libboost-dev installed. ... did not make a difference. Adding boost debugging to CMakeLists.txt

    project(pcd_write)
    set(Boost_DEBUG ON)
    set(Boost_USE_STATIC_LIBS ON)
    FIND_PACKAGE(Boost)
    find_package(PCL 1.2 REQUIRED)

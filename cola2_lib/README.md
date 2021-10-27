# COLA2 LIB

Reusable code that is used all over the COLA2 software architecture.

## Documentation

You can check the documentation API for this library [here](http://api.iquarobotics.com/202010/cola2_lib/). For instructions on how to generate the documentation, please view the documentation [readme](doc/README.md).

## Dependencies
This project depends on the libraries Boost (modules thread, date time and file system), Eigen 3 and TinyXML. Moreover, the python modules of this project are installed both for pythong 2 and 3, hence these two distributions of python need to be installed. For this project to work properly please intall the following dependencies:

```bash
sudo apt install libboost-thread-dev libboost-date-time-dev libboost-filesystem-dev libeigen3-dev libtinyxml-dev python python3
```

## Compiling/installing

To compile this project:

```bash
mkdir build
cd build
cmake ..
make
sudo make install
```

Overwriting the `CMAKE_INSTALL_PREFIX` variable to avoid installing with `sudo` priveleges is not recomennded and will not work. During the `make install` process the python pacakges will be installed onto system folders. Therefore, running `make install` without `sudo` priviledges will fail. If you want to use COLA2 LIB without installing it, please visit the section *Using COLA2 LIB without installation on a system folder* at the end of this document.

## C++ compiling and linking against this library

This library can be reused from other c++ libraries. This library provides cmake files to ease this process. For this purpose, add the following to your cmake file:

```bash
find_package(COLA2_LIB REQUIRED)
include_directories(${COLA2_LIB_INCLUDE_DIRS})

# Add your targets
target_link_library(my_target ${COLA2_LIB_LIBRARIES})
```

## Python usage

This library also provides python functionalities. Here, the following packages are provided: cola2_lib, cola2_lib.utils, cola2_lib.utils.angles, cola2_lib.utils.ned, cola2_lib.utils.saturate, cola2_lib.io.nmea_parser. 

## Using COLA2 LIB without installation on a system folder

### C++ compiling and linking
If the library is not installed (i.e. `sudo make install` has not been exectued) the `cmake` should still be able to find COLA2 LIB. If your package cannot find this library, you can help it by defing the library path in your own project cmake running: `cmake  -DCOLA2_LIB_DIR=path_to_the_build_folder ..`.

### Python

If COLA2 LIB is not installed using `sudo make install`, make sure your python path contains the python folder of this repository. This can be done on a terminal with:

```bash
export PYTHONPATH=$PYTHONPATH:path_to_cola2_lib_folder/python
```

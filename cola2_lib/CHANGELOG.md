# Changelog

## [20.10.2] - 16-12-2020

* Added support for boost 1.7 (Ubuntu 20) while keeping support for boost 1.65 (Ubuntu 18)

## [20.10.1] - 25-10-2020

* Added documentation on how to use this package without installing it.
* Corrected an error configuring the package to be used as an external library from the build folder.

## [20.10.0] - 26-10-2020

* Added mission API
* Added World Magnetic Model
* Added support for Python3
* Added encode/decode of altitude and watchdog data for comms

## [3.2.2] - 10-01-2020

* Changed cmake to allow compilation without the need of doing make install. In this case, the package is found by passing `-DCOLA2_LIB_DIR=path_to_cola2_lib/build_directory` to `cmake` or `catkin_make` of the dependent package. If the dependent package is compiled using `catkin build`, it should be configured using `catkin config --cmake-args -DCOLA2_LIB_DIR=path_to_cola2_lib/build_directory` prior to running `catkin build`. In any case, if the package is not installed, the python path variable has to be set so the python libraries of this packages are found: `export PYTHONPATH=$PYTHONPATH:path_to_cola2_lib/python`.

## [3.2.1] - 08-01-2020

* Added methods to solve [3.2.1] in ixblue package

## [3.2.0] - 22-10-2019

* Renamed Eigen::Vector3d ned2geodetic(const Eigen::Vector3d &ned) const; to Eigen::Vector3d ned2Geodetic(const Eigen::Vector3d &ned) const; to be propperly formatted camelcase-wise
* Add pressure conversion utils
* Add `createDirectory` and `deleteFile` functions to filesystem utils
* Renamed python libraries to cola2.io and cola2.utils
* Refactor cola 2 lib and splitted into two libraries, one containg ros and another one without ros. This is the one without ros.

## [3.1.0] - 25-02-2019

* First release

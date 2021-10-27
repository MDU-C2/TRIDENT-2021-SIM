cmake_minimum_required(VERSION 2.8.3)
project(cola2_lib)

# Version
set(COLA2_LIB_MAJOR_VERSION 20)
set(COLA2_LIB_MINOR_VERSION 10)
set(COLA2_LIB_PATCH_VERSION 0)
set(COLA2_LIB_VERSION ${COLA2_LIB_MAJOR_VERSION}.${COLA2_LIB_MINOR_VERSION}.${COLA2_LIB_PATCH_VERSION})
message(STATUS "${PROJECT_NAME} version ${COLA2_LIB_VERSION}")

# Offer the user the choice of overriding the installation directories
set(INSTALL_BIN_DIR "${CMAKE_INSTALL_PREFIX}/bin")
set(INSTALL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib")
set(INSTALL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}")
set(INSTALL_CMAKE_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")

# Set CMAKE_INSTALL_RPATH
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

# System dependencies
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(cmake/COLA2_LIBDependencies.cmake)

# Specify locations of header files. Your package locations should be listed
# before other locations
include_directories(
  include
)

# Lists of headers, libraries and executables for exporting
set(LIBRARIES "")

# Cola 2 io
set(COLA2_IO_CPP
    src/${PROJECT_NAME}/io/io_base.cpp
    src/${PROJECT_NAME}/io/serial_port.cpp
    src/${PROJECT_NAME}/io/tcp_socket.cpp
    src/${PROJECT_NAME}/io/udp_socket.cpp
    src/${PROJECT_NAME}/io/nmea_parser.cpp)
set(COLA2_IO_H
    include/${PROJECT_NAME}/io/io_base.h
    include/${PROJECT_NAME}/io/serial_port.h
    include/${PROJECT_NAME}/io/tcp_socket.h
    include/${PROJECT_NAME}/io/udp_socket.h
    include/${PROJECT_NAME}/io/nmea_parser.h)
add_library(cola2_io SHARED ${COLA2_IO_CPP} ${COLA2_IO_H})
target_compile_options(cola2_io PRIVATE -std=c++11 -Wall -Wextra)
target_link_libraries(cola2_io ${Boost_LIBRARIES})
set_target_properties(cola2_io PROPERTIES PUBLIC_HEADER "${COLA2_IO_H}")
install(TARGETS cola2_io
  EXPORT COLA2_LIBTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/io"
  COMPONENT dev)
set(LIBRARIES ${LIBRARIES} cola2_io)

# Cola 2 utils
set(COLA2_UTILS_CPP
    src/${PROJECT_NAME}/utils/angles.cpp
    src/${PROJECT_NAME}/utils/filesystem.cpp
    src/${PROJECT_NAME}/utils/ned.cpp
    src/${PROJECT_NAME}/utils/pressure.cpp
    src/${PROJECT_NAME}/utils/saturate.cpp
    src/${PROJECT_NAME}/utils/thread_safe_flag.cpp
 )
set(COLA2_UTILS_H
    include/${PROJECT_NAME}/utils/angles.h
    include/${PROJECT_NAME}/utils/error_macro.h
    include/${PROJECT_NAME}/utils/filesystem.h
    include/${PROJECT_NAME}/utils/ned.h
    include/${PROJECT_NAME}/utils/pressure.h
    include/${PROJECT_NAME}/utils/saturate.h
    include/${PROJECT_NAME}/utils/thread_safe_flag.h
    include/${PROJECT_NAME}/utils/thread_pool.h
 )
add_library(cola2_utils SHARED ${COLA2_UTILS_CPP} ${COLA2_UTILS_H})
target_compile_options(cola2_utils PRIVATE -std=c++11 -Wall -Wextra)
target_link_libraries(cola2_utils ${Boost_LIBRARIES})
set_target_properties(cola2_utils PROPERTIES PUBLIC_HEADER "${COLA2_UTILS_H}")
install(TARGETS cola2_utils
  EXPORT COLA2_LIBTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/utils"
  COMPONENT dev)
set(LIBRARIES ${LIBRARIES} cola2_utils)

# Cola 2 mission
set(COLA2_MISSION_CPP
    src/${PROJECT_NAME}/mission/mission.cpp
    src/${PROJECT_NAME}/mission/mission_step.cpp
    src/${PROJECT_NAME}/mission/mission_action.cpp
    src/${PROJECT_NAME}/mission/mission_maneuver.cpp
    src/${PROJECT_NAME}/mission/mission_goto.cpp
    src/${PROJECT_NAME}/mission/mission_section.cpp
    src/${PROJECT_NAME}/mission/mission_park.cpp
 )
set(COLA2_MISSION_H
    include/${PROJECT_NAME}/mission/mission.h
    include/${PROJECT_NAME}/mission/mission_step.h
    include/${PROJECT_NAME}/mission/mission_action.h
    include/${PROJECT_NAME}/mission/mission_maneuver.h
    include/${PROJECT_NAME}/mission/mission_goto.h
    include/${PROJECT_NAME}/mission/mission_section.h
    include/${PROJECT_NAME}/mission/mission_park.h
 )
add_library(cola2_mission SHARED ${COLA2_MISSION_CPP} ${COLA2_MISSION_H})
target_link_libraries(cola2_mission ${TinyXML_LIBRARIES})
set_target_properties(cola2_mission PROPERTIES PUBLIC_HEADER "${COLA2_MISSION_H}")
install(TARGETS cola2_mission
  EXPORT COLA2_LIBTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/mission"
  COMPONENT dev)
set(LIBRARIES ${LIBRARIES} cola2_mission)

# Cola 2 comms
set(COMMS_C
    src/${PROJECT_NAME}/comms/altitude.c
    src/${PROJECT_NAME}/comms/elapsed_time.c)
set(COMMS_H
    include/${PROJECT_NAME}/comms/altitude.h
    include/${PROJECT_NAME}/comms/elapsed_time.h)
add_library(cola2_comms SHARED ${COMMS_C} ${COMMS_H})
target_compile_options(cola2_comms PRIVATE -std=c99 -Wall -Wextra)
target_include_directories(cola2_comms PRIVATE include/${PROJECT_NAME}/comms)
set_target_properties(cola2_comms PROPERTIES PUBLIC_HEADER "${COMMS_H}")
install(TARGETS cola2_comms
  EXPORT COLA2_LIBTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/comms"
  COMPONENT dev)
set(LIBRARIES ${LIBRARIES} cola2_comms)

# Cola 2 wmmc
set(WMMC_C
    src/${PROJECT_NAME}/wmmc/GeomagnetismLibrary.c
    src/${PROJECT_NAME}/wmmc/magnetic_declination.c)
set(WMMC_H
    include/${PROJECT_NAME}/wmmc/GeomagnetismHeader.h
    include/${PROJECT_NAME}/wmmc/magnetic_declination.h)
add_library(cola2_wmmc SHARED ${WMMC_C} ${WMMC_H})
target_compile_definitions(cola2_wmmc PRIVATE COF_DATA_PATH=\"${INSTALL_CMAKE_DIR}/data/WMM.COF\")
target_compile_options(cola2_wmmc PRIVATE -std=c99 -Wall -Wextra)
target_include_directories(cola2_wmmc PRIVATE include/${PROJECT_NAME}/wmmc)
set_target_properties(cola2_wmmc PROPERTIES PUBLIC_HEADER "${WMMC_H}")
install(TARGETS cola2_wmmc
  EXPORT COLA2_LIBTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/wmmc"
  COMPONENT dev)
set(LIBRARIES ${LIBRARIES} cola2_wmmc)

# Cola 2 wmm
set(WMM_C
    src/${PROJECT_NAME}/wmm/magnetic_declination.cpp)
set(WMM_H
    include/${PROJECT_NAME}/wmm/magnetic_declination.h)
add_library(cola2_wmm SHARED ${WMM_C} ${WMM_H})
target_link_libraries(cola2_wmm PUBLIC cola2_wmmc)
target_compile_options(cola2_wmm PRIVATE -std=c++11 -Wall -Wextra)
set_target_properties(cola2_wmm PROPERTIES PUBLIC_HEADER "${WMM_H}")
install(TARGETS cola2_wmm
  EXPORT COLA2_LIBTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/wmm"
  COMPONENT dev)
set(LIBRARIES ${LIBRARIES} cola2_wmm)

# Python 2 & 3
set(SETUP_PY_IN "${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in")
set(SETUP_PY    "${CMAKE_CURRENT_BINARY_DIR}/setup.py")
configure_file(${SETUP_PY_IN} ${SETUP_PY} @ONLY)
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/python/cola2/comms/altitude.py.in" "${CMAKE_CURRENT_SOURCE_DIR}/python/cola2/comms/altitude.py" @ONLY)
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/python/cola2/comms/elapsed_time.py.in" "${CMAKE_CURRENT_SOURCE_DIR}/python/cola2/comms/elapsed_time.py" @ONLY)
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/python/cola2/wmm/wmm.py.in" "${CMAKE_CURRENT_SOURCE_DIR}/python/cola2/wmm/wmm.py" @ONLY)
install(CODE "execute_process(COMMAND ${PYTHON} ${SETUP_PY} install --record ${CMAKE_CURRENT_BINARY_DIR}/python_installed_files.txt)")
install(CODE "execute_process(COMMAND ${PYTHON3} ${SETUP_PY} install --record ${CMAKE_CURRENT_BINARY_DIR}/python3_installed_files.txt)")

# Add all targets to the build-tree export set
export(TARGETS cola2_io cola2_utils cola2_mission cola2_comms cola2_wmmc cola2_wmm FILE "${PROJECT_BINARY_DIR}/COLA2_LIBTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
#export(PACKAGE COLA2_LIB)

# Create the COLA2_LIBConfig.cmake and COLA2_LIBConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}" "${INSTALL_INCLUDE_DIR}")
file(RELATIVE_PATH REL_LIB_DIR "${INSTALL_CMAKE_DIR}" "${INSTALL_LIB_DIR}")
file(RELATIVE_PATH REL_BIN_DIR "${INSTALL_CMAKE_DIR}" "${INSTALL_BIN_DIR}")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include" "${Boost_INCLUDE_DIRS}" "${EIGEN3_INCLUDE_DIR}")
set(CONF_LIBRARY_DIRS "${CMAKE_LIB}")
set(CONF_EXECUTABLE_DIRS "${CMAKE_BIN}")
set(CONF_CMAKE_MODULE_PATH "${PROJECT_BINARY_DIR}")
configure_file(cmake/COLA2_LIBConfig.cmake.in "${PROJECT_BINARY_DIR}/COLA2_LIBConfig.cmake" @ONLY)
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${COLA2_LIB_CMAKE_DIR}/${REL_INCLUDE_DIR}" "${Boost_INCLUDE_DIRS}" "${EIGEN3_INCLUDE_DIR}")
set(CONF_LIBRARY_DIRS "\${COLA2_LIB_CMAKE_DIR}/${REL_LIB_DIR}")
set(CONF_EXECUTABLE_DIRS "\${COLA2_LIB_CMAKE_DIR}/${REL_BIN_DIR}")
set(CONF_CMAKE_MODULE_PATH "\${COLA2_LIB_CMAKE_DIR}")
configure_file(cmake/COLA2_LIBConfig.cmake.in "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/COLA2_LIBConfig.cmake" @ONLY)
# ... for both
configure_file(cmake/COLA2_LIBConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/COLA2_LIBConfigVersion.cmake" @ONLY)
configure_file(cmake/COLA2_LIBDependencies.cmake "${PROJECT_BINARY_DIR}/COLA2_LIBDependencies.cmake" COPYONLY)
configure_file(cmake/FindTinyXML.cmake "${PROJECT_BINARY_DIR}/FindTinyXML.cmake" COPYONLY)

# Install the COLA2_LIBConfig.cmake, COLA2_LIBDependencies.cmake and COLA2_LIBConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/COLA2_LIBConfig.cmake"
  "${PROJECT_BINARY_DIR}/COLA2_LIBConfigVersion.cmake"
  "${PROJECT_SOURCE_DIR}/cmake/COLA2_LIBDependencies.cmake"
  "${PROJECT_SOURCE_DIR}/cmake/FindTinyXML.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)
install(FILES
  "${PROJECT_SOURCE_DIR}/data/WMM.COF"
  DESTINATION "${INSTALL_CMAKE_DIR}/data" COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT COLA2_LIBTargets DESTINATION
  "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Create uninstall command
configure_file("${CMAKE_SOURCE_DIR}/cmake/uninstall_target.cmake.in"
              "${CMAKE_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P
                 "${CMAKE_BINARY_DIR}/uninstall_target.cmake")

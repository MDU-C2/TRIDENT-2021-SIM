cmake_minimum_required(VERSION 3.10)
project(stonefish_ros)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-reorder -Wunused-but-set-variable -DBT_EULER_DEFAULT_ZYX -DBT_USE_DOUBLE_PRECISION -DBUILD_PYBULLET=OFF -DBUILD_SHARED_LIBS=ON -DUSE_DOUBLE_PRECISION=ON")

#add_definitions(-DSHADER_DIR_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/external/stonefish/Library/shaders/\")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  cola2_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  roscpp
  roslib
  tf
  tf_conversions
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Bullet REQUIRED)

include_directories(external/glm)
include_directories(${BULLET_INCLUDE_DIRS})
add_subdirectory(external)

set(Stonefish_INCLUDE_DIRS external/stonefish/Library/include ${PROJECT_BINARY_DIR}/external/stonefish)
set(Stonefish_LIBRARIES Stonefish)

add_message_files(
  FILES
  Int32Stamped.msg
  ThrusterState.msg
)

add_service_files(
  FILES
  SonarSettings.srv
  SonarSettings2.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(${Stonefish_CFLAGS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stonefish_ros
  CATKIN_DEPENDS cola2_msgs 
                 geometry_msgs 
                 nav_msgs 
                 sensor_msgs
                 visualization_msgs
                 std_msgs 
                 std_srvs 
                 roscpp
                 roslib
                 tf 
                 tf_conversions
		         message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Stonefish_INCLUDE_DIRS}
)

link_directories(
  ${Stonefish_LIBRARY_DIRS}
)

#Library
add_library(stonefish_ros SHARED
src/ROSInterface.cpp
src/ROSSimulationManager.cpp
src/ROSScenarioParser.cpp
)
#add_dependencies(stonefish_ros ${catkin_EXPORTED_TARGETS})
add_dependencies(stonefish_ros ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(stonefish_ros ${catkin_LIBRARIES} ${Stonefish_LIBRARIES})

#Universal simulators with XML parser
add_executable(parsed_simulator src/parsed_simulator.cpp)
target_link_libraries(parsed_simulator ${catkin_LIBRARIES} ${Stonefish_LIBRARIES} stonefish_ros)

add_executable(parsed_simulator_nogpu src/parsed_simulator_nogpu.cpp)
target_link_libraries(parsed_simulator_nogpu ${catkin_LIBRARIES} ${Stonefish_LIBRARIES} stonefish_ros)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS parsed_simulator parsed_simulator_nogpu stonefish_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY external/stonefish/Library/shaders/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/external/stonefish/Library/shaders
  PATTERN ".svn" EXCLUDE
)

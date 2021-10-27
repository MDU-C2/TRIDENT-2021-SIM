cmake_minimum_required(VERSION 2.8.3)
project(naiad_description)

find_package(catkin REQUIRED COMPONENTS
  # COLA2 Dependencies
  cola2_lib_ros
)

# Declare things to be passed to dependent projects
catkin_package()

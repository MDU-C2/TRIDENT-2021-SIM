cmake_minimum_required(VERSION 2.8.3)
project(cola2_naiad)

find_package(catkin REQUIRED COMPONENTS
  # ROS Dependencies
  rosbridge_server
  tf
  topic_tools
  rviz
  diagnostic_aggregator
  joy

  # COLA2 Dependencies
  naiad_description
)

# Declare things to be passed to dependent projects
catkin_package()


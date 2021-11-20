cmake_minimum_required(VERSION 2.8.3)
project(mini_pupper_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  gazebo_ros_control
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
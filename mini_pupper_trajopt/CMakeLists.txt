cmake_minimum_required(VERSION 2.8.3)
project(mini_pupper_trajopt)

find_package(catkin REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY rsc/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rsc
)

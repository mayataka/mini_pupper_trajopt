cmake_minimum_required(VERSION 2.8.3)
project(mini_pupper_controller)

find_package(Eigen3 REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  controller_interface
  hardware_interface
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp 
  roslib
  controller_interface 
)

add_library(
  ${PROJECT_NAME} 
  src/mini_pupper_trotting_controller.cpp
  src/mini_pupper_pacing_controller.cpp
  src/mini_pupper_bounding_controller.cpp
  src/mini_pupper_jumping_controller.cpp
  src/mini_pupper_running_controller.cpp
)
target_include_directories(
  ${PROJECT_NAME} 
  PRIVATE
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(
  ${PROJECT_NAME} 
  PUBLIC
  ${catkin_LIBRARIES}
)
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_LIBRARIES})
endif()

foreach(dir config)
  install(DIRECTORY ${dir}/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
  )
endforeach(dir)
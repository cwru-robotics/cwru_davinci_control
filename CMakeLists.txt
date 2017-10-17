cmake_minimum_required(VERSION 2.8.3)
project(cwru_davinci_control)


find_package(Boost REQUIRED COMPONENTS system thread)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  message_generation
  sensor_msgs
  std_msgs
  actionlib_msgs
  trajectory_msgs
  controller_manager
  control_msgs
  rostest
  roslint
)

catkin_package(
  INCLUDE_DIRS include
  # DEPENDS
  LIBRARIES
  	psm_controller
  CATKIN_DEPENDS
    roscpp
    actionlib
    std_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

roslint_cpp()

# Libraries

add_library(psm_controller src/psm_controller.cpp)

target_link_libraries(psm_controller ${catkin_LIBRARIES})


# Executables

add_executable(davinci_traj_controller src/davinci_hwi.cpp)

target_link_libraries(davinci_traj_controller
	${catkin_LIBRARIES}
)


# Testing

# roslint_add_test()
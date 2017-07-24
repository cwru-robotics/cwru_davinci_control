cmake_minimum_required(VERSION 2.8.3)
project(cwru_davinci_traj_streamer)


#This is needed as part of the migration to ros jade and later
find_package(Eigen3)

# this is depreciated but still necessary for now.
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIR ${EIGEN_INCLUDE_DIR})
  set(EIGEN_DEP Eigen)
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
  set(EIGEN_DEP Eigen3)
endif()

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(catkin REQUIRED COMPONENTS
  cwru_davinci_interface
  actionlib
  message_generation
  sensor_msgs
  std_msgs
  actionlib_msgs
  trajectory_msgs
  controller_manager
)


add_action_files(FILES traj.action)

generate_messages(DEPENDENCIES sensor_msgs std_msgs actionlib_msgs trajectory_msgs)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS 
    EIGEN_DEP
    actionlib
    std_msgs
  # LIBRARIES
  CATKIN_DEPENDS
    roscpp
    actionlib
    cwru_davinci_interface
    std_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_definitions(${EIGEN_DEFINITIONS})

# Executables
add_executable(davinci_traj_interpolator_as src/traj_interpolator_both_as.cpp)
add_executable(davinci_traj_controller src/davinci_hwi.cpp)

target_link_libraries(davinci_traj_interpolator_as
	${catkin_LIBRARIES}
)
target_link_libraries(davinci_traj_controller
	${catkin_LIBRARIES}
)

add_dependencies(davinci_traj_interpolator_as
  ${cwru_davinci_traj_streamer_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencpp
)
add_dependencies(davinci_traj_controller
  ${cwru_davinci_traj_streamer_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencpp
)

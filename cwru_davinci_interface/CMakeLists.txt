cmake_minimum_required(VERSION 2.8.3)
project(cwru_davinci_interface)

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


find_package(catkin REQUIRED)

include_directories(
  include ${EIGEN_DEP}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

catkin_package(
	DEPENDS EIGEN_DEP
	LIBRARIES davinci_interface
	# CATKIN_DEPENDS
	INCLUDE_DIRS include
)

add_definitions(${EIGEN_DEFINITIONS})

# Libraries
add_library(davinci_interface src/davinci_interface.cpp)
target_link_libraries(davinci_interface ${catkin_LIBRARIES})

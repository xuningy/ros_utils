cmake_minimum_required(VERSION 3.1)

project(ros_utils)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
catkin_package(
  INCLUDE_DIRS include 
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
link_directories(${catkin_LIBRARY_DIRS} ${EIGEN3_INCLUDE_DIR})
add_definitions(${catkin_DEFINITIONS})

install(DIRECTORY include ${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
)

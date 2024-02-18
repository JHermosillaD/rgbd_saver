cmake_minimum_required(VERSION 3.0.2)
project(rgbd_saver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
)

#OPEN CV libraries
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
set (LIBS ${LIBS} ${OpenCV_LIBS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hog_pkg
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(rgbd_saver_node src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(rgbd_saver_node
  ${catkin_LIBRARIES} ${LIBS}
)

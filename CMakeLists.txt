cmake_minimum_required(VERSION 2.8.3)
project(omni_visual_odometry)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  tf
  sensor_msgs
)

find_package(OpenCV 3.0 QUIET)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES omni_visual_odometry 
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf sensor_msgs
  #DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include/omni_visual_odometry
  ${PROJECT_SOURCE_DIR}/basic_rgbd_visual_odometry
  ${PROJECT_SOURCE_DIR}/basic_rgbd_visual_odometry/include
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/basic_rgbd_visual_odometry/lib)

add_library(${PROJECT_NAME} SHARED
basic_rgbd_visual_odometry/src/visual_odometry.cpp
basic_rgbd_visual_odometry/src/visual_odometry_helpers.cpp
)

add_executable(${PROJECT_NAME}_rgbd src/RGBDVisualOdometryNode.cpp)

add_dependencies(${PROJECT_NAME}_rgbd ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}_rgbd 
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	#${PROJECT_SOURCE_DIR}/basic_rgbd_visual_odometry/lib/lib${PROJECT_NAME}.so
	${PROJECT_NAME}
	${EIGEN3_LIBS}
)


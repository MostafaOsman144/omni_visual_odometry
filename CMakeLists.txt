cmake_minimum_required(VERSION 2.8.3)
project(omni_visual_odometry)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

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

#find_package(OpenCV 3.0 QUIET)
#if(NOT OpenCV_FOUND)
#	find_package(OpenCV 2.4.3 QUIET)
#	if(NOT OpenCV_FOUND)
#		message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
#	endif()
#endif()

#find_package(Eigen3 3.1.0 REQUIRED)


## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES omni_visual_odometry
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf sensor_msgs
  #DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/omni_visual_odometry
)

add_executable(${PROJECT_NAME}_rgbd src/RGBDVisualOdometryNode.cpp)

add_dependencies(${PROJECT_NAME}_rgbd ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}_rgbd 
	${catkin_LIBRARIES}
)


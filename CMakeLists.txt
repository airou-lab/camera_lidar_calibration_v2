cmake_minimum_required(VERSION 2.8.3)
project(camera_laser_calibration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  laser_geometry
  message_filters
  roscpp
  rospy
  sensor_msgs
  tf
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)

## Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(
  cfg/dynamic_range.cfg
)

## catkin specific configuration ##
catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES camera_laser_calibration
  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_transport laser_geometry message_filters roscpp rospy sensor_msgs tf
  # DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${PROJECT_SOURCE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME}
  src/config.cc
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${CERES_LIBRARIES}
)

add_executable(corner_detecter src/corner_detecter.cc)
target_link_libraries(corner_detecter ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(calibration src/camera_laser_calibration.cc)
target_link_libraries(calibration ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(collect_laser_image_data src/collect_laser_image_data.cc)
add_dependencies(collect_laser_image_data ${PROJECT_NAME}_gencfg)  # <<< CRITICAL!!
target_link_libraries(collect_laser_image_data ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(reprojection_test src/reprojection_test.cc)
target_link_libraries(reprojection_test ${catkin_LIBRARIES} ${PROJECT_NAME})


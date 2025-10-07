cmake_minimum_required(VERSION 3.0.2)
project(uwb_uvdar_fusion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  message_generation
  mrs_lib
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

## generate added messages
#generate_messages(
#  DEPENDENCIES std_msgs
#)
add_library(OCamCalib
  include/OCamCalib/ocam_functions.cpp
  )

add_message_files(
  FILES
  UwbRange.msg
  UwbRangeStamped.msg
  ImagePointsWithFloatStamped.msg
  Point2DWithFloat.msg
  UwbUvdarResult.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime roslib
  DEPENDS OpenCV
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/uwb_uvdar_fusion_node.cpp)

# Marker publisher
add_executable(uwb_marker src/uwb_marker.cpp)
add_dependencies(uwb_marker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(uwb_marker ${catkin_LIBRARIES} OCamCalib ${EIGEN3_LIBRARIES})



add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_include_directories(OCamCalib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  )

target_link_libraries(OCamCalib
  ${OpenCV_LIBS}
  )

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES}
  OCamCalib
  ${EIGEN3_LIBRARIES}
  )


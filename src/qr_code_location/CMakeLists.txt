cmake_minimum_required(VERSION 2.8.3)
project(qr_code_location)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
cv_bridge
image_transport
)

find_package(OpenCV 3 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES QR_code_location
#  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${OpenCV_INC})


 add_executable(QR_code_generation src/QR_code_generation.cpp)
 target_link_libraries(QR_code_generation
   ${catkin_LIBRARIES}  ${OpenCV_LIBS})

 add_executable(image_capture src/image_capture.cpp)
 target_link_libraries(image_capture  ${catkin_LIBRARIES}  ${OpenCV_LIBS})

 add_executable(image_capture_to_ros src/image_capture_to_ros.cpp)
 target_link_libraries(image_capture_to_ros  ${catkin_LIBRARIES}  ${OpenCV_LIBS})

# add_executable(pose_estimation_by_QRcode src/pose_estimation_by_QRcode.cpp)
# target_link_libraries(pose_estimation_by_QRcode   ${catkin_LIBRARIES}  ${OpenCV_LIBS})


 add_executable(camera_calibration src/camera_calibration.cpp)
 target_link_libraries(camera_calibration   ${catkin_LIBRARIES}  ${OpenCV_LIBS})

 add_executable(gen_qr src/gen_qr.cpp)
 target_link_libraries(gen_qr   ${catkin_LIBRARIES}  ${OpenCV_LIBS})

 add_executable(detect_qr src/detect_qr.cpp)
 target_link_libraries(detect_qr   ${catkin_LIBRARIES}  ${OpenCV_LIBS})

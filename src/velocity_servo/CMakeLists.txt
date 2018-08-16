cmake_minimum_required(VERSION 2.8.3)
project(velocity_servo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
tf
  roscpp
  std_msgs
  qt_build
  kdl_parser
)

find_package(orocos_kdl)
find_package(OpenCV REQUIRED)
include_directories(${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS} 
 "/usr/include/eigen3")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pkgtest01
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
src
)


add_executable(${PROJECT_NAME}_main src/main.cpp)


 target_link_libraries(${PROJECT_NAME}_main
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)


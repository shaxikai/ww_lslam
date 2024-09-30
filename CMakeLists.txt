cmake_minimum_required(VERSION 2.8.3)
project(cld_in)

SET(CMAKE_BUILD_TYPE "Debug")

ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")

find_package(PCL REQUIRED QUIET)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  livox_ros_driver
  message_generation
  pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${PYTHON_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  include)

catkin_package(
  CATKIN_DEPENDS 
  geometry_msgs 
  nav_msgs 
  roscpp rospy 
  std_msgs 
  message_runtime
)

add_executable(${PROJECT_NAME}_avia_in src/avia_in.cpp)
target_link_libraries(${PROJECT_NAME}_avia_in ${catkin_LIBRARIES} ${PCL_LIBRARY_DIRS})
cmake_minimum_required(VERSION 2.8.3)
project(multicam_vo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
  cv_bridge
  image_transport
  nav_msgs
  sensor_msgs
  ladybug2 
  image_geometry
  camera_calibration_parsers
)

#find_package(OpenCV REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS
	thread
	system
)


set(CMAKE_PREFIX_PATH "/media/anarita/Ana_Rita/opencv-3.2.0")
find_package(OpenCV 3.1 REQUIRED)
set(OpenCV_INCLUDE_DIRS "/media/anarita/Ana_Rita/opencv-3.2.0/include")

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE} -msse3 -I/usr/local/boost_1_52_0 -I../")

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES my_mono_vo
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs tf cv_bridge image_transport ladybug2 image_geometry camera_calibration_parsers
    DEPENDS system_lib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    include
)

add_library(Utils
    src/utils.cpp
)

target_link_libraries(Utils
    ${catkin_LIBRARIES}
)

add_library(MulticamOdometer
    src/MonoOdometer5.cpp
    src/MonoOdometer8.cpp
    src/MulticamOdometer.cpp
    src/Match.cpp
)

target_link_libraries(MulticamOdometer
    Utils
)

add_library(MulticamVOPipeline
    src/MulticamVOPipeline.cpp
)

target_link_libraries(MulticamVOPipeline
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}    
    MulticamOdometer    
)

add_executable(multicam_vo_node
    src/main.cpp
)

target_link_libraries(multicam_vo_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    MulticamVOPipeline
)

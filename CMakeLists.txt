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

find_package(OpenCV REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS
	thread
	system
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")

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

add_library(Optimization
    src/Points.cpp
    src/ISAMOptimizer.cpp
)

target_link_libraries(Optimization
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
    tbb
    tbbmalloc
    gtsam
    Utils
)

add_library(MultiFeatures
    src/FeatureDetector.cpp
    src/FeatureMatcher.cpp
    src/Feature.cpp
    src/Match.cpp    
)

target_link_libraries(MultiFeatures
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

add_library(MulticamOdometer
    src/MonoOdometer.cpp
    src/MulticamOdometer.cpp
)

target_link_libraries(MulticamOdometer
    MultiFeatures
    Utils
)

add_library(MulticamVOPipeline
    src/MulticamVOPipeline.cpp
)

target_link_libraries(MulticamVOPipeline
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    Optimization
    MultiFeatures
    MulticamOdometer    
)

add_library(MulticamVOSimPipeline
    src/MulticamVOSimPipeline.cpp
)

target_link_libraries(MulticamVOSimPipeline
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    MulticamOdometer
    MultiFeatures
    Optimization
)

add_executable(multicam_vo_node
    src/main.cpp
)

target_link_libraries(multicam_vo_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    MulticamVOPipeline
)

add_executable(multicam_vo_sim_node
    src/main_sim.cpp
)

target_link_libraries(multicam_vo_sim_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    MulticamVOSimPipeline
)



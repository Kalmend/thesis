cmake_minimum_required(VERSION 2.8.3)
project(capture_vad_sphinx)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  audio_common_msgs
)

## System dependencies are found with CMake's conventions
find_package(PkgConfig)
pkg_check_modules(GST gstreamer-0.10 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GST_INCLUDE_DIRS})
catkin_package()


add_executable(capture_vad_sphinx src/capture_vad_sphinx.cpp)
target_link_libraries(capture_vad_sphinx  ${catkin_LIBRARIES} ${GST_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(capture_vad_sphinx  ${catkin_EXPORTED_TARGETS})

# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(capture_vad_speex)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg audio_common_msgs)

## Declare ROS messages and services
add_service_files(FILES GetSinkCapabilities.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

find_package(PkgConfig)
pkg_check_modules(GST1.0 gstreamer-1.0 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GST1.0_INCLUDE_DIRS} ${libgstreamer0.10-dev_DIRS})

add_executable(capture_vad_speex src/capture_vad_speex.cpp)
target_link_libraries(capture_vad_speex ${catkin_LIBRARIES} ${GST1.0_LIBRARIES} ${Boost_LIBRARIES} ${libgstreamer0.10-dev_LIBRARIES})
add_dependencies(capture_vad_speex capture_vad_speex_generate_messages_cpp)

install(TARGETS capture_vad_speex
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

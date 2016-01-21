cmake_minimum_required(VERSION 2.8.3)

project(capture_vad_speex)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation audio_common_msgs)

find_package(PkgConfig)
pkg_check_modules(GST1.0 gstreamer-1.0 REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GST1.0_INCLUDE_DIRS})
add_service_files(FILES GetSinkCapabilities.srv)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS std_msgs message_runtime)

add_executable(capture_vad_speex src/capture_vad_speex.cpp)
target_link_libraries(capture_vad_speex ${catkin_LIBRARIES} ${GST1.0_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(capture_vad_speex ${catkin_EXPORTED_TARGETS})




install(TARGETS capture_vad_speex
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

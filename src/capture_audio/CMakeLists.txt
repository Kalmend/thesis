cmake_minimum_required(VERSION 2.8.3)

project(capture_audio)

find_package(catkin REQUIRED COMPONENTS roscpp audio_common_msgs)

find_package(PkgConfig)
pkg_check_modules(GST1.0 gstreamer-1.0 REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GST1.0_INCLUDE_DIRS})

catkin_package()

add_executable(capture_audio src/capture_audio.cpp)
target_link_libraries(capture_audio ${catkin_LIBRARIES} ${GST1.0_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(capture_audio ${catkin_EXPORTED_TARGETS})

install(TARGETS capture_audio
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

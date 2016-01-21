cmake_minimum_required(VERSION 2.8.3)

project(play_audio_speex)

find_package(catkin REQUIRED COMPONENTS roscpp audio_common_msgs capture_vad_speex)
#/usr/lib/x86_64-linux-gnu/libgstapp-1.0.so.0
find_package(PkgConfig)
pkg_check_modules(GST1.0 gstreamer-1.0 REQUIRED)
pkg_check_modules(GSTAPP gstreamer-app-1.0 REQUIRED)

MESSAGE( STATUS "GST1.0 LIBS: " ${GST1.0_LIBRARIES} )
MESSAGE( STATUS "GSTAPP LIBS: " ${GSTAPP_LIBRARIES} )

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GST1.0_INCLUDE_DIRS})

catkin_package()

add_executable(play_audio_speex src/play_audio_speex.cpp)
target_link_libraries(play_audio_speex ${GSTAPP_LIBRARIES} ${catkin_LIBRARIES} ${GST1.0_LIBRARIES} ${Boost_LIBRARIES}) 
add_dependencies(play_audio_speex ${catkin_EXPORTED_TARGETS})

install(TARGETS play_audio_speex 
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

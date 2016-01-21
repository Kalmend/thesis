cmake_minimum_required(VERSION 2.8.3)

project(speech_recognition_speex)

find_package(catkin REQUIRED COMPONENTS roscpp audio_common_msgs capture_vad_speex)
#/usr/lib/x86_64-linux-gnu/libgstapp-1.0.so.0
find_package(PkgConfig)
pkg_check_modules(GST1.0 gstreamer-1.0 REQUIRED)
pkg_check_modules(GSTAPP gstreamer-app-1.0 REQUIRED)

MESSAGE( STATUS "GST1.0 LIBS: " ${GST1.0_LIBRARIES} )
MESSAGE( STATUS "GSTAPP LIBS: " ${GSTAPP_LIBRARIES} )

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GST1.0_INCLUDE_DIRS})

catkin_package()

add_executable(speech_recognition_speex src/speech_recognition_speex.cpp)
target_link_libraries(speech_recognition_speex ${GSTAPP_LIBRARIES} ${catkin_LIBRARIES} ${GST1.0_LIBRARIES} ${Boost_LIBRARIES}) 
add_dependencies(speech_recognition_speex ${catkin_EXPORTED_TARGETS})

install(TARGETS speech_recognition_speex 
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

cmake_minimum_required(VERSION 2.8.3)

project(speech_recognition_simple)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
)

find_package(PkgConfig)
pkg_check_modules(GST1.0 gstreamer-1.0 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GST1.0_INCLUDE_DIRS})

add_executable(speech_recognition_simple src/speech_recognition_simple.cpp)
target_link_libraries(speech_recognition_simple ${catkin_LIBRARIES} ${GST1.0_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(speech_recognition_simple ${catkin_EXPORTED_TARGETS})

install(TARGETS speech_recognition_simple
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

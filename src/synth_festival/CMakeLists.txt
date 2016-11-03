cmake_minimum_required(VERSION 2.8.3)
project(synth_festival)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# Add custom find to cmake
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH};")
find_package(Festival REQUIRED)
find_package(EST REQUIRED)
find_package(Boost REQUIRED COMPONENTS locale)

catkin_package(CATKIN_DEPENDS message_runtime)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${Festival_INCLUDE_DIR} ${EST_INCLUDE_DIR})

add_executable(synth_festival src/synth_festival.cpp)
target_link_libraries(synth_festival ${catkin_LIBRARIES} ${Festival_LIBRARIES} ${EST_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(synth_festival ${catkin_EXPORTED_TARGETS})

install(TARGETS synth_festival
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

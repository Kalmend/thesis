cmake_minimum_required(VERSION 2.8.3)
project(chatbot)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  move_base_msgs
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
)

include(FindPkgConfig)
find_package(PkgConfig)
find_package(Boost REQUIRED COMPONENTS thread )

## action messages
add_action_files (
  DIRECTORY action
  FILES NamedMoveBase.action
)
## Declare ROS messages and services
add_service_files(FILES Respond.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES actionlib_msgs std_msgs move_base_msgs)
catkin_package( CATKIN_DEPENDS message_runtime actionlib_msgs)

MESSAGE(STATUS "LINK_EXEC: " ${CMAKE_CXX_LINK_EXECUTABLE})
MESSAGE(STATUS "LINKER: " ${CMAKE_LINKER})
MESSAGE(STATUS "COMPILER: " ${CMAKE_CXX_COMPILER})

include_directories(
  include 
  ${Boost_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS} 
)

add_executable(chatbot src/chatbot_core.cpp)
target_link_libraries(chatbot ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(chatbot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


install(TARGETS chatbot
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

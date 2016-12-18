cmake_minimum_required(VERSION 2.8.3)
project(robot_prolog_connection)

set(CMAKE_CXX_FLAGS -std=c++11)

find_package(
  catkin
  REQUIRED
    prolog_common
    prolog_msgs
    prolog_serialization
    roscpp
    roscpp_nodewrap
    actionlib
    chatbot
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    robot_prolog_connection
  DEPENDS
    chatbot
    prolog_common
    prolog_msgs
    prolog_serialization
    roscpp
    roscpp_nodewrap
)

IF(NOT EXISTS  ${CMAKE_SOURCE_DIR}/../prolog_files/HR_dialogue6.pl)
    message( FATAL_ERROR "Could not find chat core! missing "  ${CMAKE_SOURCE_DIR}/../prolog_files/HR_dialogue6.pl )
ENDIF (NOT EXISTS  ${CMAKE_SOURCE_DIR}/../prolog_files/HR_dialogue6.pl)
add_definitions(-DCHAT_CORE="${CMAKE_SOURCE_DIR}/../prolog_files/HR_dialogue6.pl")

IF(NOT EXISTS  ${CMAKE_SOURCE_DIR}/../prolog_files/input.txt)
    file(WRITE ${CMAKE_SOURCE_DIR}/../prolog_files/input.txt)
ENDIF (NOT EXISTS  ${CMAKE_SOURCE_DIR}/../prolog_files/input.txt)
add_definitions(-DINPUT_FILE="${CMAKE_SOURCE_DIR}/../prolog_files/input.txt")

IF(NOT EXISTS  ${CMAKE_SOURCE_DIR}/../prolog_files/output.txt)
    file(WRITE ${CMAKE_SOURCE_DIR}/../prolog_files/output.txt)
ENDIF (NOT EXISTS  ${CMAKE_SOURCE_DIR}/../prolog_files/output.txt)
add_definitions(-DOUTPUT_FILE="${CMAKE_SOURCE_DIR}/../prolog_files/output.txt")

IF(NOT EXISTS  ${CMAKE_SOURCE_DIR}/../prolog_files/task.txt)
    file(WRITE ${CMAKE_SOURCE_DIR}/../prolog_files/task.txt)
ENDIF (NOT EXISTS  ${CMAKE_SOURCE_DIR}/../prolog_files/task.txt)
add_definitions(-DTASK_FILE="${CMAKE_SOURCE_DIR}/../prolog_files/task.txt")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  robot_prolog_connection
    src/Client.cpp
    src/RobotPrologConnection.cpp
    src/Query.cpp
    src/QueryProxy.cpp
    src/ServiceClient.cpp
    src/PrioritizedTask.cpp
)

target_link_libraries(
  robot_prolog_connection
    ${catkin_LIBRARIES}
)

add_executable(
  robot_prolog_connection_node
    src/main.cpp
)

target_link_libraries(
  robot_prolog_connection_node
    robot_prolog_connection
    ${catkin_LIBRARIES}
)

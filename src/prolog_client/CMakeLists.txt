cmake_minimum_required(VERSION 2.8.3)
project(prolog_client)

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
    prolog_client
  DEPENDS
    chatbot
    prolog_common
    prolog_msgs
    prolog_serialization
    roscpp
    roscpp_nodewrap
)

IF(NOT EXISTS  ${CMAKE_SOURCE_DIR}/../HR_dialogue5.pl)
    message( FATAL_ERROR "Could not find chat core! missing "  ${CMAKE_SOURCE_DIR}/../HR_dialogue5.pl )
ENDIF (NOT EXISTS  ${CMAKE_SOURCE_DIR}/../HR_dialogue5.pl)
add_definitions(-DCHAT_CORE="${CMAKE_SOURCE_DIR}/../HR_dialogue5.pl")

IF(NOT EXISTS  ${CMAKE_SOURCE_DIR}/../input.txt)
    file(WRITE ${CMAKE_SOURCE_DIR}/../input.txt)
ENDIF (NOT EXISTS  ${CMAKE_SOURCE_DIR}/../input.txt)
add_definitions(-DINPUT_FILE="${CMAKE_SOURCE_DIR}/../input.txt")

IF(NOT EXISTS  ${CMAKE_SOURCE_DIR}/../output.txt)
    file(WRITE ${CMAKE_SOURCE_DIR}/../output.txt)
ENDIF (NOT EXISTS  ${CMAKE_SOURCE_DIR}/../output.txt)
add_definitions(-DOUTPUT_FILE="${CMAKE_SOURCE_DIR}/../output.txt")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  prolog_client
    src/Client.cpp
    src/InteractiveClient.cpp
    src/Query.cpp
    src/QueryProxy.cpp
    src/ServiceClient.cpp
)

target_link_libraries(
  prolog_client
    ${catkin_LIBRARIES}
)

add_executable(
  interactive_client_node
    src/interactive_client_node.cpp
)

target_link_libraries(
  interactive_client_node
    prolog_client
    ${catkin_LIBRARIES}
)

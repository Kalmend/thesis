cmake_minimum_required(VERSION 2.8.3)
project(prolog_swi)

set(CMAKE_CXX_FLAGS -std=c++11)

find_package(
  catkin
  REQUIRED
    prolog_common
    roscpp
)

find_package(PkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)

#Bug in latest swi-prolog metadata sets the wrong path. lets check and fix:
MESSAGE( STATUS "SWI-prolog.h include before fix:" ${SWIPL_INCLUDE_DIRS} )
string(REPLACE prlog prolog SWIPL_INCLUDE_DIRS ${SWIPL_INCLUDE_DIRS})
MESSAGE( STATUS "SWI-prolog.h include after fix:" ${SWIPL_INCLUDE_DIRS} )

find_program(SWIPL_EXECUTABLE swipl)
add_definitions(-DSWIPL_EXECUTABLE="\\"${SWIPL_EXECUTABLE}\\"")

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    prolog_swi
  DEPENDS
    prolog_common
    roscpp
)

include_directories(
  include
  ${SWIPL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(
  prolog_swi
    src/Bindings.cpp
    src/Context.cpp
    src/Engine.cpp
    src/Exception.cpp
    src/Frame.cpp
    src/Query.cpp
    src/Term.cpp
)

target_link_libraries(
  prolog_swi
    ${SWIPL_LIBRARIES}
    ${catkin_LIBRARIES}
)

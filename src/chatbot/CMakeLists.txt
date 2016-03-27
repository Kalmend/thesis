cmake_minimum_required(VERSION 2.8.3)
project(chatbot)
add_definitions(-std=c++11)
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
#set(CMAKE_CXX_FLAGS "$(shell pkg-config swipl --cflags) $(CMAKE_CXX_FLAGS)")
#set(CMAKE_EXE_LINKER_FLAGS "--std=c++11 -shared -ld g++" ) 
#set(CMAKE_LINKER swipl-ld ) 
#set(CMAKE_CXX_COMPILER g++ ) 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

include(FindPkgConfig)
find_package(PkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)
find_package(Boost REQUIRED COMPONENTS thread)
link_directories(${SWIPL_LIBRARY_DIRS})

find_program(SWIPL_EXECUTABLE swipl)
add_definitions(-DSWIPL_EXECUTABLE="${SWIPL_EXECUTABLE}")

 STRING(REGEX REPLACE "prlog" "prolog" 
        SWIPL_INCLUDE_DIRS "${SWIPL_INCLUDE_DIRS}")
        
MESSAGE(STATUS "SWIPL_DIR: " ${SWIPL_INCLUDE_DIRS})
MESSAGE(STATUS "SWIPL_EXEC: " ${SWIPL_EXECUTABLE})
MESSAGE(STATUS "LINK_EXEC: " ${CMAKE_CXX_LINK_EXECUTABLE})
MESSAGE(STATUS "LINKER: " ${CMAKE_LINKER})
MESSAGE(STATUS "COMPILER: " ${CMAKE_CXX_COMPILER})


include_directories(
  include 
  ${SWIPL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS} 
)

catkin_package()

add_executable(chatbot src/chat_core.cpp)
target_link_libraries(chatbot ${catkin_LIBRARIES} ${GST1.0_LIBRARIES} ${SWIPL_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(chatbot ${catkin_EXPORTED_TARGETS})


install(TARGETS chatbot
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

cmake_minimum_required(VERSION 2.8.3)
project(roseus_bt)

SET(Boost_USE_STATIC_LIBS ON)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  behaviortree_ros
)

find_package(TinyXML2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS log)
find_package(fmt)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES
 CATKIN_DEPENDS
 DEPENDS TinyXML2 fmt
)

# git submodule update
find_package(Git QUIET)
option(GIT_SUBMODULE "Check submodules during build" ON)
if(GIT_SUBMODULE)
  message(STATUS "Submodule update")
  execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --remote include/rosbridgecpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE GIT_SUBMOD_RESULT)
  if(NOT GIT_SUBMOD_RESULT EQUAL "0")
    message(FATAL_ERROR "git submodule update failed with ${GIT_SUBMOD_RESULT}")
  endif()
endif()

include_directories( include ${catkin_INCLUDE_DIRS} ${TinyXML2_INCUDE_DIRS})


add_executable(create_bt_package src/create_bt_package.cpp)
add_dependencies(create_bt_package ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(create_bt_package ${catkin_LIBRARIES} ${TinyXML2_LIBRARIES} ${Boost_LIBRARIES} fmt::fmt)

add_executable(create_bt_tutorials src/create_bt_tutorials.cpp)
add_dependencies(create_bt_tutorials ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(create_bt_tutorials ${catkin_LIBRARIES} ${TinyXML2_LIBRARIES} ${Boost_LIBRARIES} fmt::fmt)

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
add_definitions(-Wall)
rosbuild_init()

set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

if(UNIX AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR} CACHE PATH "roseus install prefix" FORCE )
endif()

#rosbuild_add_boost_directories()

include_directories(/usr/include /usr/X11R6/include /$(EUSDIR)/include)
#link_directories()
rosbuild_add_library(roseus roseus.cpp)
rosbuild_add_library(eustf eustf.cpp)
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES amd64* OR
    ${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64* )
rosbuild_add_compile_flags(roseus "-O2 -Dx86_64 -DLinux -D_REENTRANT -DVERSION=\"8.26\" -DTHREADED -DPTHREAD -DX11R6_1 -Wno-write-strings -Wno-comment")
rosbuild_add_compile_flags(eustf "-O2 -Dx86_64 -DLinux -D_REENTRANT -DVERSION=\"8.26\" -DTHREADED -DPTHREAD -DX11R6_1 -Wno-write-strings -Wno-comment")
else()
rosbuild_add_compile_flags(roseus "-O2 -Di486 -DLinux -D_REENTRANT -DVERSION=\"8.26\" -DTHREADED -DPTHREAD -DX11R6_1 -Wno-write-strings -Wno-comment")
rosbuild_add_compile_flags(eustf "-O2 -Di486 -DLinux -D_REENTRANT -DVERSION=\"8.26\" -DTHREADED -DPTHREAD -DX11R6_1 -Wno-write-strings -Wno-comment")
endif()
rosbuild_add_link_flags(eustf)
#target_link_libraries(roseus)
#rosbuild_link_boost(roseus thread)

set_target_properties(roseus PROPERTIES PREFIX "" SUFFIX ".so")
set_target_properties(eustf PROPERTIES PREFIX "" SUFFIX ".so")

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/euslisp)

rosbuild_gensrv()
rosbuild_genmsg()

#add_subdirectory(test EXCLUDE_FROM_ALL)

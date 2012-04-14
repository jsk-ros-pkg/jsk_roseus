cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
add_definitions(-Wall)
rosbuild_init()

set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

if(UNIX AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR} CACHE PATH "roseus install prefix" FORCE )
endif()


find_program (SVNVERSION_CMD svnversion)
execute_process (COMMAND "${SVNVERSION_CMD}" ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE SVNVERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE)
message (STATUS "Build svn revision: ${SVNVERSION}")

#rosbuild_add_boost_directories()

rosbuild_find_ros_package(euslisp)
include_directories(/usr/include /usr/X11R6/include ${euslisp_PACKAGE_PATH}/jskeus/eus/include)
#link_directories()
rosbuild_add_library(roseus roseus.cpp)
rosbuild_add_library(eustf eustf.cpp)
rosbuild_add_library(roseus_c_util roseus_c_util.c)

# compile flags
set(COMPILE_FLAGS "-O2 -Wno-write-strings -Wno-comment")
add_definitions(-Di486 -DLinux -D_REENTRANT -DVERSION='\"${8.26}\"' -DTHREADED -DPTHREAD -DX11R6_1)
add_definitions(-DSVNVERSION='\"r${SVNVERSION}\"')
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES amd64* OR
   ${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64* )
 add_definitions(-Dx86_64)
else()
 add_definitions(-Di486)
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES Darwin)
 add_definitions(-Dx86_64)
 set(CMAKE_SHARED_LIBRARY_CREATE_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_CREATE_CXX_FLAGS} -flat_namespace -undefined suppress")
endif()

rosbuild_add_compile_flags(roseus ${COMPILE_FLAGS})
rosbuild_add_compile_flags(eustf ${COMPILE_FLAGS})
rosbuild_add_compile_flags(roseus_c_util ${COMPILE_FLAGS})
#rosbuild_add_link_flags(eustf)

#target_link_libraries(roseus)
#rosbuild_link_boost(roseus thread)

set_target_properties(roseus PROPERTIES PREFIX "" SUFFIX ".so")
set_target_properties(eustf PROPERTIES PREFIX "" SUFFIX ".so")
set_target_properties(roseus_c_util PROPERTIES PREFIX "" SUFFIX ".so")

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/euslisp)

rosbuild_gensrv()
rosbuild_genmsg()

rosbuild_add_executable(test/simple_execute_ref_server test/simple_execute_ref_server.cpp)
rosbuild_add_rostest(test/test-talker-listener.launch)
rosbuild_add_rostest(test/test-add-two-ints.launch)
rosbuild_add_rostest(test/test-simple-client.launch)
rosbuild_add_rostest(test/test-simple-client-wait.launch)
rosbuild_add_rostest(test/test-fibonacci.launch)
rosbuild_add_rostest(test/test-tf.launch)
rosbuild_add_rostest(test/test-disconnect.launch)

#add_subdirectory(test EXCLUDE_FROM_ALL)

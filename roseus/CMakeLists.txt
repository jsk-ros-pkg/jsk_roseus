cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
add_definitions(-Wall)
rosbuild_init()


set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

if(UNIX AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR} CACHE PATH "roseus install prefix" FORCE )
endif()
rospack_add_boost_directories()

rospack_add_library(roseus roseus.cpp)
rospack_add_compile_flags(roseus "-O2 -Di486 -DLinux -D_REENTRANT -DVERSION=\"8.26\" -DTHREADED -DPTHREAD -DX11R6_1 -I/usr/include -I/usr/X11R6/include -I$(EUSDIR)/include -Wno-write-strings -Wno-comment")
rospack_link_boost(roseus thread)

## custom compiling doesn't detect dependency changes well
# find_ros_package(roscpp_sessions)
# _rospack_invoke(rosoct rosoct cflags --lang=cpp --attrib=cflags export)
# _rospack_invoke(rosoct rosoct lflags --lang=cpp --attrib=lflags export)

# add_custom_command(
#   OUTPUT ${OCTAVE_MEX}
#   COMMAND ${MKOCTFILE}
#   ARGS --mex ${ROS_COMPILE_FLAGS} ${ROS_LINK_FLAGS} ${rosoct_cflags} ${rosoct_lflags} -o \"${OCTAVE_MEX}\" \"${CMAKE_CURRENT_SOURCE_DIR}/rosoct.cpp\"
#   DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/rosoct.cpp ${roscpp_sessions_PACKAGE_PATH}/include/ros/session.h
#   )
# add_custom_target(rosoct_octave ALL DEPENDS ${OCTAVE_MEX})

gensrv()
genmsg()

rospack_add_executable(hrp2v_odom hrp2_odom.cpp)
#rospack_add_executable(slam_odometry slam_odometry.cpp)
#add_subdirectory(test EXCLUDE_FROM_ALL)


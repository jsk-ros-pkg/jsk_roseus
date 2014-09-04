# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(roseus)

# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS message_generation roslang roscpp rospack rostest actionlib actionlib_msgs visualization_msgs tf geometry_msgs std_msgs std_srvs sensor_msgs visualization_msgs tf2_ros euslisp geneus)

add_definitions(-Wall)
#
execute_process(COMMAND rosversion tf2_ros OUTPUT_VARIABLE TF2_ROS_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS "tf2_ros version: ${TF2_ROS_VERSION}")
if(${TF2_ROS_VERSION} VERSION_LESS  0.4.0)
  add_definitions(-DTF2_ROS_VERSION_3)
  message(STATUS "compile with -DTF2_ROS_VERSION_3")
endif()

#set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )
#if(UNIX AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
#  set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR} CACHE PATH "roseus install prefix" FORCE )
#endif()


set(ENV{LANG} "C")
execute_process (COMMAND git rev-parse --short HEAD
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  OUTPUT_VARIABLE REPOVERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE)
message (STATUS "Build repo revision: ${REPOVERSION}")

#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()

if(EXISTS ${euslisp_SOURCE_DIR}/jskeus)
  set(euslisp_PACKAGE_PATH ${euslisp_SOURCE_DIR})
else()
  set(euslisp_PACKAGE_PATH ${euslisp_PREFIX}/share/euslisp)
endif()
message("-- Set euslisp_PACKAGE_PATH to ${euslisp_PACKAGE_PATH}")
set(euslisp_INCLUDE_DIRS ${euslisp_PACKAGE_PATH}/jskeus/eus/include)
message("-- Set euslisp_INCLUDE_DIRS to ${euslisp_INCLUDE_DIRS}")
include_directories(/usr/include /usr/X11R6/include ${euslisp_INCLUDE_DIRS})
add_library(roseus roseus.cpp)
add_library(eustf eustf.cpp)
add_library(roseus_c_util roseus_c_util.c)
target_link_libraries(roseus ${rospack_LIBRARIES} ${roscpp_LIBRARIES})
target_link_libraries(eustf  ${roscpp_LIBRARIES} ${tf_LIBRARIES} ${tf2_ros_LIBRARIES})
set_target_properties(roseus PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/euslisp)
set_target_properties(eustf PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/euslisp)
set_target_properties(roseus_c_util PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/euslisp)

# compile flags
add_definitions(-O2 -Wno-write-strings -Wno-comment)
add_definitions(-Di486 -DLinux -D_REENTRANT -DVERSION='\"9.00\"' -DTHREADED -DPTHREAD -DX11R6_1)
add_definitions('-DREPOVERSION="\\"${REPOVERSION}\\""')
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

include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(roseus ${Boost_LIBRARIES})

set_target_properties(roseus PROPERTIES PREFIX "" SUFFIX ".so")
set_target_properties(eustf PROPERTIES PREFIX "" SUFFIX ".so")
set_target_properties(roseus_c_util PROPERTIES PREFIX "" SUFFIX ".so")

add_service_files(
  FILES AddTwoInts.srv StringString.srv
)
add_message_files(
  FILES String.msg StringStamped.msg
)

# CATKIN_MIGRATION: removed during catkin migration
#file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/test)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
add_executable(simple_execute_ref_server test/simple_execute_ref_server.cpp)
target_link_libraries(simple_execute_ref_server ${roscpp_LIBRARIES} ${actionlib_LIBRARIES})
set_target_properties(simple_execute_ref_server PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
add_rostest(test/test-talker-listener.test)
add_rostest(test/test-add-two-ints.test)
add_rostest(test/test-simple-client.test)
add_rostest(test/test-simple-client-wait.test)
add_rostest(test/test-actionlib.test)
add_rostest(test/test-roseus.test)
add_rostest(test/test-tf.test)
add_rostest(test/test-disconnect.test)
add_rostest(test/test-multi-queue.test)
add_rostest(test/test-genmsg.catkin.test)
add_rostest(test/test-genmsg-oneworkspace.catkin.launch) # use launch not to run on travis/catkin

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    DEPENDS roslang roscpp rospack actionlib actionlib_msgs visualization_msgs tf geometry_msgs std_msgs std_srvs sensor_msgs visualization_msgs actionlib_tutorials coreutils tf2_ros
    CATKIN_DEPENDS message_runtime # euslisp TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

# copy bin/roseus to global bin
execute_process(COMMAND cmake -E make_directory ${CATKIN_DEVEL_PREFIX}/bin/)
execute_process(COMMAND cmake -E copy ${PROJECT_SOURCE_DIR}/bin/roseus ${CATKIN_DEVEL_PREFIX}/bin/roseus)
install(PROGRAMS bin/roseus
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
# install
install(PROGRAMS bin/roseus
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(CODE "execute_process(COMMAND cmake -E make_directory \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION} RESULT_VARIABLE _mkdir_roseus_result OUTPUT_VARIABLE _mkdir_roseus_output)
              message(\"cmake -E make_directory \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION} returns \${_mkdir_roseus_result} ... \${_mkdir_roseus_output}.\")
              execute_process(COMMAND cmake -E create_symlink ../${CATKIN_PACKAGE_BIN_DESTINATION}/roseus roseus WORKING_DIRECTORY \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}/ RESULT_VARIABLE _install_roseus_result OUTPUT_VARIABLE _install_roseus_output)
              message(\"create_symlink ../${CATKIN_PACKAGE_BIN_DESTINATION}/roseus roseus WORKING_DIRECTORY \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}/ returns \${_install_roseus_result} ... \${_install_roseus_output}.\")")
install(DIRECTORY euslisp test scripts cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)





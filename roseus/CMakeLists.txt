# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(roseus)

# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS message_generation roslang roscpp rospack topic_tools actionlib actionlib_msgs visualization_msgs tf geometry_msgs std_msgs std_srvs sensor_msgs visualization_msgs tf2_ros dynamic_reconfigure actionlib_tutorials geneus)

configure_file(${PROJECT_SOURCE_DIR}/bin/roseus.in ${PROJECT_SOURCE_DIR}/bin/roseus @ONLY)

add_definitions(-Wall)
#
execute_process(COMMAND rosversion tf2_ros OUTPUT_VARIABLE TF2_ROS_VERSION RESULT_VARIABLE TF2_ROS_RESULT OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT ${TF2_ROS_RESULT} EQUAL 0)
  set(TF2_ROS_VERSION ${tf2_ros_VERSION})
endif()
message(STATUS "tf2_ros version: ${TF2_ROS_VERSION} (${TF2_ROS_RESULT})")
if(TF2_ROS_VERSION)
  if(${TF2_ROS_VERSION} VERSION_LESS  0.4.0)
    add_definitions(-DTF2_ROS_VERSION_3)
    message(STATUS "compile with -DTF2_ROS_VERSION_3")
  endif()
endif()

#set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )
#if(UNIX AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
#  set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR} CACHE PATH "roseus install prefix" FORCE )
#endif()


set(ENV{LANG} "C")
execute_process (COMMAND git describe --tags HEAD
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  OUTPUT_VARIABLE REPOVERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT REPOVERSION)
  execute_process(COMMAND grep version ${PROJECT_SOURCE_DIR}/package.xml
    COMMAND sed -e s/[^0-9.]//g
    OUTPUT_VARIABLE REPOVERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
message (STATUS "Build repo revision: ${REPOVERSION}")

#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()

if(APPLE)
  if(NOT EXISTS /usr/local/opt/jskeus/eus)  # $EUSDIR in OS X
    message(FATAL_ERROR "jskeus is not installed via Homebrew, Please run `brew install homebrew/x11/jskeus`")
  endif()
  set(euslisp_INCLUDE_DIRS /usr/local/opt/jskeus/eus/include)
else()
  find_package(euslisp REQUIRED)
  find_package(jskeus REQUIRED)
  if(NOT euslisp_INCLUDE_DIRS)
    if(EXISTS ${euslisp_SOURCE_DIR}/jskeus)
      set(euslisp_PACKAGE_PATH ${euslisp_SOURCE_DIR})
    else()
      set(euslisp_PACKAGE_PATH ${euslisp_PREFIX}/share/euslisp)
    endif()
    message("-- Set euslisp_PACKAGE_PATH to ${euslisp_PACKAGE_PATH}")
    set(euslisp_INCLUDE_DIRS ${euslisp_PACKAGE_PATH}/jskeus/eus/include)
  endif()
endif()
message("-- Set euslisp_INCLUDE_DIRS to ${euslisp_INCLUDE_DIRS}")
include_directories(/usr/include /usr/X11R6/include ${euslisp_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_library(roseus roseus.cpp)
add_library(eustf eustf.cpp)
add_library(roseus_c_util roseus_c_util.c)
target_link_libraries(roseus ${rospack_LIBRARIES} ${roscpp_LIBRARIES})
target_link_libraries(eustf  ${roscpp_LIBRARIES} ${tf_LIBRARIES} ${tf2_ros_LIBRARIES})
set_target_properties(roseus PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/euslisp)
set_target_properties(eustf PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/euslisp)
set_target_properties(roseus_c_util PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/euslisp)

# compile flags
add_definitions(-O2 -DNDEBUG -Wno-write-strings -Wno-unused-but-set-variable -Wno-comment)
add_definitions(-Di486 -DLinux -D_REENTRANT -DVERSION='\"9.00\"' -DTHREADED -DPTHREAD -DX11R6_1)
add_definitions('-DREPOVERSION="\\"${REPOVERSION}\\""')
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "amd64(.*)" OR
   "${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64(.*)" OR
   "${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64(.*)" )
 add_definitions(-Dx86_64)
else()
 add_definitions(-Di486)
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CMAKE_SHARED_LIBRARY_CREATE_C_FLAGS "${CMAKE_SHARED_LIBRARY_CREATE_C_FLAGS} -undefined dynamic_lookup")
  set(CMAKE_SHARED_LIBRARY_CREATE_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_CREATE_CXX_FLAGS} -undefined dynamic_lookup")
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
  FILES String.msg StringStamped.msg FixedArray.msg VariableArray.msg
)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)

# copy bin/roseus to global bin
add_custom_command(OUTPUT ${CATKIN_DEVEL_PREFIX}/bin/roseus
  COMMAND cmake -E make_directory ${CATKIN_DEVEL_PREFIX}/bin/
  COMMAND cmake -E copy ${PROJECT_SOURCE_DIR}/bin/roseus ${CATKIN_DEVEL_PREFIX}/bin/roseus
  DEPENDS bin/roseus)
add_custom_target(install_roseus ALL DEPENDS ${CATKIN_DEVEL_PREFIX}/bin/roseus roseus eustf roseus_c_util)
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    DEPENDS euslisp jskeus
    CATKIN_DEPENDS roslang roscpp rospack actionlib actionlib_msgs visualization_msgs tf geometry_msgs std_msgs std_srvs sensor_msgs visualization_msgs actionlib_tutorials tf2_ros message_runtime
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
    EXPORTED_TARGETS install_roseus
    CFG_EXTRAS roseus.cmake
)

install(PROGRAMS bin/roseus
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
# install
install(CODE "execute_process(COMMAND cmake -E create_symlink ../../bin/roseus roseus WORKING_DIRECTORY \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/ RESULT_VARIABLE _install_roseus_result OUTPUT_VARIABLE _install_roseus_output)
              message(\"create_symlink ../../bin/roseus roseus WORKING_DIRECTORY \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/ returns \${_install_roseus_result} ... \${_install_roseus_output}.\")")
install(DIRECTORY euslisp test scripts cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)


include(cmake/roseus.cmake)

generate_eusdoc(euslisp/roseus.l "ROS")
generate_eusdoc(euslisp/roseus-utils.l)
generate_eusdoc(euslisp/eustf.l)
generate_eusdoc(euslisp/actionlib.l)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # CATKIN_MIGRATION: removed during catkin migration
  #file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/test)
  #set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  add_executable(simple_execute_ref_server test/simple_execute_ref_server.cpp)
  target_link_libraries(simple_execute_ref_server ${roscpp_LIBRARIES} ${actionlib_LIBRARIES})
  set_target_properties(simple_execute_ref_server PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  add_rostest(test/test-talker-listener.test)
  add_rostest(test/test-talker-listener-private-topic.test)
  add_rostest(test/test-add-two-ints.test)
  add_rostest(test/test-service-callback.test)
  add_rostest(test/test-simple-client.test)
  add_rostest(test/test-simple-client-wait.test)
  add_rostest(test/test-simple-client-500.test)
  add_rostest(test/test-simple-client-wait-500.test)
  add_rostest(test/test-simple-client-cancel.test)
  add_rostest(test/test-actionlib.test)
  add_rostest(test/test-roseus.test)
  add_rostest(test/test-tf.test)
  add_rostest(test/test-disconnect.test)
  add_rostest(test/test-multi-queue.test)
  add_rostest(test/test-namespace.test)
  add_rostest(test/test-parameter.test)
  add_rostest(test/test-anonymous.test)
  add_rostest(test/test-timer.test)
  # this seems to run other process and need pkill ... https://travis-ci.org/jsk-ros-pkg/jsk_roseus/jobs/233858193
  #add_rostest(test/test-genmsg.catkin.test)
  # this will not ok on running from run_tests, may be running test within launch file may not good.
  #add_rostest(test/test-genmsg-oneworkspace.catkin.launch) # use launch not to run on travis/catkin
  add_rostest(test/test-geneus.test)
  add_rostest(test/test-compile-message.test)
endif()

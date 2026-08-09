if($ENV{ROS_VERSION} EQUAL 2)
cmake_minimum_required(VERSION 3.8)
project(roseus)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

set(ENV_EUSDIR $ENV{EUSDIR})
if(ENV_EUSDIR)
  set(EUSDIR $ENV_EUSDIR)
else()
  set(EUSDIR /usr/share/euslisp) # for debian/ubuntu default package
endif()

if(EXISTS ${EUSDIR})
  message(STATUS "Found EusLisp: ${EUSDIR}")
else()
  message(FATAL_ERROR "EusLisp not found")
endif()
set(euslisp_INCLUDE_DIRS ${EUSDIR}/include)

ament_auto_add_library(roseus SHARED roseus2.cpp)
set_target_properties(roseus PROPERTIES
  PREFIX ""
)
list(REMOVE_ITEM ${PROJECT_NAME}_LIBRARIES roseus) # not export as library for other packages, because roseus is not a library but a plugin for euslisp

ament_generate_version_header(roseus)
target_compile_options(roseus
  PRIVATE -O2 -DNDEBUG -DLinux -D_REENTRANT -DTHREADED -DPTHREAD -DX11R6_1 -Dx86_64 -Wno-comment -Wno-deprecated-declarations)
target_include_directories(roseus PUBLIC ${euslisp_INCLUDE_DIRS})

# Generate ROS 2 interfaces from .msg/.srv files
# Use ${PROJECT_NAME}_interfaces to avoid target name conflict with the roseus library
# TestFixedArray/TestVariableArray are ROS 2 versions of FixedArray/VariableArray
# (replacing ROS 1 'time'/'duration' with builtin_interfaces equivalents)
rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
  msg/String.msg
  msg/StringStamped2.msg
  msg/TestFixedArray.msg
  msg/TestVariableArray.msg
  msg/TestName2.msg
  srv/AddTwoInts.srv
  srv/StringString.srv
  action/Fibonacci.action
  DEPENDENCIES std_msgs builtin_interfaces action_msgs unique_identifier_msgs
)

# Create typesupport symlinks: runtime looks for roseus__* but libraries are roseus_interfaces__*
# because rosidl_generate_interfaces target is named roseus_interfaces (to avoid name conflict)
set(_typesupport_suffixes
  rosidl_generator_c rosidl_typesupport_c rosidl_typesupport_cpp
  rosidl_typesupport_fastrtps_c rosidl_typesupport_fastrtps_cpp
  rosidl_typesupport_introspection_c rosidl_typesupport_introspection_cpp)
foreach(_suffix ${_typesupport_suffixes})
  install(CODE "execute_process(COMMAND \${CMAKE_COMMAND} -E create_symlink
    lib${PROJECT_NAME}_interfaces__${_suffix}.so
    \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/lib${PROJECT_NAME}__${_suffix}.so)")
endforeach()

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_clang_format(
    CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.clang-format"
    roseus2.cpp
  )
  set(_test_eus_l_dir "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_eus/${PROJECT_NAME}")
  add_test(
    NAME test_rosidl_generator_eus
    COMMAND roseus
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test-rosidl-generator-eus.l
    ${_test_eus_l_dir}
  )
  add_test(NAME test_compile_message
    COMMAND roseus ${CMAKE_CURRENT_SOURCE_DIR}/test/test-compile-message.l)
  add_test(NAME test_mark_lock
    COMMAND roseus ${CMAKE_CURRENT_SOURCE_DIR}/test/test-mark-lock-ros2.l)
  add_test(NAME test_unadvertise_service
    COMMAND roseus ${CMAKE_CURRENT_SOURCE_DIR}/test/test-unadvertise-service.l)
  add_test(NAME test_roseus
    COMMAND roseus ${CMAKE_CURRENT_SOURCE_DIR}/test/test-roseus-ros2.l)
  add_test(NAME test_nodename
    COMMAND roseus ${CMAKE_CURRENT_SOURCE_DIR}/test/test-nodename-ros2.l test_nodename)
  add_test(NAME test_nodename_dash
    COMMAND roseus ${CMAKE_CURRENT_SOURCE_DIR}/test/test-nodename-ros2.l test-nodename)
  add_test(NAME test_namespace_root
    COMMAND roseus ${CMAKE_CURRENT_SOURCE_DIR}/test/test-namespace-ros2.l /)
  add_test(NAME test_namespace_custom
    COMMAND bash -c "roseus ${CMAKE_CURRENT_SOURCE_DIR}/test/test-namespace-ros2.l /ns --ros-args -r __ns:=/ns")
  add_test(NAME test_parameter
    COMMAND roseus ${CMAKE_CURRENT_SOURCE_DIR}/test/param-test-ros2.l)
  add_test(NAME test_msg_serialize
    COMMAND roseus ${CMAKE_CURRENT_SOURCE_DIR}/test/test-msg-serialize-ros2.l)

  # Smoke tests finish in well under 10 s; 20 s leaves room for DDS discovery.
  add_launch_test(test/talker-listener.test.py TIMEOUT 20)
  add_launch_test(test/add-two-ints-server.test.py TIMEOUT 20)
  add_launch_test(test/add-two-ints-server-groupname.test.py TIMEOUT 20)
  add_launch_test(test/service-callback.test.py TIMEOUT 20)
  add_launch_test(test/multi-queue.test.py TIMEOUT 20)
  add_launch_test(test/fibonacci-action.test.py TIMEOUT 20)
  add_launch_test(test/fibonacci-action-ros2-api.test.py TIMEOUT 30)
  # Stress tests: the high iteration counts are the point
  add_launch_test(test/add-two-ints.test.py TIMEOUT 120)
  add_launch_test(test/follow-joint-trajectory.test.py TIMEOUT 60)
  add_launch_test(test/cdr-nested-empty-array.test.py TIMEOUT 60)

  set_tests_properties(
    test_mark_lock
    PROPERTIES TIMEOUT 20)
endif()

include(cmake/roseus_generate_dep_msgs.cmake)

install(FILES
  cmake/roseus_generate_dep_msgs.cmake
  DESTINATION share/${PROJECT_NAME}/cmake/
)

configure_file(bin/roseus2.in ${CMAKE_CURRENT_BINARY_DIR}/bin/roseus @ONLY)
file(CHMOD ${CMAKE_CURRENT_BINARY_DIR}/bin/roseus PERMISSIONS
  OWNER_READ OWNER_WRITE OWNER_EXECUTE
  GROUP_READ GROUP_EXECUTE
  WORLD_READ WORLD_EXECUTE)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/bin/roseus
  DESTINATION bin)

# Install roseus.so in euslisp/ dir (EusLisp load-library expects "roseus.so" in same dir as roseus.l)
install(
  TARGETS roseus
  LIBRARY DESTINATION "share/${PROJECT_NAME}/euslisp"
)

ament_auto_package(
  CONFIG_EXTRAS cmake/roseus-extras.cmake.in
  INSTALL_TO_SHARE cmake euslisp
)

else() # if($ENV{ROS_VERSION} EQUAL 2)

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

find_package(euslisp QUIET)
find_package(jskeus QUIET)
find_program(EUS_EXECUTABLE eus)
find_program(JSKEUS_EXECUTABLE irteus)
message("${euslisp_FOUND} ${jskeus_FOUND} ${EUS_EXECUTABLE} ${JSKEUS_EXECUTABLE} ${JSKEUS_EXECUTABLE}")
if(APPLE)
  if(NOT EXISTS /usr/local/opt/jskeus/eus)  # $EUSDIR in OS X
    message(FATAL_ERROR "jskeus is not installed via Homebrew, Please run `brew install homebrew/x11/jskeus`")
  endif()
  set(euslisp_INCLUDE_DIRS /usr/local/opt/jskeus/eus/include)
elseif((NOT ${euslisp_FOUND}) AND (NOT ${jskeus_FOUND}) AND (EXISTS ${EUS_EXECUTABLE}) AND (EXISTS ${JSKEUS_EXECUTABLE}) AND (EXISTS /usr/share/euslisp/include))
  set(euslisp_PACKAGE_PATH /usr/share/euslisp)
  set(euslisp_INCLUDE_DIRS /usr/share/euslisp/include)
  message("-- Set euslisp_PACKAGE_PATH to ${euslisp_PACKAGE_PATH}, this might be system installed euslisp")
  execute_process(COMMAND dpkg-query -W -f='\${Version}' euslisp
    OUTPUT_VARIABLE euslisp_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
  string(REPLACE "'" "" euslisp_VERSION "${euslisp_VERSION}")
  execute_process(COMMAND dpkg-query -W -f='\${Version}' jskeus
    OUTPUT_VARIABLE jskeus_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
  string(REPLACE "'" "" jskeus_VERSION "${jskeus_VERSION}")
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

# you need to run `catkin build --force-cmake` when you have updated eus.h
# update function using defun() for https://github.com/euslisp/EusLisp/pull/116
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/test-defun.c "#include \"eus.h\"\nvoid TEST();\nvoid test(void) {register context *ctx; pointer mod; defun(ctx,\"TEST\",mod,TEST,NULL);}")
get_directory_property(__CMAKE_COMPILE_DEFINITIONS DIRECTORY ${PROJECT_SOURCE_DIR} COMPILE_DEFINITIONS)
set(__CMAKE_COMPILE_DEFINITION "")
foreach(__D ${__CMAKE_COMPILE_DEFINITIONS})
  set(__CMAKE_COMPILE_DEFINITION ${__CMAKE_COMPILE_DEFINITION} -D${__D})
endforeach()
message(STATUS "Running test file to check defun() API, see https://github.com/euslisp/EusLisp/pull/116")
message(STATUS "COMMAND ${CMAKE_C_COMPILER} -I${euslisp_INCLUDE_DIRS} ${__CMAKE_COMPILE_DEFINITION} -c test-defun.c")

execute_process (COMMAND ${CMAKE_C_COMPILER} -I${euslisp_INCLUDE_DIRS} ${__CMAKE_COMPILE_DEFINITION} -c test-defun.c
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  RESULT_VARIABLE TEST_DEFUN)
file(WRITE ${PROJECT_SOURCE_DIR}/defun.h "// redefine defun for update defun() API (https://github.com/euslisp/EusLisp/pull/116)\n")
if(${TEST_DEFUN})
  message(STATUS "add defin() macro for euslisp < 9.24")
  file(APPEND ${PROJECT_SOURCE_DIR}/defun.h "#undef defun\n")
  file(APPEND ${PROJECT_SOURCE_DIR}/defun.h "#define defun(a, b, c, d, e) defun(a, b, c, d) // for EusLisp < 9.24\n")
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
  FILES String.msg StringStamped.msg FixedArray.msg VariableArray.msg TestName.msg
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
  add_rostest(test/test-add-two-ints-server.test)
  add_rostest(test/test-add-two-ints-server-groupname.test)
  add_rostest(test/test-service-callback.test)
  add_rostest(test/test-simple-client.test)
  add_rostest(test/test-simple-client-wait.test)
  add_rostest(test/test-simple-client-500.test)
  add_rostest(test/test-simple-client-wait-500.test)
  add_rostest(test/test-simple-client-cancel.test)
  add_rostest(test/test-unadvertise-service.test)
  add_rostest(test/test-actionlib.test)
  add_rostest(test/test-roseus.test)
  add_rostest(test/test-mark-lock.test)
  add_rostest(test/test-tf.test)
  add_rostest(test/test-disconnect.test)
  add_rostest(test/test-multi-queue.test)
  add_rostest(test/test-anymsg.test)
  add_rostest(test/test-namespace.test)
  add_rostest(test/test-nodename.test)
  add_rostest(test/test-parameter.test)
  add_rostest(test/test-anonymous.test)
  add_rostest(test/test-timer.test)
  # this seems to run other process and need pkill ... https://travis-ci.org/jsk-ros-pkg/jsk_roseus/jobs/233858193
  #add_rostest(test/test-genmsg.catkin.test)
  # this will not ok on running from run_tests, may be running test within launch file may not good.
  #add_rostest(test/test-genmsg-oneworkspace.catkin.launch) # use launch not to run on travis/catkin
  add_rostest(test/test-geneus.test)
  add_rostest(test/test-compile-message.test)
  add_rostest(test/test-print-ros-msg.test)
endif()
endif() # if($ENV{ROS_VERSION} EQUAL 2)

# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(roseus)

# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roslang roscpp rospack actionlib actionlib_msgs visualization_msgs tf geometry_msgs std_msgs std_srvs sensor_msgs visualization_msgs)

add_definitions(-Wall)

#set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )
#if(UNIX AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
#  set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR} CACHE PATH "roseus install prefix" FORCE )
#endif()


find_program (SVNVERSION_CMD svnversion)
execute_process (COMMAND "${SVNVERSION_CMD}" ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE SVNVERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE)
message (STATUS "Build svn revision: ${SVNVERSION}")

#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()

execute_process(COMMAND rospack find euslisp OUTPUT_VARIABLE euslisp_PACKAGE_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
message("-- Set euslisp_PACKAGE_PATH to ${euslisp_PACKAGE_PATH}")
if(EXISTS ${euslisp_PACKAGE_PATH}/jskeus/eus)
  set(EUSDIR ${euslisp_PACKAGE_PATH}/jskeus/eus)
else()
  set(EUSDIR ${euslisp_PACKAGE_PATH})
endif()
message("-- Set EUSDIR to ${EUSDIR}")
include_directories(/usr/include /usr/X11R6/include ${EUSDIR}/include)
add_library(roseus roseus.cpp)
add_library(eustf eustf.cpp)
add_library(roseus_c_util roseus_c_util.c)
target_link_libraries(roseus ${rospack_LIBRARIES} ${roscpp_LIBRARIES})
target_link_libraries(eustf  ${roscpp_LIBRARIES})

# compile flags
add_definitions(-O2 -Wno-write-strings -Wno-comment)
add_definitions(-Di486 -DLinux -D_REENTRANT -DVERSION='\"${8.26}\"' -DTHREADED -DPTHREAD -DX11R6_1)
add_definitions('-DSVNVERSION="\\"r${SVNVERSION}\\""')
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

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/euslisp)

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
# rosbuild_add_rostest(test/test-talker-listener.launch)
# rosbuild_add_rostest(test/test-add-two-ints.launch)
# rosbuild_add_rostest(test/test-simple-client.launch)
# rosbuild_add_rostest(test/test-simple-client-wait.launch)
# rosbuild_add_rostest(test/test-fibonacci.launch)
# rosbuild_add_rostest(test/test-tf.launch)
# rosbuild_add_rostest(test/test-disconnect.launch)
# rosbuild_add_rostest(test/test-multi-queue.launch)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    DEPENDS roslang roscpp rospack euslisp actionlib actionlib_msgs visualization_msgs bullet tf geometry_msgs std_msgs std_srvs sensor_msgs visualization_msgs actionlib_tutorials coreutils
    CATKIN-DEPENDS euslisp # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(PROGRAMS bin/roseus  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# set symlink from /opt/groovy/bin to /opt/groovy/share/roseus/roseus
install(CODE "
  file(MAKE_DIRECTORY \"\$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/bin\")
  message(\"-- CreateLink: \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/bin/roseus -> \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/roseus\")
  execute_process(COMMAND \"${CMAKE_COMMAND}\" -E create_symlink \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/roseus \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/bin/roseus)
")

install(TARGETS roseus  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/euslisp)
install(TARGETS eustf   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/euslisp)
install(TARGETS roseus_c_util  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/euslisp)
install(DIRECTORY euslisp/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/euslisp
  FILES_MATCHING PATTERN "*.l" PATTERN ".svn" EXCLUDE)

# scripts
install(DIRECTORY cmake/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
  FILES_MATCHING PATTERN "*.cmake" PATTERN ".svn" EXCLUDE)
file(GLOB scripts "${PROJECT_SOURCE_DIR}/scripts/*")
list(REMOVE_ITEM scripts "${PROJECT_SOURCE_DIR}/scripts/.svn")
install(PROGRAMS ${scripts} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

# test codes
install(TARGETS simple_execute_ref_server  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test)
install(DIRECTORY test/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
  FILES_MATCHING PATTERN "*.l" PATTERN "*.launch" PATTERN ".svn" EXCLUDE)

# dummy test target
add_custom_target(test COMMAND echo "DUMMY test target")
add_custom_target(test-results COMMAND echo "DUMMY test-results target")

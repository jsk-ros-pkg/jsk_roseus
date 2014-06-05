cmake_minimum_required(VERSION 2.8.3)
project(roseus_msgs)

find_package(catkin REQUIRED COMPONENTS euslisp) # load catkin
catkin_package() # share

set(_roseus_INSTALL_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/roseus/ros)
if(NOT euslisp_SOURCE_DIR)
  message("[roseus_msgs] generate messages for install")
  set(roseus_msgs_generate TRUE)
endif()

if (${roseus_msgs_generate})
  set(roseus_INSTALL_DIR ${_roseus_INSTALL_DIR})

  find_package(geneus) # load roseus.cmake

  set(ALL_GEN_OUTPUT_FILES_eus)

  foreach(path ${CMAKE_PREFIX_PATH})
    file(GLOB pkgs RELATIVE ${path}/share/ ${path}/share/*)
    foreach(pkg ${pkgs})
      message("[roseus_msgs] compile messages ${pkg}")
      find_package(${pkg} QUIET)
      _generate_eus_dep_msgs(${pkg})
    endforeach()
  endforeach()

  add_custom_target(roseus_msgs_generate_all_msgs ALL DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}) # generate all

  install(DIRECTORY ${roseus_INSTALL_DIR}/
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/roseus/ros
    )

  set(roseus_INSTALL_DIR) # unset

endif()




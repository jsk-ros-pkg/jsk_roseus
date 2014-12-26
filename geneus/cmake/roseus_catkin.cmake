# -*- indent-tabs-mode: nil -*-
# is called for evary packages
find_package(euslisp QUIET)
find_package(geneus QUIET)

if(EXISTS ${geneus_SOURCE_PREFIX}/scripts/genmsg_eus)
  set(geneus_PACKAGE_PATH ${geneus_SOURCE_PREFIX})
elseif(EXISTS ${geneus_SOURCE_DIR}/scripts/genmsg_eus)
  set(geneus_PACKAGE_PATH ${geneus_SOURCE_DIR})
else()
  set(geneus_PACKAGE_PATH ${geneus_PREFIX}/share/geneus)
endif()
set(GENMSG_EUS ${geneus_PACKAGE_PATH}/scripts/genmsg_eus)
set(GENSRV_EUS ${geneus_PACKAGE_PATH}/scripts/gensrv_eus)
set(GENMANIFEST_EUS ${geneus_PACKAGE_PATH}/scripts/genmanifest_eus)
if(NOT TARGET genmsg_eus)
  add_custom_target(genmsg_eus)
endif()
if(NOT TARGET gensrv_eus)
  add_custom_target(gensrv_eus)
endif()
if(NOT TARGET genmanifest_eus)
  add_custom_target(genmanifest_eus)
endif()

if(EXISTS ${euslisp_SOURCE_DIR}/jskeus)
  set(euslisp_PACKAGE_PATH ${euslisp_SOURCE_DIR})
else()
  set(euslisp_PACKAGE_PATH ${euslisp_PREFIX}/share/euslisp)
endif()
if(geneus_verbose)
  message("[roseus.camke] euslisp_PACKAGE_PATH = ${euslisp_PACKAGE_PATH}")
  message("[roseus.camke]  euseus_PACKAGE_PATH = ${geneus_PACKAGE_PATH}")
endif(geneus_verbose)
set(ROS_PACKAGE_PATH ${euslisp_PACKAGE_PATH}:${geneus_PACKAGE_PATH}:${CMAKE_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH})

macro(geneus_add_manifest arg_pkg)
  get_property(geneus_all_output_files GLOBAL PROPERTY geneus_all_output_files)
  if(NOT geneus_all_output_files)
    set(geneus_all_output_files ${ALL_GEN_OUTPUT_FILES_eus})
  endif()
  set(_output_file ${roseus_INSTALL_DIR}/${arg_pkg}/manifest.l)
  list(FIND geneus_all_output_files ${_output_file} _ret)
  if(${_ret} EQUAL -1)
    add_custom_command(OUTPUT ${_output_file}
      DEPENDS genmanifest_eus ${_depend_generate_py}
      COMMAND ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} ${GENMANIFEST_EUS} ${arg_pkg} ${_output_file} >/dev/null 2>&1
      COMMENT "Generating EusLisp manifest for upstream package ${arg_pkg}")
    list(APPEND ALL_GEN_OUTPUT_FILES_eus ${_output_file})
    list(APPEND geneus_all_output_files ${_output_file})
  endif()
  set_property(GLOBAL PROPERTY geneus_all_output_files ${geneus_all_output_files})
endmacro(geneus_add_manifest arg_pkg)

macro(geneus_add_msgs arg_pkg)
  set(input_msg_files)
  set(output_msg_files)
  # gen messages
  get_property(geneus_all_output_files GLOBAL PROPERTY geneus_all_output_files)
  if(NOT geneus_all_output_files)
    set(geneus_all_output_files ${ALL_GEN_OUTPUT_FILES_eus})
  endif()
  foreach(_msg_file ${${arg_pkg}_MESSAGE_FILES})
    set(_msg_file ${_${arg_pkg}_PACKAGE_PATH}/${_msg_file})
    get_filename_component(_msg_name ${_msg_file} NAME_WE)
    # check if 1. the file is already generated and 2. .l is newer than .msg file
    set(_output_msg ${roseus_INSTALL_DIR}/${arg_pkg}/msg/${_msg_name}.l)
    if(NOT EXISTS ${_output_msg} #not generated yet..?
        OR ${_msg_file} IS_NEWER_THAN ${_output_msg})
      #list(FIND ALL_GEN_OUTPUT_FILES_eus ${_output_msg} _ret)
      list(FIND geneus_all_output_files ${_output_msg} _ret)      
      if(${_ret} EQUAL -1 )
        list(APPEND input_msg_files ${_msg_file})
        list(APPEND output_msg_files ${_output_msg})
        list(APPEND ALL_GEN_OUTPUT_FILES_eus ${_output_msg})
        list(APPEND geneus_all_output_files ${_output_msg})
      endif()
    endif()
  endforeach()
  set_property(GLOBAL PROPERTY geneus_all_output_files ${geneus_all_output_files})
  if(output_msg_files)
    add_custom_command(OUTPUT ${output_msg_files}
      DEPENDS genmsg_eus ${_msg_file} ${_depend_generate_py}
      COMMAND ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} PYTHONPATH=${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}:$ENV{PYTHONPATH} ${GENMSG_EUS} ${input_msg_files} ":OUTPUT" ${output_msg_files} >/dev/null 2>&1
      COMMENT "Generating EusLisp message for upstream message ${input_msg_files}")
  endif(output_msg_files)
endmacro(geneus_add_msgs arg_pkg)

macro(geneus_add_srvs arg_pkg)
  set(input_srv_files)
  set(output_srv_files)
  get_property(geneus_all_output_files GLOBAL PROPERTY geneus_all_output_files)
  if(NOT geneus_all_output_files)
    set(geneus_all_output_files ${ALL_GEN_OUTPUT_FILES_eus})
  endif()
  foreach (_srv_file ${${arg_pkg}_SERVICE_FILES})
    set(_srv_file ${_${arg_pkg}_PACKAGE_PATH}/${_srv_file})
    get_filename_component(_srv_name ${_srv_file} NAME_WE)
    # check if 1. the file is already generated and 2. .l is newer than .msg file
    set(_output_srv ${roseus_INSTALL_DIR}/${arg_pkg}/srv/${_srv_name}.l)
    if(NOT EXISTS ${_output_srv} #not generated yet..?
        OR ${_srv_file} IS_NEWER_THAN ${_output_srv})
      list(FIND geneus_all_output_files ${_output_srv} _ret)
      if(${_ret} EQUAL -1)
        list(APPEND input_srv_files ${_srv_file})
        list(APPEND output_srv_files ${_output_srv})
        list(APPEND ALL_GEN_OUTPUT_FILES_eus ${_output_srv})
        list(APPEND geneus_all_output_files ${_output_srv})
      endif()
    endif()
  endforeach()
  set_property(GLOBAL PROPERTY geneus_all_output_files ${geneus_all_output_files})
  if(output_srv_files)
    add_custom_command(OUTPUT ${output_srv_files}
      DEPENDS gensrv_eus ${_srv_file} ${_depend_generate_py}
      COMMAND ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} ${GENSRV_EUS} ${input_srv_files} ":OUTPUT" ${output_srv_files} >/dev/null 2>&1
      COMMENT "Generating EusLisp service for upstream service ${arg_pkg}/srv/${_srv_name}")
  endif(output_srv_files)
endmacro(geneus_add_srvs arg_pkg)

macro(_generate_eus_dep_msgs arg_pkg)
  get_filename_component(pkg_full_path ${${arg_pkg}_DIR}/.. ABSOLUTE)

  file(GLOB ${arg_pkg}_MESSAGE_FILES "${pkg_full_path}/msg/*.msg")
  file(GLOB ${arg_pkg}_SERVICE_FILES "${pkg_full_path}/srv/*.srv")

  if(${arg_pkg}_SOURCE_DIR AND (${arg_pkg}_MESSAGE_FILES OR ${arg_pkg}_SERVICE_FILES) )# check if we need to compile python message. generating eusmessage depends on python message to get to  know md5sum
    set(_depend_generate_py ${arg_pkg}_generate_messages_py)
  else()
    set(_depend_generate_py )
  endif()
  
  geneus_add_manifest(${arg_pkg})
  geneus_add_msgs(${arg_pkg})
  geneus_add_srvs(${arg_pkg})
endmacro()

# define macros
macro(_generate_msg_srv_eus MSG_OR_SRV ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  # message("_generate_${MSG_OR_SRV}_eus\n ARG_PKG:${ARG_PKG}\n ARG_MSG:${ARG_MSG}\n ARG_IFLAGS:${ARG_IFLAGS}\n ARG_MSG_DEPS:${ARG_MSG_DEPS}\n ARG_GEN_OUTPUT_DIR:${ARG_GEN_OUTPUT_DIR}/msg")

  get_filename_component(MSG_NAME ${ARG_MSG} NAME)
  get_filename_component(MSG_SHORT_NAME ${ARG_MSG} NAME_WE)
  get_property(geneus_all_output_files GLOBAL PROPERTY geneus_all_output_files)
  if(NOT geneus_all_output_files)
    set(geneus_all_output_files ${ALL_GEN_OUTPUT_FILES_eus})
  endif()
  set(_depend_packages "")
  foreach(_msg ${ARG_MSG_DEPS})
    get_filename_component(_msg ${_msg} ABSOLUTE)
    string(REGEX REPLACE ".*/([^/]*)/${MSG_OR_SRV}/[^/]*$" "\\1" _pkg ${_msg})
    if (NOT ${${_pkg}_FOUND})
      find_package(${_pkg} QUIET)
    endif()
    if(${_pkg}_SOURCE_DIR) # source packages
      set(_depend_packages "${_depend_packages}:${${_pkg}_SOURCE_DIR}")
    else()                    # installed packages
      _generate_eus_dep_msgs(${_pkg} ${_msg})
    endif()
  endforeach()

  set(MSG_GENERATED_NAME ${MSG_SHORT_NAME})
  set(GEN_OUTPUT_FILE ${roseus_INSTALL_DIR}/${ARG_PKG}/${MSG_OR_SRV}/${MSG_GENERATED_NAME}.l)

  list(FIND geneus_all_output_files ${GEN_OUTPUT_FILE} _ret)
  if(${_ret} EQUAL -1)

    set(ROS_PACKAGE_PATH $ENV{ROS_PACKAGE_PATH})
    foreach(_space ${CATKIN_ORDERED_SPACES})
      set(ROS_PACKAGE_PATH ${_space}:${ROS_PACKAGE_PATH})
    endforeach()
    set(ROS_PACKAGE_PATH ${euslisp_PACKAGE_PATH}:${geneus_PACKAGE_PATH}:${ROS_PACKAGE_PATH})

    # compile msg
    string(TOUPPER GEN${MSG_OR_SRV}_EUS gen_msg_srv_eus)
    add_custom_command(OUTPUT ${GEN_OUTPUT_FILE}
      DEPENDS gen${MSG_OR_SRV}_eus ${ARG_MSG} ${ARG_MSG_DEPS} ${ARG_PKG}_generate_messages_py
      COMMAND ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} PYTHONPATH=${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}:$ENV{PYTHONPATH} ${${gen_msg_srv_eus}} ${ARG_MSG} ${GEN_OUTPUT_FILE} >/dev/null 2>&1
      COMMENT "Generating EusLisp file code from ${ARG_PKG}/${MSG_NAME}")

    list(APPEND ALL_GEN_OUTPUT_FILES_eus ${GEN_OUTPUT_FILE})
    list(APPEND ALL_GEN_OUTPUT_PACKAGES_eus ${GEN_OUTPUT_FILE})
    list(APPEND geneus_all_output_files ${GEN_OUTPUT_FILE})
  endif()
  set_property(GLOBAL PROPERTY geneus_all_output_files ${geneus_all_output_files})
endmacro()

macro(_generate_msg_eus ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  #_generate_msg_srv_eus("msg" "${ARG_PKG}" "${ARG_MSG}" "${ARG_IFLAGS}" "${ARG_MSG_DEPS}" "${ARG_GEN_OUTPUT_DIR}")
  _generate_msg_srv_eus("msg" "${ARG_PKG}" "${ARG_MSG}" NONE "${ARG_MSG_DEPS}" "${ARG_GEN_OUTPUT_DIR}")
endmacro()

macro(_generate_srv_eus ARG_PKG ARG_SRV ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  #_generate_msg_srv_eus("srv" "${ARG_PKG}" "${ARG_SRV}" "${ARG_IFLAGS}" "${ARG_MSG_DEPS}" "${ARG_GEN_OUTPUT_DIR}")
  _generate_msg_srv_eus("srv" "${ARG_PKG}" "${ARG_SRV}" NONE "${ARG_MSG_DEPS}" "${ARG_GEN_OUTPUT_DIR}")
endmacro()

macro(_generate_module_eus ARG_PKG ARG_GEN_OUTPUT_DIR ARG_GENERATED_FILES)
  # message("_generate_module_eus\n ARG_PKG:${ARG_PKG}\n ARG_GEN_OUTPUT_DIR:${ARG_GEN_OUTPUT_DIR}\n ARG_GENERATED_FILES:${ARG_GENERATED_FILES}\n")
  set(GEN_OUTPUT_FILE ${roseus_INSTALL_DIR}/${ARG_PKG}/manifest.l)
  set(ROS_PACKAGE_PATH ${euslisp_PACKAGE_PATH}:${geneus_PACKAGE_PATH}:${PROJECT_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH})
  geneus_add_manifest(${ARG_PKG})
endmacro()

# generate upsteram message/services
foreach(pkg ${catkin_FIND_COMPONENTS})
  if(${pkg}_SOURCE_DIR OR ${pkg}_SOURCE_PREFIX)
    if(geneus_verbose)
      message("[roseus.cmake] compile upstream package ${pkg}")
    endif(geneus_verbose)
    _generate_eus_dep_msgs(${pkg})
  endif()
endforeach()

# generate current package message/services
_generate_eus_dep_msgs(${PROJECT_NAME})

# generate message/services for installed packages
if(geneus_verbose)
  message("[roseus.cmake] compile installed package for ${PROJECT_NAME}")
endif(geneus_verbose)
set(_ROS_PACKAGE_PATH $ENV{ROS_PACKAGE_PATH})
set(ENV{ROS_PACKAGE_PATH} ${CMAKE_SOURCE_DIR}:${_ROS_PACKAGE_PATH})
#set(_rospack_depends "import rospkg; rp = rospkg.RosPack(); print ';'.join(rp.get_depends('${PROJECT_NAME}'))")
execute_process(COMMAND rospack depends ${PROJECT_NAME}#python -c "${_rospack_depends}"
  OUTPUT_VARIABLE _depend_output
  RESULT_VARIABLE _depend_failed
  OUTPUT_STRIP_TRAILING_WHITESPACE)
set(ENV{ROS_PACKAGE_PATH} ${_ROS_PACKAGE_PATH})
if(_depend_failed)
  message("[roseus.cmake] find rospack dpends fails for ${PROJECT_NAME}")
  return()
endif(_depend_failed)
if (NOT "${_depend_output}" STREQUAL "")
  string(REGEX REPLACE "\n" ";" _depend_output ${_depend_output})
endif()
foreach(_pkg ${_depend_output})
  if(NOT ${_pkg}_GENERATE_EUS_DEP_MSGS)
    set(need_compile TRUE)
    if(NOT ${_pkg}_FOUND)
      # workaround only for for groovy
      if($ENV{ROS_DISTRO} STREQUAL "groovy")
        set(_catkin_LIBRARIES ${catkin_LIBRARIES})
        set(_catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
      endif()
      find_package(${_pkg} QUIET)
      if($ENV{ROS_DISTRO} STREQUAL "groovy")
        set(catkin_LIBRARIES ${_catkin_LIBRARIES})
        set(catkin_INCLUDE_DIRS ${_catkin_INCLUDE_DIRS})
      endif()
    endif()
    if(NOT ${_pkg}_FOUND)
      set(need_compile FALSE)
    endif()
    string(REPLACE ":" ";" _cmake_prefix_path $ENV{CMAKE_PREFIX_PATH})
    foreach(_path ${_cmake_prefix_path})
      if(EXISTS ${_path}/share/roseus/ros/${_pkg})
        set(need_compile FALSE)
      endif()
    endforeach()
    if(need_compile)
      if(geneus_verbose)
        message("[roseus.cmake] compile installed package ${_pkg}")
      endif(geneus_verbose)
      _generate_eus_dep_msgs(${_pkg})
      set(${_pkg}_GENERATE_EUS_DEP_MSGS TRUE)
    endif(need_compile)
  endif(NOT ${_pkg}_GENERATE_EUS_DEP_MSGS)
endforeach()

if(NOT TARGET ${PROJECT_NAME}_${component}_ALL_GEN_OUTPUT_FILES_eus)
  add_custom_target(${PROJECT_NAME}_${component}_ALL_GEN_OUTPUT_FILES_eus ALL DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}) # generate all
endif()



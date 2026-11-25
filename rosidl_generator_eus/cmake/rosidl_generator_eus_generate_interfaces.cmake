# Copyright (c) 2026, JSK Robotics Laboratory.
# All rights reserved.
#
# Software License Agreement (BSD License)
# Author: Yoshiki Obinata

find_package(rosidl_cmake REQUIRED)

set(_output_path
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_eus/${PROJECT_NAME}")

set(_generated_files "")
foreach(_abs_idl_file ${rosidl_generate_interfaces_ABS_IDL_FILES})
  get_filename_component(_parent_folder "${_abs_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_idl_name "${_abs_idl_file}" NAME_WE)
  list(APPEND _generated_files
    "${_output_path}/${_parent_folder}/${_idl_name}.l"
  )
endforeach()

set(_dependency_files "")
set(_dependencies "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  foreach(_idl_file ${${_pkg_name}_IDL_FILES})
    rosidl_find_package_idl(_abs_idl_file "${_pkg_name}" "${_idl_file}")
    list(APPEND _dependency_files "${_abs_idl_file}")
    list(APPEND _dependencies "${_pkg_name}:${_abs_idl_file}")
  endforeach()
endforeach()

set(target_dependencies
  "${rosidl_generator_eus_BIN}"
  ${rosidl_generator_eus_GENERATOR_FILES}
  ${rosidl_generate_interfaces_ABS_IDL_FILES}
  ${_dependency_files})
foreach(dep ${target_dependencies})
  if(NOT EXISTS "${dep}")
    message(FATAL_ERROR "Target dependency '${dep}' does not exist")
  endif()
endforeach()

set(generator_arguments_file
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_eus__arguments.json")
rosidl_write_generator_arguments(
  "${generator_arguments_file}"
  PACKAGE_NAME "${PROJECT_NAME}"
  IDL_TUPLES "${rosidl_generate_interfaces_IDL_TUPLES}"
  ROS_INTERFACE_DEPENDENCIES "${_dependencies}"
  OUTPUT_DIR "${_output_path}"
  TEMPLATE_DIR "${rosidl_generator_eus_TEMPLATE_DIR}"
  TARGET_DEPENDENCIES ${target_dependencies}
)

# By default, without the settings below, find_package(Python3) will attempt
# to find the newest python version it can, and additionally will find the
# most specific version.
cmake_minimum_required(VERSION 3.20)
cmake_policy(SET CMP0094 NEW)
set(Python3_FIND_UNVERSIONED_NAMES FIRST)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_custom_command(
  OUTPUT ${_generated_files}
  COMMAND Python3::Interpreter
  ARGS ${rosidl_generator_eus_BIN}
  --generator-arguments-file "${generator_arguments_file}"
  DEPENDS ${target_dependencies}
  COMMENT "Generating EusLisp code for ROS interfaces"
  VERBATIM
)

set(_target_suffix "__rosidl_generator_eus")

if(TARGET ${rosidl_generate_interfaces_TARGET}${_target_suffix})
  message(WARNING
    "Custom target ${rosidl_generate_interfaces_TARGET}${_target_suffix}"
    " already exists")
else()
  add_custom_target(
    ${rosidl_generate_interfaces_TARGET}${_target_suffix}
    DEPENDS ${_generated_files}
  )
endif()

# Make top-level generation target depend on this
add_dependencies(
  ${rosidl_generate_interfaces_TARGET}
  ${rosidl_generate_interfaces_TARGET}${_target_suffix}
)

if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
  install(
    DIRECTORY ${_output_path}/
    DESTINATION "share/roseus/ros/${PROJECT_NAME}/"
    PATTERN "*.l"
  )
endif()

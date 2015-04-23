if(rosbuild_find_ros_package)
  rosbuild_find_ros_package(geneus)
  if(NOT TARGET ROSBUILD_genmanifest_roseus_${PROJECT_NAME})
    include(${geneus_PACKAGE_PATH}/cmake/roseus.cmake)
  endif()
endif()

###
# recursively find message dependencies for include path
macro(find_all_msg_dependences _pkg _msg_path)
  if(EXISTS ${${_pkg}_PREFIX}/share/${_pkg}/cmake/${_pkg}-msg-paths.cmake)
    include(${${_pkg}_PREFIX}/share/${_pkg}/cmake/${_pkg}-msg-paths.cmake)
    foreach(_msg ${${_pkg}_MSG_DEPENDENCIES})
      if(NOT ${_msg}_PREFIX)
	find_package(${_msg} REQUIRED)
      endif()
      if(EXISTS ${${_msg}_PREFIX}/share/${_msg}/msg)
        list(APPEND _msg_path "-I${_msg}:${${_msg}_PREFIX}/share/${_msg}/msg")
      elseif(EXISTS ${${_msg}_SOURCE_PREFIX}/msg)
        list(APPEND _msg_path "-I${_msg}:${${_msg}_SOURCE_PREFIX}/msg")
      else()
        message(WARNING "path to ${_msg} is not found. ${${_msg}_PREFIX}/share/${_msg}/msg nor ${${_msg}_SOURCE_PREFIX}/msg not exists")
      endif()
      find_all_msg_dependences(${_msg} _msg_path)
    endforeach()
  endif()
endmacro()

# generate all roseus messages
string(ASCII 27 Esc)
macro(generate_all_roseus_messages)
  set(target_pkg ${PROJECT_NAME})
  set(_${target_pkg}_generate_roseus_message_packages)
  set(_${target_pkg}_generate_roseus_message_packages_all)
  # find package that
  # 1) has <package>_(MESSAGE|SERVICE)_FILES (it have message/service files)
  # 2) does not have <package>_SOURCE_PREFIX (it is source compiled so geneus will works for this packages)
  # 3) does not have <package>_generate_messages_eus (it is not create target before)
  string(REGEX MATCH "catkin" need_catkin "$ENV{_}")
  if(need_catkin OR ${target_pkg} STREQUAL "roseus") # do not run upstream message generation on buildfirm
    get_cmake_property(_variables VARIABLES)
    foreach(_variable ${_variables})
      string(REGEX REPLACE "^(.*)_(MESSAGE|SERVICE)_FILES$" "\\1" _pkg ${_variable})
      if("${_pkg}_FOUND" AND NOT ${_pkg}_SOURCE_PREFIX AND NOT TARGET ${_pkg}_generate_messages_eus)
        _list_append_unique(_${target_pkg}_generate_roseus_message_packages_all ${_pkg})
        if(NOT EXISTS ${roseus_PREFIX}/share/roseus/ros/${_pkg})
          _list_append_unique(_${target_pkg}_generate_roseus_message_packages ${_pkg})
        endif()
      endif()
    endforeach()
  endif()
  ## add target package those who does not have msg files
  if(NOT TARGET ${target_pkg}_generate_messages_eus AND
      NOT ${target_pkg}_MESSAGE_FILES AND NOT ${target_pkg}_SERVICE_FILES AND
      NOT EXISTS ${PROJECT_SOURCE_DIR}/msg AND NOT EXISTS ${PROJECT_SOURCE_DIR}/srv AND
      NOT EXISTS ${roseus_PREFIX}/share/roseus/ros/${target_pkg})
    _list_append_unique(_${target_pkg}_generate_roseus_message_packages ${target_pkg})
  endif()
  message(STATUS "[roseus.cmake] ${target_pkg} depends on ${_${target_pkg}_generate_roseus_message_packages_all}")
  message(STATUS "[roseus.cmake] ${target_pkg} will compile ${_${target_pkg}_generate_roseus_message_packages}")

  # run _generate_{msg,srv,module}_eus, see geneus package
  set(ALL_GEN_OUTPUT_FILES_eus)
  foreach(_pkg ${_${target_pkg}_generate_roseus_message_packages})
    set(_msg_prefix "${${_pkg}_PREFIX}/share/${_pkg}/")
    set(_msg_path "-I${_pkg}:${${_pkg}_PREFIX}/share/${_pkg}/msg")
    find_all_msg_dependences(${_pkg} _msg_path)
    foreach(_msg ${${_pkg}_MESSAGE_FILES})
      _generate_msg_eus(${_pkg}
	"${_msg_prefix}/${_msg}"
	"${_msg_path}"
	""
	${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/${_pkg}
	)
    endforeach()
    foreach(_srv ${${_pkg}_SERVICE_FILES})
      _generate_srv_eus(${_pkg}
	"${_msg_prefix}/${_srv}"
	"${_msg_path}"
	""
	${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/${_pkg}
	)
    endforeach()
    _generate_module_eus(${_pkg}
      ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/${_pkg}
      "${ALL_GEN_OUTPUT_FILES_eus}"
      )
    add_custom_target(${_pkg}_generate_messages_eus ALL
      DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
      )
    if(TARGET ${PROJECT_NAME}_generate_messages_eus)
      add_dependencies(${target_pkg}_generate_messages_eus ALL ${_pkg}_generate_messages_eus)
    endif()
    install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/${_pkg}/ DESTINATION ${CMAKE_INSTALL_PREFIX}/share/roseus/ros/${_pkg}/)
  endforeach()
endmacro()


# run generate_all_roseus_messages() if this is invoked from catkin_* command and genmsg does not depends on geneus
find_program(_pkg_config_executable NAMES pkg-config DOC "pkg-config executable")
if (_pkg_config_executable)
  execute_process(COMMAND ${_pkg_config_executable} --print-requires message_generation
    OUTPUT_VARIABLE message_generation_requires
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  string(REPLACE "\n" ";" message_generation_requires ${message_generation_requires})
endif ()
message(STATUS "[roseus.cmake] message_generation Requires: ${message_generation_requires}")
if(NOT "${message_generation_requires}" MATCHES "geneus")
  message(STATUS "[roseus.cmake] generate all roseus messages")
  generate_all_roseus_messages()
endif()

# utility to make doc
macro(generate_eusdoc _lispfile)
  get_filename_component(_name ${_lispfile} NAME_WE)
  set(_lisppath "${CMAKE_CURRENT_SOURCE_DIR}/${_lispfile}")
  set(_mdfile "${_name}.md")
  set(_generate_eusdoc_command "\\\"(setq lisp::*error-handler* 'exit)\\\" \\\"(load \\\\\\\"${_lisppath}\\\\\\\")\\\" \\\"(make-document \\\\\\\"${_lisppath}\\\\\\\" \\\\\\\"${_mdfile}\\\\\\\")\\\" \\\"(exit)\\\" ")
  separate_arguments(_generate_eusdoc_command_list WINDOWS_COMMAND "${_generate_eusdoc_command}")
  #set(_roseus_exe roseus)
  find_program(_roseus_exe roseus)
  set(_ROS_PACKAGE_PATH $ENV{ROS_PACKAGE_PATH})
  string(REPLACE ";" ":" _CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}")
  if(${PROJECT_NAME} STREQUAL "roseus") # this is only for roseus package
    set(_ROS_PACKAGE_PATH ${PROJECT_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH})
    set(_CMAKE_PREFIX_PATH ${CATKIN_DEVEL_PREFIX}:${_CMAKE_PREFIX_PATH})
    set(_roseus_exe ${PROJECT_SOURCE_DIR}/bin/roseus)
  endif()
  add_custom_command(OUTPUT ${_mdfile}
    COMMAND ROS_PACKAGE_PATH=${_ROS_PACKAGE_PATH} CMAKE_PREFIX_PATH=${_CMAKE_PREFIX_PATH} ${_roseus_exe} $ENV{EUSDIR}/lib/llib/documentation.l ${_generate_eusdoc_command_list}
    DEPENDS ${_lispfile})
  add_custom_target(${PROJECT_NAME}_${_name}_generate_eusdoc ALL DEPENDS ${_mdfile} install_roseus)
  if(TARGET ${PROJECT_NAME}_generate_messages_eus)
    add_dependencies(${PROJECT_NAME}_${_name}_generate_eusdoc ${PROJECT_NAME}_generate_messages_eus)
  endif()
endmacro()

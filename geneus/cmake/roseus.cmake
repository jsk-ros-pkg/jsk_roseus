message("[roseus.camke] Loading... PROJECT_NAME=${PROJECT_NAME} USE_ROSBILD=${USE_ROSBUILD}")

# get roseus script file, all genmsg depend on this
if(NOT roseus_INSTALL_DIR)
  set(roshomedir $ENV{ROS_HOME})
  if("" STREQUAL "${roshomedir}")
    set(roshomedir "$ENV{HOME}/.ros")
  endif("" STREQUAL "${roshomedir}")
  set(roseus_INSTALL_DIR ${roshomedir}/roseus/$ENV{ROS_DISTRO})
endif()

if(NOT COMMAND rosbuild_find_ros_package) ## catkin
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

  message("[roseus.camke] euslisp_PACKAGE_PATH = ${euslisp_PACKAGE_PATH}")
  message("[roseus.camke]  euseus_PACKAGE_PATH = ${geneus_PACKAGE_PATH}")
  set(ROS_PACKAGE_PATH ${euslisp_PACKAGE_PATH}:${geneus_PACKAGE_PATH}:${CMAKE_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH})

  macro(_generate_eus_dep_msgs arg_pkg)
    get_filename_component(pkg_full_path ${${arg_pkg}_DIR}/.. ABSOLUTE)

    file(GLOB ${arg_pkg}_MESSAGE_FILES "${pkg_full_path}/msg/*.msg")
    file(GLOB ${arg_pkg}_SERVICE_FILES "${pkg_full_path}/srv/*.srv")

     if(${arg_pkg}_SOURCE_DIR AND (${arg_pkg}_MESSAGE_FILES OR ${arg_pkg}_SERVICE_FILES) )# check if we need to compile python message. generating eusmessage depends on python message to get to  know md5sum
      set(_depend_generate_py ${arg_pkg}_generate_messages_py)
    else()
      set(_depend_generate_py )
    endif()
    # gen manifest
    list(FIND ALL_GEN_OUTPUT_FILES_eus ${roseus_INSTALL_DIR}/${arg_pkg}/manifest.l _ret)
    if(${_ret} EQUAL -1)
      add_custom_command(OUTPUT ${roseus_INSTALL_DIR}/${arg_pkg}/manifest.l
        DEPENDS genmanifest_eus ${_depend_generate_py}
        COMMAND ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} ${GENMANIFEST_EUS} ${arg_pkg} ${roseus_INSTALL_DIR}/${arg_pkg}/manifest.l
        COMMENT "Generating EusLisp code for upstream package ${arg_pkg}")
      list(APPEND ALL_GEN_OUTPUT_FILES_eus ${roseus_INSTALL_DIR}/${arg_pkg}/manifest.l)
    endif()

    # gen messages
    foreach(_msg_file ${${arg_pkg}_MESSAGE_FILES})
      set(_msg_file ${_${arg_pkg}_PACKAGE_PATH}/${_msg_file})
      get_filename_component(_msg_name ${_msg_file} NAME_WE)
      list(FIND ALL_GEN_OUTPUT_FILES_eus ${roseus_INSTALL_DIR}/${arg_pkg}/msg/${_msg_name}.l _ret)
      if(${_ret} EQUAL -1)
        add_custom_command(OUTPUT ${roseus_INSTALL_DIR}/${arg_pkg}/msg/${_msg_name}.l
          DEPENDS genmsg_eus ${_msg_file} ${_depend_generate_py}
          COMMAND ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} PYTHONPATH=${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}:$ENV{PYTHONPATH} ${GENMSG_EUS} ${_msg_file} ${roseus_INSTALL_DIR}/${arg_pkg}/msg/${_msg_name}.l
          COMMENT "Generating EusLisp code for upstream message ${arg_pkg}/msg/${_msg_name}")
        list(APPEND ALL_GEN_OUTPUT_FILES_eus ${roseus_INSTALL_DIR}/${arg_pkg}/msg/${_msg_name}.l)
      endif()
    endforeach()

    # gen service
    foreach (_srv_file ${${arg_pkg}_SERVICE_FILES})
      set(_srv_file ${_${arg_pkg}_PACKAGE_PATH}/${_srv_file})
      get_filename_component(_srv_name ${_srv_file} NAME_WE)
      list(FIND ALL_GEN_OUTPUT_FILES_eus ${roseus_INSTALL_DIR}/${arg_pkg}/srv/${_srv_name}.l _ret)
      if(${_ret} EQUAL -1)
        add_custom_command(OUTPUT ${roseus_INSTALL_DIR}/${arg_pkg}/srv/${_srv_name}.l
          DEPENDS gensrv_eus ${_srv_file} ${_depend_generate_py}
          COMMAND ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} ${GENSRV_EUS} ${_srv_file} ${roseus_INSTALL_DIR}/${arg_pkg}/srv/${_srv_name}.l
          COMMENT "Generating EusLisp code for upstream service ${arg_pkg}/srv/${_srv_name}")
        list(APPEND ALL_GEN_OUTPUT_FILES_eus ${roseus_INSTALL_DIR}/${arg_pkg}/srv/${_srv_name}.l)
      endif()
    endforeach()

  endmacro()

  # define macros
  macro(_generate_msg_srv_eus MSG_OR_SRV ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
    # message("_generate_${MSG_OR_SRV}_eus\n ARG_PKG:${ARG_PKG}\n ARG_MSG:${ARG_MSG}\n ARG_IFLAGS:${ARG_IFLAGS}\n ARG_MSG_DEPS:${ARG_MSG_DEPS}\n ARG_GEN_OUTPUT_DIR:${ARG_GEN_OUTPUT_DIR}/msg")

    get_filename_component(MSG_NAME ${ARG_MSG} NAME)
    get_filename_component(MSG_SHORT_NAME ${ARG_MSG} NAME_WE)

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

    list(FIND ALL_GEN_OUTPUT_FILES_eus ${GEN_OUTPUT_FILE} _ret)
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
        COMMAND ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} PYTHONPATH=${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}:$ENV{PYTHONPATH} ${${gen_msg_srv_eus}} ${ARG_MSG} ${GEN_OUTPUT_FILE}
        COMMENT "Generating EusLisp message code from ${ARG_PKG}/${MSG_NAME}")

      list(APPEND ALL_GEN_OUTPUT_FILES_eus ${GEN_OUTPUT_FILE})
      list(APPEND ALL_GEN_OUTPUT_PACKAGES_eus ${GEN_OUTPUT_FILE})

    endif()

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

    list(FIND ALL_GEN_OUTPUT_FILES_eus ${GEN_OUTPUT_FILE} _ret)
    if(${_ret} EQUAL -1)

      set(ROS_PACKAGE_PATH ${euslisp_PACKAGE_PATH}:${geneus_PACKAGE_PATH}:${PROJECT_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH})
      add_custom_command(OUTPUT ${GEN_OUTPUT_FILE}
        DEPENDS genmanifest_eus ${ARG_GENERATED_FILES} ${ARG_PKG}_generate_messages_py
        COMMAND ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} ${GENMANIFEST_EUS}  ${ARG_PKG} ${GEN_OUTPUT_FILE}
        COMMENT "Generating EusLisp module code from ${ARG_PKG}")

      list(APPEND ALL_GEN_OUTPUT_FILES_eus ${GEN_OUTPUT_FILE})

    endif()

  endmacro()

  # generate upsteram message/services
  foreach(pkg ${catkin_FIND_COMPONENTS})
    if(${pkg}_SOURCE_DIR OR ${pkg}_SOURCE_PREFIX)
      message("[roseus.cmake] compile upstream package ${pkg}")
      _generate_eus_dep_msgs(${pkg})
    endif()
  endforeach()

  # generate current package message/services
  _generate_eus_dep_msgs(${PROJECT_NAME})

  # generate message/services for installed packages
  message("[roseus.cmake] compile installed package for ${PROJECT_NAME}")
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
      set(need_compile TRUE)
      string(REPLACE ":" ";" _cmake_prefix_path $ENV{CMAKE_PREFIX_PATH})
      foreach(_path ${_cmake_prefix_path})
        if(EXISTS ${_path}/share/roseus/ros/${_pkg})
	  set(need_compile FALSE)
        endif()
      endforeach()
      if(need_compile)
        message("[roseus.cmake] compile installed package ${_pkg}")
        _generate_eus_dep_msgs(${_pkg})
        set(${_pkg}_GENERATE_EUS_DEP_MSGS TRUE)
      endif(need_compile)
    endif(NOT ${_pkg}_GENERATE_EUS_DEP_MSGS)
  endforeach()

  if(NOT TARGET ${PROJECT_NAME}_${component}_ALL_GEN_OUTPUT_FILES_eus)
    add_custom_target(${PROJECT_NAME}_${component}_ALL_GEN_OUTPUT_FILES_eus ALL DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}) # generate all
  endif()

  return()
endif()

# for rosbuild
if(NOT ROSBUILD_init_called) # wait until rosbuild init called
  message("[roseus.camke] return since ROSBUILD_init_called is not set")
  return()
endif()

rosbuild_find_ros_package(genmsg_cpp)
rosbuild_find_ros_package(geneus)

## rule
##  1) generate msg/srv/manifest file, depending on manifest.xml, msg/ srv/.
##     this does not generate generated file under .ros/roseus/<distro>/<package>
##  2) check if depends packages need genmsg/gensrv...
##     use rospack depends to get list of depend packages
##      for each package...
##       if the package have ROS_NOBUILD fileor package does not have Makefile, then...
##         check generated file if we need to generate manifest,msg,srv
##       else (if the package does not hve ROS_NOBUILD file) do nothing
##

# for euslisp ros API. like roslib.load_mafest
macro(genmanifest_eus)
  execute_process(COMMAND find ${euslisp_PACKAGE_PATH} -name eus2 -executable
    OUTPUT_VARIABLE _eus2_output
    RESULT_VARIABLE _eus2_failed)
  if(_eus2_failed)
    message("[roseus.cmake] eus2 is not ready yet, so skip generating ${PROJECT_NAME}/manifeste.l")
    return()
  endif(_eus2_failed)

  set(genmanifest_eus_exe ${geneus_PACKAGE_PATH}/scripts/genmanifest_eus)
  set(manifest_eus_target_dir ${roseus_INSTALL_DIR}/${PROJECT_NAME})
  set(manifest_eus_target ${manifest_eus_target_dir}/manifest.l)
  if(EXISTS ${PROJECT_SOURCE_DIR}/package.xml)
    set(manifest_xml ${PROJECT_SOURCE_DIR}/package.xml)
  else()
    set(manifest_xml ${PROJECT_SOURCE_DIR}/manifest.xml)
  endif()
  message("[roseus.cmake] add custom target ROSBUILD_genmanifest_roseus_${PROJECT_NAME}")
  add_custom_command(OUTPUT ${manifest_eus_target}
    COMMAND ${genmanifest_eus_exe} ${PROJECT_NAME}
    DEPENDS ${manifest_xml} ${msggenerated})
  add_custom_target(ROSBUILD_genmanifest_roseus_${PROJECT_NAME} ALL
      DEPENDS ${manifest_eus_target})
endmacro(genmanifest_eus)
genmanifest_eus()

# Message-generation support.
macro(genmsg_eus)
  execute_process(COMMAND find ${euslisp_PACKAGE_PATH} -name eus2 -executable
    OUTPUT_VARIABLE _eus2_output
    RESULT_VARIABLE _eus2_failed)
  if(_eus2_failed)
    message("[roseus.cmake] eus2 is not ready yet, try rosmake euslisp")
    return()
  endif(_eus2_failed)

  set(_ROSBUILD_GENERATED_MSG_FILES_BAK ${_ROSBUILD_GENERATED_MSG_FILES})
  set(_ROSBUILD_GENERATED_MSG_FILES "")
  rosbuild_get_msgs(_msglist)
  set(_ROSBUILD_GENERATED_MSG_FILES ${_ROSBUILD_GENERATED_MSG_FILES_BAK})
  set(_autogen "")
  foreach(_msg ${_msglist})
    # Construct the path to the .msg file
    set(_input ${PROJECT_SOURCE_DIR}/msg/${_msg})
    rosbuild_gendeps(${PROJECT_NAME} ${_msg})
    set(genmsg_eus_exe ${geneus_PACKAGE_PATH}/scripts/genmsg_eus)

    set(_output_eus ${roseus_INSTALL_DIR}/${PROJECT_NAME}/msg/${_msg})
    string(REPLACE ".msg" ".l" _output_eus ${_output_eus})

    # Add the rule to build the .l the .msg
    add_custom_command(OUTPUT ${_output_eus} ${roseus_INSTALL_DIR}/${PROJECT_NAME}/msg
                       COMMAND ${genmsg_eus_exe} ${_input}
                       DEPENDS ${_input} ${gendeps_exe} ${${PROJECT_NAME}_${_msg}_GENDEPS} ${ROS_MANIFEST_LIST} ${msggenerated})
    list(APPEND _autogen ${_output_eus})
  endforeach(_msg)
  # Create a target that depends on the union of all the autogenerated
  # files
  message("[roseus.cmake] add custom target ROSBUILD_genmsg_roseus_${PROJECT_NAME}")
  add_custom_target(ROSBUILD_genmsg_roseus_${PROJECT_NAME} DEPENDS ${_autogen})
  # Add our target to the top-level genmsg target, which will be fired if
  # the user calls genmsg()
  add_dependencies(rospack_genmsg ROSBUILD_genmsg_roseus_${PROJECT_NAME})
endmacro(genmsg_eus)

# Call the macro we just defined.
genmsg_eus()

# Service-generation support.
macro(gensrv_eus)
  execute_process(COMMAND find ${euslisp_PACKAGE_PATH} -name eus2 -executable
    OUTPUT_VARIABLE _eus2_output
    RESULT_VARIABLE _eus2_failed)
  if(_eus2_failed)
    message("[roseus.cmake] eus2 is not ready yet, try rosmake euslisp")
    return()
  endif(_eus2_failed)

  rosbuild_get_srvs(_srvlist)
  set(_autogen "")
  foreach(_srv ${_srvlist})
    # Construct the path to the .srv file
    set(_input ${PROJECT_SOURCE_DIR}/srv/${_srv})

    rosbuild_gendeps(${PROJECT_NAME} ${_srv})
    set(gensrv_eus_exe ${geneus_PACKAGE_PATH}/scripts/gensrv_eus)

    set(_output_eus ${roseus_INSTALL_DIR}/${PROJECT_NAME}/srv/${_srv})
    string(REPLACE ".srv" ".l" _output_eus ${_output_eus})

    # Add the rule to build the .l from the .srv
    add_custom_command(OUTPUT ${_output_eus} ${roseus_INSTALL_DIR}/${PROJECT_NAME}/srv
                       COMMAND ${gensrv_eus_exe} ${_input}
                       DEPENDS ${_input} ${gendeps_exe} ${${PROJECT_NAME}_${_srv}_GENDEPS} ${ROS_MANIFEST_LIST} ${msggenerated})
    list(APPEND _autogen ${_output_eus})
  endforeach(_srv)
  # Create a target that depends on the union of all the autogenerated
  # files
  message("[roseus.cmake] add custom target ROSBUILD_gensrv_roseus_${PROJECT_NAME}")
  add_custom_target(ROSBUILD_gensrv_roseus_${PROJECT_NAME} DEPENDS ${_autogen})
  # Add our target to the top-level gensrv target, which will be fired if
  # the user calls gensrv()
  add_dependencies(rospack_gensrv ROSBUILD_gensrv_roseus_${PROJECT_NAME})
endmacro(gensrv_eus)

# Call the macro we just defined.
gensrv_eus()

# generate msg for package contains ROS_NOBUILD
macro(generate_ros_nobuild_eus)
  # if euslisp is not compiled, return from
  execute_process(COMMAND find ${euslisp_PACKAGE_PATH} -name eus2 -executable
    OUTPUT_VARIABLE _eus2_output
    RESULT_VARIABLE _eus2_failed)
  if(_eus2_failed)
    message("[roseus.cmake] eus2 is not ready yet, try rosmake euslisp")
    return()
  endif(_eus2_failed)

  # use rospack depends for packages needs to generate msg/srv
  execute_process(COMMAND rospack depends ${PROJECT_NAME} OUTPUT_VARIABLE depends_packages OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(depends_packages)
    string(REGEX REPLACE "\n" ";" depends_packages ${depends_packages})
  endif(depends_packages)
  set(_project ${PROJECT_NAME})
  list(APPEND depends_packages "${PROJECT_NAME}")
  list(LENGTH depends_packages depends_length)
  # get roseus/script files
  execute_process(COMMAND ${geneus_PACKAGE_PATH}/scripts/gengenerated_eus OUTPUT_VARIABLE md5sum_script)
  # for each packages...
  set(depends_counter 1)
  foreach(_package ${depends_packages})
    message("[roseus.cmake] [${depends_counter}/${depends_length}] Check ${_package} for ${PROJECT_NAME}")
    math(EXPR depends_counter "${depends_counter} + 1")
    # check if the package have ROS_NOBUILD
    rosbuild_find_ros_package(${_package})
    ### https://github.com/willowgarage/catkin/issues/122
    if(EXISTS ${${_package}_PACKAGE_PATH}/action AND
	(NOT EXISTS ${${_package}_PACKAGE_PATH}/msg) AND
        (NOT EXISTS ${roseus_INSTALL_DIR}/${_package}/action_msg/generated))
      message("[roseus.cmake] generate msg from action")
      file(GLOB _actions RELATIVE "${${_package}_PACKAGE_PATH}/action/" "${${_package}_PACKAGE_PATH}/action/*.action")
      foreach(_action ${_actions})
	message("[roseus.cmake] genaction.py ${_action} -o ${roseus_INSTALL_DIR}/${_package}/action_msg/")
	execute_process(COMMAND rosrun actionlib_msgs genaction.py ${${_package}_PACKAGE_PATH}/action/${_action} -o ${roseus_INSTALL_DIR}/${_package}/action_msg)
      endforeach()
      file(GLOB _action_msgs "${roseus_INSTALL_DIR}/${_package}/action_msg/*.msg")
      foreach(_action_msg ${_action_msgs})
	message("[roseus.cmake] rosrun roseus genmsg_eus ${_action_msg}")
	execute_process(COMMAND rosrun roseus genmsg_eus ${_action_msg})
      endforeach()
      file(WRITE ${roseus_INSTALL_DIR}/${_package}/action_msg/generated "generated")
    endif()
    ###
    set(msggenerated "${roseus_INSTALL_DIR}/${_package}/generated")
    set(md5sum_file "")
    if(EXISTS ${msggenerated})
      execute_process(COMMAND cat ${msggenerated} OUTPUT_VARIABLE md5sum_file)
    endif(EXISTS ${msggenerated})
    if((EXISTS ${${_package}_PACKAGE_PATH}/ROS_NOBUILD OR
	  (NOT EXISTS ${${_package}_PACKAGE_PATH}/Makefile))
	AND NOT "${md5sum_file}" STREQUAL "${md5sum_script}")
      message("[roseus.cmake] need to re-generate files, remove ${msggenerated}")
      file(REMOVE ${msggenerated})
      set(PROJECT_NAME ${_package})
      set(PROJECT_SOURCE_DIR ${${_package}_PACKAGE_PATH})
      genmanifest_eus()
      genmsg_eus()
      gensrv_eus()
      add_custom_target(ROSBUILD_gengenerated_roseus_${PROJECT_NAME} ALL DEPENDS ${msggenerated})
      add_dependencies(ROSBUILD_gengenerated_roseus_${PROJECT_NAME} ROSBUILD_genmanifest_roseus_${PROJECT_NAME})
      add_dependencies(ROSBUILD_gengenerated_roseus_${PROJECT_NAME} ROSBUILD_genmsg_roseus_${PROJECT_NAME})
      add_dependencies(ROSBUILD_gengenerated_roseus_${PROJECT_NAME} ROSBUILD_gensrv_roseus_${PROJECT_NAME})
      # write md5sum to generate if genmsg, gensrv are execute
      add_custom_command(OUTPUT ${msggenerated}
	COMMAND ${geneus_PACKAGE_PATH}/scripts/gengenerated_eus ${PROJECT_NAME} ${msggenerated})
      add_dependencies(rosbuild_precompile ROSBUILD_gengenerated_roseus_${PROJECT_NAME})
      set(PROJECT_NAME ${_project})
      set(PROJECT_SOURCE_DIR ${${_project}_PACKAGE_PATH})
    endif((EXISTS ${${_package}_PACKAGE_PATH}/ROS_NOBUILD OR
	(NOT EXISTS ${${_package}_PACKAGE_PATH}/Makefile))
      AND NOT "${md5sum_file}" STREQUAL "${md5sum_script}")
    # check the generated file
  endforeach(_package ${depends_packages})
endmacro(generate_ros_nobuild_eus)

# call the macro we just defined
generate_ros_nobuild_eus()


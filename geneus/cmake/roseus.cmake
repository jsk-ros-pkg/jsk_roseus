message("[roseus.camke] Loading... USE_ROSBILD=${USE_ROSBUILD}")

# get roseus script file, all genmsg depend on this
set(roshomedir $ENV{ROS_HOME})
if("" STREQUAL "${roshomedir}")
  set(roshomedir "$ENV{HOME}/.ros")
endif("" STREQUAL "${roshomedir}")

if(NOT COMMAND rosbuild_find_ros_package) ## catkin
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
  set(roseus_INSTALL_DIR ${roshomedir}/roseus/$ENV{ROS_DISTRO})
  set(ROS_PACKAGE_PATH ${euslisp_PACKAGE_PATH}:${geneus_PACKAGE_PATH}:${PROJECT_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH})

  macro(_generate_eus_dep_msgs arg_msg_deps)
    foreach(arg_msg_dep ${arg_msg_deps})
      get_filename_component(msg_full_path ${arg_msg_dep}   ABSOLUTE) # /../<pkg>/msg/A.msg
      get_filename_component(msg_name      ${msg_full_path} NAME_WE)  # msg
      get_filename_component(tmp           ${msg_full_path} PATH)     # /../<pkg>/msg
      get_filename_component(pkg_full_path ${tmp}           PATH)     # /../<pkg>
      get_filename_component(pkg_name      ${pkg_full_path} NAME)     # pkg
      set(ROS_PACKAGE_PATH ${pkg_full_path}:${ROS_PACKAGE_PATH})
      # compile manifest
      list(FIND ALL_GEN_OUTPUT_FILES_eus ${roseus_INSTALL_DIR}/${pkg_name}/manifest.l _ret)
      if(${_ret} EQUAL -1)
        add_custom_command(OUTPUT ${roseus_INSTALL_DIR}/${pkg_name}/manifest.l
          DEPENDS genmanifest_eus ${msg_full_path}
          COMMAND ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} ${GENMANIFEST_EUS}  ${pkg_name}
          COMMENT "Generating EusLisp code from ${pkg_name}")
        list(APPEND ALL_GEN_OUTPUT_FILES_eus ${roseus_INSTALL_DIR}/${pkg_name}/manifest.l)
      endif()
      # compile msg
      list(FIND ALL_GEN_OUTPUT_FILES_eus ${roseus_INSTALL_DIR}/${pkg_name}/msg/${msg_name}.l _ret)
      if(${_ret} EQUAL -1)
        add_custom_command(OUTPUT ${roseus_INSTALL_DIR}/${pkg_name}/msg/${msg_name}.l
          DEPENDS genmsg_eus ${msg_full_path}
          COMMAND ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} PYTHONPATH=${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}:$ENV{PYTHONPATH} ${GENMSG_EUS}  ${msg_full_path}
          COMMENT "Generating EusLisp code from ${pkg_name}/${msg_name}")
        list(APPEND ALL_GEN_OUTPUT_FILES_eus ${roseus_INSTALL_DIR}/${pkg_name}/msg/${msg_name}.l)
      endif()
    endforeach()
  endmacro()

  macro(_generate_msg_eus ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
    # message("_generate_msg_eus\n ARG_PKG:${ARG_PKG}\n ARG_MSG:${ARG_MSG}\n ARG_IFLAGS:${ARG_IFLAGS}\n ARG_MSG_DEPS:${ARG_MSG_DEPS}\n ARG_GEN_OUTPUT_DIR:${ARG_GEN_OUTPUT_DIR}/msg")
    get_filename_component(MSG_NAME ${ARG_MSG} NAME)
    get_filename_component(MSG_SHORT_NAME ${ARG_MSG} NAME_WE)

    set(_depend_packages "")
    set(_depend_targets "")
    foreach(_msg ${ARG_MSG_DEPS})
      get_filename_component(_msg ${_msg} ABSOLUTE)
      string(REGEX REPLACE ".*/([^/]*)/msg/[^/]*$" "\\1" _pkg ${_msg})
      find_package(${_pkg})
      if(${_pkg}_SOURCE_PREFIX)
        set(_depend_packages "${_depend_packages}:${${_pkg}_SOURCE_PREFIX}")
      endif()
      get_filename_component(_path ${_msg} PATH)
      get_filename_component(_path ${_path} PATH)
    endforeach()

    set(MSG_GENERATED_NAME ${MSG_SHORT_NAME})
    set(GEN_OUTPUT_FILE ${roseus_INSTALL_DIR}/${ARG_PKG}/msg/${MSG_GENERATED_NAME}.l)

    list(FIND ALL_GEN_OUTPUT_FILES_eus ${GEN_OUTPUT_FILE} _ret)
    if(${_ret} EQUAL -1)

      set(ROS_PACKAGE_PATH ${euslisp_PACKAGE_PATH}:${geneus_PACKAGE_PATH}:${_depend_packages}:${PROJECT_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH})

      # compile msg
      add_custom_command(OUTPUT ${GEN_OUTPUT_FILE}
        DEPENDS genmsg_eus ${ARG_MSG} ${ARG_MSG_DEPS} ${ARG_PKG}_generate_messages_py ${_depend_targets}
        COMMAND ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} PYTHONPATH=${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}:$ENV{PYTHONPATH} ${GENMSG_EUS}  ${ARG_MSG}
        COMMENT "Generating EusLisp code from ${ARG_PKG}/${MSG_NAME}")

      list(APPEND ALL_GEN_OUTPUT_FILES_eus ${GEN_OUTPUT_FILE})

    endif()

  endmacro()

  macro(_generate_srv_eus ARG_PKG ARG_SRV ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
    # message("_generate_srv_eus\n ARG_PKG:${ARG_PKG}\n ARG_SRV:${ARG_SRV}\n ARG_IFLAGS:${ARG_IFLAGS}\n ARG_SRV_DEPS:${ARG_MSG_DEPS}\n ARG_GEN_OUTPUT_DIR:${ARG_GEN_OUTPUT_DIR}/srv")
    get_filename_component(SRV_NAME ${ARG_SRV} NAME)
    get_filename_component(SRV_SHORT_NAME ${ARG_SRV} NAME_WE)

    set(_depend_package "")
    foreach(_msg ${ARG_MSG_DEPS})
      get_filename_component(_msg ${_msg} ABSOLUTE)
      string(REGEX REPLACE ".*/([^/]*)/msg/[^/]*$" "\\1" _path ${_msg})
      set(_depend_packages "${_depend_packages}:${_path}")
    endforeach()

    set(SRV_GENERATED_NAME ${SRV_SHORT_NAME})
    set(GEN_OUTPUT_FILE ${roseus_INSTALL_DIR}/${ARG_PKG}/srv/${SRV_GENERATED_NAME}.l)

    list(FIND ALL_GEN_OUTPUT_FILES_eus ${GEN_OUTPUT_FILE} _ret)
    if(${_ret} EQUAL -1)

      set(_depend_packages "")
      list(APPEND _depend_packages ${ARG_PKG}_generate_messages_py)

      set(ROS_PACKAGE_PATH ${euslisp_PACKAGE_PATH}:${geneus_PACKAGE_PATH}:${_depend_packages}:${PROJECT_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH})
      # find depends and set ROS_PACKAGE_PATH
     # _generate_eus_dep_msgs("${ARG_MSG_DEPS}")

      # compile srv
      add_custom_command(OUTPUT ${GEN_OUTPUT_FILE}
        DEPENDS gensrv_eus ${ARG_SRV} ${ARG_MSG_DEPS} ${_depend_packages}
        COMMAND ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} PYTHONPATH=${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}:$ENV{PYTHONPATH} ${GENSRV_EUS}  ${ARG_SRV}
        COMMENT "Generating EusLisp code from ${ARG_PKG}/${SRV_NAME}")

      list(APPEND ALL_GEN_OUTPUT_FILES_eus ${GEN_OUTPUT_FILE})

    endif()

  endmacro()

  macro(_generate_module_eus ARG_PKG ARG_GEN_OUTPUT_DIR ARG_GENERATED_FILES)
    # message("_generate_module_eus ${ARG_PKG} ${ARG_GEN_OUTPUT_DIR} ${ARG_GENERATED_FILES}")

    set(GEN_OUTPUT_FILE ${roseus_INSTALL_DIR}/${ARG_PKG}/manifest.l)

    list(FIND ALL_GEN_OUTPUT_FILES_eus ${GEN_OUTPUT_FILE} _ret)
    if(${_ret} EQUAL -1)

      set(ROS_PACKAGE_PATH ${euslisp_PACKAGE_PATH}:${geneus_PACKAGE_PATH}:${PROJECT_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH})
      add_custom_command(OUTPUT ${GEN_OUTPUT_FILE}
        DEPENDS genmanifest_eus ${ARG_GENERATED_FILES} ${ARG_PKG}_generate_messages_py
        COMMAND ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} ${GENMANIFEST_EUS}  ${ARG_PKG}
        COMMENT "Generating EusLisp code from ${ARG_PKG}")

      list(APPEND ALL_GEN_OUTPUT_FILES_eus ${GEN_OUTPUT_FILE})

    endif()

  endmacro()


  #  genlisp_INSTALL_DIR
  # set(roseus_INSTALL_DIR ${_ROS_HOME_DIR}/roseus/$ENV{ROS_DISTRO})

  # compile upstream packages
  get_cmake_property(_variableNames VARIABLES)
  foreach (_variableName ${_variableNames})
    # xmlrpcpp_FOUND_CATKIN_PROJECT
    if(_variableName MATCHES ".*_DIR$")
      string(REGEX REPLACE "^(.*)_DIR$" "\\1" pkg_name ${_variableName})

      if ( EXISTS ${${pkg_name}_DIR}/../package.xml OR EXISTS ${${pkg_name}_SOURCE_DIR} ) ## not in source
        message("[roseus.cmake] compile upstream package ${pkg_name} (${_variableName})")
        if (EXISTS ${${pkg_name}_DIR} ) ## not in source
          get_filename_component(_${pkg_name}_PACKAGE_PATH ${${pkg_name}_DIR}/.. ABSOLUTE)
          file(GLOB ${pkg_name}_MESSAGE_FILES RELATIVE ${_${pkg_name}_PACKAGE_PATH} "${_${pkg_name}_PACKAGE_PATH}/msg/*.msg")
          file(GLOB ${pkg_name}_SERVICE_FILES RELATIVE ${_${pkg_name}_PACKAGE_PATH} "${_${pkg_name}_PACKAGE_PATH}/srv/*.srv")
          set(ROS_PACKAGE_PATH ${euslisp_PACKAGE_PATH}:${geneus_PACKAGE_PATH}:${_${pkg_name}_PACKAGE_PATH}:$ENV{ROS_PACKAGE_PATH})
        elseif(EXISTS ${${pkg_name}_SOURCE_DIR} )
          ## set(${pkg_name}_PACKAGE_PATH  ${${pkg_name}_SOURCE_DIR}) ## ${${pkg_name}_MESSAGE_FILES} uses full path when source
          set(_${pkg_name}_PACKAGE_PATH  "")
          set(ROS_PACKAGE_PATH ${euslisp_PACKAGE_PATH}:${geneus_PACKAGE_PATH}:${${pkg_name}_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH})
        endif()

        # gen manifest
        list(FIND _ALL_GEN_OUTPUT_FILES_eus ${roseus_INSTALL_DIR}/${pkg_name}/manifest.l _ret)
        if(${_ret} EQUAL -1)
          add_custom_command(OUTPUT ${roseus_INSTALL_DIR}/${pkg_name}/manifest.l
            DEPENDS genmanifest_eus #${${pkg_name}_MESSAGE_FILES} ${${pkg_name}_SERVICE_FILES}
            COMMAND ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} ${GENMANIFEST_EUS} ${pkg_name}
            COMMENT "Generating EusLisp code from ${pkg_name}")
          list(APPEND _ALL_GEN_OUTPUT_FILES_eus ${roseus_INSTALL_DIR}/${pkg_name}/manifest.l)
        endif()

        # gen messages
        foreach(_msg_file ${${pkg_name}_MESSAGE_FILES})
          set(_msg_file ${_${pkg_name}_PACKAGE_PATH}/${_msg_file})
          get_filename_component(_msg_name ${_msg_file} NAME_WE)
          list(FIND _ALL_GEN_OUTPUT_FILES_eus ${roseus_INSTALL_DIR}/${pkg_name}/msg/${_msg_name}.l _ret)
          if(${_ret} EQUAL -1)
            add_custom_command(OUTPUT ${roseus_INSTALL_DIR}/${pkg_name}/msg/${_msg_name}.l
              DEPENDS genmsg_eus ${_msg_file}
              COMMAND ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} PYTHONPATH=${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}:$ENV{PYTHONPATH} ${GENMSG_EUS} ${_msg_file}
              COMMENT "Generating EusLisp code from ${pkg_name}/srv/${_msg_name}")
            list(APPEND _ALL_GEN_OUTPUT_FILES_eus ${roseus_INSTALL_DIR}/${pkg_name}/msg/${_msg_name}.l)
          endif()
        endforeach()

        # gen service
        foreach (_srv_file ${${pkg_name}_SERVICE_FILES})
          set(_srv_file ${_${pkg_name}_PACKAGE_PATH}/${_srv_file})
          get_filename_component(_srv_name ${_srv_file} NAME_WE)
          list(FIND _ALL_GEN_OUTPUT_FILES_eus ${roseus_INSTALL_DIR}/${pkg_name}/srv/${_srv_name}.l _ret)
          if(${_ret} EQUAL -1)
            add_custom_command(OUTPUT ${roseus_INSTALL_DIR}/${pkg_name}/srv/${_srv_name}.l
              DEPENDS gensrv_eus ${_srv_file}
              COMMAND ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH} ${GENSRV_EUS} ${_srv_file}
              COMMENT "Generating EusLisp code from ${pkg_name}/srv/${_srv_name}")
            list(APPEND _ALL_GEN_OUTPUT_FILES_eus ${roseus_INSTALL_DIR}/${pkg_name}/srv/${_srv_name}.l)
          endif()
        endforeach()
      endif( EXISTS ${${pkg_name}_DIR}/../package.xml OR EXISTS ${${pkg_name}_SOURCE_DIR})
    endif(_variableName MATCHES ".*_DIR$")
  endforeach(_variableName ${_variableNames}) # _variableName
  set(ALL_GEN_OUTPUT_FILES_eus ${_ALL_GEN_OUTPUT_FILES_eus})

  # #
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
  execute_process(COMMAND rosrun euslisp eus2 "(exit)"
    RESULT_VARIABLE _eus2_failed)
  if(_eus2_failed)
    message("[roseus.cmake] eus2 is not ready yet, so skip generating ${PROJECT_NAME}/manifeste.l")
    return()
  endif(_eus2_failed)

  set(genmanifest_eus_exe ${geneus_PACKAGE_PATH}/scripts/genmanifest_eus)
  set(manifest_eus_target_dir ${roshomedir}/roseus/$ENV{ROS_DISTRO}/${PROJECT_NAME})
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
  rosbuild_get_msgs(_msglist)
  set(_autogen "")
  foreach(_msg ${_msglist})
    # Construct the path to the .msg file
    set(_input ${PROJECT_SOURCE_DIR}/msg/${_msg})
    rosbuild_gendeps(${PROJECT_NAME} ${_msg})
    set(genmsg_eus_exe ${geneus_PACKAGE_PATH}/scripts/genmsg_eus)

    set(_output_eus ${roshomedir}/roseus/$ENV{ROS_DISTRO}/${PROJECT_NAME}/msg/${_msg})
    string(REPLACE ".msg" ".l" _output_eus ${_output_eus})

    # Add the rule to build the .l the .msg
    add_custom_command(OUTPUT ${_output_eus} ${roshomedir}/roseus/$ENV{ROS_DISTRO}/${PROJECT_NAME}/msg
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
  rosbuild_get_srvs(_srvlist)
  set(_autogen "")
  foreach(_srv ${_srvlist})
    # Construct the path to the .srv file
    set(_input ${PROJECT_SOURCE_DIR}/srv/${_srv})

    rosbuild_gendeps(${PROJECT_NAME} ${_srv})
    set(gensrv_eus_exe ${geneus_PACKAGE_PATH}/scripts/gensrv_eus)

    set(_output_eus ${roshomedir}/roseus/$ENV{ROS_DISTRO}/${PROJECT_NAME}/srv/${_srv})
    string(REPLACE ".srv" ".l" _output_eus ${_output_eus})

    # Add the rule to build the .l from the .srv
    add_custom_command(OUTPUT ${_output_eus} ${roshomedir}/roseus/$ENV{ROS_DISTRO}/${PROJECT_NAME}/srv
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
  execute_process(COMMAND rosrun euslisp eus2 "(exit)"
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
        (NOT EXISTS ${roshomedir}/roseus/$ENV{ROS_DISTRO}/${_package}/action_msg/generated))
      message("[roseus.cmake] generate msg from action")
      file(GLOB _actions RELATIVE "${${_package}_PACKAGE_PATH}/action/" "${${_package}_PACKAGE_PATH}/action/*.action")
      foreach(_action ${_actions})
	message("[roseus.cmake] genaction.py ${_action} -o ${roshomedir}/roseus/$ENV{ROS_DISTRO}/${_package}/action_msg/")
	execute_process(COMMAND rosrun actionlib_msgs genaction.py ${${_package}_PACKAGE_PATH}/action/${_action} -o ${roshomedir}/roseus/$ENV{ROS_DISTRO}/${_package}/action_msg)
      endforeach()
      file(GLOB _action_msgs "${roshomedir}/roseus/$ENV{ROS_DISTRO}/${_package}/action_msg/*.msg")
      foreach(_action_msg ${_action_msgs})
	message("[roseus.cmake] rosrun roseus genmsg_eus ${_action_msg}")
	execute_process(COMMAND rosrun roseus genmsg_eus ${_action_msg})
      endforeach()
      file(WRITE ${roshomedir}/roseus/$ENV{ROS_DISTRO}/${_package}/action_msg/generated "generated")
    endif()
    ###
    set(msggenerated "${roshomedir}/roseus/$ENV{ROS_DISTRO}/${_package}/generated")
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


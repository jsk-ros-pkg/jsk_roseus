cmake_minimum_required(VERSION 2.8.3)
project(roseus_msgs)

set(_roseus_INSTALL_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/roseus/ros)
if(NOT euslisp_SOURCE_DIR)
  message("[roseus_msgs] generate messages for install")
  set(roseus_msgs_generate TRUE)
endif()

if (${roseus_msgs_generate})
  set(roseus_INSTALL_DIR ${_roseus_INSTALL_DIR})

  find_package(catkin REQUIRED COMPONENTS
    rviz
    # moveit_full_pr2
    moveit_ros_planning_interface
    moveit_setup_assistant
    pr2_moveit_plugins
    # pr2eus
    pr2_msgs
    pr2_controllers_msgs
    control_msgs
    move_base_msgs
    nav_msgs
    dynamic_reconfigure

    geneus # this load roseus.cmake, so it needs to be located in the end
  )

  _generate_eus_dep_msgs(${PROJECT_NAME}) # create custom_command

   add_custom_target(roseus_msgs_generate_all_msgs ALL DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}) # generate all

  install(DIRECTORY ${roseus_INSTALL_DIR}/
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/roseus/ros
    )

  set(roseus_INSTALL_DIR) # unset

endif()




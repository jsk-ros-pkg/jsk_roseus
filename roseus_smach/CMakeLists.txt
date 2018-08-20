cmake_minimum_required(VERSION 2.8.3)
project(roseus_smach)

find_package(catkin REQUIRED COMPONENTS std_msgs actionlib smach smach_ros smach_msgs actionlib_tutorials message_generation roseus)

add_action_files(
  DIRECTORY action
  FILES
  Multiply10.action
  Sub5.action)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
#    DEPENDS 
    CATKIN_DEPENDS euslisp roseus smach smach_ros smach_msgs # TODO
#    INCLUDE_DIRS # TODO include
#    LIBRARIES # TODO
)

install(DIRECTORY sample src test
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(test/test_samples.test)
  add_rostest(test/test_samples_action_client_state.test)
  add_rostest(test/test_action_server.test)
  add_rostest(test/test_action_client_state_in_server.test)
  add_rostest(test/test_roseus_smach_utils.test)
  add_rostest(test/test_async_join_state_machine_actionlib.test)
  add_rostest(test/test_parallel_state_machine_sample.test)
endif()


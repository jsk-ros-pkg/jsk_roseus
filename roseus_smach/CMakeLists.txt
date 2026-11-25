cmake_minimum_required(VERSION 3.10)
project(roseus_smach)

if("$ENV{ROS_VERSION}" STREQUAL "")
  if(NOT "$ENV{AMENT_PREFIX_PATH}" STREQUAL "")
    set(ENV{ROS_VERSION} 2)
  else()
    set(ENV{ROS_VERSION} 1)
  endif()
  message(WARNING "ROS_VERSION is not set, assuming ROS_VERSION=$ENV{ROS_VERSION}")
endif()

if($ENV{ROS_VERSION} EQUAL 2)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_generator_eus REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  action/Multiply10.action
  action/Sub5.action
  DEPENDENCIES action_msgs
)
install(DIRECTORY sample src test
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_roseus_smach_ros2 test/ros2/test_roseus_smach_ros2.py
    TIMEOUT 600)
endif()
ament_export_dependencies(rosidl_default_runtime)
ament_package()

else() # if($ENV{ROS_VERSION} EQUAL 2)

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
    CATKIN_DEPENDS roseus smach smach_ros smach_msgs # TODO
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

endif() # if($ENV{ROS_VERSION} EQUAL 2)

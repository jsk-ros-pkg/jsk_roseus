cmake_minimum_required(VERSION 2.8.3)
project(roseus_test_genmsg2)

if(NOT USE_ROSBUILD) # catkin

find_package(catkin REQUIRED COMPONENTS message_generation roseus roscpp roseus_test_genmsg)

add_message_files(
  FILES Child.msg
)
generate_messages(
  DEPENDENCIES geometry_msgs std_msgs roseus_test_genmsg
)
catkin_package(
    CATKIN_DEPENDS message_runtime
)


else() ## rosbuild

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

rosbuild_gensrv()
rosbuild_genmsg()

endif()


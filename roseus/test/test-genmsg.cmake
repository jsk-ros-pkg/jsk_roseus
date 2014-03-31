cmake_minimum_required(VERSION 2.8.3)
project(roseus_test_genmsg)

if(NOT USE_ROSBUILD) # catkin

find_package(catkin REQUIRED COMPONENTS message_generation roseus roscpp)

add_service_files(
  FILES Empty.srv
)
add_message_files(
  FILES String.msg
)
generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)
catkin_package(
    CATKIN_DEPENDS message_runtime
)


add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

else() ## rosbuild

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

rosbuild_gensrv()
rosbuild_genmsg()

rosbuild_add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)

endif()


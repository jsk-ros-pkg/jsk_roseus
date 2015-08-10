if ("$ENV{ROS_DISTRO}" STREQUAL "jade")
  return()
endif()

cmake_minimum_required(VERSION 2.8.3)
project(roseus_mongo)

find_package(catkin REQUIRED COMPONENTS
  rostest
  mongodb_store
  mongodb_store_msgs
  roseus
)

catkin_package(
  CATKIN_DEPENDS mongodb_store mongodb_store_msgs roseus
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

add_rostest(test/test_json_encode.test)
add_rostest(test/test_json_decode.test)
add_rostest(test/test_mongo_client.test)

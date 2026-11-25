cmake_minimum_required(VERSION 3.10)
project(roseus_tutorials)

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
install(DIRECTORY launch src img config test
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_roseus_tutorials_ros2 test/ros2/test_roseus_tutorials_ros2.py TIMEOUT 120)
endif()
ament_package()

else() # if($ENV{ROS_VERSION} EQUAL 2)

find_package(catkin REQUIRED COMPONENTS
  roseus
  )

catkin_package(
#    DEPENDS
#    CATKIN_DEPENDS roseus
#    INCLUDE_DIRS # TODO include
#    LIBRARIES # TODO
)

install(DIRECTORY launch src img config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)

# if(CATKIN_ENABLE_TESTING)
#   find_package(catkin REQUIRED COMPONENTS
#     rostest)
#   add_rostest(test/test-tabletop-object-detector.test)
# endif()

# rosbuild_download_data(http://www.boj.or.jp/note_tfjgs/note/valid/img/bn_1000f_e.jpg img/1000yen.jpg fee3e1818618024c63246096db2d9507)
# rosbuild_download_data(http://www.boj.or.jp/note_tfjgs/note/valid/img/bn_2000f_d.jpg img/2000yen.jpg 7be94640b3d591b886eeb11c7e184d98)
# rosbuild_download_data(http://www.boj.or.jp/note_tfjgs/note/valid/img/bn_5000f_e.jpg img/5000yen.jpg d3118921dea5eafd296ae8130933ca1f)
# rosbuild_download_data(http://www.boj.or.jp/note_tfjgs/note/valid/img/bn_10000f_e.jpg img/10000yen.jpg 7ebda101f67a3f1a192ef096202e53f1)

endif() # if($ENV{ROS_VERSION} EQUAL 2)

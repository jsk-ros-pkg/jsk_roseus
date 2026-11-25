cmake_minimum_required(VERSION 3.10)
project(roseus_mongo)

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
install(DIRECTORY euslisp test
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)
if(BUILD_TESTING)
  # the test of mongo-client.l is skipped when mongodb_store (not released
  # for ROS 2 yet) or mongod is not found
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_roseus_mongo_ros2 test/ros2/test_roseus_mongo_ros2.py TIMEOUT 300)
endif()
ament_package()

else() # if($ENV{ROS_VERSION} EQUAL 2)

find_package(mongodb_store QUIET)
if(mongodb_store_FOUND)
  set(MONGODB_STORE_PKG mongodb_store)
else()
  message(AUTHOR_WARNING "===\nros-$ENV{ROS_DISTRO}-mongodb-store package is not found, may be it is not releaed yet\n===")
endif()

if ("$ENV{ROS_DISTRO}" STRLESS "jade") # up to indigo we need to compile msg fiel
  set(COMPILE_PACKAGES mongodb_store_msgs)
endif()

find_package(catkin REQUIRED COMPONENTS
  ${MONGODB_STORE_PKG} ${COMPILE_PACKAGES}
  roseus
)

catkin_package(
  CATKIN_DEPENDS ${MONGODB_STORE_PKG} ${COMPILE_PACKAGES} roseus
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

if(CATKIN_ENABLE_TESTING AND mongodb_store_FOUND)
  find_package(rostest REQUIRED)
  add_rostest(test/test_json_encode.test)
  add_rostest(test/test_json_decode.test)
  if(${mongodb_store_VERSION} VERSION_LESS "0.1.17")
    message(WARNING "detected old mongodb_store. Testing with old launch file")
    add_rostest(test/test_mongo_client_hydro.test)
  else()
    add_rostest(test/test_mongo_client.test)
  endif()
endif()

endif() # if($ENV{ROS_VERSION} EQUAL 2)

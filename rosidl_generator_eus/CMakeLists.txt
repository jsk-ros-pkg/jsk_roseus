cmake_minimum_required(VERSION 3.10)
project(rosidl_generator_eus NONE)

if("$ENV{ROS_VERSION}" STREQUAL "")
  if(NOT "$ENV{AMENT_PREFIX_PATH}" STREQUAL "")
    set(ENV{ROS_VERSION} 2)
  else()
    set(ENV{ROS_VERSION} 1)
  endif()
endif()

if($ENV{ROS_VERSION} EQUAL 2)

find_package(ament_cmake REQUIRED)

# every package calling rosidl_generate_interfaces() finds this package
# through rosidl_core_generators, and runs the extension
ament_index_register_resource("rosidl_generator_packages")

install(PROGRAMS scripts/gen_eus_ros2.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES cmake/rosidl_generator_eus_generate_interfaces.cmake
  DESTINATION share/${PROJECT_NAME}/cmake)

ament_package(CONFIG_EXTRAS cmake/rosidl_generator_eus-extras.cmake.in)

else()  # ROS 1: geneus generates euslisp messages

find_package(catkin REQUIRED)
catkin_package()

endif()

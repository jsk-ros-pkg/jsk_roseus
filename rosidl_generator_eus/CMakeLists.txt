cmake_minimum_required(VERSION 3.12)

project(rosidl_generator_eus)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_index_register_resource("rosidl_generator_packages")

ament_python_install_package(${PROJECT_NAME})

ament_package(
  CONFIG_EXTRAS "rosidl_generator_eus-extras.cmake.in"
)

install(
  PROGRAMS bin/rosidl_generator_eus
  DESTINATION lib/rosidl_generator_eus
)
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

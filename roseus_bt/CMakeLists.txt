cmake_minimum_required(VERSION 3.0.2)
project(roseus_bt)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  behaviortree_ros
)

find_package(TinyXML2 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES
 CATKIN_DEPENDS
 DEPENDS TinyXML2
)


include_directories( include ${catkin_INCLUDE_DIRS} ${TinyXML2_INCUDE_DIRS})


add_executable(test_parse src/test_parse.cpp)
add_dependencies(test_parse ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_parse ${catkin_LIBRARIES} ${TinyXML2_LIBRARIES})

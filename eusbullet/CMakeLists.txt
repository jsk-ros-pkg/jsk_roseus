cmake_minimum_required(VERSION 2.8.3)
project(eusbullet)

find_package(catkin COMPONENTS cmake_modules rostest)

catkin_package(
    DEPENDS euslisp jskeus
    LIBRARIES eusbullet BulletCollision BulletDynamics LinearMath ConvexDecomposition
  )


add_subdirectory(bullet3)

set(BULLET_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/bullet3/src/)
set(BULLET_LIBRARIES BulletCollision BulletDynamics LinearMath ConvexDecomposition)


include_directories(
  ${BULLET_INCLUDE_DIRS}
  )

add_library(eusbullet SHARED src/eusbullet.cpp)
target_link_libraries(eusbullet ${BULLET_LIBRARIES})

add_rostest(test/test_collision_distance.test)

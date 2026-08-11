cmake_minimum_required(VERSION 2.8.3)
project(eusbullet)

find_package(catkin COMPONENTS cmake_modules rostest)

find_package(PkgConfig)
pkg_check_modules(BULLET REQUIRED bullet)

catkin_package(
    DEPENDS euslisp jskeus
    LIBRARIES eusbullet
  )

# for debug
# message("[PKG_CONFIG_EXECUTABLE] ${PKG_CONFIG_EXECUTABLE}")
# message("[BULLET_FOUND] ${BULLET_FOUND}")
# message("[BULLET_LIBRARIES] ${BULLET_LIBRARIES}")
# message("[BULLET_INCLUDE_DIRS] ${BULLET_INCLUDE_DIRS}")

include_directories(
  ${BULLET_INCLUDE_DIRS}
  )

add_library(eusbullet SHARED src/eusbullet.cpp)
target_link_libraries(eusbullet ${BULLET_LIBRARIES})

add_rostest(test/test_collision_distance.test)

rosbuild_find_ros_package(geneus)
if(NOT TARGET ROSBUILD_genmanifest_roseus_${PROJECT_NAME})
  include(${geneus_PACKAGE_PATH}/cmake/roseus.cmake)
endif()


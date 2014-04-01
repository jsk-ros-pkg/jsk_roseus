rosbuild_find_ros_package(geneus)
message("--??aa>> ${ROSBUILD_roseus_${PROJECT_NAME}}---")
if(NOT TARGET ROSBUILD_genmanifest_roseus_${PROJECT_NAME})
  include(${geneus_PACKAGE_PATH}/cmake/roseus.cmake)
endif()


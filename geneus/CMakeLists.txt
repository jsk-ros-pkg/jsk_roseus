if(USE_ROSBUILD)
  return()
endif()

cmake_minimum_required(VERSION 2.8.3)
project(geneus)

find_package(catkin REQUIRED COMPONENTS euslisp genmsg)

catkin_package(
  CFG_EXTRAS roseus.cmake
)

install(DIRECTORY scripts
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS
)


file(WRITE ${CATKIN_DEVEL_PREFIX}/${GENMSG_LANGS_DESTINATION}/geneus "EusLisp")

install(FILES ${CATKIN_DEVEL_PREFIX}/${GENMSG_LANGS_DESTINATION}/geneus
  DESTINATION ${GENMSG_LANGS_DESTINATION})
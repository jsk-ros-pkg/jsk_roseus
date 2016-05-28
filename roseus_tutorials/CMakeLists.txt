cmake_minimum_required(VERSION 2.8.3)
project(roseus_tutorials)

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


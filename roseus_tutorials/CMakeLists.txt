cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_download_data(http://www.boj.or.jp/note_tfjgs/note/valid/img/bn_1000f_e.jpg img/1000yen.jpg fee3e1818618024c63246096db2d9507)
rosbuild_download_data(http://www.boj.or.jp/note_tfjgs/note/valid/img/bn_2000f_d.jpg img/2000yen.jpg 7be94640b3d591b886eeb11c7e184d98)
rosbuild_download_data(http://www.boj.or.jp/note_tfjgs/note/valid/img/bn_5000f_e.jpg img/5000yen.jpg d3118921dea5eafd296ae8130933ca1f)
rosbuild_download_data(http://www.boj.or.jp/note_tfjgs/note/valid/img/bn_10000f_e.jpg img/10000yen.jpg 7ebda101f67a3f1a192ef096202e53f1)

# roslaunch documentation
rosbuild_find_ros_package("jsk_tools")
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} python ${jsk_tools_PACKAGE_PATH}/bin/launchdoc-generator.py ${PROJECT_NAME} --output_dir=./build --nomakefile RESULT_VARIABLE _make_failed)


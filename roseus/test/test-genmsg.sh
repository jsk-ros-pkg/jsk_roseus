#!/bin/bash

if [ $# -gt 0 -a "$1" == "ROSBUILD" ] ; then
    ROSBUILD=TRUE
fi

CATKIN_DIR=/tmp/test_genmsg_$$
PACKAGE_NAME=roseus_test_genmsg
mkdir -p ${CATKIN_DIR}/src/${PACKAGE_NAME}

trap 'rm -fr ${CATKIN_DIR}; exit 1' 1 2 3 15

cp `rospack find roseus`/test/test-genmsg.mak   ${CATKIN_DIR}/src/${PACKAGE_NAME}/Makefile
cp `rospack find roseus`/test/test-genmsg.cmake ${CATKIN_DIR}/src/${PACKAGE_NAME}/CMakeLists.txt
cp `rospack find roseus`/test/test-genmsg.package.xml    ${CATKIN_DIR}/src/${PACKAGE_NAME}/package.xml
cp `rospack find roseus`/test/test-genmsg.manifest.xml   ${CATKIN_DIR}/src/${PACKAGE_NAME}/manifest.xml
cp `rospack find roseus`/test/test-genmsg.cpp   ${CATKIN_DIR}/src/${PACKAGE_NAME}/${PACKAGE_NAME}.cpp
cp -r `rospack find std_msgs`/msg   ${CATKIN_DIR}/src/${PACKAGE_NAME}/msg
cp -r `rospack find std_srvs`/srv   ${CATKIN_DIR}/src/${PACKAGE_NAME}/srv
cd ${CATKIN_DIR}
if [ ${ROSBUILD} ] ; then
    export ROS_PACKAGE_PATH=${CATKIN_DIR}/src/${PACKAGE_NAME}:$ROS_PACKAGE_PATH
    (cd ${CATKIN_DIR}/src/${PACKAGE_NAME}; make)
else
    catkin_make
    source ${CATKIN_DIR}/devel/setup.bash
fi
ROS_MASTER_URI=http://localhost:22422 rosrun ${PACKAGE_NAME} ${PACKAGE_NAME}
exit 0

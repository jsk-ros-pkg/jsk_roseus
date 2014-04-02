#!/bin/bash

if [ $# -gt 0 -a "$1" == "ROSBUILD" ] ; then
    ROSBUILD=TRUE
    MANIFEST=manifest.xml
else
    MANIFEST=package.xml
fi

CATKIN_DIR=/tmp/test_genmsg_$$
PACKAGE_NAME=roseus_test_genmsg
PACKAGE2_NAME=roseus_test_genmsg2
mkdir -p ${CATKIN_DIR}/src/${PACKAGE_NAME}
mkdir -p ${CATKIN_DIR}/src/${PACKAGE2_NAME}

#trap 'rm -fr ${CATKIN_DIR}; exit 1' 1 2 3 15

cp `rospack find roseus`/test/test-genmsg.mak   ${CATKIN_DIR}/src/${PACKAGE_NAME}/Makefile
cp `rospack find roseus`/test/test-genmsg.cmake ${CATKIN_DIR}/src/${PACKAGE_NAME}/CMakeLists.txt
cp `rospack find roseus`/test/test-genmsg.${MANIFEST}    ${CATKIN_DIR}/src/${PACKAGE_NAME}/${MANIFEST}
cp `rospack find roseus`/test/test-genmsg.cpp   ${CATKIN_DIR}/src/${PACKAGE_NAME}/${PACKAGE_NAME}.cpp
cp `rospack find roseus`/test/test-genmsg.l     ${CATKIN_DIR}/src/${PACKAGE_NAME}/${PACKAGE_NAME}.l
cp -r `rospack find std_msgs`/msg   ${CATKIN_DIR}/src/${PACKAGE_NAME}/msg
cp -r `rospack find std_srvs`/srv   ${CATKIN_DIR}/src/${PACKAGE_NAME}/srv

cp `rospack find roseus`/test/test-genmsg.mak    ${CATKIN_DIR}/src/${PACKAGE2_NAME}/Makefile
cp `rospack find roseus`/test/test-genmsg2.cmake ${CATKIN_DIR}/src/${PACKAGE2_NAME}/CMakeLists.txt
cp `rospack find roseus`/test/test-genmsg2.${MANIFEST}    ${CATKIN_DIR}/src/${PACKAGE2_NAME}/${MANIFEST}
mkdir -p ${CATKIN_DIR}/src/${PACKAGE2_NAME}/msg/
cp `rospack find roseus`/test/test-genmsg2.l     ${CATKIN_DIR}/src/${PACKAGE_NAME2}/${PACKAGE_NAME2}.l
cp `rospack find roseus`/test/test-genmsg2.child.msg   ${CATKIN_DIR}/src/${PACKAGE2_NAME}/msg/Child.msg

# add roseus

cd ${CATKIN_DIR}
if [ ${ROSBUILD} ] ; then
    export ROS_PACKAGE_PATH=${CATKIN_DIR}/src/:${ROS_PACKAGE_PATH}
    rospack profile
    rosmake  -V ${PACKAGE2_NAME}
else
    catkin_make
    source ${CATKIN_DIR}/devel/setup.bash
fi


# try to run roseus sample program
ROS_MASTER_URI=http://localhost:22422 rosrun roseus roseus ${CATKIN_DIR}/src/${PACKAGE_NAME}/${PACKAGE_NAME}.l $@
ROS_MASTER_URI=http://localhost:22422 rosrun roseus roseus ${CATKIN_DIR}/src/${PACKAGE_NAME2}/${PACKAGE_NAME2}.l $@

exit 0

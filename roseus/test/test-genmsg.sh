#!/bin/bash

set -e                          # exit on error

#######################
# a script to test geneus
# it supports
#   * rosbuild/catkin
#   * one workspace/multiple workspaces
#   * several dependency situation

# dependency of msg packages
#    geneus
#    geneus_dep1 (depends on geneus)
#    geneus_dep2 (depends on geneus and geneus_dep1)
#    roseus
#    roseus_dep1 (depends on roseus and geneus_dep2)
#    roseus_dep2 (depends on roseus and roseus_dep1)


# parse arguments
MANIFEST=package.xml
WORKSPACE_TYPE=MULTI
ARGV=$@
while [ $# -gt 0 ]; do
    case "$1" in 
        "--rosbuild")
            ROSBUILD=TRUE
            MANIFEST=manifest.xml
            ;;
        "--one-workspace")
            WORKSPACE_TYPE=ONE
            ;;
    esac
    shift
done

CATKIN_DIR=/tmp/test_genmsg_$$
GENEUS_DEP1=${CATKIN_DIR}/src/geneus_dep1
GENEUS_DEP2=${CATKIN_DIR}/src/geneus_dep2
ROSEUS_DEP1=${CATKIN_DIR}/src/roseus_dep1
ROSEUS_DEP2=${CATKIN_DIR}/src/roseus_dep2

mkdir -p ${GENEUS_DEP1}/{msg,srv,action}
mkdir -p ${GENEUS_DEP2}/{msg,srv,action}
mkdir -p ${ROSEUS_DEP1}/{msg,srv,action}
mkdir -p ${ROSEUS_DEP2}/{msg,srv,action}

#trap 'rm -fr ${CATKIN_DIR}; exit 1' 1 2 3 15

function add_makefile() {
    cat <<EOF > $1/Makefile
EXTRA_CMAKE_FLAGS = -DUSE_ROSBUILD:BOOL=1
include \$(shell rospack find mk)/cmake.mk
EOF
}

function add_package.xml() {
    pkg_path=$1
    pkg_name=$2
    shift
    shift
    cat <<EOF > $pkg_path/package.xml
<package> 
<name>$pkg_name</name>
<version>0.0.1</version>
<description> genmsg test for roseus</description>
<maintainer email="k-okada@jsk.t.u-tokyo.ac.jp">Kei Okada</maintainer>
<license>BSD</license>
<buildtool_depend>catkin</buildtool_depend>
<build_depend>roscpp</build_depend>
<build_depend>message_generation</build_depend>
<build_depend>geometry_msgs</build_depend>
<build_depend>actionlib_msgs</build_depend>
<build_depend>std_msgs</build_depend>
<run_depend>roscpp</run_depend>
<run_depend>message_generation</run_depend>
<run_depend>geometry_msgs</run_depend>
<run_depend>actionlib_msgs</run_depend>
<run_depend>std_msgs</run_depend>
$(for pkg in $@
do
  echo '<build_depend>'$pkg'</build_depend><run_depend>'$pkg'</run_depend>'
done)
<run_depend>message_runtime</run_depend>
</package>
EOF
}

function add_manifest.xml() {
    pkg_path=$1
    pkg_name=$2
    shift
    shift
    
    cat <<EOF > $pkg_path/manifest.xml
<package>
  <description breif="genmsg test for roseus">genmsg test for roseus</description>
  <author>Kei Okada (k-okada@jsk.t.u-tokyo.ac.jp)</author>

  <license>BSD</license>

  <depend package="roseus"/>
  <depend package="roscpp"/>
  <depend package="actionlib_msgs"/>
  <depend package="geometry_msgs"/>
$(for pkg in $@
do
  echo '<depend package="'$pkg'"/>'
done)
</package>
EOF

}

function add_cmake() {
    pkg_path=$1
    shift
    cat <<EOF >$pkg_path/CMakeLists.txt
    cmake_minimum_required(VERSION 2.8.3)
project($(basename $pkg_path))

if(NOT USE_ROSBUILD) # catkin

find_package(catkin REQUIRED COMPONENTS message_generation roscpp geometry_msgs actionlib_msgs
$(for pkg in $1
do
  echo $pkg
done)
)

add_service_files(
  FILES Empty.srv
)
add_message_files(
  FILES String.msg String2.msg
)
add_action_files(
  FILES Foo.action
)
generate_messages(
  DEPENDENCIES geometry_msgs std_msgs actionlib_msgs
$(for pkg in $2
do
  echo $pkg
done)
)
catkin_package(
    CATKIN_DEPENDS message_runtime geometry_msgs std_msgs actionlib_msgs
$(for pkg in $2
do
  echo $pkg
done)
)


add_executable(\${PROJECT_NAME} \${PROJECT_NAME}.cpp)
target_link_libraries(\${PROJECT_NAME} \${catkin_LIBRARIES})
add_dependencies(\${PROJECT_NAME} \${PROJECT_NAME}_generate_messages_cpp)

else() ## rosbuild

include(\$ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

rosbuild_gensrv()
rosbuild_genmsg()

rosbuild_add_executable(\${PROJECT_NAME} \${PROJECT_NAME}.cpp)

endif()
EOF
}

function add_cpp() {
    pkg_path=$1
    pkg_name=$2
    cat <<EOF > $pkg_path/$pkg_name.cpp
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <$pkg_name/String.h>
#include <$pkg_name/Empty.h>

bool empty($pkg_name::EmptyRequest  &req,
           $pkg_name::EmptyResponse &res){
    return true;
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "roseus_test_genmsg");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<$pkg_name::String>("talker2", 100);

    $pkg_name::EmptyRequest srv;
    ros::ServiceServer service = n.advertiseService("empty", empty);

    ros::Rate rate(10);
    while (ros::ok()) {

        $pkg_name::String msg;
        msg.data = "msg";

        pub.publish(msg);


        rate.sleep();

        ros::spinOnce();
    }

    return 0;
}

EOF
}

function add_lisp() {
    pkg_path=$1
    pkg_name=$2
    cat <<EOF > $pkg_path/$pkg_name.l
(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(ros::roseus "roseus_test_genmsg")

(deftest test-msg-instance
  (assert (ros::load-ros-manifest "$pkg_name")
          "load-ros-manifest")

  (assert (eval (read-from-string "(instance $pkg_name::String :init)"))
          "instantiating msg message")
  )

(run-all-tests)

(exit)
EOF
}

function add_msg() {
    pkg_path=$1
    parent_pkg=$2
    cat <<EOF >$pkg_path/msg/String.msg
Header header
string data
$parent_pkg/String parent_data
EOF
    cat <<EOF >$pkg_path/msg/String2.msg
Header header
string data
$parent_pkg/String parent_data
EOF
}

function add_action() {
   pkg_path=$1
   parent_pkg=$2
   cat <<EOF >$pkg_path/action/Foo.action
#goal
Header header
string data
$parent_pkg/String parent_data
---
#result
---
#feedback

EOF
}

function add_srv() {
    pkg_path=$1
    cat <<EOF >$pkg_path/srv/Empty.srv
EOF
}

# makeup packages
add_makefile ${GENEUS_DEP1} 
add_makefile ${GENEUS_DEP2} 
add_makefile ${ROSEUS_DEP1}
add_makefile ${ROSEUS_DEP2}
add_${MANIFEST} ${GENEUS_DEP1} geneus_dep1 geneus
add_${MANIFEST} ${GENEUS_DEP2} geneus_dep2 geneus geneus_dep1
add_${MANIFEST} ${ROSEUS_DEP1} roseus_dep1 roseus geneus_dep2 geneus_dep1
add_${MANIFEST} ${ROSEUS_DEP2} roseus_dep2 roseus_dep1 roseus geneus_dep1 geneus_dep2

add_cmake ${GENEUS_DEP1} 
add_cmake ${GENEUS_DEP2} "geneus_dep1" "geneus_dep1"
add_cmake ${ROSEUS_DEP1} "geneus_dep1 roseus geneus_dep2" "geneus_dep1 roseus geneus_dep2"
add_cmake ${ROSEUS_DEP2} "geneus_dep1 roseus geneus_dep2 roseus_dep1" "geneus_dep1 roseus geneus_dep2 roseus_dep1"
add_cpp ${GENEUS_DEP1} geneus_dep1
add_cpp ${GENEUS_DEP2} geneus_dep2
add_cpp ${ROSEUS_DEP1} roseus_dep1
add_cpp ${ROSEUS_DEP2} roseus_dep2
add_lisp ${GENEUS_DEP1} geneus_dep1
add_lisp ${GENEUS_DEP2} geneus_dep2
add_lisp ${ROSEUS_DEP1} roseus_dep1
add_lisp ${ROSEUS_DEP2} roseus_dep2

add_msg ${GENEUS_DEP1} std_msgs
add_msg ${GENEUS_DEP2} geneus_dep1
add_msg ${ROSEUS_DEP1} geneus_dep2
add_msg ${ROSEUS_DEP2} roseus_dep1

add_action ${GENEUS_DEP1} std_msgs
add_action ${GENEUS_DEP2} geneus_dep1
add_action ${ROSEUS_DEP1} geneus_dep2
add_action ${ROSEUS_DEP2} roseus_dep1


add_srv ${GENEUS_DEP1} std_msgs
add_srv ${GENEUS_DEP2} geneus_dep1
add_srv ${ROSEUS_DEP1} geneus_dep2
add_srv ${ROSEUS_DEP2} roseus_dep1


if [ $WORKSPACE_TYPE = ONE -a ! -e ${CATKIN_DIR}/src/jsk_roseus ]; then
    if [ ! -e `rospack find roseus`/CMakeLists.txt ]; then
        echo "$0: Could not found roseus source directory so quitting..."
        exit 0
    fi
    cp -r `rospack find roseus`/.. ${CATKIN_DIR}/src/jsk_roseus
    if [ ${ROSBUILD} ] ; then
        rm -fr ${CATKIN_DIR}/src/jsk_roseus/euslisp/build # rm if alredy rosmaked
        rm -fr ${CATKIN_DIR}/src/jsk_roseus/geneus/build # rm if alredy rosmaked
        rm -fr ${CATKIN_DIR}/src/jsk_roseus/roseus/build # rm if alredy rosmaked
    fi
fi

if [ $WORKSPACE_TYPE = ONE ]; then
    # reset environmental variables by sousing
    source /opt/ros/${ROS_DISTRO}/setup.bash
fi

cd ${CATKIN_DIR}
if [ ${ROSBUILD} ] ; then
    export ROS_PACKAGE_PATH=${CATKIN_DIR}/src/:${ROS_PACKAGE_PATH}
    rospack profile
    rosmake  -V roseus_dep2
else
    # force to clear roseus cache
    rm -rf ~/.ros/roseus/${ROS_DISTRO}
    # always call twice catkin_make
    catkin_make
    catkin_make --force-cmake
    
    source ${CATKIN_DIR}/devel/setup.bash
fi


# # try to run roseus sample program
ROSEUS_DIR=`rospack find roseus`
ROSEUS_EXE=`find $ROSEUS_DIR -type f -name roseus`
if [ ! "$ROSEUS_EXE" ]; then
    ROSEUS_EXE="rosrun roseus roseus"
fi

ROS_MASTER_URI=http://localhost:22422 ${ROSEUS_EXE} ${CATKIN_DIR}/src/geneus_dep1/geneus_dep1.l $ARGV
ROS_MASTER_URI=http://localhost:22422 ${ROSEUS_EXE} ${CATKIN_DIR}/src/geneus_dep2/geneus_dep2.l $ARGV
ROS_MASTER_URI=http://localhost:22422 ${ROSEUS_EXE} ${CATKIN_DIR}/src/roseus_dep1/roseus_dep1.l $ARGV
ROS_MASTER_URI=http://localhost:22422 ${ROSEUS_EXE} ${CATKIN_DIR}/src/roseus_dep2/roseus_dep2.l $ARGV

rm -rf ${CATKIN_DIR}

exit 0

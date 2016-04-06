#!/bin/bash

set -e                          # exit on error

#######################
# a script to test geneus
# it supports
#   * catkin
#   * one workspace/multiple workspaces
#   * several dependency situation

# dependency of msg packages
#    geneus
#    geneus_dep1 (depends on geneus)
#    geneus_dep2 (depends on geneus and geneus_dep1)
#    roseus
#    roseus_dep1 (depends on roseus and geneus_dep2)
#    roseus_dep2 (depends on roseus and roseus_dep1)
#    roseus_dep3 (build depends on roseus_dep1, roseus_dep2, generate_messages roseus_dep1, using messages in roseus_dep2) # manifest should have all depends packages


# parse arguments
MANIFEST=package.xml
WORKSPACE_TYPE=MULTI
ARGV=$@
PACKAGE=ALL
REMOVE_MSG=FALSE
while [ $# -gt 0 ]; do
    case "$1" in 
        "--one-workspace")
            WORKSPACE_TYPE=ONE
            ;;
        "--package")
            shift
            PACKAGE=$1
            ;;
        "--remove-message")
            REMOVE_MSG=TRUE
            ;;
        --gtest_output=* )
            OUTPUT=${1#--gtest_output=xml:}
            ;;
        --results-filename )
            shift
            OUTPUT=${CATKIN_RESULTS_TEST_DIR}/$1
            ;;
    esac
    shift
done

CATKIN_DIR=/tmp/test_genmsg_$$

>&2 echo -e "\e[1;32mRunning test-genmsg.sh $ARGV\e[m"
>&2 echo -e "\e[1;32m - WORKSPACE_TYPE=${WORKSPACE_TYPE}\e[m"
>&2 echo -e "\e[1;32m - PACKAGE=${PACKAGE}\e[m"
>&2 echo -e "\e[1;32m - REMOVE_MSG=${REMOVE_MSG}\e[m"
>&2 echo -e "\e[1;32m - OUTPUT=${OUTPUT}\e[m"
>&2 echo -e "\e[1;32m - CATKIN_DIR=${CATKIN_DIR}\e[m"

GENEUS_DEP1=${CATKIN_DIR}/src/geneus_dep1
GENEUS_DEP2=${CATKIN_DIR}/src/geneus_dep2
ROSEUS_DEP1=${CATKIN_DIR}/src/roseus_dep1
ROSEUS_DEP2=${CATKIN_DIR}/src/roseus_dep2
ROSEUS_DEP3=${CATKIN_DIR}/src/roseus_dep3
ROSEUS_DEP4=${CATKIN_DIR}/src/roseus_dep4

mkdir -p ${GENEUS_DEP1}/{msg,srv,action}
mkdir -p ${GENEUS_DEP2}/{msg,srv,action}
mkdir -p ${ROSEUS_DEP1}/{msg,srv,action}
mkdir -p ${ROSEUS_DEP2}/{msg,srv,action}
mkdir -p ${ROSEUS_DEP3}/{msg,srv,action}
mkdir -p ${ROSEUS_DEP4}/action

#trap 'rm -fr ${CATKIN_DIR}; exit 1' 1 2 3 15

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
<build_depend>sensor_msgs</build_depend>
<build_depend>actionlib_msgs</build_depend>
<build_depend>std_msgs</build_depend>
<build_depend>roseus</build_depend>
<run_depend>roscpp</run_depend>
<run_depend>message_generation</run_depend>
<run_depend>sensor_msgs</run_depend>
<run_depend>actionlib_msgs</run_depend>
<run_depend>std_msgs</run_depend>
<run_depend>roseus</run_depend>
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
  <depend package="sensor_msgs"/>
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

find_package(catkin REQUIRED COMPONENTS message_generation roscpp sensor_msgs actionlib_msgs
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
  DEPENDENCIES sensor_msgs std_msgs actionlib_msgs
$(for pkg in $2
do
  echo $pkg
done)
)
catkin_package(
    CATKIN_DEPENDS message_runtime roscpp sensor_msgs std_msgs actionlib_msgs
$(for pkg in $2
do
  echo $pkg
done)
)


include_directories(\${catkin_INCLUDE_DIRS})
add_executable(\${PROJECT_NAME} \${PROJECT_NAME}.cpp)
target_link_libraries(\${PROJECT_NAME} \${catkin_LIBRARIES})
add_dependencies(\${PROJECT_NAME} \${PROJECT_NAME}_generate_messages_cpp)

EOF
}

function add_cmake_only_action() {
    pkg_path=$1
    shift
    cat <<EOF >$pkg_path/CMakeLists.txt
    cmake_minimum_required(VERSION 2.8.3)
project($(basename $pkg_path))

find_package(catkin REQUIRED COMPONENTS message_generation roscpp sensor_msgs actionlib_msgs
$(for pkg in $1
do
  echo $pkg
done)
)

add_action_files(
  FILES Foo.action
)
generate_messages(
  DEPENDENCIES sensor_msgs std_msgs actionlib_msgs
$(for pkg in $2
do
  echo $pkg
done)
)
catkin_package(
    CATKIN_DEPENDS message_runtime roscpp sensor_msgs std_msgs actionlib_msgs
$(for pkg in $2
do
  echo $pkg
done)
)


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
    msg_pkg=${3:-$pkg_name}
    cat <<EOF > $pkg_path/$pkg_name.l
(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(ros::roseus "roseus_test_genmsg")

(deftest test-msg-instance
  (assert (ros::load-ros-manifest "$pkg_name")
          "load-ros-manifest")

  (assert (eval (read-from-string "(instance sensor_msgs::imu :init)"))
          "instantiating msg message")

  (assert (eval (read-from-string "(instance $msg_pkg::String :init)"))
          "instantiating msg message")

  (assert (eval (read-from-string "(instance roseus::String :init)"))
          "instantiating msg message")
  (assert (eval (read-from-string "(instance roseus::StringStamped :init)"))
          "instantiating msg message")
  (assert (ros::load-ros-manifest "roseus")
          "load-ros-manifest")

  )

(run-all-tests)

(exit)
EOF
}

function add_lisp_only_action() {
    pkg_path=$1
    pkg_name=$2
    msg_pkg=${3:-$pkg_name}
    cat <<EOF > $pkg_path/$pkg_name.l
(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(ros::roseus "roseus_test_genmsg")

(deftest test-msg-instance
  (assert (ros::load-ros-manifest "$pkg_name")
          "load-ros-manifest")

  (assert (eval (read-from-string "(instance sensor_msgs::imu :init)"))
          "instantiating msg message")

  (assert (eval (read-from-string "(instance $msg_pkg::FooGoal :init)"))
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

# makeup packages [pkg_path package_name build_depends]
add_${MANIFEST} ${GENEUS_DEP1} geneus_dep1 geneus
add_${MANIFEST} ${GENEUS_DEP2} geneus_dep2 geneus geneus_dep1
add_${MANIFEST} ${ROSEUS_DEP1} roseus_dep1 roseus geneus_dep2 geneus_dep1
add_${MANIFEST} ${ROSEUS_DEP2} roseus_dep2 roseus_dep1 roseus geneus_dep1 geneus_dep2
add_${MANIFEST} ${ROSEUS_DEP3} roseus_dep3 roseus_dep2 roseus_dep1 roseus geneus_dep1 geneus_dep2
add_${MANIFEST} ${ROSEUS_DEP4} roseus_dep4 roseus geneus_dep2 geneus_dep1

# makeup cmake files [pkg_path find_package message_depends]
add_cmake ${GENEUS_DEP1} 
add_cmake ${GENEUS_DEP2} "geneus_dep1" "geneus_dep1"
add_cmake ${ROSEUS_DEP1} "geneus_dep1 roseus geneus_dep2" "geneus_dep1 roseus geneus_dep2"
add_cmake ${ROSEUS_DEP2} "geneus_dep1 roseus geneus_dep2 roseus_dep1" "geneus_dep1 roseus geneus_dep2 roseus_dep1"
add_cmake ${ROSEUS_DEP3} "geneus_dep2 geneus_dep1 roseus roseus_dep2 roseus_dep1" "geneus_dep1 roseus geneus_dep2 roseus_dep1"
add_cmake_only_action ${ROSEUS_DEP4} "geneus_dep1 roseus geneus_dep2" "geneus_dep1 roseus geneus_dep2"

add_cpp ${GENEUS_DEP1} geneus_dep1
add_cpp ${GENEUS_DEP2} geneus_dep2
add_cpp ${ROSEUS_DEP1} roseus_dep1
add_cpp ${ROSEUS_DEP2} roseus_dep2
add_cpp ${ROSEUS_DEP3} roseus_dep3

add_lisp ${GENEUS_DEP1} geneus_dep1
add_lisp ${GENEUS_DEP2} geneus_dep2
add_lisp ${ROSEUS_DEP1} roseus_dep1
add_lisp ${ROSEUS_DEP2} roseus_dep2
add_lisp ${ROSEUS_DEP3} roseus_dep3
add_lisp ${ROSEUS_DEP3} roseus_dep3 roseus_dep2
add_lisp_only_action ${ROSEUS_DEP4} roseus_dep4

add_msg ${GENEUS_DEP1} std_msgs
add_msg ${GENEUS_DEP2} geneus_dep1
add_msg ${ROSEUS_DEP1} geneus_dep2
add_msg ${ROSEUS_DEP2} roseus_dep1
add_msg ${ROSEUS_DEP3} roseus_dep1

add_action ${GENEUS_DEP1} std_msgs
add_action ${GENEUS_DEP2} geneus_dep1
add_action ${ROSEUS_DEP1} geneus_dep2
add_action ${ROSEUS_DEP2} roseus_dep1
add_action ${ROSEUS_DEP3} roseus_dep1
add_action ${ROSEUS_DEP4} geneus_dep2

add_srv ${GENEUS_DEP1} std_msgs
add_srv ${GENEUS_DEP2} geneus_dep1
add_srv ${ROSEUS_DEP1} geneus_dep2
add_srv ${ROSEUS_DEP2} roseus_dep1
add_srv ${ROSEUS_DEP3} roseus_dep1

if [ $REMOVE_MSG = TRUE ]; then
    ROSEUS_MSG_DIR=`cut -f 1 -d: <<< $CMAKE_PREFIX_PATH`/share/roseus
    trap 'mv ${ROSEUS_MSG_DIR}/ros.bak ${ROSEUS_MSG_DIR}/ros; exit ' 1 2 3 15 EXIT
    mv ${ROSEUS_MSG_DIR}/ros ${ROSEUS_MSG_DIR}/ros.bak
    # need to copy since roseus_SOURCE_PREFIX exists
    mkdir -p ${CATKIN_DIR}/devel/share/roseus/ros/
    cp -r ${ROSEUS_MSG_DIR}/ros.bak/roseus ${CATKIN_DIR}/devel/share/roseus/ros/
fi

if [ $WORKSPACE_TYPE = ONE -a ! -e ${CATKIN_DIR}/src/jsk_roseus ]; then
    # if rospack find is source, then copy
    for pkg in geneus roseus; do
        if [ -e `rospack find $pkg`/CMakeLists.txt ]; then
            cp -Lr `rospack find $pkg` ${CATKIN_DIR}/src/$pkg
        fi
    done
fi

if [ $WORKSPACE_TYPE = ONE ]; then
    # need to keep euslisp/jskeus envirnments
    OLD_ROS_PACKAGE_PATH=`rospack find euslisp`:`rospack find jskeus`
    OLD_PKG_CONFIG_PATH=$PKG_CONFIG_PATH
    OLD_CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH
    # reset environmental variables by sousing
    source /opt/ros/${ROS_DISTRO}/setup.bash
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$OLD_ROS_PACKAGE_PATH
    export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$OLD_PKG_CONFIG_PATH
    export CMAKE_PREFIX_PATH=$OLD_CMAKE_PREFIX_PATH:$CMAKE_PREFIX_PATH
    echo "---------------"
    echo "ROS_PACKAGE_PATH  $ROS_PACKAGE_PATH"
    echo "PKG_CONFIG_PATH   $PKG_CONFIG_PATH"
    echo "CMAKE_PREFIX_PATH $CMAKE_PREFIX_PATH"
fi

cd ${CATKIN_DIR}
# always call twice catkin_make
(catkin config --no-jobserver || echo "disable internal jobserver")
if [ $PACKAGE = ALL ]; then
    MAKEFLAGS= catkin build -v -i --no-status -p1 --make-args -j1 -l1 --
    # catkin build -v -i --no-status --force-cmake ### this yeilds `make[5]: *** read jobs pipe: No such file or directory.  Stop.` when catkin run_tests
else
    MAKEFLAGS= catkin build -v -i --no-status -p1 --start-with $PACKAGE $PACKAGE --make-args -j1 -l1 --
fi
source ${CATKIN_DIR}/devel/setup.bash

# # try to run roseus sample program
EUSLISP_DIR=`rospack find euslisp`
EUSLISP_EXE=`find $ROSEUS_DIR -type f -name irteusgl`
if [ ! "$EUSLISP_EXE" ]; then
    EUSLISP_EXE="rosrun euslisp irteusgl"
fi

ROSEUS_DIR=`rospack find roseus`
ROSEUS_EXE=`find $ROSEUS_DIR -type f -name roseus`
if [ ! "$ROSEUS_EXE" ]; then
    ROSEUS_EXE="rosrun roseus roseus"
fi

if [ $PACKAGE = ALL ]; then
    ${ROSEUS_EXE} ${CATKIN_DIR}/src/geneus_dep1/geneus_dep1.l $ARGV
    ${ROSEUS_EXE} ${CATKIN_DIR}/src/geneus_dep2/geneus_dep2.l $ARGV
    ${ROSEUS_EXE} ${CATKIN_DIR}/src/roseus_dep1/roseus_dep1.l $ARGV
    ${ROSEUS_EXE} ${CATKIN_DIR}/src/roseus_dep2/roseus_dep2.l $ARGV
    ${ROSEUS_EXE} ${CATKIN_DIR}/src/roseus_dep3/roseus_dep3.l $ARGV
    ${ROSEUS_EXE} ${CATKIN_DIR}/src/roseus_dep4/roseus_dep4.l $ARGV
    rm -fr ${CAATKIN_DIR}/devel/share/roseus/ros
    rosrun roseus generate-all-msg-srv.sh
    ${ROSEUS_EXE} ${CATKIN_DIR}/src/geneus_dep1/geneus_dep1.l $ARGV
    ${ROSEUS_EXE} ${CATKIN_DIR}/src/geneus_dep2/geneus_dep2.l $ARGV
    ${ROSEUS_EXE} ${CATKIN_DIR}/src/roseus_dep1/roseus_dep1.l $ARGV
    ${ROSEUS_EXE} ${CATKIN_DIR}/src/roseus_dep2/roseus_dep2.l $ARGV
    ${ROSEUS_EXE} ${CATKIN_DIR}/src/roseus_dep3/roseus_dep3.l $ARGV
    ${ROSEUS_EXE} ${CATKIN_DIR}/src/roseus_dep4/roseus_dep4.l $ARGV
else
    ${EUSLISP_EXE} ${ROSEUS_DIR}/euslisp/roseus.l ${CATKIN_DIR}/src/$PACKAGE/$PACKAGE.l $ARGV
    rm -fr ${CAATKIN_DIR}/devel/share/roseus/ros
    rosrun roseus generate-all-msg-srv.sh
    ${EUSLISP_EXE} ${ROSEUS_DIR}/euslisp/roseus.l ${CATKIN_DIR}/src/$PACKAGE/$PACKAGE.l $ARGV
fi

if [ "$OUTPUT" ]; then
    echo "writing test results to ... $OUTPUT"
    cat <<EOF > $OUTPUT
<?xml version="1.0" encoding="utf-8"?>
<testsuite errors="0" failures="0" name="unittest.suite.TestSuite" tests="1" time="1.0">
</testsuite>
EOF
fi

rm -rf ${CATKIN_DIR}

exit 0

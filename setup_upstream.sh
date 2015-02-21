#!/bin/bash

#!/bin/bash

function usage {
    echo >&2 "usage: $0"
    echo >&2 "          [-w|--workspace] workspace to set euslisp repository,"
    exit 0
}

function error {
    usage
    exit 1
}
trap error ERR

# command line parse
OPT=`getopt -o hw: -l help,workspace: -- $*`
if [ $? != 0 ]; then
    usage
fi

eval set -- $OPT
while [ -n "$1" ] ; do
    case $1 in
        -h|--help) usage ;;
        -w|--workspace) WORKSPACE=$2; shift 2;;
        --) shift; break;;
        *) echo "Unknown option($1)"; usage;;
    esac
done

if [ ! -e $WORKSPACE/src/.rosinstall ]; then
    echo "Your workspace is not initialized yet, please run
mkdir -p $WORKSPACE/src; wstool init $WORKSPACE/src
"
    exit 1
fi

cat <<EOF >> /tmp/rosinstall.$$
- git:
    uri: http://github.com/jsk-ros-pkg/geneus
    local-name: geneus
- git:
    uri: http://github.com/euslisp/EusLisp
    local-name: euslisp
- git:
    uri: http://github.com/euslisp/jskeus
    local-name: jskeus
EOF

wstool merge /tmp/rosinstall.$$ -t $WORKSPACE/src
wstool info -t $WORKSPACE/src
wstool update --continue-on-error --abort-changed-uris -t $WORKSPACE/src

if [ "$ROS_DISTRO" == "" ] ; then
    export ROS_DISTRO=indigo
fi
(
cd $WORKSPACE/src;
mkdir -p euslisp/cmake euslisp/env-hooks
if [ ! -e euslisp/package.xml ]; then
    wget https://raw.githubusercontent.com/tork-a/euslisp-release/release/$ROS_DISTRO/euslisp/package.xml -O euslisp/package.xml
fi
for file in CMakeLists.txt cmake/euslisp-extras.cmake.in env-hooks/99.euslisp.sh.in; do
    if [ ! -e euslisp/$file ]; then
        wget https://raw.githubusercontent.com/tork-a/euslisp-release/master/patches/${file} -O euslisp/${file}
    fi
done
if [ ! -e jskeus/CMakeLists.txt ]; then
    wget https://raw.githubusercontent.com/tork-a/jskeus-release/release/$ROS_DISTRO/jskeus/CMakeLists.txt -O jskeus/CMakeLists.txt
fi
)


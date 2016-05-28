#!/bin/bash

#!/bin/bash

function usage {
    echo >&2 "usage: $0"
    echo >&2 "          [-w|--workspace] workspace to set euslisp repository,"
    echo >&2 "          [-p|--package] download only selected package,"
    exit 0
}

function error {
    usage
    exit 1
}
trap error ERR

# command line parse
OPT=`getopt -o hw:p: -l help,workspace:,package: -- $*`
if [ $? != 0 ]; then
    usage
fi

eval set -- $OPT
PACKAGES=""
while [ -n "$1" ] ; do
    case $1 in
        -h|--help) usage ;;
        -w|--workspace) WORKSPACE=$2; shift 2;;
        -p|--package) PACKAGES="$PACKAGES $2"; shift 2;;
        --) shift; break;;
        *) echo "Unknown option($1)"; usage;;
    esac
done

if [ "$PACKAGES" = "" ]; then
  PACKAGES="jsk-ros-pkg/geneus euslisp/Euslisp euslisp/jskeus"
fi

if [ ! -e $WORKSPACE/src/.rosinstall ]; then
    echo "Your workspace is not initialized yet, please run
mkdir -p $WORKSPACE/src; wstool init $WORKSPACE/src
"
    exit 1
fi

for repo_slug in $PACKAGES; do
  cat <<EOF >> /tmp/rosinstall.$$
  - git:
      uri: https://github.com/${repo_slug}.git
      local-name: ${repo_slug}
      version: master
EOF
done

wstool merge /tmp/rosinstall.$$ -t $WORKSPACE/src
wstool info -t $WORKSPACE/src
wstool update --abort-changed-uris -t $WORKSPACE/src $PACKAGES -j3 2> /dev/null

if [ "$ROS_DISTRO" == "" ] ; then
    export ROS_DISTRO=indigo
fi
(
cd $WORKSPACE/src/euslisp;
if [ -e Euslisp ]; then
    mkdir -p Euslisp/cmake Euslisp/env-hooks
    echo "Adding package.xml to Euslisp"
    wget https://raw.githubusercontent.com/tork-a/euslisp-release/release/$ROS_DISTRO/euslisp/package.xml -O Euslisp/package.xml
    for file in CMakeLists.txt cmake/euslisp-extras.cmake.in env-hooks/99.euslisp.sh.in; do
        echo "Adding $file to Euslisp"
        wget https://raw.githubusercontent.com/tork-a/euslisp-release/master/patches/${file} -O Euslisp/${file}
    done
fi
if [ -e jskeus ]; then
    echo "Adding package.xml to jskeus"
    wget https://raw.githubusercontent.com/tork-a/jskeus-release/release/$ROS_DISTRO/jskeus/package.xml -O jskeus/package.xml
    echo "Adding CMakeLists.txt to jskeus"
    wget https://raw.githubusercontent.com/tork-a/jskeus-release/master/patches/CMakeLists.txt -O jskeus/CMakeLists.txt
fi
)

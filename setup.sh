#!/bin/bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
(cd $DIR;

## manually edit changelog
## manyall add new tag, ie : git tag EusLisp-9.11
## bloom-release euslisp --track <DISTRO> --rosdistro <DISTRO>
if [ ! -e euslisp ]; then
    git clone http://github.com/euslisp/EusLisp euslisp
fi
mkdir -p euslisp/cmake euslisp/env-hooks
for file in CMakeLists.txt package.xml cmake/euslisp-extras.cmake.in env-hooks/99.euslisp.sh.in; do
    if [ ! -e euslisp/$file ]; then
        wget https://raw.githubusercontent.com/tork-a/euslisp-release/master/patches/${file} -O euslisp/${file}
    fi
done

## catkin_generate_package
## catkin_prepare_release
## bloom-release jskeus --track <DISTRO> --rosdistro <DISTRO>
if [ ! -e jskeus ]; then
    git clone http://github.com/euslisp/jskeus jskeus
fi
if [ ! -e jskeus/CMakeLists.txt ]; then
    wget https://raw.githubusercontent.com/tork-a/jskeus-release/master/patches/CMakeLists.txt -O jskeus/CMakeLists.txt
fi
if [ ! -e geneus ]; then
    git clone http://github.com/jsk-ros-pkg/geneus geneus
fi
)


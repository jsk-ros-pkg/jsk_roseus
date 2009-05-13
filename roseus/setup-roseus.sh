#!/bin/bash

# patch for rosbuild
(cd ${HOME}/ros/ros/core/rosbuild;patch < ${OLDPWD}/rosbuild.cmake.for-roseus.diff)

# generate link
ln -s ${HOME}/prog/roseus ${HOME}/ros/ros/core/experimental/roseus

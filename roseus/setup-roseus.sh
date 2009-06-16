#!/bin/bash

# patch for rosbuild
(cd ${HOME}/ros/ros/core/rosbuild;patch < ${OLDPWD}/rosbuild.cmake.for-roseus.diff)

# generate link
ln -s ${HOME}/prog/roseus ${HOME}/ros/ros/core/experimental/roseus

ln -s  `rospack find roseus`/scripts/genmsg_eus `rospack find genmsg_cpp`/genmsg_eus  
ln -s  `rospack find roseus`/scripts/gensrv_eus `rospack find genmsg_cpp`/gensrv_eus



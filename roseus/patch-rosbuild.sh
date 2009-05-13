#!/bin/bash

(cd ${HOME}/ros/ros/core/rosbuild;patch < ${OLDPWD}/rosbuild.cmake.for-roseus.diff)
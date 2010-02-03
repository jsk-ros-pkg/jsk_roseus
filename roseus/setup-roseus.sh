#!/bin/bash

(cd ${HOME}/prog/roseus;mkdir cmake;cat ${HOME}/ros/pkgs/ros_experimental/rosoct/cmake/rosoct.cmake | sed -e 's/oct/eus/g' -e 's/".m"/".l"/g' > cmake/roseus.cmake)

# generate link
ln -s ${HOME}/prog/roseus ${HOME}/ros/pkgs/ros_experimental/roseus

ln -s  `rospack find roseus`/scripts/genmsg_eus `rospack find genmsg_cpp`/genmsg_eus  
ln -s  `rospack find roseus`/scripts/gensrv_eus `rospack find genmsg_cpp`/gensrv_eus



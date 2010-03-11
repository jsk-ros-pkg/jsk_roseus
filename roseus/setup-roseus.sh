#!/bin/bash

export ROSOCT_DIR=`rospack find rosoct`

(cd ${HOME}/prog/roseus;mkdir cmake;cat ${ROSOCT_DIR}/cmake/rosoct.cmake | sed -e 's/oct/eus/g' -e 's/".m"/".l"/g' > cmake/roseus.cmake)

# generate link
ln -s ${HOME}/prog/roseus ${ROSOCT_DIR}/../roseus

ln -s  `rospack find roseus`/scripts/genmsg_eus `rospack find genmsg_cpp`/genmsg_eus  
ln -s  `rospack find roseus`/scripts/gensrv_eus `rospack find genmsg_cpp`/gensrv_eus



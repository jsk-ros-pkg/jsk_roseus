#!/bin/bash

export ROSOCT_DIR=`rospack find rosoct`

# generate link
ln -s ${HOME}/prog/roseus ${ROSOCT_DIR}/../roseus

ln -s  `rospack find roseus`/scripts/genmsg_eus `rospack find genmsg_cpp`/genmsg_eus
ln -s  `rospack find roseus`/scripts/gensrv_eus `rospack find genmsg_cpp`/gensrv_eus



#!/bin/bash
ROSEUSDIR="$(dirname $0)"
cd ${ROSEUSDIR}
ROSEUSDIR=`pwd`
export ROSOCT_DIR=`rospack find rosoct`
echo "ROSEUS=$ROSEUSDIR"
# generate link
ln -s ${ROSEUSDIR} ${ROSOCT_DIR}/../roseus

ln -s  `rospack find roseus`/scripts/genmsg_eus `rospack find genmsg_cpp`/genmsg_eus
ln -s  `rospack find roseus`/scripts/gensrv_eus `rospack find genmsg_cpp`/gensrv_eus



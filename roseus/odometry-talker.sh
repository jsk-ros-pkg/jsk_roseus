#!/bin/bash

ROSCORE_BOOTP=`ps aux | grep roscore | grep python`
if [ "${ROSCORE_BOOTP}" == "" ]; then
echo roscore does not exist. you should \'roscore\';
fi

/usr/bin/xterm -display :0.0 -geometry 80x6+150+315 -T odometry -e /bin/bash -x -c "source /home/leus/.bashrc.jsk ; source /home/leus/.bashrc.ros; cd /home/leus/prog/roseus/ ; /usr/local/eus/euslisp/Linux/bin/eus2 odometry-talker.l || sleep 30"

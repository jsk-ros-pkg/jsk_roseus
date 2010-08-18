#!/bin/sh


for file in `rospack list-names`; do
    echo $file
    cd `rospack find $file`; rm -fr msg/eus srv/eus; if [ -e build ]; then cd build; make ROSBUILD_genmsg_eus; fi
done


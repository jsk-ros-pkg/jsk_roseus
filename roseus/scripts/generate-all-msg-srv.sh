#!/bin/bash

function generate-msg-srv {
    local file=$1;
    cd `rospack find $file`; rm -fr msg/eus srv/eus; if [ -e build ]; then cd build; make ROSBUILD_genmsg_eus; fi
}


for file in `rospack list-names`; do
    echo $file
    generate-msg-srv $file
done

for dir in `rosstack list-names`; do
    echo $dir
    for file in `rosstack contents $dir`; do
	echo $file
	generate-msg-srv $file;
    done
done

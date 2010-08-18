#!/bin/bash

function generate-msg-srv {
    local dir=$1;
    echo $dir
    cd $dir; rm -fr msg/eus srv/eus; if [ -e build ]; then cd build; make ROSBUILD_genmsg_eus; fi
}


for pkg in `rospack list-names`; do
    echo "package:$pkg"
    generate-msg-srv `rospack find $pkg`
done

for stack in `rosstack list-names`; do
    echo "stack:$stack"
    for dir in `rosstack contents $stack`; do
	echo "  $dir"
	generate-msg-srv `rosstack find $stack`/$dir
    done
done
